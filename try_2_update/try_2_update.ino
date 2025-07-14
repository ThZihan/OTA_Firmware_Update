/*****************************************************************
 *  Aquarium_Server  –  Sensors + OTA over BLE (ESP32)
 *
 *  Services
 *  ─────────────
 *   1) Sensor Service          UUID 3b48b454‑03d4‑4d52‑bbad‑5997e4c65410
 *        • Temperature  notify  9f7e4255‑cacd‑4cb7‑8b0e‑73a124eb922e
 *        • TDS          notify  87aa5230‑bd9d‑494d‑8669‑7fb51b823662
 *        • Company      read    2ec1498e‑41f2‑4827‑b2ce‑3e043fa77de8
 *        • Firmware     read    94e31f48‑a2dc‑47c3‑aca9‑b0a042e6f156
 *
 *   2) OTA Service             UUID 66443771‑d481‑49b0‑be32‑8ce24ac0f09c
 *        • OTA Data     rw/n   66443772‑d481‑49b0‑be32‑8ce24ac0f09c
 *
 *  OTA protocol:  OPEN → <u32 sizeLE> → <chunks> → DONE  (HALT to cancel)
 *****************************************************************/

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Update.h>

#include <OneWire.h>
#include <DallasTemperature.h>

/* ---------- USER CONFIG ------------------------------------- */
#define TDS_PIN        4        // Analog pin for TDS probe (GPIO 4)
#define TEMP_PIN       2        // DS18B20 data pin             (GPIO 2)
#define ADC_VREF       3.3f     // ESP32 ADC reference voltage
#define SCOUNT         30       // Median‑filter sample count
/* ------------------------------------------------------------ */

/* ---------- OTA UUIDs --------------------------------------- */
const char* OTA_SERVICE_UUID = "66443771-D481-49B0-BE32-8CE24AC0F09C";
const char* OTA_CHAR_UUID    = "66443772-D481-49B0-BE32-8CE24AC0F09C";

/* ---------- SENSOR UUIDs ------------------------------------ */
const char* SENSOR_SERVICE_UUID = "3b48b454-03d4-4d52-bbad-5997e4c65410";
const char* TEMP_CHAR_UUID      = "9f7e4255-cacd-4cb7-8b0e-73a124eb922e";
const char* TDS_CHAR_UUID       = "87aa5230-bd9d-494d-8669-7fb51b823662";
const char* COMPANY_CHAR_UUID   = "2ec1498e-41f2-4827-b2ce-3e043fa77de8";
const char* FW_CHAR_UUID        = "94e31f48-a2dc-47c3-aca9-b0a042e6f156";

/* ---------- Globals – sensor side --------------------------- */
int   analogBuf[SCOUNT];
int   analogBufTemp[SCOUNT];
int   bufIndex = 0;

float temperatureC = 28.0f;
float tdsValue     = 0.0f;

unsigned long lastNotify  = 0;
const uint32_t notifyDelay = 30000;

OneWire          oneWire(TEMP_PIN);
DallasTemperature sensors(&oneWire);

BLECharacteristic *pTempChar = nullptr;
BLECharacteristic *pTdsChar  = nullptr;

/* ---------- Globals – OTA side ------------------------------ */
bool      deviceConnected   = false;
bool      otaInProgress     = false;
uint32_t  otaFileSize       = 0;
uint32_t  otaReceived       = 0;
int       lastProgressPct   = -1;

/* ------------------------------------------------------------ */
/* Helper – median filter                                       */
int getMedian(int *src, int len) {
  memcpy(analogBufTemp, src, len * sizeof(int));
  for (int j = 0; j < len - 1; ++j)
    for (int i = 0; i < len - j - 1; ++i)
      if (analogBufTemp[i] > analogBufTemp[i + 1])
        std::swap(analogBufTemp[i], analogBufTemp[i + 1]);
  return (len & 1) ? analogBufTemp[len / 2]
                   : (analogBufTemp[len / 2] + analogBufTemp[len / 2 - 1]) / 2;
}

/* ------------------------------------------------------------ */
void sampleTds() {
  analogBuf[bufIndex++] = analogRead(TDS_PIN);
  if (bufIndex == SCOUNT) bufIndex = 0;
}

float readTemperature() {
  sensors.requestTemperatures();
  float t = sensors.getTempCByIndex(0);
  return (t == DEVICE_DISCONNECTED_C) ? temperatureC : t;
}

float computeTds(float tempC) {
  float avgVoltage = getMedian(analogBuf, SCOUNT) * ADC_VREF / 4095.0f;
  float compCoeff   = 1.0f + 0.02f * (tempC - 25.0f);
  float compVolt    = avgVoltage / compCoeff;
  return (133.42f * powf(compVolt, 3) -
          255.86f * powf(compVolt, 2) +
           857.39f * compVolt) * 0.5f;
}

/* ---------- BLE callbacks ----------------------------------- */
class OTACharCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    uint8_t* data = c->getData();
    size_t   len  = c->getLength();
    if (len == 0) return;

    /* OPEN */
    if (!otaInProgress && len == 4 && memcmp(data,"OPEN",4) == 0) {
      otaInProgress = true;
      otaFileSize   = otaReceived = 0;
      lastProgressPct = -1;
      Serial.println("[OTA] Started");
      return;
    }

    /* HALT */
    if (otaInProgress && len == 4 && memcmp(data,"HALT",4) == 0) {
      Serial.println("[OTA] Cancelled");
      Update.end(false);
      otaInProgress = false;
      return;
    }

    if (!otaInProgress) return;            // ignore stray packets

    /* First 4 bytes after OPEN = size */
    if (otaFileSize == 0 && len == 4) {
      memcpy(&otaFileSize, data, 4);
      Serial.printf("[OTA] Size: %u bytes\n", otaFileSize);
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        Serial.println("[OTA] ERROR: Update.begin failed");
        otaInProgress = false;
      }
      return;
    }

    /* DONE */
    if (len == 4 && memcmp(data,"DONE",4) == 0) {
      Serial.println("[OTA] Finalising…");
      bool ok = (otaReceived == otaFileSize) &&
                Update.end(true) && Update.isFinished();
      if (ok) {
        Serial.println("[OTA] Success – rebooting");
        delay(200);
        ESP.restart();
      } else {
        Serial.println("[OTA] ERROR during finalise");
        Update.printError(Serial);
      }
      otaInProgress = false;
      return;
    }

    /* Binary chunk */
    if (otaReceived < otaFileSize) {
      size_t w = Update.write(data, len);
      if (w > 0) {
        otaReceived += w;
        int pct = (otaReceived * 100) / otaFileSize;
        if (pct != lastProgressPct) {
          lastProgressPct = pct;
          Serial.printf("[OTA] %d %%\n", pct);
        }
      }
    }
  }
};

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override { deviceConnected = true; }
  void onDisconnect(BLEServer* s) override {
    deviceConnected = false;
    if (otaInProgress) {                    // abort unfinished OTA
      Update.end(false);
      otaInProgress = false;
      Serial.println("[OTA] Aborted due to disconnect");
    }
    s->getAdvertising()->start();
  }
};

/* ------------------------------------------------------------ */
/* SETUP                                                        */
void setup() {
  Serial.begin(115200);

  /* Sensors */
  // sensors.begin();            // real DS18B20
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  randomSeed(micros());

  /* BLE ------------------------------------------------------ */
  BLEDevice::init("Aquarium_Server");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  /* -------- Sensor Service --------------------------------- */
  BLEService *pSensor = pServer->createService(SENSOR_SERVICE_UUID);

  pTempChar = pSensor->createCharacteristic(
      TEMP_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pTempChar->addDescriptor(new BLE2902());
  BLEDescriptor *tempDesc = new BLEDescriptor(BLEUUID((uint16_t)0x2901));
  tempDesc->setValue("Water temperature (°C)");
  pTempChar->addDescriptor(tempDesc);

  pTdsChar  = pSensor->createCharacteristic(
      TDS_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pTdsChar->addDescriptor(new BLE2902());
  BLEDescriptor *tdsDesc = new BLEDescriptor(BLEUUID((uint16_t)0x2901));
  tdsDesc->setValue("TDS (ppm)");
  pTdsChar->addDescriptor(tdsDesc);

  BLECharacteristic *pComp = pSensor->createCharacteristic(
      COMPANY_CHAR_UUID, BLECharacteristic::PROPERTY_READ);
  pComp->setValue("IoTexperience");

  BLECharacteristic *pFw = pSensor->createCharacteristic(
      FW_CHAR_UUID, BLECharacteristic::PROPERTY_READ);
  pFw->setValue("0.9.9");                    // bump version

  pSensor->start();

  /* -------- OTA Service ------------------------------------ */
  BLEService *pOta = pServer->createService(OTA_SERVICE_UUID);

  BLECharacteristic *pOtaChar = pOta->createCharacteristic(
      OTA_CHAR_UUID,
      BLECharacteristic::PROPERTY_READ   |
      BLECharacteristic::PROPERTY_WRITE  |
      BLECharacteristic::PROPERTY_WRITE_NR |
      BLECharacteristic::PROPERTY_NOTIFY);
  pOtaChar->addDescriptor(new BLE2902());
  pOtaChar->setCallbacks(new OTACharCallbacks());
  pOta->start();

  /* -------- Advertise both services ------------------------ */
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SENSOR_SERVICE_UUID);
  adv->addServiceUUID(OTA_SERVICE_UUID);
  adv->start();

  Serial.println("Aquarium_Server ready – waiting for client…");
}

/* ------------------------------------------------------------ */
/* LOOP                                                         */
void loop() {
  /* ---- sample every 40 ms --------------------------------- */
  static unsigned long tSample = millis();
  if (millis() - tSample >= 40) {
    tSample = millis();
    sampleTds();
  }

  /* ---- compute every 800 ms --------------------------------*/
  static unsigned long tCompute = millis();
  if (millis() - tCompute >= 800) {
    tCompute = millis();
    temperatureC = (random(0,15) / 100.0f);   // demo
    // temperatureC = readTemperature();               // real sensor
    // tdsValue     = computeTds(temperatureC);
    tdsValue     = random(0, 4);                  // demo TDS
  }

  /* ---- notify every 30 s (skip during OTA) ---------------- */
  if (deviceConnected && !otaInProgress &&
      millis() - lastNotify >= notifyDelay) {

    lastNotify = millis();
    char buf[8];

    dtostrf(temperatureC, 6, 2, buf);
    pTempChar->setValue(buf);
    pTempChar->notify();

    dtostrf(tdsValue, 6, 0, buf);
    pTdsChar->setValue(buf);
    pTdsChar->notify();

    Serial.printf("Notify | Temp %.2f °C | TDS %.0f ppm\n",
                  temperatureC, tdsValue);
  }
}
