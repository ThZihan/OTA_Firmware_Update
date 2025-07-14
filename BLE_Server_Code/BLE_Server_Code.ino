 *  - Server name      : Aquarium_Server
 *  - Service UUID     : 3b48b454-03d4-4d52-bbad-5997e4c65410
 *  - Characteristics  :
 *      • Temperature (notify)  : 9f7e4255-cacd-4cb7-8b0e-73a124eb922e
 *      • TDS         (notify)  : 87aa5230-bd9d-494d-8669-7fb51b823662
 *      • Company     (read)    : 2ec1498e-41f2-4827-b2ce-3e043fa77de8
 *      • Firmware    (read)    : 94e31f48-a2dc-47c3-aca9-b0a042e6f156
 *  - Descriptors use standard 0x2902 (CCD) for notify chars,
 *    0x2901 (User Description) for read-only chars.
 ***************************************************************/

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

/* ---------- USER CONFIG ------------------------------------- */
#define TDS_PIN        4        // Analog pin for TDS probe (GPIO 4)
#define TEMP_PIN       2        // DS18B20 data pin             (GPIO 2)
#define ADC_VREF       3.3f     // ESP32 ADC reference voltage
#define SCOUNT         30       // Samples for median filter
/* ------------------------------------------------------------- */

/* ---------- GLOBALS ----------------------------------------- */
int   analogBuf[SCOUNT];
int   analogBufTemp[SCOUNT];
int   bufIndex = 0;

float temperatureC = 25.0f;     // updated each cycle
float tdsValue    = 0.0f;

unsigned long  lastNotify   = 0;          // 30-s notify timer
const uint32_t notifyDelay  = 30000;      // ms

/* OneWire & DallasTemperature objects */
OneWire          oneWire(TEMP_PIN);
DallasTemperature sensors(&oneWire);

/* ------- BLE pointers --------------------------------------- */
BLECharacteristic *pTempChar   = nullptr;
BLECharacteristic *pTdsChar    = nullptr;

/* ------------------------------------------------------------- */
/* MEDIAN FILTER ------------------------------------------------ */
int getMedian(int *src, int len)
{
  int sorted[len];
  memcpy(sorted, src, len * sizeof(int));

  for (int j = 0; j < len - 1; ++j)
    for (int i = 0; i < len - j - 1; ++i)
      if (sorted[i] > sorted[i + 1])
        std::swap(sorted[i], sorted[i + 1]);

  return (len & 1) ? sorted[len / 2]
                   : (sorted[len / 2] + sorted[len / 2 - 1]) / 2;
}

/* ------------------------------------------------------------- */
/* SAMPLE TDS SENSOR ------------------------------------------- */
void sampleTds()
{
  analogBuf[bufIndex++] = analogRead(TDS_PIN);
  if (bufIndex == SCOUNT) bufIndex = 0;
}

/* ------------------------------------------------------------- */
/* READ TEMPERATURE FROM DS18B20 --------------------------------*/
float readTemperature()
{
  sensors.requestTemperatures();
  float t = sensors.getTempCByIndex(0);
  /* If sensor not connected, Dallas returns -127 °C; fall back   */
  return (t == DEVICE_DISCONNECTED_C) ? temperatureC : t;
}

/* ------------------------------------------------------------- */
/* CALCULATE TDS ------------------------------------------------ */
float computeTds(float tempC)
{
  for (int i = 0; i < SCOUNT; ++i) analogBufTemp[i] = analogBuf[i];

  float avgVoltage = getMedian(analogBufTemp, SCOUNT) * ADC_VREF / 4095.0f;

  /* temperature compensation */
  float compCoeff    = 1.0f + 0.02f * (tempC - 25.0f);
  float compVoltage  = avgVoltage / compCoeff;

  /* polynomial conversion (DFRobot standard) */
  return (133.42f * powf(compVoltage, 3) -
          255.86f * powf(compVoltage, 2) +
          857.39f * compVoltage) * 0.5f;
}

/* ------------------------------------------------------------- */
/* BLE SERVER CALLBACKS ---------------------------------------- */
bool deviceConnected = false;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*)   override { deviceConnected = true;  }
  void onDisconnect(BLEServer*) override { deviceConnected = false; }
};

/* ------------------------------------------------------------- */
/* SETUP -------------------------------------------------------- */
void setup()
{
  Serial.begin(115200);

  /* -------- REAL SENSOR INITIALISATION (commented) --------- */
  // sensors.begin();                 // initialise DS18B20 bus
  /* --------------------------------------------------------- */

  analogReadResolution(12);        // use full 12-bit ADC
  analogSetAttenuation(ADC_11db);  // 0-3.3 V range
  randomSeed(micros());            // PRNG for dummy data

  /* ---------------- BLE INIT ------------------------------- */
  BLEDevice::init("Aquarium_Server");

  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService("3b48b454-03d4-4d52-bbad-5997e4c65410");

  /* ---- Characteristics ---- */
  // Temperature (notify)
  pTempChar = pService->createCharacteristic(
      "9f7e4255-cacd-4cb7-8b0e-73a124eb922e",
      BLECharacteristic::PROPERTY_NOTIFY
  );
  pTempChar->addDescriptor(new BLE2902());                 // Client-Config
  BLEDescriptor *tempDesc = new BLEDescriptor(BLEUUID((uint16_t)0x2901));
  tempDesc->setValue("Water temperature (°C)");
  pTempChar->addDescriptor(tempDesc);

  // TDS (notify)
  pTdsChar  = pService->createCharacteristic(
      "87aa5230-bd9d-494d-8669-7fb51b823662",
      BLECharacteristic::PROPERTY_NOTIFY
  );
  pTdsChar->addDescriptor(new BLE2902());
  BLEDescriptor *tdsDesc = new BLEDescriptor(BLEUUID((uint16_t)0x2901));
  tdsDesc->setValue("TDS (ppm)");
  pTdsChar->addDescriptor(tdsDesc);

  // Company name (read-only)
  BLECharacteristic *pCompanyChar = pService->createCharacteristic(
      "2ec1498e-41f2-4827-b2ce-3e043fa77de8",
      BLECharacteristic::PROPERTY_READ
  );
  pCompanyChar->setValue("IoTexperience");
  BLEDescriptor *compDesc = new BLEDescriptor(BLEUUID((uint16_t)0x2901));
  compDesc->setValue("Manufacturer");
  pCompanyChar->addDescriptor(compDesc);

  // Firmware version (read-only)
  BLECharacteristic *pFwChar = pService->createCharacteristic(
      "94e31f48-a2dc-47c3-aca9-b0a042e6f156",
      BLECharacteristic::PROPERTY_READ
  );
  pFwChar->setValue("0.0.1");
  BLEDescriptor *fwDesc = new BLEDescriptor(BLEUUID((uint16_t)0x2901));
  fwDesc->setValue("Firmware version");
  pFwChar->addDescriptor(fwDesc);

  /* ---------------- Start service & advertise ------------ */
  pService->start();
  BLEAdvertising *pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID("3b48b454-03d4-4d52-bbad-5997e4c65410");
  pAdv->start();

  Serial.println("BLE Aquarium_Server ready – waiting for client...");
}

/* ------------------------------------------------------------- */
/* LOOP --------------------------------------------------------- */
void loop()
{
  /* ---- sensor sampling (every 40 ms) ---------------------- */
  static unsigned long tSample = millis();
  if (millis() - tSample >= 40) {
    tSample = millis();
    sampleTds();
  }

  /* ---- 800 ms: update dummy temperature & compute TDS ----- */
  static unsigned long tCompute = millis();
  if (millis() - tCompute >= 800) {
    tCompute = millis();

    // ---- simulate temperature between 27.86 and 28.00 °C ----
    // temperatureC = 27.86f + (random(0, 15) / 100.0f);
    temperatureC = readTemperature();   // <-- real sensor read

    // ---------------------------------------------------------

    tdsValue     = computeTds(temperatureC);
  }

  /* ---- 30-s: notify connected client ---------------------- */
  if (deviceConnected && (millis() - lastNotify >= notifyDelay)) {
    lastNotify = millis();

    char buf[8];

    dtostrf(temperatureC, 6, 2, buf);
    pTempChar->setValue(buf);
    pTempChar->notify();

    dtostrf(tdsValue, 6, 0, buf);     // integer ppm
    pTdsChar->setValue(buf);
    pTdsChar->notify();

    Serial.printf("Notify | Temp: %.2f °C  |  TDS: %.0f ppm\n",
                  temperatureC, tdsValue);
  }
}
