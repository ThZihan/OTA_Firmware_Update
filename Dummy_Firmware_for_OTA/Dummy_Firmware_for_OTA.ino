/*
  OTA-capable firmware + LED blink demo
  -------------------------------------
  • Keeps the original BLE-based OTA update service/characteristic.
  • Replaces the serial “counter” demo with a simple LED blink.
*/

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Update.h>

#ifdef _BLE_DEVICE_H_
  #error "Conflicting BLE library detected (possibly ArduinoBLE). Please remove it to proceed."
#endif

/**************  BLE/OTA GLOBALS  ****************/
BLEServer        *pServer       = nullptr;
BLECharacteristic*pCharacteristic = nullptr;
bool              deviceConnected  = false;

/* OTA state */
bool      otaInProgress      = false;
uint32_t  otaFileSize        = 0;
uint32_t  otaReceived        = 0;
int       lastProgressPercent = -1;

/* UUIDs (unchanged) */
#define SERVICE_UUID        "66443771-D481-49B0-BE32-8CE24AC0F09C"
#define CHARACTERISTIC_UUID "66443772-D481-49B0-BE32-8CE24AC0F09C"

/**************  LED BLINK GLOBALS  **************/
const uint8_t LED_PIN = 2;   // GPIO 2 on most DevKits
bool          ledState = LOW;
unsigned long previousMillis = 0;
const unsigned long BLINK_INTERVAL_MS = 1000;  // 1 s

/**************  BLE CALLBACKS  ******************/
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    deviceConnected = true;
    Serial.println("[BLE] Device connected.");
  }
  void onDisconnect(BLEServer* pServer) override {
    deviceConnected = false;

    /* Abort any partial OTA if the phone drops out */
    if (otaInProgress) {
      Serial.println("[OTA] Update cancelled.");
      Update.end(false);
      otaInProgress = false;
    }

    /* Restart advertising so the app can reconnect */
    pServer->getAdvertising()->start();
    Serial.println("[BLE] Device disconnected. Re-advertising…");
  }
};

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) override {
    uint8_t* data   = pChar->getData();
    size_t   len    = pChar->getLength();
    if (len == 0) return;

    /* ----- Control messages ------------------------------------------------ */
    if (!otaInProgress && len == 4 && memcmp(data,"OPEN",4)==0) {
      Serial.println("[OTA] Update started.");
      otaInProgress = true;
      otaFileSize   = otaReceived = 0;
      lastProgressPercent = -1;
      return;
    }
    if (otaInProgress && len == 4 && memcmp(data,"HALT",4)==0) {
      Serial.println("[OTA] Update cancelled.");
      Update.end(false);
      otaInProgress = false;
      return;
    }

    /* ----- OTA data stream -------------------------------------------------- */
    if (otaInProgress) {

      /* First 4 bytes after OPEN = file size (little-endian) */
      if (otaFileSize == 0 && len == 4) {
        memcpy(&otaFileSize, data, 4);
        Serial.printf("[OTA] Update size: %u bytes\n", otaFileSize);
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
          Serial.println("[OTA] ERROR: cannot start update.");
          otaInProgress = false;
        }
        return;
      }

      /* DONE marks the end of the transfer */
      if (len == 4 && memcmp(data,"DONE",4)==0) {
        Serial.println("[OTA] Finalising…");
        if (otaReceived != otaFileSize) {
          Serial.printf("[OTA] ERROR: size mismatch (%u/%u)\n",
                        otaReceived, otaFileSize);
          Update.end(false);
        } else if (Update.end(true) && Update.isFinished()) {
          Serial.println("[OTA] Update successful – rebooting!");
          ESP.restart();
        } else {
          Serial.println("[OTA] ERROR: failed to finalise.");
          Update.printError(Serial);
        }
        otaInProgress = false;
        return;
      }

      /* Write binary chunk */
      if (otaReceived < otaFileSize) {
        size_t written = Update.write(data, len);
        if (written > 0) {
          otaReceived += written;
          int progress = (otaReceived * 100) / otaFileSize;
          if (progress != lastProgressPercent) {
            lastProgressPercent = progress;
            Serial.printf("[OTA] %d %%\n", progress);
          }
        }
      }
      return;
    }

    /* ---------- Regular console data (unused in this demo) ----------------- */
    Serial.print("[Console] ");
    Serial.write(data, len);
    Serial.println();
  }
};

/**************************  SETUP  **************************/
void setup() {
  Serial.begin(115200);

  /* LED */
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, ledState);

  /* BLE init */
  BLEDevice::init("Wible");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ   |
      BLECharacteristic::PROPERTY_WRITE  |
      BLECharacteristic::PROPERTY_WRITE_NR |
      BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

  pService->start();
  BLEDevice::startAdvertising();
  Serial.println("[BLE] Device initialised. Advertising…");
}

/***************************  LOOP  ***************************/
void loop() {

  /* Blink only when we are NOT in the middle of an OTA transfer */
  if (!otaInProgress) {
    unsigned long now = millis();
    if (now - previousMillis >= BLINK_INTERVAL_MS) {
      previousMillis = now;
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
    }
  }

  /* Nothing else to do; OTA is interrupt-driven via callbacks */
}
