#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#define CONTROL_SERVICE_UUID  "d5060001-a904-deb9-4748-2c7f4a124842"
#define EMG_SERVICE_UUID      "d5060005-a904-deb9-4748-2c7f4a124842"
#define IMU_SERVICE_UUID      "d5060002-a904-deb9-4748-2c7f4a124842"
#define IMU_DATA_UUID         "d5060402-a904-deb9-4748-2c7f4a124842"
#define COMMAND_UUID          "d5060401-a904-deb9-4748-2c7f4a124842"

BLEUUID emgCharUUIDs[4] = {
  BLEUUID("d5060105-a904-deb9-4748-2c7f4a124842"),
  BLEUUID("d5060205-a904-deb9-4748-2c7f4a124842"),
  BLEUUID("d5060305-a904-deb9-4748-2c7f4a124842"),
  BLEUUID("d5060405-a904-deb9-4748-2c7f4a124842")
};

BLEClient*           client    = nullptr;
bool                 myoFound  = false;
bool                 scanning  = false;
BLEAdvertisedDevice* myoDevice = nullptr;

volatile int8_t emgRaw[8]    = {0};
volatile bool   newDataReady = false;
unsigned long   lastDataTime = 0;

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.isAdvertisingService(BLEUUID(CONTROL_SERVICE_UUID))) {
      Serial.println("Myo Found!");
      myoDevice = new BLEAdvertisedDevice(advertisedDevice);
      myoFound  = true;
      scanning  = false;
      BLEDevice::getScan()->stop();
    }
  }
};

void imuCallback(BLERemoteCharacteristic* pChar, uint8_t* data, size_t length, bool isNotify) {
  lastDataTime = millis();
}

void emgCallback(BLERemoteCharacteristic* pChar, uint8_t* data, size_t length, bool isNotify) {
  if (!pChar->getUUID().equals(emgCharUUIDs[0])) return;
  if (length < 16) return;

  for (int i = 0; i < 8; i++)
    emgRaw[i] = (int8_t)data[i];

  // No DAC output — all channels are digital now
  newDataReady = true;
  lastDataTime = millis();
}

void startScan() {
  if (scanning) return;
  Serial.println("Scanning...");
  scanning = true;
  myoFound = false;
  BLEScan* scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  scan->setActiveScan(true);
  scan->start(30);
}

bool connectToMyo() {
  if (client != nullptr) {
    client->disconnect();
    delete client;
    client = nullptr;
  }

  client = BLEDevice::createClient();
  if (!client->connect(myoDevice)) {
    Serial.println("Connection failed!");
    return false;
  }
  Serial.println("Connected!");

  BLERemoteService* controlService = client->getService(CONTROL_SERVICE_UUID);
  if (!controlService) return false;

  BLERemoteCharacteristic* commandChar = controlService->getCharacteristic(COMMAND_UUID);
  if (!commandChar) return false;

  uint8_t cmdEMG[] = {0x01, 0x03, 0x02, 0x01, 0x01};
  commandChar->writeValue(cmdEMG, sizeof(cmdEMG));

  BLERemoteService* imuService = client->getService(IMU_SERVICE_UUID);
  if (imuService) {
    BLERemoteCharacteristic* imuChar = imuService->getCharacteristic(IMU_DATA_UUID);
    if (imuChar) imuChar->registerForNotify(imuCallback);
  }

  BLERemoteService* emgService = client->getService(EMG_SERVICE_UUID);
  if (!emgService) return false;

  for (int i = 0; i < 4; i++) {
    BLERemoteCharacteristic* emgChar = emgService->getCharacteristic(emgCharUUIDs[i]);
    if (emgChar) emgChar->registerForNotify(emgCallback);
  }

  lastDataTime = millis();
  return true;
}

void setup() {
  Serial.begin(921600);
  BLEDevice::init("ESP32_Myo");
  startScan();
}

void loop() {
  if (myoFound && !scanning) {
    if (!connectToMyo()) {
      delay(2000);
      startScan();
    }
    myoFound = false;
  }

  if (client != nullptr && !client->isConnected()) {
    Serial.println("Lost connection! Reconnecting...");
    delay(1000);
    if (myoDevice != nullptr) {
      if (!connectToMyo()) {
        delay(2000);
        startScan();
      }
    } else {
      startScan();
    }
  }

  if (client != nullptr && client->isConnected() &&
      millis() - lastDataTime > 5000 && lastDataTime != 0) {
    Serial.println("No data! Forcing reconnect...");
    client->disconnect();
  }

  if (newDataReady) {
    newDataReady = false;

    // All 8 channels straight from BLE — CSV: CH1, CH2, CH3, CH4, CH5, CH6, CH7, CH8
    Serial.print(emgRaw[0]);    Serial.print(",");
    Serial.print(emgRaw[1]);    Serial.print(",");
    Serial.print(emgRaw[2]);    Serial.print(",");
    Serial.print(emgRaw[3]);    Serial.print(",");
    Serial.print(emgRaw[4]);    Serial.print(",");
    Serial.print(emgRaw[5]);    Serial.print(",");
    Serial.print(emgRaw[6]);    Serial.print(",");
    Serial.println(emgRaw[7]);
  }
}