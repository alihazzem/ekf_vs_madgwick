#include "BLEDevice.h"
#include "FastAccelStepper.h"
#define step 4
#define dir 15
#define step2 17
#define dir2 16
#define step3 18
#define dir3 5
#define step4 23
#define dir4 19
#define step5 13
#define dir5 12
#define step6 14
#define dir6 27
#define step7 26
#define dir7 25
#define step8 33
#define dir8 32

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *motor1 = NULL;
FastAccelStepper *motor2 = NULL;
FastAccelStepper *motor3 = NULL;
FastAccelStepper *motor4 = NULL;
FastAccelStepper *motor5 = NULL;
FastAccelStepper *motor6 = NULL;
FastAccelStepper *motor7 = NULL;
FastAccelStepper *motor8 = NULL;

// --- BLE UUIDs (Must match the TX ESP32 exactly) ---
static BLEUUID serviceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
static BLEUUID charUUID("beb5483e-36e1-4688-b7f5-ea07361b26a8");

// --- Telemetry Packet ---
#pragma pack(push, 1)
typedef struct {
    uint8_t header1;           // 0xAA
    uint8_t header2;           // 0x55
    int16_t thigh_angle_deg100;
    int16_t shin_angle_deg100;
    int16_t knee_angle_deg100;
    uint8_t checksum;
} EspTelemetry_t;
#pragma pack(pop)

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAdvertisedDevice* myDevice;

// Callback triggered whenever the TX ESP32 sends a new packet over the air
static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) 
{
    // Make sure we received exactly the right number of bytes
    if (length == sizeof(EspTelemetry_t)) {
        EspTelemetry_t* pkt = (EspTelemetry_t*)pData;

        // Verify the headers just in case
        if (pkt->header1 == 0xAA && pkt->header2 == 0x55) {
            
            // Reconstruct the floating point angles
            float trueThigh = pkt->thigh_angle_deg100 / 100.0f;
            float trueShin  = pkt->shin_angle_deg100 / 100.0f;
            float trueKnee  = pkt->knee_angle_deg100 / 100.0f;

            // Print the live data to the Serial Monitor!
                motor5->moveTo(0.7434*(trueThigh)*800);
                motor1->moveTo(0.8608*trueKnee*800);
                motor3->moveTo(0.6519*(trueKnee-trueThigh)*800);
                //motor8->moveTo(0.7434*(trueShin+trueKnee)*800);
                //motor2->moveTo(0.8608*trueKnee*800);
                //motor6->moveTo(-0.6519*(trueShin)*800);
                Serial.println(motor1->getCurrentPosition());
        }
    }

}

// Client callbacks to handle connection/disconnection
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    connected = true;
    Serial.println("Connected to TX ESP32!");
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("Disconnected from TX ESP32! Scanning to reconnect...");
    doScan = true; // Automatically start searching for the TX ESP32 again
  }
};

// Connects to the specific BLE server (the TX ESP32)
bool connectToServer() {
    Serial.print("Forming a connection to ");
    Serial.println(myDevice->getAddress().toString().c_str());
    
    BLEClient*  pClient  = BLEDevice::createClient();
    pClient->setClientCallbacks(new MyClientCallback());

    // Connect to the remote BLE Server
    pClient->connect(myDevice);  
    Serial.println(" - Connected to server");

    // Obtain a reference to the service we are after
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }

    // Obtain a reference to the characteristic
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(charUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }

    // Subscribe to notifications
    if(pRemoteCharacteristic->canNotify()) {
      pRemoteCharacteristic->registerForNotify(notifyCallback);
      Serial.println(" - Successfully registered for notifications!");
    }

    return true;
}

// Scans for BLE devices and finds our TX ESP32
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    // If the device advertises the exact Service UUID we are looking for
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
      
      Serial.print("Found TX ESP32! Address: ");
      Serial.println(advertisedDevice.getAddress().toString().c_str());

      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = false;
    }
  }
};

void setup() {
  Serial.begin(115200);
  Serial.println("Starting ESP32 Leg Receiver (Client)...");

  BLEDevice::init("");

  // Start scanning for the TX ESP32
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false); // Scan for 5 seconds
  
  Serial.println("Scanning for BLE TX Device...");

  engine.init();
  motor1 = engine.stepperConnectToPin(step);
  motor2 = engine.stepperConnectToPin(step2);
  motor3 = engine.stepperConnectToPin(step3);
  motor4 = engine.stepperConnectToPin(step4);
  motor5 = engine.stepperConnectToPin(step5);
  motor6 = engine.stepperConnectToPin(step6);
  motor7 = engine.stepperConnectToPin(step7);
  motor8 = engine.stepperConnectToPin(step8);
  if (motor1) {
    motor1->setDirectionPin(dir);
  }
  if (motor2) {
    motor2->setDirectionPin(dir2);
  }
  if (motor3) {
    motor3->setDirectionPin(dir3);
  }
  if (motor4) {
    motor4->setDirectionPin(dir4);
  }
  if (motor5) {
    motor5->setDirectionPin(dir5);
  }
  if (motor6) {
    motor6->setDirectionPin(dir6);
  }
  if (motor7) {
    motor7->setDirectionPin(dir7);
  }
  if (motor8) {
    motor8->setDirectionPin(dir8);
  }

  motor1->setSpeedInHz(200000);
  motor3->setSpeedInHz(200000);
  motor5->setSpeedInHz(200000);
  motor2->setSpeedInHz(200000);
  motor6->setSpeedInHz(200000);
  motor8->setSpeedInHz(200000);
  motor1->setAcceleration(200000);
  motor3->setAcceleration(200000);
  motor5->setAcceleration(200000);
  motor2->setAcceleration(200000);
  motor6->setAcceleration(200000);
  motor8->setAcceleration(200000);
}

void loop() {
  // If the scanner found our TX ESP32, connect to it!
  if (doConnect == true) {
    if (connectToServer()) {
      Serial.println("We are now connected to the BLE Server.");
    } else {
      Serial.println("We have failed to connect to the server.");
    }
    doConnect = false;
  }

  // If we disconnected, restart the scanner
  if (doScan) {
    BLEDevice::getScan()->start(5, false); 
    doScan = false;
  }
 

  
}
