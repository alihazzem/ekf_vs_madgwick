#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// ─── PIN CONFIG ───────────────────────────────────────────────────────────────
#define STM32_TX_PIN      17
#define STM32_RX_PIN      16
#define STM32_BAUD        921600

#define LED_R             25
#define LED_G             26
#define LED_B             27

#define CALIB_BTN         34        // active LOW, internal pullup

// ─── EMG PROCESSING CONFIG ────────────────────────────────────────────────────
#define N_CH              8
#define RMS_WINDOW        15        // samples (~75ms at 200Hz)
#define UPDATE_EVERY      5         // send to STM32 every N samples (~25ms)

// ─── CALIBRATION CONFIG ───────────────────────────────────────────────────────
#define CALIB_DURATION_MS 4000      // 2 seconds per phase

// ─── BLE UUIDs ────────────────────────────────────────────────────────────────
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

// ─── IIR FILTER COEFFICIENTS ─────────────────────────────────────────────────
// Butterworth highpass, 4th order, 20Hz cutoff, fs=200Hz
// Run this once in Python to get your exact values:
//   from scipy.signal import butter
//   print(butter(4, 20/100.0, btype='high', output='sos'))
const float SOS[2][6] = {
  { 0.62480f, -1.24960f, 0.62480f, 1.0f, -1.14948f,  0.29609f },
  { 1.00000f, -2.00000f, 1.00000f, 1.0f, -1.22534f,  0.64117f }
};
float sosState[2][N_CH][2] = {{{0}}};   // filter memory per section per channel

// ─── CALIBRATION STATE ────────────────────────────────────────────────────────
enum CalibState { CALIB_REST, CALIB_MAX, CALIB_DONE };
CalibState calibState    = CALIB_REST;
bool       calibStarted  = false;
float      calibAccum    = 0;
int        calibSamples  = 0;
unsigned long calibPhaseStart = 0;

float THR_DEAD = 0;     // derived at end of calibration
float THR_LOW  = 0;
float THR_MID  = 0;

// ─── BLE STATE ────────────────────────────────────────────────────────────────
BLEClient*           client    = nullptr;
BLEAdvertisedDevice* myoDevice = nullptr;
bool                 myoFound  = false;
bool                 scanning  = false;
unsigned long        lastDataTime = 0;

HardwareSerial STM32Serial(2);

// ─── DEBUG (moved out of callback) ───────────────────────────────────────────
volatile bool    newDebug    = false;
volatile float   debugRMS    = 0;
volatile uint8_t debugSpeed  = 0;

// ─── LED HELPERS ──────────────────────────────────────────────────────────────
enum LEDState { LED_SCANNING, LED_REST_PHASE, LED_FLEX_PHASE, LED_ACTIVE, LED_DISCONNECTED };
LEDState     currentLED      = LED_SCANNING;
unsigned long lastBlinkMs    = 0;
bool          blinkToggle    = false;

void setLED(bool r, bool g, bool b) {
  digitalWrite(LED_R, r ? HIGH : LOW);
  digitalWrite(LED_G, g ? HIGH : LOW);
  digitalWrite(LED_B, b ? HIGH : LOW);
}

void updateLED() {
  unsigned long now = millis();

  switch (currentLED) {
    case LED_SCANNING:
      // Blue slow blink — 600ms period
      if (now - lastBlinkMs > 600) {
        blinkToggle = !blinkToggle;
        lastBlinkMs = now;
        setLED(false, false, blinkToggle);
      }
      break;

    case LED_REST_PHASE:
      setLED(true, true, false);   // Yellow solid (R+G)
      break;

    case LED_FLEX_PHASE:
      setLED(true, false, false);  // Red solid
      break;

    case LED_ACTIVE:
      setLED(false, true, false);  // Green solid
      break;

    case LED_DISCONNECTED:
      // Red fast blink — 200ms period
      if (now - lastBlinkMs > 200) {
        blinkToggle = !blinkToggle;
        lastBlinkMs = now;
        setLED(blinkToggle, false, false);
      }
      break;
  }
}

// ─── BIQUAD FILTER ────────────────────────────────────────────────────────────
float applyHighpass(float x, int ch) {
  for (int s = 0; s < 2; s++) {
    float b0 = SOS[s][0], b1 = SOS[s][1], b2 = SOS[s][2];
    float a1 = SOS[s][4], a2 = SOS[s][5];
    float w  = x - a1 * sosState[s][ch][0] - a2 * sosState[s][ch][1];
    float y  = b0 * w + b1 * sosState[s][ch][0] + b2 * sosState[s][ch][1];
    sosState[s][ch][1] = sosState[s][ch][0];
    sosState[s][ch][0] = w;
    x = y;
  }
  return x;
}

// ─── SPEED QUANTIZATION WITH HYSTERESIS ──────────────────────────────────────
uint8_t rmsToSpeed(float rms) {
  static uint8_t lastSpeed = 0;
  float hys = (THR_MID - THR_DEAD) * 0.10f;  // 10% of range as hysteresis band

  if      (lastSpeed == 0)   { if (rms >= THR_DEAD + hys)    lastSpeed = 33;  }
  else if (lastSpeed == 33)  { if (rms <  THR_DEAD)           lastSpeed = 0;
                               else if (rms >= THR_LOW + hys) lastSpeed = 66;  }
  else if (lastSpeed == 66)  { if (rms <  THR_LOW  - hys)     lastSpeed = 33;
                               else if (rms >= THR_MID + hys) lastSpeed = 100; }
  else                       { if (rms <  THR_MID  - hys)     lastSpeed = 66;  }

  return lastSpeed;
}

// ─── CALIBRATION HANDLER ─────────────────────────────────────────────────────
void handleCalibration(float rms) {

  if (calibState == CALIB_REST) {
    if (!calibStarted) {
      Serial.println("[CALIB] Relax your arm...");
      currentLED        = LED_REST_PHASE;
      calibPhaseStart   = millis();
      calibStarted      = true;
      calibAccum        = 0;
      calibSamples      = 0;
    }
    calibAccum += rms;
    calibSamples++;

    if (millis() - calibPhaseStart >= CALIB_DURATION_MS) {
      THR_DEAD     = (calibAccum / calibSamples) * 1.5f;  // 1.5× noise floor
      calibState   = CALIB_MAX;
      calibStarted = false;
      Serial.printf("[CALIB] Deadband = %.1f — now flex hard!\n", THR_DEAD);
    }
  }

  else if (calibState == CALIB_MAX) {
    if (!calibStarted) {
      Serial.println("[CALIB] Flex as hard as you can...");
      currentLED        = LED_FLEX_PHASE;
      calibPhaseStart   = millis();
      calibStarted      = true;
      calibAccum        = 0;
      calibSamples      = 0;
    }
    calibAccum += rms;
    calibSamples++;

    if (millis() - calibPhaseStart >= CALIB_DURATION_MS) {
      float calibMaxRMS = calibAccum / calibSamples;
      float range       = calibMaxRMS - THR_DEAD;
      THR_LOW           = THR_DEAD + range * 0.33f;
      THR_MID           = THR_DEAD + range * 0.66f;

      calibState  = CALIB_DONE;
      currentLED  = LED_ACTIVE;

      Serial.printf("[CALIB] Done! dead=%.1f  low=%.1f  mid=%.1f  max=%.1f\n",
                    THR_DEAD, THR_LOW, THR_MID, calibMaxRMS);
    }
  }
}

// ─── RESET CALIBRATION ───────────────────────────────────────────────────────
void resetCalibration() {
  calibState   = CALIB_REST;
  calibStarted = false;
  calibAccum   = 0;
  calibSamples = 0;
  currentLED   = LED_REST_PHASE;
  Serial.println("[CALIB] Recalibrating...");
}

// ─── RMS ACCUMULATORS ────────────────────────────────────────────────────────
float    rmsAccum[N_CH] = {0};
uint16_t sampleCount    = 0;

// ─── EMG CALLBACK — only char 0 subscribed ───────────────────────────────────
void emgCallback(BLERemoteCharacteristic* pChar, uint8_t* data, size_t length, bool isNotify) {
  if (length < 16) return;
  lastDataTime = millis();

  for (int s = 0; s < 2; s++) {
    for (int ch = 0; ch < N_CH; ch++) {
      float raw      = (float)((int8_t)data[s * 8 + ch]);
      float filtered = applyHighpass(raw, ch);
      float rect     = fabsf(filtered);
      rmsAccum[ch]  += rect * rect;
    }
    sampleCount++;

    if (sampleCount >= RMS_WINDOW && sampleCount % UPDATE_EVERY == 0) {
      float avgRMS = 0;
      for (int ch = 0; ch < N_CH; ch++) {
        avgRMS       += sqrtf(rmsAccum[ch] / RMS_WINDOW);
        rmsAccum[ch] *= (float)(RMS_WINDOW - UPDATE_EVERY) / RMS_WINDOW;  // slide window
      }
      avgRMS /= N_CH;

      if (calibState != CALIB_DONE) {
        handleCalibration(avgRMS);
      } else {
        uint8_t speed = rmsToSpeed(avgRMS);
        uint8_t pkt[3] = { 0xAA, speed, (uint8_t)(0xAA ^ speed) };
        STM32Serial.write(pkt, 3);

        // Pass to loop() for debug print — never block inside callback
        debugRMS   = avgRMS;
        debugSpeed = speed;
        newDebug   = true;
      }
    }
  }
}

void imuCallback(BLERemoteCharacteristic* pChar, uint8_t* data, size_t length, bool isNotify) {
  lastDataTime = millis();
}

// ─── BLE SCAN CALLBACK ───────────────────────────────────────────────────────
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.isAdvertisingService(BLEUUID(CONTROL_SERVICE_UUID))) {
      Serial.println("Myo found!");
      myoDevice = new BLEAdvertisedDevice(advertisedDevice);
      myoFound  = true;
      scanning  = false;
      BLEDevice::getScan()->stop();
    }
  }
};

void startScan() {
  if (scanning) return;
  Serial.println("[BLE] Scanning...");
  currentLED = LED_SCANNING;
  scanning   = true;
  myoFound   = false;
  BLEScan* scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  scan->setActiveScan(true);
  scan->start(30);
}

// ─── CONNECT ─────────────────────────────────────────────────────────────────
bool connectToMyo() {
  if (client != nullptr) { client->disconnect(); delete client; client = nullptr; }

  client = BLEDevice::createClient();
  if (!client->connect(myoDevice)) {
    Serial.println("[BLE] Connection failed!");
    return false;
  }
  Serial.println("[BLE] Connected!");

  BLERemoteService* controlService = client->getService(CONTROL_SERVICE_UUID);
  if (!controlService) return false;

  BLERemoteCharacteristic* commandChar = controlService->getCharacteristic(COMMAND_UUID);
  if (!commandChar) return false;

  uint8_t cmdEMG[] = {0x01, 0x03, 0x03, 0x01, 0x01};   // raw EMG mode
  commandChar->writeValue(cmdEMG, sizeof(cmdEMG));

  BLERemoteService* imuService = client->getService(IMU_SERVICE_UUID);
  if (imuService) {
    BLERemoteCharacteristic* imuChar = imuService->getCharacteristic(IMU_DATA_UUID);
    if (imuChar) imuChar->registerForNotify(imuCallback);
  }

  BLERemoteService* emgService = client->getService(EMG_SERVICE_UUID);
  if (!emgService) return false;

  // KEY: only char 0 — all 4 are identical, subscribing to all inflates sampleCount 4×
  BLERemoteCharacteristic* emgChar = emgService->getCharacteristic(emgCharUUIDs[0]);
  if (!emgChar) return false;
  emgChar->registerForNotify(emgCallback);

  lastDataTime = millis();
  resetCalibration();   // always recalibrate on fresh connection
  return true;
}

// ─── SETUP ───────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(921600);

  pinMode(LED_R,    OUTPUT);
  pinMode(LED_G,    OUTPUT);
  pinMode(LED_B,    OUTPUT);
  pinMode(CALIB_BTN, INPUT_PULLUP);

  setLED(false, false, false);

  STM32Serial.begin(STM32_BAUD, SERIAL_8N1, STM32_RX_PIN, STM32_TX_PIN);
  BLEDevice::init("ESP32_Myo");
  startScan();
}

// ─── LOOP ────────────────────────────────────────────────────────────────────
void loop() {
  updateLED();

  // BLE connect
  if (myoFound && !scanning) {
    if (!connectToMyo()) { delay(2000); startScan(); }
    myoFound = false;
  }

  // Reconnect if dropped
  if (client != nullptr && !client->isConnected()) {
    currentLED = LED_DISCONNECTED;
    Serial.println("[BLE] Lost connection! Reconnecting...");
    delay(1000);
    if (myoDevice != nullptr) {
      if (!connectToMyo()) { delay(2000); startScan(); }
    } else {
      startScan();
    }
  }

  // Stale data watchdog — force reconnect if no packets for 5s
  if (client != nullptr && client->isConnected() &&
      lastDataTime != 0 && millis() - lastDataTime > 5000) {
    Serial.println("[BLE] No data — forcing reconnect...");
    client->disconnect();
  }

  // Calibration button — retrigger anytime
  if (digitalRead(CALIB_BTN) == LOW) {
    delay(50);   // debounce
    if (digitalRead(CALIB_BTN) == LOW) {
      resetCalibration();
      while (digitalRead(CALIB_BTN) == LOW);   // wait for release
    }
  }

  // Debug print (safe, outside BLE callback)
  if (newDebug) {
    newDebug = false;
    Serial.printf("[EMG] RMS=%.1f  Speed=%d\n", debugRMS, debugSpeed);
  }
}