#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <arduinoFFT.h>

#define SERVICE_UUID        "12345678-1234-1234-1234-123456789abc"
#define CHARACTERISTIC_UUID "abcdefab-1234-1234-1234-abcdefabcdef"

// ~6 Hz streaming to HTML
uint16_t sampleCounterForStream = 0;
const float STREAM_RATE_HZ = 6.0f;

const int EEG_PIN = A0;
const uint16_t SAMPLES        = 256;
const float    SAMPLE_RATE_HZ = 256.0f;

double vReal[SAMPLES];
double vImag[SAMPLES];

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLE_RATE_HZ);

const unsigned long SAMPLE_PERIOD_US = (unsigned long)(1e6 / SAMPLE_RATE_HZ);

const float AREF_V   = 3.3f;
const int   ADC_BITS = 12;
const int   ADC_MAX  = (1 << ADC_BITS) - 1;

// FFT bin indices (already in bins, not Hz)
const uint16_t DELTA_LOW  = 1,  DELTA_HIGH = 4;
const uint16_t THETA_LOW  = 4,  THETA_HIGH = 8;
const uint16_t ALPHA_LOW  = 8,  ALPHA_HIGH = 13;
const uint16_t BETA_LOW   = 13, BETA_HIGH  = 30;
const uint16_t GAMMA_LOW  = 30, GAMMA_HIGH = 45;

// --------- TIME-DOMAIN DENOISING ---------

// Slow baseline tracker for high-pass (DC / drift removal)
float notchState = 0.0f;
const float NOTCH_COEFF = 0.05f;   // smaller = slower; removes lower-frequency drift

// Simple 1st-order low-pass filter (currently bypassed in rawMv)
float lpState = 0.0f;
const float LP_ALPHA = 0.3f;       // less smoothing than 0.1 if you decide to use it

// --------- BAND POWERS (no smoothing) ---------

float deltaBP = 0.0f, thetaBP = 0.0f, alphaBP = 0.0f;
float betaBP  = 0.0f, gammaBP = 0.0f;

// --------- BLE CALLBACKS ---------

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    deviceConnected = true;
  }
  void onDisconnect(BLEServer* pServer) override {
    deviceConnected = false;
    BLEDevice::startAdvertising();
  }
};

// --------- HELPERS ---------

float bandPower(uint16_t lowBin, uint16_t highBin) {
  if (highBin > (SAMPLES / 2)) {
    highBin = SAMPLES / 2;
  }
  float sum = 0.0f;
  uint16_t count = 0;
  for (uint16_t i = lowBin; i <= highBin; i++) {
    float mag = vReal[i];
    sum += mag * mag;  // power ~ magnitude^2
    count++;
  }
  return (count > 0) ? sqrtf(sum / count) : 0.0f;
}

// --------- SETUP ---------

void setup() {
  Serial.begin(115200);
  analogSetAttenuation(ADC_11db);
  analogReadResolution(ADC_BITS);
  Serial.println("raw_mV,delta,theta,alpha,beta,gamma");

  // BLE setup
  BLEDevice::init("ESP32");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );

  pService->start();
  pServer->getAdvertising()->start();

  Serial.println("BLE ready!");
}

// --------- LOOP ---------

void loop() {
  // --------- SAMPLE BLOCK ---------
  unsigned long tNext = micros();

  for (uint16_t i = 0; i < SAMPLES; i++) {
    while (micros() < tNext) {}      // wait for next sample time
    tNext += SAMPLE_PERIOD_US;

    int raw = analogRead(EEG_PIN);

    // Convert ADC reading to mV (roughly centered around 0)
    float sample = ((float)raw / (float)ADC_MAX) * AREF_V * 1000.0f - (AREF_V * 500.0f);

    // 1) HIGH-PASS / DC REMOVAL
    notchState = notchState + NOTCH_COEFF * (sample - notchState);
    float hpSample = sample - notchState;

    // 2) LOW-PASS FILTER (currently bypassed for raw)
    lpState = lpState + LP_ALPHA * (hpSample - lpState);
    // float rawNow = lpState;     // more smoothing
    float rawNow = hpSample;       // less smoothing, just high-pass

    // store for FFT
    vReal[i] = rawNow;
    vImag[i] = 0.0;

    // -------- 6 Hz BLE streaming of all channels --------
    if (deviceConnected) {
      sampleCounterForStream++;
      uint16_t samplesPerUpdate = (uint16_t)(SAMPLE_RATE_HZ / STREAM_RATE_HZ); // ~43 for 256/6
      if (sampleCounterForStream >= samplesPerUpdate) {
        sampleCounterForStream = 0;

        char buf[80];
        snprintf(
          buf, sizeof(buf),
          "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
          rawNow, deltaBP, thetaBP, alphaBP, betaBP, gammaBP
        );
        pCharacteristic->setValue((uint8_t*)buf, strlen(buf));
        pCharacteristic->notify();
      }
    }

    // -------- Serial debug (optional) --------
    Serial.print(rawNow, 2);   Serial.print(',');
    Serial.print(deltaBP, 2);  Serial.print(',');
    Serial.print(thetaBP, 2);  Serial.print(',');
    Serial.print(alphaBP, 2);  Serial.print(',');
    Serial.print(betaBP, 2);   Serial.print(',');
    Serial.println(gammaBP, 2);
  }

  // last sample in this block
  float rawMv = (float)vReal[SAMPLES - 1];

  // --------- FFT + INSTANT BAND POWERS ---------
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  deltaBP = bandPower(DELTA_LOW,  DELTA_HIGH);
  thetaBP = bandPower(THETA_LOW,  THETA_HIGH);
  alphaBP = bandPower(ALPHA_LOW,  ALPHA_HIGH);
  betaBP  = bandPower(BETA_LOW,   BETA_HIGH);
  gammaBP = bandPower(GAMMA_LOW,  GAMMA_HIGH);

  // (Optional) once-per-block BLE send; usually not  // if (deviceConnected) {
  //   char buf[80];
  //   snprintf(
  //     buf, sizeof(buf),
  //     "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
  //     rawMv, deltaBP, thetaBP, alphaBP, betaBP, gammaBP
  //   );
  //   pCharacteristic->setValue((uint8_t*)buf, strlen(buf));
  //   pCharacteristic->notify();
  // }
}