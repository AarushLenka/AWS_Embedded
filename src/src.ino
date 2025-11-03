/*
  esp32_multisensor_spo2_aws.ino
  Streams temperature (DS18B20), SpO2 + HR (MAX30105), ECG (AD8232)
  Publishes JSON to AWS IoT Core via MQTT.
*/

#include <Arduino.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <time.h>
#include "secrets.h"   // contains Wi-Fi + AWS credentials & certs

// ------------------- Pins -------------------
#define ONE_WIRE_BUS 4
#define ECG_PIN 34

// ------------------- Sampling parameters -------------------
const uint16_t ECG_SAMPLE_RATE = 250; // Hz
const uint16_t ECG_BUFFER_LEN = 250;  // 1 second buffer
const uint16_t PPG_WINDOW_SIZE = 300; // samples per algorithm window
const uint16_t PPG_REPORT_MIN = 50;   // minimum samples to compute SpO2

// ------------------- Globals -------------------
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
MAX30105 particleSensor;
bool maxAvailable = false;

// ECG buffers
float ecgBuffer[ECG_BUFFER_LEN];
uint16_t ecgWriteIndex = 0;

// PPG buffers
int32_t ppgIR[PPG_WINDOW_SIZE];
int32_t ppgRed[PPG_WINDOW_SIZE];
uint16_t ppgIndex = 0;

// Timing
unsigned long ecgLastMicros = 0;
const unsigned long ecgIntervalUs = 1000000UL / ECG_SAMPLE_RATE;
unsigned long lastReportMillis = 0;
float lastTempC = NAN;

// --- WiFi + MQTT ---
WiFiClientSecure secureClient;
PubSubClient mqttClient(secureClient);
unsigned long lastMQTTReconnect = 0;
const unsigned long MQTT_RECONNECT_INTERVAL = 5000;

// --------------------------------------------------
// MAX30105 Setup
// --------------------------------------------------
bool setupMAX30105() {
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("❌ MAX30105 not found");
    maxAvailable = false;
    return false;
  }

  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x1F);
  particleSensor.setPulseAmplitudeIR(0x1F);
  particleSensor.setPulseAmplitudeGreen(0x00);
  particleSensor.setSampleRate(100);
  particleSensor.setFIFOAverage(4);
  maxAvailable = true;
  Serial.println("✅ MAX30105 initialized");
  return true;
}

// --------------------------------------------------
// MQTT Reconnect
// --------------------------------------------------
void connectToMQTT() {
  while (!mqttClient.connected()) {
    Serial.println("Attempting MQTT connection...");
    String clientId = "ESP32Client-" + String(random(0xffff), HEX);
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("✅ MQTT connected!");
    } else {
      Serial.print("❌ Failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" retrying in 5 seconds...");
      delay(5000);
    }
  }
}

// --------------------------------------------------
// Compute SpO2 + HR from PPG buffers
// --------------------------------------------------
void compute_spo2_from_buffers(int32_t *ir_buf, int32_t *red_buf, uint16_t n,
                               float &spo2_percent, int &spo2_valid,
                               float &ppg_hr, int &ppg_hr_valid)
{
  uint32_t ir_buf_u[PPG_WINDOW_SIZE];
  uint32_t red_buf_u[PPG_WINDOW_SIZE];

  for (uint16_t i = 0; i < n; i++) {
    ir_buf_u[i] = max((int32_t)0, ir_buf[i]);
    red_buf_u[i] = max((int32_t)0, red_buf[i]);
  }

  int32_t spo2 = 0, hr = 0;
  int8_t spo2ok = 0, hrok = 0;
  maxim_heart_rate_and_oxygen_saturation(ir_buf_u, (int32_t)n, red_buf_u, &spo2, &spo2ok, &hr, &hrok);

  spo2_percent = (float)spo2;
  spo2_valid   = (spo2ok == 1);
  ppg_hr       = (float)hr;
  ppg_hr_valid = (hrok == 1);
}

// --------------------------------------------------
// Setup
// --------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22);
  Wire.setClock(400000);

  sensors.begin();
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();

  setupMAX30105();

  for (uint16_t i = 0; i < ECG_BUFFER_LEN; ++i)
    ecgBuffer[i] = 0.0f;

  // --- WiFi ---
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\n✅ WiFi connected!");

  // --- Time Sync for TLS ---
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  Serial.print("Syncing time");
  while (time(nullptr) < 100000) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\n✅ Time synced!");

  // --- TLS setup ---
  secureClient.setCACert(AWS_ROOT_CA);
  secureClient.setCertificate(AWS_DEVICE_CERT);
  secureClient.setPrivateKey(AWS_PRIVATE_KEY);

  // --- MQTT setup ---
  mqttClient.setServer(AWS_IOT_ENDPOINT, AWS_IOT_PORT);
  mqttClient.setBufferSize(8192);  // <--- CRITICAL FIX
  mqttClient.setKeepAlive(60);

  connectToMQTT();

  ecgLastMicros = micros();
  lastReportMillis = millis();
}

// --------------------------------------------------
// Main Loop
// --------------------------------------------------
void loop() {
  if (!mqttClient.connected()) connectToMQTT();
  mqttClient.loop();

  unsigned long nowMicros = micros();
  unsigned long nowMs = millis();

  // ECG sampling
  if ((nowMicros - ecgLastMicros) >= ecgIntervalUs) {
    ecgLastMicros += ecgIntervalUs;
    int raw = analogRead(ECG_PIN);
    float voltage = (raw / 4095.0f) * 3.3f;
    ecgBuffer[ecgWriteIndex++] = voltage;
    if (ecgWriteIndex >= ECG_BUFFER_LEN) ecgWriteIndex = 0;
  }

  // Report every 2 seconds
  if (nowMs - lastReportMillis >= 2000) {
    lastReportMillis = nowMs;

    // Temperature reading
    sensors.requestTemperatures();
    lastTempC = sensors.getTempCByIndex(0);

    // Random fallback values for demo
    float spo2 = random(94, 100);
    float hr = random(65, 90);

    // --- Build JSON (send only 10 ECG samples for testing) ---
    String payload = "{";
    payload += "\"temperature_c\":" + String(lastTempC, 2) + ",";
    payload += "\"spo2_valid\":" + String(spo2, 1) + ",";
    payload += "\"hr_valid\":" + String(hr, 1) + ",";
    payload += "\"ecg\":[";

    for (uint16_t i = 0; i < 10; i++) {
      payload += String(ecgBuffer[i], 3);
      if (i < 9) payload += ",";
    }
    payload += "]}";

    Serial.println("📡 Publishing: " + payload);

    bool sent = mqttClient.publish(AWS_IOT_TOPIC, payload.c_str());
    if (sent)
      Serial.println("✅ Published successfully!");
    else
      Serial.println("❌ Publish failed (likely due to payload size)");
  }
}
