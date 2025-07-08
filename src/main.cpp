////////////////////////////////////////////////////////////////////////////////////////
//                                                                                    //
//                                      Dimmo                                         //
//                      * TRIAC FIRING ANGLE CONTROL SYSTEM *                         //
//                                                                                    //
////////////////////////////////////////////////////////////////////////////////////////
//                                   Version: 2.9                                     //
////////////////////////////////////////////////////////////////////////////////////////
//                                                                                    //
//   Description:                                                                     //
//   This project implements an advanced closed-loop control system for TRIAC firing  //
//   angle control with IoT capabilities. Features include precise AC load dimming,   //
//   real-time power monitoring, and multi-protocol remote control. The system        //
//   incorporates hardware safety measures and supports secure OTA updates.           //
//                                                                                    //
////////////////////////////////////////////////////////////////////////////////////////
//                  * Features marked with (+) will be added soon *                   //
////////////////////////////////////////////////////////////////////////////////////////
//                                                                                    //
//   Features:                                                                        //
//     Zero-crossing detection with hardware debouncing                               //
//     Automatic frequency measurement for 50/60Hz AC signals                         //
//     Dynamic firing angle control (0° to 180°) with microsecond precision           //
//     LCD display for real-time monitoring                                           //
//     Remote control capability via MQTT                                             //
//     Multi-protocol remote control:                                                 //
//      - Secure MQTT over TLS                                                        //
//      + WebSerial (HTTP) and TelnetStream access (in development)                   //
//     Advanced memory monitoring with auto-recovery                                  //
//     NTP-synchronized timestamps (Tehran UTC+3:30)                                  //
//     Encrypted OTA updates with rollback protection                                 //
//                                                                                    //
////////////////////////////////////////////////////////////////////////////////////////
//                                                                                    //
//   Hardware:                                                                        //
//   - ESP32-WROOM microcontroller                                                    //
//   - H11AA1 optocoupler with noise filtering circuit                                //
//   - BT139 TRIAC with snubber network                                               //
//   - 16x2 I2C LCD with backlight control                                            //
//   - ZMPT101B voltage sensor + ACS712 current sensor                                //
//                                                                                    //
////////////////////////////////////////////////////////////////////////////////////////
//                                                                                    //
//   Please note that this project is still under development and may contain bugs.   //
//                                                                                    //
////////////////////////////////////////////////////////////////////////////////////////


#include <Arduino.h>
#include <Secrets.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>

#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include <NTPClient.h>

// #include <EEPROM.h>
// #include <SPIFFS.h>


// ================ Hardware Configuration ================
LiquidCrystal_I2C lcd(0x27, 16, 2);  // LCD connected to I2C (GPIO 21 = SDA, GPIO 22 = SCL)


#define VREF 3.3      // 3.3V or 5V (based on your setup)
#define adcResolution 4095 // ESP32 12-bit ADC resolution (0-4095)

// Pin Definitions
#define zeroCrossingPin 17    // Zero-crossing detector
#define TriacTrigger 4       // TRIAC gate
#define zmptPin 36           // ZMPT101B voltage sensor
#define acsPin 39            // ACS712 current sensor

// Memory Thresholds (for ESP32-WROOM with 320KB RAM)
#define WARNING_HEAP_THRESHOLD   80000  // 80KB (25% of total)
#define CRITICAL_HEAP_THRESHOLD  50000  // 50KB (15% of total) 
#define RESTART_HEAP_THRESHOLD   30000  // 30KB (10% of total)
#define MAX_RESTARTS             3      // Max allowed restarts

// ================ Time Configuration ================
WiFiUDP ntpUDP;
// Tehran UTC+3:30 (12600 seconds), update every 60 seconds
NTPClient timeClient(ntpUDP, "pool.ntp.org", 12600, 60000); 

// ================ Device Identification ================
// const String deviceId = "PWR-" + String(WiFi.macAddress()).substring(9, 14); // Last 5 chars of MAC
const String deviceId = "esp32-client-" + String(WiFi.macAddress());

// ================ Power Measurement Variables ================
volatile unsigned long previousMicros = 0;  // will store last time of the pulse
volatile unsigned long durationSum = 0;     // accumulates pulse width (period)
volatile unsigned int pulseCount = 0;       // accumulates pulse count (frequency)
volatile float measuredFrequency = 50.0;    // Initial assumed frequency

// TRIAC Control
volatile int firingAngle = 0;               // Firing angle in degrees (0° to 180°)
volatile float timePerDegree = 55.56;       // Time per degree in microseconds (for 50Hz)
volatile unsigned long delayTime = firingAngle * timePerDegree; // Delay in microseconds
bool fireTriac = false; // Flag to indicate when to fire TRIAC

// Sensor Values
volatile float measuredVoltage = 0;          // Measured voltage from ZMPT101B sensor
volatile float measuredCurrent = 0;          // Measured current from ACS712 sensor
volatile float measuredPower = 0;            // Measured power (Vrms * Irms)
volatile float measuredPowerConsumption = 0; // Accumulated power consumption (kWh)

// ================ ZMPT101B Configuration ================
// Calibration values obtained from the sketch: ADC_Readings
const int adc_max = 2399;               // Maximum sensor value during calibration
const int adc_min = 1477;               // Minimum sensor value during calibration

const float volt_multi = 230;                     // RMS voltage obtained from a multimeter
const float volt_multi_p = volt_multi * 1.4142;   // Peak voltage = RMS voltage * 1.4142
const float volt_multi_n = -volt_multi_p;         // Negative peak voltage

const float minVoltage = 40.0;
const float maxVoltage = 300.0;

// ================ ACS712 Configuration =================
// Sensor Configuration
#define numSample 100            // Number of samples per AC cycle
#define numCycles 10             // Number of AC cycles to average

const float mVperAmp = 66.0;     // ACS712-30A sensitivity (66mV/A)

// Calibration values
float expectedZero = 0.0;        // Calibrated zero-current ADC value
float voltageOffset = 0.0;       // Fine-tune zero current
float currentSlope = 1.0;        // Sensitivity adjustment (scaling)

// Constants for AC cycle duration and sampling
const float acCycleDuration = 1000.0 / measuredFrequency; // AC cycle duration in milliseconds
const int TOTAL_SAMPLES = numSample * numCycles; // Total samples over all cycles
const float samplingInterval = acCycleDuration / numSample; // Sampling interval (ms)
const int SAMPLE_DELAY_US = samplingInterval * 1000; // Sampling interval in microseconds


// ================ POWER METER Configuration ================
// volatile float realPower = 0;
// volatile float apparentPower = 0;
// volatile float powerFactor = 0;

// ================ Network Configuration ================
const char *ssid = WIFI_SSID;
const char *password = WIFI_PASSWORD;
const char *mqtt_broker = MQTT_BROKER;
const char *mqtt_topic = MQTT_TOPIC;
const char *debug_topic = DEBUG_TOPIC;
const char *mqtt_username = MQTT_USERNAME;
const char *mqtt_password = MQTT_PASSWORD;
const int mqtt_port = MQTT_PORT;

// Load DigiCert Global Root CA ca_cert, which is used by EMQX Serverless Deployment
const char* ca_cert = CERTIFICATE_AUTHORITY;

// WiFi and MQTT client initialization
WiFiClientSecure esp_client;
PubSubClient mqtt_client(esp_client);

// ================ OTA Configuration ================
const char* ota_hostname = OTA_HOSTNAME;
const char* ota_password = OTA_PASSWORD;
const int ota_port = OTA_PORT;

// ================ Timer Interrupts ================
// Timer interrupt for periodic reporting
hw_timer_t *reportTimer = NULL;     // Declare a timer variable
volatile bool reportFlag = true;    // Flag to indicate it's time to report
int timerInterval = 10000000;       // 10-second interval (10,000,000 µs)

// Timer-based TRIAC firing (instead of delay in ISR)
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Mutex for critical sections
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// ================ ERROR HANDLING ==================
// Error codes
#define ERR_UNKNOWN_CMD           0
#define ERR_NO_SIGNAL             1
#define ERR_INVALID_MEASUREMENT   2
#define ERR_FREQ_OUT_OF_RANGE     3
#define ERR_VOLTAGE_OUT_OF_RANGE  4
#define ERR_INVALID_MSG           5
#define ERR_INVALID_VALUE         6

// ==================== Functions ====================

void copyright();

void setupNTP();
String getTimestamp();
String createLogPayload(const String& message);
void publishDebug(const String& message);

void setupOTA();

void connectToWiFi();
void connectToMQTT();

void IRAM_ATTR zeroCrossingISR();
void IRAM_ATTR fireTriacISR();
void IRAM_ATTR reportISR();

void enableInterrupts();
void disableInterrupts();
void emergencyStop();

float reportFrequency();
float reportVoltage();
float reportCurrent();
void publishFrequency(float Frequency);
void publishVoltage(float Voltage);
void publishCurrent(float Current);
void publishPower(float Power);
void publishPowerConsumption(float PowerConsumption);

bool isNumeric(String str);
void mqttCallback(char *topic, byte *payload, unsigned int length);

void checkMemory();
void handleError(int errorCode, const char* message);

// void calculateTruePower();
// void updatePowerConsumption(float powerWatts);
// void saveEnergyData();
// bool loadEnergyData();
// uint32_t calculateChecksum(EnergyData& data);

//------------------------------------------------------------
// Initialize NTP time client
void setupNTP() {
  timeClient.begin();
  if (!timeClient.forceUpdate()) {
    Serial.println("Failed to get NTP time!");
  }
}

//------------------------------------------------------------
// Generate a timestamp in the format YYYY-MM-DD HH:MM:SS
String getTimestamp() {
  timeClient.update();
  time_t epoch = timeClient.getEpochTime();
  struct tm *ptm = gmtime(&epoch);
  
  // Convert format
  char buf[30];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", ptm);
  return String(buf);
}

//------------------------------------------------------------
// Create a structured JSON debug message
String createLogPayload(const String& message) {
  return "{\"device\":\"" + deviceId + "\","
         "\"timestamp\":\"" + getTimestamp() + "\","
         "\"timezone\":\"Asia/Tehran\","
         "\"message\":\"" + message + "\"}";
}

//------------------------------------------------------------
// Publish debug messages with throttling
void publishDebug(const String& message) {
  static unsigned long lastSend = 0;
  static String buffer;
  
  buffer += message;
  
  if (millis() - lastSend >= 500 && mqtt_client.connected()) {
    String payload = createLogPayload(buffer);
    mqtt_client.publish(debug_topic, payload.c_str());
    buffer = "";
    lastSend = millis();
  }
  
  Serial.print(message);
}

//------------------------------------------------------------
// Setup secure OTA for firmware updates
void setupOTA() {
  // Configure OTA
  ArduinoOTA.setPort(ota_port);
  ArduinoOTA.setHostname(ota_hostname);
  ArduinoOTA.setPassword(ota_password);
  ArduinoOTA.setRebootOnSuccess(true);
  ArduinoOTA.setMdnsEnabled(true);

  // OTA Event Handlers
  ArduinoOTA.onStart([]() {
    String type = (ArduinoOTA.getCommand() == U_FLASH) ? "firmware" : "filesystem";
    
    // User feedback
    Serial.printf("\nOTA Started: %s update\n", type.c_str());
    publishDebug("\nOTA Started: " + type);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("OTA Update");
    lcd.setCursor(0, 1);
    lcd.printf("Mode: %s", type.c_str());
    delay(50);
    
    // Freeze all non-essential operations
    emergencyStop();
    disableInterrupts();
    mqtt_client.disconnect();
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    static unsigned long lastUpdate = 0;
    int percent = (progress * 100) / total;
    
    // Throttle updates to 10Hz max
    if(millis() - lastUpdate > 100) {
      Serial.printf("Progress: %d%%\r", percent);
      // publishDebug("\nProgress: " + String(percent));
      lcd.clear();
      lcd.setCursor(0, 1);
      lcd.printf("Uploading: %3d%%", percent);
      lastUpdate = millis();
    }
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA Finished, restarting...");
    publishDebug("\nOTA Finished, restarting...");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Update complete");
    lcd.setCursor(0, 1);
    lcd.print("Rebooting...");
    delay(1000);
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("\nOTA Error[%u]: ", error);
    // publishDebug("\nOTA Error: " + String(error));
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("OTA Error");
    
    const char* errorMsg = "";
    switch(error) {
      case OTA_AUTH_ERROR:    errorMsg = "Auth Failed"; break;
      case OTA_BEGIN_ERROR:   errorMsg = "Begin Failed"; break;
      case OTA_CONNECT_ERROR: errorMsg = "Connect Failed"; break;
      case OTA_RECEIVE_ERROR: errorMsg = "Receive Failed"; break;
      case OTA_END_ERROR:     errorMsg = "End Failed"; break;
      default:                errorMsg = "Unknown Error"; break;
    }
    
    Serial.println(errorMsg);
    publishDebug("\n[Error]Details: " + String(errorMsg));
    lcd.setCursor(0, 1);
    lcd.print(errorMsg);
    
    // Attempt recovery after delay
    delay(5000);
    enableInterrupts();
    ESP.restart();
  });

  // Start OTA service
  ArduinoOTA.begin();
  
  Serial.println("\nOTA Update Service Ready");
  Serial.printf("Hostname: %s.local\n", ota_hostname);
  Serial.printf("IP: %s\n", WiFi.localIP().toString().c_str());
  Serial.printf("Port: %d\n", ota_port);

  publishDebug("\nOTA Update Service Ready");
  delay(50);
  publishDebug("\nHostname: " + String(ota_hostname) + ".local");
  delay(50);
  publishDebug("\nIP: " + WiFi.localIP().toString());
  delay(50);
  publishDebug("\nPort: " + String(ota_port));
  delay(50);
  
  // Display on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("OTA Ready");
  lcd.setCursor(0, 1);
  lcd.print(WiFi.localIP());
}

//------------------------------------------------------------
// Connect to WiFi network
void connectToWiFi() {
  // WiFi.mode(WIFI_STA);
  // WiFi.setHostname(ota_hostname);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to : ");
  Serial.print(ssid);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Connecting to : ");
  lcd.setCursor(0,1);
  lcd.print(ssid);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConnected to WiFi");
}

//------------------------------------------------------------
// Connect to MQTT broker
void connectToMQTT() {
  // Configure MQTT
  esp_client.setCACert(ca_cert);  // Set root CA certificate
  mqtt_client.setServer(mqtt_broker, mqtt_port);
  mqtt_client.setKeepAlive(60);
  mqtt_client.setCallback(mqttCallback);

  while (!mqtt_client.connected()) {
    // String clientId = "ESP32-" + String(random(0xffff), HEX);
    Serial.printf("Connecting to MQTT Broker as %s...\n", deviceId.c_str());

    if (mqtt_client.connect(deviceId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("Connected to MQTT broker");
      publishDebug("\nConnected to MQTT broker");
      delay(50);
      mqtt_client.subscribe(mqtt_topic);  // Subscribe to the topic
      mqtt_client.publish(mqtt_topic, "Hi EMQX I'm ESP32 ^^");  // Publish message upon connection
    } else {
      Serial.print("Failed to connect to MQTT broker, rc=");
      Serial.print(mqtt_client.state());
      Serial.println(" Retrying in 5 seconds.");
      publishDebug("\nMQTT Connection Failed: " + String(mqtt_client.state()) + "\nRetrying in 5 seconds...");
      delay(5000);
    }
  }
}

//------------------------------------------------------------
// Interrupt handler for zero-crossing detection
void IRAM_ATTR zeroCrossingISR() {
  unsigned long currentMicros = micros();

  portENTER_CRITICAL_ISR(&mux);  // Protect against race conditions
  // durationSum += (currentMicros - previousMicros) & 0xFFFFFFFF;  // Overflow-safe
  durationSum += currentMicros - previousMicros;
  previousMicros = currentMicros;
  pulseCount++;
  portEXIT_CRITICAL_ISR(&mux);

  timerWrite(timer, 0);  // Reset timer
  timerAlarmWrite(timer, delayTime, false); // Set delay based on firing angle
  timerAlarmEnable(timer);  // Start timer

  // if (fireTriac) {
  // timerAlarmWrite(timer, delayTime, false);   // Auto-resets timer
  // timerAlarmEnable(timer);
  // }
}

//------------------------------------------------------------
// Timer interrupt for TRIAC firing
void IRAM_ATTR fireTriacISR() {
  if (fireTriac) {
  portENTER_CRITICAL_ISR(&timerMux);
  digitalWrite(TriacTrigger, HIGH);
  delayMicroseconds(10); // Short pulse to trigger TRIAC
  digitalWrite(TriacTrigger, LOW);
  portEXIT_CRITICAL_ISR(&timerMux);
  } else {
    digitalWrite(TriacTrigger, LOW);
  }
}

//------------------------------------------------------------
// Timer interrupt for periodic reporting
void IRAM_ATTR reportISR() {
  reportFlag = true;  // Set the flag
}

//------------------------------------------------------------
// Enable interrupts and timers for normal operation
void enableInterrupts() {
  attachInterrupt(digitalPinToInterrupt(zeroCrossingPin), zeroCrossingISR, CHANGE);
  timerAlarmEnable(timer);
  timerAlarmEnable(reportTimer);
}

//------------------------------------------------------------
// Disable interrupts and timers for safety
void disableInterrupts() {
  detachInterrupt(digitalPinToInterrupt(zeroCrossingPin));
  timerAlarmDisable(timer);
  timerAlarmDisable(reportTimer);
}

//------------------------------------------------------------
// Interrupt Safety Critical Section
void emergencyStop() {
  fireTriac = false;
  digitalWrite(TriacTrigger, LOW);
}

//------------------------------------------------------------
// Report frequency and time per degree
float reportFrequency() {
  float frequency = 0;
  portENTER_CRITICAL(&mux);

  if (pulseCount > 0 && durationSum > 0) {
    // Each pulse represents half-cycle (zero-crossing), so full cycle is 2 pulses
    // float avgHalfPeriod = (float)durationSum / (float)pulseCount; // µs per half-cycle
    // frequency = 1e6 / (2.0 * avgHalfPeriod); // Convert to Hz (full cycle)

    frequency = 1e6 / float(durationSum) * (float(pulseCount) / 4.0);

    // frequency = (1e6 * float(pulseCount)) / (2.0 * float(durationSum));

    // Validate reasonable frequency range (45-65Hz for most power systems)
    // if (frequency <= 45.0 || frequency >= 65.0) {
    //   frequency = 0; // Indicate error
    // }

  } else {
    frequency = 0;
    // Serial.println("No signal"); // This is redundant with the debug output below
  }

  // clear counters
  durationSum = 0;
  pulseCount = 0;
  portEXIT_CRITICAL(&mux);

  // Debug output
  if (frequency > 45.0 && frequency < 65.0) {
    // Time per degree calculation (based on the full AC cycle)
    timePerDegree = (1e6 / frequency) / 360.0; // µs per degree
    Serial.printf("Frequency: %.2f Hz | Time/Degree: %.2f µs\n", frequency, timePerDegree);
  } else if (frequency <= 0) {
    Serial.println("Frequency measurement error - no signal detected");
  } else {
    Serial.printf("Frequency measurement error - %.2f Hz out of range (should be 45-65Hz)\n", frequency);
  }

  return frequency;
}

//------------------------------------------------------------
// Read voltage from ZMPT10B sensor
float reportVoltage() {
  float adc_sample;
  float volt_inst = 0;
  float sum = 0;
  int N = 0;

  unsigned long startTime = millis();

  // Duration of 0.5 seconds (500ms)
  while ((millis() - startTime) < 500) {
    adc_sample = analogRead(zmptPin);         // Sensor voltage
    volt_inst = map(adc_sample, adc_min, adc_max, volt_multi_n, volt_multi_p);
    sum += sq(volt_inst);                    // Sum of Squares
    N++;
  }

  // Serial.print("Sum of Squares: ");
  // Serial.println(sum);
  // Serial.print("Number of Samples: ");
  // Serial.println(N);

  float rmsVoltage = sqrt(sum / N);                     // RMS equation

  // Add validation
  if (rmsVoltage < minVoltage || rmsVoltage > maxVoltage) {
    // handleError(ERR_VOLTAGE_OUT_OF_RANGE, "Voltage out of range");
    return 0;
  }

  return rmsVoltage;
}

//------------------------------------------------------------
// Calibrate the sensor to get the zero-current ADC value.
void calibrateACS712() {
  float sum = 0;
  int numSamples = 3000; // Take 3000 samples to get an accurate baseline
  for (int i = 0; i < numSamples; i++) {
    float adcValue = analogRead(acsPin);
    sum += adcValue;

    // Optional: Ignore outliers (basic noise filter during calibration)
    if (abs(adcValue - expectedZero) > 10) {
      continue; // Ignore values that deviate significantly
    }

    delayMicroseconds(100); // Small delay to prevent oversampling noise
  }
  expectedZero = sum / numSamples;
  
  Serial.print("Calibrated Zero ADC: ");
  Serial.println(expectedZero, 1);
  publishDebug("\nCalibrated Zero ADC: " + String(expectedZero, 1));

}

//------------------------------------------------------------
// Measure the current and return the RMS current value in milliamps (mA).
float reportCurrent() {
  float sumSquares = 0;          // Sum of squared current values
  float sumAdc = 0;              // Sum of raw ADC values
  float sumVoltage = 0;          // Sum of calculated voltages

  // Perform the measurement
  for (int i = 0; i < TOTAL_SAMPLES; i++) {
    // Step 1: Read raw ADC
    float adcValue = analogRead(acsPin);
    sumAdc += adcValue;

    // Step 2: Convert to voltage
    float voltage = (adcValue / adcResolution) * VREF;
    sumVoltage += voltage;

    // Step 3: Convert to current
    float current = ((adcValue - expectedZero) * VREF / adcResolution) * 1000.0 / mVperAmp; // Current in mA

    // Step 4: Square the current value for RMS
    sumSquares += sq(current);

    // Step 5: Wait for the next sample
    delayMicroseconds(SAMPLE_DELAY_US);
  }

  // Calculate RMS current
  float rmsCurrent = sqrt(sumSquares / TOTAL_SAMPLES);
  rmsCurrent = (rmsCurrent + voltageOffset) * currentSlope; // Fine-tuned RMS current

  rmsCurrent = 45; // Placeholder for debugging

  // Ignore small currents (noise threshold)
  if (rmsCurrent < 0.1) {
    rmsCurrent = 0.0; // Treat as zero
  }

  return rmsCurrent; // Return the measured RMS current
}

//------------------------------------------------------------
// Power calculations
// void calculateTruePower() {
// }

//------------------------------------------------------------
// Publish frequency to MQTT
void publishFrequency(float Frequency) {
  // Convert the float to a string with 2 decimal places
  char frequencyStr[10]; // Buffer to hold the formatted string
  snprintf(frequencyStr, sizeof(frequencyStr), "%.2f", Frequency);

  // Format the message as "F <frequency>"
  char mqttMessage[20]; // Buffer to hold the final MQTT message
  snprintf(mqttMessage, sizeof(mqttMessage), "F %s", frequencyStr);

  // Publish the message to the MQTT topic
  mqtt_client.publish(mqtt_topic, mqttMessage);

  // Debug output
  Serial.print("Published frequency: ");
  Serial.println(mqttMessage);
}

//------------------------------------------------------------
// Publish Voltage to MQTT
void publishVoltage(float Voltage) {
  // Convert the float to a string with 2 decimal places
  char voltageStr[10]; // Buffer to hold the formatted string
  snprintf(voltageStr, sizeof(voltageStr), "%.2f", Voltage);

  // Format the message as "V <voltage>"
  char mqttMessage[20]; // Buffer to hold the final MQTT message
  snprintf(mqttMessage, sizeof(mqttMessage), "V %s", voltageStr);

  // Publish the message to the MQTT topic
  mqtt_client.publish(mqtt_topic, mqttMessage);

  // Debug output
  Serial.print("Published voltage: ");
  Serial.println(mqttMessage);
}

//------------------------------------------------------------
// Publish Current to MQTT
void publishCurrent(float Current) {
  // Convert the float to a string with 2 decimal places
  char CurrentStr[10]; // Buffer to hold the formatted string
  snprintf(CurrentStr, sizeof(CurrentStr), "%.2f", Current);

  // Format the message as "A <Current>"
  char mqttMessage[20]; // Buffer to hold the final MQTT message
  snprintf(mqttMessage, sizeof(mqttMessage), "I %s", CurrentStr);

  // Publish the message to the MQTT topic
  mqtt_client.publish(mqtt_topic, mqttMessage);

  // Debug output
  Serial.print("Published Current: ");
  Serial.println(mqttMessage);
}

//------------------------------------------------------------
// Publish Power to MQTT
void publishPower(float Power) {
  // Convert the float to a string with 2 decimal places
  char PowerStr[10]; // Buffer to hold the formatted string
  snprintf(PowerStr, sizeof(PowerStr), "%.2f", Power);

  // Format the message as "P <Power>"
  char mqttMessage[20]; // Buffer to hold the final MQTT message
  snprintf(mqttMessage, sizeof(mqttMessage), "P %s", PowerStr);

  // Publish the message to the MQTT topic
  mqtt_client.publish(mqtt_topic, mqttMessage);

  // Debug output
  Serial.print("Published Power: ");
  Serial.println(mqttMessage);
}

//------------------------------------------------------------
// Publish Power Consumption to MQTT
void publishPowerConsumption(float PowerConsumption) {
  // Convert the float to a string with 2 decimal places
  char PowerConsumptionStr[10]; // Buffer to hold the formatted string
  snprintf(PowerConsumptionStr, sizeof(PowerConsumptionStr), "%.2f", PowerConsumption);

  // Format the message as "PC <PowerConsumption>"
  char mqttMessage[20]; // Buffer to hold the final MQTT message
  snprintf(mqttMessage, sizeof(mqttMessage), "PC %s", PowerConsumptionStr);

  // Publish the message to the MQTT topic
  mqtt_client.publish(mqtt_topic, mqttMessage);

  // Debug output
  Serial.print("Published Power Consumption: ");
  Serial.println(mqttMessage);
}

//------------------------------------------------------------
// Helper function to check if string is numeric
bool isNumeric(String str) {
  for (unsigned int i = 0; i < str.length(); i++) {
    if (!isdigit(str.charAt(i))) {
      return false;
    }
  }
  return str.length() > 0;
}

//------------------------------------------------------------
// MQTT callback function
void mqttCallback(char *topic, byte *payload, unsigned int length) {
  // Safety check for empty payload
  if (length == 0) {
    Serial.println("Empty message received");
    return;
  }

  // Create buffer with space for null terminator
  char messageTemp[length + 1]; // Use a char array instead of String
  unsigned int index = 0;

  // Copy printable characters to messageTemp
  for (unsigned int i = 0; i < length; i++) {
    if (isPrintable(payload[i])) {
      messageTemp[index++] = (char)payload[i];
    }
    // Prevent buffer overflow
    if (index >= length) break;
  }
  messageTemp[index] = '\0'; // Null-terminate the string

  // Create String object and trim leading/trailing whitespace
  String message = String(messageTemp);
  message.trim();

  Serial.println("\n-----------------------");
  Serial.print("Message received on topic: ");
  Serial.println(topic);
  Serial.print("Message: ");
  Serial.println(message);

  // Process the message based on its content
  if (message == "Hi EMQX I'm ESP32 ^^") {
    Serial.println("Connection established with MQTT Broker");
  } 
  else if (message == "ON") {
    fireTriac = true;
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("Triac turned ON");
  } 
  else if (message == "OFF") {
    fireTriac = false;
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("Triac turned OFF");
  } 
  else if (isNumeric(message)) {  // Handle all numeric values 0-100
    int brightness = message.toInt();
    if (brightness >= 0 && brightness <= 100) {
      firingAngle = map(brightness, 0, 100, 160, 20); // Map brightness to firing angle
      // firingAngle = constrain(firingAngle, 20, 160); // Clamp to safe range
      
      delayTime = firingAngle * timePerDegree;
      
      Serial.printf("Brightness: %d%% | Firing Angle: %d° | Delay: %dμs\n", 
                   brightness, firingAngle, delayTime);
      
      // Auto power management (ON if ≥15%, OFF if <15%)
      bool shouldBeOn = brightness >= 15;
      if (shouldBeOn != fireTriac) {
        fireTriac = shouldBeOn;
        digitalWrite(LED_BUILTIN, shouldBeOn ? HIGH : LOW);
        Serial.printf("Triac auto-%s\n", shouldBeOn ? "ON" : "OFF");
      }
    } else {
      Serial.println("Invalid brightness value (0-100 allowed)");
    }
  }
  else if (message.startsWith("F ")) {
    float frequency = message.substring(2).toFloat();
    Serial.print("Frequency: ");
    Serial.print(frequency);
    Serial.println("Hz");
  } 
  else if (message.startsWith("V ")) {
    float voltage = message.substring(2).toFloat();
    Serial.print("Voltage: ");
    Serial.print(voltage);
    Serial.println("V");
  } 
  else if (message.startsWith("I ")) {
    float current = message.substring(2).toFloat();
    Serial.print("Current: ");
    Serial.print(current);
    Serial.println("A");
  }
  else if (message.startsWith("P ")) {
    float power = message.substring(2).toFloat();
    Serial.print("Power: ");
    Serial.print(power);
    Serial.println("W");
  }
  else if (message.startsWith("PC ")) {
    float powerConsumption = message.substring(3).toFloat();
    Serial.print("Power Consumption: ");
    Serial.print(powerConsumption);
    Serial.println("kWh");
  }
  else {
    Serial.println("Unrecognized message format");
  }

  Serial.println("-----------------------");

  // Update LCD display
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Msg: ");
  lcd.print(message.substring(0, 11)); // Show first 11 chars
  lcd.setCursor(0, 1);
  if (isNumeric(message)) {
    lcd.print("Bright: ");
    lcd.print(message);
    lcd.print("%");
  } else {
    lcd.print(message.substring(11, 20)); // Show next 9 chars if long
  }
}

//------------------------------------------------------------
// Heap-Memory check function
void checkMemory() {
  static unsigned long lastCheck = 0;
  static uint32_t lowestHeap = UINT32_MAX;
  static bool warningSent = false;
  static bool criticalSent = false;
  static uint8_t restartCount = 0;

  if (millis() - lastCheck > 30000) { // Check every 30 seconds
    uint32_t freeHeap = ESP.getFreeHeap();
    
    // 1. Track and report lowest heap observed
    if (freeHeap < lowestHeap) {
      lowestHeap = freeHeap;
      char msg[70];
      snprintf(msg, sizeof(msg), 
              "[MEM] New lowest: %u bytes (Current: %u)", 
              lowestHeap, freeHeap);
      publishDebug(msg);
    }

    // 2. Warning level (80KB)
    if (freeHeap < WARNING_HEAP_THRESHOLD) {
      if (!warningSent) {
        char warning[80];
        snprintf(warning, sizeof(warning), 
                "[MEM] WARNING: Heap %u bytes (Lowest: %u)", 
                freeHeap, lowestHeap);
        publishDebug(warning);
        warningSent = true;
        
        // Optional: Visual indicator
        lcd.setCursor(15, 0);
        lcd.print("!");
      }
    } else if (freeHeap > WARNING_HEAP_THRESHOLD + 10000) {
      warningSent = false;
      lcd.setCursor(15, 0);
      lcd.print(" "); // Clear warning indicator
    }

    // 3. Critical level (50KB)
    if (freeHeap < CRITICAL_HEAP_THRESHOLD) {
      if (!criticalSent) {
        char critical[80];
        snprintf(critical, sizeof(critical), 
                "[MEM] CRITICAL: Heap %u bytes", freeHeap);
        publishDebug(critical);
        criticalSent = true;
        
        // Optional: Disable non-essential features
        mqtt_client.loop(); // Flush pending messages
        ArduinoOTA.end(); // Disable OTA updates
      }
      
      // 4. Emergency restart (30KB)
      if (freeHeap < RESTART_HEAP_THRESHOLD) {
        if (++restartCount <= MAX_RESTARTS) {
          publishDebug("[MEM] EMERGENCY RESTART");
          delay(100); // Ensure message is sent
          ESP.restart();
        } else {
          publishDebug("[MEM] FATAL: Halting system");
          lcd.clear();
          lcd.print("MEMORY FAULT");
          while(1) { // Hardware reset required
            delay(1000);
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Blink LED
          }
        }
      }
    } else {
      criticalSent = false;
    }

    // 5. Fragmentation check (Optional)
    if (lowestHeap < (freeHeap * 0.7)) { // >30% difference
      char frag[90];
      snprintf(frag, sizeof(frag), 
              "[MEM] Fragmentation: Current %u vs Lowest %u", 
              freeHeap, lowestHeap);
      publishDebug(frag);
    }

    lastCheck = millis();
  }
}

//------------------------------------------------------------
// Error handler function
void handleError(int errorCode, const char* message) {
  Serial.printf("ERROR %d: %s\n", errorCode, message);

  publishDebug("[ERROR] Code " + String(errorCode) + ": " + String(message));

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ERROR:");
  lcd.print(errorCode);
  lcd.setCursor(0, 1);
  lcd.print(message);
}

//=====================================================================================
void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  // analogReadResolution(12);      // Set to ESP32's 12-bit ADC

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();

  // Configure pins
  pinMode(TriacTrigger, OUTPUT);
  digitalWrite(TriacTrigger, LOW);

  pinMode(zeroCrossingPin, INPUT_PULLUP);
  
  // Initialize the LED_BUILTIN pin as an output
  pinMode(LED_BUILTIN, OUTPUT);

  // Connect to network
  connectToWiFi();
  setupNTP();
  setupOTA();
  connectToMQTT();

  timeClient.update(); // Force initial update

  // Perform ACS712 calibration
  calibrateACS712();

  // Perform ZMPT10B calibration
  // calibrateZMPT10B();

  // Initialize timer for firing TRIAC
  timer = timerBegin(0, 80, true);  // Timer 0, 80 prescaler (1 tick = 1µs)
  timerAttachInterrupt(timer, &fireTriacISR, true);

  // Configure the timer to trigger periodically
  reportTimer = timerBegin(1, 80, true);  // Timer 1, prescaler 80 (1 tick = 1µs)
  timerAttachInterrupt(reportTimer, &reportISR, true);  // Attach the ISR
  timerAlarmWrite(reportTimer, timerInterval, true);  //interval in µs
  timerAlarmEnable(reportTimer);  // Enable the timer

  // Attach zero-crossing interrupt
  attachInterrupt(digitalPinToInterrupt(zeroCrossingPin), zeroCrossingISR, CHANGE);

  publishDebug("\nSystem Initialization Complete\n");
  delay(50);
}

//=====================================================================================
void loop() {
  // Handle network services
  ArduinoOTA.handle(); // Must be called frequently for OTA updates

  // Add this safety check
  if(WiFi.status() != WL_CONNECTED) {
    emergencyStop();
    connectToWiFi();
  }

  // Maintain MQTT connection
  if (!mqtt_client.connected()) {
    connectToMQTT();
  }
  mqtt_client.loop();  // Process MQTT messages

  // Report periodically
  if (reportFlag) {
    // Report data to MQTT
    measuredFrequency = reportFrequency();
    measuredVoltage   = reportVoltage();
    measuredCurrent   = reportCurrent();
    measuredPower     = measuredVoltage * (measuredCurrent * 0.001); // Convert mA to A
    measuredPowerConsumption = measuredPower / 3600000; // Convert to kWh for 1 hour
    measuredPowerConsumption = measuredPowerConsumption * 24; // Convert to kWh for 24 hours

    // Publish data to MQTT
    publishFrequency(measuredFrequency);
    publishVoltage(measuredVoltage);
    publishCurrent(measuredCurrent);
    publishPower(measuredPower);
    publishPowerConsumption(measuredPowerConsumption);

    reportFlag = false;
  }

  // Check memory usage
  checkMemory();
}


///////////////////////////////////////////////////////////////////////////////////////
//                                                                                   //
//   Author: Nariman Ziaie                                                           //
//   Date:   July 8, 2025                                                            //
//   Github: https://github.com/Nariman-Z/Dimmo                                      //
//                                                                                   //
///////////////////////////////////////////////////////////////////////////////////////