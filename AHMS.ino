#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30100_PulseOximeter.h"
#include <DallasTemperature.h>
#include <OneWire.h>
#include <MPU6050.h>
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>
#include <BlynkSimpleShieldEsp8266.h>

// OLED Display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

// MAX30102 (Heart Rate and SpO₂ Sensor)
Adafruit_MAX30105 max30102;

// DS18B20 (Temperature Sensor)
OneWire oneWire(2);
DallasTemperature tempSensor(&oneWire);

// MPU6050 (Accelerometer/Gyroscope)
MPU6050 mpu;

// WiFi and Blynk Credentials (will be used with ESP8266 WiFi module)
char ssid[] = "Your_SSID";
char pass[] = "Your_PASSWORD";
char auth[] = "Your_Blynk_Auth_Token";

// GSM Module
SoftwareSerial gsmSerial(12, 13); // GSM TX, RX
#define GUARDIAN_NUMBER "+1234567890"
#define HOSPITAL_NUMBER "+0987654321"

// ECG Sensor
#define ECG_PIN A0
#define LEAD_OFF_PLUS_PIN 7
#define LEAD_OFF_MINUS_PIN 8

// Emergency Button
#define PANIC_BUTTON_PIN 4

// GPS Data
String latitude = "N/A";
String longitude = "N/A";

// DFPlayer Mini for voice alerts
SoftwareSerial mySoftwareSerial(5, 6); // RX, TX
DFRobotDFPlayerMini myDFPlayer;

// ESP8266 WiFi module
SoftwareSerial espSerial(10, 11); // RX, TX for ESP8266 module

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // OLED Initialization
  if (!display.begin(SSD1306_I2C_ADDRESS, 0x3C)) {
    Serial.println("OLED initialization failed.");
    for (;;);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Initialize MAX30102
  if (!max30102.begin()) {
    Serial.println("MAX30102 not detected.");
    for (;;);
  }
  max30102.setup();  // Setup the sensor
  
  // Initialize DS18B20
  tempSensor.begin();

  // Initialize MPU6050
  mpu.initialize();

  // Initialize ECG Sensor
  pinMode(LEAD_OFF_PLUS_PIN, INPUT);
  pinMode(LEAD_OFF_MINUS_PIN, INPUT);

  // Initialize Emergency Button
  pinMode(PANIC_BUTTON_PIN, INPUT_PULLUP);

  // Initialize GSM
  gsmSerial.begin(9600);

  // Initialize DFPlayer Mini
  mySoftwareSerial.begin(9600);
  if (!myDFPlayer.begin(mySoftwareSerial)) {
    Serial.println("DFPlayer Mini not detected.");
    for (;;);
  }
  myDFPlayer.volume(20); // Set volume to 20 (0-30)

  // Initialize ESP8266 (WiFi module)
  espSerial.begin(115200); // ESP8266 baud rate
  initializeWiFi();

  // Initialize Blynk (via WiFi)
  Blynk.begin(auth, espSerial);
}

void loop() {
  Blynk.run();
  monitorVitals();

  if (digitalRead(PANIC_BUTTON_PIN) == LOW) {
    handleEmergencyButton();
  }
}

void monitorVitals() {
  float temperature = readTemperature();
  float heartRate = readHeartRate();
  float bloodPressure = calculateBloodPressure(heartRate);
  float spo2 = readSpO2();
  int ecgValue = readECG();

  displayData(temperature, heartRate, bloodPressure, spo2, ecgValue);
  sendToBlynk(temperature, heartRate, bloodPressure, spo2, ecgValue);
  
  if (isAbnormal(temperature, heartRate, bloodPressure, spo2)) {
    fetchGPSLocation();
    alertGuardianAndHospital(temperature, heartRate, bloodPressure, spo2, ecgValue);
  }
}

float readTemperature() {
  tempSensor.requestTemperatures();
  return tempSensor.getTempCByIndex(0);
}

float readHeartRate() {
  long irValue = max30102.getIR();  // Get the IR value from MAX30102
  if (irValue < 50000) {
    return 0; // No finger detected
  }
  // Heart rate calculation based on the IR value. Placeholder logic.
  return random(60, 100); 
}

float calculateBloodPressure(float heartRate) {
  return 120 + (heartRate - 60) * 0.5; // Placeholder for calculation
}

float readSpO2() {
  int16_t redValue = max30102.getRed();
  int16_t irValue = max30102.getIR();
  if (irValue < 50000) {
    return 0; // No finger detected
  }
  // SpO2 calculation logic. Placeholder logic.
  return random(95, 100); 
}

int readECG() {
  if (digitalRead(LEAD_OFF_PLUS_PIN) == HIGH || digitalRead(LEAD_OFF_MINUS_PIN) == HIGH) {
    return 0; // Lead off detected
  }
  return analogRead(ECG_PIN); // Reading ECG data from the analog pin
}

void displayData(float temp, float hr, float bp, float spo2, int ecg) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Temp: ");
  display.print(temp);
  display.println(" C");

  display.print("Heart Rate: ");
  display.print(hr);
  display.println(" BPM");

  display.print("Blood Pressure: ");
  display.print(bp);
  display.println(" mmHg");

  display.print("SpO2: ");
  display.print(spo2);
  display.println(" %");

  display.print("ECG: ");
  display.print(ecg);
  display.println(" mV");

  display.display();
}

void sendToBlynk(float temp, float hr, float bp, float spo2, int ecg) {
  Blynk.virtualWrite(V1, temp);
  Blynk.virtualWrite(V2, hr);
  Blynk.virtualWrite(V3, bp);
  Blynk.virtualWrite(V4, spo2);
  Blynk.virtualWrite(V5, ecg);
}

bool isAbnormal(float temp, float hr, float bp, float spo2) {
  return temp > 37.5 || hr < 60 || hr > 100 || bp > 140 || spo2 < 95;
}

void fetchGPSLocation() {
  gsmSerial.println("AT+CGPSINFO");
  delay(2000);
  while (gsmSerial.available()) {
    String gpsData = gsmSerial.readString();
    if (gpsData.indexOf("+CGPSINFO") != -1) {
      int comma1 = gpsData.indexOf(",");
      latitude = gpsData.substring(10, comma1);
      int comma2 = gpsData.indexOf(",", comma1 + 1);
      longitude = gpsData.substring(comma1 + 1, comma2);
    }
  }
}

void alertGuardianAndHospital(float temp, float hr, float bp, float spo2, int ecg) {
  String message = "EMERGENCY ALERT\n";
  message += "Vitals:\n";
  message += "Temp: " + String(temp) + "C\n";
  message += "HR: " + String(hr) + " BPM\n";
  message += "BP: " + String(bp) + " mmHg\n";
  message += "SpO2: " + String(spo2) + "%\n";
  message += "ECG: " + String(ecg) + "\n";
  message += "Location: https://maps.google.com/?q=" + latitude + "," + longitude;

  sendSMS(GUARDIAN_NUMBER, message);
  sendSMS(HOSPITAL_NUMBER, message);
}

void handleEmergencyButton() {
  fetchGPSLocation();
  alertGuardianAndHospital(0, 0, 0, 0, 0); // Send minimal info
  myDFPlayer.play(1); // Play emergency alert sound
}

void sendSMS(String number, String message) {
  gsmSerial.println("AT+CMGF=1");
  delay(100);
  gsmSerial.println("AT+CMGS=\"" + number + "\"");
  delay(100);
  gsmSerial.println(message);
  delay(100);
  gsmSerial.write(26); // ASCII for CTRL+Z
}

void initializeWiFi() {
  espSerial.println("AT+RST"); // Reset the module
  delay(1000);
  espSerial.println("AT+CWMODE=1"); // Set WiFi mode to Station
  delay(1000);
  espSerial.println("AT+CWJAP=\"" + String(ssid) + "\",\"" + String(pass) + "\""); // Connect to WiFi
  delay(5000);
}
