#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30100_PulseOximeter.h"
#include <MPU6050.h>
#include "DFRobotDFPlayerMini.h"
#include <OneWire.h>
#include <DallasTemperature.h>



// OLED Display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// MAX30100 sensor settings
PulseOximeter pox;
#define REPORTING_PERIOD_MS 1000  // Report every 1 second
uint32_t tsLastReport = 0;

// Blood pressure calibration constants
#define BASELINE_SBP 120
#define BASELINE_DBP 80
#define CALIBRATION_CONSTANT_SBP 0.3
#define CALIBRATION_CONSTANT_DBP 0.1
#define MAX_READINGS 20
int heartRateBuffer[MAX_READINGS] = { 0 };
int bufferIndex = 0;
int bufferCount = 0;

// MPU6050 Fall Detection settings
MPU6050 mpu;
#define FALL_THRESHOLD 200
#define STABILIZATION_THRESHOLD 20
#define STABILIZATION_TIME 1000

bool fallDetected = false;
unsigned long stabilizationStartTime = 0;
bool fingerDetected = false;


#define FPSerial Serial1

DFRobotDFPlayerMini myDFPlayer;

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2
#define TEMPERATURE_PRECISION 9  // Lower resolution

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

int numberOfDevices;  // Number of temperature devices found

DeviceAddress tempDeviceAddress;  // We'll use this variable to store a found device address










// Callback for MAX30100 on beat detection
void onBeatDetected() {
  fingerDetected = true;
}

// Function to reset for a new blood pressure reading
void resetForNextReading() {
  fingerDetected = false;
  bufferIndex = 0;
  bufferCount = 0;

  memset(heartRateBuffer, 0, sizeof(heartRateBuffer));

  if (!pox.begin()) {
    Serial.println("FAILED to reinitialize MAX30100 sensor");
    return;
  }
  pox.setIRLedCurrent(MAX30100_LED_CURR_24MA);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Place your finger");
  display.println(" & other sensors");
  display.println("to start reading.");
  display.display();
  Serial.println("Place your finger & other sensors to start reading.");
}

// Function to calculate and display average blood pressure
void calculateAndDisplayAverage() {
  if (bufferCount == 0) {
    Serial.println("No valid readings captured.");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("No valid readings.");
    display.println("Try again.");
    display.display();
    return;
  }

  int totalHeartRate = 0;
  for (int i = 0; i < bufferCount; i++) {
    totalHeartRate += heartRateBuffer[i];
  }

  int averageHeartRate = totalHeartRate / bufferCount;
  float averageSystolicBP = BASELINE_SBP + CALIBRATION_CONSTANT_SBP * averageHeartRate;
  float averageDiastolicBP = BASELINE_DBP + CALIBRATION_CONSTANT_DBP * averageHeartRate;
  float averageSpO2 = pox.getSpO2();

  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Avg Heart Rate: ");
  display.print(averageHeartRate);
  display.println(" bpm");
  display.print("Avg SpO2: ");
  display.print(averageSpO2, 1);
  display.println(" %");
  display.print("Avg Sys BP: ");
  display.print(averageSystolicBP, 1);
  display.println(" mmHg");
  display.print("Avg Dia BP: ");
  display.print(averageDiastolicBP, 1);
  display.println(" mmHg");
  display.display();

  Serial.println("Final Average Readings:");
  Serial.print("Heart Rate: ");
  Serial.print(averageHeartRate);
  Serial.println(" bpm");
  Serial.print("SpO2: ");
  Serial.print(averageSpO2, 1);
  Serial.println(" %");
  Serial.print("Systolic BP: ");
  Serial.print(averageSystolicBP, 1);
  Serial.println(" mmHg");
  Serial.print("Diastolic BP: ");
  Serial.print(averageDiastolicBP, 1);
  Serial.println(" mmHg");

  delay(2000);

  // Post-reading instruction
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Remove your finger.");
  display.println("Place it again for");
  display.println("a new reading.");
  display.display();


  Serial.println("Remove your finger. Place it again for a new reading");
  
      printTemperature();
}

// Initialize MPU6050
void initFallDetection() {
  Serial.println("Initialize MPU6050");
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  mpu.calibrateGyro();
  mpu.setThreshold(3);
  Serial.println("MPU6050 initialized successfully");
}

// Function to check for falls
void checkForFalls() {
  Vector normGyro = mpu.readNormalizeGyro();
  float totalGyro = sqrt(normGyro.XAxis * normGyro.XAxis + normGyro.YAxis * normGyro.YAxis + normGyro.ZAxis * normGyro.ZAxis);

  if (!fallDetected && totalGyro > FALL_THRESHOLD) {
    fallDetected = true;
    Serial.println("ALERT: Fall detected!");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Fall Detected!");
    display.display();
    myDFPlayer.play(1);  // Play the first track (0001.mp3)
    stabilizationStartTime = millis();
  }

  if (fallDetected) {
    if (totalGyro < STABILIZATION_THRESHOLD) {
      if (millis() - stabilizationStartTime >= STABILIZATION_TIME) {
        fallDetected = false;
        Serial.println("Stabilization complete. Fall state cleared.");

        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("Stabilized.");
        display.println("Place your finger");
        display.println("to start reading.");
        display.display();
        myDFPlayer.stop();  // Stop the music
        resetForNextReading();
      }
    } else {
      stabilizationStartTime = millis();
    }
  }
}




void ECGcalculation() {
  if (digitalRead(10) == 1 || digitalRead(11) == 1) {
    Serial.println(0);
  } else {
    Serial.println(analogRead(A0));
  }
  delay(10);
}


// function to print the temperature for a device
void printTemperature() {
  // call sensors.requestTemperatures() to issue a global temperature
  // request to all devices on the bus
  Serial.print("Requesting temperatures Hold Sensor...");
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Requesting temperatures");
  display.print(" Hold Sensor...");
  display.display();
  delay(10000);
  sensors.requestTemperatures();  // Send the command to get temperatures
  Serial.println("DONE");



  float tempC = sensors.getTempC(tempDeviceAddress);
  if (tempC == DEVICE_DISCONNECTED_C) {
    Serial.println("Error: Could not read temperature data");
    return;
  }

  Serial.print("Temp C: ");
  Serial.print(tempC);
  Serial.print(" Temp F: ");
  Serial.println(DallasTemperature::toFahrenheit(tempC));  // Converts tempC to Fahrenheit

  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Temp C: ");
  display.print(tempC);
  display.print("Temp F: ");
  display.print(DallasTemperature::toFahrenheit(tempC), 1);
  display.display();
  delay(2000);
}


// function to print a device address
void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

void setup() {

  FPSerial.begin(9600);  // Initialize the serial communication with DFPlayer Mini
  Serial.begin(115200);
  pinMode(10, INPUT);  //ECG LO+
  pinMode(11, INPUT);  //ECG LO-

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 allocation failed");
    for (;;)
      ;
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  if (!pox.begin()) {
    Serial.println("FAILED to initialize MAX30100 sensor");
    display.println("FAILED to initialize MAX30100 sensor");
    display.display();
    for (;;)
      ;
  }
  pox.setIRLedCurrent(MAX30100_LED_CURR_24MA);
  pox.setOnBeatDetectedCallback(onBeatDetected);

  // Initialize DFPlayer Mini
  if (!myDFPlayer.begin(FPSerial, /*isACK = */ true, /*doReset = */ true)) {
    Serial.println(F("Unable to begin! Please check the connection and SD card."));
    display.println("Unable to begin! Please ");
    display.println("check the connection and SD card.");
    display.display();
    while (true) { delay(0); }  // Halt the program if DFPlayer is not detected
  }

  Serial.println(F("DFPlayer Mini initialized."));
  myDFPlayer.volume(10);  // Set the volume (0 to 30)

  // Start up the library
  sensors.begin();

  // Grab a count of devices on the wire
  numberOfDevices = sensors.getDeviceCount();

  // locate devices on the bus
  Serial.print("Locating devices...");

  Serial.print("Found ");
  Serial.print(numberOfDevices, DEC);
  Serial.println(" devices.");

  // report parasite power requirements
  Serial.print("Parasite power is: ");
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");

  // Loop through each device, print out address
  for (int i = 0; i < numberOfDevices; i++) {
    // Search the wire for address
    if (sensors.getAddress(tempDeviceAddress, i)) {
      Serial.print("Found device ");
      Serial.print(i, DEC);
      Serial.print(" with address: ");
      printAddress(tempDeviceAddress);
      Serial.println();

      Serial.print("Setting resolution to ");
      Serial.println(TEMPERATURE_PRECISION, DEC);

      // set the resolution to TEMPERATURE_PRECISION bit (Each Dallas/Maxim device is capable of several different resolutions)
      sensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);

      Serial.print("Resolution actually set to: ");
      Serial.print(sensors.getResolution(tempDeviceAddress), DEC);
      Serial.println();
    } else {
      Serial.print("Found ghost device at ");
      Serial.print(i, DEC);
      Serial.print(" but could not detect address. Check power and cabling");
    }
  }

  initFallDetection();
  resetForNextReading();
}


void loop() {
  pox.update();
  checkForFalls();

  static unsigned long startTime = 0;
  static bool waitingForReadings = false;

  if (fingerDetected && !waitingForReadings) {
    waitingForReadings = true;
    startTime = millis();
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Collecting readings...");
    display.println("Keep steady.");
    display.display();

    Serial.println("Collecting Readings.....Keep steady.");
  }

  if (waitingForReadings) {
    if (millis() - startTime <= 10000) {  // Collect readings for 10 seconds
      float heartRate = pox.getHeartRate();
      float spo2 = pox.getSpO2();

      if (heartRate > 40 && heartRate < 200 && spo2 > 70 && spo2 < 100) {
        heartRateBuffer[bufferIndex] = round(heartRate);
        bufferIndex = (bufferIndex + 1) % MAX_READINGS;
        if (bufferCount < MAX_READINGS) bufferCount++;
      }
    } else {
      waitingForReadings = false;
      calculateAndDisplayAverage();
      resetForNextReading();
    }
  }

  ECGcalculation();
}
