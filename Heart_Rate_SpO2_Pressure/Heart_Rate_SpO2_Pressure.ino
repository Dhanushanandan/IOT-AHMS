#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30100_PulseOximeter.h"

// OLED Display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// MAX30100 sensor settings
PulseOximeter pox;
uint32_t tsLastReport = 0;

#define REPORTING_PERIOD_MS 1000 // Report every 1 second

bool fingerDetected = false;

// Blood pressure calibration constants
#define BASELINE_SBP 120  // Baseline Systolic BP
#define BASELINE_DBP 80   // Baseline Diastolic BP
#define CALIBRATION_CONSTANT_SBP 0.5
#define CALIBRATION_CONSTANT_DBP 0.3

#define MAX_READINGS 20  // Store the last 20 heart rate readings
int heartRateBuffer[MAX_READINGS] = {0}; // Circular buffer for heart rates
int bufferIndex = 0;                     // Current index in the buffer
int bufferCount = 0;                     // Count of readings in the buffer

void onBeatDetected() {
    fingerDetected = true; // Finger detected on beat
}

void resetForNextReading() {
    fingerDetected = false;
    bufferIndex = 0;
    bufferCount = 0;

    // Clear buffer
    memset(heartRateBuffer, 0, sizeof(heartRateBuffer));

    // Reinitialize the sensor
    if (!pox.begin()) {
        Serial.println("FAILED to reinitialize MAX30100 sensor");
        return;
    }
    pox.setIRLedCurrent(MAX30100_LED_CURR_24MA);

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Place your finger");
    display.println("to start reading.");
    display.display();
    Serial.println("Place your finger to start reading.");
}

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
    float averageSpO2 = pox.getSpO2();  // Use the latest SpO2 reading as average

    // Display average readings on OLED
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Avg Heart Rate: ");
    display.print(averageHeartRate); // Display as integer
    display.println(" bpm");
    display.print("Avg SpO2: ");
    display.print(averageSpO2, 1); // Display one decimal point for SpO2
    display.println(" %");
    display.print("Avg Sys BP: ");
    display.print(averageSystolicBP, 1);
    display.println(" mmHg");
    display.print("Avg Dia BP: ");
    display.print(averageDiastolicBP, 1);
    display.println(" mmHg");
    display.display();

    // Print average readings to Serial Monitor
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
}

void setup() {
    Serial.begin(115200);

    // Initialize OLED display
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println("SSD1306 allocation failed");
        for (;;);
    }
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    // Initialize PulseOximeter sensor
    if (!pox.begin()) {
        Serial.println("FAILED to initialize MAX30100 sensor");
        display.println("FAILED to initialize MAX30100 sensor");
        display.display();
        for (;;);
    } else {
        Serial.println("MAX30100 sensor initialized successfully");
        display.println("MAX30100 sensor initialized successfully");
        display.display();
    }

    pox.setIRLedCurrent(MAX30100_LED_CURR_24MA);
    pox.setOnBeatDetectedCallback(onBeatDetected);

    // Initial prompt to place finger
    resetForNextReading();
}

void loop() {
    static unsigned long startTime = 0; // Start time for the 10-second wait
    static bool waitingForReadings = false; // State to track if the system is waiting for readings
    static bool displayingResult = false;  // State to track if the result is being displayed

    pox.update();

    if (fingerDetected && !waitingForReadings && !displayingResult) {
        // Start the 10-second reading collection
        waitingForReadings = true;
        startTime = millis();
        Serial.println("Collecting readings. Please keep your finger steady.");
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("Collecting readings");
        display.println("Please keep your finger steady.");
        display.display();
    }

    if (waitingForReadings) {
        if (millis() - startTime <= 10000) { // Collect readings for 10 seconds
            float heartRate = pox.getHeartRate();
            float spo2 = pox.getSpO2();

            // Validate and store heart rate readings
            if (heartRate > 40 && heartRate < 200 && spo2 > 70 && spo2 < 100) {
                heartRateBuffer[bufferIndex] = round(heartRate);
                bufferIndex = (bufferIndex + 1) % MAX_READINGS;
                if (bufferCount < MAX_READINGS) bufferCount++;
            }
        } else {
            // 10 seconds have passed, calculate and display the average
            waitingForReadings = false;
            displayingResult = true;
            calculateAndDisplayAverage();
        }
    }

    if (displayingResult) {
        delay(2000); // Reduced delay after displaying results
        displayingResult = false;

        Serial.println("Remove your finger and place again to start a new reading.");
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("Remove finger");
        display.println("Ready for next reading.");
        display.display();
        delay(2000); // Short delay to show the "Remove finger" message
        resetForNextReading(); // Prepare for the next session
    }

    // Report every 1 second
    if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
        tsLastReport = millis();
    }
}
