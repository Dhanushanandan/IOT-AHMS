int sensorPin = A0;      // Pin to read the sensor data (e.g., analog input from a pulse sensor)
int threshold = 620;     // Threshold for detecting a heartbeat (adjust as needed)
long previousMillis = 0; // Store last time a heartbeat was detected
int BPM = 0;             // Variable to store BPM
int beatCount = 0;       // Count the number of detected beats
int beatOldTime = 0;     // Store the last time a beat was detected
boolean belowThreshold = true;

void setup() {
  // Start serial communication at 9600 baudrate
  Serial.begin(9600);
}

void loop() {
  // Read sensor data
  int sensorValue = analogRead(sensorPin);
  
  // Detect a beat when the signal goes above the threshold
  if (sensorValue > threshold && belowThreshold) {
    // Calculate the time difference between this beat and the last one
    long currentMillis = millis();
    int beatTime = currentMillis - beatOldTime;
    
    // Calculate BPM (beats per minute)
    if (beatTime > 0) {
      BPM = 60000 / beatTime;  // Convert time between beats to BPM
      beatOldTime = currentMillis;  // Update last beat time
    }

    // Send the BPM value to the Processing sketch
    Serial.println(BPM);
    
    // Increment the beat count and set belowThreshold to false
    beatCount++;
    belowThreshold = false;
  }

  // If the sensor value is below the threshold, reset the flag
  if (sensorValue < threshold) {
    belowThreshold = true;
  }
  
  // Optional: Send a '!' character for lead off detection (like in your Processing code)
  // Uncomment the next line if you want to use lead-off detection
  // if (sensorValue == 512) {
  //   Serial.println("!"); // Send a lead off detection signal
  // }
  
  delay(10);  // Small delay to smooth out readings
}
