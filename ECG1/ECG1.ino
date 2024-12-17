int lastReading = 0;   // To store the previous reading
int LO_minus_pin = 11;  // Lead-off detection pin LO-
int LO_plus_pin = 10;   // Lead-off detection pin LO+

void setup() {
  Serial.begin(9600);  // Initialize serial communication
  pinMode(LO_minus_pin, INPUT);  // Set pin 11 (LO-) as input
  pinMode(LO_plus_pin, INPUT);   // Set pin 10 (LO+) as input
}

void loop() {
  // Check the LO- and LO+ pins multiple times with a delay to avoid false detection
  int loMinusState = digitalRead(LO_minus_pin);
  int loPlusState = digitalRead(LO_plus_pin);
  
  // Allow a small delay to stabilize
  delay(50);

  // Recheck the LO- and LO+ states
  loMinusState = digitalRead(LO_minus_pin);
  loPlusState = digitalRead(LO_plus_pin);

  if (loMinusState == LOW || loPlusState == LOW) {
    // If either of LO- or LO+ is LOW, it's a lead-off condition (electrodes not connected)
    Serial.println(0);  // Output 0 when disconnected
  } else {
    int ecgValue = analogRead(A0);  // Read ECG data from pin A0

    // Simple filtering to avoid sudden spikes
    if (abs(ecgValue - lastReading) > 5) {
      Serial.println(ecgValue);  // Output ECG data if it changes significantly
    }

    lastReading = ecgValue;  // Update the last reading
  }

  delay(10);  // Adjust delay for sampling rate
}
