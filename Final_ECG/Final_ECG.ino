// Pins
const int ecgPin = A0; // Connect to the OUT pin of AD8232
const int loPlusPin = 10; // Connect to LO+ pin of AD8232 (optional)
const int loMinusPin = 11; // Connect to LO- pin of AD8232 (optional)

void setup() {
  Serial.begin(115200); // Initialize serial communication
  pinMode(loPlusPin, INPUT); // Configure LO+ as input
  pinMode(loMinusPin, INPUT); // Configure LO- as input
}

void loop() {
  int ecgValue = analogRead(ecgPin); // Read the ECG signal from A0
  
  // Lead-off detection
  int loPlusStatus = digitalRead(loPlusPin);
  int loMinusStatus = digitalRead(loMinusPin);

  if (loPlusStatus == 1 || loMinusStatus == 1) {
    Serial.println("Lead off detected!");
  } else {
    // Output the ECG value
    Serial.println(ecgValue);
  }
  
  delay(10); // Small delay for smoother serial output
}
