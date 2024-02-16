// Define motor controller pins for 'B' side
#define ENB 5 // Speed control for motor(s) on 'B' side
#define IN3 6 // Direction pin 1 for motor(s) on 'B' side
#define IN4 4 // Direction pin 2 for motor(s) on 'B' side

// Define the pin receiving the signal
#define SIGNAL_PIN 12

void setup() {
  // Initialize motor control pins as outputs
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Initialize the signal pin as an input
  pinMode(SIGNAL_PIN, INPUT);
}

void loop() {
  // Read the signal from the other Arduino
  int signalState = digitalRead(SIGNAL_PIN);

  if (signalState == HIGH) {
    // Signal is HIGH, start the motor(s) on 'B' side
    analogWrite(ENB, 200); // Set speed (0-255), 255 for full speed
    digitalWrite(IN3, LOW); // Set direction
    digitalWrite(IN4, HIGH);  // Set direction
  } else {
    // Signal is LOW, stop the motor(s)
    digitalWrite(ENB, 0); // Stop the motor(s) by setting speed to 0
    digitalWrite(IN3, HIGH); // Direction doesn't matter when stopped
    digitalWrite(IN4, HIGH); // Direction doesn't matter when stopped
  }

  // Add a short delay to reduce processing overhead
  delay(10);
}