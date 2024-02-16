#include <SoftwareSerial.h>

// Define HC-05 pins
const int bluetoothTx = 2; // Connect TX of HC-05 to pin 2 of Arduino
const int bluetoothRx = 3; // Connect RX of HC-05 to pin 3 of Arduino

SoftwareSerial bluetooth(bluetoothRx, bluetoothTx); // Swapped the pins for SoftwareSerial

void setup() {
  Serial.begin(9600); // Serial monitor baud rate
  bluetooth.begin(9600); // Bluetooth module baud rate
}

void loop() {
  if (bluetooth.available()) {
    // Read the received byte:
    int receivedValue = bluetooth.read();
    bluetooth.println(receivedValue); // Echo back the received value
    Serial.println(receivedValue);
    if (receivedValue >= 0 && receivedValue <= 100) {
      Serial.println("Slot 1");
     
    } else if (receivedValue >= 101 && receivedValue <= 255) {
      Serial.println("Slot 2");```
      `
    }
    else 
    Serial.println("Slot 3");
  }
}