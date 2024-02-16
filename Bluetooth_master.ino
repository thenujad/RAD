#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Define HC-05 pins
const int bluetoothTx = 3; // Connect TX of HC-05 to pin 2 of Arduino
const int bluetoothRx = 2; // Connect RX of HC-05 to pin 3 of Arduino

SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

const int trigPin1 = 4; // Ultrasonic sensor 1 trigger pin
const int echoPin1 = 5; // Ultrasonic sensor 1 echo pin
const int trigPin2 = 6; // Ultrasonic sensor 2 trigger pin
const int echoPin2 = 7; // Ultrasonic sensor 2 echo pin
const int trigPin3 = 8; // Ultrasonic sensor 3 trigger pin
const int echoPin3 = 9; // Ultrasonic sensor 3 echo pin
//const int trigPin4 = 8; // Ultrasonic sensor 2 trigger pin
//const int echoPin4 = 9; // Ultrasonic sensor 2 echo pin
//const int trigPin5 = 10; // Ultrasonic sensor 3 trigger pin
//const int echoPin5 = 11; // Ultrasonic sensor 3 echo pin

// Define values to be sent via Bluetooth
int setValue1 = 1;
int setValue2 = 127;


void setup() {
  Serial.begin(9600);
  bluetooth.begin(9600); // Bluetooth module baud rate
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  //pinMode(trigPin4, OUTPUT);
  //pinMode(echoPin4, INPUT);
  //pinMode(trigPin5, OUTPUT);
  //pinMode(echoPin5, INPUT);

  // initialize the LCD
  lcd.init();
  lcd.clear();

  // Turn on the backlight
  lcd.backlight();
}

void loop() {
  // Measure distance for sensor 1
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  float duration1 = pulseIn(echoPin1, HIGH);
  float distance1 = duration1 * 0.0354 / 2;

  // Measure distance for sensor 2
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  float duration2 = pulseIn(echoPin2, HIGH);
  float distance2 = duration2 * 0.0354 / 2;

  // Calculate final values
  float finalValue1 = distance1;
  float finalValue2 = distance2;

  // Calculate width
  float width;
  if (finalValue1 <= 0 || finalValue2 <= 0) {
    // Both sensors report default distance, no obstacle detected
    width = 15;
  } else {
    // At least one sensor reports a non-default distance, calculate width
    width = 15 - (finalValue1 + finalValue2);
  }

  // Measure distance for sensor 3
  digitalWrite(trigPin3, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin3, LOW);
  float duration3 = pulseIn(echoPin3, HIGH);
  float distance3 = duration3 * 0.0354 / 2;

  //calculating height
  float height;
  height = 13 - distance3;

  /*// Measure distance for sensor 4
  digitalWrite(trigPin4, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin4, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin4, LOW);
  float duration4 = pulseIn(echoPin4, HIGH);
  float distance4 = duration4 * 0.0354 / 2;

  // Measure distance for sensor 5
  digitalWrite(trigPin5, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin5, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin5, LOW);
  float duration5 = pulseIn(echoPin5, HIGH);
  float distance5 = duration5 * 0.0354 / 2;

  // Calculate final values
  float finalValue4 = distance4;
  float finalValue5 = distance5;

  // Calculate length
  float length;
  if (finalValue4 <= 0 || finalValue5 <= 0) {
    // Both sensors report default distance, no obstacle detected
    length = 15;
  } else {
    // At least one sensor reports a non-default distance, calculate width
    length = 15 - finalValue4 - finalValue5;
  }*/

  // Output results to Serial
  Serial.print("Final Value of Ultrasonic Sensor 1: ");
  Serial.println(finalValue1);
  Serial.print("Final Value of Ultrasonic Sensor 2: ");
  Serial.println(finalValue2);
  Serial.print("Width: ");
  Serial.print(width);
  Serial.println(" cm");
  Serial.print("Final Value of Ultrasonic Sensor 3: ");
  Serial.println(distance2);
  Serial.print("Height: ");
  Serial.print(height);
  Serial.println(" cm");
  /*Serial.print("Final Value of Ultrasonic Sensor 4: ");
  Serial.println(finalValue4);
  Serial.print("Final Value of Ultrasonic Sensor 5: ");
  Serial.println(finalValue5);*/
  

  float volume = width*height*height;

  Serial.print("Volume: ");
  Serial.print(volume);
  Serial.println(" cm3");

  // Output width to LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Volume:");
  lcd.print(volume);
  lcd.print("cm3");
  /*lcd.println("Height:");
  lcd.print(height);
  lcd.print("cm");*/

  // Check if the volume falls within a certain range and send the appropriate value via Bluetooth
  if (volume >= 50 && volume <= 150) {
    bluetooth.print("setValue1: ");
    bluetooth.println(setValue1);
  } else if (volume >= 156 && volume <= 400) {
    bluetooth.print("setValue2: ");
    bluetooth.println(setValue2);
  } else {
    // Volume not within specified ranges
    bluetooth.println("Volume out of range");
  }
  
  delay(1000); // Delay before next measurement
}