#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver myServo = Adafruit_PWMServoDriver();

//------------*ARM DEFINITIONS*-----------------
#define SHOULDER 12
#define VERTICAL 14
#define FORWARD 13
#define GRIP 15

#define GRIP_OPEN 300
#define GRIP_CLOSE 150
#define GRIP_CLOSE_LARGE 200
#define SHOULDER_LEFT 645
#define SHOULDER_RIGHT 150
#define SHOULDER_CENTER 400
#define FORWARD_MAX 500
#define FORWARD_MIN 350
#define FORWARD_CENTER 330
#define VERTICAL_MAX 550
#define VERTICAL_CENTER 400
#define VERTICAL_MIN 150

//------------*IR DEFINITIONS*-----------------
#define LIR 4
#define RIR 2
#define CIR 3

#define LM1 7
#define LM2 8
#define LMS 9

#define RM1 12
#define RM2 13
#define RMS 11

#define THRESHOLD 500

int carSpeed = 120;

int stopCount = 0;

bool rightIR;
bool centerIR;
bool leftIR;
bool pLeft = false;
bool pRight = false;

void setup() {
  Serial.begin(9600);

  //Setting up IR pinmodes
  pinMode(LIR, INPUT);
  pinMode(RIR, INPUT);
  pinMode(CIR, INPUT);

  pinMode(LM1, OUTPUT);
  pinMode(LM2, OUTPUT);
  pinMode(LMS, OUTPUT);

  pinMode(RM1, OUTPUT);
  pinMode(RM2, OUTPUT);
  pinMode(RMS, OUTPUT);

  analogWrite(LMS, carSpeed);
  analogWrite(RMS, carSpeed);

  //Setting up Arm
  myServo.begin();
  myServo.setPWMFreq(60);
}

void loop() {

  // readIrInputs();
  readIrInputs();
  moveCar();
  // pickUpObject();

}

//------------------------------GENERAL METHODS-------------------------
void getBluetoothSignal() {
}
void test() {
  openArm();
  delay(2000);
  closeArm();
  delay(2000);
  myServo.setPWM(FORWARD, 0, FORWARD_MIN);
  delay(2000);
  moveShoulderToLeft();
  delay(2000);
  myServo.setPWM(VERTICAL, 0, VERTICAL_MAX);
  delay(2000);
}
//------------------------------CAR METHODS-------------------------

void readIrInputs() {
  // rightIR = analogRead(RIR) >= THRESHOLD;
  // centerIR = analogRead(CIR) >= THRESHOLD;
  // leftIR = analogRead(LIR) >= THRESHOLD;

  rightIR = digitalRead(RIR);
  centerIR = digitalRead(CIR);
  leftIR = digitalRead(LIR);

  Serial.print("Right : ");
  Serial.println(rightIR);

  Serial.print("Center : ");
  Serial.println(centerIR);

  Serial.print("Left : ");
  Serial.println(leftIR);
}

void moveCar() {
  
  /If black line detected corresponding IR turns LOW/
  if (leftIR == LOW && rightIR == HIGH) {
    // Only the left sensor detects the line, turn left
    pRight = true;
    pLeft = false;
    turnRightCar();
  } else if (leftIR == HIGH && rightIR == LOW) {
    // Only the right sensor detects the line, turn right
    pLeft=true;
    pRight = false;
    turnLeftCar();
  } else if (centerIR == LOW && leftIR == HIGH && rightIR == HIGH) {
    // Center sensor detect the line, go forward
    pRight=false;
    pLeft = false;
    goForwardCar();
  } else if (leftIR == HIGH && centerIR == HIGH && rightIR == HIGH) {
    //No Line detected
    if(pRight && !pLeft) 
      while(digitalRead(CIR)==HIGH)
        turnRightCar();
    if(pLeft && !pRight) 
      while(digitalRead(CIR)==LOW)
        turnLeftCar();
    if(!pLeft && !pRight || pLeft && pRight) goForwardCarBit();
    // /stopCar();

  } else if (leftIR == LOW && centerIR == LOW && rightIR == LOW) {
    // No Line detected
    stopCar();
  } else {
    // No sensor detects the line, stop
    stopCar();
  }
}

void goForwardCar() {
  Serial.println("Going Forward...");
  digitalWrite(RM1, HIGH);
  digitalWrite(RM2, LOW);
  digitalWrite(LM1, HIGH);
  digitalWrite(LM2, LOW);
}

void goForwardCarBit() {
  Serial.println("Going Forward...");
  digitalWrite(RM1, HIGH);
  digitalWrite(RM2, LOW);
  digitalWrite(LM1, HIGH);
  digitalWrite(LM2, LOW);
  delay(50);
}

void turnLeftCar() {
 
   
    digitalWrite(RM1, HIGH);
    digitalWrite(RM2, LOW);
    digitalWrite(LM1, LOW);
    digitalWrite(LM2, HIGH);
   
}

void turnRightCar() {

    digitalWrite(LM1, HIGH);
    digitalWrite(LM2, LOW);
    digitalWrite(RM1, LOW);
    digitalWrite(RM2, HIGH);
  
}

void stopCar() {
  Serial.println("Stopping..");
  digitalWrite(RM1, LOW);
  digitalWrite(RM2, LOW);
  digitalWrite(LM1, LOW);
  digitalWrite(LM2, LOW);
}

//-------------------------------ARM METHODS------------------------
void pickUpObject() {
  moveShoulderToLeft();
  delay(500);
  moveArmDown();
  delay(1000);
  moveArmForward();
  delay(1000);
  openArm();
  delay(1000);
  closeArm();
  delay(200);
  moveArmBackward();
  moveArmUp();
  delay(200);
  moveShoulderToCenterFromLeft();
  moveShoulderToRight();
  delay(500);
  moveArmDown();
  delay(1000);
  moveArmForward();
  delay(500);
  moveArmUp();
  delay(200);
  openArm();
  delay(1000);
  closeArm();
  delay(500);
  moveArmBackward();
  delay(1000);
  moveShoulderToCenterFromRight();
}

void dropObject() {
  moveShoulderToLeft();
  delay(500);
  moveArmDown();
  delay(1000);
}

void initializeArm() {
  myServo.setPWM(GRIP, 0, GRIP_CLOSE);
  myServo.setPWM(SHOULDER, 0, SHOULDER_CENTER);
  myServo.setPWM(VERTICAL, 0, VERTICAL_MIN);
  myServo.setPWM(FORWARD, 0, FORWARD_MAX);
}

void moveShoulderToCenterFromLeft() {
  for (uint16_t pulselen = SHOULDER_LEFT; pulselen > SHOULDER_CENTER; pulselen--) {
    myServo.setPWM(SHOULDER, 0, pulselen);
    delay(20);
  }
}

void moveShoulderToCenterFromRight() {
  for (uint16_t pulselen = SHOULDER_RIGHT; pulselen < SHOULDER_CENTER; pulselen++) {
    myServo.setPWM(SHOULDER, 0, pulselen);
    delay(20);
  }
}

void moveShoulderToRight() {
  for (uint16_t pulselen = SHOULDER_CENTER; pulselen > SHOULDER_RIGHT; pulselen--) {
    myServo.setPWM(SHOULDER, 0, pulselen);
    delay(20);
  }
}

void moveShoulderToLeft() {
  for (uint16_t pulselen = SHOULDER_CENTER; pulselen < SHOULDER_LEFT; pulselen++) {
    myServo.setPWM(SHOULDER, 0, pulselen);
    delay(20);
  }
}

void openArm() {
  myServo.setPWM(GRIP, 0, GRIP_OPEN);
}

void closeArm() {
  for (int i = GRIP_OPEN; i > GRIP_CLOSE; i--) {
    myServo.setPWM(GRIP, 0, i);
    delay(20);
  }
}

void closeArmNotPressure() {
  for (int i = GRIP_OPEN; i > 200; i--) {
    myServo.setPWM(GRIP, 0, i);
    delay(20);
  }
}

void moveArmBackward() {
  for (uint16_t pulselen = FORWARD_MAX; pulselen > FORWARD_MIN; pulselen--) {
    myServo.setPWM(FORWARD, 0, pulselen);
    delay(20);
  }
}

void moveArmForward() {
  for (uint16_t pulselen = FORWARD_MIN; pulselen < FORWARD_MAX; pulselen++) {
    myServo.setPWM(FORWARD, 0, pulselen);
    delay(20);
  }
}

void moveArmUp() {
  for (uint16_t pulselen = VERTICAL_MIN; pulselen < VERTICAL_MAX; pulselen++) {
    myServo.setPWM(VERTICAL, 0, pulselen);
    delay(20);
  }
}

void moveArmDown() {
  for (uint16_t pulselen = VERTICAL_MAX; pulselen > VERTICAL_MIN; pulselen--) {
    myServo.setPWM(VERTICAL, 0, pulselen);
    delay(20);
  }
}