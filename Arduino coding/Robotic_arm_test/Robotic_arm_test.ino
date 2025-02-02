#include <ESP32Servo.h>  // Include the Servo library

// Motor pins
#define PIN_EN_M_LEFT 22
#define PIN_1_M_LEFT 17
#define PIN_2_M_LEFT 5
#define PIN_EN_M_RIGHT 23
#define PIN_1_M_RIGHT 18
#define PIN_2_M_RIGHT 19

// Sensor pins
#define PIN_LINE_LEFT 4
#define PIN_LINE_RIGHT 2

// Servo pins
#define PIN_SERVO_BASE 25
#define PIN_SERVO_SHOULDER 26
#define PIN_SERVO_ELBOW 27
#define PIN_SERVO_GRIPPER 14

// PI controller parameters
float Kp = 0.059;
float Ki = 10.7;

float Kp_right = 0.0646;
float Ki_left = 2.93;

// Variables
int BASE_SPEED = 150;
float integral = 0;
unsigned long previousTime = 0;

// Servo objects
Servo servoBase, servoShoulder, servoElbow, servoGripper;

void setup() {
  Serial.begin(9600);

  // Motor pin setup
  pinMode(PIN_EN_M_LEFT, OUTPUT);
  pinMode(PIN_1_M_LEFT, OUTPUT);
  pinMode(PIN_2_M_LEFT, OUTPUT);
  pinMode(PIN_EN_M_RIGHT, OUTPUT);
  pinMode(PIN_1_M_RIGHT, OUTPUT);
  pinMode(PIN_2_M_RIGHT, OUTPUT);

  // Sensor pin setup
  pinMode(PIN_LINE_LEFT, INPUT);
  pinMode(PIN_LINE_RIGHT, INPUT);

  // Attach servos
  servoBase.attach(PIN_SERVO_BASE);
  servoShoulder.attach(PIN_SERVO_SHOULDER);
  servoElbow.attach(PIN_SERVO_ELBOW);
  servoGripper.attach(PIN_SERVO_GRIPPER);
}

void loop() {
  // Read sensor values
  int sensorLeft = digitalRead(PIN_LINE_LEFT);
  int sensorRight = digitalRead(PIN_LINE_RIGHT);

  // Calculate error
  int error = sensorRight - sensorLeft;

  // PI control
  unsigned long currentTime = millis();
  float elapsedTime = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;

  integral += error * elapsedTime;
  float controlSignal = Kp * error + Ki * integral;

  int leftSpeed = BASE_SPEED - controlSignal;
  int rightSpeed = BASE_SPEED + controlSignal;

  // Constrain speed to 0-255
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // Control logic for different sensor states
  if (sensorLeft == 0 && sensorRight == 0) {
    moveForward(leftSpeed, rightSpeed);
  } else if (sensorLeft == 0 && sensorRight == 1) {
    moveRight(leftSpeed, rightSpeed);
  } else if (sensorLeft == 1 && sensorRight == 0) {
    moveLeft(leftSpeed, rightSpeed);
  } else {
    stopMotors();
    operateArm();  // Call function to operate the robotic arm when no line is detected
  }

  delay(50);
}

void moveForward(int leftSpeed, int rightSpeed) {
  digitalWrite(PIN_1_M_LEFT, LOW);
  digitalWrite(PIN_2_M_LEFT, HIGH);
  analogWrite(PIN_EN_M_LEFT, leftSpeed);

  digitalWrite(PIN_1_M_RIGHT, LOW);
  digitalWrite(PIN_2_M_RIGHT, HIGH);
  analogWrite(PIN_EN_M_RIGHT, rightSpeed);
}

void moveRight(int leftSpeed, int rightSpeed) {
  digitalWrite(PIN_1_M_LEFT, LOW);
  digitalWrite(PIN_2_M_LEFT, HIGH);
  analogWrite(PIN_EN_M_LEFT, BASE_SPEED);

  digitalWrite(PIN_1_M_RIGHT, LOW);
  digitalWrite(PIN_2_M_RIGHT, LOW);
  analogWrite(PIN_EN_M_RIGHT, 0);
}

void moveLeft(int leftSpeed, int rightSpeed) {
  digitalWrite(PIN_1_M_LEFT, LOW);
  digitalWrite(PIN_2_M_LEFT, LOW);
  analogWrite(PIN_EN_M_LEFT, 0);

  digitalWrite(PIN_1_M_RIGHT, LOW);
  digitalWrite(PIN_2_M_RIGHT, HIGH);
  analogWrite(PIN_EN_M_RIGHT, BASE_SPEED);
}

void stopMotors() {
  digitalWrite(PIN_1_M_LEFT, LOW);
  digitalWrite(PIN_2_M_LEFT, LOW);
  analogWrite(PIN_EN_M_LEFT, 0);

  digitalWrite(PIN_1_M_RIGHT, LOW);
  digitalWrite(PIN_2_M_RIGHT, LOW);
  analogWrite(PIN_EN_M_RIGHT, 0);
}

void operateArm() {
  
  servoBase.write(90);     
  servoShoulder.write(45);  
  servoElbow.write(135);    
  servoGripper.write(30);   

  delay(4000);  

  
  servoBase.write(0);      
  servoShoulder.write(90);  
  servoElbow.write(90);     
  servoGripper.write(90);   

  delay(500);  
}
