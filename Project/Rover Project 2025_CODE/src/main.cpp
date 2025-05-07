#include <WiFi.h>

// Motors
int enableRightMotor = 27;
int rightMotorPin1 = 14;
int rightMotorPin2 = 33;
int enableLeftMotor = 32;
int leftMotorPin1 = 12;
int leftMotorPin2 = 13;
#define MAX_MOTOR_SPEED 255
int speed_motor = 200;
const int PWMFreq = 1000;
const int PWMResolution = 8;
const int rightMotorPWMSpeedChannel = 4;
const int leftMotorPWMSpeedChannel = 5;

// IR Sensors
int leftMostIR = 36;
int leftIR = 39;
int rightIR = 34;
int rightMostIR = 25;

// Proximity Sensor
int proximityTrig = 22;
int proximityEcho = 20;

// Buzzer
int buzzer= 4;

// Initialize Wifi Ports
WiFiServer serverMotor(1);
WiFiServer serverIR(2);
WiFiServer serverVelocity(3);
WiFiClient clientMotor;
WiFiClient clientIR;
WiFiClient clientVelocity;


// Change Motor Speed
void rotateMotor(int rightMotorSpeed, int leftMotorSpeed) {
  digitalWrite(rightMotorPin1, rightMotorSpeed > 0);
  digitalWrite(rightMotorPin2, rightMotorSpeed < 0);
  digitalWrite(leftMotorPin1,  leftMotorSpeed > 0);
  digitalWrite(leftMotorPin2,  leftMotorSpeed < 0);
  ledcWrite(rightMotorPWMSpeedChannel, abs(rightMotorSpeed));
  ledcWrite(leftMotorPWMSpeedChannel, abs(leftMotorSpeed));
}

float distance() {
  // Trigger the ultrasonic pulse
  digitalWrite(proximityTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(proximityTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(proximityTrig, LOW);
  // Measure the duration of the echo pulse
  long duration = pulseIn(proximityEcho, HIGH, 30000); // 30 ms timeout (~5 m)
  // Convert time to distance (speed of sound = 34300 cm/s)
  float distance_cm = (duration / 2.0) * 0.0343;
  return distance_cm;  // return distance in cm
}

// Setup Pins
void setUpPinModes() {
  // Right Motor
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  // Left Motor
  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);
  // Motor PWM
  ledcSetup(rightMotorPWMSpeedChannel, PWMFreq, PWMResolution);
  ledcSetup(leftMotorPWMSpeedChannel, PWMFreq, PWMResolution);
  ledcAttachPin(enableRightMotor, rightMotorPWMSpeedChannel);
  ledcAttachPin(enableLeftMotor, leftMotorPWMSpeedChannel);
  rotateMotor(0, 0);
  // IR Sensors
  pinMode(leftMostIR, INPUT);
  pinMode(leftIR, INPUT);
  pinMode(rightIR, INPUT);
  pinMode(rightMostIR, INPUT);
  // Proximity Sensor
  pinMode(proximityTrig, OUTPUT);
  pinMode(proximityEcho, INPUT);
  // Buzzer
  pinMode(buzzer, OUTPUT);
}

void setup() {
  setUpPinModes();
  Serial.begin(9600); // Need to Ensure we are at correct baud rate to see in serial monitor
  WiFi.softAP("ESP32-ROVER", "12345678");
  delay(100);

  // Start Each Loop
  serverMotor.begin();
  serverIR.begin();
  serverVelocity.begin();

  // Turn On LED When Wifi Connection is Established
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  // Accept new clients
  if (!clientMotor || !clientMotor.connected()) {
    clientMotor = serverMotor.available();
    if (clientMotor) Serial.println("Motor client connected");
  }
  if (!clientIR || !clientIR.connected()) {
    clientIR = serverIR.available();
    if (clientIR) Serial.println("IR client connected");
  }
  if (!clientVelocity || !clientVelocity.connected()) {
    clientVelocity = serverVelocity.available();
    if (clientVelocity) Serial.println("Velocity client connected");
  }

  // Motor Control
  if (clientMotor && clientMotor.connected() && clientMotor.available()) {
    char command = clientMotor.read();
    int rightMotorSpeed = 0;
    int leftMotorSpeed = 0;
    // Manual
    if (command == 'U') {
      rightMotorSpeed = speed_motor;
      leftMotorSpeed = speed_motor;
      rotateMotor(rightMotorSpeed, leftMotorSpeed);
    } else if (command == 'D') {
      rightMotorSpeed = -speed_motor;
      leftMotorSpeed = -speed_motor;
      rotateMotor(rightMotorSpeed, leftMotorSpeed);
    } else if (command == 'L') {
      rightMotorSpeed = speed_motor;
      leftMotorSpeed = -speed_motor;
      rotateMotor(rightMotorSpeed, leftMotorSpeed);
    } else if (command == 'R') {
      rightMotorSpeed = -speed_motor;
      leftMotorSpeed = speed_motor;
      rotateMotor(rightMotorSpeed, leftMotorSpeed);
    } else if (command == 'S') {
      rightMotorSpeed = 0;
      leftMotorSpeed = 0;
      rotateMotor(rightMotorSpeed, leftMotorSpeed);
    // Automatic
    } else if (command == 'A') {
      if (digitalRead(leftMostIR) == 1) {
        rotateMotor(MAX_MOTOR_SPEED, MAX_MOTOR_SPEED*.75);
      } else if (digitalRead(leftIR) == 1) {
        rotateMotor(MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
      } else if (digitalRead(rightIR) == 1) {
        rotateMotor(MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
      } else if (digitalRead(rightMostIR) == 1) {
        rotateMotor(MAX_MOTOR_SPEED*.75, MAX_MOTOR_SPEED);
      } else {
        rotateMotor(0, 0);
      }
    }
  }

  // Send IR Sensor Data
  static unsigned long lastSendTime = 0;
  if (clientIR && clientIR.connected() && millis() - lastSendTime >= 100) {
    lastSendTime = millis();
    String irData = String(digitalRead(leftMostIR)) +
                    String(digitalRead(leftIR)) +
                    String(digitalRead(rightIR)) +
                    String(digitalRead(rightMostIR));
    clientIR.println(irData);
  }

  // Change Velocity
  if (clientVelocity && clientVelocity.connected() && clientVelocity.available()) {
    char velocity = clientVelocity.read();
    if (isdigit(velocity)) {
      int velocityValue = velocity - '0';
      speed_motor = MAX_MOTOR_SPEED * (velocityValue / 9.0);
    }
  }
  
  // Proximity Interrupt
  float d = distance();

  if (d > 0 && d < 5) {
    while (true) {
      float d_check = distance();
      if (d_check < 0 || d_check >= 5) break;
      rotateMotor(0, 0);           // Stop motor
      digitalWrite(buzzer, HIGH);  // Activate buzzer
    }
    digitalWrite(buzzer, LOW);     // Turn off buzzer
  }

  delay(5); // Minimal delay to avoid CPU hogging
}
