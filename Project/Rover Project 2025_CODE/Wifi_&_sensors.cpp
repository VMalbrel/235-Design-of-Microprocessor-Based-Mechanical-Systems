#include <WiFi.h>

// Motors
int enableRightMotor = 27;
int rightMotorPin1 = 14;
int rightMotorPin2 = 33;
int enableLeftMotor = 32;
int leftMotorPin1 = 12;
int leftMotorPin2 = 13;

// IR Sensors
int leftMostIR  = 39;
int leftIR      = 34;
int rightIR     = 36;
int rightMostIR = 4;

#define MAX_MOTOR_SPEED 255
int speed_motor = 200;

const int PWMFreq = 1000;
const int PWMResolution = 8;
const int rightMotorPWMSpeedChannel = 4;
const int leftMotorPWMSpeedChannel = 5;

WiFiServer serverMotor(1234);
WiFiServer serverIR(5678);

WiFiClient clientMotor;
WiFiClient clientIR;

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed) {
  digitalWrite(rightMotorPin1, rightMotorSpeed > 0);
  digitalWrite(rightMotorPin2, rightMotorSpeed < 0);
  digitalWrite(leftMotorPin1,  leftMotorSpeed > 0);
  digitalWrite(leftMotorPin2,  leftMotorSpeed < 0);

  ledcWrite(rightMotorPWMSpeedChannel, abs(rightMotorSpeed));
  ledcWrite(leftMotorPWMSpeedChannel, abs(leftMotorSpeed));
}

void setUpPinModes() {
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  pinMode(leftMostIR, INPUT);
  pinMode(leftIR, INPUT);
  pinMode(rightIR, INPUT);
  pinMode(rightMostIR, INPUT);

  ledcSetup(rightMotorPWMSpeedChannel, PWMFreq, PWMResolution);
  ledcSetup(leftMotorPWMSpeedChannel, PWMFreq, PWMResolution);
  ledcAttachPin(enableRightMotor, rightMotorPWMSpeedChannel);
  ledcAttachPin(enableLeftMotor, leftMotorPWMSpeedChannel);

  rotateMotor(0, 0);
}

void setup() {
  setUpPinModes();
  Serial.begin(115200);

  WiFi.softAP("ESP32-ROVER", "12345678");
  delay(100);
  Serial.println("Wi-Fi AP started: " + WiFi.softAPIP().toString());

  serverMotor.begin();
  serverIR.begin();
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

  // Handle motor command
  if (clientMotor && clientMotor.connected() && clientMotor.available()) {
    char command = clientMotor.read();
    int rightMotorSpeed = 0;
    int leftMotorSpeed = 0;

    if (command == 'U') {
      rightMotorSpeed = speed_motor;
      leftMotorSpeed = speed_motor;
    } else if (command == 'D') {
      rightMotorSpeed = -speed_motor;
      leftMotorSpeed = -speed_motor;
    } else if (command == 'L') {
      rightMotorSpeed = speed_motor;
      leftMotorSpeed = -speed_motor;
    } else if (command == 'R') {
      rightMotorSpeed = -speed_motor;
      leftMotorSpeed = speed_motor;
    } else if (command == '1') {
      speed_motor = 100;
    } else if (command == '2') {
      speed_motor = 180;
    } else if (command == '3') {
      speed_motor = 255;
    }

    rotateMotor(rightMotorSpeed, leftMotorSpeed);
  }

  // Handle sensor data
  static unsigned long lastSendTime = 0;
  if (clientIR && clientIR.connected() && millis() - lastSendTime >= 100) {
    lastSendTime = millis();
    String irData = String(digitalRead(leftMostIR)) + "," +
                    String(digitalRead(leftIR)) + "," +
                    String(digitalRead(rightIR)) + "," +
                    String(digitalRead(rightMostIR));
    clientIR.println(irData);
  }

  delay(5); // Minimal delay to avoid CPU hogging
}
