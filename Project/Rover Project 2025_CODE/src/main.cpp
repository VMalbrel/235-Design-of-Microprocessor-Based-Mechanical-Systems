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
int leftMostIR = 39;
int leftIR = 34;
int rightIR = 36;
int rightMostIR = 4;

// Proximity Sensor
int proximity = 0;
// Buzzer
int buzzer = 0;

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
  // pinMode(proximity, INPUT);
  // // Buzzer
  // pinMode(buzzer, OUTPUT);
}

void setup() {
  setUpPinModes();
  Serial.begin(115200); // Need to Ensure we are at correct baud rate to see in serial monitor
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

  // Manual Motor Control
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
    }
    rotateMotor(rightMotorSpeed, leftMotorSpeed);
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
  
  // // Proximity Interrupt
  // int distance = digitalRead(proximity);
  // if (distance < 1) {
  //   while (distance < 1) {
  //     rotateMotor(0, 0);
  //     digitalWrite(buzzer, HIGH);
  //     delay(.25);
  //     int distance = digitalRead(proximity);
  //   }
  //   digitalWrite(buzzer, LOW);
  // }


  delay(5); // Minimal delay to avoid CPU hogging
}
