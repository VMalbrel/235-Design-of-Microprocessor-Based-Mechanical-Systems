#include <WiFi.h>

// Right motor
int enableRightMotor = 27; 
int rightMotorPin1   = 14;
int rightMotorPin2   = 33;

// Left motor
int enableLeftMotor = 32;
int leftMotorPin1   = 12;
int leftMotorPin2   = 13;

// IR Sensors
int leftMostIR  = 39; 
int leftIR      = 34; 
int rightIR     = 36; 
int rightMostIR = 4;  

// Motor control settings
#define MAX_MOTOR_SPEED 255
int speed_motor = 200;

const int PWMFreq = 1000;
const int PWMResolution = 8;
const int rightMotorPWMSpeedChannel = 4;
const int leftMotorPWMSpeedChannel = 5;

// Servers
WiFiServer serverMotor(1234); // Commands
WiFiServer serverIR(5678);    // Sensors

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed) {
  if (rightMotorSpeed < 0) {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);    
  } else if (rightMotorSpeed > 0) {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);      
  } else {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, LOW);      
  }

  if (leftMotorSpeed < 0) {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);    
  } else if (leftMotorSpeed > 0) {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);      
  } else {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, LOW);      
  }

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

  // Setup Wi-Fi
  const char* ssid = "ESP32-ROVER";
  const char* password = "12345678";
  WiFi.softAP(ssid, password);
  delay(100);

  Serial.println("Access Point started.");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  serverMotor.begin();
  serverIR.begin();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  // Handle motor server
  WiFiClient clientMotor = serverMotor.available();
  if (clientMotor) {
    Serial.println("Motor client connected.");
    while (clientMotor.connected()) {
      if (clientMotor.available()) {
        char command = clientMotor.read();
        int rightMotorSpeed = 0;
        int leftMotorSpeed = 0;

        if (command == 'U') {
          rightMotorSpeed = speed_motor;
          leftMotorSpeed  = speed_motor;
        }
        else if (command == 'D') {
          rightMotorSpeed = -speed_motor;
          leftMotorSpeed  = -speed_motor;
        }
        else if (command == 'L') {
          rightMotorSpeed = speed_motor;
          leftMotorSpeed  = -speed_motor;
        }
        else if (command == 'R') {
          rightMotorSpeed = -speed_motor;
          leftMotorSpeed  = speed_motor;
        }
        else if (command == '1') {
          speed_motor = 100;
        }
        else if (command == '2') {
          speed_motor = 180;
        }
        else if (command == '3') {
          speed_motor = 255;
        }
        else {
          rightMotorSpeed = 0;
          leftMotorSpeed  = 0;
        }

        rotateMotor(rightMotorSpeed, leftMotorSpeed);

        clientMotor.println("Command received: " + String(command));
      }
    }
    clientMotor.stop();
    Serial.println("Motor client disconnected.");
  }

  // Handle IR sensor server
  WiFiClient clientIR = serverIR.available();
  if (clientIR) {
    Serial.println("IR client connected.");
    while (clientIR.connected()) {
      int lmIR = digitalRead(leftMostIR);
      int lIR  = digitalRead(leftIR);
      int rIR  = digitalRead(rightIR);
      int rmIR = digitalRead(rightMostIR);

      String irData = String(lmIR) + "," + String(lIR) + "," + String(rIR) + "," + String(rmIR);
      clientIR.println(irData);

      delay(100);  // Send IR data every 100ms
    }
    clientIR.stop();
    Serial.println("IR client disconnected.");
  }

  delay(10);
}
