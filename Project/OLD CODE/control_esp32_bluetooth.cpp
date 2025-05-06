#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

// Right motor
int enableRightMotor = 27; // ENA
int rightMotorPin1   = 14; // IN1
int rightMotorPin2   = 33; // IN2

// Left motor
int enableLeftMotor = 32; // ENB
int leftMotorPin1   = 12; // IN3
int leftMotorPin2   = 13; // IN4

#define MAX_MOTOR_SPEED 255

const int PWMFreq = 1000; /* 1 KHz */
const int PWMResolution = 8;
const int rightMotorPWMSpeedChannel = 4;
const int leftMotorPWMSpeedChannel = 5;


void rotateMotor(int rightMotorSpeed, int leftMotorSpeed)
{
  if (rightMotorSpeed < 0)
  {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);    
  }
  else if (rightMotorSpeed > 0)
  {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);      
  }
  else
  {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, LOW);      
  }

  if (leftMotorSpeed < 0)
  {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);    
  }
  else if (leftMotorSpeed > 0)
  {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);      
  }
  else
  {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, LOW);      
  }

  ledcWrite(rightMotorPWMSpeedChannel, abs(rightMotorSpeed));
  ledcWrite(leftMotorPWMSpeedChannel, abs(leftMotorSpeed));  
}

void setUpPinModes()
{
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);

  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  // Set up PWM for speed control
  ledcSetup(rightMotorPWMSpeedChannel, PWMFreq, PWMResolution);
  ledcSetup(leftMotorPWMSpeedChannel, PWMFreq, PWMResolution);
  ledcAttachPin(enableRightMotor, rightMotorPWMSpeedChannel);
  ledcAttachPin(enableLeftMotor, leftMotorPWMSpeedChannel);

  rotateMotor(0, 0);
}

void setup()
{
  setUpPinModes();
  Serial.begin(115200);
  SerialBT.begin("BitemeAI"); // Bluetooth name as shown on your phone
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}


void loop()
{static unsigned long lastSent = 0;

  if (SerialBT.hasClient() && millis() - lastSent > 300) {
    SerialBT.println("READY");  // Or any short string
    lastSent = millis();
  }
  int rightMotorSpeed = 0;
  int leftMotorSpeed = 0;

  if (SerialBT.available()) {
    char command = SerialBT.read();

    if (command == 'U') {
      // Up pressed
      rightMotorSpeed = MAX_MOTOR_SPEED;
      leftMotorSpeed  = MAX_MOTOR_SPEED;
    }
    else if (command == 'D') {
      // Down pressed
      rightMotorSpeed = -MAX_MOTOR_SPEED;
      leftMotorSpeed  = -MAX_MOTOR_SPEED;
    }
    else if (command == 'L') {
      // Left pressed
      rightMotorSpeed = MAX_MOTOR_SPEED;
      leftMotorSpeed  = -MAX_MOTOR_SPEED;
    }
    else if (command == 'R') {
      // Right pressed
      rightMotorSpeed = -MAX_MOTOR_SPEED;
      leftMotorSpeed  = MAX_MOTOR_SPEED;
    }
    else {
      // Any other command = stop
      rightMotorSpeed = 0;
      leftMotorSpeed  = 0;
    }

    rotateMotor(rightMotorSpeed, leftMotorSpeed);
  }
  else if (SerialBT.hasClient()) {
    // No command available, but still connected: STOP the rover
    rotateMotor(0, 0);
  }

  delay(10); // Helps avoid busy-waiting and CPU overload
}
