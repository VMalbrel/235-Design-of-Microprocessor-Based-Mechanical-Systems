#include <WiFi.h>

// Right Motor
int enableRightMotor = 27; // ENA
int rightMotorPin1   = 14; // IN1
int rightMotorPin2   = 33; // IN2

// Left Motor
int enableLeftMotor = 32; // ENB
int leftMotorPin1   = 12; // IN3
int leftMotorPin2   = 13; // IN4

// IR Sensors
int leftMostIR = 39; //A1
int leftIR = 34; // A2
int rightIR = 36; // A3
int rightMostIR = 4; //A4

// Motor Settings
#define MAX_MOTOR_SPEED 255
int speedMotorMax = 255;
int speedMotorMedium = 180;
int speedMotorOFF = 0;

const int PWMFreq = 1000; /* 1 KHz */
const int PWMResolution = 8;
const int rightMotorPWMSpeedChannel = 4;
const int leftMotorPWMSpeedChannel = 5;

// Operation Variable Initialization
static bool ON = false;
static bool automatic = false;

WiFiServer server(1234);  // Create a TCP server on port 1234

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

  pinMode(leftMostIR, INPUT);
  pinMode(leftIR, INPUT);
  pinMode(rightIR, INPUT);
  pinMode(rightMostIR, INPUT);

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

  // Setup Wi-Fi Access Point
  const char* ssid = "ESP32-ROVER";
  const char* password = "12345678";

  WiFi.softAP(ssid, password);
  delay(100);

  Serial.println("Access Point started.");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  server.begin();  // Start TCP server
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop()
{
  WiFiClient client = server.available();  // Wait for a new client

  if (client) {
    Serial.println("Client connected.");
    while (client.connected()) {
      if (client.available()) {
        char command = client.read();
        
        // ON OFF Buttons
        if (command = 'S') {
          ON = true;
        }
        if (command = 'S') {
          ON = false;
          rightMotorSpeed = speedMotorOFF;
          leftMotorSpeed  = speedMotorOFF;
        }
        // Check mode of operation
        if (command = 'A') // Automatic
          automatic = true;
        if (command = 'M') // Manual
          automatic = false;

        // Read IR Sensors
        int lmIR = digitalRead(leftMostIR); // Left Most IR Sensor
        int lIR = digitalRead(leftIR); // Left Middle IR Sensor
        int rIR = digitalRead(rightIR); // Right Middle IR Sensor
        int rmIR = digitalRead(rightMostIR); // Right Most IR Sensor
        
        // Automatic Operation - IR Sensor
        if automatic == true {
          if ON == true {
            if lmIR == 1 { // Left Most IR Sensor Active
              rightMotorSpeed = speedMotorMax;
              leftMotorSpeed  = -speedMotorMedium;
            }
            if lIR == 1 { // Left Middle IR Sensor Active
              rightMotorSpeed = speedMotorMax;
              leftMotorSpeed  = speedMotorMedium;
            }
            if rIR == 1 { // Right Middle IR Sensor Active
              rightMotorSpeed = speedMotorMedium;
              leftMotorSpeed  = speedMotorMax;
            }
            if rmIR == 1 { // Right Most IR Sensor Active
              rightMotorSpeed = -speedMotorMedium;
              leftMotorSpeed  = speedMotorMax;
            }
            else {
              rightMotorSpeed = speedMotorOFF;
              leftMotorSpeed  = speedMotorOFF;
            }
          }
        }

        // Manual Operation
        if automatic == false {
          if (command == 'U') { // Forward
            rightMotorSpeed = speed_motor;
            leftMotorSpeed  = speed_motor;
            delay(100);
          }
          else if (command == 'D') { // Backward
            rightMotorSpeed = -speed_motor;
            leftMotorSpeed  = -speed_motor;
            delay(100);
          }
          else if (command == 'L') { // Left Turn
            rightMotorSpeed = speed_motor;
            leftMotorSpeed  = -speed_motor;
            delay(100);
          }
          else if (command == 'R') { // Right Turn
            rightMotorSpeed = -speed_motor;
            leftMotorSpeed  = speed_motor;
            delay(100);
          }
          // Change Motor Speed
          else if (command == '1') { // Slow
            speed_motor = 100;
          }
          else if (command == '2') { // Medium
            speed_motor = speedMotorMedium;
          }
          else if (command == '3') { // Fast
            speed_motor = speedMotorMax;
          }
          // Stop Motors
          rightMotorSpeed = speedMotorOFF;
          leftMotorSpeed  = speedMotorOFF;
        }
        
        // Turn on Motors
        rotateMotor(rightMotorSpeed, leftMotorSpeed);

        // Optional: send back confirmation
        client.println("Command received: " + String(command));

        // Send IR Data
        Serial.print("IR: LM=");
        Serial.print(lmIR);
        Serial.print(", L=");
        Serial.print(lIR);
        Serial.print(", R=");
        Serial.print(rIR);
        Serial.print(", RM=");
        Serial.println(rmIR);
      }
    }
    client.stop();
    Serial.println("Client disconnected.");
  }
  delay(10);  // Prevent overload
}
