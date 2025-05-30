#include <WiFi.h>

// Right motor
int enableRightMotor = 27; // ENA
int rightMotorPin1   = 14; // IN1
int rightMotorPin2   = 33; // IN2

// Left motor
int enableLeftMotor = 32; // ENB
int leftMotorPin1   = 12; // IN3
int leftMotorPin2   = 13; // IN4

#define MAX_MOTOR_SPEED 255
int speed_motor = 200;

const int PWMFreq = 1000; /* 1 KHz */
const int PWMResolution = 8;
const int rightMotorPWMSpeedChannel = 4;
const int leftMotorPWMSpeedChannel = 5;

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
          speed_motor = 100;  // slow
        }
        else if (command == '2') {
          speed_motor = 180;  // medium
        }
        else if (command == '3') {
          speed_motor = 255;  // fast
        }
        else {
          rightMotorSpeed = 0;
          leftMotorSpeed  = 0;
        }

        rotateMotor(rightMotorSpeed, leftMotorSpeed);

        // Optional: send back confirmation
        client.println("Command received: " + String(command));
      }
    }
    client.stop();
    Serial.println("Client disconnected.");
  }

  delay(10);  // Prevent overload
}
