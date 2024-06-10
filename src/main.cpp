#include <Arduino.h>
#include <SPI.h>
#include <WiFiNINA.h>

// WiFi credentials
const char ssid[] = "example_ssid";       // your network SSID (name)
const char pass[] = "example_password";   // your network password

int status = WL_IDLE_STATUS;
WiFiServer server(80);

// Pin definitions for control
const int enablePin1 = 2;  // Enable pin for motor 1 (EL)
const int directionPin1 = 3;  // Direction pin for motor 1 (Z/F)
const int speedPin1 = 5;  // Speed control pin for motor 1 (VR)

const int enablePin2 = 6;  // Enable pin for motor 2 (EL)
const int directionPin2 = 7;  // Direction pin for motor 2 (Z/F)
const int speedPin2 = 9;  // Speed control pin for motor 2 (VR) 

// Pin definition for Hall sensor
const int hallSensorPin = 10;  // Hall sensor input pin

// Pin definition for return button
const int returnButtonPin = 11; // Return button input pin

volatile int position = 0;
volatile bool magnetDetected = false;
volatile bool returnButtonPressed = false;
int targetPosition = -1;
unsigned long lastMotorActivityTime = 0;

void startMotors() {
  digitalWrite(enablePin1, HIGH);
  digitalWrite(directionPin1, LOW);
  analogWrite(speedPin1, 255);

  digitalWrite(enablePin2, HIGH);
  digitalWrite(directionPin2, HIGH);
  analogWrite(speedPin2, 255);

  Serial.println("Motors are moving forward at full speed");
  lastMotorActivityTime = millis();
}

void reverseMotors() {
  digitalWrite(enablePin1, HIGH);
  digitalWrite(directionPin1, HIGH);
  analogWrite(speedPin1, 255);

  digitalWrite(enablePin2, HIGH);
  digitalWrite(directionPin2, LOW);
  analogWrite(speedPin2, 255);

  Serial.println("Motors are moving in reverse at full speed");
  lastMotorActivityTime = millis();
}

void stopMotors() {
  digitalWrite(enablePin1, LOW);
  analogWrite(speedPin1, 0);  
  digitalWrite(enablePin2, LOW);
  analogWrite(speedPin2, 0);  

  Serial.println("Motors stopped");
}

void hallSensorISR() {
  magnetDetected = true;
}

void returnButtonISR() {
  returnButtonPressed = true;
}

void executeCommand(String command, WiFiClient &client) {
  if (command == "START") {
    startMotors();
    client.println("Motors started");
  } else if (command == "STOP") {
    stopMotors();
    client.println("Motors stopped");
  } else if (command == "REVERSE") {
    reverseMotors();
    client.println("Motors reversed");
  } else if (command.startsWith("GOTO")) {
    int newPos = command.substring(4).toInt();
    if (newPos >= 0 && newPos <= 15) {
      targetPosition = newPos;
      if (targetPosition > position) {
        startMotors();
      } else if (targetPosition < position) {
        reverseMotors();
      }
      client.print("Moving to position: ");
      client.println(targetPosition);
    } else {
      client.println("Invalid position");
    }
  } else if (command == "SENSOR") {
    client.print("Current Position: ");
    client.println(position);
  } else if (command == "RETURN") {
    returnButtonPressed = true;
    client.println("Returning to original position");
  }
}

void setup() {
  // Initialize serial for debugging
  Serial.begin(9600);

  // Initialize pins for motor 1
  pinMode(enablePin1, OUTPUT);
  pinMode(directionPin1, OUTPUT);
  pinMode(speedPin1, OUTPUT);

  // Initialize pins for motor 2
  pinMode(enablePin2, OUTPUT);
  pinMode(directionPin2, OUTPUT);
  pinMode(speedPin2, OUTPUT);

  // Initialize pin for Hall sensor
  pinMode(hallSensorPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(hallSensorPin), hallSensorISR, FALLING);

  // Initialize pin for return button
  pinMode(returnButtonPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(returnButtonPin), returnButtonISR, FALLING);

  // Connect to WiFi
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);
    delay(10000);
  }

  Serial.println("Connected to WiFi");

  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  server.begin();
}

void loop() {
  // Check for magnet detection and update position
  if (magnetDetected) {
    magnetDetected = false;
    if (digitalRead(directionPin1) == LOW) {
      position++;
    } else {
      position--;
    }

    // Bound the position within 0 and 15
    position = max(0, min(position, 15));

    // Stop motors if out of bounds or target reached
    if (position == 0 || position == 15 || position == targetPosition) {
      stopMotors();
      targetPosition = -1;
    }

    Serial.print("Current Position: ");
    Serial.println(position);
  }

  // Handle return button press
  if (returnButtonPressed) {
    Serial.println("Return button pressed");
    returnButtonPressed = false;
    targetPosition = -1;  // Cancel any current target position
    reverseMotors();  // Move backwards to the original position
  }

  // Handle client connections
  WiFiClient client = server.available();
  if (client) {
    String currentLine = "";
    String command = "";  // Declare command string here
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        if (c == '\n') {
          if (currentLine.length() == 0) {
            // Send HTTP headers
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/plain");
            client.println("Access-Control-Allow-Origin: *");
            client.println();

            // Execute command based on parsed input
            executeCommand(command, client);

            client.println();
            break;
          } else {
            currentLine = "";
          }
        } else if (c != '\r') {
          currentLine += c;
          // Capture the command when the line starts with "GET /"
          if (currentLine.startsWith("GET /")) {
            command = currentLine.substring(5, currentLine.indexOf(' ', 5));  // Adjust to properly extract the command
          }
        }
      }
    }
    client.stop();
  }

  // Check if 30 seconds have passed since the last motor activity
  if (millis() - lastMotorActivityTime > 30000) {
    // Check if the robot is not at the home position and not moving
    if (position != 0 && !digitalRead(enablePin1) && !digitalRead(enablePin2)) {
      Serial.println("No motor activity for 30 seconds. Returning home.");
      reverseMotors();  // Move backwards to the original position
    }
  }

  delay(100);  // Reduce potential for serial flooding
}