#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

// Motor driver pins
const int leftMotorForward = D1;
const int leftMotorBackward = D2;
const int rightMotorForward = D3;
const int rightMotorBackward = D4;
const int leftMotorSpeed = D5;  // EN_A for left motor
const int rightMotorSpeed = D6; // EN_B for right motor

const int max_speed = 160;  //assuming 9V
const int normal_speed = 130; // assuming 6V

// 160 = 8.5v
// 130 = 7.5v

// WiFi details
const char* ssid = "#r";
const char* password = "qwju8977";

// Create a web server on port 80
ESP8266WebServer server(80);

void setup() {
  Serial.begin(115200);
  
  // Initialize motor pins
  pinMode(leftMotorForward, OUTPUT);
  pinMode(leftMotorBackward, OUTPUT);
  pinMode(rightMotorForward, OUTPUT);
  pinMode(rightMotorBackward, OUTPUT);
  pinMode(leftMotorSpeed, OUTPUT);
  pinMode(rightMotorSpeed, OUTPUT);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Set up URL routes
  server.on("/", handleRoot);
  server.begin();
  Serial.println("Server started");
}

// Movement functions
void moveForward(int speed) {
  analogWrite(leftMotorSpeed, speed);
  analogWrite(rightMotorSpeed, speed);
  digitalWrite(leftMotorForward, HIGH);
  digitalWrite(leftMotorBackward, LOW);
  digitalWrite(rightMotorForward, HIGH);
  digitalWrite(rightMotorBackward, LOW);
}

void moveBackward(int speed) {
  analogWrite(leftMotorSpeed, speed);
  analogWrite(rightMotorSpeed, speed);
  digitalWrite(leftMotorForward, LOW);
  digitalWrite(leftMotorBackward, HIGH);
  digitalWrite(rightMotorForward, LOW);
  digitalWrite(rightMotorBackward, HIGH);
}

void turnRight() {
  analogWrite(leftMotorSpeed, normal_speed);  
  analogWrite(rightMotorSpeed, 0);
  digitalWrite(leftMotorForward, HIGH);
  digitalWrite(leftMotorBackward, LOW);
  digitalWrite(rightMotorForward, LOW);
  digitalWrite(rightMotorBackward, LOW);
}

void turnLeft() {
  analogWrite(leftMotorSpeed, 0);
  analogWrite(rightMotorSpeed, normal_speed);  
  digitalWrite(leftMotorForward, LOW);
  digitalWrite(leftMotorBackward, LOW);
  digitalWrite(rightMotorForward, HIGH);
  digitalWrite(rightMotorBackward, LOW);
}

void fastTurnRight() {
  analogWrite(leftMotorSpeed, max_speed);
  analogWrite(rightMotorSpeed, max_speed);
  digitalWrite(leftMotorForward, HIGH);
  digitalWrite(leftMotorBackward, LOW);
  digitalWrite(rightMotorForward, LOW);
  digitalWrite(rightMotorBackward, HIGH);
}

void fastTurnLeft() {
  analogWrite(leftMotorSpeed, max_speed);
  analogWrite(rightMotorSpeed, max_speed);
  digitalWrite(leftMotorForward, LOW);
  digitalWrite(leftMotorBackward, HIGH);
  digitalWrite(rightMotorForward, HIGH);
  digitalWrite(rightMotorBackward, LOW);
}

void stopMotors() {
  analogWrite(leftMotorSpeed, 0);
  analogWrite(rightMotorSpeed, 0);
  digitalWrite(leftMotorForward, LOW);
  digitalWrite(leftMotorBackward, LOW);
  digitalWrite(rightMotorForward, LOW);
  digitalWrite(rightMotorBackward, LOW);
}

// Handle incoming HTTP requests
void handleRoot() {
  String command = server.arg("command");
  
  if (command == "forward") {
    moveForward(normal_speed);  // ~6V for forward
    Serial.println("..................forward............");
  } else if (command == "fast_forward") {
    moveForward(max_speed);  // ~9V for fast forward
    Serial.println("..................fast forward............");
  } else if (command == "backward") {
    moveBackward(normal_speed);  // ~6V for backward
    Serial.println("..................backward............");
  } else if (command == "fast_backward") {
    moveBackward(max_speed);  // ~9V for fast backward
    Serial.println("..................fast backward............");
  } else if (command == "right") {
    turnRight();
    Serial.println("..................right............");
  } else if (command == "left") {
    turnLeft();
    Serial.println("..................left............");
  } else if (command == "fast_right") {
    fastTurnRight();
    Serial.println("..................Fast right............");
  } else if (command == "fast_left") {
    fastTurnLeft();
    Serial.println("..................Fast left............");
  } else if (command == "stop") {
    stopMotors();
    Serial.println("..................stop............");
  }
  server.send(200, "text/plain", "Command received: " + command);
}

void loop() {
  // Handle client requests
  server.handleClient();
}
