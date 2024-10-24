#include <ESP8266WiFi.h>
#include <WebSocketsServer.h>
#include <ESP8266WebServer.h>

// Motor control pins
const int motorLeft1 = D1;
const int motorLeft2 = D2;
const int motorRight1 = D3;
const int motorRight2 = D4;
const int enablePinLeft = D5;
const int enablePinRight = D6;

const int max_speed = 160;  //assuming 8.5V
const int normal_speed = 130; // assuming 7.5V

// WebSocket server
WebSocketsServer webSocket = WebSocketsServer(81);

// WiFi credentials
const char* ssid = "#r";
const char* password = "qwju8977";

// Function to handle WebSocket messages
void handleWebSocketMessage(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  String message = (char*)payload;
  Serial.println("Received message: " + message);

  // Split the entire path pattern by commas (",")
  String commands[10]; // Adjust array size based on maximum expected commands
  int commandCount = 0;
  
  while (message.length() > 0) {
    int index = message.indexOf(',');
    if (index == -1) {
      commands[commandCount++] = message;
      break;
    } else {
      commands[commandCount++] = message.substring(0, index);
      message = message.substring(index + 1);
    }
  }

  // Execute each command sequentially
  for (int i = 0; i < commandCount; i++) {
    String command = commands[i];
    command.trim();
    
    int spaceIndex = command.indexOf(' ');
    if (spaceIndex != -1) {
      String direction = command.substring(0, spaceIndex);
      int duration = command.substring(spaceIndex + 1).toInt();

      if (direction == "forward") {
        forward();
        Serial.println("..................forward...............");
      } else if (direction == "fast_forward") {
        fastForward();
        Serial.println("..................fast forward..........");
      } else if (direction == "backward") {
        backward();
        Serial.println("..................backward..............");
      } else if (direction == "fast_backward") {
        fastBackward();
        Serial.println("..................fast backward.........");
      } else if (direction == "left") {
        left();
        Serial.println("..................left..................");
      } else if (direction == "right") {
        right();
        Serial.println("..................right.................");
      } else if (direction == "fast_left") {
        fastLeft();
        Serial.println("..................Fast left.............");
      } else if (direction == "fast_right") {
        fastRight();
        Serial.println("..................Fast right............");
      } else if (direction == "stop") {
        stopMotors();
        Serial.println("..................stop..................");
      }

      delay(duration); // Run the command for the given duration
      Serial.print("......................");
      Serial.print(duration);
      Serial.println("......................");
      stopMotors(); // Stop the motors after the duration
    }
  }
}

// Motor control functions (same as before)
void forward() {
  analogWrite(enablePinLeft, normal_speed); 
  analogWrite(enablePinRight, normal_speed);
  digitalWrite(motorLeft1, HIGH);
  digitalWrite(motorLeft2, LOW);
  digitalWrite(motorRight1, HIGH);
  digitalWrite(motorRight2, LOW);
}

void fastForward() {
  analogWrite(enablePinLeft, max_speed);
  analogWrite(enablePinRight, max_speed);
  digitalWrite(motorLeft1, HIGH);
  digitalWrite(motorLeft2, LOW);
  digitalWrite(motorRight1, HIGH);
  digitalWrite(motorRight2, LOW);
}

void backward() {
  analogWrite(enablePinLeft, normal_speed); 
  analogWrite(enablePinRight, normal_speed);
  digitalWrite(motorLeft1, LOW);
  digitalWrite(motorLeft2, HIGH);
  digitalWrite(motorRight1, LOW);
  digitalWrite(motorRight2, HIGH);
}

void fastBackward() {
  analogWrite(enablePinLeft, max_speed); 
  analogWrite(enablePinRight, max_speed);
  digitalWrite(motorLeft1, LOW);
  digitalWrite(motorLeft2, HIGH);
  digitalWrite(motorRight1, LOW);
  digitalWrite(motorRight2, HIGH);
}

void left() {
  analogWrite(enablePinLeft, normal_speed);
  digitalWrite(motorLeft1, HIGH);
  digitalWrite(motorLeft2, LOW);
  stopRightMotor();
}

void right() {
  analogWrite(enablePinRight, normal_speed);
  digitalWrite(motorRight1, HIGH);
  digitalWrite(motorRight2, LOW);
  stopLeftMotor();
}

void fastLeft() {
  analogWrite(enablePinLeft, normal_speed);
  analogWrite(enablePinRight, normal_speed);
  digitalWrite(motorLeft1, HIGH);
  digitalWrite(motorLeft2, LOW);
  digitalWrite(motorRight1, LOW);
  digitalWrite(motorRight2, HIGH);
}

void fastRight() {
  analogWrite(enablePinRight, normal_speed);
  analogWrite(enablePinLeft, normal_speed);
  digitalWrite(motorRight1, HIGH);
  digitalWrite(motorRight2, LOW);
  digitalWrite(motorLeft1, LOW);
  digitalWrite(motorLeft2, HIGH);
}

void stopMotors() {
  analogWrite(enablePinLeft, 0);
  analogWrite(enablePinRight, 0);
}

void stopLeftMotor() {
  analogWrite(enablePinLeft, 0);
}

void stopRightMotor() {
  analogWrite(enablePinRight, 0);
}

void setup() {
  Serial.begin(115200);
  
  // Initialize motor pins
  pinMode(motorLeft1, OUTPUT);
  pinMode(motorLeft2, OUTPUT);
  pinMode(motorRight1, OUTPUT);
  pinMode(motorRight2, OUTPUT);
  pinMode(enablePinLeft, OUTPUT);
  pinMode(enablePinRight, OUTPUT);
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi");

  // Start WebSocket server
  webSocket.begin();
  webSocket.onEvent(handleWebSocketMessage);

  Serial.print("Robot IP: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  // Handle WebSocket connection
  webSocket.loop();
}
