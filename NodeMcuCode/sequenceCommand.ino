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

int commandCountGlobal = 0;
String commands[10]; // Array to hold command actions
int commandLength[10]; // Array to hold command lengths
int index = 0;

// Function to handle WebSocket messages
void handleWebSocketMessage(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    String message = (char*)payload;
    Serial.println("Received message: " + message);

    // Reset command count and index for new messages
    int commandCount = 0;
    if (message != "stop") {
        for (int i = 0; i < message.length(); i += 3) {
            commands[commandCount] = String(message[i]); // Store the command action
            commandLength[commandCount] = message.substring(i + 1, i + 3).toInt(); // Convert the 2-digit length to integer
            commandCount++;
        }
        commandCountGlobal = commandCount;
        index = 0;
    }

    // Execute each command sequentially
    if (index < commandCountGlobal) {
        String direction = commands[index];
        int &length = commandLength[index]; // Use reference to directly modify length

        if (length > 0) {
            if (direction == "f") {
                forward();
                Serial.println("..................forward...............");
            } else if (direction == "b") {
                backward();
                Serial.println("..................backward..............");
            } else if (direction == "l") {
                left();
                Serial.println("..................left..................");
            } else if (direction == "r") {
                right();
                Serial.println("..................right.................");
            } else if (direction == "s") {
                stopMotors();
                Serial.println("..................stop..................");
            }
            length--; // Decrement the remaining length for this command
        } else {
            index++; // Move to the next command when current length reaches zero
        }
    }else{
      stopMotors();
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
