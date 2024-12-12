#include <ESP8266WiFi.h>
#include <WebSocketsServer.h>
#include <ESP8266WebServer.h>
#include <Servo.h>

// Motor control pins
const int motorLeft1 = D1;
const int motorLeft2 = D2;
const int motorRight1 = D7; //just changed
const int motorRight2 = D8; //just changed
const int enablePinLeft = D5;
const int enablePinRight = D6;

Servo servo;
const int servoPin = D4;

int loop_scale_high = 5;
int loop_scale = 3;

const int max_speed = 170;  //assuming 8.5V
const int normal_speed = 130; // assuming 7.5V

const int left_normal_speed = 150;
const int left_max_speed = 170;

// WebSocket server
WebSocketsServer webSocket = WebSocketsServer(81);

// WiFi credentials
const char* ssid = "Gojo";
const char* password = "jasiratp";

int commandCountGlobal = 0;
String commands[10]; // Array to hold command actions
long commandLength[10]; // Array to hold command lengths
int commandIndex = 0;

// Function to handle WebSocket messages
void handleWebSocketMessage(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    String message = (char*)payload;
    Serial.println("Received message: " + message);

    // Reset command count and commandIndex for new messages
    int commandCount = 0;

    if (message.startsWith("set_loop_scale:")) {
        // Extract the new loop_scale value
        String valueStr = message.substring(15); // Remove "set_loop_scale:"
        int newLoopScale = valueStr.toInt();
        if (newLoopScale > 0) {
            loop_scale_high = newLoopScale;
            Serial.println("Updated loop_scale to: " + String(loop_scale));
            // String message = "loop_scale updated to " + String(loop_scale);
            webSocket.sendTXT(0, "updated loop_scale");
            //webSocket.sendTXT(0, message.c_str());
        } else {
            webSocket.sendTXT(0, "Invalid loop_scale value.");
        }
    }else if(message.startsWith("rot_scale:")){
      // Extract the new loop_scale value
        String valueStr = message.substring(10); // Remove "set_loop_scale:"
        int newLoopScale = valueStr.toInt();
        if (newLoopScale > 0) {
            loop_scale = newLoopScale;
            Serial.println("Updated loop_scale to: " + String(loop_scale));
            // String message = "loop_scale updated to " + String(loop_scale);
            webSocket.sendTXT(0, "updated loop_scale");
            //webSocket.sendTXT(0, message.c_str());
        } else {
            webSocket.sendTXT(0, "Invalid loop_scale value.");
        }
    }
    //  else if (message == "get_status") {
    //     // Send current status back
    //     String status = "loop_scale:" + String(loop_scale) +
    //                     ", max_speed:" + String(max_speed) +
    //                     ", normal_speed:" + String(normal_speed) +
    //                     ", left_max_speed:" + String(left_max_speed) +
    //                     ", left_normal_speed:" + String(left_normal_speed);
    //     webSocket.sendTXT(0, "responded");
    //     webSocket.sendTXT(0, status.c_str());
    // } 
    else if (message == "k") {
      kick();
      commandCountGlobal = 0;
    } else if (message == "stop") {
      stopMotors();
      commandCountGlobal = 0;
    } else if (message != "stop" && message != "") {
        for (int i = 0; i < message.length(); i += 3) {
            commands[commandCount] = String(message[i]); // Store the command action
            String hexaCount = message.substring(i + 1, i + 3);
            if(commands[commandCount] =="f" || commands[commandCount] =="b"){
              commandLength[commandCount] = strtol(hexaCount.c_str(), nullptr, 16) * loop_scale_high; // Convert the 2-digit length to integer
            }
            else if(commands[commandCount] =="B" || commands[commandCount] =="F"){
              commandLength[commandCount] = strtol(hexaCount.c_str(), nullptr, 16) * 5; // Convert the 2-digit length to integer
            }
            else{
              commandLength[commandCount] = strtol(hexaCount.c_str(), nullptr, 16) * loop_scale; // Convert the 2-digit length to integer
            }
            Serial.println(commandLength[commandCount]);
            
            commandCount++;
        }
        commandCountGlobal = commandCount;
        commandIndex = 0;
    }
}

void commandExecute() {
    // Execute each command sequentially
    if (commandIndex < commandCountGlobal) {
        String direction = commands[commandIndex];
        long &length = commandLength[commandIndex]; // Use reference to directly modify length
        String stringLength =String(length);

        if (length > 0) {
            if (direction == "f") {
                forward();
                Serial.println("..................forward..............."+stringLength);
            } else if (direction == "F") {
                fastForward();
                Serial.println("..................fast forward.............."+stringLength);
            } else if (direction == "b") {
                backward();
                Serial.println("..................backward.............."+stringLength);
            } else if (direction == "B") {
                fastBackward();
                Serial.println("..................fast backward.............."+stringLength);
            } else if (direction == "l") {
                left();
                Serial.println("..................left.................."+stringLength);
            } else if (direction == "L") {
                fastLeft();
                Serial.println("..................fast_left.................."+stringLength);
            } else if (direction == "R") {
                fastRight();
                Serial.println("..................fast_right.................."+stringLength);
            } else if (direction == "r") {
                right();
                Serial.println("..................right................."+stringLength);
            } else if (direction == "s") {
                stopMotors();
                Serial.println("..................stop.................."+stringLength);
            }else if (direction == "S") {
                slowStopMotor();
                Serial.println("..................stop.................."+stringLength);
            }
            length--; // Decrement the remaining length for this command
        } else {
            commandIndex++; // Move to the next command when current length reaches zero
        }
    } else {
        stopMotors();
        // Send "completed" message back to the connected client
        // Serial.println("sequence completed");
        webSocket.sendTXT(0, "completed");
    }
}

// Motor control functions (modified to use left_normal_speed for the left motor)
void forward() {
  analogWrite(enablePinLeft, left_normal_speed); 
  analogWrite(enablePinRight, normal_speed);
  digitalWrite(motorLeft1, HIGH);
  digitalWrite(motorLeft2, LOW);
  digitalWrite(motorRight1, HIGH);
  digitalWrite(motorRight2, LOW);
}

void fastForward() {
  analogWrite(enablePinLeft, left_max_speed);
  analogWrite(enablePinRight, max_speed);
  digitalWrite(motorLeft1, HIGH);
  digitalWrite(motorLeft2, LOW);
  digitalWrite(motorRight1, HIGH);
  digitalWrite(motorRight2, LOW);
}

void backward() {
  analogWrite(enablePinLeft, left_normal_speed); 
  analogWrite(enablePinRight, normal_speed);
  digitalWrite(motorLeft1, LOW);
  digitalWrite(motorLeft2, HIGH);
  digitalWrite(motorRight1, LOW);
  digitalWrite(motorRight2, HIGH);
}

void fastBackward() {
  analogWrite(enablePinLeft, left_max_speed); 
  analogWrite(enablePinRight, max_speed);
  digitalWrite(motorLeft1, LOW);
  digitalWrite(motorLeft2, HIGH);
  digitalWrite(motorRight1, LOW);
  digitalWrite(motorRight2, HIGH);
}

void left() {
  analogWrite(enablePinLeft, left_normal_speed);
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
  analogWrite(enablePinLeft, left_normal_speed);
  analogWrite(enablePinRight, normal_speed);
  digitalWrite(motorLeft1, HIGH);
  digitalWrite(motorLeft2, LOW);
  digitalWrite(motorRight1, LOW);
  digitalWrite(motorRight2, HIGH);
}

void fastRight() {
  analogWrite(enablePinRight, normal_speed);
  analogWrite(enablePinLeft, left_normal_speed);
  digitalWrite(motorRight1, HIGH);
  digitalWrite(motorRight2, LOW);
  digitalWrite(motorLeft1, LOW);
  digitalWrite(motorLeft2, HIGH);
}

void stopMotors() {
  analogWrite(enablePinLeft, 0);
  analogWrite(enablePinRight, 0);
  servo.write(0);   // returning to initial position, make sure at 0 degree.
}

// Slow stop motor function
void slowStopMotor() {
    int leftSpeed = analogRead(enablePinLeft); // Current speed of the left motor
    int rightSpeed = analogRead(enablePinRight); // Current speed of the right motor
    int step = 10; // Speed reduction step
    int delayTime = 50; // Delay between each step (in milliseconds)

    while (leftSpeed > 0 || rightSpeed > 0) {
        if (leftSpeed > 0) {
            leftSpeed -= step;
            if (leftSpeed < 0) leftSpeed = 0; // Prevent negative values
            analogWrite(enablePinLeft, leftSpeed);
        }
        if (rightSpeed > 0) {
            rightSpeed -= step;
            if (rightSpeed < 0) rightSpeed = 0; // Prevent negative values
            analogWrite(enablePinRight, rightSpeed);
        }
        delay(delayTime);
    }

    // Ensure motors are completely stopped
    analogWrite(enablePinLeft, 0);
    analogWrite(enablePinRight, 0);

    Serial.println("Motors stopped slowly.");
}

void stopLeftMotor() {
  analogWrite(enablePinLeft, 0);
}

void stopRightMotor() {
  analogWrite(enablePinRight, 0);
}

void kick() {
  servo.write(180);
  delay(340);
  servo.write(0);
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

  servo.attach(servoPin);
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println(".");
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
  commandExecute();
}
