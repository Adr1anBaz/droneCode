/*
  Drone PCB Test Firmware - ESP32-S3 Xiao Super Mini

  Motor Configuration (Cross/X layout, top view):

        Front
    M1 (CW)    M2 (CCW)
        \      /
         \    /
          \/
          /\
         /  \
    M3 (CCW)   M4 (CW)
        Back

  Motor 1 (D10) - Front Left  - Clockwise      - Red/Blue wires  - Propeller Type A
  Motor 2 (D9)  - Front Right - Counter-CW     - White/Black wires - Propeller Type B
  Motor 3 (D5)  - Back Left   - Counter-CW     - White/Black wires - Propeller Type B
  Motor 4 (D6)  - Back Right  - Clockwise      - Red/Blue wires  - Propeller Type A

  CW motors (1,4): Red/Blue wires, Type A propellers (pusher)
  CCW motors (2,3): White/Black wires, Type B propellers (puller)

  WiFi: AP mode - SSID "DroneTest" / Password "drone1234"
  Control: HTTP server on 192.168.4.1:80
  PWM: expressed in percentage (0-100%)
*/

#include <WiFi.h>
#include <WebServer.h>

// --- Pin definitions ---
#define MOTOR1_PIN D10
#define MOTOR2_PIN D9
#define MOTOR3_PIN D5
#define MOTOR4_PIN D6

// --- PWM configuration ---
#define PWM_FREQ 20000
#define PWM_RESOLUTION 8
#define PWM_MAX 255

// --- WiFi AP credentials ---
const char* AP_SSID = "DroneTest";
const char* AP_PASS = "drone1234";

WebServer server(80);

float motor_pwm[4] = {0, 0, 0, 0};

uint8_t percentToDuty(float percent) {
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;
  return (uint8_t)((percent / 100.0) * PWM_MAX);
}

void handleRoot() {
  String html = "<html><body><h1>Drone PCB Test</h1>";
  html += "<p>M1: " + String(motor_pwm[0], 1) + "%</p>";
  html += "<p>M2: " + String(motor_pwm[1], 1) + "%</p>";
  html += "<p>M3: " + String(motor_pwm[2], 1) + "%</p>";
  html += "<p>M4: " + String(motor_pwm[3], 1) + "%</p>";
  html += "<p>POST to /motor with JSON: {\"motor\": 1-4, \"pwm\": 0-100}</p>";
  html += "<p>POST to /all with JSON: {\"pwm\": 0-100}</p>";
  html += "<p>POST to /stop to stop all motors</p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleSetMotor() {
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"error\":\"no body\"}");
    return;
  }

  String body = server.arg("plain");

  int motorIdx = -1;
  float pwmVal = -1;

  int mPos = body.indexOf("\"motor\"");
  if (mPos >= 0) {
    int colon = body.indexOf(':', mPos);
    motorIdx = body.substring(colon + 1).toInt();
  }

  int pPos = body.indexOf("\"pwm\"");
  if (pPos >= 0) {
    int colon = body.indexOf(':', pPos);
    pwmVal = body.substring(colon + 1).toFloat();
  }

  if (motorIdx < 1 || motorIdx > 4 || pwmVal < 0 || pwmVal > 100) {
    server.send(400, "application/json", "{\"error\":\"invalid params. motor:1-4, pwm:0-100\"}");
    return;
  }

  motor_pwm[motorIdx - 1] = pwmVal;
  applyPWM(motorIdx - 1);

  String resp = "{\"motor\":" + String(motorIdx) + ",\"pwm\":" + String(pwmVal, 1) + "}";
  server.send(200, "application/json", resp);
}

void handleSetAll() {
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"error\":\"no body\"}");
    return;
  }

  String body = server.arg("plain");
  float pwmVal = -1;

  int pPos = body.indexOf("\"pwm\"");
  if (pPos >= 0) {
    int colon = body.indexOf(':', pPos);
    pwmVal = body.substring(colon + 1).toFloat();
  }

  if (pwmVal < 0 || pwmVal > 100) {
    server.send(400, "application/json", "{\"error\":\"invalid pwm (0-100)\"}");
    return;
  }

  for (int i = 0; i < 4; i++) {
    motor_pwm[i] = pwmVal;
    applyPWM(i);
  }

  String resp = "{\"all_pwm\":" + String(pwmVal, 1) + "}";
  server.send(200, "application/json", resp);
}

void handleStop() {
  for (int i = 0; i < 4; i++) {
    motor_pwm[i] = 0;
    applyPWM(i);
  }
  server.send(200, "application/json", "{\"status\":\"all motors stopped\"}");
}

void handleStatus() {
  String resp = "{\"motors\":[";
  for (int i = 0; i < 4; i++) {
    resp += String(motor_pwm[i], 1);
    if (i < 3) resp += ",";
  }
  resp += "]}";
  server.send(200, "application/json", resp);
}

void applyPWM(int motorIndex) {
  uint8_t duty = percentToDuty(motor_pwm[motorIndex]);
  switch (motorIndex) {
    case 0: analogWrite(MOTOR1_PIN, duty); break;
    case 1: analogWrite(MOTOR2_PIN, duty); break;
    case 2: analogWrite(MOTOR3_PIN, duty); break;
    case 3: analogWrite(MOTOR4_PIN, duty); break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== Drone PCB Test Firmware ===");

  // Configure motor pins as output
  pinMode(MOTOR1_PIN, OUTPUT);
  pinMode(MOTOR2_PIN, OUTPUT);
  pinMode(MOTOR3_PIN, OUTPUT);
  pinMode(MOTOR4_PIN, OUTPUT);

  // Start with all motors off
  analogWrite(MOTOR1_PIN, 0);
  analogWrite(MOTOR2_PIN, 0);
  analogWrite(MOTOR3_PIN, 0);
  analogWrite(MOTOR4_PIN, 0);

  // Set up WiFi Access Point
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(500);

  Serial.print("AP SSID: ");
  Serial.println(AP_SSID);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  // Set up HTTP routes
  server.on("/", HTTP_GET, handleRoot);
  server.on("/motor", HTTP_POST, handleSetMotor);
  server.on("/all", HTTP_POST, handleSetAll);
  server.on("/stop", HTTP_POST, handleStop);
  server.on("/status", HTTP_GET, handleStatus);
  server.begin();

  Serial.println("HTTP server started on port 80");
  Serial.println("Ready for testing.");
}

void loop() {
  server.handleClient();
}
