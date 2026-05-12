/*
  Drone Orientation Control Firmware - ESP32-S3 Xiao Super Mini
  Mounted on GyroTuner gimbal (3-axis free rotation, no flight)

  SAFETY:
    - Max PWM: 50%
    - Rate limiter: max +0.5% per control cycle (very slow ramp)
    - Pre-spin: all motors go to 5% before PID starts
    - All PWM changes logged via Serial

  Motor Config (top view):
    M1 (D10) - Front Left  - CCW - Black/White - Prop B
    M2 (D9)  - Front Right - CW  - Red/Blue    - Prop A
    M3 (D8)  - Back Left   - CW  - Red/Blue    - Prop A
    M4 (D6)  - Back Right  - CCW - Black/White - Prop B

  IMU: MPU6050 via I2C
    SDA = D5 (PA8_A4)
    SCL = D4 (PA11_A3)

  WiFi: AP mode - "DroneControl" / "drone1234"
  Control: WebSocket on 192.168.4.1:81, HTTP on :80
*/

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Wire.h>
#include <math.h>

// --- Pin definitions ---
#define MOTOR1_PIN D10
#define MOTOR2_PIN D9
#define MOTOR3_PIN D8
#define MOTOR4_PIN D6

#define I2C_SDA D5
#define I2C_SCL D4

// --- PWM limits ---
#define PWM_MIN 0.0
#define PWM_MAX 50.0
#define PWM_SPIN_MIN 5.0
#define PWM_RAMP_RATE 0.5    // max % change per control cycle (250Hz -> takes 90 cycles = 0.36s per 1%)
#define PWM_RESOLUTION 8
#define PWM_DUTY_MAX 255

// --- WiFi ---
const char* AP_SSID = "DroneControl";
const char* AP_PASS = "drone1234";

WebServer server(80);
WebSocketsServer ws(81);

// --- MPU6050 ---
#define MPU_ADDR 0x68

float gyro_offset[3] = {0, 0, 0};
float accel_offset[3] = {0, 0, 0};
bool calibrated = false;

// --- Orientation state ---
float angle_roll = 0;
float angle_pitch = 0;
float angle_yaw = 0;

float target_roll = 0;
float target_pitch = 0;
float target_yaw = 0;

// --- PID ---
struct PID {
  float kp, ki, kd;
  float integral;
  float prev_error;
  float output;
};

PID pid_roll  = {1.0, 0.01, 0.5, 0, 0, 0};
PID pid_pitch = {1.0, 0.01, 0.5, 0, 0, 0};
PID pid_yaw   = {1.5, 0.02, 0.7, 0, 0, 0};

float throttle_base = 10.0;

// --- Motor PWM values ---
float motor_pwm_target[4] = {0, 0, 0, 0};  // what PID wants
float motor_pwm_actual[4] = {0, 0, 0, 0};  // what is actually applied (rate limited)

// --- Timing ---
unsigned long last_loop_time = 0;
unsigned long last_ws_send = 0;
unsigned long last_serial_log = 0;
#define LOOP_INTERVAL_US 4000   // 250Hz control loop
#define WS_SEND_INTERVAL 100    // 100ms telemetry
#define SERIAL_LOG_INTERVAL 200 // 200ms serial log

// --- State machine ---
enum State {
  STATE_IDLE,
  STATE_CALIBRATING,
  STATE_PRE_SPIN,      // ramping all motors to 5% before PID
  STATE_RUNNING,
  STATE_MANUAL_TEST    // all motors at 5% for testing
};
State current_state = STATE_IDLE;
int calibration_samples = 0;
#define CALIBRATION_COUNT 500

unsigned long pre_spin_start = 0;
#define PRE_SPIN_DURATION 3000  // 3 seconds at 5% before PID activates

// --- Rate limiter ---
float rate_limit(float current, float target, float max_change) {
  float diff = target - current;
  if (diff > max_change) diff = max_change;
  if (diff < -max_change) diff = -max_change;
  return current + diff;
}

// --- MPU6050 functions ---
void mpu_write(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void mpu_init() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  delay(100);
  mpu_write(0x6B, 0x00); // wake up
  mpu_write(0x1B, 0x08); // gyro +/- 500 deg/s
  mpu_write(0x1C, 0x08); // accel +/- 4g
  mpu_write(0x1A, 0x03); // DLPF 44Hz
}

void mpu_read(float* ax, float* ay, float* az, float* gx, float* gy, float* gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14);

  int16_t raw_ax = (Wire.read() << 8) | Wire.read();
  int16_t raw_ay = (Wire.read() << 8) | Wire.read();
  int16_t raw_az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // temp
  int16_t raw_gx = (Wire.read() << 8) | Wire.read();
  int16_t raw_gy = (Wire.read() << 8) | Wire.read();
  int16_t raw_gz = (Wire.read() << 8) | Wire.read();

  *ax = raw_ax / 8192.0;
  *ay = raw_ay / 8192.0;
  *az = raw_az / 8192.0;
  *gx = raw_gx / 65.5;
  *gy = raw_gy / 65.5;
  *gz = raw_gz / 65.5;
}

// --- Calibration ---
void calibration_step() {
  float ax, ay, az, gx, gy, gz;
  mpu_read(&ax, &ay, &az, &gx, &gy, &gz);

  gyro_offset[0] += gx;
  gyro_offset[1] += gy;
  gyro_offset[2] += gz;
  accel_offset[0] += ax;
  accel_offset[1] += ay;
  accel_offset[2] += (az - 1.0);

  calibration_samples++;

  if (calibration_samples >= CALIBRATION_COUNT) {
    for (int i = 0; i < 3; i++) {
      gyro_offset[i] /= CALIBRATION_COUNT;
      accel_offset[i] /= CALIBRATION_COUNT;
    }
    calibrated = true;
    current_state = STATE_IDLE;

    angle_roll = 0;
    angle_pitch = 0;
    angle_yaw = 0;
    target_roll = 0;
    target_pitch = 0;
    target_yaw = 0;

    String msg = "{\"event\":\"calibrated\",\"offsets\":{\"gx\":" + String(gyro_offset[0], 4) +
                 ",\"gy\":" + String(gyro_offset[1], 4) +
                 ",\"gz\":" + String(gyro_offset[2], 4) + "}}";
    ws.broadcastTXT(msg);
    Serial.println("[CAL] Calibration complete");
  }
}

// --- Complementary filter ---
void update_angles(float dt) {
  float ax, ay, az, gx, gy, gz;
  mpu_read(&ax, &ay, &az, &gx, &gy, &gz);

  gx -= gyro_offset[0];
  gy -= gyro_offset[1];
  gz -= gyro_offset[2];
  ax -= accel_offset[0];
  ay -= accel_offset[1];
  az -= accel_offset[2];
  az += 1.0;

  float accel_roll  = atan2(ay, az) * 180.0 / M_PI;
  float accel_pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;

  angle_roll  = 0.98 * (angle_roll  + gx * dt) + 0.02 * accel_roll;
  angle_pitch = 0.98 * (angle_pitch + gy * dt) + 0.02 * accel_pitch;
  angle_yaw  += gz * dt;
}

// --- PID compute ---
float compute_pid(PID* p, float error, float dt) {
  p->integral += error * dt;
  if (p->integral > 20) p->integral = 20;
  if (p->integral < -20) p->integral = -20;

  float derivative = (error - p->prev_error) / dt;
  p->prev_error = error;

  p->output = p->kp * error + p->ki * p->integral + p->kd * derivative;
  return p->output;
}

// --- Motor mixing ---
void compute_motors() {
  float dt = LOOP_INTERVAL_US / 1000000.0;

  float err_roll  = target_roll  - angle_roll;
  float err_pitch = target_pitch - angle_pitch;
  float err_yaw   = target_yaw  - angle_yaw;

  float out_roll  = compute_pid(&pid_roll,  err_roll,  dt);
  float out_pitch = compute_pid(&pid_pitch, err_pitch, dt);
  float out_yaw   = compute_pid(&pid_yaw,   err_yaw,   dt);

  // Clamp PID output to prevent spikes
  float pid_max = 15.0;
  if (out_roll > pid_max) out_roll = pid_max;
  if (out_roll < -pid_max) out_roll = -pid_max;
  if (out_pitch > pid_max) out_pitch = pid_max;
  if (out_pitch < -pid_max) out_pitch = -pid_max;
  if (out_yaw > pid_max) out_yaw = pid_max;
  if (out_yaw < -pid_max) out_yaw = -pid_max;

  // Motor mixing for X config:
  // M1 (FL, CCW): throttle - roll + pitch + yaw
  // M2 (FR, CW):  throttle + roll + pitch - yaw
  // M3 (BL, CW):  throttle - roll - pitch - yaw
  // M4 (BR, CCW): throttle + roll - pitch + yaw
  motor_pwm_target[0] = throttle_base - out_roll + out_pitch + out_yaw;
  motor_pwm_target[1] = throttle_base + out_roll + out_pitch - out_yaw;
  motor_pwm_target[2] = throttle_base - out_roll - out_pitch - out_yaw;
  motor_pwm_target[3] = throttle_base + out_roll - out_pitch + out_yaw;

  // Hard clamp targets
  for (int i = 0; i < 4; i++) {
    if (motor_pwm_target[i] < PWM_MIN) motor_pwm_target[i] = PWM_MIN;
    if (motor_pwm_target[i] > PWM_MAX) motor_pwm_target[i] = PWM_MAX;
  }
}

// Apply rate-limited PWM to motors
void apply_motors() {
  for (int i = 0; i < 4; i++) {
    motor_pwm_actual[i] = rate_limit(motor_pwm_actual[i], motor_pwm_target[i], PWM_RAMP_RATE);
    uint8_t duty = (uint8_t)((motor_pwm_actual[i] / 100.0) * PWM_DUTY_MAX);
    switch (i) {
      case 0: analogWrite(MOTOR1_PIN, duty); break;
      case 1: analogWrite(MOTOR2_PIN, duty); break;
      case 2: analogWrite(MOTOR3_PIN, duty); break;
      case 3: analogWrite(MOTOR4_PIN, duty); break;
    }
  }
}

void stop_motors() {
  for (int i = 0; i < 4; i++) {
    motor_pwm_target[i] = 0;
    motor_pwm_actual[i] = 0;
  }
  analogWrite(MOTOR1_PIN, 0);
  analogWrite(MOTOR2_PIN, 0);
  analogWrite(MOTOR3_PIN, 0);
  analogWrite(MOTOR4_PIN, 0);
  Serial.println("[MOT] All motors OFF");
}

// --- Serial logging ---
void serial_log() {
  Serial.print("[PWM] T:");
  Serial.print(motor_pwm_target[0], 1); Serial.print(",");
  Serial.print(motor_pwm_target[1], 1); Serial.print(",");
  Serial.print(motor_pwm_target[2], 1); Serial.print(",");
  Serial.print(motor_pwm_target[3], 1);
  Serial.print(" A:");
  Serial.print(motor_pwm_actual[0], 1); Serial.print(",");
  Serial.print(motor_pwm_actual[1], 1); Serial.print(",");
  Serial.print(motor_pwm_actual[2], 1); Serial.print(",");
  Serial.print(motor_pwm_actual[3], 1);
  Serial.print(" ANG:");
  Serial.print(angle_roll, 1); Serial.print(",");
  Serial.print(angle_pitch, 1); Serial.print(",");
  Serial.println(angle_yaw, 1);
}

// --- WebSocket handler ---
void ws_event(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_TEXT) {
    String msg = String((char*)payload);

    if (msg == "calibrate") {
      stop_motors();
      calibrated = false;
      calibration_samples = 0;
      gyro_offset[0] = gyro_offset[1] = gyro_offset[2] = 0;
      accel_offset[0] = accel_offset[1] = accel_offset[2] = 0;
      current_state = STATE_CALIBRATING;
      ws.sendTXT(num, "{\"event\":\"calibrating\",\"samples\":500}");
      Serial.println("[CAL] Starting calibration...");
    }
    else if (msg == "start") {
      if (!calibrated) {
        ws.sendTXT(num, "{\"error\":\"not calibrated\"}");
        return;
      }
      // Reset PID state
      pid_roll.integral = 0; pid_roll.prev_error = 0;
      pid_pitch.integral = 0; pid_pitch.prev_error = 0;
      pid_yaw.integral = 0; pid_yaw.prev_error = 0;
      // Go to pre-spin first
      current_state = STATE_PRE_SPIN;
      pre_spin_start = millis();
      for (int i = 0; i < 4; i++) motor_pwm_target[i] = PWM_SPIN_MIN;
      ws.sendTXT(num, "{\"event\":\"pre_spin\",\"duration_ms\":3000}");
      Serial.println("[CTL] Pre-spin: ramping to 5% on all motors...");
    }
    else if (msg == "stop") {
      current_state = STATE_IDLE;
      stop_motors();
      ws.sendTXT(num, "{\"event\":\"stopped\"}");
    }
    else if (msg == "test") {
      // Manual test: all motors at 5%
      current_state = STATE_MANUAL_TEST;
      for (int i = 0; i < 4; i++) motor_pwm_target[i] = PWM_SPIN_MIN;
      ws.sendTXT(num, "{\"event\":\"manual_test\",\"pwm\":5}");
      Serial.println("[TEST] Manual test: all motors -> 5%");
    }
    else if (msg == "test_stop") {
      current_state = STATE_IDLE;
      stop_motors();
      ws.sendTXT(num, "{\"event\":\"test_stopped\"}");
    }
    else if (msg.startsWith("target:")) {
      String vals = msg.substring(7);
      int c1 = vals.indexOf(',');
      int c2 = vals.indexOf(',', c1 + 1);
      target_roll  = vals.substring(0, c1).toFloat();
      target_pitch = vals.substring(c1 + 1, c2).toFloat();
      target_yaw   = vals.substring(c2 + 1).toFloat();
      ws.sendTXT(num, "{\"event\":\"target_set\",\"roll\":" + String(target_roll, 1) +
                 ",\"pitch\":" + String(target_pitch, 1) + ",\"yaw\":" + String(target_yaw, 1) + "}");
      Serial.print("[TGT] New target: ");
      Serial.print(target_roll, 1); Serial.print(",");
      Serial.print(target_pitch, 1); Serial.print(",");
      Serial.println(target_yaw, 1);
    }
    else if (msg.startsWith("throttle:")) {
      throttle_base = msg.substring(9).toFloat();
      if (throttle_base < 0) throttle_base = 0;
      if (throttle_base > PWM_MAX) throttle_base = PWM_MAX;
      ws.sendTXT(num, "{\"event\":\"throttle_set\",\"value\":" + String(throttle_base, 1) + "}");
      Serial.print("[THR] Throttle: "); Serial.println(throttle_base, 1);
    }
    else if (msg.startsWith("pid:")) {
      String vals = msg.substring(4);
      int c1 = vals.indexOf(',');
      int c2 = vals.indexOf(',', c1 + 1);
      int c3 = vals.indexOf(',', c2 + 1);
      String axis = vals.substring(0, c1);
      float kp = vals.substring(c1 + 1, c2).toFloat();
      float ki = vals.substring(c2 + 1, c3).toFloat();
      float kd = vals.substring(c3 + 1).toFloat();

      PID* target_pid = NULL;
      if (axis == "roll") target_pid = &pid_roll;
      else if (axis == "pitch") target_pid = &pid_pitch;
      else if (axis == "yaw") target_pid = &pid_yaw;

      if (target_pid) {
        target_pid->kp = kp;
        target_pid->ki = ki;
        target_pid->kd = kd;
        target_pid->integral = 0;
        ws.sendTXT(num, "{\"event\":\"pid_set\",\"axis\":\"" + axis + "\"}");
        Serial.print("[PID] "); Serial.print(axis);
        Serial.print(" Kp="); Serial.print(kp, 3);
        Serial.print(" Ki="); Serial.print(ki, 3);
        Serial.print(" Kd="); Serial.println(kd, 3);
      }
    }
    else if (msg.startsWith("ramp:")) {
      float new_ramp = msg.substring(5).toFloat();
      if (new_ramp > 0 && new_ramp <= 5.0) {
        // Can't modify #define, so we won't use this in this version
        ws.sendTXT(num, "{\"event\":\"ramp_info\",\"note\":\"ramp is fixed at 0.5%/cycle\"}");
      }
    }
  }
}

// --- HTTP ---
void handleRoot() {
  String html = "<html><body><h1>Drone Orientation Control</h1>";
  html += "<p>WebSocket: ws://192.168.4.1:81</p>";
  html += "<p>State: ";
  switch (current_state) {
    case STATE_IDLE: html += "IDLE"; break;
    case STATE_CALIBRATING: html += "CALIBRATING"; break;
    case STATE_PRE_SPIN: html += "PRE-SPIN"; break;
    case STATE_RUNNING: html += "RUNNING"; break;
    case STATE_MANUAL_TEST: html += "MANUAL TEST"; break;
  }
  html += "</p>";
  html += "<p>Max PWM: 50% | Ramp: 0.5%/cycle | Min: 5%</p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

// --- Setup ---
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Drone Orientation Control v2 ===");
  Serial.println("SAFETY: Max 50%, Ramp 0.5%/cycle, Pre-spin 3s");

  pinMode(MOTOR1_PIN, OUTPUT);
  pinMode(MOTOR2_PIN, OUTPUT);
  pinMode(MOTOR3_PIN, OUTPUT);
  pinMode(MOTOR4_PIN, OUTPUT);
  stop_motors();

  mpu_init();
  Serial.println("[HW] MPU6050 OK");

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(500);
  Serial.print("[NET] AP IP: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, handleRoot);
  server.begin();

  ws.begin();
  ws.onEvent(ws_event);

  Serial.println("[NET] WiFi 'DroneControl' pw 'drone1234'");
  Serial.println("[NET] WS: ws://192.168.4.1:81");
  Serial.println("Ready.");

  last_loop_time = micros();
}

// --- Loop ---
void loop() {
  ws.loop();
  server.handleClient();

  unsigned long now = micros();
  if (now - last_loop_time >= LOOP_INTERVAL_US) {
    float dt = (now - last_loop_time) / 1000000.0;
    last_loop_time = now;

    switch (current_state) {
      case STATE_CALIBRATING:
        calibration_step();
        break;

      case STATE_PRE_SPIN:
        update_angles(dt);
        apply_motors();  // ramp limited, will slowly reach 5%
        if (millis() - pre_spin_start >= PRE_SPIN_DURATION) {
          current_state = STATE_RUNNING;
          ws.broadcastTXT("{\"event\":\"running\"}");
          Serial.println("[CTL] PID control ACTIVE");
        }
        break;

      case STATE_RUNNING:
        update_angles(dt);
        compute_motors();
        apply_motors();  // rate limited
        break;

      case STATE_MANUAL_TEST:
        apply_motors();  // ramp to 5%
        break;

      case STATE_IDLE:
        if (calibrated) update_angles(dt);
        break;
    }
  }

  // Telemetry via WebSocket
  unsigned long now_ms = millis();
  if (now_ms - last_ws_send >= WS_SEND_INTERVAL) {
    last_ws_send = now_ms;
    String msg;
    if (current_state == STATE_CALIBRATING) {
      msg = "{\"cal_progress\":" + String((calibration_samples * 100) / CALIBRATION_COUNT) + "}";
    } else {
      msg = "{\"roll\":" + String(angle_roll, 2) +
            ",\"pitch\":" + String(angle_pitch, 2) +
            ",\"yaw\":" + String(angle_yaw, 2) +
            ",\"target\":[" + String(target_roll, 1) + "," + String(target_pitch, 1) + "," + String(target_yaw, 1) + "]" +
            ",\"pwm_target\":[" + String(motor_pwm_target[0], 1) + "," + String(motor_pwm_target[1], 1) + "," + String(motor_pwm_target[2], 1) + "," + String(motor_pwm_target[3], 1) + "]" +
            ",\"pwm_actual\":[" + String(motor_pwm_actual[0], 1) + "," + String(motor_pwm_actual[1], 1) + "," + String(motor_pwm_actual[2], 1) + "," + String(motor_pwm_actual[3], 1) + "]" +
            ",\"state\":\"" + String(
              current_state == STATE_RUNNING ? "RUNNING" :
              current_state == STATE_PRE_SPIN ? "PRE_SPIN" :
              current_state == STATE_MANUAL_TEST ? "TEST" : "IDLE") + "\"}";
    }
    ws.broadcastTXT(msg);
  }

  // Serial log
  if (now_ms - last_serial_log >= SERIAL_LOG_INTERVAL) {
    last_serial_log = now_ms;
    if (current_state == STATE_RUNNING || current_state == STATE_PRE_SPIN || current_state == STATE_MANUAL_TEST) {
      serial_log();
    }
  }
}
