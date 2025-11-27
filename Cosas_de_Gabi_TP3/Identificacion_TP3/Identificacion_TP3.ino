#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

MPU6050 mpu;
Servo servo;

// ---------- Pines ----------
const int SERVO_PIN = 9;
const int POT_PIN   = A0;

// ---------- Calibración del potenciómetro ----------
const float A_cal = -0.27523f;
const float B_cal = 232.5688f;

float potToAngle(int adc) {
  return A_cal * adc + B_cal;
}

// ---------- Servo ----------
const int PWM_MIN = 700;
const int PWM_MAX = 2300;

int angToPWM(int angRef) {
  angRef = constrain(angRef, 0, 180);
  long us = map(angRef, 0, 180, PWM_MIN, PWM_MAX);
  return (int)us;
}

void moverServo_deg(int angRef) {
  servo.writeMicroseconds(angToPWM(angRef));
}

// ---------- Escalón ----------
const int SERVO_CENTER_DEG = 90;
const int STEP_DEG = 60;
const float T_STEP_S = 6.0f;

// ---------- Serial ----------
const long BAUD = 115200;

// ---------- Filtro complementario ----------
const float ALPHA_CF = 0.98f;
static float theta_cf_deg = 0.0f;
static bool  cf_init = false;
static unsigned long last_cf_ms = 0;

// ---------- Calibración IMU ----------
float gyro_bias_gx = 0.0f;
float accel_bias_deg = 0.0f;

void calibrarIMU(int N = 500) {
  Serial.println(F("# Calibrando IMU (mantener quieto)..."));
  long sum_gx = 0;
  float sum_acc = 0.0f;

  for (int i = 0; i < N; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    float ay_g = (float)ay / 16384.0f;
    float az_g = (float)az / 16384.0f;
    float theta_acc = atan2(ay_g, az_g) * 180.0f / PI;

    sum_gx += gx;
    sum_acc += theta_acc;
    delay(5);
  }

  gyro_bias_gx = (float)sum_gx / (float)N;
  accel_bias_deg = sum_acc / (float)N;

  Serial.print(F("# Gyro bias gx: ")); Serial.println(gyro_bias_gx, 2);
  Serial.print(F("# Accel bias: ")); Serial.println(accel_bias_deg, 2);
  Serial.println(F("# IMU calibrada."));
}

// ---------- Setup ----------
void setup() {
  Serial.begin(BAUD);
  Wire.begin();
  Wire.setClock(400000);
  mpu.initialize();
  delay(100);

  if (!mpu.testConnection()) {
    Serial.println("Error: MPU6050 no conectado");
    while (1) { delay(100); }
  }

  servo.attach(SERVO_PIN);
  moverServo_deg(SERVO_CENTER_DEG);
  delay(600);

  calibrarIMU();
  last_cf_ms = millis();

  // ---------- Encabezado CSV ----------
  Serial.println("t_s,phi_deg,theta_deg");
}

// ---------- Loop ----------
void loop() {
  static unsigned long t0_ms = millis();
  unsigned long now_ms = millis();
  float t_s = (now_ms - t0_ms) / 1000.0f;

  // ---------- Escalón ----------
  int phi_ref_deg = (t_s >= T_STEP_S) ?
                     (SERVO_CENTER_DEG + STEP_DEG) :
                     SERVO_CENTER_DEG;
  moverServo_deg(phi_ref_deg);

  // ---------- Lectura potenciómetro ----------
  int adc_pot = analogRead(POT_PIN);
  float phi_deg = potToAngle(adc_pot);

  // ---------- Lectura IMU ----------
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float ay_g = (float)ay / 16384.0f;
  float az_g = (float)az / 16384.0f;
  float theta_acc_deg = atan2(ay_g, az_g) * 180.0f / PI - accel_bias_deg;

  float gx_dps = ((float)gx - gyro_bias_gx) / 131.0f;
  float dt_cf = (millis() - last_cf_ms) / 1000.0f;
  if (dt_cf > 0.1f) dt_cf = 0.01f;

  if (!cf_init) {
    theta_cf_deg = theta_acc_deg;
    cf_init = true;
  } else {
    float pred = theta_cf_deg + gx_dps * dt_cf;
    theta_cf_deg = ALPHA_CF * pred + (1.0f - ALPHA_CF) * theta_acc_deg;
  }
  last_cf_ms = millis();

  // ---------- Envío CSV (solo t, φ, θ) ----------
  Serial.print(t_s, 3); Serial.print(",");
  Serial.print(phi_deg, 3); Serial.print(",");
  Serial.println(theta_cf_deg, 3);

  delay(10); // ~100 Hz
}
