#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

MPU6050 mpu;
Servo servo;

// ---------- Pines ----------
const int SERVO_PIN = 9;
const int POT_PIN   = A0;

// ---------- Calibración potenciómetro ----------
float A_cal = -0.26239f;
float B_cal = 229.329f;

// ---------- Servo ----------
const int PWM_MIN = 700;
const int PWM_MAX = 2100;
const int SERVO_CENTER_DEG = 90;

int angToPWM(int angRef) {
  angRef = constrain(angRef, 0, 180);
  long us = map(angRef, 0, 180, PWM_MIN, PWM_MAX);
  return constrain(us, PWM_MIN, PWM_MAX);
}

void moverServo_deg(int angRef) { servo.writeMicroseconds(angToPWM(angRef)); }

// ---------- IMU / filtro ----------
const float ALPHA_CF = 0.98f;   // 0.98*gyro + 0.02*acc
const float Ts_s     = 0.02f;   // 50 Hz
float theta_cf_deg   = 0.0f;
bool  cf_init        = false;
unsigned long last_cf_ms = 0;

// Sesgos de calibración
float gyro_bias_gx_LSB = 0.0f;
float accel_bias_deg   = 0.0f;

// ---------- Excitación ----------
const float DUR_TOTAL_S = 100.0f;
const float STEP1_T     = 5.0f;
const float STEP2_T     = 12.0f;
const int   STEP_AMPL   = 20;    // ±° sobre centro
const float W_EXC       = 1.5f;  // rad/s
const float AMP_EXC     = 10.0f; // °

// ---------- Serial ----------
const long BAUD = 115200;

// ---------- Calibración IMU ----------
void calibrarIMU(int N=600, int warmup_ms=1500) {
  Serial.println(F("# Warm-up IMU..."));
  delay(warmup_ms);

  long sum_gx = 0;
  float sum_theta_acc = 0.0f;

  Serial.println(F("# Calibrando IMU (mantener quieto)"));
  for (int i = 0; i < N; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    float ay_g = (float)ay / 16384.0f;
    float az_g = (float)az / 16384.0f;
    float theta_acc_deg = atan2(ay_g, az_g) * 180.0f / PI;

    sum_gx        += gx;
    sum_theta_acc += theta_acc_deg;
    delay(5);
  }

  gyro_bias_gx_LSB = (float)sum_gx / (float)N;
  accel_bias_deg   = sum_theta_acc / (float)N;

  Serial.print(F("# gyro_bias_gx (LSB): ")); Serial.println(gyro_bias_gx_LSB, 3);
  Serial.print(F("# accel_bias_theta (deg): ")); Serial.println(accel_bias_deg, 3);
  Serial.println(F("# IMU calibrada."));
}

// ---------- Lectura IMU con filtro ----------
float leerThetaPenduloDeg() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float ay_g = (float)ay / 16384.0f;
  float az_g = (float)az / 16384.0f;

  // Acelerómetro corregido
  float theta_acc_deg = atan2(ay_g, az_g) * 180.0f / PI - accel_bias_deg;
  // Giroscopio corregido
  float gx_dps = ((float)gx - gyro_bias_gx_LSB) / 131.0f;

  // Delta tiempo
  unsigned long now_ms = millis();
  float dt_cf = (last_cf_ms == 0) ? Ts_s : (now_ms - last_cf_ms) / 1000.0f;
  if (dt_cf <= 0.0f || dt_cf > 0.1f) dt_cf = Ts_s;

  // Filtro complementario
  if (!cf_init) {
    theta_cf_deg = theta_acc_deg;
    cf_init = true;
  } else {
    float pred = theta_cf_deg + gx_dps * dt_cf;
    theta_cf_deg = ALPHA_CF * pred + (1.0f - ALPHA_CF) * theta_acc_deg;
  }
  last_cf_ms = now_ms;

  return theta_cf_deg;
}

// ---------- Función de ángulo del potenciómetro ----------
float potToAngle(int adc) {
  return A_cal * adc + B_cal;
}

// ---------- Setup ----------
void setup() {
  Serial.begin(BAUD);
  Wire.begin();
  Wire.setClock(400000);
  mpu.initialize();
  delay(200);
  if (!mpu.testConnection()) {
    Serial.println(F("Error: IMU no conectada."));
    while (1) { delay(100); }
  }

  servo.attach(SERVO_PIN);
  moverServo_deg(SERVO_CENTER_DEG);
  delay(1000);

  calibrarIMU(); // Calibración IMU

  // ---------- Auto-centrado del potenciómetro ----------
  Serial.println(F("# Calibrando centro del brazo..."));
  const int Ncenter = 100;
  long acc_pot = 0;
  for (int i = 0; i < Ncenter; i++) {
    acc_pot += analogRead(POT_PIN);
    delay(5);
  }
  float adc_centro = acc_pot / (float)Ncenter;
  // Recalibra B_cal para forzar φ = 0 en el centro actual
  B_cal = -A_cal * adc_centro;
  Serial.print(F("# ADC centro: ")); Serial.println(adc_centro, 2);
  Serial.print(F("# Nuevo B_cal: ")); Serial.println(B_cal, 3);
  Serial.println(F("# φ=0 en reposo asegurado."));

  last_cf_ms = millis();
  Serial.println(F("t_s,phi_ref_deg,phi_deg,theta_deg"));
}

// ---------- Loop ----------
void loop() {
  static unsigned long t0_ms = millis();
  unsigned long now_ms = millis();
  float t_s = (now_ms - t0_ms) / 1000.0f;

  // ---------- Entrada compuesta ----------
  int phi_ref_deg = SERVO_CENTER_DEG;
  if (t_s >= STEP1_T && t_s < STEP2_T)
    phi_ref_deg = SERVO_CENTER_DEG + STEP_AMPL;
  else if (t_s >= STEP2_T)
    phi_ref_deg = SERVO_CENTER_DEG - STEP_AMPL;
  phi_ref_deg += (int)(AMP_EXC * sin(W_EXC * t_s));

  moverServo_deg(phi_ref_deg);

  // ---------- Salidas ----------
  int adc_pot = analogRead(POT_PIN);
  float phi_deg = potToAngle(adc_pot); // ya centrado en 0
  float theta_deg = leerThetaPenduloDeg();

  // ---------- CSV ----------
  Serial.print(t_s, 3);          Serial.print(",");
  Serial.print(phi_ref_deg - SERVO_CENTER_DEG, 3); // entrada centrada
  Serial.print(",");
  Serial.print(phi_deg, 3);
  Serial.print(",");
  Serial.println(theta_deg, 3);

  // ---------- Fin ----------
  if (t_s > DUR_TOTAL_S) {
    moverServo_deg(SERVO_CENTER_DEG);
    while (1) { delay(100); }  // detener
  }

  delay((int)(Ts_s * 1000));
}
