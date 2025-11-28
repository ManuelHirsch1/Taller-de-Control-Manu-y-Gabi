#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

MPU6050 mpu;
Servo servo;

// ---------- Pines ----------
const int SERVO_PIN = 9;
const int POT_PIN   = A0;   // no lo usamos acá, pero lo dejo por compatibilidad

// ---------- Calibración del potenciómetro (no usado ahora) ----------
const float A_cal = -0.261627f;
const float B_cal = 251.16192f;

float potToAngle(int adc) {
  return A_cal * adc + B_cal;
}

// ---------- Servo ----------
const int PWM_MIN = 700;
const int PWM_MAX = 2300;
const int SERVO_CENTER_DEG = 90;  // “equilibrio” mecánico

// ---------- Excitación por escalones ----------
// u_deg = ángulo de ENTRADA (offset respecto al centro del servo)
const int   N_STEPS         = 5;
const float step_values_deg[N_STEPS] = { 0.0f, 20.0f, -20.0f, 40.0f, -50.0f };
// duración de cada escalón (ajustá a gusto)
const float STEP_DURATION_S = 6.0f;

// ---------- Muestreo ----------
const float Ts_s = 0.020f;   // 20 ms (50 Hz)

// ---------- Filtro complementario (IMU) ----------
const float ALPHA_CF = 0.98f;
static float theta_cf_deg = 0.0f;
static bool  cf_init = false;
static unsigned long last_cf_ms = 0;

// ---------- Serial ----------
const long BAUD = 115200;

// ---------- Funciones auxiliares ----------
int angToPWM(float angRef) {
  if (angRef < 0)   angRef = 0;
  if (angRef > 180) angRef = 180;
  long us = map((int)angRef, 0, 180, PWM_MIN, PWM_MAX);
  if (us < PWM_MIN) us = PWM_MIN;
  if (us > PWM_MAX) us = PWM_MAX;
  return (int)us;
}

void moverServo_deg(float angRef) {
  servo.writeMicroseconds(angToPWM(angRef));
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
  moverServo_deg(SERVO_CENTER_DEG); // arranca centrado
  delay(1000);

  // Cabecera CSV
  Serial.println("t_s,u_deg,theta_pend_deg");

  last_cf_ms = millis();
}

// ---------- Loop principal ----------
void loop() {
  static unsigned long t0_ms        = millis();
  static unsigned long last_loop_ms = millis();
  static bool finished              = false;

  unsigned long now_ms = millis();
  float t_s = (now_ms - t0_ms) / 1000.0f;

  // Si ya terminamos todos los escalones, mandar fin y congelar
  float total_duration = N_STEPS * STEP_DURATION_S;
  if (!finished && t_s >= total_duration) {
    Serial.println("END");
    finished = true;
  }

  if (finished) {
    // Mantener el servo en la última posición y no hacer más nada
    delay(100);
    return;
  }

  // ----- 1. Determinar escalón actual -----
  int step_idx = (int)(t_s / STEP_DURATION_S);
  if (step_idx < 0) step_idx = 0;
  if (step_idx >= N_STEPS) step_idx = N_STEPS - 1;

  float u_deg = step_values_deg[step_idx]; // entrada (offset en grados)
  float servo_deg = SERVO_CENTER_DEG + u_deg;

  if (servo_deg < 0.0f)   servo_deg = 0.0f;
  if (servo_deg > 180.0f) servo_deg = 180.0f;

  moverServo_deg(servo_deg);

  // ----- 2. Leer IMU (ángulo del péndulo) -----
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float ay_g = (float)ay / 16384.0f;
  float az_g = (float)az / 16384.0f;
  float theta_acc_deg = atan2(ay_g, az_g) * 180.0f / PI;

  float gx_dps = (float)gx / 131.0f;
  float dt_cf  = (millis() - last_cf_ms) / 1000.0f;
  if (dt_cf > 0.1f) dt_cf = Ts_s; // evita saltos grandes

  if (!cf_init) {
    theta_cf_deg = theta_acc_deg;
    cf_init = true;
  } else {
    float pred = theta_cf_deg + gx_dps * dt_cf; // integrar gyro
    theta_cf_deg = ALPHA_CF * pred + (1.0f - ALPHA_CF) * theta_acc_deg;
  }
  last_cf_ms = millis();

  float theta_pend_deg = theta_cf_deg;

  // ----- 3. Enviar tiempo, entrada y salida -----
  Serial.print(t_s, 3);
  Serial.print(",");
  Serial.print(u_deg, 3);
  Serial.print(",");
  Serial.println(theta_pend_deg, 3);

  // ----- 4. Mantener 20 ms de muestreo -----
  unsigned long loop_end_ms = millis();
  long elapsed = loop_end_ms - last_loop_ms;
  if (elapsed < 20) delay(20 - elapsed);
  last_loop_ms = millis();
}