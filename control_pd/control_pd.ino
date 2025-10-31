#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

MPU6050 mpu;
Servo servo;

// ---------- Pines ----------
const int SERVO_PIN = 9;
const int POT_PIN   = A0;

// ---------- Calibración del potenciómetro ----------
const float A_cal = -0.261627f;
const float B_cal = 251.16192f;

float potToAngle(int adc) {
  return A_cal * adc + B_cal;
}

// ---------- Servo ----------
const int PWM_MIN = 700;
const int PWM_MAX = 2300;
const int SERVO_CENTER_DEG = 90;

// ---------- Control ----------
const float Ts_s = 0.020f;   // 20 ms (50 Hz)
const float Kp = 0.7f;       // Ganancia proporcional
const float Kd = 0.03f;      // Ganancia derivativa (ajustá libremente)
const float REF_DEG = 0.0f;  // referencia = péndulo hacia abajo

// ---------- Filtro complementario (IMU) ----------
const float ALPHA_CF = 0.98f;
static float theta_cf_deg = 0.0f;
static bool  cf_init = false;
static unsigned long last_cf_ms = 0;

// ---------- Derivada filtrada (Tustin bilineal) ----------
const float tau_f = 0.023f;   // Constante de tiempo del filtro (~7 Hz)
const float r_tu  = 2.0f * tau_f / Ts_s;

// Coeficientes Tustin
const float a0 = 1.0f + r_tu;
const float a1 = 1.0f - r_tu;
const float b0 = 2.0f / Ts_s;
const float b1 = -2.0f / Ts_s;

// Predivididos (para eficiencia)
const float c0 = b0 / a0;
const float c1 = b1 / a0;
const float d1 = a1 / a0;

// Estados del filtro derivativo
float x_prev = 0.0f;   // θ[k-1]
float ydot_f = 0.0f;   // derivada filtrada actual (°/s)

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

  Serial.println("t_s,theta_pend_deg");
  last_cf_ms = millis();
}

// ---------- Loop principal ----------
void loop() {
  static unsigned long t0_ms = millis();
  static unsigned long last_loop_ms = millis();
  unsigned long now_ms = millis();
  float t_s = (now_ms - t0_ms) / 1000.0f;

  // ----- 1. Leer IMU (ángulo del péndulo) -----
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

  // ----- 2. Derivada filtrada (Tustin bilineal) -----
  float xk = theta_pend_deg; // entrada actual
  // ydot_f[k] = c0*x[k] + c1*x[k-1] - d1*ydot_f[k-1]
  ydot_f = c0 * xk + c1 * x_prev - d1 * ydot_f;
  x_prev = xk; // actualizar memoria

  // ----- 3. Control PD -----
  float error_deg = REF_DEG - theta_pend_deg;
  float u_deg = -Kp * error_deg - Kd * ydot_f;  // PD (derivada sobre la salida)

  // ----- 4. Convertir u a posición del servo -----
  float servo_deg = SERVO_CENTER_DEG + u_deg;
  if (servo_deg < 0) servo_deg = 0;
  if (servo_deg > 180) servo_deg = 180;

  moverServo_deg(servo_deg);

  // ----- 5. Enviar tiempo y salida -----
  Serial.print(t_s, 3);
  Serial.print(",");
  Serial.println(theta_pend_deg, 3);

  // ----- 6. Mantener 20 ms de muestreo -----
  unsigned long loop_end_ms = millis();
  long elapsed = loop_end_ms - last_loop_ms;
  if (elapsed < 20) delay(20 - elapsed);
  last_loop_ms = millis();
}
