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
const float Ki = 0.5f;      // 💥 Ganancia integral (ajustá libremente)
const float REF_DEG = 0.0f;  // referencia = péndulo hacia abajo

// ---------- Filtro complementario (IMU) ----------
const float ALPHA_CF = 0.98f;
static float theta_cf_deg = 0.0f;
static bool  cf_init = false;
static unsigned long last_cf_ms = 0;

// ---------- Integrador ----------
float I_term = 0.0f;           // valor acumulado del integrador
const float I_MAX = 50.0f;     // anti-windup (límite superior)
const float I_MIN = -50.0f;    // anti-windup (límite inferior)

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

  // ----- 2. Control PI -----
  float error_deg = REF_DEG - theta_pend_deg;

  // integrar el error
  I_term += Ki * Ts_s * error_deg;

  // anti-windup (limitar acumulador)
  if (I_term > I_MAX) I_term = I_MAX;
  if (I_term < I_MIN) I_term = I_MIN;

  // control total
  float u_deg = -Kp * error_deg - I_term;  // nota: signo negativo para estabilizar

  // ----- 3. Convertir u a posición del servo -----
  float servo_deg = SERVO_CENTER_DEG + u_deg;
  if (servo_deg < 0) servo_deg = 0;
  if (servo_deg > 180) servo_deg = 180;

  moverServo_deg(servo_deg);

  // ----- 4. Enviar tiempo y salida -----
  Serial.print(t_s, 3);
  Serial.print(",");
  Serial.println(theta_pend_deg, 3);

  // ----- 5. Mantener 20 ms de muestreo -----
  unsigned long loop_end_ms = millis();
  long elapsed = loop_end_ms - last_loop_ms;
  if (elapsed < 20) delay(20 - elapsed);
  last_loop_ms = millis();
}
