#include <Servo.h>
#include <Wire.h>
#include <MPU6050.h>

Servo servo;
MPU6050 mpu;

const int SERVO_PIN = 6;
const int potPin = A0; // por ahora no lo imprimimos, pero lo dejamos disponible

// ================= Servo / PWM =================
const int PWM_MIN = 900;
const int PWM_MAX = 2100;

// Mapea 0–180° → µs y satura en [PWM_MIN, PWM_MAX]
int angToPWM(int angRef) {
  int pulse = map(angRef, 0, 180, 800, 2200);
  if (pulse < PWM_MIN) pulse = PWM_MIN;
  if (pulse > PWM_MAX) pulse = PWM_MAX;
  return pulse;
}
void moverServo_deg(int angRef) { servo.writeMicroseconds(angToPWM(angRef)); }

// ============== Calibración pote → ángulo (tuyos) ==============
const float A_cal = -0.412371;
const float B_cal = 305.051544;
float angRealDesdeADC(int adc) { return A_cal * adc + B_cal; } // no usado en CSV ahora

// ================= IMU / Filtro complementario =================
float theta_deg = 0.0f;     // salida: ángulo péndulo
float theta0_deg = 0.0f;    // offset inicial para que θ=0 con péndulo abajo
float gyro_bias_x = 0.0f;   // bias del giroscopio eje X
const float alpha = 0.98f;  // peso del gyro
const int   GYRO_SENS = 131; // LSB/(°/s) @ ±250 dps

// Si necesitás invertir signo según montaje, cambiá a -1
const int SIGN_THETA = +1;

// ================== Muestreo ==================
const float TS = 0.01f;         // 0.01 s → 100 Hz aprox.
unsigned long lastMicros = 0;

// ================== Escalón en referencia ===================
// Centro típico del servo (en "grados de servo": 0–180). Suele ser ~90.
const int SERVO_CENTER_DEG = 90;
// Amplitud del escalón (deg de servo). p.ej. +5 mueve el brazo 5° desde el centro
const int STEP_DEG = +45;
// Momento en que aplicamos el escalón (s)
const float T_STEP = 6.0f;

// ================== Serial ===================
const long BAUD = 115200;

// ====== Utilidad segura para saturar ángulo del servo ======
int clampServoDeg(int a) {
  if (a < 0) a = 0;
  if (a > 180) a = 180;
  return a;
}

void setup() {
  Serial.begin(BAUD);
  Wire.begin();

  servo.attach(SERVO_PIN);
  moverServo_deg(SERVO_CENTER_DEG); // ir al centro

  mpu.initialize();
  delay(100);
  // Comprobación simple (opcional)
  // if (!mpu.testConnection()) { Serial.println("MPU6050 no conectado"); }

  // ---------- Calibración rápida de bias gyro y offset de ángulo ----------
  const unsigned long calib_ms = 2000;
  unsigned long t0 = millis();
  long n = 0;

  double sum_gx = 0.0;
  double sum_angAcc = 0.0;

  while (millis() - t0 < calib_ms) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Escalas acelerómetro para ángulo "acotado" inicial
    float ay_g = (float)ay / 16384.0f;
    float az_g = (float)az / 16384.0f;
    float angAcc = atan2(ay_g, az_g) * 180.0f / PI; // como en tu código base

    sum_gx += gx;
    sum_angAcc += angAcc;
    n++;
    delay(2);
  }
  if (n > 0) {
    gyro_bias_x = (float)(sum_gx / (double)n);
    theta0_deg   = (float)(sum_angAcc / (double)n); // referencia: péndulo abajo
  }

  // Inicializo filtro con 0 (restando offset para que arranque alrededor de 0)
  theta_deg = 0.0f;

  lastMicros = micros();

  // Encabezado CSV
  Serial.println("t_s,theta_deg,phi_ref_deg");
}

void loop() {
  // Bucle cronometrado ~TS
  unsigned long now = micros();
  if (now - lastMicros < (unsigned long)(TS * 1e6)) return;
  float dt = (now - lastMicros) / 1e6f;
  lastMicros = now;

  // ---------- Lectura IMU ----------
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Acelerómetro → ángulo (como tu implementación: atan2(ay, az))
  float ay_g = (float)ay / 16384.0f;
  float az_g = (float)az / 16384.0f;
  float angAcc = atan2(ay_g, az_g) * 180.0f / PI;

  // Gyro X → °/s (resto bias)
  float gyroRate = ((float)gx - gyro_bias_x) / (float)GYRO_SENS;

  // Filtro complementario (resto theta0 para que péndulo abajo sea 0°)
  float angAcc_zeroed = (angAcc - theta0_deg);
  theta_deg = alpha * (theta_deg + SIGN_THETA * gyroRate * dt)
            + (1.0f - alpha) * (SIGN_THETA * angAcc_zeroed);

  // ---------- Generación de referencia (escalón) ----------
  float t_s = (millis()) / 1000.0f; // tiempo desde arranque
  static int phi_ref_deg = SERVO_CENTER_DEG;
  if (t_s >= T_STEP) {
    phi_ref_deg = clampServoDeg(SERVO_CENTER_DEG + STEP_DEG);
  } else {
    phi_ref_deg = SERVO_CENTER_DEG;
  }

  // Mover servo a la referencia
  moverServo_deg(phi_ref_deg);

  // ---------- CSV: t_s, theta_deg, phi_ref_deg ----------
  Serial.print(t_s, 4);       Serial.print(",");
  Serial.print(theta_deg, 4); Serial.print(",");
  Serial.println(phi_ref_deg);
}

