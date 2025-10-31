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

// ---------- Servo: mapeo grados -> microsegundos ----------
const int PWM_MIN = 700;   // rango seguro para MG996R
const int PWM_MAX = 2300;

int angToPWM(int angRef) {
  if (angRef < 0)   angRef = 0;
  if (angRef > 180) angRef = 180;
  long us = map(angRef, 0, 180, PWM_MIN, PWM_MAX);
  return (int)us;
}

void moverServo_deg(int angRef) {
  servo.writeMicroseconds(angToPWM(angRef));
}

// ---------- Escalón ----------
const int SERVO_CENTER_DEG = 90;   // posición central
const int STEP_DEG = 60;           // amplitud del escalón
const float T_STEP_S = 6.0f;       // tiempo en que ocurre el escalón

// ---------- Serial ----------
const long BAUD = 115200;

// ---------- Filtro complementario (IMU) ----------
const float ALPHA_CF = 0.98f;   // peso del giroscopio
static float theta_cf_deg = 0.0f;
static bool  cf_init = false;
static unsigned long last_cf_ms = 0;

void setup() {
  Serial.begin(BAUD);
  Wire.begin();
  Wire.setClock(400000); // I2C rápido
  mpu.initialize();
  delay(100);

  if (!mpu.testConnection()) {
    Serial.println("Error: MPU6050 no conectado");
    while (1) { delay(100); }
  }

  servo.attach(SERVO_PIN);
  moverServo_deg(SERVO_CENTER_DEG); // arranca centrado
  delay(600); // tiempo de asentamiento

  // ---------- Calibración dinámica del potenciómetro ----------
  long acc = 0;
  const int N = 50;
  for (int i = 0; i < N; i++) {
    acc += analogRead(POT_PIN);
    delay(5);
  }
  float adc_centro = acc / (float)N;
  float B_cal_rt = 90.0f - A_cal * adc_centro; // autoajuste dinámico

  Serial.println("t_s,theta_pot_deg,theta_cf_deg");
}

void loop() {
  static unsigned long t0_ms = millis();
  unsigned long now_ms = millis();
  float t_s = (now_ms - t0_ms) / 1000.0f;

  // ---------- Generación del escalón ----------
  int phi_ref_deg = (t_s >= T_STEP_S) ? (SERVO_CENTER_DEG + STEP_DEG) : SERVO_CENTER_DEG;
  moverServo_deg(phi_ref_deg);

  // ---------- Lectura del potenciómetro ----------
  int adc_pot = analogRead(POT_PIN);
  float theta_pot_deg = potToAngle(adc_pot);

  // ---------- Lectura del MPU6050 ----------
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Acelerómetro → ángulo absoluto (roll)
  float ay_g = (float)ay / 16384.0f;
  float az_g = (float)az / 16384.0f;
  float theta_acc_deg = atan2(ay_g, az_g) * 180.0f / PI;

  // Giroscopio → velocidad angular (°/s)
  float gx_dps = (float)gx / 131.0f;

  // Calcular intervalo de tiempo real
  float dt_cf = (last_cf_ms == 0) ? 0.0f : (now_ms - last_cf_ms) / 1000.0f;
  last_cf_ms = now_ms;

  // Filtro complementario
  if (!cf_init) {
    theta_cf_deg = theta_acc_deg;
    cf_init = true;
  } else {
    float pred = theta_cf_deg + gx_dps * dt_cf; // integrar giroscopio
    theta_cf_deg = ALPHA_CF * pred + (1.0f - ALPHA_CF) * theta_acc_deg;
  }

  // ---------- Enviar datos por Serial (CSV) ----------
  Serial.print(t_s, 3);
  Serial.print(",");
  Serial.print(theta_pot_deg, 3);
  Serial.print(",");
  Serial.println(theta_cf_deg, 3);

  delay(10); // ~100 Hz de muestreo
}

