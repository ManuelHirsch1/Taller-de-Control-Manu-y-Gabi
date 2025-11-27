#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

MPU6050 mpu;
Servo servo;

// ---------- Pines ----------
const int SERVO_PIN = 9;
const int POT_PIN   = A0;

// ---------- Calibración potenciómetro ----------
float A_cal = -0.261627f;
float B_cal = 251.16192f;
float potToAngle(int adc) { return A_cal * adc + B_cal; }

// ---------- Servo ----------
const int PWM_MIN = 700;
const int PWM_MAX = 2300;
const int SERVO_CENTER_DEG = 90;

int angToPWM(int angRef) {
  angRef = constrain(angRef, 0, 180);
  return map(angRef, 0, 180, PWM_MIN, PWM_MAX);
}
void moverServo_deg(int angRef) { servo.writeMicroseconds(angToPWM(angRef)); }

// ---------- Ensayo multi-escalón ----------
const float Ts_s       = 0.02f;   // 50 Hz
const float T_STEP_S   = 6.0f;    // duración por nivel
const int N_STEPS      = 6;
const int STEP_AMPLS[N_STEPS] = { 0, 20, -20, 40, -40, 0 };

// ---------- Filtro complementario ----------
const float ALPHA_CF = 0.98f;
float theta_cf_deg = 0.0f;
bool  cf_init = false;
unsigned long last_cf_ms = 0;

// ---------- Serial ----------
const long BAUD = 115200;

// ---------- Setup ----------
void setup() {
  Serial.begin(BAUD);
  Wire.begin();
  Wire.setClock(400000);
  mpu.initialize();
  delay(200);

  if (!mpu.testConnection()) {
    Serial.println("Error: MPU6050 no conectado");
    while (1) delay(100);
  }

  servo.attach(SERVO_PIN);
  moverServo_deg(SERVO_CENTER_DEG);
  delay(1000);

  Serial.println("t_s,step_index,phi_ref_deg,phi_deg,theta_deg");
  last_cf_ms = millis();
}

// ---------- Loop ----------
void loop() {
  static unsigned long t0_ms = millis();
  unsigned long now_ms = millis();
  float t_s = (now_ms - t0_ms) / 1000.0f;

  // 1) índice del escalón actual
  int idx = (int)(t_s / T_STEP_S);
  if (idx >= N_STEPS) {
    moverServo_deg(SERVO_CENTER_DEG);
    while (1) delay(100);
  }

  // 2) comando al servo
  int phi_ref_deg = SERVO_CENTER_DEG + STEP_AMPLS[idx];
  moverServo_deg(phi_ref_deg);

  // 3) lectura potenciómetro
  int adc_pot = analogRead(POT_PIN);
  float phi_deg = potToAngle(adc_pot);

  // 4) lectura IMU (θ)
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float ay_g = (float)ay / 16384.0f;
  float az_g = (float)az / 16384.0f;
  float theta_acc_deg = atan2(ay_g, az_g) * 180.0f / PI;
  float gx_dps = (float)gx / 131.0f;
  float dt_cf = (last_cf_ms == 0) ? Ts_s : (now_ms - last_cf_ms) / 1000.0f;
  last_cf_ms = now_ms;
  if (!cf_init) { theta_cf_deg = theta_acc_deg; cf_init = true; }
  else {
    float pred = theta_cf_deg + gx_dps * dt_cf;
    theta_cf_deg = ALPHA_CF * pred + (1 - ALPHA_CF) * theta_acc_deg;
  }

  // 5) salida CSV
  Serial.print(t_s, 3); Serial.print(",");
  Serial.print(idx); Serial.print(",");
  Serial.print(phi_ref_deg - SERVO_CENTER_DEG, 3); Serial.print(",");
  Serial.print(phi_deg, 3); Serial.print(",");
  Serial.println(theta_cf_deg, 3);

  delay((int)(Ts_s * 1000));
}
