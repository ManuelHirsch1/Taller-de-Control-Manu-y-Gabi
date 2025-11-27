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

// ---------- Servo ----------
const int PWM_MIN = 700;
const int PWM_MAX = 2300;
const int SERVO_CENTER_DEG = 90;

// ---------- Control ----------
const float Ts_s = 0.020f;   // 20 ms (50 Hz)
const float REF_DEG = 0.0f;  // referencia (péndulo vertical)

// ---------- Filtro complementario / IMU ----------
const float ALPHA_CF = 0.98f;
float theta_cf_deg = 0.0f;
bool cf_init = false;
unsigned long last_cf_ms = 0;

// ---------- Sesgos IMU ----------
float gyro_bias_gx_LSB = 0.0f;
float accel_bias_deg   = 0.0f;

// ---------- Serial ----------
const long BAUD = 115200;

// ---------- Matrices del modelo (discreto) ----------
const float Ad[4][4] = {
  { 0.93441f,   -0.16612f,   -0.062325f,  -0.049569f },
  { 0.15506f,    0.98663f,   -0.0050149f, -0.0040103f },
  { 0.0062724f,  0.079642f,   0.99987f,   -0.00010752f },
  { 0.000084082f, 0.0015964f, 0.039999f,   1.0f }
};


const float Bd[4] = {
  0.038765f,
  0.0031362f,
  0.000084082f,
  0.00000084346f
};


const float Cd[4] = {
  -1.5561f,
  -0.33845f,
  -0.25334f,
  -0.049822f
};


// ---------- Ganancias (discreto) ----------
const float Kd[4] = {
  2.0333f,
  0.23292f,
  0.10156f,
  -0.05157f
};


const float Ld[4] = {
   0.62151f,
   2.1705f,
 -12.353f,
  10.753f
};



// ---------- Estados iniciales de las variables de estados ----------
float x_hat[4] = {0.0f, 0.0f, 0.0f, 0.0f};

// ---------- Límite de control ----------
const float U_MAX = 50.0f;

// ---------- Funciones auxiliares ----------
int angToPWM(float angRef) {
  angRef = constrain(angRef, 0, 180);
  long us = map((int)angRef, 0, 180, PWM_MIN, PWM_MAX);
  return constrain(us, PWM_MIN, PWM_MAX);
}

void moverServo_deg(float angRef) {
  servo.writeMicroseconds(angToPWM(angRef));
}

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

  float theta_acc_deg = atan2(ay_g, az_g) * 180.0f / PI - accel_bias_deg;
  float gx_dps = ((float)gx - gyro_bias_gx_LSB) / 131.0f;

  unsigned long now_ms = millis();
  float dt_cf = (last_cf_ms == 0) ? Ts_s : (now_ms - last_cf_ms) / 1000.0f;
  if (dt_cf <= 0.0f || dt_cf > 0.1f) dt_cf = Ts_s;

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

  Serial.println(F("t_s,theta_deg,y_hat,u"));
  last_cf_ms = millis();
}

// ---------- Loop principal ----------
void loop() {
  static unsigned long tiempoactual_1 = 0;
  unsigned long tiempoactual = millis();

  if (tiempoactual - tiempoactual_1 >= (unsigned long)(Ts_s * 1000)) {

    // --- 1. Leer IMU calibrada ---
    float y = leerThetaPenduloDeg();   // en grados

    // --- 2. Observador ---
    float y_hat = Cd[0]*x_hat[0] + Cd[1]*x_hat[1] + Cd[2]*x_hat[2] + Cd[3]*x_hat[3];
    float error = y - y_hat;
    float x_next[4];

    // --- 3. Control ---
    float u = 0.0f;
    for (int j = 0; j < 4; j++) u += -Kd[j] * x_hat[j];
    u = constrain(u, -U_MAX, U_MAX);

    // --- 4. Actuar servo ---
    float servo_deg = SERVO_CENTER_DEG + u;
    moverServo_deg(servo_deg);

    // --- 5. Actualizar observador ---
    for (int i = 0; i < 4; i++) {
      float Ax = 0.0f;
      for (int j = 0; j < 4; j++) Ax += Ad[i][j] * x_hat[j];
      x_next[i] = Ax + Bd[i]*u + Ld[i]*error;
    }
    for (int i = 0; i < 4; i++) x_hat[i] = x_next[i];

    // --- 6. Log ---
    Serial.print(millis()/1000.0, 3);
    Serial.print(",");
    Serial.print(y, 3);
    Serial.print(",");
    Serial.print(y_hat, 3);
    Serial.print(",");
    Serial.println(u, 3);

    // --- 7. Actualizar tiempos ---
    tiempoactual_1 = tiempoactual;
  }
}
