// === Lectura del potenciómetro + MPU6050 con filtro complementario ===
// Tomas Hirsch, 2025

#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// ---------- Pines ----------
const int POT_PIN = A0;

// ---------- Calibración potenciómetro ----------
const float A_cal = -0.261627;
const float B_cal = 251.16192;

// ---------- Filtro complementario ----------
float alpha = 0.98;     // peso del giroscopio
float dt = 0.01;        // periodo de muestreo (s)
unsigned long prevTime = 0;
float angle_gyro = 0.0; // ángulo estimado por integración del giroscopio
float angle = 0.0;      // salida final del filtro

// ---------- Función: conversión ADC → ángulo ----------
float potToAngle(int adc) {
  return A_cal * adc + B_cal;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("Error: no se detecta el MPU6050.");
    while (1);
  }

  Serial.println("=== Lectura de potenciómetro + MPU6050 ===");
  Serial.println("Ángulo brazo principal (pot) y brazo colgante (MPU6050)");
  Serial.println("==============================================\n");

  prevTime = millis();
}

void loop() {
  // ---------- 1. Leer potenciómetro ----------
  int adc = analogRead(POT_PIN);
  float ang_principal = potToAngle(adc);

  // ---------- 2. Leer MPU6050 ----------
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Acelerómetro → ángulo absoluto (en grados)
  // Suponiendo eje X como el eje del brazo colgante
  float angle_acc = atan2(ay, az) * 180.0 / PI;

  // Giroscopio → velocidad angular (en °/s)
  float gyroRate = gx / 131.0;  // sensibilidad típica ±250°/s

  // Calcular dt real
  unsigned long now = millis();
  dt = (now - prevTime) / 1000.0;
  prevTime = now;

  // Filtro complementario
  angle_gyro += gyroRate * dt;
  angle = alpha * (angle + gyroRate * dt) + (1 - alpha) * angle_acc;

  // ---------- 3. Mostrar resultados ----------
  Serial.print("Pot ADC: ");
  Serial.print(adc);
  Serial.print("\tBrazo Principal: ");
  Serial.print(ang_principal, 1);
  Serial.print("°\tBrazo Colgante (MPU): ");
  Serial.print(angle, 1);
  Serial.println("°");

  delay(10); // 100 Hz aprox.
}

