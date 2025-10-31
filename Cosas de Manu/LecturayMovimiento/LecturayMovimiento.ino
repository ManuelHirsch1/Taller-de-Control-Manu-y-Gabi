#include <Servo.h>
#include <Wire.h>
#include <MPU6050.h>

Servo servo;
MPU6050 mpu;

const int potPin = A0;

// Servo
const int PWM_MIN = 200;
const int PWM_MAX = 2800;

// Calibración potenciómetro → ángulo
const float A_cal = -0.466;
const float B_cal = 337.016296;

// Variables filtro complementario
float angPendulo = 0.0;
unsigned long tPrev = 0;
const float alpha = 0.98;   // peso del giroscopio

void setup() {
  Serial.begin(115200);
  servo.attach(6);

  Wire.begin();
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU6050 conectado correctamente.");
  } else {
    Serial.println("Error: MPU6050 no conectado.");
  }

  tPrev = millis();
}

int angToPWM(int angRef) {
  int pulse = map(angRef, 0, 180, 800, 2200);
  if (pulse < PWM_MIN) pulse = PWM_MIN;
  if (pulse > PWM_MAX) pulse = PWM_MAX;
  return pulse;
}

void moverServo(int angRef) {
  int pulse = angToPWM(angRef);
  servo.writeMicroseconds(pulse);
}

float angRealDesdeADC(int adc) {
  return A_cal * adc + B_cal;
}

void loop() {
  int angRef = 90;
  moverServo(angRef);

  // Potenciómetro
  int valor = analogRead(potPin);
  float angReal = angRealDesdeADC(valor);

  // MPU6050
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Tiempo entre muestras
  unsigned long tNow = millis();
  float dt = (tNow - tPrev) / 1000.0; // en segundos
  tPrev = tNow;

  // Acelerómetro → ángulo en grados
  float ax_g = (float)ax / 16384.0;
  float ay_g = (float)ay / 16384.0;
  float az_g = (float)az / 16384.0;
  float angAcc = atan2(ay_g, az_g) * 180.0 / PI;

  // Giroscopio → velocidad angular en °/s (escala: 131 LSB/°/s para ±250 dps)
  float gyroRate = (float)gx / 131.0;

  // Filtro complementario
  angPendulo = alpha * (angPendulo + gyroRate * dt) + (1 - alpha) * angAcc;

  // Imprimir datos
  Serial.print(angRef);     Serial.print(",");
  Serial.print(angReal);    Serial.print(",");
  Serial.println(angPendulo);

  delay(10); // muestreo ~100 Hz
}



