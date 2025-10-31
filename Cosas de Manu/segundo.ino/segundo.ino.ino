#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

unsigned long lastTime = 0;  
float anguloGyroX = 0.0;   // integración del giroscopio

void setup() {
  Serial.begin(115200);

  if (!mpu.begin()) {
    Serial.println("No se encontró el MPU6050, verifique conexiones!");
    while (1) {
      delay(10);
    }
  }

  // Configurar rangos
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("Ax, Ay, Az, Gx, Gy, Gz, Angulo_GyroX, Angulo_AccX");
  
  lastTime = millis();
}

void loop() {
  // Lectura de eventos
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;  // Δt en segundos
  lastTime = currentTime;

  // 1. Aceleraciones (m/s^2)
  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  // 2. Giroscopios (deg/s)
  float gyroX_deg_s = g.gyro.x * RAD_TO_DEG;
  float gyroY_deg_s = g.gyro.y * RAD_TO_DEG;
  float gyroZ_deg_s = g.gyro.z * RAD_TO_DEG;

  // 3. Ángulo en eje X con giroscopio (integración)
  anguloGyroX += gyroX_deg_s * dt;

  // 4. Ángulo en eje X con acelerómetro
  float anguloAccX = atan2(ay, az) * RAD_TO_DEG;

  // Enviar por Serial (CSV)
  //Serial.print(ax); Serial.print(",");
  //Serial.print(ay); Serial.print(",");
  //Serial.print(az); Serial.print(",");
  //Serial.print(gyroX_deg_s); Serial.print(",");
  //Serial.print(gyroY_deg_s); Serial.print(",");
  //Serial.print(gyroZ_deg_s); Serial.print(",");
  //Serial.print(anguloGyroX); Serial.print(",");
  Serial.println(anguloAccX);

  delay(10); // ≈100 Hz
}


