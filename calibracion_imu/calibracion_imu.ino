#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

const int N_SAMPLES = 500;
const float G_REF = 9.80665;

float bias[3];   // bias x,y,z (m/s^2)
float gain[3];   // gain x,y,z (adimensional)

void setup() {
  Serial.begin(115200);
  while(!Serial) delay(10);
  Serial.println("Inicializando MPU6050...");

  if (!mpu.begin()) {
    Serial.println("No se encontró el MPU6050, verifique conexiones!");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);
  calibrar6Posiciones();
  Serial.println("Calibracion lista. Iniciando medidas corregidas...");
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float raw[3] = { a.acceleration.x, a.acceleration.y, a.acceleration.z };
  // aplicar correccion por eje: primero restar bias luego multiplicar por gain
  float corr[3];
  for (int i=0;i<3;i++){
    corr[i] = (raw[i] - bias[i]) * gain[i];
  }
  float norm = sqrt(corr[0]*corr[0] + corr[1]*corr[1] + corr[2]*corr[2]);

  Serial.print("CORR_AX: "); Serial.print(corr[0],6);
  Serial.print("  CORR_AY: "); Serial.print(corr[1],6);
  Serial.print("  CORR_AZ: "); Serial.print(corr[2],6);
  Serial.print("  NORM: "); Serial.println(norm,6);

  delay(300);
}

/* ---------- funciones ---------- */

void calibrar6Posiciones(){
  // orden de posiciones: +X, -X, +Y, -Y, +Z, -Z
  const char* instrucciones[6] = {
    "+X hacia arriba (X apunta al cielo). Luego presiona ENTER.",
    "-X hacia arriba (X apunta al suelo/invertido). Luego ENTER.",
    "+Y hacia arriba. Luego ENTER.",
    "-Y hacia arriba. Luego ENTER.",
    "+Z hacia arriba. Luego ENTER.",
    "-Z hacia arriba. Luego ENTER."
  };

  float medias[6][3];

  for (int p=0; p<6; p++){
    Serial.println("======================================");
    Serial.print("Posicion "); Serial.print(p+1); Serial.print("/6: ");
    Serial.println(instrucciones[p]);
    Serial.println("Esperando ENTER...");
    esperarEnter();

    // muestreo
    long sumx=0, sumy=0, sumz=0;
    for (int i=0;i<N_SAMPLES;i++){
      sensors_event_t a,g,temp;
      mpu.getEvent(&a,&g,&temp);
      // a.acceleration está en m/s^2 según Adafruit
      // sumamos como float multiplicado por 1000 para evitar overflow en long?
      // pero valores ~10 m/s2 * 500 = 5000, cabe en long sin problema
      sumx += (long) round(a.acceleration.x * 1000.0);
      sumy += (long) round(a.acceleration.y * 1000.0);
      sumz += (long) round(a.acceleration.z * 1000.0);
      delay(5);
    }
    // medias en m/s^2 (deshacer factor 1000)
    medias[p][0] = (sumx / (float)N_SAMPLES) / 1000.0;
    medias[p][1] = (sumy / (float)N_SAMPLES) / 1000.0;
    medias[p][2] = (sumz / (float)N_SAMPLES) / 1000.0;

    Serial.print("Media (m/s^2): ");
    Serial.print(medias[p][0],6); Serial.print(" ");
    Serial.print(medias[p][1],6); Serial.print(" ");
    Serial.println(medias[p][2],6);
    delay(500);
  }

  // asignar lecturas
  float raw_px_x = medias[0][0], raw_px_y = medias[0][1], raw_px_z = medias[0][2];
  float raw_nx_x = medias[1][0], raw_nx_y = medias[1][1], raw_nx_z = medias[1][2];

  float raw_py_x = medias[2][0], raw_py_y = medias[2][1], raw_py_z = medias[2][2];
  float raw_ny_x = medias[3][0], raw_ny_y = medias[3][1], raw_ny_z = medias[3][2];

  float raw_pz_x = medias[4][0], raw_pz_y = medias[4][1], raw_pz_z = medias[4][2];
  float raw_nz_x = medias[5][0], raw_nz_y = medias[5][1], raw_nz_z = medias[5][2];

  // calcular bias y scale por eje usando sólo la componente que interesa:
  // para eje X usamos las lecturas donde X apunta +/- g (miramos la componente X)
  bias[0] = (raw_px_x + raw_nx_x) / 2.0;
  float scale_x = (raw_px_x - raw_nx_x) / 2.0; // debe ser aproximadamente G_REF
  gain[0] = G_REF / scale_x;

  bias[1] = (raw_py_y + raw_ny_y) / 2.0;
  float scale_y = (raw_py_y - raw_ny_y) / 2.0;
  gain[1] = G_REF / scale_y;

  bias[2] = (raw_pz_z + raw_nz_z) / 2.0;
  float scale_z = (raw_pz_z - raw_nz_z) / 2.0;
  gain[2] = G_REF / scale_z;

  Serial.println("----- RESULTADOS CALIBRACION -----");
  Serial.print("bias_x (m/s2): "); Serial.println(bias[0],6);
  Serial.print("bias_y (m/s2): "); Serial.println(bias[1],6);
  Serial.print("bias_z (m/s2): "); Serial.println(bias[2],6);
  Serial.print("gain_x: "); Serial.println(gain[0],6);
  Serial.print("gain_y: "); Serial.println(gain[1],6);
  Serial.print("gain_z: "); Serial.println(gain[2],6);
  Serial.println("----------------------------------");
}

// Espera ENTER en el monitor serie
void esperarEnter(){
  while (true){
    if (Serial.available()){
      char c = Serial.read();
      // aceptar cualquier caracter como "continuar"
      return;
    }
    delay(10);
  }
}



