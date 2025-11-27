// === CÓDIGO DE CALIBRACIÓN DE POTENCIÓMETRO ===
// Lee el valor ADC y calcula el ángulo usando la ecuación lineal.
// Permite verificar linealidad y sentido de giro en el Serial Monitor o Plotter.

const int POT_PIN = A0;

// --- Calibración inicial ---
// (estos son los coeficientes que vas a ajustar)
const float A_cal = -0.27523f;
const float B_cal = 232.5688f;

float potToAngle(int adc) {
  return A_cal * adc + B_cal;
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println(F("# === Calibración de potenciómetro ==="));
  Serial.println(F("# Mueve lentamente el brazo y observa los valores."));
  Serial.println(F("# Formato: ADC, ángulo_deg"));
  Serial.println(F("ADC,phi_deg"));
}

void loop() {
  int adc = analogRead(POT_PIN);
  float phi = potToAngle(adc);

  // Imprimir en formato CSV
  Serial.print(adc);
  Serial.print(",");
  Serial.println(phi, 3);

  delay(50); // ~20 Hz de actualización, suave para el Plotter
}
