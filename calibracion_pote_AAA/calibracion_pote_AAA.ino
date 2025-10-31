// === Lectura del potenciómetro para calibración manual ===
// Tomas Hirsch, 2025

const int POT_PIN = A0;  // pin donde está conectado el potenciómetro

void setup() {
  Serial.begin(115200);
  Serial.println("=== Calibración manual del potenciómetro ===");
  Serial.println("Mové el brazo y observá el valor ADC en el monitor serie.");
  Serial.println("Anotá los valores para dos ángulos conocidos (ej: 0° y 180°).");
  Serial.println("Luego calculá A y B con las fórmulas:");
  Serial.println("A = (ang2 - ang1) / (adc2 - adc1)");
  Serial.println("B = ang1 - A * adc1");
  Serial.println("===========================================\n");
}

void loop() {
  int adc = analogRead(POT_PIN);
  Serial.print("ADC = ");
  Serial.println(adc);
  delay(200);  // refresca cada 0.2 segundos
}

