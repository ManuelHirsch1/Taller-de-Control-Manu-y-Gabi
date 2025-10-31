#include <Servo.h>
Servo servo;
const int potPin = A0;

int refs[] = {30, 150};   // elegí tus dos referencias
const int N = 400;         // muestras para promedio

int angToPWM(int ang){ 
  return constrain(map(ang, 0, 180, 800, 2200), 900, 2100); 
}

void setup(){
  Serial.begin(115200);
  servo.attach(9);
  delay(500);
  int adcVals[2];   

  for(int i=0; i<2; i++){
    servo.writeMicroseconds(angToPWM(refs[i]));
    delay(800); // esperar que asiente
    long acc = 0;
    for(int k=0; k<N; k++){ 
      acc += analogRead(potPin); 
      delay(2); 
    }
    adcVals[i] = acc / N;
    Serial.print("Ref "); Serial.print(refs[i]);
    Serial.print(" -> ADC promedio = "); 
    Serial.println(adcVals[i]);
  }

  // Suponiendo que ANG_1=refs[0] y ANG_2=refs[1]
  float ANG1 = refs[0], ANG2 = refs[1];
  float A = (ANG2 - ANG1) / float(adcVals[1] - adcVals[0]);
  float B = ANG1 - A * adcVals[0];

  Serial.print("Calibracion: A = "); Serial.println(A, 6);
  Serial.print("Calibracion: B = "); Serial.println(B, 6);
  Serial.println("Usa: angReal = A*ADC + B");
}

void loop(){}
