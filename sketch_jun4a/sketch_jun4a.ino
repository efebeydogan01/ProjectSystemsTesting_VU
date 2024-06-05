#include <math.h>

#define TEMP_SENSOR A6
#define Vin 3.3
#define R1 15000
#define K0 0.00102119
#define K1 0.000222468
#define K2 1.33342e-7
#define kelvin 273.15

double getTemp(int tempSensorVal) {
  double voltageOut = (tempSensorVal / 1024.0) * 3.3;
  double rt = voltageOut * R1 / (Vin * (1 - voltageOut / Vin));
  double T = 1.0 / (K0 + K1 * log(rt) + K2 * log(rt) * log(rt) * log(rt));
  double celsius = T - kelvin;
  return celsius;
}

void setup() {
}

void loop() {
  // char strBuf[50];
  // sprintf(strBuf, "Temperature: %.3f", getTemp(analogRead(TEMP_SENSOR)));
  int tempSensor = analogRead(TEMP_SENSOR);
  double temperature = getTemp(tempSensor);
  Serial.print("Temperature: ");
  Serial.println(temperature, 3);
  delay(500);
}