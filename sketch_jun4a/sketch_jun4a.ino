#include <math.h>
#include "Adafruit_LiquidCrystal.h"

#define TEMP_SENSOR A6
#define SAL_SENSOR A3

#define Vin 3.3
#define R1 15000
#define K0 0.00102119
#define K1 0.000222468
#define K2 1.33342e-7
#define kelvin 273.15
#define maxSize 30

// Connect via i2c, default address #0 (A0-A2 not jumpered)
Adafruit_LiquidCrystal lcd(0);

long curSize = 0;
double runningAvg[maxSize];

void addToArr(double temperature) {
  runningAvg[curSize % maxSize] = temperature;
  curSize++;
}

double calculateAvg() {
  long boundary = 0;
  if (curSize < 30) {
    boundary = curSize;
  }
  else {
    boundary = maxSize;
  }

  double res = 0;
  for (long i = 0; i < boundary; i++) {
    res += runningAvg[i];
  }

  return res / boundary;
}

double getTemp(int tempSensorVal) {
  double voltageOut = (tempSensorVal / 1024.0) * 3.3;
  double rt = voltageOut * R1 / (Vin * (1 - voltageOut / Vin));
  double T = 1.0 / (K0 + K1 * log(rt) + K2 * log(rt) * log(rt) * log(rt));
  double celsius = T - kelvin;
  return celsius;
}

void setup() {
  Serial.begin(115200);
  // while(!Serial);
  Serial.println("LCD Character Backpack I2C Test.");

  // set up the LCD's number of rows and columns:
  if (!lcd.begin(16, 2)) {
    Serial.println("Could not init backpack. Check wiring.");
    while(1);
  }
  Serial.println("Backpack init'd.");
}

void loop() {
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(0, 1);

  int tempSensor = analogRead(TEMP_SENSOR);
  double temperature = getTemp(tempSensor);
  addToArr(temperature);
  double tempAverage = calculateAvg();
  lcd.print(tempAverage);
  Serial.print("Temperature: ");
  Serial.println(tempAverage, 3);
  delay(500);
}