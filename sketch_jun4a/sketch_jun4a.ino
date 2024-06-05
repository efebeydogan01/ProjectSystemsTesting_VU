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
#define salConst1 15.1747217178
#define salConst2 2.89491343498

// Connect via i2c, default address #0 (A0-A2 not jumpered)
Adafruit_LiquidCrystal lcd(0);

long tempSize = 0;
long salSize = 0;
double runningAvgTemp[maxSize];
double runningAvgSal[maxSize];

void addToArr(double* arr, double val, long& size) {
  arr[size % maxSize] = val;
  size = size + 1;
}

double calculateAvg(double* arr, long size) {
  long boundary = 0;
  if (size < 30) {
    boundary = size;
  }
  else {
    boundary = maxSize;
  }

  double res = 0;
  for (long i = 0; i < boundary; i++) {
    res += arr[i];
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

double getSalinity(int salSensorVal) {
  double Vout = salSensorVal * 5.0/3;
  double salinity = Vout * 16.3;
  return salinity;
}

void readAndPrintTemp() {
  int tempSensor = analogRead(TEMP_SENSOR);
  double temperature = getTemp(tempSensor);
  addToArr(runningAvgTemp, temperature, tempSize);
  double tempAverage = calculateAvg(runningAvgTemp, tempSize);

  lcd.setCursor(0, 1);
  lcd.print("Temperature: ");
  lcd.print(tempAverage);
  Serial.print("Temperature: ");
  Serial.println(tempAverage, 3);
}

void readAndPrintSalinity() {
  int salSensor = analogRead(SAL_SENSOR);
  double salinity = getSalinity(salSensor);
  addToArr(runningAvgSal, salinity, salSize);
  double salAverage = calculateAvg(runningAvgSal, salSize);

  Serial.print("Salinity: ");
  Serial.println(salAverage, 3);
  lcd.setCursor(0, 0);
  lcd.print("Salinity: ");
  lcd.print(salAverage);
}

void setup() {
  Serial.begin(115200);
  // while(!Serial);
  Serial.println("LCD Character Backpack I2C Test.");

  // set up the LCD's number of rows and columns:
  if (!lcd.begin(30, 2)) {
    Serial.println("Could not init backpack. Check wiring.");
    while(1);
  }
  Serial.println("Backpack init'd.");
}

void loop() {
  // readAndPrintTemp();
  readAndPrintSalinity();

  delay(500);
}