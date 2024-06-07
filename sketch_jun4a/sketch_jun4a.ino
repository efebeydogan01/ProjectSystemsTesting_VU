#include <math.h>
#include "Adafruit_LiquidCrystal.h"

// NOTE: we got a new box because SW3 pin had a part stuck inside

// Wire connections:

// For the Temperature sensor (blue cable):
// D20 -> A
// GND -> GND (on the right) (red cable)

// For the Salinity sensor (green cable):
// D17 -> B1
// GND -> GND (on the right) (red cable)

// For the Heater (purple cable):
// D2 -> HTR
// GND -> GND (on the right) (red cable)

// For the Switches (yellow cables):
// D5 -> SW1
// D5 -> SW2
// D6 -> SW3
// D7 -> SW4

// For the Motors:
// orange cables:
// D11 -> EN (MOTOR B)
// D12 -> STP (MOTOR B)
// D13 -> DIR (MOTOR B)
// D14 -> EN (MOTOR A)
// D15 -> STP (MOTOR A)
// D16 -> DIR (MOTOR A)
// white cables:
// GND -> M0 (MOTOR A)
// GND -> M1 (MOTOR A)
// GND -> M2 (MOTOR A)
// GND -> M0 (MOTOR B)
// GND -> M1 (MOTOR B)
// GND -> M2 (MOTOR B)
// grey cable:
// 3.3V -> SLP

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

#define LEDS 9
#define SWITCH1 4
#define SWITCH2 5
#define SWITCH3 6
#define SWITCH4 7
#define POT A3
#define BUZ 8
#define HEATER 2

#define MOT_B_EN 11
#define MOT_B_STP 12
#define MOT_B_DIR 13
#define MOT_A_EN 14
#define MOT_A_STP 15
#define MOT_A_DIR 16

#define STEP_DELAY 1000
#define NUMBER_OF_STEPS 10000

#define MOTOR_A 0
#define MOTOR_B 1

#define COMPRESS 0
#define DECOMPRESS 1

#define TEMP_LOWER_BOUND 30
#define TEMP_UPPER_BOUND 40

#define SAL_LOWER_BOUND 10
#define SAL_UPPER_BOUND 20

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
  } else {
    boundary = maxSize;
  }

  double res = 0;
  for (long i = 0; i < boundary; i++) {
    res += arr[i];
  }

  return res / boundary;
}

// SALINITY FUNCTIONS

double getSalinity(int salSensorVal) {
  double Varduino = salSensorVal * (3.3 / 1024.0);  // convert to voltage (0-3.3V)
  double Vout = Varduino * (5.0 / 3.0);
  double salinity = 15.1747217178 * Vout - 2.89491343498;  // more accurate way
  if (salinity < 0) salinity = 0;
  // double salinity = 16.3 * Vout; // less accurate way
  return salinity;
}

double readSalinity() {
  int salSensor = analogRead(SAL_SENSOR);
  double salinity = getSalinity(salSensor);
  addToArr(runningAvgSal, salinity, salSize);
  double salAverage = calculateAvg(runningAvgSal, salSize);

  return salAverage;
}

void readAndPrintSalinity() {
  double salAverage = readSalinity();

  // Serial.print("Salinity: ");
  // Serial.println(salAverage, 3);
  lcd.setCursor(0, 1);
  lcd.print("Salinity: ");
  lcd.print(salAverage);
}

void salinityOperations() {
  double curSal = readSalinity();
  String formatSal = String(curSal, 1);
  lcd.setCursor(7, 0);
  lcd.print("|S: ");
  lcd.print(formatSal);

  if (curSal < SAL_LOWER_BOUND) {
    // lcd.setCursor(7, 0);
    // lcd.print("|Sal low!");
    lcd.setCursor(7, 1);
    lcd.print("|Add salt");
    delay(2000);

    lcd.setCursor(7, 1);
    lcd.print("|ADJ VALV");
    delay(5000);

    activate_motor(MOTOR_B, COMPRESS, NUMBER_OF_STEPS);
    delay(2000);
    activate_motor(MOTOR_B, DECOMPRESS, NUMBER_OF_STEPS);
  } else if (curSal >= SAL_LOWER_BOUND && curSal <= SAL_UPPER_BOUND) {
    // lcd.setCursor(7, 0);
    // lcd.print("|Sal norm");
    delay(2000);
  } else {
    // lcd.setCursor(7, 0);
    // lcd.print("|Sal hi!");
    lcd.setCursor(7, 1);
    lcd.print("|Add fres");
    delay(2000);

    lcd.setCursor(7, 1);
    lcd.print("|ADJ VALV");
    delay(5000);

    activate_motor(MOTOR_A, DECOMPRESS, NUMBER_OF_STEPS);
    delay(1000);
    activate_motor(MOTOR_B, COMPRESS, NUMBER_OF_STEPS);
  }
}

// TEMPERATURE FUNCTIONS

double getTemp(int tempSensorVal) {
  double voltageOut = (tempSensorVal / 1024.0) * 3.3;
  double rt = voltageOut * R1 / (Vin * (1 - voltageOut / Vin));
  double T = 1.0 / (K0 + K1 * log(rt) + K2 * log(rt) * log(rt) * log(rt));
  double celsius = T - kelvin;
  return celsius;
}

double readTemp() {
  int tempSensor = analogRead(TEMP_SENSOR);
  double temperature = getTemp(tempSensor);
  addToArr(runningAvgTemp, temperature, tempSize);
  double tempAverage = calculateAvg(runningAvgTemp, tempSize);

  return tempAverage;
}

void tempOperations() {
  double curTemp = readTemp();
  String formatTemp = String(curTemp, 1);
  lcd.setCursor(0, 0);
  lcd.print("T: ");
  lcd.print(formatTemp);

  if (curTemp < TEMP_LOWER_BOUND) {
    // lcd.setCursor(0, 0);
    // lcd.print("Tmp low");
    lcd.setCursor(0, 1);
    lcd.print("Htr on");
    digitalWrite(HEATER, HIGH);
  } else if (TEMP_LOWER_BOUND <= curTemp && curTemp <= TEMP_UPPER_BOUND) {
    digitalWrite(HEATER, LOW);
    // lcd.setCursor(0, 0);
    // lcd.print("Tmp nrml");
    lcd.setCursor(0, 1);
    lcd.print("Htr off");
  } else {
    // lcd.setCursor(0, 0);
    // lcd.print("Tmp hi!");
    lcd.setCursor(0, 1);
    lcd.print("Add ice");
  }
  delay(2000);
}

void readAndPrintTemp() {
  double tempAverage = readTemp();

  lcd.setCursor(0, 0);
  lcd.print("Temperat: ");
  lcd.print(tempAverage);
  Serial.print("Temperature: ");
  Serial.println(tempAverage, 3);
}

void setup() {
  Serial.begin(115200);
  // while(!Serial);
  Serial.println("LCD Character Backpack I2C Test.");

  // set up the LCD's number of rows and columns:
  if (!lcd.begin(30, 2)) {
    Serial.println("Could not init backpack. Check wiring.");
    while (1)
      ;
  }
  Serial.println("Backpack init'd.");

  pinMode(LEDS, OUTPUT);
  pinMode(SWITCH1, INPUT);
  pinMode(SWITCH2, INPUT);
  pinMode(SWITCH3, INPUT);
  pinMode(SWITCH4, INPUT);
  pinMode(HEATER, OUTPUT);

  // pinMode(POT, INPUT);
  pinMode(BUZ, OUTPUT);
  digitalWrite(BUZ, LOW);
}

void readAndPrintSwitches3and4() {
  lcd.setCursor(0, 0);
  if (digitalRead(SWITCH3)) {
    lcd.print("SW3 HIGH");
  } else {
    lcd.print("SW3 LOW ");
  }
  if (digitalRead(SWITCH4)) {
    lcd.print("SW4 HIGH");
  } else {
    lcd.print("SW4 LOW ");
  }
  // Serial.print("Potentiometer: ");
  // Serial.println(analogRead(POT));
}

void testHeaterOperation() {
  lcd.setCursor(0, 1);
  if (digitalRead(SWITCH2)) {
    lcd.print("SW2 HIGH-HEAT ON");
    digitalWrite(HEATER, HIGH);
  } else {
    lcd.print("SW2 LOW-HEAT OFF");
    digitalWrite(HEATER, LOW);
  }
}

void readAndPrintSwitch2() {
  lcd.setCursor(0, 0);
  if (digitalRead(SWITCH2)) {
    lcd.print("SW2 HIGH");
  } else {
    lcd.print("SW2 LOW ");
  }
}

void activate_motor(bool motorId, bool direction, int number_of_steps) {
  int enablePin = motorId ? MOT_B_EN : MOT_A_EN;
  int stepPin = motorId ? MOT_B_STP : MOT_A_STP;
  int directionPin = motorId ? MOT_B_DIR : MOT_A_DIR;

  // Enable motor
  digitalWrite(enablePin, LOW);

  // Begin with (de)compressing
  if (motorId) {
    digitalWrite(directionPin, !direction);
  } else {
    digitalWrite(directionPin, direction);
  }
  for (int i = 0; i < number_of_steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(STEP_DELAY);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(STEP_DELAY);
  }

  // Disable motor
  digitalWrite(enablePin, HIGH);
}

void testMotorsOperation() {
  delay(5000);
  Serial.println("Comp Mot A");
  activate_motor(MOTOR_A, COMPRESS, NUMBER_OF_STEPS);
  delay(1000);
  Serial.println("Decomp Mot A");
  activate_motor(MOTOR_A, DECOMPRESS, NUMBER_OF_STEPS);
  delay(1000);
  Serial.println("Comp Mot B");
  activate_motor(MOTOR_B, COMPRESS, NUMBER_OF_STEPS);
  delay(1000);
  Serial.println("Decomp Mot B");
  activate_motor(MOTOR_B, DECOMPRESS, NUMBER_OF_STEPS);
}

void loop() {
  // check temp
  tempOperations();

  // check salinity
  salinityOperations();

  // readAndPrintTemp(); // working (can be used as unit test)
  // readAndPrintSalinity();  // working (can be used as unit test)
  // readAndPrintSwitches3and4();  // working (can be used as unit test) -> will use for pumps
  // readAndPrintSwitch2();  // working (can be used as unit test) -> will use for heater
  // testMotorsOperation(); // working (can be used as unit test) TODO set it up to work with switches
  // testHeaterOperation(); // working with Switch 2 (can be used as unit test)
}
