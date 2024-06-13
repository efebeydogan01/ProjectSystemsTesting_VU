#include <math.h>
#include "Adafruit_LiquidCrystal.h"

// NOTE: Always start the prgoram with the switch 1 down and birng back the syringes to most back, then turn switch 1 up
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
// D4 -> SW1
// D5 -> SW2
// D6 -> SW3
// D7 -> SW4

// For the Motors (COMPRESS = PUSH, DECOMPRESS = PULL):
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
#define maxSize 5000
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
#define STEPS_BY_HAND 2000
#define STEPS_BY_HAND_SETUP_SUCK 1000
#define STEPS_BY_HAND_NORMAL_SUCK 42000

#define MOTOR_A 0
#define MOTOR_B 1

#define FRESH_PUMP 0
#define SALT_PUMP 1

#define COMPRESS 0
#define DECOMPRESS 1

#define TEMP_LOWER_BOUND 30
#define TEMP_UPPER_BOUND 40

#define SAL_LOWER_BOUND 10
#define SAL_UPPER_BOUND 20

#define MAX_STEPS 42000
#define ONE_STEP_VOL 35 / 42000.0  //mililitres

#define INITIAL_BEAKER_VOL 500
#define MAX_VOL_ADDITION 300

#define HEATER_ENABLED true

#define CALIBRATION_DURATION 15

// Connect via i2c, default address #0 (A0-A2 not jumpered)
Adafruit_LiquidCrystal lcd(0);

long tempSize = 0;
long salSize = 0;
long salVOutSize = 0;
double* runningAvgTemp = new double[maxSize];
double* runningAvgSal = new double[maxSize];
double* runningAvgSalVout = new double[maxSize];

// int freshCyclesForward = 0;
// int freshCyclesBackwards = 0;
// int salineCyclesForward = 0;
// int salineCyclesBackwards = 0;
int freshPumpPosition = 0;
int salinePumpPosition = 0;
double curVolume = 0;
bool salinityOutOfRange = false;
bool isPumpAtLimit = false;

double salFormulaA = 15.1747217178;
double salFormulaB = -2.89491343498;

void cleanLCDArea(int rowPos, int row, int numSpaces) {
  char* buf = new char[numSpaces + 1];
  for (int i = 0; i < numSpaces; i++) {
    buf[i] = ' ';
  }
  buf[numSpaces] = '\0';
  lcd.setCursor(rowPos, row);
  lcd.print(buf);
  delete[] buf;
}

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
  // double salinity = 15.1747217178 * Vout - 2.89491343498;  // more accurate way
  // if (salinity < 0) salinity = 0;
  // double salinity = 16.3 * Vout;  // less accurate way
  double salinity = salFormulaA * Vout + salFormulaB;  // more accurate way with calibration
  if (salinity < 0) salinity = 0;
  return salinity;
}


void readRawSalinity() {
  int salSensorVal = analogRead(SAL_SENSOR);
  double Varduino = salSensorVal * (3.3 / 1024.0);  // convert to voltage (0-3.3V)
  double Vout = Varduino * (5.0 / 3.0);
  // double salinity = getSalinity(salSensor);
  addToArr(runningAvgSalVout, Vout, salVOutSize);
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
  // String formatSal = String(curSal, 1);
  lcd.setCursor(7, 0);
  lcd.print("|S: ");
  lcd.print(curSal);
  lcd.print(" ");
  Serial.print("SW3-4: ");
  Serial.print(isSwitch3or4Down());
  Serial.print(" SW1: ");
  Serial.println(isSwitch1Down());

  if (!isSwitch3or4Down() && !isSwitch1Down()) {
    Serial.println("IF");
    if (curSal < SAL_LOWER_BOUND) {
      // lcd.setCursor(7, 0);
      // lcd.print("|Sal low!");
      if (!isPumpAtLimit) {
        lcd.setCursor(7, 1);
        lcd.print("|Add salt");
      }
      // delay(2000);

      if (!salinityOutOfRange) {
        lcd.setCursor(7, 0);
        lcd.print("|ADJ VALV");
        delay(5000);
        salinityOutOfRange = true;
      }

      activate_motor(MOTOR_B, COMPRESS, NUMBER_OF_STEPS);
      // activate_motor(MOTOR_B, DECOMPRESS, NUMBER_OF_STEPS);
    } else if (curSal >= SAL_LOWER_BOUND && curSal <= SAL_UPPER_BOUND) {
      salinityOutOfRange = false;
    } else {
      // lcd.setCursor(7, 0);
      // lcd.print("|Sal hi!");
      if (!isPumpAtLimit) {
        lcd.setCursor(7, 1);
        lcd.print("|Add fres");
      }
      // delay(2000);

      if (!salinityOutOfRange) {
        lcd.setCursor(7, 0);
        lcd.print("|ADJ VALV");
        delay(5000);
        salinityOutOfRange = true;
      }


      // activate_motor(MOTOR_A, DECOMPRESS, NUMBER_OF_STEPS);
      // delay(1000);
      activate_motor(MOTOR_A, COMPRESS, NUMBER_OF_STEPS);
    }
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

  if (HEATER_ENABLED) {
    if (curTemp < TEMP_LOWER_BOUND) {
      // lcd.setCursor(0, 0);
      // lcd.print("Tmp low");
      lcd.setCursor(0, 1);
      lcd.print("Htr on ");
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
  }
  // delay(2000);
}

void readAndPrintTemp() {
  double tempAverage = readTemp();

  lcd.setCursor(0, 0);
  lcd.print("Temperat: ");
  lcd.print(tempAverage);
  Serial.print("Temperature: ");
  Serial.println(tempAverage, 3);
}


void calibrateSalSensor() {
  // prompt user for 15 seconds to dip sensor in 5ppt
  lcd.setCursor(0, 0);
  lcd.print("Dip sensor in");

  for (int i = CALIBRATION_DURATION; i > 0; i--) {
    cleanLCDArea(0, 1, 16);
    lcd.setCursor(0, 1);
    lcd.print("5ppt within ");
    lcd.print(i);
    lcd.print("\"");
    delay(1000);  // Delay for 1 second
  }
  lcd.clear();


  lcd.clear();
  lcd.print("Reading...");

  // read 30 Vout values
  for (int i = 0; i < maxSize; i++) {
    readRawSalinity();
  }
  // testing
  // for (int j = 0; j < maxSize; j++) {
  //   Serial.print(runningAvgSalVout[j]);
  //   Serial.print(" ");
  // }
  // Serial.println();
  double salVoutAverage5ppt = calculateAvg(runningAvgSalVout, salVOutSize);
  delay(2000);
  // Serial.print("VOUT 5: ");
  // Serial.println(salVoutAverage5ppt);

  // prompt user for 15 seconds to dip sensor in 15ppt
  lcd.setCursor(0, 0);
  lcd.print("Dip sensor in");

  for (int i = CALIBRATION_DURATION; i > 0; i--) {
    cleanLCDArea(0, 1, 16);
    lcd.setCursor(0, 1);
    lcd.print("15ppt within ");
    lcd.print(i);
    lcd.print("\"");
    delay(1000);  // Delay for 1 second
  }
  lcd.clear();

  lcd.clear();
  lcd.print("Reading...");

  // read 30 Vout values
  for (int i = 0; i < maxSize; i++) {
    readRawSalinity();
  }
  // testing
  // for (int j = 0; j < maxSize; j++) {
  //   Serial.print(runningAvgSalVout[j]);
  //   Serial.print(" ");
  // }
  // Serial.println();
  double salVoutAverage15ppt = calculateAvg(runningAvgSalVout, salVOutSize);
  delay(2000);
  // Serial.print("VOUT 15: ");
  // Serial.println(salVoutAverage15ppt);

  // calculate a and b values for the salinity formula
  salFormulaA = 10 / (salVoutAverage15ppt - salVoutAverage5ppt);
  salFormulaB = 15 - salFormulaA * salVoutAverage15ppt;

  Serial.print("SAL A: ");
  Serial.print(salFormulaA);
  Serial.print(" SAL B: ");
  Serial.println(salFormulaB);

  // prompt user for 15 seconds to replace beaker
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Calibration ok");
  // lcd.setCursor(0, 1);
  // lcd.print("Repl beak in 15\"");
  // delay(15000);

  for (int i = CALIBRATION_DURATION; i > 0; i--) {
    cleanLCDArea(0, 1, 16);
    lcd.setCursor(0, 1);
    lcd.print("Repl beak in ");
    lcd.print(i);
    lcd.print("\"");
    delay(1000);  // Delay for 1 second
  }
  lcd.clear();

  lcd.clear();
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

  if (!HEATER_ENABLED)
    digitalWrite(HEATER, LOW);

  delay(1000);
  calibrateSalSensor();
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

bool isSwitch1Down() {
  return digitalRead(SWITCH1);
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

bool checkLimitsValid(bool motorId, bool direction, int number_of_steps) {
  if (!motorId) {
    if (direction == COMPRESS && freshPumpPosition + number_of_steps >= MAX_STEPS) {
      return false;
    } else if (direction == DECOMPRESS && freshPumpPosition - number_of_steps <= 0) {
      return false;
    }
  } else {
    if (direction == COMPRESS && salinePumpPosition + number_of_steps >= MAX_STEPS) {
      return false;
    } else if (direction == DECOMPRESS && salinePumpPosition - number_of_steps <= 0) {
      return false;
    }
  }
  return true;
}

void calculatePositions(bool motorId, bool direction, int number_of_steps) {
  if (!motorId) {
    if (direction == COMPRESS) {
      // freshCyclesForward += number_of_steps;
      freshPumpPosition += number_of_steps;
    } else {
      // freshCyclesBackwards += number_of_steps;
      freshPumpPosition -= number_of_steps;
    }
  } else {
    if (direction == COMPRESS) {
      // salineCyclesForward += number_of_steps;
      salinePumpPosition += number_of_steps;
    } else {
      // salineCyclesBackwards += number_of_steps;
      salinePumpPosition -= number_of_steps;
    }
  }
}

void printLimit() {
  lcd.setCursor(7, 1);
  lcd.print("|LIMIT   ");
  delay(2000);
  cleanLCDArea(7, 1, 9);
}

void activate_motor(bool motorId, bool direction, int number_of_steps) {
  // Serial.print(motorId);
  // Serial.println(direction);
  if (!isSwitch1Down()) {
    if (!checkLimitsValid(motorId, direction, number_of_steps)) {
      if (motorId == FRESH_PUMP) {
        if (direction == COMPRESS) {
          number_of_steps = MAX_STEPS - freshPumpPosition;
        } else {
          number_of_steps = freshPumpPosition;
        }
      } else {
        if (direction == COMPRESS) {
          number_of_steps = MAX_STEPS - salinePumpPosition;
        } else {
          number_of_steps = salinePumpPosition;
        }
      }
    }
  }

  if (number_of_steps == 0) {
    isPumpAtLimit = true;
    printLimit();
    return;
  }
  isPumpAtLimit = false;

  if (direction == COMPRESS && curVolume + ONE_STEP_VOL * number_of_steps >= MAX_VOL_ADDITION) {
    lcd.setCursor(7, 0);
    lcd.print("|OVERFLOW");
    return;
  }

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

  if (!isSwitch1Down()) {
    calculatePositions(motorId, direction, number_of_steps);
    if (direction == COMPRESS)
      curVolume += ONE_STEP_VOL * number_of_steps;
  }
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

bool isSwitch3or4Down() {
  return !digitalRead(SWITCH3) || !digitalRead(SWITCH4);
}

void operateMotorSuckWithSwitches3and4() {
  lcd.setCursor(0, 0);
  if (!digitalRead(SWITCH3)) {  // sw3 press
                                // lcd.print("SW3 HIGH");
                                // Enable motor
    // digitalWrite(MOT_A_EN, LOW);
    if (!isSwitch1Down())
      activate_motor(MOTOR_A, DECOMPRESS, STEPS_BY_HAND_NORMAL_SUCK);
    else
      activate_motor(MOTOR_A, DECOMPRESS, STEPS_BY_HAND_SETUP_SUCK);
    salinityOutOfRange = false;
  }

  if (!digitalRead(SWITCH4)) {  // sw4 press
                                // lcd.print("SW4 HIGH");
    // digitalWrite(MOT_B_EN, LOW);
    if (!isSwitch1Down())
      activate_motor(MOTOR_B, DECOMPRESS, STEPS_BY_HAND_NORMAL_SUCK);
    else
      activate_motor(MOTOR_B, DECOMPRESS, STEPS_BY_HAND_SETUP_SUCK);
    salinityOutOfRange = false;
  }
}

void operateMotorPumpWithSwitches3and4() {
  // lcd.setCursor(0, 0);
  if (!digitalRead(SWITCH3)) {  // sw3 press
                                // lcd.print("SW3 HIGH");
                                // Enable motor
    int enablePin = MOT_A_EN;
    digitalWrite(enablePin, LOW);
    activate_motor(MOTOR_A, COMPRESS, STEPS_BY_HAND);
    salinityOutOfRange = false;
  } else {
    // Disable motor
    int enablePin = MOT_A_EN;
    digitalWrite(enablePin, HIGH);
  }
  if (!digitalRead(SWITCH4)) {  // sw4 press
                                // lcd.print("SW4 HIGH");
    int enablePin = MOT_B_EN;
    digitalWrite(enablePin, LOW);
    activate_motor(MOTOR_B, COMPRESS, STEPS_BY_HAND);
    salinityOutOfRange = false;
  } else {
    int enablePin = MOT_B_EN;
    digitalWrite(enablePin, HIGH);
  }
}

void checkBringBackSyringes() {
  if (freshPumpPosition == MAX_STEPS) {
    salinityOutOfRange = false;
    lcd.setCursor(7, 0);
    lcd.print("|ADJ VALV");
    delay(5000);
    lcd.setCursor(7, 1);
    lcd.print("|SUCKING ");
    activate_motor(MOTOR_A, DECOMPRESS, STEPS_BY_HAND_NORMAL_SUCK);
    cleanLCDArea(7, 0, 9);
    cleanLCDArea(7, 1, 9);
  }

  if (salinePumpPosition == MAX_STEPS) {
    salinityOutOfRange = false;
    lcd.setCursor(7, 0);
    lcd.print("|ADJ VALV");
    delay(5000);
    lcd.setCursor(7, 1);
    lcd.print("|SUCKING ");
    activate_motor(MOTOR_B, DECOMPRESS, STEPS_BY_HAND_NORMAL_SUCK);
    cleanLCDArea(7, 0, 9);
    cleanLCDArea(7, 1, 9);
  }
}

void loop() {
  // check temp
  tempOperations();

  // check salinity
  salinityOperations();

  // readAndPrintTemp(); // working (can be used as unit test)
  // readAndPrintSalinity();  // working (can be used as unit test)
  // readAndPrintSwitches3and4();  // working (can be used as unit test) -> will use for pumps
  if (digitalRead(SWITCH2)) {
    operateMotorPumpWithSwitches3and4();
  } else {
    operateMotorSuckWithSwitches3and4();
  }

  checkBringBackSyringes();

  // readAndPrintSwitch2();  // working (can be used as unit test) -> will use for heater
  // testMotorsOperation(); // working (can be used as unit test) TODO set it up to work with switches
  // testHeaterOperation(); // working with Switch 2 (can be used as unit test)
  // Serial.print("Fresh forw: ");
  // Serial.print(" Fresh back: ");
  // Serial.print(freshCyclesBackwards);
  // Serial.print(" Saline forw: ");
  // Serial.print(salineCyclesForward);
  // Serial.print(" Saline back: ");
  // Serial.println(salineCyclesBackwards);
  // Serial.print("Fresh pos: ");
  // Serial.print(freshPumpPosition);
  // Serial.print(" Saline pos: ");
  // Serial.print(salinePumpPosition);
  // Serial.print(" Current vol added: ");
  // Serial.println(curVolume);
}
