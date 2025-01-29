/*
  EVA-RT LITE v1.0 - main code for robotracer EVA-RT series
  
    Created on: Jan, 2025
    Edited by Mauricio Tovar
  
      DRIVERS:            IXF9201SG
      NUMBER OF SENSORS:  12A/3s
      MICROCONTROLLER:    ATMEGA328P-AU
      BATTERY:            2S
      CONTROL:            PD
      MOTORS:             1020

  This code is the simplified version of the program used for
  the All Chile Robot Contest 2025. Conceived and designed to
  facilitate the reading and understanding of the fundamental
  concepts.

  This code consists of the base control of the robot using the
  functions included in the Arduino core library. It includes
  the reading of the side sensors for detecting the goal marker.

  Copyright (C) 2025 EXOTIC TEAM MX

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as published
    by the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

  This code is part of the EVA-RT Github repository. See more
  in the next link:
  
  https://github.com/EXOTIC-TEAM-MX/EVA-RT
*/

//  ============================
//  U S E R   V A R I A B L E S
//  ============================

#define KP 0.04  //  proportional component coefficient (0.04)
#define KD 0.6   //  derivative component coefficient (0.6)

#define SPEED 60       //  base PWM motors speed value (60)(min 20)
#define SPEED_IMP 215  //  base PWM impeller speed value (215)

#define INVERT_SENSOR_READS false  //  white line and black background if false (true = black line)

#define SPEED_TIME_MS 50    //  time until robot gets base speed (50)
#define RUN_TIME_MS 6000    //  time until robot turns off automatically since start running (6000)
#define MAX_OUT_TIME_MS 10  //  maximum time sensors can be out of line until robot turns off (10)

//  ===================================
//  A D V A N C E D   V A R I A B L E S
//  ===================================

#define OUT_OUTTER_SPEED 255  //  outter side motor PWM value when sensors are offline (255)
#define OUT_INNER_SPEED -255  //  inner side motor PWM value when sensors are offline (-255)
#define MAX_IMP_SPEED 200     //  max impeller PWM value (200)

#define MARK_TIMER_MS 50     //  time while mark state is enabled (50)
#define START_DELAY_MS 1000  //  time before robot start running (1000)
#define STOP_TIME_MS 75      //  time while robot is breaking after is turned off (75)

#define SENSOR_THRESHOLD_PERCENT 75
#define CALIBRATION_NOISE_DIVISOR 8
#define CALIBRATION_AVG_SAMPLES 8
#define CALIBRATION_COLLECTOR_SAMPLES 10

//  ===================================
//  C O N S T A N T   V A R I A B L E S
//  ===================================

//  user input pins
#define PIN_BUTTON_1 11
#define PIN_BUTTON_2 12
#define PIN_LED_1 2
#define PIN_LED_2 7
#define PIN_LED_3 13

//  sensor pins
#define PIN_SENSORS_MUX_S0 A0
#define PIN_SENSORS_MUX_S1 A1
#define PIN_SENSORS_MUX_S2 A2
#define PIN_SENSORS_MUX_S3 A3
#define PIN_SENSORS_COMMON A4
#define PIN_MARK_SENSOR_0 A5
#define PIN_MARK_SENSOR_1 A6

//  motor driver pins
#define PIN_PWM_A 5
#define PIN_PWM_B 6
#define PIN_PWM_C 10
#define PIN_DIR_A 3
#define PIN_DIR_B 4

#define NUM_SENSORS 12      //  ammount of line sensors
#define NUM_MARK_SENSORS 2  //  ammount of side mark sensors
#define TOTAL_SENSORS NUM_SENSORS + NUM_MARK_SENSORS
#define INDEX_MS0 NUM_SENSORS
#define INDEX_MS1 NUM_SENSORS + 1
#define FIRST_SENSOR_MUX_SWITCH_CONFIG 2  //  mux switch configuration for first line sensor (S0)
#define LAST_SENSOR_MUX_SWITCH_CONFIG 13  //  mux switch configuration for last line sensor (S11)

//  =================================
//  P R I V A T E   V A R I A B L E S
//  =================================

int sensorValues[TOTAL_SENSORS];
unsigned int maxSensorValues[TOTAL_SENSORS];
unsigned int minSensorValues[TOTAL_SENSORS];
int sensorThreshold[TOTAL_SENSORS];

bool markState[2];
bool pastMarkSensorValues[2];

unsigned long startTime;              //  instant time before start running
unsigned long currentTime;            //  urrent time
unsigned long outStartTime;           //  last time before sensors gets offline
unsigned long markRisingEdgeTime[2];  //  instant time when marks sensors reads rising edge

bool isOnLine = false;
bool lastDetectedSide;
bool isRunning = false;
bool isOffEnabled = false;

int speedDecrement = 0;
bool isSpeedBase = false;

int error;
int previousError;

void setup() {
  //  ===================
  //  M A I N   S E T U P
  //  ===================

  //  user I/O interface setup
  pinMode(PIN_BUTTON_1, INPUT_PULLUP);
  pinMode(PIN_BUTTON_2, INPUT_PULLUP);
  pinMode(PIN_LED_1, OUTPUT);
  pinMode(PIN_LED_2, OUTPUT);
  pinMode(PIN_LED_3, OUTPUT);

  //  sensor switch pins setup
  pinMode(PIN_SENSORS_MUX_S0, OUTPUT);
  pinMode(PIN_SENSORS_MUX_S1, OUTPUT);
  pinMode(PIN_SENSORS_MUX_S2, OUTPUT);
  pinMode(PIN_SENSORS_MUX_S3, OUTPUT);

  //  motor driver pins setup
  pinMode(PIN_DIR_A, OUTPUT);
  pinMode(PIN_DIR_B, OUTPUT);

  //  =========================
  //  R O B O T   B O O T I N G
  //  =========================

  //  boot display animation
  digitalWrite(PIN_LED_3, HIGH);
  delay(750);
  digitalWrite(PIN_LED_3, LOW);

  //  calibration button trigger
  while (digitalRead(PIN_BUTTON_1)) {
  };

  //  ===================================
  //  S E N S O R   C A L I B R A T I O N
  //  ===================================
  delay(250);
  resetCalibrationValues();

  while (digitalRead(PIN_BUTTON_1)) {
    PINB |= 1 << 5;  //  toggle LED 3
    calibrateSensors();
  }
  digitalWrite(PIN_LED_3, LOW);
  delay(250);

  //  get sensor threshold
  for (byte i = 0; i < TOTAL_SENSORS; i++) {
    sensorThreshold[i] = (maxSensorValues[i] - minSensorValues[i]) * ((float)(SENSOR_THRESHOLD_PERCENT) / 100) + minSensorValues[i];
  }

  //  =======================
  //  R E A D Y   T O   R U N
  //  =======================

  //  start run button trigger
  while (digitalRead(PIN_BUTTON_1)) {
  };
  digitalWrite(PIN_LED_3, HIGH);
  delay(250);
  digitalWrite(PIN_LED_3, LOW);

  delay(1000);  //  wait 1 second to enable the impeller

  digitalWrite(PIN_LED_1, HIGH);
  digitalWrite(PIN_LED_2, HIGH);
  setImpeller(SPEED_IMP);
  delay(START_DELAY_MS);
  //  must stop impeller until motors speed gets half of base speed value
  setImpeller(0);
  digitalWrite(PIN_LED_3, HIGH);

  //  set start values
  isRunning = true;

  startTime = millis();
  currentTime = startTime;
  outStartTime = startTime;
}

//  =============================================================
//                       M A I N   L O O P
//  =============================================================

void loop() {
  while (isRunning) {
    //  get current time
    currentTime = millis();

    //  speed increase method
    if (!isSpeedBase) {
      //  if running elapsed time is higher or equal to SPEED_TIME_MS
      //  set impeller speed and set speed decrement value to zero
      if (currentTime - startTime >= SPEED_TIME_MS) {
        setImpeller(SPEED_IMP);
        speedDecrement = 0;
        isSpeedBase = true;
      } else {
        //  interpolate speed decrement ramp value in function of time elapsed after start running
        speedDecrement = SPEED - (SPEED * (currentTime - startTime) / SPEED_TIME_MS);

        //  set impeller speed at mid of speed decrement ramp time
        if (currentTime - startTime >= SPEED_TIME_MS / 2) {
          setImpeller(SPEED_IMP);
        }
      }
    }

    //  get position and error
    unsigned int position = readLine();
    error = position - (NUM_SENSORS - 1) * 1000 / 2;

    //  calculate error difference
    int dx = error - previousError;
    previousError = error;  //  update previous error after

    updateMarkState();

    //  if is online and position is no equal to the limits, calculate control signal and PWM values
    if (isOnLine && position && position != (NUM_SENSORS - 1) * 1000) {
      //  check last detected side
      if (sensorValues[0] > sensorThreshold[0] || sensorValues[1] > sensorThreshold[1]) {
        lastDetectedSide = 0;
      }
      if (sensorValues[NUM_SENSORS - 1] > sensorThreshold[NUM_SENSORS - 1] || sensorValues[NUM_SENSORS - 2] > sensorThreshold[NUM_SENSORS - 2]) {
        lastDetectedSide = 1;
      }

      //  calculate control signal
      long controlSignal = (error * KP) + ((long)(dx)*KD);

      //  constrain control signal
      controlSignal = (controlSignal < -510) ? -510 : (controlSignal > 510) ? 510
                                                                            : controlSignal;

      //  get motor PWM values
      int valPWMA = SPEED + controlSignal - speedDecrement;
      int valPWMB = SPEED - controlSignal - speedDecrement;

      //  constrain motor PWM values
      valPWMA = (valPWMA < -255) ? -255 : (valPWMA > 255) ? 255
                                                          : valPWMA;
      valPWMB = (valPWMB < -255) ? -255 : (valPWMB > 255) ? 255
                                                          : valPWMB;

      //  set motors PWM
      setMotors(valPWMA, valPWMB);

      outStartTime = currentTime;  //  update last time online
    } else {
      //  ===================================
      //  O U T   O F   L I N E   M E T H O D
      //  ===================================

      //  get out elapsed time
      unsigned long outElapsedTime = currentTime - outStartTime;

      //  if out elapsed time is higher or equal to MAX_OUT_TIME_MS, stop running
      if (outElapsedTime >= MAX_OUT_TIME_MS) { isRunning = false; }

      //  set motors in function of last detected side before out
      if (!lastDetectedSide) {
        setMotors(OUT_INNER_SPEED, OUT_OUTTER_SPEED);  //  robot turns left
      } else {
        setMotors(OUT_OUTTER_SPEED, OUT_INNER_SPEED);  //  robot turns right
      }
    }

    //  button off trigger
    isRunning = (!digitalRead(PIN_BUTTON_1) || !digitalRead(PIN_BUTTON_2)) ? false : isRunning;

    //  off by running time trigger
    if (currentTime - startTime >= RUN_TIME_MS) { isRunning = false; }

    //  update previous mark sensor values
    pastMarkSensorValues[0] = sensorValues[INDEX_MS0];
    pastMarkSensorValues[1] = sensorValues[INDEX_MS1];
  }

  //  =============================================================
  //                       R O B O T   O F F
  //  =============================================================

  setMotors(-SPEED, -SPEED);  //  instant brake
  digitalWrite(PIN_LED_3, LOW);
  delay(75);  //  brake time

  setMotors(0, 0);
  digitalWrite(PIN_LED_1, LOW);
  digitalWrite(PIN_LED_2, LOW);
  digitalWrite(PIN_LED_3, HIGH);

  delay(250);
  setImpeller(0);
  delay(2000);  //  two seconds delay after robot gets goal (very important)
  digitalWrite(PIN_LED_3, LOW);

  //  robot end programm animation
  while (1) {
    digitalWrite(PIN_LED_1, HIGH);
    digitalWrite(PIN_LED_2, HIGH);
    delay(50);
    digitalWrite(PIN_LED_1, LOW);
    digitalWrite(PIN_LED_2, LOW);
    delay(1000);
  }
}

void updateMarkState() {
  for (byte i = 0; i < 2; i++) {
    //  save instant time on rising edge
    if (sensorValues[INDEX_MS0 + i] && !pastMarkSensorValues[i]) {
      markRisingEdgeTime[i] = currentTime;
      markState[i] = true;

      //  if right mark is detected, set off enable flag to true
      //  if left mark is detected, set to false
      isOffEnabled = i ? true : false;
    }

    if (markState[i]) {
      //  set off enable flag while left mark state is true
      if (!i) { isOffEnabled = false; }

      //  set mark state to false if its elapsed time is higher or equal to MARK_TIMER_MS
      if (currentTime - markRisingEdgeTime[i] >= MARK_TIMER_MS) {
        markState[i] = false;
        //  if index is higher to zero (right mark index), off enabled flag is true
        //  and if 1 second has passed since robot started running
        if (i && isOffEnabled && currentTime - startTime >= 1000) {
          isRunning = false;  // comment this line to disable goal mark stop
        }
      }
    }
  }
}

//  resets minimum and maximmun sensor values
void resetCalibrationValues() {
  for (byte i = 0; i < TOTAL_SENSORS; i++) {
    maxSensorValues[i] = 0;
    minSensorValues[i] = 1023;
  }
}

//  returns line position using weighted average calculation
unsigned int readLine() {
  uint32_t wtd = 0;  //  temporal sum of weighted sensor values
  uint32_t sum = 0;  //  temporal sum of sensor values

  //  get calibrated sensor read values
  readCalibratedSensors();

  for (byte i = 0; i < NUM_SENSORS; i++) {
    //  add to sum if actual sensor value is higher to zero
    if (sensorValues[i]) {
      isOnLine = true;
      wtd += (uint32_t)sensorValues[i] * i * 1000;
      sum += sensorValues[i];
    }
  }

  //  if weighted sum is higher to zero return weighted average
  return wtd ? wtd / sum : 0;
}

//  updates sensor values with direct raw measurements (inverted or not inverted)
void readRawSensors() {
  //  line sensor reading loop
  for (byte i = FIRST_SENSOR_MUX_SWITCH_CONFIG; i <= LAST_SENSOR_MUX_SWITCH_CONFIG; i++) {
    PORTC = (PORTC & ~0b1111) | i;                                                      //  set mux switch address direct from I/O registers
    sensorValues[i - FIRST_SENSOR_MUX_SWITCH_CONFIG] = analogRead(PIN_SENSORS_COMMON);  //  read analog value from mux common pin
  }

  //  read analog value from mark sensor pins
  sensorValues[INDEX_MS0] = analogRead(PIN_MARK_SENSOR_0);
  sensorValues[INDEX_MS1] = analogRead(PIN_MARK_SENSOR_1);

  //  sensor values invert method
  if (INVERT_SENSOR_READS) {
    for (byte i = 0; i < TOTAL_SENSORS; i++) {
      sensorValues[i] = 1023 - sensorValues[i];
    }
  }
}

//  updates sensor values with sampled average raw measurements
void readSampledSensors() {
  int sumSensorValues[TOTAL_SENSORS];  //  temporal sum of each sensor samples

  //  set sum values to zero
  for (byte i = 0; i < TOTAL_SENSORS; i++) {
    sumSensorValues[i] = 0;
  }

  for (byte j = 0; j < CALIBRATION_AVG_SAMPLES; j++) {
    //  update raw sensor values
    readRawSensors();

    //  adds readings to sum of each sensor
    for (byte i = 0; i < TOTAL_SENSORS; i++) {
      sumSensorValues[i] += sensorValues[i];
    }
  }

  //  update sensor values with sum average
  for (byte i = 0; i < TOTAL_SENSORS; i++) {
    sensorValues[i] = (sumSensorValues[i] + (CALIBRATION_AVG_SAMPLES >> 1)) / CALIBRATION_AVG_SAMPLES;
  }
}

//  calculates fixed range raw sensor values
void readCalibratedSensors() {
  //  get actual raw sensor readings
  readRawSensors();

  for (byte i = 0; i < NUM_SENSORS; i++) {
    unsigned int range = maxSensorValues[i] - sensorThreshold[i];

    if (sensorValues[i] > sensorThreshold[i]) {
      if (sensorValues[i] < maxSensorValues[i]) {
        //  scale value to fixed range if raw sensor value is higher than threshold and lower than maximum
        sensorValues[i] = ((sensorValues[i] - sensorThreshold[i]) * 255) / range;
      } else {
        sensorValues[i] = 255;  //  set to 255 if raw sensor value is higher than maximum calibrated value
      }
    } else {
      sensorValues[i] = 0;  //  set to zero if raw sensor value is lower than threshold
    }
  }

  //  set mark sensors values to 1 if its raw value is higher than threshold
  sensorValues[INDEX_MS0] = sensorValues[INDEX_MS0] > sensorThreshold[INDEX_MS0] ? 1 : 0;
  sensorValues[INDEX_MS1] = sensorValues[INDEX_MS1] > sensorThreshold[INDEX_MS1] ? 1 : 0;
}

//  updates minimum and maximum sensor values
void calibrateSensors() {
  int highestSample[TOTAL_SENSORS];  //  temporal highest collector sample for each sensor
  int lowestSample[TOTAL_SENSORS];   //  temporal lowest collector sample for each sensor

  //  samples recollector
  for (byte j = 0; j < CALIBRATION_AVG_SAMPLES; j++) {
    readSampledSensors();

    for (byte i = 0; i < TOTAL_SENSORS; i++) {
      //  update maximum value collected
      if ((j == 0) || (sensorValues[i] > highestSample[i])) {
        highestSample[i] = sensorValues[i];
      }

      //  update minimum value collected
      if ((j == 0) || (sensorValues[i] < lowestSample[i])) {
        lowestSample[i] = sensorValues[i];
      }
    }
  }

  //  update the min and max values
  for (byte i = 0; i < TOTAL_SENSORS; i++) {
    // update maximum only if the min of all collected readings was still higher than it
    // (we got all collected readings in a row higher than the existing maximum)
    if (lowestSample[i] > maxSensorValues[i]) {
      maxSensorValues[i] = lowestSample[i];
    }

    // update minimum only if the max of all collected readings was still lower than it
    // (we got all collected readings in a row lower than the existing minimum)
    if (highestSample[i] < minSensorValues[i]) {
      minSensorValues[i] = highestSample[i];
    }
  }
}

void setImpeller(int _pwm) {
  analogWrite(PIN_PWM_C, (_pwm <= 0) ? 0 : (_pwm >= 255) ? 255
                                                         : _pwm);
}

void setMotors(int _pwm_a, int _pwm_b) {
  //  set direction output on motor A
  if (_pwm_a > 0) {
    digitalWrite(PIN_DIR_A, LOW);  //  forward
  } else if (_pwm_a < 0) {
    digitalWrite(PIN_DIR_A, HIGH);  //  reverse
  }

  //  set direction output on motor B
  if (_pwm_b > 0) {
    digitalWrite(PIN_DIR_B, LOW);  //  forward
  } else if (_pwm_b < 0) {
    digitalWrite(PIN_DIR_B, HIGH);  //  reverse
  }

  //  update PWM duty on motor drivers
  analogWrite(PIN_PWM_A, abs(_pwm_a));
  analogWrite(PIN_PWM_B, abs(_pwm_b));
}
