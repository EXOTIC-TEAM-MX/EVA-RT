/*
  EVA-RT v1.2.2 - main code for robotracer EVA-RT series
  Copyright (c) 2025 EXOTIC TEAM MX. All right reserved.

      DRIVERS:            IXF9201SG
      NUMBER OF SENSORS:  12A/3s
      MICROCONTROLLER:    ATMEGA328P-AU
      BATTERY:            2S
      CONTROL:            PD
      MOTORS:             1020

  This code is the original version used in the All Chile
  Robot Contest 2025.

  This code consist of the main code for line control,
  interacting directly with the microcontroller registers
  in order to optimize execution time and memory space.

  The code was made in order to develop, test and debug all
  the functions and algorithms of the robot, which is why it
  includes a series of diagnostic tools that facilitate the
  visualization of all these methods. So there may be errors
  and/or bad programming practices.

    Created on: Jan, 2025
    Author: Mauricio Tovar

  This code is part of the EVA-RT Github repository. See more
  in the next link:
  
  https://github.com/EXOTIC-TEAM-MX/EVA-RT
*/

//  ===================================
//  C O N S T A N T   V A R I A B L E S
//  ===================================

#define DM_NO_MODE 0            //  no display mode enabled
#define DM_RUN 1                //  turn LED C while robot is running
#define DM_SPEED_INCREMENT 2    //  start speed increment method
#define DM_OUT_DIRECTION 3      //  turn LED on last side detected
#define DM_COUNTERCORRECTION 4  //  counter correction times
#define DM_MARK_SENSORS 5       //  turns LED while mark is detected
#define DM_MARK_TIMERS 6        //  turns LED while mark state is enabled
#define DM_MARK_EDGES 7         //  turns LED on rising edges and turns down on down edges
#define DM_CROSS_METHODS 8      //  cross methods

//  user input pins
#define PIN_BUTTON_1 11
#define PIN_BUTTON_2 12
#define PIN_LED_1 2
#define PIN_LED_2 7
#define PIN_LED_3 13

//  sensor pins
#define PIN_SENSORS_COMMON A0
#define PIN_SENSORS_SWITCH_0 A1
#define PIN_SENSORS_SWITCH_1 A1
#define PIN_SENSORS_SWITCH_2 A2
#define PIN_SENSORS_SWITCH_3 A3
#define PIN_MARK_SENSOR_0 A5
#define PIN_MARK_SENSOR_1 A6

//  motor driver pins
#define PIN_PWM_A 5
#define PIN_PWM_B 6
#define PIN_PWM_C 10
#define PIN_DIR_A 3
#define PIN_DIR_B 4

#define ADC_GND_CHANNEL 0b1111
#define ADC_MUX_CHANNEL 0b0100
#define ADC_MARK_SENSOR_0 0b0101  //0b0101
#define ADC_MARK_SENSOR_1 0b0110  //0b0110
#define ADC_START_CONVERSION 0b11000000
#define SENSOR_SWITCH_MASK 0b1111

#define NUM_SENSORS 12
#define NUM_MARK_SENSORS 2
#define TOTAL_SENSORS NUM_SENSORS + NUM_MARK_SENSORS
#define INDEX_MS0 NUM_SENSORS
#define INDEX_MS1 NUM_SENSORS + 1
#define FIRST_SENSOR_SWITCH_CONFIG 2
#define LAST_SENSOR_SWITCH_CONFIG 13

#define OUT_OUTTER_SPEED 255
#define OUT_INNER_SPEED -255

#define MAX_IMP_SPEED 245
#define MIN_MARK_TIMER_MS 10
#define MIN_CROSS_WARNING_TIME_MS 20

//  SPEED 40 VALUES: MRK_T_MS(30) CW_T_MS(60) IMP(180)
//  SPEED 60 VALUES: MRK_T_MS(20) CW_T_MS(40) IMP(235)

//  =================================
//  C O N T R O L   V A R I A B L E S
//  =================================

#define ALPHA 1
#define KP 0.06
#define KD 0.6

#define SPEED 40
#define SPEED_IMP 180

#define RUN_TIME_MS 10000
#define SPEED_TIME_MS 10
#define MAX_OUT_TIME_MS 10
#define RUN_TIME_INCREMENT_S 1000

#define STOP_TIME_MS 75

//  ===================================
//  A D V A N C E D   V A R I A B L E S
//  ===================================

#define START_WAIT_MS 1000
#define START_DELAY_MS 1000
#define INVERT_SENSOR_READS false
byte DISPLAY_MODE = DM_RUN;

bool isConsoleMode = false;
bool areMotorsEnabled = true;

#define MARK_TIMER_MS 30
#define SENSOR_MASK false

#define SENSOR_THRESHOLD_PERCENT 75
#define CALIBRATION_NOISE_DIVISOR 8
#define CALIBRATION_AVG_SAMPLES 8
#define CALIBRATION_COLLECTOR_SAMPLES 10

#define CROSS_SENSOR_OFSET 3
#define CORRECT_CROSS_ERROR true
#define CROSS_WARNING_METHOD false
#define CROSS_WARNING_TIME_MS 60

#define FIXED_MAX_TIME_VALUES false
#define FIXED_MIN_TIME_VALUES false

//  =================================
//  P R I V A T E   V A R I A B L E S
//  =================================

int sensorValues[TOTAL_SENSORS];
unsigned int maxSensorValues[TOTAL_SENSORS];
unsigned int minSensorValues[TOTAL_SENSORS];
byte sensorThreshold[TOTAL_SENSORS];
bool previousLinePrint[NUM_SENSORS];

bool markState[2];
bool pastMarkSensorValues[2];

unsigned long startTime;
unsigned long currentTime;
unsigned long outStartTime;
unsigned long correctionStartTime;
unsigned long runTime = RUN_TIME_MS;
unsigned long markDetectedTime[2];
unsigned long crossStartTime;

bool isOnLine = false;
bool lastDetectedSide;
bool isEnabled = false;
bool isOut;
bool isStartMarkDetected = false;
bool isOffEnabled = false;
bool isOnCross = false;
bool crossWarning = false;
bool isImpOn = false;

int speedDecrement = 0;
bool isSpeedBase = false;

float filteredError = 0;
float previousFilteredError = 0;
int CX;

int error;
int valPWMA;
int valPWMB;

String header;
String message;

int _speed_ = SPEED;
int _speedImp_ = SPEED_IMP;
int _markTimer_ = MARK_TIMER_MS;
int _crossWarningTime_ = CROSS_WARNING_TIME_MS;

bool _sensorMask_ = SENSOR_MASK;
bool _crossWarningMethod_ = CROSS_WARNING_METHOD;
bool _fixedMaxTimeValues_ = FIXED_MAX_TIME_VALUES;
bool _fixedMinTimeValues_ = FIXED_MIN_TIME_VALUES;

void setup() {
  //  ===================
  //  M A I N   S E T U P
  //  ===================

  if (isConsoleMode) {
    Serial.begin(115200);
    printBar();
    consoleLine("EVA-RT V1.2.2");
  }

  cli();  //  disable global interrupts

  // user I/O interface setup
  DDRD |= 1 << DDD2;   //  LED 1
  DDRD |= 1 << DDD7;   //  LED 2
  DDRB |= 1 << DDB5;   //  LED 3
  PORTB |= 1 << DDB3;  //  user button 1
  PORTB |= 1 << DDB4;  //  user button 2

  //  set ADC switch output pins
  DDRC |= 0b1111;

  //  set AVcc as ADC voltage reference
  ADMUX |= 0b01 << REFS0;

  //  set ADC right adjusted result bit
  ADMUX |= 1 << ADLAR;

  //  select ADC channel
  ADMUX |= 4;

  //  set ADC prescaler
  ADCSRA = (ADCSRA & ~0b111) | 0b100;

  sei();  //  enable global interrupts

  //  =========================
  //  R O B O T   B O O T I N G
  //  =========================
  /*
  while (1) {
    readRawSensors();
    printSensorValues();
  }*/

  //  debug mode trigger
  if (readBttn_1() || readBttn_2()) {
    debugMode();
  }

  //  boot display
  LedAnim(PIN_LED_3, 10, 25);

  //  wait for selection
  consoleLine("SELECT OPERATION> CALIBRATE[1] / CHANGE PARAMETERS[2]");

  while (!readBttn_1() && !readBttn_2()) {
  };
  
  //  parameter change menu
  if (readBttn_2()) {
    parameterEditionMenu();
  }

  //  ===================================
  //  S E N S O R   C A L I B R A T I O N
  //  ===================================

  delay(250);
  resetCalibrationValues();
  resetLinePrint();
  consoleLine("CALIBRATING...  PRESS [1] TO END");

  while (!readBttn_1()) {
    PINB |= 1 << 5;  //  toggle LED D13 (Red)
    calibrateSensors();
  }
  getSensorThreshold();

  setLED_C(DISPLAY_MODE, LOW);
  consoleLine("CALIBRATION COMPLETE!");
  delay(250);

  if (isConsoleMode) {
    printCalibrationValues();
    delay(2000);
  }
  /*
  while (1) {
    readCalibratedSensors();
    printSensorValues();
  }*/

  //  =======================
  //  R E A D Y   T O   R U N
  //  =======================

  consoleLine("READY TO RUN");
  printRunParameters();
  confirmLEDAnimation();

  if (_fixedMaxTimeValues_) {
    _markTimer_ = MIN_MARK_TIMER_MS;
    _crossWarningTime_ = MIN_CROSS_WARNING_TIME_MS;
  }

  consoleLine("___ PRESS [1] TO START RUN OR [2] FOR INCREASE SPEED ___ +5");

  while (!readBttn_1()) {
    if (readBttn_2() && _speed_ < 140) {
      _speed_ += 5;
      _speedImp_ += (_speedImp_ < MAX_IMP_SPEED) ? 14 : 0;
      _markTimer_ -= (!_fixedMaxTimeValues_ && _markTimer_ > MIN_MARK_TIMER_MS) ? 3 : 0;
      _crossWarningTime_ -= (!_fixedMaxTimeValues_ && _crossWarningTime_ > MIN_CROSS_WARNING_TIME_MS) ? 5 : 0;

      printRunParameters();
      setLED_C(DISPLAY_MODE, HIGH);
      delay(300);
      setLED_C(DISPLAY_MODE, LOW);
      delay(250);
    }
  }
  setLED_C(DISPLAY_MODE, HIGH);
  delay(250);
  setLED_C(DISPLAY_MODE, LOW);

  delay(START_WAIT_MS);
  consoleLine("RUNNING");
  setLED_A(DISPLAY_MODE, HIGH);
  setLED_B(DISPLAY_MODE, HIGH);
  setImpeller(_speedImp_);
  delay(START_DELAY_MS);

  setLEDS(DISPLAY_MODE, LOW);
  setImpeller(0);
  setLEDS(DM_RUN, HIGH);
  setLED_C(DM_CROSS_METHODS, _crossWarningMethod_);
  printRunUnits();

  //  set start values
  isEnabled = true;

  startTime = millis();
  currentTime = startTime;
  outStartTime = startTime;
}

void loop() {
  while (isEnabled) {

    //  get current time
    currentTime = millis();

    if (isConsoleMode) {
      header = "\t";
      message = "";
    }

    //  increase speed method
    if (!isSpeedBase) {
      if (currentTime - startTime >= SPEED_TIME_MS) {
        setImpeller(_speedImp_);

        setLED_A(DM_SPEED_INCREMENT, HIGH);
        setLED_B(DM_SPEED_INCREMENT, HIGH);

        speedDecrement = 0;
        isSpeedBase = true;
        addHeader("SPEED BASE");
      } else {
        speedDecrement = SPEED - (SPEED * (currentTime - startTime) / SPEED_TIME_MS);
        if (currentTime - startTime >= SPEED_TIME_MS / 2) {
          setImpeller(_speedImp_);

          setLED_C(DM_SPEED_INCREMENT, HIGH);
          addHeader("IMP ON   ");
        }
      }
    }

    //  get position and error
    unsigned int position = readLine();
    error = (CORRECT_CROSS_ERROR && isOnCross) ? 0 : (position - (NUM_SENSORS - 1) * 1000 / 2);
    filteredError = ALPHA * (float)(error) + (1 - ALPHA) * filteredError;

    //  calculate derivative
    //int dx = error - previuosError;                             //  non filtered error
    int dxFilteredError = filteredError - previousFilteredError;  //  filtered error

    setLED_A(DM_MARK_SENSORS, sensorValues[INDEX_MS0]);
    setLED_B(DM_MARK_SENSORS, sensorValues[INDEX_MS1]);

    updateMarkState();

    //  check if on line
    if (isOnLine && position && position != (NUM_SENSORS - 1) * 1000) {

      //  get control signal
      long controlSignal = (error * KP) + ((long)(dxFilteredError)*KD);

      //  constrain control signal
      controlSignal = (controlSignal < -510) ? -510 : (controlSignal > 510) ? 510
                                                                            : controlSignal;
      if (isConsoleMode) {
        CX = controlSignal;
      }

      //  motor power
      valPWMA = _speed_ + controlSignal - speedDecrement;
      valPWMB = _speed_ - controlSignal - speedDecrement;

      //  constrain motor power values
      valPWMA = (valPWMA < -255) ? -255 : (valPWMA > 255) ? 255
                                                          : valPWMA;
      valPWMB = (valPWMB < -255) ? -255 : (valPWMB > 255) ? 255
                                                          : valPWMB;

      //  set Motors speed
      setMotorA(valPWMA);
      setMotorB(valPWMB);

      isOut = false;
      outStartTime = currentTime;
      setLEDS(DM_OUT_DIRECTION, LOW);
    } else {
      //  =======================
      //    O U T   M E T H O D
      //  =======================

      //  one execution code after out
      if (!isOut) {
        isOut = true;
        addHeader("OUT OF LINE: " + String(lastDetectedSide));
        setLED_C(DM_OUT_DIRECTION, HIGH);
      }

      //  get out elapsed time
      unsigned long outElapsedTime = currentTime - outStartTime;
      addMessage("eTime(mS): " + String(outElapsedTime));

      if (outElapsedTime >= MAX_OUT_TIME_MS) {
        isEnabled = false;
      }

      //  set motors in function of last detected side before out
      if (!lastDetectedSide) {
        setLED_A(DM_OUT_DIRECTION, HIGH);

        setMotorA(OUT_INNER_SPEED);
        setMotorB(OUT_OUTTER_SPEED);

        if (isConsoleMode) {
          valPWMA = OUT_INNER_SPEED;
          valPWMB = OUT_OUTTER_SPEED;
        }

      } else {
        setLED_B(DM_OUT_DIRECTION, HIGH);

        setMotorA(OUT_OUTTER_SPEED);
        setMotorB(OUT_INNER_SPEED);

        if (isConsoleMode) {
          valPWMA = OUT_OUTTER_SPEED;
          valPWMB = OUT_INNER_SPEED;
        }
      }
    }

    //  button off trigger
    isEnabled = (readBttn_1() || readBttn_2()) ? false : isEnabled;
/*
    //  off by time timer trigger
    if (currentTime - startTime >= runTime) {
      isEnabled = false;
    }
*/
    pastMarkSensorValues[0] = sensorValues[INDEX_MS0];
    pastMarkSensorValues[1] = sensorValues[INDEX_MS1];

    previousFilteredError = filteredError;
    //previuosError = error;

    if (isConsoleMode) {
      Serial.print(header + "\t\t");
      Serial.print(String(currentTime - startTime) + "\t");
      Serial.print(String(valPWMA) + "\t");
      Serial.print(String(valPWMB) + "\t");
      Serial.print(String(error) + "\t");
      Serial.print(String(dxFilteredError) + "\t");
      Serial.print(String(CX) + "\t");
      Serial.print(String(error * KP) + "\t");
      Serial.print(String((long)(dxFilteredError)*KD) + "\t");
      Serial.println(message);
    }
  }
  //  =============================================================
  //                       R O B O T   O F F
  //  =============================================================

  setMotorA(-SPEED);
  setMotorB(-SPEED);
  setLED_A(DISPLAY_MODE, HIGH);
  setLED_B(DISPLAY_MODE, HIGH);
  setLED_C(DISPLAY_MODE, LOW);

  delay(STOP_TIME_MS);

  setMotorA(0);
  setMotorB(0);
  setLED_A(DISPLAY_MODE, LOW);
  setLED_B(DISPLAY_MODE, LOW);

  delay(STOP_TIME_MS * 4);

  setImpeller(0);
  setLED_C(DISPLAY_MODE, LOW);
  consoleLine("ROBOT TURNED OFF");

  delay(1000);

  while (true) {
    setLED_A(DISPLAY_MODE, HIGH);
    setLED_B(DISPLAY_MODE, HIGH);
    delay(50);
    setLED_A(DISPLAY_MODE, LOW);
    setLED_B(DISPLAY_MODE, LOW);
    delay(1000);
  }
}