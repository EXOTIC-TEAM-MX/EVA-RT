/*
  System - auxilliary code for robotracer EVA-RT series

    Created on: Jan, 2025
    Edited by Mauricio Tovar
    
  Copyright (C) 2025 EXOTIC TEAM MX

  This code is the original version used in the All Chile
  Robot Contest 2025.

  This code provides a parameter edition menu and debugging
  functionalities for the robot. It allows users to modify
  key parameters, test motors and sensors, and print debugging
  information through the console.
  
  This code is part of the EVA-RT Github repository. See more
  in the next link:
  
  https://github.com/EXOTIC-TEAM-MX/EVA-RT
*/

void parameterEditionMenu() {
  consoleLine("------ EDITION MENU ------");
  printAdvanceParameters();

  //  LED ANIMATION
  for (byte i = 0; i < 6; i++) {
    byte _delay_time = 75;
    setLED_A(DISPLAY_MODE, HIGH);
    delay(_delay_time);
    setLED_A(DISPLAY_MODE, LOW);
    delay(_delay_time);
    setLED_B(DISPLAY_MODE, HIGH);
    delay(_delay_time);
    setLED_B(DISPLAY_MODE, LOW);
    delay(_delay_time);
  }

  while (!readBttn_1()) {
    if (readBttn_2()) {
      byte parameterIndex = numberSelection(4);

      switch (parameterIndex) {
        case 1:
          consoleLine("SENSOR MASK");
          _sensorMask_ = !_sensorMask_;
          break;

        case 2:
          consoleLine("CROSS WARNING");
          _crossWarningMethod_ = !_crossWarningMethod_;
          break;

        case 3:
          consoleLine("FIXED MAX VALUES");
          _fixedMaxTimeValues_ = !_fixedMaxTimeValues_;
          break;

        case 4:
          consoleLine("FIXED MIN VALUES");
          _fixedMinTimeValues_ = !_fixedMinTimeValues_;

        case 5:
          consoleLine("DISPLAY MODE");
          setLED_A(DISPLAY_MODE, HIGH);
          setLED_B(DISPLAY_MODE, HIGH);
          DISPLAY_MODE = numberSelection(8);
          setLED_A(DISPLAY_MODE, LOW);
          setLED_B(DISPLAY_MODE, LOW);
          break;
      }

      LedAnim(PIN_LED_3, 5, 25);
    }
  }

  printAdvanceParameters();
  consoleLine("*SELECTION DONE*");
  confirmLEDAnimation();

  consoleLine("PRESS [1] TO START CALIBRATION");
  while (!readBttn_1()) {
  };
}

byte numberSelection(byte _maxValue) {
  byte number = 0;
  while (!readBttn_1()) {
    if (readBttn_2()) {
      setLED_C(DISPLAY_MODE, HIGH);
      delay(300);
      setLED_C(DISPLAY_MODE, LOW);
      delay(250);
      number++;
      if (number == _maxValue) {
        return number;
      }
    }
  }

  delay(250);
  return number;
}

void printBar() {
  Serial.println("==================================================");
}

void printLines() {
  Serial.println("--------------------------------------------------");
}

void consoleLine(String _text) {
  if (isConsoleMode) {
    Serial.println(_text);
  }
}

void printRunParameters() {
  //consoleLine("SPEED: " + String(_speed_) + "\t IMP: " + String(_speedImp_)
  //            + "\tMARK_T_MS: " + String(_markTimer_) + "\tCROSS_W_T_MS: " + String(_crossWarningTime_));
}

void printAdvanceParameters() {
  consoleLine("MASK: " + String(_sensorMask_)
              + "\tCROSS: " + String(_crossWarningMethod_)
              + "\tFX_MAX_V: " + String(_fixedMaxTimeValues_)
              + "\tFX_MIN_V: " + String(_fixedMinTimeValues_)
              + "\tDISP_M: " + String(DISPLAY_MODE));
}

void printRunUnits() {
  if (isConsoleMode) {
    Serial.println("------------------------------------------------------------------------------------------------------------------------------");
    Serial.println("Header\t\t\tK\tPWMA\tPWMB\te\tdx\tcx\tPc\tDc\tMessage");
    Serial.println("------------------------------------------------------------------------------------------------------------------------------");
  }
}

void addHeader(String _text) {
  if (isConsoleMode) {
    header = _text;
  }
}

void addMessage(String _text) {
  if (isConsoleMode) {
    message += _text;
  }
}

void debugMode() {
  consoleLine("[ DEBUG MODE SELECTED ]");

  delay(300);
  setLEDS(DISPLAY_MODE, HIGH);
  delay(300);
  setLEDS(DISPLAY_MODE, LOW);
  delay(300);
  setLEDS(DISPLAY_MODE, HIGH);
  delay(300);
  setLEDS(DISPLAY_MODE, LOW);
  delay(300);

  consoleLine("SELECT OPERATION> TEST MOTORS[1] / TEST SENSORS[2]");

  while (!readBttn_1() && !readBttn_2()) {
  };

  if (readBttn_1()) {
    LedAnim(PIN_LED_1, 5, 25);
    areMotorsEnabled = true;
    //consoleLine("____ MOTOR TEST ____");
    //consoleLine("SELECT OPERATION> TEST WHEELS[1] / TEST IMPELLER[2]");
    while (true) {
      if (readBttn_1()) {
        testMotors(25, 500);
      }

      if (readBttn_2()) {
        delay(250);
        isImpOn = !isImpOn;
      }
      setImpeller(isImpOn ? 200 : 0);
      setLED_C(DISPLAY_MODE, isImpOn);
    }
  }

  if (readBttn_2()) {
    LedAnim(PIN_LED_2, 5, 25);
    Serial.begin(115200);
    //consoleLine("____ SENSOR TEST ____");

    while (true) {
      readRawSensors();
      printSensorValues();
    }
  }
}

void STOP() {
  while (1) {
    delay(50);
  }
}