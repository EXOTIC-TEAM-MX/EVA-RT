/*
  Sensors - auxilliary code for robotracer EVA-RT series

    Created on: Jan, 2025
    Edited by Mauricio Tovar
    
  Copyright (C) 2025 EXOTIC TEAM MX

  This code is the original version used in the All Chile
  Robot Contest 2025.

  This code consist of reading and processing sensor data.
  It includes functions for calibrating, reading, and
  interpreting sensor values to determine the line's position.
  
  This code is part of the EVA-RT Github repository. See more
  in the next link:
  
  https://github.com/EXOTIC-TEAM-MX/EVA-RT
*/

void resetCalibrationValues() {
  for (byte i = 0; i < TOTAL_SENSORS; i++) {
    maxSensorValues[i] = 0;
    minSensorValues[i] = 255;
  }
}

void resetLinePrint() {
  for (byte i = 0; i < NUM_SENSORS; i++) {
    previousLinePrint[i] = true;
  }
}

void getSensorThreshold() {
  for (byte i = 0; i < TOTAL_SENSORS; i++) {
    sensorThreshold[i] = (maxSensorValues[i] - minSensorValues[i]) * ((float)(SENSOR_THRESHOLD_PERCENT) / 100) + minSensorValues[i];
  }
}

uint16_t readLine() {
  uint32_t avg = 0;
  uint32_t sum = 0;

  readCalibratedSensors();

  for (byte i = 0; i < NUM_SENSORS; i++) {
    if (sensorValues[i]) {
      avg += (uint32_t)sensorValues[i] * i * 1000;
      sum += sensorValues[i];
    }
  }

  return sum ? avg / sum : 0;
}


void calibrateSensors() {
  int highestSample[TOTAL_SENSORS];
  int lowestSample[TOTAL_SENSORS];

  //  samples recollector
  for (byte j = 0; j < CALIBRATION_COLLECTOR_SAMPLES; j++) {

    readSampledSensors();

    for (byte i = 0; i < TOTAL_SENSORS; i++) {
      if ((j == 0) || (sensorValues[i] > highestSample[i])) {
        highestSample[i] = sensorValues[i];
      }

      if ((j == 0) || (sensorValues[i] < lowestSample[i])) {
        lowestSample[i] = sensorValues[i];
      }
    }
  }

  for (byte i = 0; i < TOTAL_SENSORS; i++) {
    if (lowestSample[i] > maxSensorValues[i]) {
      maxSensorValues[i] = lowestSample[i];
    }
    if (highestSample[i] < minSensorValues[i]) {
      minSensorValues[i] = highestSample[i];
    }
  }
}

void readCalibratedSensors() {
  isOnLine = false;
  readRawSensors();

  for (byte i = 0; i < NUM_SENSORS; i++) {
    unsigned int range = maxSensorValues[i] - sensorThreshold[i];

    if (sensorValues[i] > sensorThreshold[i]) {
      isOnLine = true;
      if (sensorValues[i] < maxSensorValues[i]) {
        sensorValues[i] = ((sensorValues[i] - sensorThreshold[i]) * 255) / range;
      } else {
        sensorValues[i] = 255;
      }
    } else {
      sensorValues[i] = 0;
    }
  }

  sensorValues[INDEX_MS0] = sensorValues[INDEX_MS0] > sensorThreshold[INDEX_MS0] ? 1 : 0;
  sensorValues[INDEX_MS1] = sensorValues[INDEX_MS1] > sensorThreshold[INDEX_MS1] ? 1 : 0;

  if (isOnLine) {
    //  set last detected side
    lastDetectedSide = (sensorValues[0] || sensorValues[1])                               ? 0
                       : (sensorValues[NUM_SENSORS - 1] || sensorValues[NUM_SENSORS - 2]) ? 1
                                                                                          : lastDetectedSide;
  }

  if (_sensorMask_) {
    //  clear previous sensor print
    for (byte i = 0; i < NUM_SENSORS; i++) {
      previousLinePrint[i] = 0;
    }

    // update previous sensor print
    for (byte i = 0; i < NUM_SENSORS; i++) {
      if (sensorValues[i]) {
        previousLinePrint[i] = true;

        //  one sensor adyacent
        //if (i) previousLinePrint[i - 1] = true;
        //if (i < NUM_SENSORS - 1) previousLinePrint[i + 1] = true;

        //  two sensor adyacent
        if (i > 1) {
          previousLinePrint[i - 1] = true;
          previousLinePrint[i - 2] = true;
        }
        if (i < NUM_SENSORS - 2) {
          previousLinePrint[i + 1] = true;
          previousLinePrint[i + 2] = true;
        }
      }
    }

    if (!isOnLine) {
      previousLinePrint[lastDetectedSide ? (NUM_SENSORS - 1) : 0] = true;
      previousLinePrint[lastDetectedSide ? (NUM_SENSORS - 2) : 1] = true;
    }
  }

  //  cross methods
  if (sensorValues[(NUM_SENSORS / 2) - CROSS_SENSOR_OFSET - 1] && sensorValues[(NUM_SENSORS / 2) + CROSS_SENSOR_OFSET]) {
    isOnCross = true;
    crossStartTime = currentTime;
    crossWarning = true;
    isOffEnabled = _crossWarningMethod_ ? false : isEnabled;
    setLED_B(DM_CROSS_METHODS, HIGH);
  } else {
    isOnCross = false;
  }

  //  reset cross warning timer
  if (crossWarning && currentTime - crossStartTime >= _crossWarningTime_) {
    crossWarning = false;
    setLED_B(DM_CROSS_METHODS, LOW);
  }

  setLED_A(DM_CROSS_METHODS, isOnCross);
}

void readSampledSensors() {
  int sumSensorValues[NUM_SENSORS + 2];

  for (byte i = 0; i < NUM_SENSORS + 2; i++) {
    sumSensorValues[i] = 0;
  }

  for (byte j = 0; j < CALIBRATION_AVG_SAMPLES; j++) {
    readRawSensors();

    for (byte i = 0; i < NUM_SENSORS; i++) {
      sumSensorValues[i] += sensorValues[i];
    }

    sumSensorValues[INDEX_MS0] += sensorValues[INDEX_MS0];
    sumSensorValues[INDEX_MS1] += sensorValues[INDEX_MS1];
  }

  for (byte i = 0; i < NUM_SENSORS; i++) {
    sensorValues[i] = (sumSensorValues[i] + (CALIBRATION_AVG_SAMPLES >> 1)) / CALIBRATION_AVG_SAMPLES;
  }

  sensorValues[INDEX_MS0] = (sumSensorValues[INDEX_MS0] + (CALIBRATION_AVG_SAMPLES >> 1)) / CALIBRATION_AVG_SAMPLES;
  sensorValues[INDEX_MS1] = (sumSensorValues[INDEX_MS1] + (CALIBRATION_AVG_SAMPLES >> 1)) / CALIBRATION_AVG_SAMPLES;
}

void readRawSensors() {
  //  read GND
  selectADCChannel(ADC_GND_CHANNEL);
  startADCConversion();
  waitForADC();
  disableADC();

  selectADCChannel(ADC_MUX_CHANNEL);

  for (byte i = FIRST_SENSOR_SWITCH_CONFIG; i <= LAST_SENSOR_SWITCH_CONFIG; i++) {
    if (!_sensorMask_ || previousLinePrint[i - FIRST_SENSOR_SWITCH_CONFIG]) {
      PORTC = (PORTC & ~0b1111) | i;  //  set switch address

      //  read MUX
      startADCConversion();
      waitForADC();
      sensorValues[i - FIRST_SENSOR_SWITCH_CONFIG] = ADCH;
      disableADC();
    } else {
      sensorValues[i - FIRST_SENSOR_SWITCH_CONFIG] = INVERT_SENSOR_READS ? 255 : 0;
    }
    sensorValues[i - FIRST_SENSOR_SWITCH_CONFIG] = INVERT_SENSOR_READS ? 255 - sensorValues[i - FIRST_SENSOR_SWITCH_CONFIG] : sensorValues[i - FIRST_SENSOR_SWITCH_CONFIG];
  }

  //  read MS0
  selectADCChannel(ADC_MARK_SENSOR_0);
  startADCConversion();
  waitForADC();
  sensorValues[INDEX_MS0] = INVERT_SENSOR_READS ? 255 - ADCH : ADCH;
  disableADC();

  //  read MS1
  selectADCChannel(ADC_MARK_SENSOR_1);
  startADCConversion();
  waitForADC();
  sensorValues[INDEX_MS1] = INVERT_SENSOR_READS ? 255 - ADCH : ADCH;
  disableADC();
}

void printSensorValues() {
  for (byte i = 0; i < NUM_SENSORS; i++) {
    Serial.print(String(sensorValues[i]) + "\t");
  }
  Serial.println("\t" + String(sensorValues[INDEX_MS0]) + "\t" + String(sensorValues[INDEX_MS1]));
}

void printPreviousSensorPrint() {
  for (byte i = 0; i < NUM_SENSORS; i++) {
    Serial.print(previousLinePrint[i]);
    Serial.print("\t");
  }
  Serial.println();
}

void printCalibrationValues() {
  //
  Serial.print("MAX:\t");
  for (byte i = 0; i < NUM_SENSORS; i++) {
    Serial.print(String(maxSensorValues[i]) + "\t");
  }
  Serial.println("\t" + String(maxSensorValues[INDEX_MS0]) + "\t" + String(maxSensorValues[INDEX_MS1]));

  //
  Serial.print("MIN:\t");
  for (byte i = 0; i < NUM_SENSORS; i++) {
    Serial.print(String(minSensorValues[i]) + "\t");
  }
  Serial.println("\t" + String(minSensorValues[INDEX_MS0]) + "\t" + String(minSensorValues[INDEX_MS1]));

  //
  Serial.print("TH:\t");
  for (byte i = 0; i < NUM_SENSORS; i++) {
    Serial.print(String(sensorThreshold[i]) + "\t");
  }
  Serial.println("\t" + String(sensorThreshold[INDEX_MS0]) + "\t" + String(sensorThreshold[INDEX_MS1]));
}

//  macros
void startADCConversion() {
  ADCSRA |= ADC_START_CONVERSION;
}

void waitForADC() {
  while (bit_is_set(ADCSRA, ADSC))
    ;
}

void disableADC() {
  ADCSRA &= ~(1 << ADEN);
}

void selectADCChannel(uint8_t channel) {
  ADMUX = (ADMUX & ~0b1111) | channel;
}