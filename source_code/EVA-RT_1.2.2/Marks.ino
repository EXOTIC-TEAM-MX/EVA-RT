/*
  Marks - auxilliary code for robotracer EVA-RT series

    Created on: Jan, 2025
    Edited by Mauricio Tovar
    
  Copyright (C) 2025 EXOTIC TEAM MX
  
  This code is the original version used in the All Chile
  Robot Contest 2025.

  This code consist of the detection of track marks. It
  ensures proper timing and response to detect marks while
  providing visual feedback via user LEDs.

  This code is part of the EVA-RT Github repository. See more
  in the next link:
  
  https://github.com/EXOTIC-TEAM-MX/EVA-RT
*/

void updateMarkState() {
  for (byte i = 0; i < 2; i++) {
    //  set timer on rising edge
    if (sensorValues[INDEX_MS0 + i] && !pastMarkSensorValues[i]) {
      markDetectedTime[i] = currentTime;
      markState[i] = true;

      isOffEnabled = i ? true : false;

      addHeader(String(i) + " MARK ON");
    }

    //  reset timer
    if (markState[i]) {
      addMessage("\teTime(L): " + String(currentTime - markDetectedTime[0]));

      if (!i) {
        isOffEnabled = false;
      }

      if (currentTime - markDetectedTime[i] >= _markTimer_) {
        markState[i] = false;
        if (i && isOffEnabled && currentTime - startTime >= 1000 && (_crossWarningMethod_ ? !crossWarning : true)) {
          isEnabled = false;
        }

        addHeader(String(i) + " MARK OFF");
      }
    }
  }

  //  display rising edges
  if (sensorValues[INDEX_MS0] && !pastMarkSensorValues[0]) {
    setLED_A(DM_MARK_EDGES, HIGH);
  } else if (!sensorValues[INDEX_MS0] && pastMarkSensorValues[0]) {
    setLED_A(DM_MARK_EDGES, LOW);
  }

  if (sensorValues[INDEX_MS1] && !pastMarkSensorValues[1]) {
    setLED_B(DM_MARK_EDGES, HIGH);
  } else if (!sensorValues[INDEX_MS1] && pastMarkSensorValues[1]) {
    setLED_B(DM_MARK_EDGES, LOW);
  }

  setLED_A(DM_MARK_TIMERS, markState[0]);
  setLED_B(DM_MARK_TIMERS, markState[1]);
}