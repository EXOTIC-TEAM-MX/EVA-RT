/*
  UserIO - auxilliary code for robotracer EVA-RT series
  Copyright (c) 2025 EXOTIC TEAM MX. All right reserved

  This code is the original version used in the All Chile
  Robot Contest 2025.

  This code manages button inputs and LED control for the
  robot, handling user interactions and visual feedback
  through LEDs.

    Created on: Jan, 2025
    Author: Mauricio Tovar

  This code is part of the EVA-RT Github repository. See more
  in the next link:
  
  https://github.com/EXOTIC-TEAM-MX/EVA-RT
*/

bool readBttn_1() {
  return !((PINB & (1 << PINB3)) >> PINB3);  //  EVA
}

bool readBttn_2() {
  return !((PINB & (1 << PINB4)) >> PINB4);
}

bool readReady() {
  return (PINB & (1 << PINB1)) >> PINB1;
}

bool readGo() {
  return (PINB & (1 << PINB0)) >> PINB0;
}

void setLED_A(byte _mode, bool _state) {
  if (_mode == DISPLAY_MODE) {
    PORTD = (PORTD & ~(1 << PORTD2)) | (_state << PORTD2);
  }
}

void setLED_B(byte _mode, bool _state) {
  if (_mode == DISPLAY_MODE) {
    PORTD = (PORTD & ~(1 << PORTD7)) | (_state << PORTD7);
  }
}

void setLED_C(byte _mode, bool _state) {
  if (_mode == DISPLAY_MODE) {
    PORTB = (PORTB & ~(1 << PORTB5)) | (_state << PORTB5);
  }
}

void setLEDS(byte _mode, bool _state) {
  if (_mode == DISPLAY_MODE) {
    PORTD = (PORTD & ~(1 << PORTD2)) | (_state << PORTD2);
    PORTD = (PORTD & ~(1 << PORTD7)) | (_state << PORTD7);
    PORTB = (PORTB & ~(1 << PORTB5)) | (_state << PORTB5);
  }
}

void confirmLEDAnimation() {
  byte _delay_time = 200;
  setLEDS(DISPLAY_MODE, LOW);
  delay(_delay_time);
  setLEDS(DISPLAY_MODE, HIGH);
  delay(_delay_time);
  setLEDS(DISPLAY_MODE, LOW);
  delay(_delay_time);
  setLEDS(DISPLAY_MODE, HIGH);
  delay(_delay_time);
  setLEDS(DISPLAY_MODE, LOW);
  delay(_delay_time);
}

void LedAnim(byte _pin, byte _cycles, int _time) {
  digitalWrite(_pin, LOW);
  delay(250);
  for (int i = 0; i < _cycles; i++) {
    digitalWrite(_pin, HIGH);
    delay(_time);
    digitalWrite(_pin, LOW);
    delay(_time);
  }
}