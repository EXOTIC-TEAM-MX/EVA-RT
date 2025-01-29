/*
  Motors - auxilliary code for robotracer EVA-RT series

    Created on: Jan, 2025
    Edited by Mauricio Tovar
    
  Copyright (C) 2025 EXOTIC TEAM MX

  This code is the original version used in the All Chile
  Robot Contest 2025.

  This code consist of the motor control functions for the
  line following differential drive method. It allows
  control the impeller motor. It includes a testing routine.
  
  This code is part of the EVA-RT Github repository. See more
  in the next link:
  
  https://github.com/EXOTIC-TEAM-MX/EVA-RT
*/

void setImpeller(int _pwm) {
  if (areMotorsEnabled) {
    analogWrite(PIN_PWM_C, (_pwm >= 0 && _pwm <= 255) ? _pwm : 0);
  }
}

void setMotorA(int _pwm) {
  if (areMotorsEnabled) {
    analogWrite(PIN_PWM_A, abs(_pwm));

    PORTD = (_pwm > 0)   ? (PORTD & ~(1 << PORTD3))  //  forward
            : (_pwm < 0) ? (PORTD | (1 << PORTD3))   //  reverse
                         : PORTD;                    //  no change
  }
}

void setMotorB(int _pwm) {
  if (areMotorsEnabled) {
    analogWrite(PIN_PWM_B, abs(_pwm));

    PORTD = (_pwm > 0)   ? (PORTD & ~(1 << PORTD4))  //  forward
            : (_pwm < 0) ? (PORTD | (1 << PORTD4))   //  reverse
                         : PORTD;                    //  no change
  }
}

void testMotors(int _pwm, int _time) {
  delay(500);

  consoleLine("*MOTOR A+*");
  setMotorA(_pwm);
  setLED_A(DISPLAY_MODE, HIGH);
  delay(_time);
  setMotorA(0);
  setLED_A(DISPLAY_MODE, LOW);
  delay(_time + 1500);
  consoleLine("*MOTOR A-*");
  setMotorA(-_pwm);
  setLED_A(DISPLAY_MODE, HIGH);
  delay(_time);
  setMotorA(0);
  setLED_A(DISPLAY_MODE, LOW);
  delay(_time + 1500);

  consoleLine("*MOTOR B+*");
  setMotorB(_pwm);
  setLED_B(DISPLAY_MODE, HIGH);
  delay(_time);
  setMotorB(0);
  setLED_B(DISPLAY_MODE, LOW);
  delay(_time + 1500);
  consoleLine("*MOTOR B-*");
  setMotorB(-_pwm);
  setLED_B(DISPLAY_MODE, HIGH);
  delay(_time);
  setMotorB(0);
  setLED_B(DISPLAY_MODE, LOW);
  delay(_time + 1500);

  consoleLine("SELECT OPERATION> TEST WHEEL MOTORS[1] / TEST IMPELLER[2]");
}