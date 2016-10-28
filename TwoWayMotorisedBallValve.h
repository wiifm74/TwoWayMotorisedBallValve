/*
    File:   TwoWayMotorisedBallValve.cpp
    Title:    TwoWayMotorisedBallValve.cpp
    Author(s):  wiifm
    Created:    Oct 2016
    License:    GNU General Public License v3

    LICENSE:
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/

#ifndef TWO_WAY_MOTORISED_BALL_VALVE_H
#define TWO_WAY_MOTORISED_BALL_VALVE_H

#define MAX_CYCLE 5000
#define MAX_MOTOR_SPEED 255
#define CALIBRATION_CYCLES 3
#define DEBOUNCE_COUNT 5
#define DEBOUNCE_DELAY 5

#define OPENING 1
#define CLOSING -1
#define STOPPED 0

class TwoWayMotorisedBallValve {

    static volatile bool openLimitReached;
    static volatile bool closeLimitReached;
    static uint8_t openLimitPin;
    static uint8_t closeLimitPin;

  public:

    void begin(double* SetPoint, uint8_t OpenValvePin, uint8_t CloseValvePin, uint8_t MotorSpeedPin, uint8_t OpenLimitPin, uint8_t CloseLimitPin) {

      setPoint = SetPoint;
      openValvePin = OpenValvePin;
      closeValvePin = CloseValvePin;
      motorSpeedPin = MotorSpeedPin;
      openLimitPin = OpenLimitPin;
      closeLimitPin = CloseLimitPin;

      pinMode(openValvePin, OUTPUT);
      pinMode(closeValvePin, OUTPUT);
      pinMode(motorSpeedPin, OUTPUT);
      pinMode (openLimitPin, INPUT_PULLUP);
      pinMode (closeLimitPin, INPUT_PULLUP);

      // set the default motor speed to maximum in both directions
      openMotorSpeed = MAX_MOTOR_SPEED;
      closeMotorSpeed = MAX_MOTOR_SPEED;

      closeLimitReached = (digitalRead(closeLimitPin) == LOW ? true : false);
      openLimitReached = (digitalRead(openLimitPin) == LOW ? true : false);

      calibrate();
      
      fullyClose();
      currentPosition = 0;

      lastUpdate = millis() - 1;

    }

    long fullyClose() {

      unsigned long startTime, endTime;
      startTime = millis();

      while (!closeLimitReached) motor(CLOSING);

      endTime = millis();
      motor(STOPPED);

      return (long) (endTime - startTime);

    }

    long fullyOpen() {

      unsigned long startTime, endTime;
      startTime = millis();

      while (!openLimitReached) motor(OPENING);

      endTime = millis();
      motor(STOPPED);

      return (long) (endTime - startTime);

    }

    void update() {

      static unsigned long now;

      now = millis();
      if ((now - lastUpdate) > 0) {

        *setPoint = floor(min(max(0, *setPoint), cycleTime)+0.5);

        currentPosition = currentPosition + (currentStatus * (now - lastUpdate));
        lastUpdate = now;

        if (currentStatus != STOPPED) {
          if (openLimitReached) {
            motor(STOPPED);
            currentPosition = cycleTime;
          }
          if (closeLimitReached) {
            motor(STOPPED);
            currentPosition = 0;
          }
          if (currentPosition == *setPoint) motor(STOPPED);
        }

        if ((currentPosition < *setPoint) && (currentStatus != OPENING)) motor(OPENING);
        if ((currentPosition > *setPoint) && (currentStatus != CLOSING)) motor(CLOSING);

    // Un-comment the line below to plot target position and current positioon using Serial Plotter.  NOTE:  All other Serial.print commands must be commented out of the code.
        //Serial.print(*setPoint); Serial.print(","); Serial.println(currentPosition);
      }

    }

    long getCycleTime() {

      return cycleTime;

    }

    bool isFullyClosed() {

      return (currentPosition == 0);
      
    }

    bool isFullyOpened() {

      return (currentPosition == cycleTime);

    }

  private:

        void motor(int Direction) {

      currentStatus = Direction;

      switch (Direction) {
        case OPENING:
          detachInterrupt(digitalPinToInterrupt(closeLimitPin));
          closeLimitReached = false;
          analogWrite (motorSpeedPin, openMotorSpeed);
          digitalWrite (openValvePin, HIGH);
          digitalWrite (closeValvePin,  LOW);
          attachInterrupt(digitalPinToInterrupt(openLimitPin), openLimitISR, FALLING);
          break;
        case CLOSING:
          detachInterrupt(digitalPinToInterrupt(openLimitPin));
          openLimitReached = false;
          analogWrite (motorSpeedPin, closeMotorSpeed);
          digitalWrite (closeValvePin, HIGH);
          digitalWrite (openValvePin,  LOW);
          attachInterrupt(digitalPinToInterrupt(closeLimitPin), closeLimitISR, FALLING);
          break;
        case STOPPED:
          detachInterrupt(digitalPinToInterrupt(openLimitPin));
          detachInterrupt(digitalPinToInterrupt(closeLimitPin));
          digitalWrite (openValvePin, LOW);
          digitalWrite (closeValvePin,  LOW);
      }

    }

    static void openLimitISR() {

      if (!openLimitReached) {
        openLimitReached = debounceInputPullupPin(openLimitPin);
      }

    }

    static void closeLimitISR() {

      if (!closeLimitReached) {
        closeLimitReached = debounceInputPullupPin(closeLimitPin);
      }

    }

    static bool debounceInputPullupPin(uint8_t Pin) {

      int currentState;
      int lastState = LOW;
      int count = 0;

      while (count <= DEBOUNCE_COUNT) {

        delay(DEBOUNCE_DELAY);
        currentState = digitalRead(Pin);

        if (currentState == lastState) {
          count++;
        } else {
          count = 0;
          lastState = currentState;
        }
      }
      return (currentState == LOW);
    }

    void calibrate() {

      long totalOpeningTime = 0;
      long totalClosingTime = 0;
      float factor = 1;

      fullyClose();

      for (int i = 0; i < CALIBRATION_CYCLES; i++) {
        totalOpeningTime = totalOpeningTime + fullyOpen();
        totalClosingTime = totalClosingTime + fullyClose();
      }

      if ((totalOpeningTime - totalClosingTime) > 0) {
        cycleTime = (int) (((float) totalOpeningTime / CALIBRATION_CYCLES) + 0.5);
        //need to slow closing down
        factor = (float) totalClosingTime / (float) totalOpeningTime;
        closeMotorSpeed = (int) ((openMotorSpeed * factor) + 0.5);
      }

      if ((totalOpeningTime - totalClosingTime) < 0) {
        cycleTime = (int) (((float) totalClosingTime / CALIBRATION_CYCLES) + 0.5);
        //need to slow opening down
        factor = (float) totalOpeningTime / (float) totalClosingTime;
        openMotorSpeed = (int) ((closeMotorSpeed * factor) + 0.5);
      }

    }

    uint8_t closeValvePin;
    uint8_t openValvePin;
    uint8_t motorSpeedPin;

    unsigned long lastUpdate;

    double *setPoint;
    long currentPosition;
    int currentStatus;

    int openMotorSpeed;
    int closeMotorSpeed;
    long cycleTime;

};
#endif
