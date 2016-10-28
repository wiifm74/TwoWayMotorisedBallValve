#include <PID_v1.h>
#include <TwoWayMotorisedBallValve.h>

#define PIN_INPUT 0
#define MOTOR_SPEED_PIN 10
#define OPEN_VALVE_PIN 12
#define CLOSE_VALVE_PIN 13
#define OPEN_LIMIT_PIN 2
#define CLOSE_LIMIT_PIN 3

volatile bool TwoWayMotorisedBallValve::openLimitReached;
volatile bool TwoWayMotorisedBallValve::closeLimitReached;
uint8_t TwoWayMotorisedBallValve::openLimitPin;
uint8_t TwoWayMotorisedBallValve::closeLimitPin;

//Define Variables we'll be connecting to
double PIDSetpoint;
double PIDInput;
double ValveSetPoint;

//PID tuning parameters
double Kp = 2, Ki = 5, Kd = 1;

//Objects
PID myPID(&PIDInput, &ValveSetPoint, &PIDSetpoint, Kp, Ki, Kd, DIRECT);
TwoWayMotorisedBallValve valve;

void setup() {

  //initialize the variables we're linked to
  PIDInput = analogRead(PIN_INPUT);
  PIDSetpoint = 92;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  // Initialise Two Way Motorised Ball Valve
  valve.begin(&ValveSetPoint, OPEN_VALVE_PIN, CLOSE_VALVE_PIN, MOTOR_SPEED_PIN, OPEN_LIMIT_PIN, CLOSE_LIMIT_PIN);

}

void loop() {

  PIDInput = analogRead(PIN_INPUT);
  myPID.Compute();
  valve.update();

}

