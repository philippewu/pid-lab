// LIBRARIES
#include <TimerOne.h>
#include <AutoPID.h>
#include <AccelStepper.h>


// STEPPER MOTOR INSTANTIATION
long velocityScalar = 50; // scalar value for velocity
const long pidScalar = 20; // constant scalar value for PID (can be tuned)
AccelStepper myStepper(AccelStepper::DRIVER, 7, 6); // attach Stepper Driver (Pulse, Dir)

// LOAD CELL VARIABLES
String readSensor; // string object holding sensor value at given time
char c; // next character in serial data stream
const int updateInterval = 100;  // time between loadcell reading and PID update
bool isPolling; // read weights from cell or no
long cellMode = -1; // 1 is tension calibration, -1 is compression calibration
volatile int disconnectTimeout = 300; // if load cell data not received, wait this many milliseconds before automatically terminating

// SOFTWARE COMMAND VARIABLES
String readPC; // string object holding PC command at given time
char p; // next character in serial data stream
long instruction, velocity, code; // instruction: last 4 digits; velocity: speed and direction of motor; code: first digit
bool pidMode = false; // is the system in PID mode?

// PID PARAMETERS AND INSTANTIATION
#define OUTPUT_MIN -10000
#define OUTPUT_MAX 10000
#define KP 0.3 // proportional gain
#define KI 0 // integral gain
#define KD 0 // derivative gain

double load, setPoint, outputSpeed; // load: cell reading; setPoint: target load; outputSpeed: PID calculated response
AutoPID winchPID(&load, &setPoint, &outputSpeed, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD); // PID constructor

// BREAK DETECTION
double prevLoad, prevSetPoint; // 1-time memory
const int breakThresh = 10; // percentage drop for break classification

// POSITIVE FEEDBACK DETECTION
double err, prevErr, delta, EMA; // err: error value; prevErr: error memory; delta: error differential; EMA: exponential moving average
float discount = 0.95; // discount rate
int count = 0; // consecutive positive EMA count
bool newCommand = false; // has a new command been passed?


// SETUP: this function runs once
void setup() {
  myStepper.setMaxSpeed(OUTPUT_MAX); // set max stepper motor speed
  Serial.begin(9600); // setup communication with PC
  pinMode(2, INPUT); // manual override joystick button pin assigned to pin 2

  while (!Serial) {
    ; // wait for serial port to connect.
  }
  Serial2.begin(9600); // setup communication with load cell
  while (!Serial2) {
    ; // wait for serial port to connect.
  }
  delay(1000); // wait before polling sensor
  isPolling = true; // enable polling sensor
  winchPID.setTimeStep(updateInterval); // PID update time milliseconds
  Timer1.initialize(100000); // start timer for feedback and disconnect detection
  Timer1.attachInterrupt(detectFeedback); // call function at interval
}

// LOOP: this function loops
void loop() {

  // while loop activates when pendant is pressed
  while(digitalRead(2) == LOW) {
    myStepper.setSpeed(map(analogRead(0), 0, 1023, OUTPUT_MIN, OUTPUT_MAX));  // give manual control the same speed range as PID output
    myStepper.runSpeed();
  }

  // run winch in PID mode or manual speed mode
  if(pidMode) {
    winchPID.run();
    myStepper.setSpeed(pidScalar * outputSpeed);
    myStepper.runSpeed();
  }
  else {
    myStepper.setSpeed(velocityScalar * velocity);
    myStepper.runSpeed();
  }

  // check for break detection or serial disconnect
  if(pidMode) {
    if(setPoint >= prevSetPoint && ((prevLoad - load) * 100) / prevLoad > breakThresh && prevLoad > 10) { // break detection conditionals
      reset();
      prevSetPoint = 0;
      Serial.println(0); // break detection error code
    }
    if(disconnectTimeout <= 0) {
      reset();
      Serial.println(2); // serial disconnect error code
    }
  }
}

// SERIALEVENT: this function runs when there are serial values from software and reads the message
void serialEvent() {
  p = char(Serial.read()); // read next character
  if(p == 'e') { // break character found
    instruction = readPC.toInt(); // transforms string to a long integer
    readPC = ""; // reset read value to empty
  }
  else {
    readPC += p; // append character to string
  }
  decode(); // decode message
}

// SERIALEVENT2: this function runs when there are serial values from the load cell and reads the message
void serialEvent2() {
  if(isPolling) {
    c = char(Serial2.read()); // read next character
    if(c == '\n') { // break character found
      prevLoad = load;
      load = cellMode * readSensor.toInt() / 1000; // transforms string to a long integer and convert to pounds
      readSensor = ""; // reset read value to empty
      disconnectTimeout = 300; // reset serial disconnect time bank
    }
    else {
      readSensor += c; // append character to string
    }
  }
}

// DECODE: this function decodes messages from software and is called by serialEvent()
void decode() {
  code = instruction / 10000; // divide instruction by 10000 and store in int to get code from 1st digit
  instruction -= code * 10000; // subtract code * 10000 from the rest of the instruction to recover the last 4 digits

  if(code == 1) { // PID mode with positive values
    pidMode = true;
    setPoint = instruction;
  }
  else if(code == 2) { // PID mode with negative values
    pidMode = true;
    setPoint = -instruction;
  }
  else if(code == 3) { // manual control with positive values
    pidMode = false;
    velocity = instruction;
  }
  else if(code == 4) { // manual control with negative values
    pidMode = false;
    velocity = -instruction;
  }
  else if(code == 5) { // velocity multiplier control
    pidMode = false;
    velocityScalar = instruction;
  }

  if(pidMode) { // procedure when receiving a new PID command
    newCommand = true;
    prevSetPoint = setPoint;
    EMA = 0;
  }
}

// RESET: this function freezes system in place and is called whenever the system needs to stop moving
void reset() {
  pidMode = false; // switch to velocity mode
  velocity = 0; // set velocity to zero
}

// DETECTFEEDBACK: this function detects positive feedback loops and monitors serial disconnects
void detectFeedback() {
  if(pidMode) {

    prevErr = err; // set new previous error
    err = abs(setPoint - load); // calculate new error

    if(newCommand) { // initialize delta to be zero when t=0
      delta = 0;
      newCommand = false;
    }
    else {
      delta = err - prevErr; // error differential
    }

    EMA += (1 - discount) * (delta - EMA); // exponential moving average calculation
    if(EMA > 0.1) { // count consecutive increases
      count++;
    }
    else {
      count = 0;
    }

    if(EMA > 1.5 || count >= 15) { // positive feedback loop conditionals
      reset();
      Serial.println(1); // positive feedback detected error code
    }
  }
  disconnectTimeout -= 100; // decrement to reach zero to trigger timeout
}