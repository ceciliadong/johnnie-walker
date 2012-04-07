#include <SoftwareSerial.h>
#include <stdarg.h>
/**
 *
 * Date   :  4/6/2012
 * Author :  NYIT Johnnie Walker Group
 *
 * This version is aiming at providing the
 * first formal release version of the 
 * walker code
 * which should be v. 1.0 officially
 *
 * 1. Integrated control scheme
 * 2. android communication
 *
 **/
// Digital pin 18 is the serial transmit pin to the Sabertooth 2x5
#define MOTOR_TX_PIN 18
// Digital pin 19 is the serial receive pin to the Sabertooth 2x52
// but is not used, needed for SoftwareSerial Function
#define MOTOR_RX_PIN 19
// Set to 9600 through Sabertooth dip switch 4 down
#define MOTOR_BAUDRATE 9600
#define SENSOR_BAUDRATE 9600
// Create a object to control Sabertooth 2x5 motor controller
SoftwareSerial MotorSerial = SoftwareSerial(MOTOR_RX_PIN, MOTOR_TX_PIN);
// Simplified serial Limits for each motor
// Commands are sent as single bytes
// Sending a value of 1-127 will command motor 1 
// Sending a value of 128-255 will command motor 2 
// Sending a value of 0 will shut down both motors
int MOTORLEFT_FORWARD_SPEED[] =  {192, 199, 206, 213, 220, 227, 234, 241, 248, 255};
int MOTORRIGHT_FORWARD_SPEED[] = {64,  71,  78,  85,  92,  99,  106, 113, 120, 127};
int MOTORLEFT_REVERSE_SPEED[] =  {191, 184, 177, 170, 163, 156, 149, 142, 135, 128}; // from slow to fast
int MOTORRIGHT_REVERSE_SPEED[] = {63,  57,  50,  43,  36,  29,  22,  15,  8,   1}; // from slow to fast
// for speed tracking purpose
int currentSpeedLeft = 0, currentSpeedRight = 0;

// pins of side sensors
// from left --> right clockwise
// rearleft frontleft leftfront rightfront frontright rearright
const int ULTRASONIC_SENSOR_TOTAL = 6;
int ultrasonicSensorPins[ULTRASONIC_SENSOR_TOTAL] = {30, 31, 32, 33, 34, 35}; // {leftFront, leftRear, rightFront, rightRear}
int ultrasonicSensorValues[ULTRASONIC_SENSOR_TOTAL] = {0, 0, 0, 0, 0, 0};

// pins of optical sensor arrays
const int OPTICAL_SENSOR_TOTAL = 8;
int opticalSensorPins[OPTICAL_SENSOR_TOTAL] = {8, 9, 10, 11, 12, 13, 14, 15};
int opticalSensorValues[OPTICAL_SENSOR_TOTAL] = {0, 0, 0, 0, 0, 0, 0, 0};
int opticalBaseline[OPTICAL_SENSOR_TOTAL] = {600, 600, 600, 600, 600, 600, 600, 600};

// definitions related to joysticks
#define JOYSTICK_BAUDRATE 9600  // Set baudrate of Joystick sensors to 9600
const int JOYSTICK_X = 1;       // Analog input pin A0 that the X-axis of joystick is attached to
const int JOYSTICK_Y = 2;      // Analog input pin A1 that the Y-axis of joystick is attached to
int joystickX = 0;   // Horizontal value from Joystick sensor
int joystickY = 0;  // Vertical value from Joystick sensor
// definitionss related to mode change buttons
const int BUTTONPIN = 7;  // Digital pin 7 reference for pushbutton
int mode = 0;  // Variable to store the current mode, 0 for joystick mode, 1 for integration mode
int buttonState = 0;

/**
 * Initialization section
 **/
void initMotors() {
  pinMode(MOTOR_TX_PIN, OUTPUT); // Set digital pin (transmit pin) to output mode
  MotorSerial.begin(MOTOR_BAUDRATE); // Set the Serial baud rate for the motor shield 
  delay(20); // 0.02 seconds wait for motor serial to initialize
  setMotorSpeed(currentSpeedLeft, currentSpeedRight); // Initial motors speed
}

void initSensors() {
  // initialize side sensors (IR sensors)
  for(int i = 0; i < ULTRASONIC_SENSOR_TOTAL; i++)
    pinMode(ultrasonicSensorPins[i], OUTPUT);
  // initialize bottom facing sensors (optical sensor array)
  for(int i = 0; i < OPTICAL_SENSOR_TOTAL; i++)
    pinMode(opticalSensorPins[i], INPUT);
  Serial.begin(SENSOR_BAUDRATE);
  delay(20);
}

void initPushbutton() {
  pinMode(BUTTONPIN, INPUT);  // initialize the pushbutton pin as an input
}

void initJoystick() {
  // Initalize all ports communicate with Joystick sensor
  pinMode( JOYSTICK_X, INPUT );
  delay(10);
  pinMode( JOYSTICK_Y, INPUT );
  Serial.begin(JOYSTICK_BAUDRATE);  
  // 0.02 second time delay for the IR sensors to init 
  delay(20);
}

/**
 * getters
 **/
// Ultrasonic sensor values are stored
// as centimeters
void getUltrasonicSensorValues() {
  for(int i = 0; i < ULTRASONIC_SENSOR_TOTAL; i++) {
    int npin = ultrasonicSensorPins[i];
    pinMode(npin, OUTPUT);
    digitalWrite(npin, LOW);
    delay(5);
    digitalWrite(npin, HIGH);
    delay(5);
    digitalWrite(npin, LOW);
    pinMode(npin, INPUT);
    ultrasonicSensorValues[i] = pulseIn(npin, HIGH) / 58;
    delay(5);
  }
  delay(5);
}

// threshold between black and white should be 600
void getOpticalSensorValues() {
  for(int i = 0; i < OPTICAL_SENSOR_TOTAL; i++)
    opticalSensorValues[i] = analogRead(opticalSensorPins[i]);
  delay(5);
}

// convert tht raw joystick value to a integer range from 0 to 100
int normalizeCoordinate(int value) {
  if(value > 400 && value < 600) return 50;
  // Transforms the joystick's messurement into a values between 0 and 100 
  return (value / 10); 
}

// get the normalized joystic value
void getJoystickValues() {
  joystickX = normalizeCoordinate(analogRead(JOYSTICK_X));
  delay(10);
  joystickY = normalizeCoordinate(analogRead(JOYSTICK_Y));
}

/**
 * motor control and motions
 **/
int maxSpeed = 35; // the maximum value of maxSpeed is 62
void setMotorSpeed(int left, int right) {
  
  // save current state
  currentSpeedLeft = left;
  currentSpeedRight = right;
  
  // truncate impossible values
  if(left > maxSpeed) left = maxSpeed;
  if(left < -maxSpeed) left = -maxSpeed;
  if(right > maxSpeed) right = maxSpeed;
  if(right < -maxSpeed) right = -maxSpeed;
  
  // if either of them is not zero
  if(left || right) {
    left = 192 + left;
    right = 64 + right;
  }
  
  // with Arduino 1.0 this function needs to be changed to write
  MotorSerial.print(left, BYTE);
  delay(10); // Serial communication with the motors
  MotorSerial.print(right, BYTE);
   
}

/**
 * logic part
 **/
int leftBaseline, rightBaseline;
int minDis = 40, maxDis = 200;
void establishUltrasonicBaseline() {
  getUltrasonicSensorValues();
  leftBaseline = max(min(ultrasonicSensorValues[0], ultrasonicSensorValues[1]), minDis);
  rightBaseline = max(min(ultrasonicSensorValues[4], ultrasonicSensorValues[5]), minDis);
}
 
void establishOpticalBaseline() {
  for(int i = 0; i < 20; i++) {
    getOpticalSensorValues();
    for(int i = 0; i < OPTICAL_SENSOR_TOTAL; i++)
      opticalBaseline[i] = (opticalBaseline[i] + opticalSensorValues[i]) / 2;
    delay(250);
  }
  delay(5000);
}

// initialize logic
void initLogic() {
  establishUltrasonicBaseline();
  //establishOpticalBaseline();
  setMotorSpeed(15, 15);
}

// define variable for mode three
int pw = 0, iw = 0, dw = 0;
int baseSpeed = 20;
int minSpeed = 10;
// functions
// pid control side wall following
void followSideWall() {
  // if no wall is in proper range
  // let the joystick take over!
  if(leftBaseline > maxDis && rightBaseline > maxDis) { 
    joystickControl();
    return;
  }
  // pid control self driving
  // cross track error
  int cte = leftBaseline < rightBaseline ? 
            leftBaseline - min(ultrasonicSensorValues[0], ultrasonicSensorValues[1])
            : min(ultrasonicSensorValues[4], ultrasonicSensorValues[5]) - rightBaseline;
  dw = cte - pw; // D
  iw += cte; // I
  pw = cte; // P
  int correction = pw/2 - dw/2; // I is not used at this moment
  Serial.println(correction);
  if(abs(correction) > 15) correction = correction / abs(correction) * 15;
  int left = baseSpeed + correction;
  int right = baseSpeed - correction;
  if(left < minSpeed) left = minSpeed;
  if(right < minSpeed) right = minSpeed;
  setMotorSpeed(left, right);
  delay(50);
}

// joystick y change counter
// joystick x change counter
int jycc = 0, jxcc = 0;
int cycle = 20;
void respondToJoystick() {
  int x = joystickX, y = joystickY; 
  jxcc += sq(x-50)*((x-50)/abs(x-50)) / 10; // n/abs(n) is the sign!
  jycc += sq(y-50)*((y-50)/abs(y-50)) / 10; 
  // change baselines
  leftBaseline -= jxcc / cycle;
  rightBaseline += jxcc / cycle;
  // roll back
  if(leftBaseline < minDis || rightBaseline < minDis) {
    leftBaseline += jxcc / cycle;
    rightBaseline -= jxcc / cycle;
  }
  if(leftBaseline > maxDis) leftBaseline = min(ultrasonicSensorValues[0], ultrasonicSensorValues[1]);
  if(rightBaseline > maxDis) rightBaseline = min(ultrasonicSensorValues[4], ultrasonicSensorValues[5]);
  jxcc %= cycle;
  // change speed
  baseSpeed += jycc / cycle;
  if(baseSpeed > maxSpeed) baseSpeed = maxSpeed;
  if(baseSpeed < minSpeed + 5) baseSpeed = minSpeed + 5;
  jycc %= cycle;
}

// handle the brick wall
boolean respondToBrickWall() {
  int left = ultrasonicSensorValues[2], right = ultrasonicSensorValues[3];
  if(min(left, right) < 50) { 
    setMotorSpeed(0, 0);
    mode = flip(mode); // transition to mannual mode
    return true;
  }
  return false;
}

// mode three!! CHEERS!
void modeThree() {
  // get the values first!
  getUltrasonicSensorValues();
  getJoystickValues();
  // respond to joystick
  respondToJoystick();
  // respond to brick wall first!
  if(respondToBrickWall()) return;
  // follow the side wall
  followSideWall();
  delay(10);
}

// calibrate the baseline along the way
void calibrate() {
  // get sensor values
  getOpticalSensorValues();
  // iterate
  for(int i = 0; i < OPTICAL_SENSOR_TOTAL; i++) {
    // calculate 
    int diff = opticalSensorValues[i] - opticalBaseline[i];
    if(diff > -100 && diff < 100) {
      opticalBaseline[i] = (opticalBaseline[i]+opticalSensorValues[i])/2;
      opticalSensorValues[i] = 0;
    } 
    else if (diff < -100) opticalSensorValues[i] = 0;
  }
}

// tape-following function
// with PID control
// targetPosition is the index of one specific sensor
// that is used to follow the tape
// in other words, the implied "targetPosition sensor"
// should stay on the tape to follow it
int pt, it = 0, dt = 0;
// need to be revised to work with the new setMotorSpeed function
void followTape(const int targetPosition) {
  // read and calibrate the sensor value
  calibrate();
  // PID
  // calculate cross track error
  int currentPosition = targetPosition;
  for(int i = 0; i < OPTICAL_SENSOR_TOTAL; i++) {
    if(opticalSensorValues[i]) {
      currentPosition = i;
      break;
    }
  }
  // PID algorithm written in a more readable format
  int cte = currentPosition - targetPosition;
  dt = cte - pt;
  pt = cte;
  it += cte;
  int correction = (pt+dt)/2 + it/10;
  int leftSpeed = 3 + correction;
  int rightSpeed = 3 - correction;
  //if(leftSpeed > 4) leftSpeed = 4;
  if(leftSpeed < 0) leftSpeed = 0;
  //if(rightSpeed > 4) rightSpeed = 4;
  if(rightSpeed < 0) rightSpeed = 0;
  setMotorSpeed(leftSpeed, rightSpeed);
  delay(40); // modify the sampling rate for better performance
}

void modeZero() {
  followTape(OPTICAL_SENSOR_TOTAL/2);
}

void modeOne() {
  followSideWall();
}

int flip(int s) {
  return (s+1) % 2;
}

// button!
void respondToButton() {
  buttonState = digitalRead(BUTTONPIN);  // get the state of button from the digital pin
  if(buttonState == HIGH) {
    if(!mode) initLogic(); // transition from joystick control to auto driving
    mode = flip(mode);
  }
  delay(50);
}

// joystick!
// joystick control with boolean return value
void joystickControl() {
  getJoystickValues();
  // bottom right is (0, 0)
  int x = joystickX;
  int y = joystickY;
  int yDir = y >= 50 ? 1 : -1;
  int xDir = x > 50 ? 1 : -1;
  int basev = (y - 50) / 2; // base speed
  int turnf = xDir * (x - 50) / 3; // turn factor
  // turn left
  if(xDir == 1) setMotorSpeed(basev, basev + yDir*turnf);
  // turn right
  else setMotorSpeed(basev + yDir*turnf, basev);
}

void modeJs() {
  joystickControl();
}

/**
 * main loop
 **/
void setup() {
  initPushbutton();
  initJoystick();
  initSensors();
  initMotors();
}

void loop() {
  respondToButton();
  if(mode) modeThree();
  else modeJs();
}





