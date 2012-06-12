#include <SoftwareSerial.h>
#include <stdarg.h>
/**
 *
 * Date   :  6/8/2012
 * Authro :  NYIT Johnnie Walker Group
 *
 * This file includes basic implementation for automatic walker
 * The implementation notes and known problems are listed below
 * We deliberately leaved out the implementations of tape-following parts
 * since they may not be used
 * but they can still be easily found in previous versions of the code
 * 
 * Note: The version of arduino IDE used to develop this program is "Arduino 0022"
 *       It will possibily cause errors if other versions of IDE is used
 *
 * Note: In order to upload code to arduino board make sure the bluetooth module is no connected to the TX and RX pins
 *       Detach BT module upload code then reattach
 *
 * The lasted walker we tested on is the one with 
 * 1. Two motors connected to the sabertooth motor shield
 * 2. Six ultrasonic sensors with two on the left, two in the front and two on the right
 *
 * Implemented
 * 1. Wall-following algorithm using basic PID control
 * 2. Receive and respond to commands from an android app
 *
 * Known problems/unimplemented parts
 * 1. The secenario with a wall in front is not properly solved, a delay algorithm has been implemented but is unstable
 * 2. PID weights/coefficients needed to be adjusted to adapt to different environments
 * 3. Cannot send out information to android app
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

// pins of side sensors
// from left --> right clockwise
// rearleft frontleft leftfront centerfront rightfront frontright rearright
const int ULTRASONIC_SENSOR_TOTAL = 6;
int ultrasonicSensorPins[ULTRASONIC_SENSOR_TOTAL] = {30, 31, 32, 33, 34, 35}; 
int ultrasonicSensorValues[ULTRASONIC_SENSOR_TOTAL] = {0, 0, 0, 0, 0, 0};

// definitions related to joysticks
#define JOYSTICK_BAUDRATE 9600  // Set baudrate of Joystick sensors to 9600
const int JOYSTICK_X = 1;       // Analog input pin A1 that the X-axis of joystick is attached to
const int JOYSTICK_Y = 2;      // Analog input pin A2 that the Y-axis of joystick is attached to
int joystickX = 0;   // Horizontal value from Joystick sensor
int joystickY = 0;  // Vertical value from Joystick sensor
// definitionss related to mode change buttons
const int BUTTONPIN = 7;  // Digital pin 7 reference for pushbutton

#define MODE_JS 0
#define MODE_AUTO 1
#define PAUSE 2

boolean clearedwall;
boolean isbluetoothon = true;  

int mode = 0;  // Variable to store the current mode, 0 for joystick mode, 1 for integration mode
int buttonState = 0;

// for speed tracking purpose
int currentSpeedLeft = 0, currentSpeedRight = 0;

//Values to adjust for enviornment calibration
int baseSpeed = 20;   //speed to begin walker at
int maxSpeed = 25;    //range 1 -> 62
int rMaxSpeed = -17;  //reverse max speed range -1 -> -62 
int minDis = 20, maxDis = 100; //used in PID wall following aka auto mode

//PID Adjustment Variables
int maxfrontwalldistance = 50;     //first buffer, the walker has enough time to make a turn or adjustment
int minfrontwalldistance = 20;  //second buffer, the walker is to close and can no longer turn out of the way
int cycle = 20;

/*Adjust the value of brick to test different possible brick wall solutions, which integrate the front sensors into the 
* wall following mode
*    when brick = 0 the walker stops and enters joystick mode at the first buffer, the user make the turn using the joystick and presses the yellow button to reenter auto mode
*    when brick = 1 the walker makes a delayed turn based on the direction of the joystick
*    when brick = 2 the walker makes a turn based on the sensor values and resumes auto mode once the front sensors are no longer obstructed
*/
int brick = 2;
int brickwalldelay = 5000; //1used only when brick=1   5000ms = 5sec

/**
  * Functions for intializations
  **/
void initMotors() {
  pinMode(MOTOR_TX_PIN, OUTPUT); // Set digital pin (transmit pin) to output mode
  MotorSerial.begin(MOTOR_BAUDRATE); // Set the Serial baud rate for the motor shield 
  delay(20); // 0.02 seconds wait for motor serial to initialize
  setMotorSpeed(currentSpeedLeft, currentSpeedRight); // Initial motors speed
}

void initSensors() {
  for(int i = 0; i < ULTRASONIC_SENSOR_TOTAL; i++)
    pinMode(ultrasonicSensorPins[i], OUTPUT);
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
// for debugging purpose
void printUltrasonics() {
  for(int i = 0; i < ULTRASONIC_SENSOR_TOTAL; i++) {
    Serial.print(ultrasonicSensorValues[i]);
    Serial.print(" ");
  }
  Serial.println();
}
 
// Ultrasonic sensor values are stored
// as centimeters
void getUltrasonicSensorValues() {
  for(int i = 0; i < ULTRASONIC_SENSOR_TOTAL; i++) {
    int npin = ultrasonicSensorPins[i];
    pinMode(npin, OUTPUT);
    digitalWrite(npin, LOW);
    delay(2);
    digitalWrite(npin, HIGH);
    delay(2);
    digitalWrite(npin, LOW);
    pinMode(npin, INPUT);
    ultrasonicSensorValues[i] = pulseIn(npin, HIGH) / 58;
    delay(2);
  }
  if (!isbluetoothon){
  printUltrasonics();  //enable to view the data from the ultrasonic sensors
  Serial.println();
  delay(5);}
}

// helper function
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
// the maximum value of maxSpeed is 62
// which is the physical limit of the motor
// values larger than 62 will cause unknown problems

void setMotorSpeed(int left, int right) { 
  // truncate invalid values
  if(left > maxSpeed) left = maxSpeed;
  if(left < rMaxSpeed) left = rMaxSpeed;
  if(right > maxSpeed) right = maxSpeed;
  if(right < rMaxSpeed) right = rMaxSpeed;
  
  // if either of them is not zero
  if(left || right) {
    left = 192 + left;
    right = 64 + right;
  }
  
  // with Arduino 1.0 this function needs to be changed to write
  MotorSerial.print(left, BYTE);
  delay(10); // Serial communication with the motors
  MotorSerial.print(right, BYTE);
   
  // update current state
  currentSpeedLeft = left;
  currentSpeedRight = right;
}

/**
  * logic part
  **/
int leftBaseline, rightBaseline;

// baseline establishment
// this is called everytime the walker starts to
// follow a wall
// the original distance to the wall is stored
// and used as a baseline throughout the course
void establishUltrasonicBaseline() {
  getUltrasonicSensorValues();
  leftBaseline = max((ultrasonicSensorValues[0] + ultrasonicSensorValues[1]) / 2, minDis);
  rightBaseline = max((ultrasonicSensorValues[5] + ultrasonicSensorValues[6]) / 2, minDis);
}

// initialize logic
void initLogic() {
  establishUltrasonicBaseline();
  baseSpeed = 20;
  setMotorSpeed(20, 20);
}

// define variable for wall following
int pw = 0, iw = 0, dw = 0;
// this is deliberately seperated from the maxSpeed defined below
// since they have different logical meanings while sharing a similar name
int minSpeed = 20;
// functions
// pid control side wall following
void followSideWall() {
  // if no wall is in proper range
  // let the joystick take over!
  if(leftBaseline > maxDis && rightBaseline > maxDis) { 
    //\joystickControl();
    return;
  }
  
  // pid control self driving
  // cross track error
  int lr = ultrasonicSensorValues[0];
  int lf = ultrasonicSensorValues[1];
  int rf = ultrasonicSensorValues[4];
  int rr = ultrasonicSensorValues[5];
  int cte = leftBaseline < rightBaseline ? 
            leftBaseline - (lf + lr)/2//  + (lr-lf)/2
            : (rf + rr)/2 - rightBaseline;// + (rf-rr)/2;
            
  // calculated pid
  dw = cte - pw; // D
  if(abs(dw) > 20) dw = 20;
  iw += cte; // I
  pw = cte; // P
  
  // the coefficients should be adjusted for different environments
  int correction = pw/3 + dw/10; 
  //Serial.print("correction: ");
  //Serial.println(correction);
  
  // bound max correction by taking the distance to wall and speed into consideration
  int maxCorrection = 12-(min(leftBaseline, rightBaseline)/10) + baseSpeed/12;
  if(abs(correction) > maxCorrection) correction = correction / abs(correction) * maxCorrection;
  
  // control the motors
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
// this function is used to respond to
// joystick controls while following the wall
// joystick can be used to adjust the speed and modify the distance to walls
// which is implemented by modifying the baseline at runtime
void respondToJoystick() {
  
  int x = joystickX, y = joystickY; 
  jxcc += sq(x-50)*((x-50)/abs(x-50)) / 10; // n/abs(n) is the sign!
  jycc += sq(y-50)*((y-50)/abs(y-50)) / 10; 
  
  // change baselines based on x input from joystick
  leftBaseline -= jxcc / cycle;
  rightBaseline += jxcc / cycle;
  // roll back if they fall of the cliff
  if(leftBaseline < minDis || rightBaseline < minDis) {
    leftBaseline += jxcc / cycle;
    rightBaseline -= jxcc / cycle;
  }
  // bound the max value
  // which is necessary since ultrasonic has a limit sense range
  if(leftBaseline > maxDis) leftBaseline = min(ultrasonicSensorValues[0], ultrasonicSensorValues[1]);
  if(rightBaseline > maxDis) rightBaseline = min(ultrasonicSensorValues[5], ultrasonicSensorValues[6]);
  jxcc %= cycle;
  
  // change speed based on y input from joystick
  baseSpeed += jycc / cycle;
  if(baseSpeed > maxSpeed) baseSpeed = maxSpeed;
  if(baseSpeed < minSpeed + 5) baseSpeed = minSpeed + 5;
  jycc %= cycle;
}

// handle the front wall
// only works when brick = 0
// it will only stop with this version of implementation
// more thoughts should be put into brick wall handling
boolean respondToFrontWall() {
  int left = ultrasonicSensorValues[2], right = ultrasonicSensorValues[3];
  if(left < maxfrontwalldistance || right < maxfrontwalldistance) { 
    setMotorSpeed(minSpeed, minSpeed); //minimum speed which will allow turns
    mode = MODE_JS; // transition to mannual mode
    return true; // front wall is encountered
  }
  return false; // no front wall encountered
}

// front wall handler
// used when walker is in joystick mode
// this function causes the walker to never run into a wall
boolean jsrespondToFrontWall(){
  int left = ultrasonicSensorValues[2], right = ultrasonicSensorValues[3];
  if(left < minfrontwalldistance || right < minfrontwalldistance) {
        setMotorSpeed(0,0);
        return true;
  }
return false;
}

// priority right turn brick wall handler
// use of delays which is a very poor implementation, for testing purposes only
// this funtion is only used when brick = 1
void brickrespondToFrontWall(){
  int left = ultrasonicSensorValues[2], right = ultrasonicSensorValues[3];

  if(left < maxfrontwalldistance || right < maxfrontwalldistance) { 
    getJoystickValues();// choose side to turn algorithm
    int xx = joystickX;
    xx > 50 ? setMotorSpeed(0,minSpeed) : setMotorSpeed(minSpeed,0); //choose side
    //setMotorSpeed(minSpeed,0); // 90 degree RIGHT
    //setMotorSpeed(0.minSpeed); // 90 degree LEFT
    delay(brickwalldelay);
  }
}

// when brick = 2 this funtion is used
// this solution allows the walker to make turns to avoid obstacles based on the values received from the fron sensor
// this solution may be the way to go but requires further thought and a much better algorithm
void brickrespondnodelay(){
  int left = ultrasonicSensorValues[2], right = ultrasonicSensorValues[3];
  if(left < maxfrontwalldistance || right < maxfrontwalldistance) { 
    getJoystickValues();  

    getUltrasonicSensorValues();
    int leftside = ultrasonicSensorValues[0] + ultrasonicSensorValues[1], rightside = ultrasonicSensorValues[4] + ultrasonicSensorValues[5];
        if (leftside > rightside){
          setMotorSpeed(0,minSpeed);//begin right turn 
           clearedwall = false;
         }
        else {
        setMotorSpeed(minSpeed,0);
        clearedwall = false;
        }
                  respondToJoystick();
    while(!clearedwall){
      getUltrasonicSensorValues();
      getJoystickValues();
      respondToButton();
      respondToJoystick();
      jsrespondToFrontWall();

      if(ultrasonicSensorValues[2] > maxfrontwalldistance && ultrasonicSensorValues[3] > maxfrontwalldistance) 
      {
       clearedwall = true; 
      }
  }
}
}

// helper
int flip(int s) { return (s+1) % 2; }

// button!
void respondToButton() {
  buttonState = digitalRead(BUTTONPIN);  // get the state of button from the digital pin
  if(buttonState == HIGH) {
    while(buttonState == HIGH) buttonState = digitalRead(BUTTONPIN);
    if(!mode) initLogic(); // transition from joystick control to auto driving
    else initJsmode();
    mode = flip(mode);
  }
  delay(10);
}

// everytime the walker is switched to joystick mode
// the basespeed should be reset
void initJsmode() {
  baseSpeed = 20;
}

// joystick control
void joystickControl() {
  getJoystickValues();
  // bottom right is (0, 0)
  int x = joystickX;
  int y = joystickY;
  // directions
  int yDir = y >= 50 ? 1 : -1;
  int xDir = x > 50 ? 1 : -1;
  // base speed
  int temp = yDir > 0 ? (maxSpeed - 5) * (abs(y-50) > 20)
                      : (rMaxSpeed + 5) * (abs(y-50) > 20); //pressure speed adjustment
  int basev = temp * (abs(y - 50)-10) / 50;
  // turn factor
  int turnf = xDir * (x - 50) / 3;
  
  // turn left
  if(xDir == 1) setMotorSpeed(basev, basev + yDir*turnf);
  // turn right
  else setMotorSpeed(basev + yDir*turnf, basev);
}


// automatic mode
void modeAuto() {
  // get the values first
  getUltrasonicSensorValues();
  getJoystickValues();
  // respond to joystick
  respondToJoystick();
  // respond to brick wall
    if (brick == 1) { brickrespondToFrontWall(); }
    if (brick == 2) { brickrespondnodelay(); }
  else if(respondToFrontWall()){ return; } 
  // follow the side wall
  followSideWall();
  delay(10);
}

// joystick mode
void modeJs() {
  // front wall is the highest priority
  // since we don't want patients to get hurt
  getUltrasonicSensorValues();
  if(jsrespondToFrontWall()) return;
  joystickControl();
}

/**
  * Stuff for communication with Anrdoid app
  **/
void initCommunication() {
  Serial.begin(9600);
}
  
void pause() {
  setMotorSpeed(0, 0);
  mode = PAUSE; 
}

void resume() { mode = MODE_JS; }

void brickwall() { brick = brick == 0 ? 1 : 0; }  //allows button on android phone to switch between brick = 0 and brick = 1

// the value of baseSpeed will never go out of
// the boundaries since respondToJoystick() always
// provide checkings
void slowDown() { baseSpeed -= 2; }
void speedUp() { baseSpeed += 2; }

// turn by modifying baselines
void turnLeft() { 
  if(leftBaseline > minDis+1 && rightBaseline < maxDis-1) {
    leftBaseline -= 2;
    rightBaseline += 2;
  }
}

void turnRight() { 
  if(leftBaseline < maxDis+1 && rightBaseline > minDis-1) {
    leftBaseline += 2;
    rightBaseline -= 2;
  }
}

// decide whether the walker is reservable or not
void flipReverse() { 
  if(rMaxSpeed != 0) rMaxSpeed = 0;
  else rMaxSpeed = -maxSpeed/2;
}

void switchToAuto() {
  initLogic();
  mode = MODE_AUTO;
}

void switchToJs() { mode = MODE_JS; }

void sendCondition() {
  // not implemented
}

// the signal received from the android app
// is going to be characters
// which are encoded instructions
void respondToAndroid() {
  char c = Serial.read();
  switch(c) {
    case 'S': pause();
              break;
    case 'U': speedUp();
              break;
    case 'D': slowDown();
              break;
    case 'A': switchToAuto();
              break;
    case 'Q': turnLeft();
              break;
    case 'E': turnRight();
              break;
    case 'V': flipReverse();
              break;
    case 'J': switchToJs();
              break;
    case 'R': resume();
              break;
    case 'B': brickwall();
              break;
    default : ;// do nothing
  }
}

/**
  * main loop
  **/
void setup() {
  initPushbutton();
  initJoystick();
  initSensors();
  initMotors();
  initCommunication();
}

void loop() {
  respondToAndroid();
  respondToButton();
  sendCondition();
  if(mode == MODE_AUTO) modeAuto();
  else if(mode == MODE_JS) modeJs();
  // if mode == PAUSE, it's stopped, waiting for resume
}
