/****************************************/
/**** Motor Controls
/****************************************/
/**** Sabertooth v1.03 
/****************************************/
/**** AME 218-2003 Industrial 12v Motors
/****************************************/
#include <SoftwareSerial.h>
#include <stdarg.h>
/***********************************************************/
/**Labels for use with the Sabertooth 2x5 motor controller**/
/**Refer to \Jonnie Walker Project\Motor Shield\Sabertooth2X5QuickStart.doc
/***********************************************************/
// Digital pin 18 is the serial transmit pin to the Sabertooth 2x5
#define MOTOR_TX_PIN 18
// Digital pin 19 is the serial receive pin to the Sabertooth 2x5, but is not used, needed for SoftwareSerial Function
#define MOTOR_RX_PIN 19
// Set to 9600 through Sabertooth dip switch 4 down
#define MOTOR_BAUDRATE 9600
// Create a object to control Sabertooth 2x5 motor controller
SoftwareSerial MotorSerial = SoftwareSerial(MOTOR_RX_PIN, MOTOR_TX_PIN);
/* Simplified serial Limits for each motor
   Commands are sent as single bytes. 
   Sending a value of 1-127 will command motor 1 Sending a value of 128-255 will command motor 2. 
   Sending a value of 0 will shut down both motors. */
int MOTORLEFT_FORWARD_SPEED[] =  {192, 199, 206, 213, 220, 227, 234, 241, 248, 255};
int MOTORRIGHT_FORWARD_SPEED[] = { 64,  71,  78,  85,  92,  99, 106, 113, 120, 127};
int MOTORLEFT_REVERSE_SPEED[] =  {191, 184, 177, 170, 163, 156, 149, 142, 135, 128}; // from slow to fast
int MOTORRIGHT_REVERSE_SPEED[] = { 63,  57,  50,  43,  36,  29,  22,  15,   8,   1}; // from slow to fast

int currentSpeedLeft = 0, currentSpeedRight = 0;

void initMotors() {
  pinMode(MOTOR_TX_PIN, OUTPUT);      // Set digital pin (transmit pin) to output mode
  MotorSerial.begin(MOTOR_BAUDRATE);  // Set the Serial baud rate for the motor shield 
  delay(20);           // 0.02 seconds wait for motor serial to initialize
  setMotorSpeed(currentSpeedLeft, currentSpeedRight);  // Initial motors speed
}

void setMotorSpeed(int left, int right) {	
  // decide the exact speed value
  int leftSpeed = left >= 0 ? MOTORLEFT_FORWARD_SPEED[left] : MOTORLEFT_REVERSE_SPEED[-left];
  int rightSpeed = right >= 0 ? MOTORRIGHT_FORWARD_SPEED[right] : MOTORRIGHT_REVERSE_SPEED[-right];
  // send value to arduino
  MotorSerial.print(leftSpeed, BYTE);  // with Arduino 1.0 this function needs to be changed to write 
  delay(10);                                // Serial communication with the motors
  MotorSerial.print(rightSpeed, BYTE);
  // save current stat
  currentSpeedLeft = left;
  currentSpeedRight = right; 
}

void turnLeft() { setMotorSpeed(4, 5); }
void turnRight() { setMotorSpeed(5, 4); }
void moveBackward() { setMotorSpeed(-5, -5); }
void moveForward() { setMotorSpeed(5, 5); }
void motorStop() { setMotorSpeed(0, 0); }
void moveCW() { setMotorSpeed(5, -5); } // move clockwise
void moveCC() { setMotorSpeed(-5, 5); } // move counter clockwise

/***********************/
/**** Sensory Data
/***********************/
/**** Sharp IR 2Y0A21 
/***********************/
// Set baudrate of IR sensors to 9600
#define IR_BAUDRATE 9600
/* Define array to hold analog input values from IR sensors
/* Index represents different sensor
/* sensorValues[CenterLeft, CenterRight, LeftFront, LeftRear, RightFront, RightRear] */
// 0 - 5, side sensors, 6 - 11 photo sensors on the bottom
int sensorPins[] = {7, 8, 3, 4, 5, 6, 9, 10, 11, 12, 13, 14};
int sensorValues[12];

void initIRSensors() {
  // Initalize all ports communicate with IR sensors
  for(int i = 0; i < 12; i++) pinMode(sensorPins[i], INPUT);
  Serial.begin(IR_BAUDRATE);
  delay(20); // 0.002 second time delay for the IR sensors to init
}

void getSensorValues() {
  for(int i = 0; i < 12; i++) sensorValues[i] = analogRead(sensorPins[i]);
}

/*****************************
*** SENSOR LEARNING SECTION
*****************************/
// front, left, right
int baseLine[3];
// which side the walker is closer to
// for now, the walker will only follow the closer side
// thie scheme may cause bugs, such as when both walls are at the exact same distance
// but it is good enough for now
boolean leftCloser; 

void establishBaseline() {
  getSensorValues();
  for(int i = 1; i < 3; i++) {
    baseLine[i] = (sensorValues[2*i] + sensorValues[2*i+1])/2;
    if(baseLine[i] > 600 || baseLine[i] < 200) baseLine[i] = 400;
  }
  leftCloser = baseLine[1] > baseLine[2];
  baseLine[0] = leftCloser ? baseLine[1] : baseLine[2];
}

void updateBaseLine() {
  getSensorValues();
  int tmp;
  for(int i = 1; i < 3; i++) {
    tmp = ((sensorValues[2*i] + sensorValues[2*i+1])/2 + baseLine[i])/2;
    if(tmp > 200 && tmp < 600 && (tmp-baseLine[i] < 50) && (tmp-baseLine[i] > -50)) baseLine[i] = tmp;
  }
}

/*************************************
/***** Mode 0 follow black tape\
*************************************/
const int THRESHOLD = 600;

int downCondition() {
  getSensorValues();
  for(int i = 0; i < 6; i++) {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(sensorValues[i+6]);
  }
  Serial.println();
  int result = 0;
  for(int i = 0; i < 6; i++)
    if(sensorValues[6+i] > THRESHOLD) result += i > 2 ? 1 : 10;
  return result;
}

void modeZero() {
  int condition = downCondition();
  Serial.println(condition);
  int l = condition / 10, r = condition % 10;
  if(l && r) motorStop();
  else if(l) {
    //if(l > 2) setMotorSpeed(1, 5);
    //else 
    turnLeft();
  }
  else if(r) {
    //if(r > 2) setMotorSpeed(5, 1);
    //else
    turnRight();
  }
  else moveForward();
}

/******************************************/
/*** Follow Wall controls Using Side Sensors 
/******************************************/ 
// get the encoded position of the walker
short lPos(int x, int y) {
  short pos = 0;
  if(x < 200 && y < 200) return 0;
  if(x > baseLine[1] && y > baseLine[1]) pos += 4;
  if(x - y >= 50) pos += 1;
  if(y - x >= 50) pos += 2;
  return pos;
}

short rPos(int x, int y) {
  short pos = 0;
  if(x < 200 && y < 200) return 0;
  if(x > baseLine[2] && y > baseLine[2]) pos += 4;
  // front closer
  if(x - y >= 50) pos += 1;
  // back closer
  if(y - x >= 50) pos += 2;
  return pos;
}

// follow the wall smooth working version
// only right side is implemented
void modeWall() {
  getSensorValues();
  int fl = sensorValues[0], fr = sensorValues[1];
  // if there is a wall in front
  if(fl > baseLine[0] || fr > baseLine[0]) {
    if(leftCloser) frontLeftWallTurn();
    else frontRightWallTurn();
  }
  int rf = sensorValues[4], rb = sensorValues[5];
  int lf = sensorValues[2], lb = sensorValues[3];
  short rpos = rPos(rf, rb); 
  short lpos = lPos(lf, lb);
  if(leftCloser) switch(lpos) {
    case 0: moveForward(); break;
    case 6: moveForward(); break;
    case 1: turnRight(); break;
    case 4: turnRight(); break;
    case 5: turnRight(); break;
    case 2: {
      if(lf < 250) delayTurnLeft();
      else turnLeft();
      break;
    }
  }
  else switch(rpos) {
    case 0: moveForward(); break;
    case 6: moveForward(); break;
    case 1: turnLeft(); break;
    case 4: turnLeft(); break;
    case 5: turnLeft(); break;
    case 2: {
      if(rf < 250) delayTurnRight();
      else turnRight();
      break;
    }
  }
}

/************************
*****ADVANCED MOTIONS
************************/
void delayTurnRight() {
  int rf, rb;
  do {
    getSensorValues();
    rb = sensorValues[5];
    setMotorSpeed(4, 5);
  } while(rb > 250);
  do {
    getSensorValues();
    rf = sensorValues[4];
    setMotorSpeed(6, 2);
  } while(rf < baseLine[2]);
}

// stupidity
// should've put both delay turn together
void delayTurnLeft() {
  int lf, lb;
  do {
    getSensorValues();
    lb = sensorValues[3];
    setMotorSpeed(5, 4);
  } while(lb > 250);
  do {
    getSensorValues();
    lf = sensorValues[2];
    setMotorSpeed(2, 6);
  } while (lf < baseLine[1]);
}

// for now it will stop instead of turn
void frontRightWallTurn() {
  motorStop();
}

void frontLeftWallTurn() {
  motorStop();
}

/***********************/
/**** Joystick Data
/***********************/
#define JOYSTICK_BAUDRATE 9600  // Set baudrate of Joystick sensors to 9600
const int JOYSTICK_X = 2;       // Analog input pin A0 that the X-axis of joystick is attached to
const int JOYSTICK_Y = 1;      // Analog input pin A1 that the Y-axis of joystick is attached to

int JoyStick_X_VAL = 0;   // Horizontal value from Joystick sensor
int JoyStick_Y_VAL = 0;  // Vertical value from Joystick sensor

void initJoystick() {
  // Initalize all ports communicate with Joystick sensor
  pinMode( JOYSTICK_X, INPUT );
  delay(10);
  pinMode( JOYSTICK_Y, INPUT );
  Serial.begin(JOYSTICK_BAUDRATE);  
  // 0.02 second time delay for the IR sensors to init 
  delay(20);
}

void getJoyStickValue() {
  JoyStick_X_VAL = normalize(analogRead(JOYSTICK_X));
  delay(10);
  JoyStick_Y_VAL = normalize(analogRead(JOYSTICK_Y));
}

int normalize(int value) {
  //return value;
  if(value > 400 && value < 600) return 50;
  return (value / 10); // Transforms the joystick's messurement into a values between 0 and 100 
}

/***************************/
/*** Joystick Mode controls
/***************************/
// joystick control with boolean return value
void modeJs() {
  getJoyStickValue();
  // bottom right is (0, 0)
  int x = JoyStick_X_VAL;
  int y = JoyStick_Y_VAL;
  int yDir = y >= 50 ? 1 : -1;
  int xDir = x > 50 ? 1 : -1;
  int basev = (y - 50) / 10; // base speed
  int turnf = xDir * (x - 50) / 15; // turn factor
  // turn left
  if(xDir == 1) setMotorSpeed(basev, basev + yDir*turnf);
  // turn right
  else setMotorSpeed(basev + yDir*turnf, basev);
}

/********************************/
/**** Mode Selector Yellow Button
/********************************/
const int buttonPin = 7;  // Digital pin 8 reference for pushbutton
int mode = 0;  // Variable to store the current mode
int buttonState;

void initPushbutton() {
  pinMode(buttonPin, INPUT);  // initialize the pushbutton pin as an input
}

void changeMode() {
  buttonState = digitalRead(buttonPin);  // get the state of button from the digital pin
  if (mode == 0) {
    //The button is pushed
    if (buttonState == HIGH) {   
      mode = 1;  // The mode changed to 1 for the walker
      delay(300);
      establishBaseline();
    }      
  } else if (mode == 1) {
    // joystick interuption
    getJoyStickValue();
    int x = JoyStick_X_VAL;
    int y = JoyStick_Y_VAL;
    if (x != 50 || y != 50 || buttonState == HIGH) {
      mode = 0;  // The mode changed to 0 for the walker 
      delay(300);
    } 
  }
}

/***********************/
/** BEGIN MOVEMENT **/
/***********************/
void setup() {
  initMotors();
  initIRSensors();
  initJoystick();
}

void loop() {
  modeZero();
  /*
  changeMode();
  if (mode==1) modeWall();
  else modeJs();
  */
}
