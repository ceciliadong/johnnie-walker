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

#define MOTORRIGHT_REVERSE 32    // Values for motor1 to run reverse in full
#define MOTORLEFT_REVERSE  160  // Values for motor2 to run reverse in full
#define MOTORS_RANGE       63   // The integer space between values

/** The array values indicate 10 different motor speeds 
/** MOTORLEFT_FORWARD_SPEED[0] = MOTORLEFT STOP
/** MOTORLEFT_FORWARD_SPEED[1] = MOTORLEFT FORWARD SLOWEST
/** MOTORLEFT_FORWARD_SPEED[9] = MOTORLEFT FORWARD FASTEST  */

int MOTORRIGHT_FORWARD_SPEED[] = {64,   71,  78,  85,  92,  99, 106, 113, 120, 127};
int MOTORLEFT_FORWARD_SPEED[] =  {192, 199, 206, 213, 220, 227, 234, 241, 248, 255};

// Speeds for both two Motors
int speedMotorRight;  // Speed for motor 1
int speedMotorLeft;  // Speed for motor 2

// Speed Mode Control Values: Range 0 -> 10   -1 is reverse mode  0 is stop mode
int ControlMotorRight = 0;
int ControlMotorLeft  = 0;

void initMotors()
{
  pinMode(MOTOR_TX_PIN, OUTPUT);      // Set digital pin (transmit pin) to output mode
  MotorSerial.begin(MOTOR_BAUDRATE);  // Set the Serial baud rate for the motor shield
  
  delay(20);           // 0.02 seconds wait for motor serial to initialize
  setMotorSpeed(ControlMotorRight,ControlMotorLeft);  // Initial motors speed
}

void setMotorSpeed(int RM1_Speed, int LM2_Speed)
{
  if (RM1_Speed == -1 && LM2_Speed == -1)    // Move in Reverse
    {
      speedMotorRight = MOTORRIGHT_REVERSE;
      speedMotorLeft  = MOTORLEFT_REVERSE;
    }
  else if (RM1_Speed != -1 && LM2_Speed == -1)// Move Counter Clockwise Circle
    {
      speedMotorRight = MOTORRIGHT_FORWARD_SPEED[RM1_Speed];
      speedMotorLeft  = MOTORLEFT_REVERSE;
    }  
  else if (RM1_Speed == -1 && LM2_Speed != -1)// Move Clockwise Circle
    {
      speedMotorRight = MOTORRIGHT_REVERSE;
      speedMotorLeft  = MOTORLEFT_FORWARD_SPEED[LM2_Speed];
    }  
  else
    {
      speedMotorRight = MOTORRIGHT_FORWARD_SPEED[RM1_Speed]; //Grab speed from motor speed array
      speedMotorLeft  = MOTORLEFT_FORWARD_SPEED[LM2_Speed];
    }
    
      MotorSerial.print(speedMotorRight,BYTE);  // with Arduino 1.0 this function needs to be changed to write 
      delay(10);                                // Serial communication with the motors
      MotorSerial.print(speedMotorLeft,BYTE);
    
      ControlMotorRight = RM1_Speed;  //Track current motor speed mode
      ControlMotorLeft  = LM2_Speed;
}

void turnLeft()
{
    setMotorSpeed(6,4);  // Turn Left by reducing the speed of left motor  
}

void turnRight()
{
    setMotorSpeed(4,6);  // Turn Right by reducing the speed of right motor
}

void MoveBackward()
{
    setMotorSpeed(-1,-1);
}

void MoveForward() // Choose the motor that is moving the fastest and continue with that speed
{
  setMotorSpeed(5,5);  
}

void MotorsStop()
{
  setMotorSpeed(0,0);  
}

void MoveCircle_CW()
{
  setMotorSpeed(-1,4);  // Move the walker in a clockwise direction
}

void MoveCircle_CC()
{
  setMotorSpeed(4,-1);  // Move the walker in a clockwise direction
}

/***********************/
/**** Sensory Data
/***********************/
/**** Sharp IR 2Y0A21 
/***********************/

// Set baudrate of IR sensors to 9600
#define IR_BAUDRATE 9600

// Analog pins reference for IR sensors
const int LEFT_IR_F   = A3;  // Left front IR sensor is Analog In pin A3
const int LEFT_IR_R   = A4;  // Left rear IR sensor is Analog In pin A4 
const int RIGHT_IR_F  = A5;  // Right front IR sensor is Analog In pin A5 
const int RIGHT_IR_R  = A6;  // Right rear IR sensor is Analog In pin A6 
const int CENTER_IR_L = A7;  // Center left IR sensor is Analog In pin A7  
const int CENTER_IR_R = A8;  // Center right IR sensor is Analog In pin A8 

/* Define array to hold analog input values from IR sensors
/* Index represents different sensor
/* sensorValues[CenterLeft, CenterRight, LeftFront, LeftRear, RightFront, RightRear] */
int sensorValues [] = {0,0,0,0,0,0};
// center, left, right
int boundary [3];
boolean leftCloser;

void initIRSensors()
{
  // Initalize all ports communicate with IR sensors
  pinMode( CENTER_IR_L, INPUT );
  pinMode( CENTER_IR_R, INPUT );
  pinMode( LEFT_IR_F, INPUT );
  pinMode( LEFT_IR_R, INPUT );
  pinMode( RIGHT_IR_F, INPUT );
  pinMode( RIGHT_IR_R, INPUT );

  Serial.begin(IR_BAUDRATE);
  delay(20); // 0.002 second time delay for the IR sensors to init
}

void getSensorValues()
{
  sensorValues [0] = analogRead(CENTER_IR_L);
  sensorValues [1] = analogRead(CENTER_IR_R);
  sensorValues [2] = analogRead(LEFT_IR_F);
  sensorValues [3] = analogRead(LEFT_IR_R);
  sensorValues [4] = analogRead(RIGHT_IR_F);
  sensorValues [5] = analogRead(RIGHT_IR_R);
}

void getBoundary()
{
  getSensorValues();
  for(int i = 0; i < 3; i++) {
    boundary[i] = (sensorValues[2*i] + sensorValues[2*i+1])/2;
    if(boundary[i] > 600 || boundary[i] < 200) boundary[i] = 400;
  }
  leftCloser = boundary[1] > boundary[2];
}

/********************************/
/**** Front Sensors
/********************************/
// front wall dance
void frontWallDance() {
    MoveBackward();
    delay(2000);
    MoveCircle_CW();
    delay(2000);
}

/******************************************/
/*** Follow Wall controls Using Side Sensors 
/******************************************/ 

// get the encoded position of the walker
short rPos(int x, int y) {
  short pos = 0;
  if(x < 200 && y < 200) return 0;
  if(x > boundary[2] && y > boundary[2]) pos += 4;
  // front closer
  if(x - y >= 50) pos += 1;
  // back closer
  if(y - x >= 50) pos += 2;
  return pos;
}

short lPos(int x, int y) {
  short pos = 0;
  if(x < 200 && y < 200) return 0;
  if(x > boundary[1] && y > boundary[1]) pos += 4;
  if(x - y >= 50) pos += 1;
  if(y - x >= 50) pos += 2;
  return pos;
}

// follow the wall smooth working version
// only right side is implemented
void modeWall() {
  getSensorValues();
  int fSen = sensorValues[1];
  if(fSen > 600) frontWallDance();
  int rf = sensorValues[4], rb = sensorValues[5];
  int lf = sensorValues[2], lb = sensorValues[3];
  short rpos = rPos(rf, rb); 
  short lpos = lPos(lf, lb);
  // short pos = (lpos << 3) + rpos;
  // left position processing and
  // right position processing
  // are seperated
  if(!leftCloser) switch(rpos) {
    case 0: MoveForward(); break;
    case 6: MoveForward(); break;
    case 1: turnLeft(); break;
    case 4: turnLeft(); break;
    case 5: turnLeft(); break;
    case 2: {
      if(rf < 250) delayTurnRight();
      else turnRight();
      break;
    }
  }
  else switch(lpos) {
    case 0: MoveForward(); break;
    case 6: MoveForward(); break;
    case 1: turnRight(); break;
    case 4: turnRight(); break;
    case 5: turnRight(); break;
    case 2: {
      if(lf < 250) delayTurnLeft();
      else turnLeft();
      break;
    }
  } 
}


// advanced motions
void delayTurnRight() {
  int rf, rb;
  do {
    getSensorValues();
    rb = sensorValues[5];
    setMotorSpeed(5, 4);
  } while(rb > 250);
  do {
    getSensorValues();
    rf = sensorValues[4];
    setMotorSpeed(2, 6);
  } while(rf < boundary[2]);
}

// stupidity
// should've put both delay turn together
void delayTurnLeft() {
  int lf, lb;
  do {
    getSensorValues();
    lb = sensorValues[3];
    setMotorSpeed(4, 5);
  } while(lb > 250);
  do {
    getSensorValues();
    lf = sensorValues[2];
    setMotorSpeed(6, 2);
  } while (lf < boundary[1]);
  delay(500);
}

// experimental mode
// shakes like crazy
void modeDance() {
  getSensorValues();
  frontWallDance();
  int x = sensorValues[4];
  int y = sensorValues[5];
  if(x > boundary[2]) turnLeft();
  else if(y > boundary[2]) turnRight();
  else MoveCircle_CW();
}

/***********************/
/**** Joystick Data
/***********************/

#define JOYSTICK_BAUDRATE 9600  // Set baudrate of Joystick sensors to 9600
const int JOYSTICK_X = 2;       // Analog input pin A0 that the X-axis of joystick is attached to
const int JOYSTICK_Y = 1;      // Analog input pin A1 that the Y-axis of joystick is attached to

int JoyStick_X_VAL = 0;   // Horizontal value from Joystick sensor
int JoyStick_Y_VAL = 0;  // Vertical value from Joystick sensor

void initJoystick()
{
  // Initalize all ports communicate with Joystick sensor
  pinMode( JOYSTICK_X, INPUT );
  delay(10);
  pinMode( JOYSTICK_Y, INPUT );
  Serial.begin(JOYSTICK_BAUDRATE);  
  
  // 0.02 second time delay for the IR sensors to init 
  delay(20);
}

void getJoyStickValue ()
{
  JoyStick_X_VAL = normalize(analogRead(JOYSTICK_X));
  delay(10);
  JoyStick_Y_VAL = normalize(analogRead(JOYSTICK_Y));
}

int normalize (int value)
{
  //return value;
  if(value > 400 && value < 600) return 50;
  return (value / 10); // Transforms the joystick's messurement into a values between 0 and 100 
}

/***************************/
/*** Joystick Mode controls
/***************************/

// joystick control prototype
void modeJs() {
  getJoyStickValue();
  int x = JoyStick_X_VAL;
  int y = JoyStick_Y_VAL;
  if(y < 50) {
    MoveBackward();
    return;
  }
  int dir = x > 50 ? 1 : -1;
  // assume the range is [0, 100]
  int basev = (y - 50) / 10; // base speed
  int turnf = dir * (x - 50) / 15; // turn factor
  //turn left
  if(dir == 1) setMotorSpeed(basev+turnf, basev);
  //turn right
  else setMotorSpeed(basev, basev+turnf);
}


/********************************/
/**** Mode Selector Yellow Button
/********************************/

const int buttonPin = 7;  // Digital pin 8 reference for pushbutton
int mode = 0;  // Variable to store the current mode
int buttonState;

void initPushbutton()
{
  pinMode(buttonPin, INPUT);  // initialize the pushbutton pin as an input
}

void changeMode()
{
  buttonState = digitalRead(buttonPin);  // get the state of button from the digital pin
   
  if (mode == 0){
    //The button is pushed
    if (buttonState == HIGH) {   
      mode = 1;  // The mode changed to 1 for the walker
      delay(300);
      getBoundary();
    }      
  } else if (mode == 1) {
    // The button is pushed
    if (buttonState == HIGH) {
      mode = 0;  // The mode changed to 0 for the walker 
      delay(300);
    } 
  }
}

/***********************/
/** BEGIN MOVEMENT **/
/***********************/

void setup()
{
  initMotors();
  initIRSensors();
  initJoystick();
}

void loop()
{
   /*getSensorValues();
    Serial.println(sensorValues [5]);
    delay(0);  */
 
  changeMode();
  if (mode==1){
     //modeDance();
     modeWall();
  }
  modeJs();
 //modeWall();

 
    //modeDance();
   //MoveBackward();
   //delay(2000);
   //MoveCircle_CW();
   //delay(2000);
   //MoveCircle_CC();
   //delay(2000);
}
