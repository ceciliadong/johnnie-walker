/***********************/
/**** Motor Controls
/***********************/
/**** Sabertooth v1.03 
/***********************/
/**** AME 218-2003 Industrial 12v Motors
/***********************/

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

#define MOTORRIGHT_REVERSE 1    // Values for motor1 to run reverse in full
#define MOTORLEFT_REVERSE  128  // Values for motor2 to run reverse in full
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
  if (RM1_Speed == -1 || LM2_Speed == -1)
    {
      speedMotorRight = MOTORRIGHT_REVERSE;
      speedMotorLeft  = MOTORLEFT_REVERSE;
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
  setMotorSpeed(4,4);  
}

void MotorsStop()
{
  setMotorSpeed(0,0);  
}

void MoveCircle()
{
  setMotorSpeed(-1,4)  // Move the walker in a clockwise direction
}

/***********************/
/**** Sensory Data
/***********************/
/**** Sharp IR 2Y0A21 
/***********************/

// Set baudrate of IR sensors to 9600
#define IR_BAUDRATE 9600

// Analog pins reference for IR sensors
const int LEFT_IR_F = A3;  // Left front IR sensor is Analog In pin A3
const int LEFT_IR_R = A4;  // Left rear IR sensor is Analog In pin A4 
const int RIGHT_IR_F = A5;  // Right front IR sensor is Analog In pin A5 
const int RIGHT_IR_R = A6;  // Right rear IR sensor is Analog In pin A6 
const int CENTER_IR_L = A7;  // Center left IR sensor is Analog In pin A7  
const int CENTER_IR_R = 8;  // Center right IR sensor is Analog In pin A8 

/* Define array to hold analog input values from IR sensors
/* Index represents different sensor
/* sensorValues[CenterLeft, CenterRight, LeftFront, LeftRear, RightFront, RightRear] */
int sensorValues [] = {0,0,0,0,0,0};
int boundary [3];

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
}

void setup()
{
  initMotors();
  initIRSensors();
  getBoundary();
}

void loop()
{
 /*getSensorValues();
 Serial.println(sensorValues [5]);
 delay(0);  //Test Sensor Values */
 
 modeWall();
}

/***********************/
/**** Follow Wall controls
/***********************/
/**** Using Side Sensors  
/***********************/
short rPos(int x, int y) {
  //Serial.println(x);
  //Serial.println(y);
  //Serial.println();
  //delay(100);
  short pos = 0;
  if(x > boundary[2] && y > boundary[2]) pos += 4;
  // front closer
  if(x - y >= 50) pos += 1;
  // back closer
  if(y - x >= 50) pos += 2;
  return pos;
}

// follow the wall prototype
void modeWall() {
  getSensorValues();
  short rpos = rPos(sensorValues[4], sensorValues[5]); 
  short pos = rpos;
  switch(pos) {
    case 0: MoveForward(); break;
    case 6: MoveForward(); break;
    case 1: turnLeft(); break;
    case 4: turnLeft(); break;
    case 5: turnLeft(); break;
    case 2: turnRight(); break;
  }
}

/***********************/
/**** Joystick Data
/***********************/


