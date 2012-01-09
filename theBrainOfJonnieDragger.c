/**
 * Author	:	Jay Zhuang
 * Date		:	Dec 5, 2011
 *
 * This is the C code used by the walker to automate
 * its movement and respond to certain conditions
 *
 * This file is a start point of our work 
 * based on the source code written by Admin Larkin
 * 
 * The aim of the first version is to 
 * clean up original source code and 
 * modify/improve its readabily based on
 * our gourp's perspective
 *
 * For easier navigation thourgh this file
 * This file is divided into several sections
 * Search for the section title to jump to certain section
 *
 * Section Titles:
 *
 *  DEFINE SECTION
 *  INIT SECTION
 *  VALUE GETTERS SECTION
 *  LOGIC & CONTROL SECTION
 *		LOGIC SECTION
 *		CONTROL SECTION
 *  OUTPUT SECTION
 *  ARDUINO SECTION
 *
 **/

// Original Header
/*******************************************************************************************************************************/
/**** Programmed by Adam Larkin ********* Created at EST 22:01, 2011-09-14 ********** Last Updated at EST 18:00, 2011-09-14 ****/
/****                                                       V0.5.01                                                         ****/
/****                                                                                                                       ****/
/****          The codes intend use a motor controller to drive two motors with the control of a Joystick sensor.           ****/
/****          The working conditions are limited by six IR sensors. Two are located on the front side of the car.          ****/
/****                  Two are located on the right side. The other two are located on the left side.                       ****/
/****        They will limit the motors in case of it is too close to or too far away from a target, such as a wall.        ****/
/****                            Detailed documents later will be released on my webstie.                                   ****/
/****                                                                                                                       ****/
/****                                  Email & Gtalk: alark51@gmail.com                                                     ****/
/****                                                                                                                       ****/
/*******************************************************************************************************************************/
/****                                                                                                                       ****/
/****           You can use the codes without any limitation. But when you use it, please keep all information.             ****/
/****                 No errors occupied when compiled. But there must be some bugs when tested in reality.                 ****/
/****             Pleas contact us when you find anything wrong, especially logical errors. I'll appreciate it.             ****/
/****                                                                                                                       ****/
/*******************************************************************************************************************************/
/**** 2011-06-07 18:00   Fix a bug for emergency stop.                                                                      ****/
/**** 2011-05-09 02:00   Add a new method to adjust speed by itself in 3 seconds under all conditions.                      ****/
/**** 2011-04-28 15:34   Back to v3.10, and use new IR sensors.                                                             ****/
/**** 2011-04-03 10:20   Add some delays in different part to avoid running jerk. Just try.                                 ****/
/**** 2011-04-02 13:02   Add a new pushbutton sensor, and program it. Now the system can switch modes.                      ****/
/**** 2011-04-01 00:15   Fix a bug that the walker will run automatically                                                   ****/
/**** 2011-03-19 16:31   Find a good way to keep the range of IR sensors between 20cm and 150cm.                            ****/
/**** 2011-03-15 23:57   Find a wrong formula in the motor speed control part. Bug fixed. It works for the hardware system. ****/
/**** 2011-03-15 02:14   Fix a bug found in the motor speed part. Find a better way to control speeds of two motors.        ****/
/**** 2011-03-13 03:11   Finish main codes. Need to be tested it in reality.                                                ****/
/*******************************************************************************************************************************/

#include <SoftwareSerial.h>	// Arduino library for serial communication
#include <stdarg.h>			// C library for va_list
							// which is used in the function p(char*) by the original author
							// for some sort of output


/**
 *
 * DEFINE SECTION
 *
 * This section contains a whole bunch of value
 * defined by the original author
 *
 **/
/***********************************************************/
/**Labels for use with the Sabertooth 2x5 motor controller**/
/***********************************************************/

// Digital pin 18 is the serial transmit pin to the Sabertooth 2x5
#define SABER_TX_PIN 18

// NOT USED (but still init'd)
// Digital pin 19 is the serial receive pin from the Sabertooth 2x5
#define SABER_RX_PIN 19

// Set to 9600 through Sabertooth dip switches
#define SABER_BAUDRATE 9600

// Simplified serial Limits for each motor
#define SABER_MTRRIGHT_FULL_FORWARD  127  // Values for motor1 to run forward in full
#define SABER_MTRRIGHT_FULL_REVERSE    1  // Values for motor1 to run reverse in full
#define SABER_MTRRIGHT_FULL_STOP      64  // Values for motor1 to stop

#define SABER_MTRLEFT_FULL_FORWARD  255  // Values for motor2 to run forward in full
#define SABER_MTRLEFT_FULL_REVERSE  128  // Values for motor2 to run reverse in full
#define SABER_MTRLEFT_FULL_STOP     192  // Values for motor2 to stop
#define SABER_MOTOR_RANGE           63

// Motor level to send when issuing the full stop command
#define SABER_ALL_STOP 0

// Speeds for both two Motors
float speedMotorRight = 64;  // Speed for motor 1
float speedMotorLeft = 192;  // Speed for motor 2

float smoothMotorRight = 64;
float smoothMotorLeft = 192;
float MotorAlpha = 4;

float X1 = 225;
float X2 = 350;
float X3 = 450;

float Y1 = 188;
float Y2 = 288;
float Y3 = 438;

float YR  = 0.2;
float YP1 = 0.2;
float YP2 = 0.3;
float YP3 = 0.4;
float YP4 = 0.6;

float XP1 = 0.55;
float XP2 = 0.65;
float XP3 = 0.75;
float XP4 = 0.8;

// Create a object to control Sabertooth 2x5 motor controller
SoftwareSerial SaberSerial = SoftwareSerial(SABER_RX_PIN, SABER_TX_PIN);

/************************************************/
/**Labels for use with the 1113 Joystick sensor**/
/************************************************/

// Analog Pins reference for Joystick sensor
const int JOYSTICK_X = 2;  // Analog input pin A0 that the X-axis of joystick is attached to 
const int JOYSTICK_Y = 1;  // Analog input pin A1 that the Y-axis of joystick is attached to

// Set baudrate of Joystick sensors to 9600
#define JOYSTICK_BAUDRATE 9600

// Define variables to hold analog input values from Joystick sensor
float JoyStick_X_VAL = 0;  // Horizontal value from Joystick sensor
float JoyStick_Y_VAL = 0;  // Vertical value from Joystick sensor

int JoyAlpha = 1;

// The values when joystick stay in the center
const float Center_X_VAL = 512.00;  // Horizontal value when Joystick stay in center
const float Center_Y_VAL = 512.00;  // Vertical value when Joystick stay in center

/*******************************************************/
/**Labels for use with the SHARP GP2Y0A02YK IR sensors**/
/*******************************************************/

// Analog pins reference for IR sensors
const int LEFT_IR_F = 3;  // Left front IR sensor is Analog In pin A3
const int LEFT_IR_R = 4;  // Left rear IR sensor is Analog In pin A4 
const int RIGHT_IR_F = 5;  // Right front IR sensor is Analog In pin A5 
const int RIGHT_IR_R = 6;  // Right rear IR sensor is Analog In pin A6 
const int CENTER_IR_L = 7;  // Center left IR sensor is Analog In pin A7  
const int CENTER_IR_R = 8;  // Center right IR sensor is Analog In pin A8 

// Set baudrate of IR sensors to 9600
#define IR_BAUDRATE 9600

//Define variables to hold analog input values from IR sensors
int Left_IR_F_VAL = 0;  // Value from left front IR sensor
int Left_IR_R_VAL = 0;  // Value from left rear IR sensor
int Right_IR_F_VAL = 0;  // Value from right front IR sensor
int Right_IR_R_VAL = 0;  // Value from right rear IR sensor
int Center_IR_L_VAL = 0;  // Value from center left IR sensor
int Center_IR_R_VAL = 0;  // Value from center right IR sensor

/**************************************/
/**Labels for use with the pushbutton**/
/**************************************/
const int buttonPin = 7;  // Digital pin 8 reference for pushbutton
const int ledPin =  13;      // the number of the LED pin
int mode = 0;  // Variable to store the current mode
int buttonState;

/**
 * 
 * INIT SECTION
 *
 * This section contains functions that
 * are used to initialize different components
 * of the walker
 * Most of the init functions are invoked in the function
 * void setup()
 * in ARDUINO SECTION
 *
 **/
// Initialize Joystick sensor
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

// Initialize IR sensors
void initIRSensors()
{
    // Initalize all ports communicate with IR sensors
    pinMode( LEFT_IR_F, INPUT );
    pinMode( LEFT_IR_R, INPUT );
    pinMode( RIGHT_IR_F, INPUT );
    pinMode( RIGHT_IR_R, INPUT );
    pinMode( CENTER_IR_L, INPUT );
    pinMode( CENTER_IR_R, INPUT );
    
    Serial.begin(IR_BAUDRATE);
    
    // 0.002 second time delay for the IR sensors to init 
    delay(20);
}

// Initialize pushbutton
void initPushbutton()
{
    pinMode(buttonPin, INPUT);  // initialize the pushbutton pin as an input
    pinMode(ledPin, OUTPUT);
}

// Initialize Sabertooth motor controller 
void initSabertooth()
{
    // Init software UART to communicate with the Sabertooth 2x5
    pinMode(SABER_TX_PIN, OUTPUT);
    SaberSerial.begin(SABER_BAUDRATE); 
    
    // 0.02 second time delay for the Sabertooth to init
    delay(20);
    
    // Send full stop command
    setEngineSpeed(512.00,512.00);
}

/**
 *
 * VALUE GETTERS SECTION
 *
 * This section contains functions used to
 * fetch/get values from different components
 *
 **/
// Calculate the distance values from IR sensor
void getDistances()
{
    int vrf=0, vrr=0, vcr=0, vcl=0, vlf=0, vlr=0;
    int samples = 8;
    //  float factor = 0.0048828125;
    
    for(int i=0;i < samples;i++){
        vrf += analogRead(RIGHT_IR_F);
        vlr += analogRead(LEFT_IR_R);
        vcr += analogRead(CENTER_IR_R);
        vrr += analogRead(RIGHT_IR_R);
        vlf += analogRead(LEFT_IR_F);
        vcl += analogRead(CENTER_IR_L);
    }
    
    Right_IR_R_VAL  = vrr >> 3;
    Right_IR_F_VAL  = vrf >> 3;
    Left_IR_F_VAL   = vlf >> 3;
    Left_IR_R_VAL   = vlr >> 3;
    Center_IR_R_VAL = vcr >> 3;
    Center_IR_L_VAL = vcl >> 3;
}

#define IRmin  85
#define IRmax  625
//Get values from all IR sensors
void getValuesFromIRSenors()
{
    getDistances();
    
    if(Left_IR_F_VAL < IRmin || Left_IR_F_VAL > IRmax) {
        Left_IR_F_VAL = 0;
    }
    
    if(Left_IR_R_VAL < IRmin || Left_IR_R_VAL > IRmax) {
        Left_IR_R_VAL = 0;
    }
    
    if(Center_IR_L_VAL < IRmin || Center_IR_L_VAL > IRmax) {
        Center_IR_L_VAL = 0;
    }
    
    if(Center_IR_R_VAL < IRmin || Center_IR_R_VAL > IRmax) {
        Center_IR_R_VAL = 0;
    }
    
    if(Right_IR_F_VAL < IRmin || Right_IR_F_VAL > IRmax) {
        Right_IR_F_VAL = 0;
    }
    
    if(Right_IR_R_VAL < IRmin || Right_IR_R_VAL > IRmax) {
        Right_IR_R_VAL = 0;
    }
    
}

// Get all values from Joystick sensor
void getValuesFromJoystick()
{
    JoyStick_X_VAL = analogRead(JOYSTICK_X);  // Get horizontal value (X-AXIS) from Joystick sensor
    delay(10);
    JoyStick_Y_VAL = analogRead(JOYSTICK_Y);  // Get vertical value (Y-AXIS) from Joystick sensor
    
    // To keep the the Joystick stable when it is powered without moving.
    if(JoyStick_X_VAL < 534 && JoyStick_X_VAL > 490 ){
        JoyStick_X_VAL = 512.00;
    }
    
    if(JoyStick_Y_VAL < 562 && JoyStick_Y_VAL > 462)
        JoyStick_Y_VAL = 512.00;  
    
}

void getMode()
{
    
    buttonState = digitalRead(buttonPin);  // get the state of button from the digital pin
    if (mode == 0){
        //The button is pushed
        if (buttonState == HIGH) {   
            mode = 1;  // The mode changed to 1 for the walker
            digitalWrite(13, HIGH);
            delay(300);
        }      
    } else if (mode == 1) {
        // The button is pushed
        if (buttonState == HIGH) {
            mode = 0;  // The mode changed to 0 for the walker 
            digitalWrite(13, LOW);
            delay(300);
        } 
    }
    
}


/**
 *
 * LOGIC & CONTROL SECTION
 *
 * Functions defined in this section are mainly used
 * in function
 * void loop()
 * in ARDUINO SECTION
 *
 * This section contains functions that are used to
 * represent logic and do real controls
 * Since logic and control are connected firmly
 * I put these two together, the following code are
 * further divided to LOGIC SECTION and CONTROL SECTION
 * However, this further division might be inaccurate
 *
 **/

// LOGIC SECTION

#define FarMax       100
#define FarMin       180
#define CloseMin     250
#define CloseMax     560
#define CenterClose  200

// Determine if the left side is too close to a target
boolean leftTooClose()
{
    if( (Left_IR_F_VAL > CloseMin) && 
       (Left_IR_R_VAL > CloseMin) )  // The range is between 10cm to 30cm
        return true;
    else
        return false;
}

// Determine if the left side is too far away from a target
boolean leftTooFar()
{
    if((Left_IR_F_VAL > FarMax && Left_IR_F_VAL < FarMin) && 
       (Left_IR_R_VAL > FarMax && Left_IR_R_VAL < FarMin))  // The range is bewteen 50cm to 80cm
        return true;
    else
        return false;
}

// Determine if the right side is too close to a target
boolean rightTooClose()
{
    if( ( Right_IR_F_VAL > CloseMin ) || 
       ( Right_IR_R_VAL > CloseMin ) )  // The range is between 10cm to 30cm
        return true;
    else
        return false;
}

// Determine if the right side is too far away from a target
boolean rightTooFar()
{
    if((Right_IR_F_VAL > FarMax && Right_IR_F_VAL < FarMin) || 
       (Right_IR_R_VAL > FarMax && Right_IR_R_VAL < FarMin))  // The range is between 50cm to 80cm
        return true;
    else
        return false;
}

// Determine if the front side is too close to a target
boolean centerTooClose()
{
    
    if((Center_IR_R_VAL > CenterClose) || 
       (Center_IR_L_VAL > CenterClose))  // The range is distance between 10cm and 50cm
        return true;
    else
        return false;
}

// determine if the walker should keep stoping  
boolean stopIt()
{
    if(JoyStick_X_VAL == 512.00 && JoyStick_Y_VAL == 512.00)
        return true;
    else
        return false;
}

// CONTROL SECTION

// Nothing happened, it will run contronlled by joystick
void run()
{
    setEngineSpeed(JoyStick_X_VAL, JoyStick_Y_VAL); // Run with the value of joystick
}

void setEngineSpeed(float XValue, float YValue)
{
    float xdiff, ydiff;
    int fwd = 1;
    // If emergency 
    if(XValue == 512.00 && YValue == 512.00) {
        // Motor1 and motor2 will be stop
        speedMotorRight = SABER_MTRRIGHT_FULL_STOP;  
        speedMotorLeft = SABER_MTRLEFT_FULL_STOP;
        adjustSpeed();
        return;
    }
    
    xdiff = XValue - Center_X_VAL;
    ydiff = YValue - Center_Y_VAL;
    speedMotorRight = SABER_MOTOR_RANGE;
    speedMotorLeft = SABER_MOTOR_RANGE;
    
    if(ydiff > Y3) {
        speedMotorRight *= YP4;
        speedMotorLeft *= YP4;
    }
    else if(ydiff > Y2) {
        speedMotorRight *= YP3;
        speedMotorLeft *= YP3;
    }
    else if(ydiff > Y1) {
        speedMotorRight *= YP2;
        speedMotorLeft *= YP2;
    }
    else if (ydiff > 0){
        speedMotorRight *= YP1;
        speedMotorLeft *= YP1;
    }
    else if(ydiff >= -Y2){
        speedMotorRight = 0;
        speedMotorLeft = 0;
    }
    /*
     else if(ydiff > -Y1) {
     speedMotorRight *= -YP1;
     speedMotorLeft *= -YP1;
     }
     */  
    else if(ydiff < -Y2) {
        speedMotorRight *= -YR;
        speedMotorLeft *= -YR;
    }
    /*
     else if(ydiff > -Y3) {
     speedMotorRight *= -YP3;
     speedMotorLeft *= -YP3;
     }
     else {
     speedMotorRight *= -YP4;
     speedMotorLeft *= -YP4;
     }
     */
    // Turn Left, Slow Left Motor
    if(xdiff > 0){
        if(xdiff > X3){
            speedMotorLeft *= XP1;
        }
        else if(xdiff > X2){
            speedMotorLeft *= XP2;
        }
        else if(xdiff > X1){
            speedMotorLeft *= XP3;
        }
        else{
            speedMotorLeft *= XP4;
        }
    }
    // Turn Right, Slow Right Motor
    else{
        if(xdiff > -X1){
            speedMotorRight *= XP4;
        }
        else if(xdiff > -X2){
            speedMotorRight *= XP3;
        }
        else if(xdiff > -X3){
            speedMotorRight *= XP2;
        }
        else{
            speedMotorRight *= XP1;
        } 
    }
    
    speedMotorRight += SABER_MTRRIGHT_FULL_STOP;
    speedMotorLeft += SABER_MTRLEFT_FULL_STOP;
    
    adjustSpeed();
}

// This method will help to adjust speed of motors linearly in 3 seconds.
void adjustSpeed()
{
    if( speedMotorRight == 512.0 && speedMotorLeft == 512.0 ){
        smoothMotorRight = SABER_MTRRIGHT_FULL_STOP;
        smoothMotorLeft = SABER_MTRLEFT_FULL_STOP;
    }
    else{
        smoothMotorRight += ((speedMotorRight - smoothMotorRight)/MotorAlpha);
        smoothMotorLeft += ((speedMotorLeft - smoothMotorLeft)/MotorAlpha);
    } 
    
    SaberSerial.print( smoothMotorRight , BYTE );  // Set speed of motor1
    delay(10);
    SaberSerial.print( smoothMotorLeft , BYTE );  // Set speed of motor1
    
}

// Anti the walker running automatically
void antiAutoRun()
{
    setEngineSpeed(512.00, 512.00);
}

// The front side is too close to a target
void antiCenterClose()
{
    if(JoyStick_Y_VAL < Center_Y_VAL)
        run();
    else
        setEngineSpeed(512.00, 512.00);
}

void turnLeft()
{
    setEngineSpeed(Center_X_VAL + X1, JoyStick_Y_VAL);
}

void turnRight()
{
    setEngineSpeed(Center_X_VAL - X1, JoyStick_Y_VAL);
}

// The left side is too close to a target
void antiLeftClose()
{ 
    /*  // Rear sensor located on left side is closer than the front one  
     if(Left_IR_R_VAL > Left_IR_F_VAL) {
     run();  // Keep running till the left side is too far away from a target
     
     // Left sensors are equidistant from target
     // Front sensor located on left side is closer than the rear one  
     } else {
     turnRight();
     }
     */
    turnRight();
}

// The left side is too far away from a target
void antiLeftFar()
{
    /*
     // Rear sensor located on left side is more far than the front one
     if(Left_IR_R_VAL < Left_IR_F_VAL) {
     run();  // Keep running till the left side is too close to a target
     
     // Right sensors are equidistant from target
     // Values of two sensor are the same 
     } else {
     turnLeft();
     }
     */
    turnLeft();
}

// The right side is too close to a target
void antiRightClose()
{   /*
     // Rear sensor located on right side is closer than the front one
     if(Right_IR_R_VAL > Right_IR_F_VAL) {
     run();  // Keep running till the right side is too far away from a target
     
     // Front sensor located on right side is closer than the rear one  
     // Values of two sensor are the same  
     } else {
     turnLeft();
     }*/
    turnLeft();
}

// The right side is too far away from a target
void antiRightFar()
{/*
  // Rear sensor located on right side is more far than the front one
  if(Right_IR_R_VAL < Right_IR_F_VAL) {
  run();  // Keep running till the right side is too close to a target
  
  // Front sensor located on right side is more far than the rear one  
  // Values of two sensor are the same  
  } else {
  turnRight();
  }*/
    turnRight();
}

/**
 *
 * OUTPUT SECTION
 *
 * This section is confusing even to me
 * But I do spot two functions that
 * are used to print sth to somewhere
 * The purpose of these two functions might be
 * providing feedbacks to user
 *
 **/

void p(char *fmt, ... ){
    char tmp[128]; // resulting string limited to 128 chars
    va_list args;
    va_start (args, fmt );
    vsnprintf(tmp, 128, fmt, args);
    va_end (args);
    Serial.print(tmp);
}

void PrintStatus(char* st)
{
    p("\t");
    p(st);
    /*
     p("Lft Fr Rr: ");
     Serial.print(Left_IR_F_VAL);
     p("\t");
     Serial.print(Left_IR_R_VAL);
     p("\n");
     */
    
    /*
     Serial.print(Right_IR_F_VAL);
     p("\t");
     Serial.print(Right_IR_R_VAL);
     p("\n");
     Serial.print(Center_IR_L_VAL);
     p("\t");
     Serial.print(Center_IR_R_VAL);
     */
    p("\n");
    /*
     Serial.print(JoyStick_X_VAL);
     p("\t");
     Serial.print(JoyStick_Y_VAL);
     p("\t");
     Serial.print( smoothMotorLeft , BYTE );  // Set speed of motor1
     p("\t");
     Serial.print( smoothMotorRight , BYTE );  // Set speed of motor1
     p("\n");
     
     Serial.print(speedMotorRight, BYTE);
     p("\t");
     Serial.print(speedMotorLeft, BYTE);
     p("\n");
     */
}


/**
 * 
 * ARDUINO SECTION
 *
 * This section contains two functions that
 * required by arduino to initialize and run
 *
 **/
void setup()
{
    initJoystick();  // Initialize Joystick sensor
    initIRSensors();  // Initialize IR sensors
    initSabertooth();  // Initialize Sabertooth motor controller
    initPushbutton();  // Initialize the pushbutton
    pinMode(13, OUTPUT);   
}

void loop() 
{
    getValuesFromIRSenors();  // get values from all IR sensors
    getValuesFromJoystick();  // get the values from Joystick
    getMode();  // get current mode from pushbutton
    delay(100); 
    
    // When the walker work as mode 2
    if(mode==1) {
        // It will fix the problem that the walker run automatically.
        if(stopIt()) {
            antiAutoRun();
            PrintStatus("Assist Stop");
            
            // When the center side is too close to a target
        } else if(centerTooClose()) {
            antiCenterClose();
            PrintStatus("Center Too Close");
            
            // When the left side is too close to a target 
        } else if(leftTooClose()) {
            antiLeftClose();
            PrintStatus("Left Too Close");
            
            // When the right side is too close to a target   
        } else if(rightTooClose()) {
            antiRightClose();
            PrintStatus("Right Too Close");
            
            // When the left side is too far away from a target  
        } else if(leftTooFar()) {
            antiLeftFar();
            PrintStatus("Left Too Far");
            
            // When the right side is too far away from a target
        } else if(rightTooFar()) {
            antiRightFar();
            PrintStatus("Right Too Far");
            
            // When it is good that run in a good range from a target or when there's no target
        } else {
            run();
            PrintStatus("Assist Run");
        }
        // When the walker work as mode 1
    } else {
        run();
        PrintStatus("**Manual Mode**");
    }
    
}
