#include <SoftwareSerial.h>
#include <stdarg.h>
/**
 *
 * Date   :  2/25/2012
 * Author :  NYIT Johnnie Walker Group
 *
 * Newer version of Johnnie Walker code
 * This version still got problems and issues
 * that can be find in the previous versions
 * However, the main purpose of this version
 * is to start to integrate more sophisticated
 * algorithms such as artificial intelligence
 *
 **/
// Digital pin 18 is the serial transmit pin to the Sabertooth 2x5
#define MOTOR_TX_PIN 18
// Digital pin 19 is the serial receive pin to the Sabertooth 2x5
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
int MOTORRIGHT_FORWARD_SPEED[] = {64,  71,  78,  85,  92,  99, 106, 113, 120, 127};
int MOTORLEFT_REVERSE_SPEED[] =  {191, 184, 177, 170, 163, 156, 149, 142, 135, 128}; // from slow to fast
int MOTORRIGHT_REVERSE_SPEED[] = {63,  57,  50,  43,  36,  29,  22,  15,   8,   1}; // from slow to fast
// for speed tracking purpose
int currentSpeedLeft = 0, currentSpeedRight = 0;
// pins of side sensors
const int SIDE_SENSOR_TOTAL = 4;
int sideSensorPins[4] = {3, 4, 5, 6}; // {leftFront, leftRear, rightFront, rightRear}
int sideSensorValues[4] = {0, 0, 0, 0};
// pins of front facing ultrasonic sensors
const int LEFT_ULTRASONIC_PIN = 31, RIGHT_ULTRASONIC_PIN = 33;
int frontLeft = 0, frontRight = 0;
// pins of optical sensor arrays
const int OPTICAL_SENSOR_TOTAL = 6;
const int OPTICAL_THRESH = 700;
int opticalSensorPins[6] = {9, 10, 11, 12, 13, 14};
int opticalSensorValues[6] = {0, 0, 0, 0, 0, 0};
int opticalBaseline[6] = {600, 600, 600, 600, 600, 600};
// definitions related to joysticks
#define JOYSTICK_BAUDRATE 9600  // Set baudrate of Joystick sensors to 9600
const int JOYSTICK_X = 2;       // Analog input pin A0 that the X-axis of joystick is attached to
const int JOYSTICK_Y = 1;      // Analog input pin A1 that the Y-axis of joystick is attached to
int joystickX = 0;   // Horizontal value from Joystick sensor
int joystickY = 0;  // Vertical value from Joystick sensor
// definitionss related to mode change buttons
const int BUTTONPIN = 7;  // Digital pin 7 reference for pushbutton
int mode = 0;  // Variable to store the current mode
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
  // initialize ultrasonic sensors just for safety
  pinMode(LEFT_ULTRASONIC_PIN, OUTPUT);
  pinMode(RIGHT_ULTRASONIC_PIN, OUTPUT);
  // initialize side sensors (IR sensors)
  for(int i = 0; i < SIDE_SENSOR_TOTAL; i++) 
    pinMode(sideSensorPins[i], INPUT);
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
void getUltrasonicValues() {
  const int L = LEFT_ULTRASONIC_PIN, R = RIGHT_ULTRASONIC_PIN;
  pinMode(L, OUTPUT); 
  pinMode(R, OUTPUT);
  digitalWrite(L, LOW); 
  digitalWrite(R, LOW);
  delay(5);
  digitalWrite(L, HIGH); 
  digitalWrite(R, HIGH);
  delay(5);
  digitalWrite(L, LOW); 
  digitalWrite(R, LOW);
  pinMode(L, INPUT), pinMode(R, INPUT);
  frontLeft = pulseIn(L, HIGH) / 58;
  frontRight = pulseIn(R, HIGH) / 58;
}

void getSideSensorValues() {
  for(int i = 0; i < SIDE_SENSOR_TOTAL; i++) 
    sideSensorValues[i] = analogRead(sideSensorPins[i]);
  delay(5);
}

// threshold between black and white should be 600
void getOpticalSensorValues() {
  for(int i = 0; i < OPTICAL_SENSOR_TOTAL; i++) {
    opticalSensorValues[i] = analogRead(opticalSensorPins[i]);
    Serial.print(opticalSensorValues[i]);
    Serial.print(" ");
  }
  Serial.println();
  delay(5);
}

int normalizeCoordinate(int value) {
  //return value;
  if(value > 400 && value < 600) return 50;
  // Transforms the joystick's messurement into a values between 0 and 100 
  return (value / 10); 
}

void getJoyStickValue() {
  joystickX = normalizeCoordinate(analogRead(JOYSTICK_X));
  delay(10);
  joystickY = normalizeCoordinate(analogRead(JOYSTICK_Y));
}

/**
 * motor control and motions
 **/
void setMotorSpeed(int left, int right) {	
  // decide the exact speed value
  int leftSpeed = left >= 0 ? MOTORLEFT_FORWARD_SPEED[left] 
    : MOTORLEFT_REVERSE_SPEED[-left];
  int rightSpeed = right >= 0 ? MOTORRIGHT_FORWARD_SPEED[right] 
    : MOTORRIGHT_REVERSE_SPEED[-right];
  // send value to arduino
  // with Arduino 1.0 this function needs to be changed to write
  MotorSerial.print(leftSpeed, BYTE);
  delay(10); // Serial communication with the motors
  MotorSerial.print(rightSpeed, BYTE);
  // save current stat
  currentSpeedLeft = left;
  currentSpeedRight = right; 
}

void turnLeft() { 
  setMotorSpeed(4, 5); 
}
void turnRight() { 
  setMotorSpeed(5, 4); 
}
void moveBackward() { 
  setMotorSpeed(-5, -5); 
}
void moveForward() { 
  setMotorSpeed(5, 5); 
}
void motorStop() { 
  setMotorSpeed(0, 0); 
}
void moveCW() { 
  setMotorSpeed(5, -5); 
} // move clockwise
void moveCC() { 
  setMotorSpeed(-5, 5); 
} // move counter clockwise

/**
 * logic part
 **/
double uniform = (double)1/(double)4;
double probability[4];
double pl, pr; // for wall following
int baseline; // value holder
int lowBoundary = 200, highBoundary = 700;
// establish a baseline/threshold to 
// tell between extremely close and medium close
void establishBaseline() {
  // get side sensor values first
  getSideSensorValues();
  // calculate baseline for both sides
  int lb = (sideSensorValues[0] + sideSensorValues[1]) / 2;
  int rb = (sideSensorValues[2] + sideSensorValues[1]) / 2;
  baseline = lb > rb ? lb : rb; // get the bigger one as baseline value
  // if base line is too big or too small
  // set it to be a reasonable value
  if(baseline < lowBoundary || baseline > highBoundary)
    baseline = (lowBoundary + highBoundary) / 2;
}

// initialize logic
void initLogic() {
  establishBaseline();
  establishOpticalBaseline();
  // set probability to be uniform
  for(int i = 0; i < 4; i++) probability[i] = uniform;
  pl = pr = 0.5;
}

// coefficient used to update probabilities
double coefficient(int sensorValue) {
  return (double)sensorValue/(double)100;
}

// normalize probabilities so they adds up to 1
void normalizeProbability() {
  double sum = 0;
  for(int i = 0; i < 4; i++) sum += probability[i];
  for(int i = 0; i < 4; i++) probability[i] /= sum;
}

// sense and calculate probabilities
void sense() {
  getSideSensorValues();
  double sum = 0;
  for(int i = 0; i < 4; i++) 
    probability[i] *= coefficient(sideSensorValues[i]);
  normalizeProbability();
}

// update probabilities(predict) and 
// move the real motor!
void move() {
  // find the index of the one with max probabilities
  double pMax = 0;
  int maxIndex = 0;
  for(int i = 0; i < 4; i++) if(probability[i] > pMax) {
    pMax = probability[i];
    maxIndex = i;
  }
  // control the motors
  // if it's leftfront or rightrear
  if(maxIndex == 0 || maxIndex == 3) turnRight();
  else if(maxIndex == 1 || maxIndex == 2) turnLeft();
  else motorStop();
  // predict the probabilities after the motion
}

// AI wall folloing function
void followWall() {
  sense();
  move();
}

int maximum(int a, int b) {
  return a > b ? a : b;
}

int sumation(int a, int b) {
  return a+b;
}

void establishOpticalBaseline() {
  for(int i = 0; i < 20; i++) {
    getOpticalSensorValues();
    for(int i = 0; i < OPTICAL_SENSOR_TOTAL; i++) {
      opticalBaseline[i] = (opticalBaseline[i] + opticalSensorValues[i]) / 2;
      Serial.print(opticalBaseline[i]);
      Serial.print(" ");
    }
    Serial.println();
    delay(250);
  }
  delay(5000);
}

void calibrate() {
  getOpticalSensorValues();
  for(int i = 0; i < OPTICAL_SENSOR_TOTAL; i++) {
    int diff = opticalBaseline[i] - opticalSensorValues[i];
    if(diff > -50 && diff < 50) {
      opticalBaseline[i] = (opticalBaseline[i]+opticalSensorValues[i])/2;
      opticalSensorValues[i] = 0;
    }
  }
}

// following tape function
// with some quickly put-together AI stuff
void followTape() {  
  // sense
  getOpticalSensorValues();
  calibrate();
  int sleft = 500 + opticalSensorValues[0]*3 + opticalSensorValues[1]*2 + opticalSensorValues[2];
  int sright = 500 + opticalSensorValues[5]*3 + opticalSensorValues[4]*2 + opticalSensorValues[3];
  pl *= sleft;
  pr *= sright;
  // normalize
  double sum = pl + pr;
  pl /= sum;
  pr /= sum;
  Serial.println(pl);
  Serial.println(pr);
  Serial.println();
  // move
  int leftSpeed = 6 * pr;
  int rightSpeed = 6 * pl;
  setMotorSpeed(leftSpeed, rightSpeed);
  // predict probability
  pl *= (0.8*(double)leftSpeed + 0.2*(double)rightSpeed);
  pr *= (0.8*(double)rightSpeed + 0.2*(double)leftSpeed);
}

// syntax sugar
void modeZero() {
  followTape();
}

// syntax sugar
void modeOne() {
  followWall();
}

// button!
void changeMode() {
  buttonState = digitalRead(BUTTONPIN);  // get the state of button from the digital pin
  if (mode == 0) {
    //The button is pushed
    if (buttonState == HIGH) {   
      mode = 1;  // The mode changed to 1 for the walker
      delay(300);
      establishBaseline();
    }      
  } 
  else if (mode == 1) {
    // joystick interuption
    getJoyStickValue();
    int x = joystickX;
    int y = joystickY;
    if (x != 50 || y != 50 || buttonState == HIGH) {
      mode = 0;  // The mode changed to 0 for the walker 
      delay(300);
    } 
  }
}

// joystick!
// joystick control with boolean return value
void joystickControl() {
  getJoyStickValue();
  // bottom right is (0, 0)
  int x = joystickX;
  int y = joystickY;
  int yDir = y >= 50 ? 1 : -1;
  int xDir = x > 50 ? 1 : -1;
  int basev = (y - 50) / 10; // base speed
  int turnf = xDir * (x - 50) / 15; // turn factor
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
  initLogic();
}

void loop() {
  modeZero();
}




