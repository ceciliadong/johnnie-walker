#include "everything"


void setup(); // arduino initialization routine

void loop(); // arduino running loop

struct pair {
	int x, y;
} // pair

enum sensorPosition {front, left, right, bottom}

//low-level functions

/**
 *
 * getSensorValues returns the current value of the queried
 * sensor pairs
 *
 **/
pair getSensorValues(const sensorPosition position);

/**
 *
 * getJoyStickValue returns a pair that describes the current
 * position of the joystick
 *
 **/
pair getJoyStickValue();

// motor speed setters
void setLeftMotorSpeed();
void setRightMotorSpeed();

// basic movements
void turnLeft();
void turnRight();
void moveForward();
void moveBackward();
void stop();
// advanced movements should be defined for
// smoother motions
// it is hard to think of any advanced movements for now










// logic part

// mode 0

// isBlack returns true if the given sensor value represents
// black area
bool isBlack(const int sensorValue);

// mode 0
voide mode0() {
	pair sensorValues = getSensorValues(sensorPosition.bottom);
	int leftSensorValue = sensorValues.x, rightSensorValue = sensorValues.y;
	// encode the condition
	short condition = (isBlack(leftSensorValue) << 1) + isBlack(rightSensorValue);
	switch (condition) {
		// both white
		case 0: moveForward(); break;
		// right is black
		case 1: turnRight(); break;
		// left is black
		case 2: turnLeft(); break;
		// both black
		case 3: stop(); break;
		// error
		default: stop();
	}
}

// mode 1 : joystick
void mode1() {
	pair joyStickValue = getJoyStickValue();	
	int leftMotorSpeed, rightMotorSpeed;
	// do some math here to cal the speeds
}

// mode 2 : surrounding sensors

// leftTooClose / rightTooClose / frontTooClose returns a number indicating
// the position of the walker
// 4 possible conditions:
// 0 for not close
// 1, 2 for not parallel conditions
// 3 for parallel (within a certain tolerance / error)
short getPositionLeft(pair sensorValues);
short getPositionRight(pair sensorValues);
short getPositionFront(pair sensorValues);

// theoretically, there are 4^3 = 64 possibilities to handle
// it is guaranteed that if all these possibilities are handled properly
// the movement of the walker should be very smooth
// practically (at least for now)
// only limited number of conditions should be handled
// because some conditions will not happen in normal conditions (paralleled walls are assumed)
// the numbers will further decrease since the project is used
// in a controlled environment
void mode2() {
	short leftCondition = getPositionLeft(getSensorValues(sensorPosition.left)); 
	short rightCondition = getPositionRight(getSensorValues(sensorPosition.right)); 
	short frontCondition = getPositionFront(getSensorValues(sensorPosition.front)); 
	// encode the condition
	short condition = (leftCondition << 4) + (rightCondition << 2) + frontCondition;
	switch (condition) {
		// close to nothing or parellel to walls
		case 0, 12, 48 : moveForward(); break;
		// fron facing wall
		case 1, 2, 3 : stop(); break;
		// front closer with wall on the right
		// back close with wall on the left
		// or a combanition of both
		case 4, 16, 20 : turnLeft(); break;
		// opposite 
		case 8, 32, 40 : turnRight(); break;
	}
}
