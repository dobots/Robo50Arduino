/*

TODO 

*check on wheel encoder overflow
*reading motor commands to interrupt?
*fix additional motors..

*/

/*
pinout
0 and 1 are for serial to pc
2 is for accel interrupt
3 is free (PWM)
4 to 7, additional motors PWM
8 to 11, wheels motors PWM
12 and 13 additional motor PWM
14 and 15 free (serial channel)
16 and 17 are reserved for bluetooth (serial channel)
18 and 19 encoder interrupts
20 and 21 for I2C Wire library
22 to 47 free
48 to 51 addiional motors digital outs
52 and 52 wheel motors digital outs

A0 to A3 current sensing wheel motor drivers
A4 to A13 free
A14 and A15 wheel encoders second (quadrature) line

*/


#include "aJSON.h"
#include "MPU6050.h"

// messages
#define HEADER 0xA5

#define SENSOR_DATA 0
#define DRIVE_COMMAND 1
#define MOTOR_COMMAND 2
#define CONTROL_COMMAND 3
#define DISCONNECT 4
#define SENSOR_REQUEST 5

#define INT_T 0
#define DOUBLE_T 1
#define STRING_T 2
#define BOOL_T 3

// other constants

#define MAXINCIDENTCOUNT  100
#define OUTLIER_CHECKS 10
#define OUTLIER_THRESHOLD 5

//// motors

#define ELEVATOR 0
#define PUMP 1
#define VACUUM 2
#define BRUSH 3

#define PWM_A 4
#define PWM_B 5
#define PWM_C 6
#define PWM_D 7
int PWM_MOTORS[4] = {PWM_A, PWM_B, PWM_C, PWM_D};

#define CURRENT_SENSE_A A12
#define CURRENT_SENSE_B A13
#define CURRENT_SENSE_C A14
#define CURRENT_SENSE_D A15
int CURRENT_SENSE_MOTORS[4] = {CURRENT_SENSE_A, CURRENT_SENSE_B, CURRENT_SENSE_C, CURRENT_SENSE_D};

#define DIRECTION_A 45  // elevator
						// + direction -> UP
						// - direction -> DOWN
#define DIRECTION_B 44	// pump
						// + direction
						// - direction
#define DIRECTION_C 47	// vacuum
						// + direction -> blowing
						// - direction -> sucking -> OK
#define DIRECTION_D 46 	// brush 
						// - direction -> OK
						// + direction -> wrong
int DIRECTION_MOTORS[4] = {DIRECTION_A, DIRECTION_B, DIRECTION_C, DIRECTION_D};

#define CURRENT_LIMIT_A 1000
#define CURRENT_LIMIT_B 1000
#define CURRENT_LIMIT_C 1000
#define CURRENT_LIMIT_D 1000
int CURRENT_LIMIT_MOTORS[4] = {CURRENT_LIMIT_A, CURRENT_LIMIT_B, CURRENT_LIMIT_C, CURRENT_LIMIT_D};

bool MOTOR_INVERTED[4] = {false, true, true, true};

//// wheels

#define PWM_LEFT 8
#define PWM_RIGHT 9
#define DIRECTION_LEFT_FW 27
#define DIRECTION_LEFT_BW 29
#define DIRECTION_RIGHT_FW 25
#define DIRECTION_RIGHT_BW 23

#define CURRENT_SENSE_LEFT A0
#define CURRENT_SENSE_RIGHT A2

#define CURRENT_LIMIT_DRIVE 1000

//// flash light

#define FLASH_LIGHT 10

//// accelero / gyro
#define MAX_AG_HISTORY 5
#define AX 0
#define AY 1
#define AZ 2
#define GX 3
#define GY 4
#define GZ 5

#define ACCELEROMETER_RANGE 2.0 // +- 2g
#define GYROSCOPE_RANGE 250.0   // +- 250 deg/s

//// compass
#define MAX_HEADING_HISTORY 5

//// left encoder
#define c_LeftEncoderInterrupt 4 //(interrupt 4 is on pin 19)
#define c_LeftEncoderPinA 19
#define c_LeftEncoderPinB A14
#define LeftEncoderIsReversed
volatile bool _LeftEncoderBSet;

//// Right encoder
#define c_RightEncoderInterrupt 5   //(interrupt 5 is on pin 18)
#define c_RightEncoderPinA 18
#define c_RightEncoderPinB A15
volatile bool _RightEncoderBSet;

// bumper
// 31 and 33


void setup();
void loop();

void timerCallback();
void receiveCommands();
void handleControlCommand(aJsonObject* json);
void handleDisconnect(aJsonObject* json);
void handleMotorCommand(aJsonObject* json);
void handleDriveCommand(aJsonObject* json);
void handleSensorRequest(aJsonObject* json);
void readCompass();
void readAG();
void HandleLeftMotorInterruptA();
void HandleRightMotorInterruptA();
void sendData();
void drive(int leftSpeed, int rightSpeed);
void secdrive(int value,int motor);
int capSpeed(int value);
void PushFront(int val, int* valueList, int len);
float formatAcceleroValue(int value);
float formatGyroValue(int value);
float formatCompassValue(int value);
void resetAG();
void resetCompass();
int getType(aJsonObject* json);
void decodeMotorCommand(aJsonObject* json, int* motor_id, int* direction, int* speed);
void decodeDriveCommand(aJsonObject* json, int* left, int* right);
void flashLight(int speed);
void stop(int motor);
void senseMotor(int motor);
void senseLeftRight();
void print();
void setMotor(int id, int value);
void handleInput();
void timerCB();