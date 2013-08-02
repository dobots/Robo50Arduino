/*

TODO

*check whether encoders/code are still worker
*check on wheel encoder overflow
*make wheel driver function proportional to allow for turns from standing still
   (instead of adding 1 to the current speed every loop)
*seperate motor incident parameters per motor
*make pump work! (write a seperate driver function for it? perhaps use timer?)
*get bumpers on interrupts (may require a uc with more interrupts ;-)

*/


/*********************************
*******      pinout      *********
**********************************

0 and 1 are for serial to pc
2 is for accel interrupt
3 is free (PWM / interrupt)
4 to 7, additional motors PWM
8 and 9, wheels motors PWM
10 flashlight
11 free (PWM)
12 and 13 free (PWM)
14 and 15 are reserved for bluetooth (serial channel)
16 and 17 free (serial channel)
18 and 19 encoder interrupts
20 and 21 for I2C Wire library
22 free
23 right wheel direction backward
24 free
25 right wheel direction forward
26 free
27 left wheel direction forward
28 free
29 left wheel direction backward
30 free
31 bumper sensor 1
32 free
33 bumper sensor 2
34 to 43 free
44 motor B direction (pump)
45 motor A direction (elevator)
46 motor D direction (brush)
47 motor C direction (vacuum)
48 to 53 free
A0 and A1 current sensing left wheel motor driver
A2 and A3 current sensing right wheel motor driver
A4 to A7 free
A8 and A9 wheel encoders second (quadrature) line
A10 and A11 free
A12 to A15 current sensing additional motors (A, B, C, D; elevator, pump, vacuum, brush respectively)

*****************************************/
#include "Arduino.h"
#include "Wire.h"
#include "aJSON.h"
#include "MPU6050.h"
#include "TimerOne.h"
#include "log.h"
//#include "digitalWriteFast.h"  //could be used to improve performance


// message constants
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

//// motors
#define ELEVATOR 0 //these numbers + 1 is how they are used in the cpp file
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

#define DIRECTION_A 45  // elevator, + direction -> UP, - direction -> DOWN
#define DIRECTION_B 44	// pump
#define DIRECTION_C 47	// vacuum, both directions ok
#define DIRECTION_D 46 	// brush, - direction -> OK, + direction -> wrong
int DIRECTION_MOTORS[4] = {DIRECTION_A, DIRECTION_B, DIRECTION_C, DIRECTION_D};
bool DIRECTION_INVERTED[4] = {false, false, true, true};

#define CURRENT_LIMIT_A 1000
#define CURRENT_LIMIT_B 1000
#define CURRENT_LIMIT_C 1000
#define CURRENT_LIMIT_D 1000
int CURRENT_LIMIT_MOTORS[4] = {CURRENT_LIMIT_A, CURRENT_LIMIT_B, CURRENT_LIMIT_C, CURRENT_LIMIT_D};

bool MOTOR_INVERTED[4] = {false, true, true, true};

//// wheel motors
#define PWM_LEFT 8
#define PWM_RIGHT 9
#define DIRECTION_LEFT_FW 27
#define DIRECTION_LEFT_BW 29
#define DIRECTION_RIGHT_FW 25
#define DIRECTION_RIGHT_BW 23
#define CURRENT_SENSE_LEFT A0
#define CURRENT_SENSE_RIGHT A2
#define CURRENT_LIMIT_DRIVE 1000
#define COMMAND_TIMEOUT 100000    ///TODO: set this to 500ms or so for use with ROS
#define INCIDENT_TIMEOUT 5000
#define MAX_INCIDENT_COUNT 50

//// flash light
#define FLASH_LIGHT 10

//// bumper
#define BUMPER1 31
#define BUMPER2 33

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

//// right encoder
#define c_RightEncoderInterrupt 4 //(interrupt 4 is on pin 19)
#define c_RightEncoderPinA 19
#define c_RightEncoderPinB A14
//#define RightEncoderIsReversed   //example, just to let you know this is checked in the cpp file
//volatile bool _RightEncoderBSet; //required for working quadrature encoder

//// left encoder
#define c_LeftEncoderInterrupt 5   //(interrupt 5 is on pin 18)
#define c_LeftEncoderPinA 18
#define c_LeftEncoderPinB A15
//volatile bool _LeftEncoderBSet;


////////////////////////////////
void setup();
void loop();

//communication
void receiveCommands();
void handleControlCommand(aJsonObject* json);
void handleDisconnect(aJsonObject* json);
void handleMotorCommand(aJsonObject* json);
void handleDriveCommand(aJsonObject* json);
void handleSensorRequest(aJsonObject* json);
void sendData();
void handleInput(int incoming);

//sensors
void senseMotor(int motor);
void senseLeftRight();
void readCompass();
void readAG();
void HandleLeftMotorInterruptA();
void HandleRightMotorInterruptA();

//actuators
void drive();
void secdrive(int motor);
int capSpeed(int value);
void flashLight(int speed);
void setMotor(int motor_id);
void stop(int motor);

//helper functions
void pushFront(int val, int* valueList, int len);
void pushIncident();
void pushMotorIncident(int motor_id);
float formatAcceleroValue(int value);
float formatGyroValue(int value);
float formatCompassValue(int value);
void resetSensors();
int getType(aJsonObject* json);
void decodeMotorCommand(aJsonObject* json, int* motor_id, int* speed);
void decodeDriveCommand(aJsonObject* json, int* left, int* right);

//TODO (pump)
//void reset();  //todo: pump functions
//void timerCB();
