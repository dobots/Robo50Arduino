/*

TODO 
*fix pin assignments
*esp for wheel encoders
*fix function declarations
*check on wheel encoder overflow
*reading motor commands to interrupt?

*/

#include "aJSON.h"
#include "MPU6050.h"

//messages
#define HEADER 0xA5

#define SENSOR_DATA 0
#define DRIVE_COMMAND 1
#define MOTOR_COMMAND 2
#define CONTROL_COMMAND 3
#define DISCONNECT 4

#define INT_T 0
#define DOUBLE_T 1
#define STRING_T 2
#define BOOL_T 3

//constants
#define MAXINCIDENTCOUNT  100
#define ACCELEROMETER_RANGE 2.0 // +- 2g
#define GYROSCOPE_RANGE 250.0   // +- 250 deg/s

// Pin assignments
#define E1  8    // motor 1
#define E2  9
#define M1  52

#define S1  A0
#define S2  A1
  
#define E3  10   // motor 2
#define E4  11
#define M2  53
 
#define E7  4   // motor 2
#define E8  5
#define M4  49
 
#define E9  2   // motor 2
#define E10 3
#define M5  48

#define S3  A2
#define S4  A3

#define E5  6   // hulpmotor 1
#define M3  51
#define E6  7   // hulpmotor 2
#define M4  50

#define CURRENTLIMIT 1000
#define WAIT1        5000

//// accelero / gyro
#define MAX_AG_HISTORY 5
#define AX 0
#define AY 1
#define AZ 2
#define GX 3
#define GY 4
#define GZ 5

//// compass
#define MAX_HEADING_HISTORY 5

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;

//// wheel encoders
#define c_LeftEncoderInterrupt 4
#define c_LeftEncoderPinA 19
#define c_LeftEncoderPinB A4
#define LeftEncoderIsReversed
volatile bool _LeftEncoderBSet;

 
// Right encoder
#define c_RightEncoderInterrupt 5
#define c_RightEncoderPinA 18
#define c_RightEncoderPinB A5
volatile bool _RightEncoderBSet;



void setup();
void loop();

void receiveCommands();
void handleControlCommand(aJsonObject* json);
void handleDisconnect(aJsonObject* json);
void handleMotorCommand(aJsonObject* json);
void handleDriveCommand(aJsonObject* json);
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
