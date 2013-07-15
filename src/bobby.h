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

#ifndef BOBBY_H
#define BOBBY_H

#include "Arduino.h"
#include "aJSON.h"

#include "sensors.h"
#include "messaging.h"

#define MAXINCIDENTCOUNT  100
#define OUTLIER_CHECKS 10
#define OUTLIER_THRESHOLD 5

// MOTOR id definition

#define ELEVATOR 0
#define PUMP 1
#define VACUUM 2
#define BRUSH 3

//// Pin assignments

#define PWM_A 4
#define PWM_B 5
#define PWM_C 6
#define PWM_D 7
extern int PWM_MOTORS[4];

#define CURRENT_SENSE_A A12
#define CURRENT_SENSE_B A13
#define CURRENT_SENSE_C A14
#define CURRENT_SENSE_D A15
extern int CURRENT_SENSE_MOTORS[4];

#define DIRECTION_A 45  // elevator
						// - direction -> UP
						// + direction -> DOWN
#define DIRECTION_B 44	// pump
						// + direction
						// - direction
#define DIRECTION_C 47	// vacuum
						// + direction -> blowing
						// - direction -> sucking -> OK
#define DIRECTION_D 46 	// brush 
						// - direction -> OK
						// + direction -> wrong
extern int DIRECTION_MOTORS[4];

#define PWM_LEFT 8
#define PWM_RIGHT 9
#define DIRECTION_LEFT_FW 27
#define DIRECTION_LEFT_BW 29
#define DIRECTION_RIGHT_FW 25
#define DIRECTION_RIGHT_BW 23

#define CURRENT_SENSE_LEFT A0
#define CURRENT_SENSE_RIGHT A2

#define FLASH_LIGHT 10


//// current limit definitions

#define CURRENT_LIMIT_A 1000
#define CURRENT_LIMIT_B 1000
#define CURRENT_LIMIT_C 1000
#define CURRENT_LIMIT_D 1000
extern int CURRENT_LIMIT_MOTORS[4];

#define CURRENT_LIMIT_DRIVE 1000



extern int val;
extern int id;
extern int speed;
extern int delay_time;

void setup();
void loop();

class CommHandler;
class SensorHandler;

// namespace bobby
// {
	class Bobby {

	private:
		CommHandler *mCommHandler;
		SensorHandler *mSensorHandler;

		int mCurrentSpeedMotor[4];
		int mPeakSenseMotor[4];
		int mCurrentSenseMotor[4];

		int mCurrentLeftSpeed;
		int mCurrentRightSpeed;
		int mCurrentSenseLeft;
		int mCurrentSenseRight;
		int mPeakSenseLeft;
		int mPeakSenseRight;

		int capSpeed(int value);
		void pushFront(int val, int* valueList, int len);

		void senseLeftRight();
		void senseMotor(int motor);
		void secdrive_pt(int value,int motor);

	public:
		Bobby();
		~Bobby();

		void drive(int leftSpeed, int rightSpeed);
		void motor(int motor_id, int speed);
		void estop(int motor_id);
		void flashLight(int speed);

		void fillMotorData(aJsonObject *json);

		void loop();
	};
// }


#endif /* BOBBY_H */