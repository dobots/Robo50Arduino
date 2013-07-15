#ifndef SENSORS_H
#define SENSORS_H

#include "aJSON.h"
#include "MPU6050.h"

#include "bobby.h"

#define HMC6352Address 0x42

//// compass
#define MAX_HEADING_HISTORY 5

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

//// wheel encoders, these occupy serial 1 due to the interrupts
#define LEFT_ENCODER_INTERRUPT 4 //(interrupt 4 is on pin 19)
// #define c_LeftEncoderPinA 4 // what are they used for?
#define LEFT_ENCODER A6
#define LEFT_ENCODER_REVERSED
 
// Right encoder
#define RIGHT_ENCODER_INTERRUPT 5   //(interrupt 5 is on pin 18)
// #define c_RightEncoderPinA 5 // what are they used for?
#define RIGHT_ENCODER A7
#undef RIGHT_ENCODER_REVERSED

#undef DEBUG


class Bobby;

// namespace bobby
// {
	class SensorHandler {

	private:
		Bobby *mRobot;

		//// accelero / gyro
		MPU6050 mAcceleroGyro;

		bool mAgConnected;
		int mAgValue[6];
		int mAgHistory[6][MAX_AG_HISTORY];
		long mAgAverage[6];
		int mAgMax[6];
		int mAgMin[6];

		//// compass

		// class default I2C address is 0x68
		// specific I2C addresses may be passed as a parameter here
		// AD0 low = 0x68 (default for InvenSense evaluation board)
		// AD0 high = 0x69

		// This is calculated in the setup() function
		int mCompassAddress;

		byte mHeadingData[2];
		int mHeadingValue;
		int mHeadingHistory[MAX_HEADING_HISTORY];
		long mHeadingAverage;
		int mHeadingMax;
		int mHeadingMin;

		// why volatile ??
		volatile int mLeftEncoderTicks;
		volatile int mRightEncoderTicks;
		volatile bool mLeftEncoderBitSet;
		volatile bool mRightEncoderBitSet;

		void readCompass();
		void readAG();
		void handleLeftMotorInterrupt();
		void handleRightMotorInterrupt();
		void resetAG();
		void resetCompass();
		float formatAcceleroValue(int value);
		float formatGyroValue(int value);
		float formatCompassValue(int value);
		void pushFront(int val, int* valueList, int len);

	public:
		SensorHandler(Bobby *robot);

		void setupSensors();
		void fillSensorData(aJsonObject *json);
		void readSensors();

	};
// }

#endif /* SENSORS_H */