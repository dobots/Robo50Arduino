
#include "sensors.h"

// namespace bobby
// {
	SensorHandler::SensorHandler(Bobby *robot) :
			mHeadingValue(0.0), mHeadingAverage(-1L), mHeadingMax(-1), mHeadingMin(10000),
			mAgConnected(false),
			mLeftEncoderTicks(0), mRightEncoderTicks(0) {
		// init accelero / gyro arrays
		memset(&mAgValue, 0, sizeof(mAgValue));
		memset(&mAgHistory, 0, sizeof(mAgHistory));
		memset(&mAgAverage, 0, sizeof(mAgAverage));
		memset(&mAgMax, 0, sizeof(mAgMax));
		memset(&mAgMin, 0, sizeof(mAgMin));

		// init compass arrays
		memset(&mHeadingData, 0, sizeof(mHeadingData));
		memset(&mHeadingHistory, 0, sizeof(mHeadingHistory));

		mRobot = robot;	
	}

	void SensorHandler::setupSensors() {

		// connection to compass, uses pins 20 and 21
		Wire.begin();

		//setup compass
		// Shift the device's documented slave address (0x42) 1 bit right
		// This compensates for how the TWI library only wants the
		// 7 most significant bits (with the high bit padded with 0)
		mCompassAddress = HMC6352Address >> 1;   // This results in 0x21 as the address to pass to TWI
		resetCompass();

		//setup accelero/gyro
		// initialize device
		// Serial.println("Initializing Accelero/Gyro MPU6050...");
		mAcceleroGyro.initialize();
		// verify connection
		// Serial.println("Testing connection...");
		mAgConnected = mAcceleroGyro.testConnection();
		// Serial.println(mAgConnected ? "MPU6050 connection successful" : "MPU6050 connection failed");
		resetAG();

		// //setup pulse counter callbacks
		// // Quadrature encoders
		// // Left encoder
		// pinMode(c_LeftEncoderPinA, INPUT);      // sets pin A as input
		// digitalWrite(c_LeftEncoderPinA, LOW);  // turn on pullup resistors
		// pinMode(LEFT_ENCODER, INPUT);      // sets pin B as input
		// digitalWrite(LEFT_ENCODER, LOW);  // turn on pullup resistors
		// attachInterrupt(LEFT_ENCODER_INTERRUPT, HandleLeftMotorInterruptA, RISING);
		// // Right encoder
		// pinMode(c_RightEncoderPinA, INPUT);      // sets pin A as input
		// digitalWrite(c_RightEncoderPinA, LOW);  // turn on pullup resistors
		// pinMode(RIGHT_ENCODER, INPUT);      // sets pin B as input
		// digitalWrite(RIGHT_ENCODER, LOW);  // turn on pullup resistors
		// attachInterrupt(RIGHT_ENCODER_INTERRUPT, HandleRightMotorInterruptA, RISING);

	}

	void SensorHandler::fillSensorData(aJsonObject *json) {
		aJsonObject *group, *sub, *item;

		// COMPASS
		group = aJson.createObject();
		aJson.addNumberToObject(group, "heading", formatCompassValue(mHeadingValue));
		aJson.addItemToObject(json, "compass", group);

		// ACCELERO
		group = aJson.createObject();
		aJson.addNumberToObject(group, "x", formatAcceleroValue(mAgValue[AX]));
		aJson.addNumberToObject(group, "y", formatAcceleroValue(mAgValue[AY]));
		aJson.addNumberToObject(group, "z", formatAcceleroValue(mAgValue[AZ]));
		aJson.addItemToObject(json, "accelero", group);

		// GYRO
		group = aJson.createObject();
		aJson.addNumberToObject(group, "x", formatGyroValue(mAgValue[GX]));
		aJson.addNumberToObject(group, "y", formatGyroValue(mAgValue[GY]));
		aJson.addNumberToObject(group, "z", formatGyroValue(mAgValue[GZ]));
		aJson.addItemToObject(json, "gyro", group);

		// ENCODER
		group = aJson.createObject();
		aJson.addNumberToObject(group, "rightEncoder", mRightEncoderTicks);
		aJson.addNumberToObject(group, "leftEncoder", mLeftEncoderTicks);
		aJson.addItemToObject(json, "odom", group);

	}

	void SensorHandler::readSensors() {
		readCompass();
		readAG();
	}

	void SensorHandler::readCompass() {
		// Send a "A" command to the HMC6352
		// This requests the current heading data
		Wire.beginTransmission(mCompassAddress);
		Wire.write("A");              // The "Get Data" command
		Wire.endTransmission();
		delay(2);                   // The HMC6352 needs at least a 70us (microsecond) delay after this command.
		// Read the 2 heading bytes, MSB first. The resulting 16bit word is the compass heading in 10th's of a degree
		// For example: a heading of 1345 would be 134.5 degrees
		Wire.requestFrom(mCompassAddress, 2);        // Request the 2 byte heading (MSB comes first)
		int i = 0;
		while(Wire.available() && i < 2)
		{ 
			mHeadingData[i] = Wire.read();
			i++;
		}
		mHeadingValue = mHeadingData[0]*256 + mHeadingData[1];  // Put the MSB and LSB together

		#ifdef DEBUG
			pushFront(mHeadingValue, mHeadingHistory, MAX_HEADING_HISTORY);

			//calculate new min / max values
			if (mHeadingValue > mHeadingMax) {
				mHeadingMax = mHeadingValue;
			}

			if (mHeadingValue < mHeadingMin) {
				mHeadingMin = mHeadingValue;
			}

			//calculate new rolling average
			long nHeadingAvg = 0L;
			int elements = 0;    
			for (int i = 0; i < MAX_HEADING_HISTORY; i++)
			{
				if (mHeadingHistory[i] == -1.0) continue;
				elements++;
				nHeadingAvg += mHeadingHistory[i];
			}

			mHeadingAverage = (long) nHeadingAvg / elements;
		#endif

	}

	void SensorHandler::readAG() {

		if (!mAgConnected) return;

		// read raw accel/gyro measurements from device
		mAcceleroGyro.getMotion6(&mAgValue[AX], &mAgValue[AY], &mAgValue[AZ], &mAgValue[GX], &mAgValue[GY], &mAgValue[GZ]);

		#ifdef DEBUG
			for (int j = 0; j < 6; ++j)
			{
				// mAgValue[j] = mAgValue[j] / (32767 / 2) * 9.81;

				pushFront(mAgValue[j], mAgHistory[j], MAX_AG_HISTORY);

				//calculate the new min / max values
				if (mAgValue[j] > mAgMax[j]) {
					mAgMax[j] = mAgValue[j];
				}

				if (mAgValue[j] < mAgMin[j]) {
					mAgMin[j] = mAgValue[j];
				}

				//calculate the new rolling average
				int elements = 0;
				long average = 0L;
				for (int i = 0; i < MAX_AG_HISTORY; i++)
				{
					if (mAgHistory[j][i] == -1.0) continue;
					elements++;
					average += mAgHistory[j][i];
				}
				mAgAverage[j] = (long) average /  elements;
			}
		#endif
	}

	//encoder interrupts

	// Interrupt service routines for the left motor's quadrature encoder
	void SensorHandler::handleLeftMotorInterrupt()
	{
		// Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
		mLeftEncoderBitSet = analogRead(LEFT_ENCODER) < 512 ? false : true;   // read the input pin

		// and adjust counter + if A leads B
		#ifdef LEFT_ENCODER_REVERSED
		  mLeftEncoderTicks -= mLeftEncoderBitSet ? -1 : +1;
		#else
		  mLeftEncoderTicks += mLeftEncoderBitSet ? -1 : +1;
		#endif
	}

	// Interrupt service routines for the right motor's quadrature encoder
	void SensorHandler::handleRightMotorInterrupt()
	{
		// Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
		mRightEncoderBitSet = analogRead(RIGHT_ENCODER) < 512 ? false : true;   // read the input pin  //digitalReadFast

		// and adjust counter + if A leads B
		#ifdef RIGHT_ENCODER_REVERSED
		  mRightEncoderTicks -= mRightEncoderBitSet ? -1 : +1;
		#else
		  mRightEncoderTicks += mRightEncoderBitSet ? -1 : +1;
		#endif
	}

	void SensorHandler::resetAG() {
		#ifdef DEBUG
			for (int i = 0; i < 6; ++i)
			{
				mAgValue[i] = 0;
				for (int j = 0; j < MAX_AG_HISTORY; ++j)
				{
					mAgHistory[i][j] = -1;
				}
				mAgAverage[i] = -1L;
				mAgMax[i] = -1000000;
				mAgMin[i] = 1000000;
			}
		#endif
	}

	void SensorHandler::resetCompass() {
		#ifdef DEBUG
			for (int i = 0; i < MAX_HEADING_HISTORY; i++) {
				mHeadingHistory[i] = -1;
			}

			mHeadingAverage = -1L;
			mHeadingMax = -1000000;
			mHeadingMin = 1000000;
		#endif
	}

	void SensorHandler::pushFront(int val, int* valueList, int len)
	{
		for (int i = len-2; i >= 0; i--) 
		{
		valueList[i+1] = valueList[i];
		}
		valueList[0] = val;
	}

	float SensorHandler::formatAcceleroValue(int value) {
		return float(value / 32767.0 * ACCELEROMETER_RANGE);
	}

	float SensorHandler::formatGyroValue(int value) {
		return float(value / 32767.0 * GYROSCOPE_RANGE);
	}

	float SensorHandler::formatCompassValue(int value) {
		return float(value / 10.0);
	}
// }