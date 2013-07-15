
#include "bobby.h"

//#include "digitalWriteFast.h"  //no need as we use analogRead (unfortunately..)

// JSON message is of the format:
// {"compass":{"heading":119.00000},"accelero":{"x":0.04712,"y":0.00049,"z":0.97757},"gyro":{"x":-0.39674,"y":-1.95318,"z":-1.65563}}

int val = 0;
int id = 4;
int speed = 0;
int delay_time = 10;

int PWM_MOTORS[4] = {PWM_A, PWM_B, PWM_C, PWM_D};
int CURRENT_SENSE_MOTORS[4] = {CURRENT_SENSE_A, CURRENT_SENSE_B, CURRENT_SENSE_C, CURRENT_SENSE_D};
int DIRECTION_MOTORS[4] = {DIRECTION_A, DIRECTION_B, DIRECTION_C, DIRECTION_D};
int CURRENT_LIMIT_MOTORS[4] = {CURRENT_LIMIT_A, CURRENT_LIMIT_B, CURRENT_LIMIT_C, CURRENT_LIMIT_D};

Bobby *robot;


// --------------------------------------------------------------------
void setup() {
	robot = new Bobby();

	// //connect to computer, uses pin 0 and 1
	Serial.begin(115200);
	
	// /*//connect to bluetooth, uses pin 16 and 17
	// Serial2.begin(57600);
	// //  Serial2.flush();	
	// //  Serial2.setTimeout(10);*/
	
	// // createJson();

	//setup motors
	pinMode(PWM_A, OUTPUT);  
	pinMode(PWM_B, OUTPUT);  
	pinMode(PWM_C, OUTPUT);  
	pinMode(PWM_D, OUTPUT);  

	pinMode(DIRECTION_LEFT_FW, OUTPUT);
	pinMode(DIRECTION_LEFT_BW, OUTPUT);
	pinMode(DIRECTION_RIGHT_FW, OUTPUT);
	pinMode(DIRECTION_RIGHT_BW, OUTPUT);

	pinMode(DIRECTION_A, OUTPUT);
	pinMode(DIRECTION_B, OUTPUT);
	pinMode(DIRECTION_C, OUTPUT);
	pinMode(DIRECTION_D, OUTPUT);

	pinMode(PWM_LEFT, OUTPUT);
	pinMode(PWM_RIGHT, OUTPUT);

	pinMode(CURRENT_SENSE_LEFT, INPUT);
	pinMode(CURRENT_SENSE_RIGHT, INPUT);

	pinMode(CURRENT_SENSE_A, INPUT);
	pinMode(CURRENT_SENSE_B, INPUT);
	pinMode(CURRENT_SENSE_C, INPUT);
	pinMode(CURRENT_SENSE_D, INPUT);

	Serial.println("ready");
	Serial.print("motor: ");
	Serial.println(id);
	Serial.print("val: ");
	Serial.println(val);
	Serial.print("speed: ");
	Serial.println(speed);
	Serial.print("delay: ");
	Serial.println(delay_time);
}


void loop()
{
	robot->loop();
} 

// namespace bobby 
// {
	Bobby::Bobby() :
			mCurrentLeftSpeed(0), mCurrentRightSpeed(0), 
			mCurrentSenseLeft(0), mCurrentSenseRight(0),
			mPeakSenseLeft(0), mPeakSenseRight(0) {
		memset(&mCurrentSpeedMotor, 0, sizeof(mCurrentSpeedMotor));
		memset(&mPeakSenseMotor, 0, sizeof(mPeakSenseMotor));
		memset(&mCurrentSenseMotor, 0, sizeof(mCurrentSenseMotor));

		mSensorHandler = new SensorHandler(this);
		mCommHandler = new CommHandler(this, mSensorHandler, &Serial);

	}

	Bobby::~Bobby() {}

	void Bobby::loop() {

		mCommHandler->receiveCommands();

		mSensorHandler->readSensors();

		mCommHandler->sendData();

	}

	void Bobby::flashLight(int speed) {
		analogWrite(FLASH_LIGHT, speed);
	}

	void Bobby::estop(int motor_id) {

		if ((motor_id < 1) || (motor_id > 4))
			return;

		int index = motor_id -1;
		int pwmPin = PWM_MOTORS[index];

		mCurrentSpeedMotor[index] = 0;
		analogWrite(pwmPin, 0);   //PWM Speed Control
	}

	void Bobby::fillMotorData(aJsonObject *json) {
		aJsonObject *group, *sub, *item;

		// WHEELS
		group = aJson.createObject();

		// .. LEFT
		sub = aJson.createObject();
		item = aJson.createObject();
		aJson.addNumberToObject(item, "present", mCurrentSenseLeft);
		aJson.addNumberToObject(item, "peak", mPeakSenseLeft);
		aJson.addItemToObject(sub, "current", item);
		aJson.addNumberToObject(sub, "speed", mCurrentLeftSpeed);
		aJson.addItemToObject(group, "left", sub);

		// .. RIGHT
		sub = aJson.createObject();
		item = aJson.createObject();
		aJson.addNumberToObject(item, "present", mCurrentSenseRight);
		aJson.addNumberToObject(item, "peak", mPeakSenseRight);
		aJson.addItemToObject(sub, "current", item);
		aJson.addNumberToObject(sub, "speed", mCurrentRightSpeed);
		aJson.addItemToObject(group, "right", sub);
		
		aJson.addItemToObject(json, "wheels", group);

		// MOTORS
		group = aJson.createObject();

		// .. ELEVATOR
		sub = aJson.createObject();
		item = aJson.createObject();
		aJson.addNumberToObject(item, "present", mCurrentSenseMotor[ELEVATOR]);
		aJson.addNumberToObject(item, "peak", mPeakSenseMotor[ELEVATOR]);
		aJson.addItemToObject(sub, "current", item);
		aJson.addNumberToObject(sub, "speed", mCurrentSpeedMotor[ELEVATOR]);
		aJson.addItemToObject(group, "elevator", sub);

		// .. PUMP
		sub = aJson.createObject();
		item = aJson.createObject();
		aJson.addNumberToObject(item, "present", mCurrentSenseMotor[PUMP]);
		aJson.addNumberToObject(item, "peak", mPeakSenseMotor[PUMP]);
		aJson.addItemToObject(sub, "current", item);
		aJson.addNumberToObject(sub, "speed", mCurrentSpeedMotor[PUMP]);
		aJson.addItemToObject(group, "pump", sub);

		// .. VACUUM
		sub = aJson.createObject();
		item = aJson.createObject();
		aJson.addNumberToObject(item, "present", mCurrentSenseMotor[VACUUM]);
		aJson.addNumberToObject(item, "peak", mPeakSenseMotor[VACUUM]);
		aJson.addItemToObject(sub, "current", item);
		aJson.addNumberToObject(sub, "speed", mCurrentSpeedMotor[VACUUM]);
		aJson.addItemToObject(group, "vacuum", sub);

		// .. BRUSH
		sub = aJson.createObject();
		item = aJson.createObject();
		aJson.addNumberToObject(item, "present", mCurrentSenseMotor[BRUSH]);
		aJson.addNumberToObject(item, "peak", mPeakSenseMotor[BRUSH]);
		aJson.addItemToObject(sub, "current", item);
		aJson.addNumberToObject(sub, "speed", mCurrentSpeedMotor[BRUSH]);
		aJson.addItemToObject(group, "brush", sub);

		aJson.addItemToObject(json, "motors", group);	

	}

	// actuator functions
	void Bobby::drive(int leftSpeed, int rightSpeed)
	{
		int count = 0;
		int incidentcount = 0;

		// int mCurrentLeftSpeed = 0;
		// int mCurrentRightSpeed = 

		leftSpeed = capSpeed(leftSpeed);
		rightSpeed = capSpeed(rightSpeed);

		Serial.print("drive(");
		Serial.print(leftSpeed);
		Serial.print(",");
		Serial.print(rightSpeed);
		Serial.println(")");

		while ((incidentcount < MAXINCIDENTCOUNT) && ((mCurrentLeftSpeed != capSpeed(leftSpeed)) || (mCurrentRightSpeed != capSpeed(rightSpeed)))) {
			senseLeftRight();

			//Serial.print("mCurrentSenseLeft :");Serial.println(mCurrentSenseLeft);
			//Serial.print("mCurrentSenseRight :");Serial.println(mCurrentSenseRight);
			if ( (mCurrentSenseLeft < CURRENT_LIMIT_DRIVE) ) { // check for current
				if (leftSpeed < mCurrentLeftSpeed) {
				mCurrentLeftSpeed--;
				} else if (leftSpeed > mCurrentLeftSpeed) {
				mCurrentLeftSpeed++;
				}
			}
			else {
				if (leftSpeed < mCurrentLeftSpeed) {
				mCurrentLeftSpeed++;
				} else if (leftSpeed > mCurrentLeftSpeed) {
				mCurrentLeftSpeed--;
				}
				incidentcount++;
				if (incidentcount > MAXINCIDENTCOUNT) {
				mCurrentRightSpeed = 0; // reset speed
				mCurrentLeftSpeed = 0;
				}
			}

			if ( (mCurrentSenseRight < CURRENT_LIMIT_DRIVE) ) { // check for current
				if (rightSpeed < mCurrentRightSpeed) {
				mCurrentRightSpeed--;
				} else if (rightSpeed > mCurrentRightSpeed) {
				mCurrentRightSpeed++;
				}
			}
			else {
				if (rightSpeed < mCurrentRightSpeed) {
				mCurrentRightSpeed++;
				} else if (rightSpeed > mCurrentRightSpeed) {
				mCurrentRightSpeed--;
				}
				incidentcount++;
				if (incidentcount > MAXINCIDENTCOUNT) {
				mCurrentRightSpeed = 0; // reset speed
				mCurrentLeftSpeed = 0;
				}
			}
			// Serial.print("Count :");Serial.println(count);
			
			mCurrentLeftSpeed = capSpeed(mCurrentLeftSpeed);
			mCurrentRightSpeed = capSpeed(mCurrentRightSpeed);

			if (mCurrentLeftSpeed > 0) {
				digitalWrite(DIRECTION_LEFT_FW, HIGH);
				digitalWrite(DIRECTION_LEFT_BW, LOW);
			} else if (mCurrentLeftSpeed < 0) {
				digitalWrite(DIRECTION_LEFT_FW, LOW);
				digitalWrite(DIRECTION_LEFT_BW, HIGH);
			} else {
				digitalWrite(DIRECTION_LEFT_FW, LOW);
				digitalWrite(DIRECTION_LEFT_BW, LOW);
			}

			if (mCurrentRightSpeed > 0) {
				digitalWrite(DIRECTION_RIGHT_FW, HIGH);
				digitalWrite(DIRECTION_RIGHT_BW, LOW);
			} else if (mCurrentRightSpeed < 0) {
				digitalWrite(DIRECTION_RIGHT_FW, LOW);
				digitalWrite(DIRECTION_RIGHT_BW, HIGH);
			} else {
				digitalWrite(DIRECTION_RIGHT_FW, LOW);
				digitalWrite(DIRECTION_RIGHT_BW, LOW);
			}

			analogWrite(PWM_LEFT, abs(mCurrentLeftSpeed));   //PWM Speed Control
			analogWrite(PWM_RIGHT, abs(mCurrentRightSpeed));   //PWM Speed Control

			Serial.print("left: ");
			Serial.print(mCurrentLeftSpeed);
			Serial.print(", right: ");
			Serial.print(mCurrentRightSpeed);
			Serial.print(", senseLeft: ");
			Serial.print(mCurrentSenseLeft);
			Serial.print(", senseRight: ");
			Serial.println(mCurrentSenseRight);

			delay(delay_time);

		}

	}

	// void drive(int leftSpeed, int rightSpeed)
	// {
	//         int count = 0;
	//         int incidentcount = 0;

	//         int motor1pin;
	//         int motor2pin;

	// 	int left = capSpeed(leftSpeed);  //can we not take these out?
	// 	int right = capSpeed(rightSpeed);

	//         mCurrentSenseLeft = 0;
	//         mCurrentSenseRight = 0;
	//         mPeakSenseLeft = 0;
	//         mPeakSenseRight = 0;       

	//         digitalWrite(M1,HIGH);  
	//         digitalWrite(M2,HIGH);

	//     //Serial.println("in Drive");
	//       analogWrite(E1, 0);   //PWM Speed Control
	//       analogWrite(E2, 0);   //PWM Speed Control
	//       analogWrite(E3, 0);   //PWM Speed Control
	//       analogWrite(E4, 0);   //PWM Speed Control
	//     //Serial.println("speed set to zero");

	// 	//setDirection(&left, &right);
	//     if (leftSpeed > 0 && rightSpeed > 0)
	//     { // forward direction
	//       motor1pin = E1;
	//       motor2pin = E3;    
	//     } 
	//     else if (leftSpeed < 0 && rightSpeed < 0)
	//     {  // backward
	//       motor1pin = E2;
	//       motor2pin = E4;    
	//     }
	//     else if (leftSpeed < 0 && rightSpeed > 0)
	//     { // turning right
	//       motor1pin = E2;
	//       motor2pin = E3;    
	//     }
	//     else  // only option left: if (leftSpeed > 0 && rightSpeed < 0)
	//     { // turning left
	//       motor1pin = E1;
	//       motor2pin = E4;    
	//     }

	//     while ((incidentcount < MAXINCIDENTCOUNT) && (mCurrentLeftSpeed != abs(capSpeed(leftSpeed))) && (mCurrentRightSpeed != abs(capSpeed(rightSpeed)))) {
	//       mCurrentSenseLeft = analogRead(S1);
	//       mCurrentSenseRight = analogRead(S3);
	//       mPeakSenseLeft = (mCurrentSenseLeft > mPeakSenseLeft ? mCurrentSenseLeft : mPeakSenseLeft); // track maximum current sensed
	//       mPeakSenseRight = (mCurrentSenseRight > mPeakSenseRight ? mCurrentSenseRight : mPeakSenseRight);
	//       //Serial.print("mCurrentSenseLeft :");Serial.println(mCurrentSenseLeft);
	//       //Serial.print("mCurrentSenseRight :");Serial.println(mCurrentSenseRight);
	//       if ( (mCurrentSenseLeft < CURRENTLIMIT) &&  (mCurrentSenseRight < CURRENTLIMIT) ) { // check for current
	//         count++;  // if ok increase speed
	//       }
	//       else {
	//         count = count--; // if not ok decrease
	//         incidentcount++;
	//         if (incidentcount > MAXINCIDENTCOUNT) {
	//           count = 0; // reset speed
	//         }
	//       }
	//       // Serial.print("Count :");Serial.println(count);

	//       if (count > abs(capSpeed(leftSpeed)))
	//         left = abs(capSpeed(leftSpeed));
	//       else
	//         left = count;

	//       if (count > abs(capSpeed(rightSpeed)))
	//         right = abs(capSpeed(rightSpeed));
	//       else
	//         right = count;

	//       analogWrite(motor1pin, left);   //PWM Speed Control
	//       analogWrite(motor2pin, right);   //PWM Speed Control

	//       mCurrentLeftSpeed = left;
	//       mCurrentRightSpeed = right;
	//       //Serial.print("mCurrentLeftSpeed :");Serial.println(mCurrentLeftSpeed);
	//       //Serial.print("mCurrentRightSpeed :");Serial.println(mCurrentRightSpeed);
	//       delay(10);
	//     }

	//   //Serial.println(left);
	//   //Serial.println(right);
	//   //Serial.println(mPeakSenseLeft);
	//   //Serial.println(mPeakSenseRight);

	// }

	// //driver function for one motor
	// void secdrive(int value,int motor)
	// {
	//   Serial.print("secrive(");
	//   Serial.print(motor);
	//   Serial.print(",");
	//   Serial.print(value);
	//   Serial.println(")");

	//     int count = 0;
	//     int incidentcount = 0;

	//     int motorPin;
	//     int motorPin1;
	//     int motorPin2;
	//     int enablePin;

	//     int currentSensePin;
	//     int pwmPin;
	//     int directionPin;

	//     int* curSpeed;
	//     int* sense;
	//     int* report;

	//     if (motor == 1) {
	//       pwmPin          = PWM_A;
	//       directionPin    = DIRECTION_A;
	//       currentSensePin = CURRENT_SENSE_A;

	//       curSpeed        = &mCurrentSpeedMotor1;
	//       sense           = &mCurrentSenseMotor1;
	//       report          = &mPeakSenseMotor1;
	//     } else if (motor == 2) {
	//       pwmPin          = PWM_B;
	//       directionPin    = DIRECTION_B;
	//       currentSensePin = CURRENT_SENSE_B;

	//       curSpeed        = &mCurrentSpeedMotor2;
	//       sense           = &mCurrentSenseMotor2;
	//       report          = &mPeakSenseMotor2;
	//     } else if (motor == 3) {
	//       pwmPin          = PWM_C;
	//       directionPin    = DIRECTION_C;
	//       currentSensePin = CURRENT_SENSE_C;

	//       curSpeed        = &mCurrentSpeedMotor3;
	//       sense           = &mCurrentSenseMotor3;
	//       report          = &mPeakSenseMotor3;
	//     } else if (motor == 4) {
	//       pwmPin          = PWM_D;
	//       directionPin    = DIRECTION_D;
	//       currentSensePin = CURRENT_SENSE_D;

	//       curSpeed        = &mCurrentSpeedMotor4;
	//       sense           = &mCurrentSenseMotor4;
	//       report          = &mPeakSenseMotor4;
	//     } else {
	//       return;
	//     }

	//     while ((incidentcount < MAXINCIDENTCOUNT) && (*curSpeed != capSpeed(value))) {
	//       // *sense = analogRead(currentSensePin);
	//       // *report = (*sense > *report ? *sense : *report);

	//       // //Serial.print("mCurrentSenseLeft :");Serial.println(mCurrentSenseLeft);
	//       // //Serial.print("mCurrentSenseRight :");Serial.println(mCurrentSenseRight);
	//       // if ( (*sense < CURRENTLIMIT) ) { // check for current
	//         if (value < *curSpeed) {
	//           (*curSpeed)--;
	//         } else {
	//           (*curSpeed)++;
	//         }
	//       // }
	//       // else {
	//       //   if (value < *curSpeed) {
	//       //     (*curSpeed)++;
	//       //   } else {
	//       //     (*curSpeed)--;
	//       //   }
	//       //   incidentcount++;
	//       //   if (incidentcount > MAXINCIDENTCOUNT) {
	//       //     count = 0; // reset speed
	//       //   }
	//       // }
	//       // Serial.print("Count :");Serial.println(count);

	//       *curSpeed = capSpeed(*curSpeed);

	//       // if (*curSpeed > 0) {
	//       //   digitalWrite(directionPin, HIGH);
	//       // } else {
	//       //   digitalWrite(directionPin, LOW);
	//       // }

	//       analogWrite(pwmPin, *curSpeed);   //PWM Speed Control

	//       Serial.print("pin: ");
	//       Serial.print(pwmPin);
	//       Serial.print(", curSpeed: ");
	//       Serial.print(*curSpeed);
	//       Serial.print(", sense: ");
	//       Serial.print(*sense);
	//       Serial.print(", report: ");
	//       Serial.println(*report);

	//       delay(10);
	//     }

	//     Serial.println("done");

	// }

	void Bobby::senseLeftRight() {
		mCurrentSenseLeft = analogRead(CURRENT_SENSE_LEFT);
		mCurrentSenseRight = analogRead(CURRENT_SENSE_RIGHT);
		mPeakSenseLeft = max(mPeakSenseLeft, mCurrentSenseLeft);
		mPeakSenseRight = max(mPeakSenseRight, mCurrentSenseRight);
	}

	void Bobby::senseMotor(int motor) {
		mCurrentSenseMotor[motor] = analogRead(CURRENT_SENSE_MOTORS[motor]);
		mPeakSenseMotor[motor] = max(mPeakSenseMotor[motor], mCurrentSenseMotor[motor]);
	}

	//driver function for one motor
	void Bobby::secdrive_pt(int value,int motor)
	{
		Serial.print("secdrive(");
		Serial.print(motor);
		Serial.print(",");
		Serial.print(value);
		Serial.println(")");

		int count = 0;
		int incidentcount = 0;

		int motorPin;
		int motorPin1;
		int motorPin2;
		int enablePin;

		int currentSensePin;
		int pwmPin;
		int directionPin;

		int *curSpeed = NULL;
		int *sense = NULL;
		int *report = NULL;

		// if (motor == 1) {
		// pwmPin          = PWM_A;
		// directionPin    = DIRECTION_A;
		// currentSensePin = CURRENT_SENSE_A;

		// // curSpeed        = &mCurrentSpeedMotor1;
		// // sense           = &mCurrentSenseMotor1;
		// // report          = &mPeakSenseMotor1;
		// } else if (motor == 2) {
		// pwmPin          = PWM_B;
		// directionPin    = DIRECTION_B;
		// currentSensePin = CURRENT_SENSE_B;

		// // curSpeed        = &mCurrentSpeedMotor2;
		// // sense           = &mCurrentSenseMotor2;
		// // report          = &mPeakSenseMotor2;
		// } else if (motor == 3) {
		// pwmPin          = PWM_C;
		// directionPin    = DIRECTION_C;
		// currentSensePin = CURRENT_SENSE_C;

		// // curSpeed        = &mCurrentSpeedMotor3;
		// // sense           = &mCurrentSenseMotor3;
		// // report          = &mPeakSenseMotor3;
		// } else if (motor == 4) {
		// pwmPin          = PWM_D;
		// directionPin    = DIRECTION_D;
		// currentSensePin = CURRENT_SENSE_D;

		// // curSpeed        = &mCurrentSpeedMotor4;
		// // sense           = &mCurrentSenseMotor4;
		// // report          = &mPeakSenseMotor4;
		// } else {
		// return;
		// }

		if ((motor < 1) || (motor > 4))
			return;

		int index = motor -1;

		pwmPin = PWM_MOTORS[index];
		directionPin = DIRECTION_MOTORS[index];
		currentSensePin = CURRENT_SENSE_MOTORS[index];

		curSpeed = &(mCurrentSpeedMotor[index]);
		sense = &(mCurrentSenseMotor[index]);
		report = &(mPeakSenseMotor[index]);

		while ((incidentcount < MAXINCIDENTCOUNT) && (*curSpeed != capSpeed(value))) {
			// *sense = analogRead(currentSensePin);
			// *report = (*sense > *report ? *sense : *report);
			senseMotor(index);

			// //Serial.print("mCurrentSenseLeft :");Serial.println(mCurrentSenseLeft);
			// //Serial.print("mCurrentSenseRight :");Serial.println(mCurrentSenseRight);
			if ( (*sense < CURRENT_LIMIT_MOTORS[index]) ) { // check for current
				if (value < *curSpeed) {
				(*curSpeed)--;
				} else {
				(*curSpeed)++;
				}
			}
			else {
				if (value < *curSpeed) {
				(*curSpeed)++;
				} else {
				(*curSpeed)--;
				}
				incidentcount++;
				if (incidentcount > MAXINCIDENTCOUNT) {
				*curSpeed = 0; // reset speed
				analogWrite(pwmPin, abs(*curSpeed));
				return;
				}
			}
			// Serial.print("Count :");Serial.println(count);
			
			*curSpeed = capSpeed(*curSpeed);

			if (*curSpeed > 0) {
				digitalWrite(directionPin, HIGH);
			} else {
				digitalWrite(directionPin, LOW);
			}

			analogWrite(pwmPin, abs(*curSpeed));   //PWM Speed Control

			Serial.print("pin: ");
			Serial.print(pwmPin);
			Serial.print(", curSpeed: ");
			Serial.print(*curSpeed);
			Serial.print(", sense: ");
			Serial.print(*sense);
			Serial.print(", report: ");
			Serial.println(*report);

			delay(delay_time);
		}

		Serial.println("done");

	}

	//driver function for one motor
	void Bobby::motor(int motor_id, int speed)
	{
		Serial.print("secrive(");
		Serial.print(motor_id);
		Serial.print(",");
		Serial.print(speed);
		Serial.println(")");

		int count = 0;
		int incidentcount = 0;

		int motorPin;
		int motorPin1;
		int motorPin2;
		int enablePin;

		int currentSensePin;
		int pwmPin;
		int directionPin;

		int curSpeed = 0;
		int sense = 0;
		int report = 0;

		if ((motor_id < 1) || (motor_id > 4))
			return;

		int index = motor_id -1;

		pwmPin          = PWM_MOTORS[index];
		directionPin    = DIRECTION_MOTORS[index];
		currentSensePin = CURRENT_SENSE_MOTORS[index];

		curSpeed        = mCurrentSpeedMotor[index];
		// sense           = mCurrentSenseMotor[index];
		report          = mPeakSenseMotor[index];

		while ((incidentcount < MAXINCIDENTCOUNT) && (curSpeed != capSpeed(speed))) {
			senseMotor(index);

			// //Serial.print("mCurrentSenseLeft :");Serial.println(mCurrentSenseLeft);
			// //Serial.print("mCurrentSenseRight :");Serial.println(mCurrentSenseRight);
			if ( (mCurrentSenseMotor[index] < CURRENT_LIMIT_MOTORS[index]) ) { // check for current
				if (speed < curSpeed) {
					curSpeed--;
				} else {
					curSpeed++;
				}
			}
			else {
				if (speed < curSpeed) {
					curSpeed++;
				} else {
					curSpeed--;
				}
				incidentcount++;
				if (incidentcount > MAXINCIDENTCOUNT) {
					curSpeed = 0; // reset speed
					analogWrite(pwmPin, abs(curSpeed));
					return;
				}
			}
			// Serial.print("Count :");Serial.println(count);
			
			curSpeed = capSpeed(curSpeed);

			if (curSpeed > 0) {
				digitalWrite(directionPin, HIGH);
			} else {
				digitalWrite(directionPin, LOW);
			}

			analogWrite(pwmPin, abs(curSpeed));   //PWM Speed Control

			Serial.print("pin: ");
			Serial.print(pwmPin);
			Serial.print(", curSpeed: ");
			Serial.print(curSpeed);
			Serial.print(", sense: ");
			Serial.print(mCurrentSenseMotor[index]);
			Serial.print(", report: ");
			Serial.println(report);

			delay(delay_time);
		}

		// if (motor == 1) {
		//   // mCurrentSpeedMotor1 = curSpeed;
		//   mCurrentSenseMotor1 = sense;
		//   mPeakSenseMotor1 = report;
		// } else if (motor == 2) {
		//   // mCurrentSpeedMotor2 = curSpeed;
		//   mCurrentSenseMotor2 = sense;
		//   mPeakSenseMotor2 = report;
		// } else if (motor == 3) {
		//   // mCurrentSpeedMotor3 = curSpeed;
		//   mCurrentSenseMotor3 = sense;
		//   mPeakSenseMotor3 = report;
		// } else if (motor == 4) {
		//   // mCurrentSpeedMotor4 = curSpeed;
		//   mCurrentSenseMotor4 = sense;
		//   mPeakSenseMotor4 = report;
		// } else {
		//   return;
		// }

		mCurrentSpeedMotor[index]      = curSpeed;
		// mCurrentSenseMotor[index]  = sense;
		mPeakSenseMotor[index]      = report;

		Serial.println("done");

	}

	// //driver function for one motor
	// void secdrive(int value,int motor)
	// {
	//     int count = 0;

	//     int motorPin;
	//     int motorPin1;
	//     int motorPin2;
	//     int enablePin;

	//     int curSpeed = 0;

	//     if (motor == 1) {
	//       motorPin1 = E5;
	//       motorPin2 = E6;
	//       enablePin = M3;
	//       curSpeed = mCurrentSpeedMotor1;
	//     } else if (motor == 2) {
	//       motorPin1 = E7;
	//       motorPin2 = E8;
	//       enablePin = M4;
	//       curSpeed = mCurrentSpeedMotor2;
	//     } else if (motor == 3) {
	//       motorPin1 = E9;
	//       motorPin2 = E10;
	//       enablePin = M5;
	//       curSpeed = mCurrentSpeedMotor3;
	//     }

	//     digitalWrite(enablePin,HIGH); // enable selected motor 

	//     //Serial.println("in Drive");

	//     //analogWrite(motorPin1, 0);   //PWM Speed Control set to zero
	//     //analogWrite(motorPin2, 0);   //PWM Speed Control

	//     //Serial.println("speed set to zero");

	//   //setDirection(&left, &right);
	//     if (curSpeed > 0)
	//     { // forward direction
	//       motorPin = motorPin1;
	//     } 
	//     else
	//     {  // backward
	//       motorPin = motorPin2;
	//     }

	//     while ((curSpeed != value)) {

	//       if (curSpeed < value)
	//         curSpeed++;  //  increase speed
	//       else
	//         curSpeed--;  //  decrease spedd
	//         // Serial.print("Count :");Serial.println(count);
	//       if (curSpeed == 0) {
	//         if (motorPin == motorPin1)
	//           motorPin = motorPin2;
	//         else
	//           motorPin = motorPin1;
	//       }

	//       // curSpeed = count;

	//       analogWrite(motorPin, curSpeed);   //PWM Speed Control
	//       delay(5);
	//     }

	//     // report motor speed to global
	//     if (motor == 1) {
	//       mCurrentSpeedMotor1 = curSpeed;
	//     } else if (motor == 2) {
	//       mCurrentSpeedMotor2 = curSpeed;
	//     } else if (motor == 3) {
	//       mCurrentSpeedMotor3 = curSpeed;
	//     }
	// }

	int Bobby::capSpeed(int value)
	{
	// return max(min(value,249),-249);
		return max(min(value,255),-255);   
	}


	///helper functions

	void Bobby::pushFront(int val, int* valueList, int len)
	{
		for (int i = len-2; i >= 0; i--) 
		{
			valueList[i+1] = valueList[i];
		}
		valueList[0] = val;
	}
// }