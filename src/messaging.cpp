
#include "messaging.h"
#include "bobby.h"

// namespace bobby 
// {
	CommHandler::CommHandler(Bobby *robot, SensorHandler *sensorHandler, Stream *stream) :
			mSerialStream(stream) {
		mRobot = robot;
		mSensorHandler = sensorHandler;
	}

	void print() {
		Serial.print("motor: ");
		Serial.println(id);
		Serial.print("val: ");
		Serial.println(val);
		Serial.print("speed: ");
		Serial.println(speed);
		Serial.print("delay: ");
		Serial.println(delay_time);
	}

	void CommHandler::receiveCommands()
	{
		// aJsonObject* item;
		// if (mSerialStream.available()) {
		// 	item = aJson.parse(&mSerialStream);
		//  } else {
		//    return;
		//  }

		// aJson.print(item, &mSerialStream);
		// Serial.println(" ");

		// switch(getType(item)) {
		// 	case DRIVE_COMMAND:
		// 		handleDriveCommand(item);
		// 		break;
		// 	case MOTOR_COMMAND:
		// 		handleMotorCommand(item);
		// 		break;
		// 	case CONTROL_COMMAND:
		// 		handleControlCommand(item);
		// 		break;
		// 	case DISCONNECT:
		// 		handleDisconnect(item);
		// 		break;
		// }
		// aJson.deleteItem(item);
		if (Serial.available()) {
			int incoming = Serial.read();
			switch(incoming) {
			case 'g':
				sendData();
				break;
			case '1':
				// for (int i = 0; i < 100; ++i) {
				//   secmRobot->drive(i, 4);
				//   delay(1000);
				// }
				// val = 130;1
				// secmRobot->drive(130, id);
				mRobot->drive(speed, speed);
				break;
			case '2':
				// val = 70;
				// secmRobot->drive(70, id);
				mRobot->drive(-speed, -speed);
				break;
			case '3':
				// val = 0;
				// secmRobot->drive(0, id);
				mRobot->drive(0, 0);
				break;
			case '4':
				mRobot->drive(-speed, speed); // right
				// val = 255;
				// secmRobot->drive(255, id);
				// digitalWrite(DIRECTION_D, HIGH);
				break;
			case 'q':
				mRobot->drive(speed, -speed);
				// val = 255;
				// secmRobot->drive(255, id);
				// digitalWrite(DIRECTION_D, HIGH);
				break;
			case 'f':
				Serial.println("flashing");
				mRobot->flashLight(speed);
				break;
			case '5':
				mRobot->estop(id);
				// secmRobot->drive(val, id);
				// digitalWrite(DIRECTION_D, LOW);
				break;
			case '6':
				mRobot->motor(id, val);
				break;
			case '7':
				mRobot->motor(id, 0);
				break;
			case '8':
				mRobot->motor(id, 200);
				break;
			case '9':
				mRobot->motor(id, 50);
				break;
			case '0':
				mRobot->motor(id, -50);
				break;
			case 'z':
				speed += 10;
				Serial.print("speed: ");
				Serial.println(speed);
				break;
			case 'x':
				speed -= 10;
				Serial.print("speed: ");
				Serial.println(speed);
				break;
			case 'c':
				delay_time += 10;
				Serial.print("delay: ");
				Serial.println(delay_time);
				break;
			case 'v':
				delay_time -= 10;
				Serial.print("delay: ");
				Serial.println(delay_time);
				break;   
			case 'u':
				val += 10;
				Serial.print("Val: ");
				Serial.println(val);
				break;
			case 'd':
				val -= 10;
				Serial.print("Val: ");
				Serial.println(val);
				break;
			case 'i':
				val += 1;
				Serial.print("Val: ");
				Serial.println(val);
				break;
			case 'k':
				val -= 1;
				Serial.print("Val: ");
				Serial.println(val);
				break;
			case 'w':
				id += 1;
				Serial.print("Motor: ");
				Serial.println(id);
				break;
			case 's':
				id -= 1;
				Serial.print("Motor: ");
				Serial.println(id);
				break;
			case 'p':
				print();
				break;
			default:
				Serial.print("incoming: ");
				Serial.print((char)incoming);
				Serial.print(" (");
				Serial.print(incoming);
				Serial.println(")");
				break;
			}
		}
	}

	void CommHandler::handleControlCommand(aJsonObject* json) {
	  // is not necessary for now
	}

	void CommHandler::handleDisconnect(aJsonObject* json) {
		// is sent when the phone disconnects from the robot
		// best to turn off all motors here
		mRobot->drive(0, 0);
	}

	void CommHandler::handleMotorCommand(aJsonObject* json) {
		//Serial.println("1");
		int id = -1, direction = -1, value = -1;
		decodeMotorCommand(json, &id, &direction, &value);
		mRobot->motor(id, value);
		//Serial.println("2");
		// TODO: add here the function call to mRobot->drive the brush motor
		//  id is in case we want to control other mothers, currently a value of 1 is sent
		//  direction, either 0 or 1, where 1 is "forward"
		//  speed, a value between 0 and 255
	}

	void CommHandler::handleDriveCommand(aJsonObject* json) {
		int leftSpeed = 0, rightSpeed = 0;
		decodeDriveCommand(json, &leftSpeed, &rightSpeed);
		mRobot->drive(leftSpeed, rightSpeed);
	}

	// JSON message is of the format:
	// {"compass":{"heading":119.00000},"accelero":{"x":0.04712,"y":0.00049,"z":0.97757},"gyro":{"x":-0.39674,"y":-1.95318,"z":-1.65563}}


	void CommHandler::sendData() {
		aJsonObject *json;

		json = aJson.createObject();

		mSensorHandler->fillSensorData(json);

		// mMotorHandler->fillMotorData(json);
		mRobot->fillMotorData(json);

		aJson.print(json, &mSerialStream);
		Serial.println();
		aJson.deleteItem(json);
	}

	int CommHandler::getType(aJsonObject* json) {
		aJsonObject* header;
		aJsonObject* type;
		header = aJson.getObjectItem(json, "header");
		type = aJson.getObjectItem(header, "type");
		return type->valueint;
	}

	void CommHandler::decodeMotorCommand(aJsonObject* json, int* motor_id, int* direction, int* speed) {
		aJsonObject* data = aJson.getObjectItem(json, "data");

		aJsonObject* motor_id_j = aJson.getObjectItem(data, "motor_id");
		*motor_id = motor_id_j->valueint;

		aJsonObject* direction_j = aJson.getObjectItem(data, "direction");
		*direction = direction_j->valueint;

		aJsonObject* speed_j = aJson.getObjectItem(data, "speed");
		*speed = speed_j->valueint;
	}

	void CommHandler::decodeDriveCommand(aJsonObject* json, int* left, int* right) {
		aJsonObject* data = aJson.getObjectItem(json, "data");

		aJsonObject* left_j = aJson.getObjectItem(data, "left");
		*left = left_j->valueint;

		aJsonObject* right_j = aJson.getObjectItem(data, "right");
		*right = right_j->valueint;
	}
// }