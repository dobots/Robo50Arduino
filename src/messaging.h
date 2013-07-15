
#ifndef MESSAGING_H
#define MESSAGING_H

#include "aJSON.h"
#include "bobby.h"

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

class Bobby;
class SensorHandler;

// namespace bobby
// {
	class CommHandler {

	private:
		aJsonStream mSerialStream;

		Bobby *mRobot;
		SensorHandler *mSensorHandler;

		void handleControlCommand(aJsonObject* json);
		void handleDisconnect(aJsonObject* json);
		void handleMotorCommand(aJsonObject* json);
		void handleDriveCommand(aJsonObject* json);
		int getType(aJsonObject* json);
		void decodeMotorCommand(aJsonObject* json, int* motor_id, int* direction, int* speed);
		void decodeDriveCommand(aJsonObject* json, int* left, int* right);

	public:
		CommHandler(Bobby *robot, SensorHandler *sensorHandler, Stream *stream);

		void receiveCommands();
		void sendData();

	};
// }

#endif /* MESSAGING_H */