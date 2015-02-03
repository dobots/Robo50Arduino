#define SHORT_MESSAGE
//#define LONG_MESSAGE

// message constants
#define HEADER 0xA5

// message type enum
#define SENSOR_DATA 0
#define DRIVE_COMMAND 1
#define MOTOR_COMMAND 2
#define CONTROL_COMMAND 3
#define DISCONNECT 4
#define SENSOR_REQUEST 5

// parameter type enum
#define INT_T 0
#define DOUBLE_T 1
#define STRING_T 2
#define BOOL_T 3

#ifdef SHORT_MESSAGE
	#define FIELD_ID		"i"
	#define FIELD_DATA		"d"
	#define FIELD_ACCELERO	"a"
	#define FIELD_GYRO		"g"
	#define FIELD_ODOM		"o"
	#define FIELD_BUMPER	"b"
	#define FIELD_WHEELS	"w"
	#define FIELD_LEFT		"l"
	#define FIELD_RIGHT		"r"
	#define FIELD_SPEED		"s"
#elif LONG_MESSAGE
	#define FIELD_ID		"id"
	#define FIELD_DATA		"data"
	#define FIELD_ACCELERO	"accelero"
	#define FIELD_GYRO		"gyro"
	#define FIELD_ODOM		"odom"
	#define FIELD_BUMPER	"bumper"
	#define FIELD_WHEELS	"wheels"
	#define FIELD_LEFT		"left"
	#define FIELD_RIGHT		"right"
	#define FIELD_SPEED		"speed"
#endif