
#ifdef MINIMAL_PROTOCOL

#define FIELD_ID "i"
#define FIELD_LEFT "l"
#define FIELD_RIGHT "r"
#define FIELD_X "x"
#define FIELD_Y "y"
#define FIELD_Z "z"
#define FIELD_SPEED "s"

#define GROUP_BUMPER "b"
#define GROUP_ACCELERO "a"
#define GROUP_GYRO "g"
#define GROUP_ODOM "o"
#define GROUP_WHEELS "w"
#define GROUP_LEFT "l"
#define GROUP_RIGHT "r"
#define GROUP_DATA "d"

#else

#define FIELD_ID "id"
#define FIELD_LEFT "left"
#define FIELD_RIGHT "right"
#define FIELD_X "x"
#define FIELD_Y "y"
#define FIELD_Z "z"
#define FIELD_SPEED "speed"

#define GROUP_BUMPER "bumper"
#define GROUP_ACCELERO "accelero"
#define GROUP_GYRO "gyro"
#define GROUP_ODOM "odom"
#define GROUP_WHEELS "wheels"
#define GROUP_LEFT "left"
#define GROUP_RIGHT "right"
#define GROUP_DATA "data"

#endif