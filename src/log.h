#ifndef _H_LOG
#define _H_LOG

#include "Arduino.h"
#include "debug.h"

extern int log_level;

#if defined(DEBUG) && defined(DEBUG_BT)
	#define LOGi(loglevel, fmt, ...) printf(fmt "\r\n", ##__VA_ARGS__); bt_write(fmt "\r\n", ##__VA_ARGS__)
	#define LOGd(loglevel, fmt, ...) printf(fmt "\r\n", ##__VA_ARGS__); bt_write(fmt "\r\n", ##__VA_ARGS__)
#else
#ifdef DEBUG
	#define LOGi(loglevel, fmt, ...) printf(fmt "\r\n", ##__VA_ARGS__)
	#define LOGd(loglevel, fmt, ...) printf(fmt "\r\n", ##__VA_ARGS__)
#else
#ifdef DEBUG_BT
	#define LOGi(loglevel, fmt, ...) bt_write(fmt "\r\n", ##__VA_ARGS__)
	#define LOGd(loglevel, fmt, ...) bt_write(fmt "\r\n", ##__VA_ARGS__)
#else
	#define LOGi(loglevel, fmt, ...)
	#define LOGd(loglevel, fmt, ...)
#endif
#endif
#endif


extern Stream* bt_serial;

void setBluetoothSerial(Stream* serial);

/**
 * write a string with printf functionality.
 */
int bt_write(const char *str, ...);

#endif
