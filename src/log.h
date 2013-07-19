#ifndef _H_LOG
#define _H_LOG

#include "Arduino.h"

extern bool debug_on;
extern int log_level;

void initLogging(Stream *stream);
void LOGi(const int loglevel, const char* fmt, ... );
void LOGd(const int loglevel, const char* fmt, ... );

#endif
