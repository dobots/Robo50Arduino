#include "log.h"

// --------------------------------------------------------------------
// GLOBAL VAR
// --------------------------------------------------------------------

bool debug_on = true;
int log_level = 3;

// --------------------------------------------------------------------
// LOCAL VAR
// --------------------------------------------------------------------

Stream *serialLine;

// --------------------------------------------------------------------
// FUNCTIONS
// --------------------------------------------------------------------

// --------------------------------------------------------------------
// create a output function
// This works because Serial.write, although of
// type virtual, already exists.
int uart_write (char c, FILE *stream)
{
    serialLine->write(c) ;
    return 0;
}

void initLogging(Stream *stream) {
  serialLine = stream;

  fdevopen( &uart_write, 0 );
}

void LOGi(const int loglevel, const char* fmt, ... )
{
  if (loglevel <= log_level) {
    va_list argptr;
    va_start(argptr, fmt);
    vprintf(fmt, argptr);
    va_end(argptr);
    serialLine->println("");
  }
}

void LOGd(const int loglevel, const char* fmt, ... )
{  
  if (debug_on && loglevel <= log_level) {
    va_list argptr;
    va_start(argptr, fmt);
    vprintf(fmt, argptr);
    va_end(argptr);
    serialLine->println("");
  }
}

