#include "log.h"

Stream* bt_serial = NULL;

void setBluetoothSerial(Stream* serial) {
	bt_serial = serial;
}

/**
 * A write function with a format specifier. 
 */
int bt_write(const char *str, ...) {
	if (bt_serial == NULL) return 0;
	
	char buffer[128];
	va_list ap;
	va_start(ap, str);
	int16_t len = vsprintf(NULL, str, ap);
	va_end(ap);

	if (len < 0) return len;

	// if strings are small we do not need to allocate by malloc
	if (sizeof buffer >= len + 1UL) {
		va_start(ap, str);
		len = vsprintf(buffer, str, ap);
		va_end(ap);
		for(int i = 0; i < len; ++i) {
			bt_serial->write(buffer[i]);
		}
	} else {
		char *p_buf = (char*)malloc(len + 1);
		if (!p_buf) return -1;
		va_start(ap, str);
		len = vsprintf(p_buf, str, ap);
		va_end(ap);
		for(int i = 0; i < len; ++i) {
			bt_serial->write(p_buf[i]);
		}
		free(p_buf);
	}
	return len;
}