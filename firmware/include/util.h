#ifndef UTIL_H
#define UTIL_H

void DEBUG(const char* message, ...) {
    va_list args;
    va_start(args, message);
    char buffer[SERIAL_BUFFER_SIZE];
    vsnprintf(buffer, SERIAL_BUFFER_SIZE, message, args);
    va_end(args);
    USBSerial.print("DEBUG: ");
    USBSerial.println(buffer);
}

#endif // UTIL_H