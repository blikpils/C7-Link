#pragma once
#include <Arduino.h>
#include <stdarg.h>

// Tiny logger so the serial output is easy to scan.
//
//   [03:25] STAT   OK   rpm=62.4 ...      <- normal line, once a second
//   [03:40] WATCH  connected              <- normal event
//   !! [04:12] BIKE   no data for 1.5 s   <- something went wrong (starts with "!!")
//
// Search the monitor for "!!" to jump straight to the failures.

inline void logLine(const char* prefix, const char* tag, const char* fmt, va_list args) {
    char msg[200];
    vsnprintf(msg, sizeof(msg), fmt, args);
    unsigned long s = millis() / 1000UL;
    Serial.printf("%s[%02lu:%02lu] %-6s %s\n", prefix, s / 60, s % 60, tag, msg);
}

inline void logInfo(const char* tag, const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    logLine("", tag, fmt, args);
    va_end(args);
}

inline void logAlert(const char* tag, const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    logLine("!! ", tag, fmt, args);
    va_end(args);
}
