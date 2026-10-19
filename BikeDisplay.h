#pragma once
#include <stdint.h>

// Starts the OLED and a background task that redraws it.
// If the display isn't found, this prints a message and carries on without it
// (the bike / Bluetooth side keeps working).
void initDisplay();

// Hand the latest values to the display. Returns immediately: it only stores
// the numbers, the background task does the (slow) drawing a few times a second.
void setDisplayValues(uint16_t resistance, uint16_t cadenceTicks, float rpm);

// Connection info shown under the bike readings: whether the watch is connected,
// how long since the last good bike frame (shown only once it counts as lost),
// the speed being sent, how many times the watch has disconnected (and the last
// reason code), and how many bad frames were dropped. Also returns immediately.
void setDisplayConnection(bool watchUp, uint32_t bikeAgeMs, float speedKmh,
                          uint16_t drops, int lastDisconnectReason, uint32_t rejected);
