#ifndef BIKE_BLUETOOTH_H
#define BIKE_BLUETOOTH_H

#include <Arduino.h>
#include <NimBLEDevice.h>
#include "BikeLog.h"

void initBluetooth();
bool isBluetoothConnected();
float bleEstimatedSpeedKmh();   // speed last sent to the watch, for the status line
uint16_t bleDisconnectCount();  // how many times the watch has disconnected since boot
int bleLastDisconnectReason();  // reason code of the most recent disconnect (0 = none yet)
void sendCyclingDataToWatch(float cadenceRPM, uint16_t resistanceLevel);

#endif
