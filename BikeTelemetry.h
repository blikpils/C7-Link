#ifndef BIKE_TELEMETRY_H
#define BIKE_TELEMETRY_H

#include <Arduino.h>

struct ParsedPacket {
  bool isValid;
  unsigned int sequenceCounter;
  String header;
  String payload;
  String checksum;
};

struct BikeTelemetry {
  int resistance;
  long cadence;
  int counter;
};

// Function declarations
BikeTelemetry parseTelemetry(String payload);
ParsedPacket parsePacket(String rawData);
float calculateRPM(long rawCadenceTicks);

#endif
