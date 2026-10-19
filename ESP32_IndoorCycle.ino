#include "BikeTelemetry.h"
#include "BikeBluetooth.h"
#include "BikeDisplay.h"     // OLED lives in BikeDisplay.cpp now


// Set to 1 to send fake data when a packet can't be parsed (handy for testing
// the BLE side without the bike). Leave at 0 for normal use.
#define DEMO_MODE 0

unsigned long lastValidPacketMs = 0;
uint16_t lastResistanceLevel = 0;
float lastRpmShown = 0.0f;               // last cadence sent to the watch (for the status line)
unsigned long rejectedFrames = 0;        // frames thrown away as out of range
unsigned long heldZeroFrames = 0;        // zero-cadence frames covered by the 1.5 s hold

// ---- Status LED (blue, active-low: LOW = on) ----
// On when Bluetooth is connected, off otherwise.
// If your LED turns out to be the other way round, swap LED_ON and LED_OFF.
#define LED_PIN 8
#define LED_ON  LOW
#define LED_OFF HIGH

void updateStatusLed() {
  digitalWrite(LED_PIN, isBluetoothConnected() ? LED_ON : LED_OFF);
}

const int D2_PIN = 6; // Connected to target D2
const int D3_PIN = 7; // Connected to target D3
const int TX_PIN = 4; // Connected to target RX (D0)
const int RX_PIN = 5; // Connected to target TX (D1)

void sendConfigCommand(String writeValue, unsigned int delayMs = 500) {
  delay(delayMs);
  Serial1.print(writeValue);
  Serial1.flush();
  logInfo("BIKE", "sent config command %s", writeValue.c_str());
}

BikeTelemetry parseTelemetry(String payload) {
  BikeTelemetry data = {0, 0, 0};
  if (payload.length() < 26) return data;
  
  String resHex = payload.substring(2, 8);
  uint8_t rLow  = strtol(resHex.substring(0, 2).c_str(), NULL, 16);
  uint8_t rMid  = strtol(resHex.substring(2, 4).c_str(), NULL, 16);
  uint8_t rHigh = strtol(resHex.substring(4, 6).c_str(), NULL, 16);
  data.resistance = ((uint32_t)rHigh << 16) | ((uint16_t)rMid << 8) | rLow;
  
  String cadenceHex = payload.substring(20, 24);
  uint8_t cLow  = strtol(cadenceHex.substring(0, 2).c_str(), NULL, 16);
  uint8_t cHigh = strtol(cadenceHex.substring(2, 4).c_str(), NULL, 16);
  data.cadence = ((uint16_t)cHigh << 8) | cLow;
  
  String counterHex = payload.substring(24, 26);
  data.counter = (uint8_t)strtol(counterHex.c_str(), NULL, 16);
  
  return data;
}

ParsedPacket parsePacket(String rawData) {
  ParsedPacket pkt;
  pkt.isValid = false;

  if (rawData.startsWith("<")) rawData.remove(0, 1);
  if (rawData.length() < 20) return pkt; 

  String lengthHex = rawData.substring(0, 2);
  pkt.header       = rawData.substring(2, 8);
  String counterHex = rawData.substring(8, 12);
  pkt.checksum     = rawData.substring(rawData.length() - 8);
  pkt.payload      = rawData.substring(12, rawData.length() - 8);
  pkt.sequenceCounter = strtol(counterHex.c_str(), NULL, 16);
  pkt.isValid = true;

  return pkt;
}

float calculateRPM(long rawCadenceTicks) {
  if (rawCadenceTicks <= 0) return 0.0f;
  return (float)rawCadenceTicks / 100.0f;
}

// ---------------------------------------------------------------------------
// SIMULATOR: with SIM_MODE 1 no bike is needed. loop() is fed fake frames in the
// real packet format, so the parser, sanity checks, hold logic and BLE code all
// run exactly as on the bike.
//
// Profile (all times since boot):
//   optional ramp 0 -> SIM_RPM over SIM_RAMP_SECONDS (0 = none)
//   then steady SIM_RPM with a slight wobble
// With SIM_GLITCHES 1 it also injects the failures we are hunting:
//   every 60 s: bike goes silent for 3 s (t%60 in 45..48)
//   every 90 s: one frame with cadence 0 (t%90 in 30..31)
//   every 120 s: one garbled frame with cadence 65535 (t%120 in 100..101)
// ---------------------------------------------------------------------------
#define SIM_MODE      0
#define SIM_RAMP_SECONDS 0    // 0 = steady SIM_RPM from the first second (clean baseline); e.g. 30 = ramp up from 0
#define SIM_GLITCHES  0       // 1 = inject the silent gap / zero / garbled frames described above
#define SIM_RPM       62.0f   // ~22 km/h with SIM_RESISTANCE 7 (see spd= in the status line)
#define SIM_RESISTANCE 7      // 0..100, same scale the watch gets

#if SIM_MODE
static String simHex(uint32_t value, int bytes) {   // little-endian hex, 2 chars per byte
  String s;
  char buf[3];
  for (int i = 0; i < bytes; i++) {
    snprintf(buf, sizeof(buf), "%02X", (unsigned)((value >> (8 * i)) & 0xFF));
    s += buf;
  }
  return s;
}

static String simMakeFrame(uint32_t cadenceTicks, uint32_t resistanceRaw, uint8_t counter) {
  // Layout matches parsePacket()/parseTelemetry(): len(2) header(6) seq(4) payload(26) checksum(8)
  String payload = "00" + simHex(resistanceRaw, 3) + "000000000000"
                   + simHex(cadenceTicks, 2) + simHex(counter, 1);
  return "<1E" "000000" + simHex(counter, 2) + payload + "00000000";
}

static bool simFrameAvailable() {
  static unsigned long lastFrameMs = 0;
  unsigned long now = millis();
#if SIM_GLITCHES
  unsigned long t = now / 1000UL;
  if (t % 60 >= 45 && t % 60 < 48) return false;       // bike silent
#endif
  if (now - lastFrameMs < 100) return false;           // ~10 frames per second
  lastFrameMs = now;
  return true;
}

static String simReadFrame() {
  static uint8_t counter = 0;
  counter++;
  float t = millis() / 1000.0f;
  float rpm = (SIM_RAMP_SECONDS > 0 && t < (float)SIM_RAMP_SECONDS)
                  ? SIM_RPM * (t / (float)SIM_RAMP_SECONDS)
                  : SIM_RPM + 2.0f * sinf(t / 5.0f);
#if SIM_GLITCHES
  unsigned long ts = millis() / 1000UL;
  if (ts % 90 >= 30 && ts % 90 < 31) {                 // one zero-cadence frame per window
    static unsigned long lastZeroWindow = 0;
    if (lastZeroWindow != ts / 90 + 1) { lastZeroWindow = ts / 90 + 1; rpm = 0.0f; }
  }
  if (ts % 120 >= 100 && ts % 120 < 101) {             // one garbled frame per window
    static unsigned long lastGarbleWindow = 0;
    if (lastGarbleWindow != ts / 120 + 1) {
      lastGarbleWindow = ts / 120 + 1;
      return simMakeFrame(65535, (uint32_t)(SIM_RESISTANCE / 100.0f * 4095.0f), counter);
    }
  }
#endif
  uint32_t resRaw = (uint32_t)(SIM_RESISTANCE / 100.0f * 4095.0f + 0.5f);
  return simMakeFrame((uint32_t)(rpm * 100.0f), resRaw, counter);
}
#endif

static bool frameAvailable() {
#if SIM_MODE
  return simFrameAvailable();
#else
  return Serial1.available();
#endif
}

static String readFrame() {
#if SIM_MODE
  return simReadFrame();
#else
  return Serial1.readStringUntil(';');
#endif
}

void setup() {
  Serial1.setRxBufferSize(2048);
  Serial.begin(115200);
  //delay(1000);

  // Status LED off until a device connects
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_OFF);

  // OLED: starts its own background task (see BikeDisplay.cpp)
  initDisplay();

  // Initialize Bluetooth Server & Advertising
  initBluetooth();

  // Initialize control pins as outputs, default HIGH
  pinMode(D2_PIN, OUTPUT);
  pinMode(D3_PIN, OUTPUT);
  digitalWrite(D2_PIN, HIGH);
  digitalWrite(D3_PIN, HIGH);

  delay(1000);

  Serial1.begin(230400, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial1.setTimeout(50);   // default is 1000 ms: a half-received frame would freeze loop() for a second
  Serial.begin(230400);
  

  sendConfigCommand("<090002330002000B006028D9E9;");

  logInfo("BOOT", "ready. SIM_MODE=%d SIM_GLITCHES=%d DEMO_MODE=%d  (search the log for \"!!\" to find problems)",
          SIM_MODE, SIM_GLITCHES, DEMO_MODE);
}

void loop() {
  updateStatusLed();

  // Drain every frame that is waiting (up to 20 per pass) instead of one per loop,
  // so a backlog can't build up in the RX buffer and make the data stale/garbled.
  int framesThisPass = 0;
  while (frameAvailable() && framesThisPass++ < 20) {
    String response = readFrame();
    response.trim();

    ParsedPacket result = parsePacket(response);





    if (result.isValid) {
      // Log each new packet header the first time it appears (up to 6), so we can
      // tell which frame types the bike sends without flooding the log.
      static String seenHeaders[6];
      static int numSeen = 0;
      bool known = false;
      for (int i = 0; i < numSeen; i++) if (seenHeaders[i] == result.header) known = true;
      if (!known && numSeen < 6) {
        seenHeaders[numSeen++] = result.header;
        if (numSeen == 1) logInfo("PKT", "first frame header=%s len=%d", result.header.c_str(), (int)response.length());
        else              logAlert("PKT", "NEW frame type header=%s len=%d (seen %d types)", result.header.c_str(), (int)response.length(), numSeen);
      }

      if (response.length() > 42) {
        BikeTelemetry telemetry = parseTelemetry(result.payload);

        // Sanity limits: reject garbled/truncated frames (cadence ticks are RPM*100,
        // resistance raw is 0..4095).
        if (telemetry.cadence > 20000 || telemetry.resistance > 4095) {
          rejectedFrames++;
          static unsigned long lastRejectMs = 0;
          if (millis() - lastRejectMs >= 1000) {     // at most one line per second
            lastRejectMs = millis();
            logAlert("REJECT", "bad frame dropped: cadence=%ld (max 20000) res=%d (max 4095), %lu rejected so far",
                     telemetry.cadence, telemetry.resistance, rejectedFrames);
          }
          continue;
        }
        lastValidPacketMs = millis();

        float currentRpm = calculateRPM(telemetry.cadence);

        // Hold the last non-zero cadence for 1.5 s so one bad/short frame doesn't
        // register as a stop (which drags the average down on the watch).
        static unsigned long lastNonZeroMs = 0;
        static float lastNonZeroRpm = 0.0f;
        if (currentRpm > 0.0f) {
          lastNonZeroMs = millis();
          lastNonZeroRpm = currentRpm;
        } else if (millis() - lastNonZeroMs < 1500) {
          currentRpm = lastNonZeroRpm;
          heldZeroFrames++;
        }
        lastRpmShown = currentRpm;

        uint16_t currentResistance = (uint16_t)((telemetry.resistance / 4095.0f) * 100.0f);
        lastResistanceLevel = currentResistance;

        // Send live data to Apple Watch / iPhone via NimBLE (throttled to ~1 Hz internally)
        sendCyclingDataToWatch(currentRpm, currentResistance);


        // Just stores the values; the display task redraws a few times a second
        setDisplayValues(currentResistance, telemetry.cadence, currentRpm);
      }
    }
  }

#if DEMO_MODE
  // Runs independently of the serial port: if no real bike data has arrived
  // recently, keep feeding fake cadence so the watch always has something.
  if (millis() - lastValidPacketMs > 1500) {
    sendCyclingDataToWatch(69.420f, 9.0f);
  }
#else
  // No bike data for 1.5 s: keep notifying with 0 RPM instead of going silent, so the
  // watch sees "stopped" with a live sensor rather than a dead one it may drop.
  if (millis() - lastValidPacketMs > 1500) {
    sendCyclingDataToWatch(0.0f, lastResistanceLevel);
  }
#endif

  // Keep the OLED's connection rows current (this only stores numbers; the display task draws)
  setDisplayConnection(isBluetoothConnected(), millis() - lastValidPacketMs, bleEstimatedSpeedKmh(),
                       bleDisconnectCount(), bleLastDisconnectReason(), rejectedFrames);

  // Mark the moment the bike goes quiet and the moment it comes back
  static bool bikeSilent = false;
  static unsigned long lastPacketBeforeSilence = 0;
  bool bikeStale = (millis() - lastValidPacketMs > 1500);
  if (bikeStale && !bikeSilent) {
    bikeSilent = true;
    lastPacketBeforeSilence = lastValidPacketMs;
    logAlert("BIKE", "no data for 1.5 s, sending 0 RPM to the watch until it returns");
  } else if (!bikeStale && bikeSilent) {
    bikeSilent = false;
    logInfo("BIKE", "data is back after a %lu ms gap", lastValidPacketMs - lastPacketBeforeSilence);
  }

  // One line per second. The word after STAT is the health check:
  //   OK        everything flowing
  //   NO-WATCH  nothing connected to the ESP32
  //   NO-BIKE   no valid bike data for over 1.5 s
  // Other fields: bike= ms since the last good bike frame, rx= bytes waiting in the
  // serial buffer (should stay near 0), rej= bad frames dropped, held= zero frames covered.
  static unsigned long lastStatusMs = 0;
  if (millis() - lastStatusMs >= 1000) {
    lastStatusMs = millis();
    bool watchUp = isBluetoothConnected();
    const char* state = !watchUp ? (bikeStale ? "NO-WATCH+NO-BIKE" : "NO-WATCH")
                                 : (bikeStale ? "NO-BIKE" : "OK");
    char line[160];
    snprintf(line, sizeof(line), "%-8s rpm=%.1f res=%u spd=%.1f ble=%d bike=%lums rx=%d heap=%u rej=%lu held=%lu",
             state, lastRpmShown, (unsigned)lastResistanceLevel, bleEstimatedSpeedKmh(),
             watchUp ? 1 : 0, (unsigned long)(millis() - lastValidPacketMs),
             Serial1.available(), (unsigned)ESP.getFreeHeap(), rejectedFrames, heldZeroFrames);
    if (watchUp && !bikeStale) logInfo("STAT", "%s", line);
    else                       logAlert("STAT", "%s", line);
  }
}
