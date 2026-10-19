#include "BikeDisplay.h"
#include "BikeLog.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT  128
#define OLED_RESET     -1      // -1 = sharing the board reset
#define SCREEN_ADDRESS 0x3C    // try 0x3D if it fails to initialise

// I2C pins for the ESP32-C3 Super Mini
#define I2C_SDA 21
#define I2C_SCL 20

// How often the screen may be redrawn (and only if a value changed).
// A full 128x128 refresh sends 2 KB over I2C, so don't make this too small.
#define DISPLAY_REFRESH_MS 250

static Adafruit_SH1107 display = Adafruit_SH1107(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

// Latest values, written by loop(), read by the display task.
// Single 32-bit writes are atomic on the ESP32, so no lock is needed for this.
static volatile int shownResistance = 0;
static volatile int shownCadence    = 0;
static volatile int shownRpmX10     = 0;   // RPM * 10, so it can be compared as an integer

// Connection info
static volatile int shownWatchUp    = 0;   // 1 = watch connected
static volatile int shownBikeAgeDs  = -1;  // -1 = bike data OK, else tenths of a second since the last good frame
static volatile int shownSpeedX10   = 0;   // km/h * 10
static volatile int shownDrops      = 0;   // watch disconnects since boot
static volatile int shownReason     = 0;   // last disconnect reason code (0 = none yet)
static volatile int shownRejected   = 0;   // bad frames dropped

struct Snapshot {
    int resistance, cadence, rpmX10;
    int watchUp, bikeAgeDs, speedX10, drops, reason, rejected;
};

static Snapshot takeSnapshot() {
    Snapshot s;
    s.resistance = shownResistance;
    s.cadence    = shownCadence;
    s.rpmX10     = shownRpmX10;
    s.watchUp    = shownWatchUp;
    s.bikeAgeDs  = shownBikeAgeDs;
    s.speedX10   = shownSpeedX10;
    s.drops      = shownDrops;
    s.reason     = shownReason;
    s.rejected   = shownRejected;
    return s;
}

// Label on the left, value at x=64 (6 px per character, so values can be ~10 characters)
static void drawRow(int y, const char* label, const char* value) {
    display.setCursor(0, y);
    display.print(label);
    display.setCursor(64, y);
    display.print(value);
}

static void drawScreen(const Snapshot& s) {
    char buf[24];
    display.clearDisplay();

    // Bike readings
    snprintf(buf, sizeof(buf), "%d", s.resistance);
    drawRow(0, "Resistance", buf);
    snprintf(buf, sizeof(buf), "%d", s.cadence);
    drawRow(10, "Cadence", buf);
    snprintf(buf, sizeof(buf), "%.1f", s.rpmX10 / 10.0f);
    drawRow(20, "RPM", buf);

    display.drawFastHLine(0, 31, SCREEN_WIDTH, SH110X_WHITE);

    // Connection info
    drawRow(36, "Watch", s.watchUp ? "ON" : "OFF");

    if (s.bikeAgeDs < 0) {
        snprintf(buf, sizeof(buf), "OK");
    } else {
        int ds = s.bikeAgeDs > 999 ? 999 : s.bikeAgeDs;
        snprintf(buf, sizeof(buf), "NONE %d.%ds", ds / 10, ds % 10);
    }
    drawRow(46, "Bike", buf);

    snprintf(buf, sizeof(buf), "%.1f km/h", s.speedX10 / 10.0f);
    drawRow(56, "Speed", buf);

    if (s.drops == 0) snprintf(buf, sizeof(buf), "0");
    else              snprintf(buf, sizeof(buf), "%d (%d)", s.drops, s.reason);
    drawRow(66, "Drops", buf);

    snprintf(buf, sizeof(buf), "%d", s.rejected);
    drawRow(76, "Rejected", buf);

    display.display();               // slow (I2C), which is why this runs in its own task
}

static void displayTask(void*) {
    Snapshot last;
    memset(&last, 0xFF, sizeof(last));      // differs from anything real, so the first pass draws
    for (;;) {
        Snapshot now = takeSnapshot();
        if (memcmp(&now, &last, sizeof(Snapshot)) != 0) {
            drawScreen(now);
            last = now;
        }
        vTaskDelay(pdMS_TO_TICKS(DISPLAY_REFRESH_MS));
    }
}

void initDisplay() {
    Wire.begin(I2C_SDA, I2C_SCL);

    if (!display.begin(SCREEN_ADDRESS, true)) {      // true = reset display
        logAlert("OLED", "SH1107 not found, continuing without display");
        return;
    }
    display.setTextColor(SH110X_WHITE);
    display.setTextSize(1);
    drawScreen(takeSnapshot());

    xTaskCreate(displayTask, "display", 4096, nullptr, 1, nullptr);
}

void setDisplayValues(uint16_t resistance, uint16_t cadenceTicks, float rpm) {
    shownResistance = resistance;
    shownCadence    = cadenceTicks;
    shownRpmX10     = (int)(rpm * 10.0f + 0.5f);
}

void setDisplayConnection(bool watchUp, uint32_t bikeAgeMs, float speedKmh,
                          uint16_t drops, int lastDisconnectReason, uint32_t rejected) {
    shownWatchUp   = watchUp ? 1 : 0;
    shownBikeAgeDs = (bikeAgeMs > 1500) ? (int)(bikeAgeMs / 100) : -1;   // only shows once it counts as lost
    shownSpeedX10  = (int)(speedKmh * 10.0f + 0.5f);
    shownDrops     = drops;
    shownReason    = lastDisconnectReason;
    shownRejected  = (int)rejected;
}
