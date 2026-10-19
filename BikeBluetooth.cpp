#include "BikeBluetooth.h"
#include <math.h>
#include <esp_timer.h>

// How often to send data to the watch (milliseconds)
// ~1 Hz like a real CSC sensor: at 250 ms most notifications carry no new
// crank/wheel event, and receivers average those as zero speed/cadence.
#define BLE_UPDATE_INTERVAL_MS 1000

// ---- Speed / distance estimate (tune these against the console) ----
// Estimated power (W) = RPM * (POWER_BASE + POWER_PER_RES * resistance)
// Estimated speed (km/h) = SPEED_K * cuberoot(power)
static const float POWER_BASE            = 0.6f;    // W per RPM at resistance 0
static const float POWER_PER_RES         = 0.045f;  // extra W per RPM per resistance level
static const float SPEED_K               = 5.9f;    // km/h per cube-root-watt
static const float WHEEL_CIRCUMFERENCE_M = 2.1f;    // metres per simulated wheel turn

// Bluetooth SIG UUIDs (Cycling Speed and Cadence profile)
#define CSC_SERVICE_UUID      "1816"
#define CSC_MEASUREMENT_UUID  "2A5B"
#define CSC_FEATURE_UUID      "2A5C"
#define CSC_LOCATION_UUID     "2A5D"
#define SC_CONTROL_POINT_UUID "2A55"

// Custom (non-SIG) UUIDs for resistance. Generate your own if you like.
#define CUSTOM_SERVICE_UUID   "7b1c0001-4f6a-4e1d-9c2b-3a5d8e7f1a20"
#define RESISTANCE_UUID       "7b1c0002-4f6a-4e1d-9c2b-3a5d8e7f1a20"

static NimBLEServer*         bleServer;
static NimBLECharacteristic* measurementCharacteristic;
static NimBLECharacteristic* resistanceCharacteristic;

static uint32_t cumulativeCrankRevolutions = 0;
static uint16_t lastCrankEventTime = 0;
static uint32_t cumulativeWheelRevolutions = 0;
static uint16_t lastWheelEventTime = 0;
static unsigned long lastBleUpdate = 0;
static float lastEstSpeedKmh = 0.0f;

// Estimate power from cadence + resistance, then speed from power.
// Pure calculation: it doesn't depend on whether a watch is connected.
static double estimateSpeedKmh(float cadenceRPM, uint16_t resistanceLevel) {
    if (cadenceRPM <= 0.0f) return 0.0;
    float estPowerW = cadenceRPM * (POWER_BASE + POWER_PER_RES * (float)resistanceLevel);
    return (double)(SPEED_K * cbrtf(estPowerW));
}

static uint16_t disconnectCount = 0;
static int lastDisconnectReasonCode = 0;

float bleEstimatedSpeedKmh() { return lastEstSpeedKmh; }
uint16_t bleDisconnectCount() { return disconnectCount; }
int bleLastDisconnectReason() { return lastDisconnectReasonCode; }

// NimBLE disconnect reasons are 0x200 + the Bluetooth HCI error code.
static const char* disconnectReason(int reason) {
    switch (reason) {
        case 520: return "link timeout: signal lost, out of range, or board reset/brown-out";
        case 531: return "the watch hung up";
        case 534: return "the ESP32 ended the link";
        case 574: return "connection setup failed";
        default:  return "other, look up the code";
    }
}

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* server, NimBLEConnInfo& connInfo) override {
        logInfo("WATCH", "connected");
    }

    void onDisconnect(NimBLEServer* server, NimBLEConnInfo& connInfo, int reason) override {
        disconnectCount++;
        lastDisconnectReasonCode = reason;
        logAlert("WATCH", "DISCONNECTED, reason %d (%s). Advertising again.", reason, disconnectReason(reason));
        NimBLEDevice::startAdvertising();
    }
};

// Minimal SC Control Point stub. The CSC spec requires this characteristic when
// wheel revolution data is supported, and some clients check for it.
// We reply "op code not supported" to every request.
class ScControlPointCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo& connInfo) override {
        NimBLEAttValue val = characteristic->getValue();
        uint8_t requestOpCode = val.size() > 0 ? val.data()[0] : 0;
        uint8_t response[3] = { 0x10, requestOpCode, 0x02 }; // response code, request op, "not supported"
        characteristic->setValue(response, sizeof(response));
        characteristic->indicate();
    }
};

bool isBluetoothConnected() {
    return bleServer && bleServer->getConnectedCount() > 0;
}

void initBluetooth() {
    NimBLEDevice::init("ESP32 Indoor Cycle");

    bleServer = NimBLEDevice::createServer();
    bleServer->setCallbacks(new ServerCallbacks());

    NimBLEService* cscService = bleServer->createService(CSC_SERVICE_UUID);

    // CSC Measurement
    measurementCharacteristic = cscService->createCharacteristic(
        CSC_MEASUREMENT_UUID,
        NIMBLE_PROPERTY::NOTIFY
    );

    // CSC Feature: wheel + crank revolution data supported
    NimBLECharacteristic* feature = cscService->createCharacteristic(
        CSC_FEATURE_UUID,
        NIMBLE_PROPERTY::READ
    );
    uint16_t features = 0x0003;
    feature->setValue((uint8_t*)&features, sizeof(features));

    // Sensor Location (0 = Other)
    NimBLECharacteristic* location = cscService->createCharacteristic(
        CSC_LOCATION_UUID,
        NIMBLE_PROPERTY::READ
    );
    uint8_t sensorLocation = 0;
    location->setValue(&sensorLocation, sizeof(sensorLocation));

    // SC Control Point (stub)
    NimBLECharacteristic* controlPoint = cscService->createCharacteristic(
        SC_CONTROL_POINT_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::INDICATE
    );
    controlPoint->setCallbacks(new ScControlPointCallbacks());

    cscService->start();

    // Resistance lives in its own custom service so the standard CSC service stays clean
    NimBLEService* customService = bleServer->createService(CUSTOM_SERVICE_UUID);
    resistanceCharacteristic = customService->createCharacteristic(
        RESISTANCE_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );
    uint16_t initialResistance = 0;
    resistanceCharacteristic->setValue((uint8_t*)&initialResistance, sizeof(initialResistance));
    customService->start();

    NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
    advertising->addServiceUUID(CSC_SERVICE_UUID);
    advertising->setAppearance(0x0485);      // Cycling: speed and cadence sensor
    advertising->enableScanResponse(true);
    advertising->setName("ESP32 Indoor Cycle");
    advertising->start();

    logInfo("BLE", "advertising as \"ESP32 Indoor Cycle\"");
}

void sendCyclingDataToWatch(float cadenceRPM, uint16_t resistanceLevel) {
    unsigned long now = millis();

    // Update the speed first, so the display and status line show it even with no watch connected
    lastEstSpeedKmh = (float)estimateSpeedKmh(cadenceRPM, resistanceLevel);

    // Throttle to ~1 Hz, like a real CSC sensor
    if (now - lastBleUpdate < BLE_UPDATE_INTERVAL_MS) return;

    // Nobody listening, nothing to do
    // (the status line already shows ble=0 / NO-WATCH, so no extra log here)
    if (!bleServer || bleServer->getConnectedCount() == 0) return;

    // First run (or first send after a long idle/disconnected period): assume one
    // interval. Otherwise use the real elapsed time, so short stalls aren't dropped.
    float dt = (float)(now - lastBleUpdate) / 1000.0f;
    if (dt > 5.0f) dt = BLE_UPDATE_INTERVAL_MS / 1000.0f;
    lastBleUpdate = now;

    // CSC event time unit is 1/1024 s, wrapping at 65536.
    // esp_timer_get_time() is 64-bit, so it doesn't wrap every ~71 minutes like micros().
    uint16_t nowTicks = (uint16_t)(((uint64_t)esp_timer_get_time() * 1024ULL / 1000000ULL) & 0xFFFF);

    // Crank: track revolutions as a continuous phase. When whole revolutions
    // complete during this interval, stamp the event with the moment the LAST
    // one finished (the fractional part of the phase tells us how long ago),
    // not with "now". That keeps delta-revs / delta-time equal to the true rate.
    static double crankPhase = 0.0;
    double crankRate = cadenceRPM / 60.0;                 // revs per second
    uint32_t prevCrankWhole = (uint32_t)crankPhase;
    crankPhase += crankRate * dt;
    uint32_t newCrankRevs = (uint32_t)crankPhase - prevCrankWhole;
    if (newCrankRevs > 0 && crankRate > 0.0) {
        double secondsAgo = (crankPhase - floor(crankPhase)) / crankRate;
        lastCrankEventTime = nowTicks - (uint16_t)(secondsAgo * 1024.0);
        cumulativeCrankRevolutions += newCrankRevs;
    }

    // Wheel: same approach, driven by an estimated speed.
    // Estimate power from cadence + resistance, then speed from power
    double estSpeedKmh = (double)lastEstSpeedKmh;

    static double wheelPhase = 0.0;
    double wheelRate = estSpeedKmh / 3.6 / WHEEL_CIRCUMFERENCE_M;  // revs per second
    uint32_t prevWheelWhole = (uint32_t)wheelPhase;
    wheelPhase += wheelRate * dt;
    uint32_t newWheelRevs = (uint32_t)wheelPhase - prevWheelWhole;
    if (newWheelRevs > 0 && wheelRate > 0.0) {
        double secondsAgo = (wheelPhase - floor(wheelPhase)) / wheelRate;
        lastWheelEventTime = nowTicks - (uint16_t)(secondsAgo * 1024.0);
        cumulativeWheelRevolutions += newWheelRevs;
    }

    // Pack CSC Measurement (flags 0x03 = wheel + crank data present)
    uint8_t data[11];
    data[0]  = 0x03;
    data[1]  = cumulativeWheelRevolutions & 0xFF;
    data[2]  = (cumulativeWheelRevolutions >> 8) & 0xFF;
    data[3]  = (cumulativeWheelRevolutions >> 16) & 0xFF;
    data[4]  = (cumulativeWheelRevolutions >> 24) & 0xFF;
    data[5]  = lastWheelEventTime & 0xFF;
    data[6]  = (lastWheelEventTime >> 8) & 0xFF;
    data[7]  = cumulativeCrankRevolutions & 0xFF;
    data[8]  = (cumulativeCrankRevolutions >> 8) & 0xFF;
    data[9]  = lastCrankEventTime & 0xFF;
    data[10] = (lastCrankEventTime >> 8) & 0xFF;

    measurementCharacteristic->setValue(data, sizeof(data));
    measurementCharacteristic->notify();

    // Resistance (custom characteristic)
    // Only notify when it actually changed, to keep radio traffic down
    static int32_t lastSentResistance = -1;
    if ((int32_t)resistanceLevel != lastSentResistance) {
        lastSentResistance = resistanceLevel;
        resistanceCharacteristic->setValue((uint8_t*)&resistanceLevel, sizeof(resistanceLevel));
        resistanceCharacteristic->notify();
    }
}