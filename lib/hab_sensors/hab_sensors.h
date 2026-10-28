#pragma once

#include <Arduino.h>

#include "hab_packet.h"

struct HabSensorStatus {
  bool imu_ok;
  bool mag_ok;
  bool gps_ok;   // hardware/UART up (fix may still be false)
  bool sd_ok;
};

// Init I2C peripherals + GPS UART + SD. Soft-fails individually.
// Call after Wire/display begin (shares I2C). Enables Vext.
HabSensorStatus habSensorsBegin();

// Poll GPS UART (call frequently).
void habSensorsPollGps();

// Fill sample from current sensor state. Always sets ts_ms = millis().
// Flags reflect validity of fields written.
void habSensorsReadSample(HabSample& out, char* gpsTimeBuf, size_t gpsTimeCap);

const HabSensorStatus& habSensorsStatus();

// Session directory created on this boot, e.g. "/20261028_063015", or "".
const char* habSensorsSessionDir();

// Append one CSV data row to <session>/flight.csv.
bool habSensorsLogSample(const HabSample& s, const char* gps_time);

// Append a PING RF event (RSSI/SNR) to <session>/ping.csv.
bool habSensorsLogPing(uint16_t seq, float rssi_dbm, float snr_db);
