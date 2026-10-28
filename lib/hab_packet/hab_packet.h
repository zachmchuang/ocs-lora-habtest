#pragma once

#include <Arduino.h>
#include <stdint.h>

static const uint8_t HAB_PKT_MAGIC   = 0xA5;
static const uint8_t HAB_PKT_VERSION = 1;

enum HabPktType : uint8_t {
  HAB_PKT_TELEM = 1,
  HAB_PKT_PING  = 2,
  HAB_PKT_ACK   = 3,
};

enum HabFlags : uint8_t {
  HAB_FLAG_GPS_OK = 1 << 0,
  HAB_FLAG_IMU_OK = 1 << 1,
  HAB_FLAG_MAG_OK = 1 << 2,
};

// Packed sample carried in TELEM and ACK bodies.
struct HabSample {
  uint32_t ts_ms;       // millis or GPS-derived marker
  int32_t  lat_e7;      // degrees * 1e7
  int32_t  lon_e7;
  int32_t  alt_mm;      // altitude millimetres
  int16_t  tilt_cdeg;   // centi-degrees (0.01°)
  int16_t  pitch_cdeg;
  int16_t  roll_cdeg;
  int16_t  heading_cdeg;
  uint8_t  sats;
  uint8_t  flags;       // HabFlags
};

struct HabPacket {
  uint8_t   type;
  uint16_t  seq;
  HabSample sample;     // meaningful for TELEM/ACK; zeroed for PING
};

static const size_t HAB_PKT_HEADER_LEN = 1 + 1 + 1 + 2;  // magic,ver,type,seq
static const size_t HAB_SAMPLE_LEN =
    4 + 4 + 4 + 4 + 2 + 2 + 2 + 2 + 1 + 1;  // 26
static const size_t HAB_PKT_TELEM_LEN = HAB_PKT_HEADER_LEN + HAB_SAMPLE_LEN;
static const size_t HAB_PKT_PING_LEN  = HAB_PKT_HEADER_LEN;
static const size_t HAB_PKT_MAX_LEN   = HAB_PKT_TELEM_LEN;

size_t habPacketEncode(const HabPacket& pkt, uint8_t* out, size_t outCap);
bool   habPacketDecode(const uint8_t* data, size_t len, HabPacket& out);

void habSampleClear(HabSample& s);

// USB / SD formatters
void habPrettyPrintTelem(Print& out, const HabPacket& pkt, float rssi, float snr);
void habPrettyPrintAck(Print& out, const HabPacket& pkt, float rssi, float snr,
                       int32_t rtt_ms);
void habCsvHeaderGround(Print& out);
void habCsvLineGround(Print& out, const HabPacket& pkt, float rssi, float snr,
                      const char* rx_utc, int32_t rtt_ms);
void habCsvHeaderPayload(Print& out);
void habCsvLinePayload(Print& out, const HabSample& s, const char* gps_time);
