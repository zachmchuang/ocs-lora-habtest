#include "hab_packet.h"

#include <string.h>

static void writeU16(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static void writeU32(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
  p[2] = (uint8_t)((v >> 16) & 0xFF);
  p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static void writeI32(uint8_t* p, int32_t v) { writeU32(p, (uint32_t)v); }

static void writeI16(uint8_t* p, int16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static uint16_t readU16(const uint8_t* p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t readU32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

static int32_t readI32(const uint8_t* p) { return (int32_t)readU32(p); }

static int16_t readI16(const uint8_t* p) {
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static size_t encodeSample(const HabSample& s, uint8_t* out) {
  writeU32(out + 0, s.ts_ms);
  writeI32(out + 4, s.lat_e7);
  writeI32(out + 8, s.lon_e7);
  writeI32(out + 12, s.alt_mm);
  writeI16(out + 16, s.tilt_cdeg);
  writeI16(out + 18, s.pitch_cdeg);
  writeI16(out + 20, s.roll_cdeg);
  writeI16(out + 22, s.heading_cdeg);
  out[24] = s.sats;
  out[25] = s.flags;
  return HAB_SAMPLE_LEN;
}

static bool decodeSample(const uint8_t* in, HabSample& s) {
  s.ts_ms        = readU32(in + 0);
  s.lat_e7       = readI32(in + 4);
  s.lon_e7       = readI32(in + 8);
  s.alt_mm       = readI32(in + 12);
  s.tilt_cdeg    = readI16(in + 16);
  s.pitch_cdeg   = readI16(in + 18);
  s.roll_cdeg    = readI16(in + 20);
  s.heading_cdeg = readI16(in + 22);
  s.sats         = in[24];
  s.flags        = in[25];
  return true;
}

void habSampleClear(HabSample& s) {
  memset(&s, 0, sizeof(s));
}

size_t habPacketEncode(const HabPacket& pkt, uint8_t* out, size_t outCap) {
  const bool withSample =
      (pkt.type == HAB_PKT_TELEM || pkt.type == HAB_PKT_ACK);
  const size_t need = withSample ? HAB_PKT_TELEM_LEN : HAB_PKT_PING_LEN;
  if (!out || outCap < need) {
    return 0;
  }

  out[0] = HAB_PKT_MAGIC;
  out[1] = HAB_PKT_VERSION;
  out[2] = pkt.type;
  writeU16(out + 3, pkt.seq);

  if (withSample) {
    encodeSample(pkt.sample, out + HAB_PKT_HEADER_LEN);
  }
  return need;
}

bool habPacketDecode(const uint8_t* data, size_t len, HabPacket& out) {
  if (!data || len < HAB_PKT_PING_LEN) {
    return false;
  }
  if (data[0] != HAB_PKT_MAGIC || data[1] != HAB_PKT_VERSION) {
    return false;
  }

  out.type = data[2];
  out.seq  = readU16(data + 3);
  habSampleClear(out.sample);

  if (out.type == HAB_PKT_PING) {
    return len >= HAB_PKT_PING_LEN;
  }
  if (out.type == HAB_PKT_TELEM || out.type == HAB_PKT_ACK) {
    if (len < HAB_PKT_TELEM_LEN) {
      return false;
    }
    return decodeSample(data + HAB_PKT_HEADER_LEN, out.sample);
  }
  return false;
}

static void printOptFloat(Print& out, const char* label, bool ok, float v,
                          const char* unit) {
  out.print(label);
  if (!ok) {
    out.println(F("n/a"));
    return;
  }
  out.print(v, 2);
  out.println(unit);
}

void habPrettyPrintTelem(Print& out, const HabPacket& pkt, float rssi,
                         float snr) {
  const HabSample& s = pkt.sample;
  out.println(F("---- TELEM ----"));
  out.printf("seq=%u  RSSI=%.1f dBm  SNR=%.1f dB\n", pkt.seq, rssi, snr);
  out.printf("ts_ms=%lu  sats=%u  flags=0x%02X\n", (unsigned long)s.ts_ms,
             s.sats, s.flags);

  if (s.flags & HAB_FLAG_GPS_OK) {
    out.printf("lat=%.7f  lon=%.7f  alt=%.1f m\n", s.lat_e7 / 1e7,
               s.lon_e7 / 1e7, s.alt_mm / 1000.0);
  } else {
    out.println(F("gps=n/a"));
  }

  printOptFloat(out, "tilt=", s.flags & HAB_FLAG_IMU_OK, s.tilt_cdeg / 100.0f,
                " deg");
  printOptFloat(out, "heading=", s.flags & HAB_FLAG_MAG_OK,
                s.heading_cdeg / 100.0f, " deg");
  out.println(F("---------------"));
}

void habPrettyPrintAck(Print& out, const HabPacket& pkt, float rssi, float snr,
                       int32_t rtt_ms) {
  const HabSample& s = pkt.sample;
  out.println(F("---- ACK ------"));
  out.printf("seq=%u  RTT=%ld ms  RSSI=%.1f  SNR=%.1f\n", pkt.seq,
             (long)rtt_ms, rssi, snr);
  if (s.flags & HAB_FLAG_GPS_OK) {
    out.printf("lat=%.7f  lon=%.7f  alt=%.1f m\n", s.lat_e7 / 1e7,
               s.lon_e7 / 1e7, s.alt_mm / 1000.0);
  } else {
    out.println(F("gps=n/a"));
  }
  printOptFloat(out, "tilt=", s.flags & HAB_FLAG_IMU_OK, s.tilt_cdeg / 100.0f,
                " deg");
  printOptFloat(out, "heading=", s.flags & HAB_FLAG_MAG_OK,
                s.heading_cdeg / 100.0f, " deg");
  out.println(F("---------------"));
}

void habCsvHeaderGround(Print& out) {
  out.println(F("CSV,rx_utc,rssi_dbm,snr_db,type,seq,lat,lon,alt_m,tilt_deg,"
                "heading_deg,gps_ok,imu_ok,mag_ok,rtt_ms"));
}

void habCsvLineGround(Print& out, const HabPacket& pkt, float rssi, float snr,
                      const char* rx_utc, int32_t rtt_ms) {
  const HabSample& s = pkt.sample;
  const bool gps = s.flags & HAB_FLAG_GPS_OK;
  const bool imu = s.flags & HAB_FLAG_IMU_OK;
  const bool mag = s.flags & HAB_FLAG_MAG_OK;

  out.print(F("CSV,"));
  out.print(rx_utc ? rx_utc : "");
  out.printf(",%.1f,%.1f,%u,%u,", rssi, snr, pkt.type, pkt.seq);

  if (gps) {
    out.printf("%.7f,%.7f,%.3f,", s.lat_e7 / 1e7, s.lon_e7 / 1e7,
               s.alt_mm / 1000.0);
  } else {
    out.print(F(",,,"));
  }

  if (imu) {
    out.printf("%.2f,", s.tilt_cdeg / 100.0);
  } else {
    out.print(F(","));
  }

  if (mag) {
    out.printf("%.2f,", s.heading_cdeg / 100.0);
  } else {
    out.print(F(","));
  }

  out.printf("%d,%d,%d,", gps ? 1 : 0, imu ? 1 : 0, mag ? 1 : 0);
  if (rtt_ms >= 0) {
    out.println(rtt_ms);
  } else {
    out.println();
  }
}

void habCsvHeaderPayload(Print& out) {
  out.println(F("ts_ms,gps_time,lat,lon,alt_m,tilt_deg,pitch_deg,roll_deg,"
                "heading_deg,sats,fix_ok,imu_ok,mag_ok"));
}

void habCsvLinePayload(Print& out, const HabSample& s, const char* gps_time) {
  const bool gps = s.flags & HAB_FLAG_GPS_OK;
  const bool imu = s.flags & HAB_FLAG_IMU_OK;
  const bool mag = s.flags & HAB_FLAG_MAG_OK;

  out.printf("%lu,", (unsigned long)s.ts_ms);
  out.print(gps_time ? gps_time : "");
  out.print(',');

  if (gps) {
    out.printf("%.7f,%.7f,%.3f,", s.lat_e7 / 1e7, s.lon_e7 / 1e7,
               s.alt_mm / 1000.0);
  } else {
    out.print(F(",,,"));
  }

  if (imu) {
    out.printf("%.2f,%.2f,%.2f,", s.tilt_cdeg / 100.0, s.pitch_cdeg / 100.0,
               s.roll_cdeg / 100.0);
  } else {
    out.print(F(",,,"));
  }

  if (mag) {
    out.printf("%.2f,", s.heading_cdeg / 100.0);
  } else {
    out.print(F(","));
  }

  out.printf("%u,%d,%d,%d\n", s.sats, gps ? 1 : 0, imu ? 1 : 0, mag ? 1 : 0);
}
