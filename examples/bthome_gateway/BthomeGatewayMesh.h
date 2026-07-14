#pragma once

#include "SensorMesh.h"
#include "BleScanner.h"

// 16-byte channel secret, base64. Must match the channel the bridge configures on
// the companion radio ("PlantPal"). This default is a placeholder — set the real
// PSK per-env in platformio.local.ini (gitignored), never commit it.
#ifndef PLANTPAL_CHANNEL_PSK
  #define PLANTPAL_CHANNEL_PSK "REPLACE-WITH-16-BYTE-PSK-BASE64"
#endif

// Group-datagram application id (MeshCore dev/testing range FF00-FFFF).
#ifndef PLANTPAL_DATA_TYPE
  #define PLANTPAL_DATA_TYPE 0xFF01
#endif

// How often (at most) each BLE device's latest advert is forwarded onto the mesh.
#ifndef FORWARD_INTERVAL_SECS
  #define FORWARD_INTERVAL_SECS 300
#endif

// Minimum gap between any two mesh sends, so several due devices don't burst-flood.
#ifndef MIN_SEND_GAP_MILLIS
  #define MIN_SEND_GAP_MILLIS 5000
#endif

#define MAX_TRACKED_DEVICES 12

/// A MeshCore sensor node that also bridges BTHome BLE adverts onto the mesh.
///
/// The BleScanner captures raw BTHome v2 service data from nearby nodes (air, pump,
/// fan, irrigation, ...). This class keeps the freshest advert per BLE device and
/// forwards it as an encrypted group datagram on the PlantPal channel, at most once
/// per FORWARD_INTERVAL_SECS per device. The companion radio at the far end hands
/// the datagrams to the C# bridge, which decodes the BTHome payload.
///
/// Wire format of the datagram payload (after the GRP_DATA data_type/len header):
///   [0]     format version (1)
///   [1..6]  BLE MAC, display order
///   [7]     RSSI as seen by this gateway (int8, dBm)
///   [8]     service data length N
///   [9..]   raw BTHome v2 service data (starts with the device-info byte)
class BthomeGatewayMesh : public SensorMesh {
public:
  BthomeGatewayMesh(mesh::MainBoard& board, mesh::Radio& radio, mesh::MillisecondClock& ms,
                    mesh::RNG& rng, mesh::RTCClock& rtc, mesh::MeshTables& tables)
      : SensorMesh(board, radio, ms, rng, rtc, tables) {}

  /// Derives the group channel from PLANTPAL_CHANNEL_PSK. Call once after begin().
  /// Returns false on a malformed PSK (node keeps running, just doesn't forward).
  bool BeginChannel();

  /// Records the freshest advert for its device; actual sending happens in FlushPending().
  void ProcessAdvert(const ScannedAdvert& advert);

  /// Sends any adverts whose per-device forward interval has elapsed. Call every loop().
  void FlushPending();

protected:
  void onSensorDataRead() override;
  int querySeriesData(uint32_t start_secs_ago, uint32_t end_secs_ago, MinMaxAvg dest[], int max_num) override {
    return 0;
  }
  bool handleCustomCommand(uint32_t sender_timestamp, char* command, char* reply) override;

private:
  struct TrackedDevice {
    bool inUse = false;
    bool hasPending = false;
    bool everForwarded = false;
    unsigned long lastForwardMillis = 0;
    uint32_t forwardedCount = 0;
    ScannedAdvert latest = {};
  };

  mesh::GroupChannel _channel;
  bool _channelReady = false;
  TrackedDevice _devices[MAX_TRACKED_DEVICES];
  unsigned long _lastSendMillis = 0;
  uint32_t _totalForwarded = 0;
  uint32_t _packetPoolFailures = 0;
  Trigger _lowBatt, _criticalBatt;

  TrackedDevice* findOrAddDevice(const uint8_t mac[6]);
  bool sendAdvert(const ScannedAdvert& advert);
};
