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

// How long to buffer BLE adverts before flushing the whole (de-duplicated) batch
// onto the mesh. At the end of every window the buffer is sent out and cleared,
// then a new window starts.
#ifndef FORWARD_AFTER_SECS
  #define FORWARD_AFTER_SECS 60
#endif

// Minimum gap between any two mesh sends, so a big batch doesn't burst-flood.
#ifndef MIN_SEND_GAP_MILLIS
  #define MIN_SEND_GAP_MILLIS 500
#endif

// Max distinct adverts held per window. Adverts beyond this are dropped (counted).
#define MAX_BUFFERED_ADVERTS 32

/// A MeshCore sensor node that also bridges BLE adverts (of any kind) onto the mesh.
///
/// The BleScanner captures raw advertising payloads from any nearby BLE device.
/// This class buffers distinct adverts (mac + payload) seen during a
/// FORWARD_AFTER_SECS window, deduplicating exact repeats, then forwards the whole
/// batch as encrypted group datagrams on the PlantPal channel and starts a fresh
/// window. The companion radio at the far end hands the datagrams to the C# bridge.
///
/// If a whole window elapses with zero adverts seen, an empty heartbeat datagram
/// (format version 0, no further bytes) is sent instead, so the far end can tell
/// the gateway/channel path is alive even when there's nothing to report.
///
/// At the end of every window (batch or heartbeat), a plain GRP_TXT status message
/// ("<name>: found N BLE device(s) this window" / "no BLE devices found this
/// window") is also sent, so the summary is readable in the phone app's chat.
///
/// Wire format of the datagram payload (after the GRP_DATA data_type/len header):
///   [0]     format version (1 = advert record, 0 = heartbeat/empty marker)
///   [1..6]  BLE MAC, display order (advert record only)
///   [7]     RSSI as seen by this gateway (int8, dBm) (advert record only)
///   [8]     advert payload length N (advert record only)
///   [9..]   raw legacy BLE advertising payload (AD structures) (advert record only)
class BleGatewayMesh : public SensorMesh {
public:
  BleGatewayMesh(mesh::MainBoard& board, mesh::Radio& radio, mesh::MillisecondClock& ms,
                 mesh::RNG& rng, mesh::RTCClock& rtc, mesh::MeshTables& tables)
      : SensorMesh(board, radio, ms, rng, rtc, tables) {}

  /// Derives the group channel from PLANTPAL_CHANNEL_PSK. Call once after begin().
  /// Returns false on a malformed PSK (node keeps running, just doesn't forward).
  bool BeginChannel();

  /// Buffers the advert for the current window, unless it's an exact duplicate of
  /// one already buffered. Actual sending happens in FlushPending().
  void ProcessAdvert(const ScannedAdvert& advert);

  /// Sends the buffered batch once the window elapses, one advert per call (paced
  /// by MIN_SEND_GAP_MILLIS), then clears the buffer and starts a new window. Call
  /// every loop().
  void FlushPending();

protected:
  void onSensorDataRead() override;
  int querySeriesData(uint32_t start_secs_ago, uint32_t end_secs_ago, MinMaxAvg dest[], int max_num) override {
    return 0;
  }
  bool handleCustomCommand(uint32_t sender_timestamp, char* command, char* reply) override;

private:
  mesh::GroupChannel _channel;
  bool _channelReady = false;

  ScannedAdvert _buffer[MAX_BUFFERED_ADVERTS];
  uint8_t _bufferCount = 0;
  unsigned long _windowStart = 0;
  bool _flushing = false;
  uint8_t _flushCursor = 0;

  unsigned long _lastSendMillis = 0;
  uint32_t _totalForwarded = 0;
  uint32_t _bufferFullDrops = 0;
  uint32_t _packetPoolFailures = 0;
  Trigger _lowBatt, _criticalBatt;

  bool _heartbeatSent = false;
  bool _statusSent = false;

  static bool hasServiceData(const ScannedAdvert& advert);
  bool isDuplicate(const ScannedAdvert& advert) const;
  bool sendAdvert(const ScannedAdvert& advert);
  bool sendHeartbeat();
  bool sendStatusText(uint8_t count);
};
