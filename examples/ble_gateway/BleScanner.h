#pragma once

#include <Arduino.h>

/// One BLE advert captured by the scanner: raw legacy advertising payload plus metadata.
/// MAC is stored in display order (the order you'd read it off a label: aa:bb:cc:dd:ee:ff).
struct ScannedAdvert {
  uint8_t mac[6];
  int8_t rssi;
  uint8_t dataLen;
  uint8_t data[31];   // BLE_GAP_SCAN_BUFFER_MAX for legacy (non-extended) advertising
};

/// Continuously scans (passive) for BLE adverts of any kind and queues them for the
/// main loop — this is a generic BLE advert gateway, not tied to any one payload format.
///
/// The nRF52 SoftDevice dispatches scan callbacks on Bluefruit's internal callback
/// task, NOT the Arduino loop task, so adverts are handed over through a FreeRTOS
/// queue instead of being processed in the callback. Call TryDequeue() from loop()
/// to drain them safely.
class BleScanner {
public:
  /// Starts the BLE stack and a never-ending passive scan. Returns false if the
  /// stack or scan could not be started (the node keeps running without BLE).
  bool Begin();

  /// Pops the next queued advert. Returns false when the queue is empty.
  bool TryDequeue(ScannedAdvert& out);

  /// Total adverts seen since boot (including ones dropped by a full queue).
  uint32_t SeenCount() const;

  /// Adverts dropped because the main loop didn't drain the queue fast enough.
  uint32_t DroppedCount() const;

private:
  QueueHandle_t _queue = nullptr;
};
