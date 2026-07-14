#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

/// One BLE advert captured by the scanner: raw BTHome v2 service data plus metadata.
/// MAC is stored in display order (the order you'd read it off a label: aa:bb:cc:dd:ee:ff).
struct ScannedAdvert {
  uint8_t mac[6];
  int8_t rssi;
  uint8_t serviceDataLen;
  uint8_t serviceData[96];
};

/// Continuously scans (passive, extended-advertising capable) for BLE adverts that
/// carry BTHome v2 service data (UUID 0xFCD2) and queues them for the main loop.
///
/// NimBLE invokes its callbacks on the BLE host task, NOT the Arduino loop task,
/// so adverts are handed over through a FreeRTOS queue instead of being processed
/// in the callback. Call TryDequeue() from loop() to drain them safely.
class BleScanner {
public:
  /// Starts the BLE stack and a never-ending passive scan. Returns false if the
  /// stack or scan could not be started (the node keeps running without BLE).
  bool Begin();

  /// Pops the next queued advert. Returns false when the queue is empty.
  bool TryDequeue(ScannedAdvert& out);

  /// Total BTHome adverts seen since boot (including ones dropped by a full queue).
  uint32_t SeenCount() const;

  /// Adverts dropped because the main loop didn't drain the queue fast enough.
  uint32_t DroppedCount() const;

private:
  QueueHandle_t _queue = nullptr;
};
