#include "BleScanner.h"

#include <bluefruit.h>

namespace {

QueueHandle_t g_queue = nullptr;
volatile uint32_t g_seenCount = 0;
volatile uint32_t g_droppedCount = 0;

/// Runs on Bluefruit's callback task — must only copy data and enqueue, never touch the mesh.
void onAdvReport(ble_gap_evt_adv_report_t* report) {
  if (g_queue != nullptr) {
    g_seenCount = g_seenCount + 1;

    ScannedAdvert advert = {};
    // The SoftDevice stores addresses little-endian; flip to display order for the wire format.
    const uint8_t* raw = report->peer_addr.addr;
    for (int i = 0; i < 6; i++) {
      advert.mac[i] = raw[5 - i];
    }
    advert.rssi = report->rssi;
    advert.dataLen = (uint8_t)min((int)report->data.len, (int)sizeof(advert.data));
    memcpy(advert.data, report->data.p_data, advert.dataLen);

    if (xQueueSend(g_queue, &advert, 0) != pdTRUE) {
      g_droppedCount = g_droppedCount + 1;
    }
  }

  // The scanner pauses itself after every report that reaches this callback;
  // resume it or the scan silently stops after the first advert.
  Bluefruit.Scanner.resume();
}

}  // namespace

bool BleScanner::Begin() {
  _queue = xQueueCreate(16, sizeof(ScannedAdvert));
  if (_queue == nullptr) return false;
  g_queue = _queue;

  Bluefruit.begin(0, 1);   // no peripheral role needed, just central/observer for scanning
  Bluefruit.setTxPower(4);

  Bluefruit.Scanner.setRxCallback(onAdvReport);
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.useActiveScan(false);   // don't round-trip for scan-response; advert payload only
  Bluefruit.Scanner.setInterval(160, 80);   // 100ms interval, 50ms window (units of 0.625ms)
  return Bluefruit.Scanner.start(0);        // 0 = scan forever
}

bool BleScanner::TryDequeue(ScannedAdvert& out) {
  if (_queue == nullptr) return false;
  return xQueueReceive(_queue, &out, 0) == pdTRUE;
}

uint32_t BleScanner::SeenCount() const { return g_seenCount; }

uint32_t BleScanner::DroppedCount() const { return g_droppedCount; }
