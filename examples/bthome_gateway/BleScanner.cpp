#include "BleScanner.h"

#include <NimBLEDevice.h>

// BTHome v2 service data UUID (allocated to Allterco Robotics / used by all BTHome devices).
static const NimBLEUUID BTHOME_SERVICE_UUID((uint16_t)0xFCD2);

namespace {

/// Runs on the NimBLE host task — must only copy data and enqueue, never touch the mesh.
class AdvertCallbacks : public NimBLEScanCallbacks {
public:
  QueueHandle_t queue = nullptr;
  volatile uint32_t seenCount = 0;
  volatile uint32_t droppedCount = 0;

  void onResult(const NimBLEAdvertisedDevice* device) override {
    if (queue == nullptr || !device->haveServiceData()) return;

    std::string data = device->getServiceData(BTHOME_SERVICE_UUID);
    if (data.empty()) return;   // not a BTHome advert

    seenCount = seenCount + 1;

    ScannedAdvert advert = {};
    // NimBLE stores addresses little-endian; flip to display order for the wire format.
    const uint8_t* raw = device->getAddress().getBase()->val;
    for (int i = 0; i < 6; i++) {
      advert.mac[i] = raw[5 - i];
    }
    advert.rssi = (int8_t)device->getRSSI();
    advert.serviceDataLen = (uint8_t)min(data.size(), sizeof(advert.serviceData));
    memcpy(advert.serviceData, data.data(), advert.serviceDataLen);

    if (xQueueSend(queue, &advert, 0) != pdTRUE) {
      droppedCount = droppedCount + 1;
    }
  }

  void onScanEnd(const NimBLEScanResults& results, int reason) override {
    // The scan should never end; if the stack stops it (host reset etc.), restart.
    NimBLEDevice::getScan()->start(0, false, true);
  }
};

AdvertCallbacks advertCallbacks;

}  // namespace

bool BleScanner::Begin() {
  _queue = xQueueCreate(16, sizeof(ScannedAdvert));
  if (_queue == nullptr) return false;
  advertCallbacks.queue = _queue;

  if (!NimBLEDevice::init("")) return false;

  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(&advertCallbacks, false);
  scan->setActiveScan(false);       // BTHome data is in the advert itself; no scan-response needed
  scan->setDuplicateFilter(false);  // we want repeats — freshest reading wins
  scan->setMaxResults(0);           // don't accumulate results in RAM; callbacks only
  return scan->start(0, false, true);
}

bool BleScanner::TryDequeue(ScannedAdvert& out) {
  if (_queue == nullptr) return false;
  return xQueueReceive(_queue, &out, 0) == pdTRUE;
}

uint32_t BleScanner::SeenCount() const { return advertCallbacks.seenCount; }

uint32_t BleScanner::DroppedCount() const { return advertCallbacks.droppedCount; }
