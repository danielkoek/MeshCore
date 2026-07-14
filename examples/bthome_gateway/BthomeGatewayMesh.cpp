#include "BthomeGatewayMesh.h"

#include <base64.hpp>

bool BthomeGatewayMesh::BeginChannel() {
  memset(_channel.secret, 0, sizeof(_channel.secret));
  int len = decode_base64((unsigned char*)PLANTPAL_CHANNEL_PSK, strlen(PLANTPAL_CHANNEL_PSK), _channel.secret);
  if (len != 16 && len != 32) {
    Serial.println("BthomeGateway: invalid PLANTPAL_CHANNEL_PSK, forwarding disabled");
    return false;
  }
  mesh::Utils::sha256(_channel.hash, sizeof(_channel.hash), _channel.secret, len);
  _channelReady = true;
  return true;
}

void BthomeGatewayMesh::ProcessAdvert(const ScannedAdvert& advert) {
  TrackedDevice* device = findOrAddDevice(advert.mac);
  if (device == nullptr) return;  // table full — ignore newcomers rather than evict

  device->latest = advert;   // freshest advert wins; sent when the interval elapses
  device->hasPending = true;
}

void BthomeGatewayMesh::FlushPending() {
  if (!_channelReady) return;

  unsigned long now = millis();
  if (now - _lastSendMillis < MIN_SEND_GAP_MILLIS) return;   // one send per gap, max

  for (int i = 0; i < MAX_TRACKED_DEVICES; i++) {
    TrackedDevice& device = _devices[i];
    if (!device.inUse || !device.hasPending) continue;

    bool intervalElapsed = (now - device.lastForwardMillis) >= (unsigned long)FORWARD_INTERVAL_SECS * 1000UL;
    if (device.everForwarded && !intervalElapsed) continue;

    if (sendAdvert(device.latest)) {
      device.hasPending = false;
      device.everForwarded = true;
      device.lastForwardMillis = now;
      device.forwardedCount++;
      _totalForwarded++;
      _lastSendMillis = now;
    } else {
      _packetPoolFailures++;
    }
    return;   // at most one send per call; the gap timer paces the rest
  }
}

bool BthomeGatewayMesh::sendAdvert(const ScannedAdvert& advert) {
  // GRP_DATA payload = [data_type LE][app_len] followed by the app payload
  // documented in the class comment.
  uint8_t blob[3 + 9 + sizeof(advert.serviceData)];
  uint8_t appLen = 9 + advert.serviceDataLen;

  blob[0] = (uint8_t)(PLANTPAL_DATA_TYPE & 0xFF);
  blob[1] = (uint8_t)(PLANTPAL_DATA_TYPE >> 8);
  blob[2] = appLen;
  blob[3] = 0x01;  // format version
  memcpy(&blob[4], advert.mac, 6);
  blob[10] = (uint8_t)advert.rssi;
  blob[11] = advert.serviceDataLen;
  memcpy(&blob[12], advert.serviceData, advert.serviceDataLen);

  auto pkt = createGroupDatagram(PAYLOAD_TYPE_GRP_DATA, _channel, blob, 3 + appLen);
  if (pkt == nullptr) return false;   // packet pool exhausted; retry next flush

  sendFlood(pkt);
  return true;
}

BthomeGatewayMesh::TrackedDevice* BthomeGatewayMesh::findOrAddDevice(const uint8_t mac[6]) {
  for (int i = 0; i < MAX_TRACKED_DEVICES; i++) {
    if (_devices[i].inUse && memcmp(_devices[i].latest.mac, mac, 6) == 0) return &_devices[i];
  }
  for (int i = 0; i < MAX_TRACKED_DEVICES; i++) {
    if (!_devices[i].inUse) {
      _devices[i].inUse = true;
      return &_devices[i];
    }
  }
  return nullptr;
}

void BthomeGatewayMesh::onSensorDataRead() {
  float battVoltage = getVoltage(TELEM_CHANNEL_SELF);
  alertIf(battVoltage < 3.4f, _criticalBatt, HIGH_PRI_ALERT, "Gateway battery is critical!");
  alertIf(battVoltage < 3.6f, _lowBatt, LOW_PRI_ALERT, "Gateway battery is low");
}

bool BthomeGatewayMesh::handleCustomCommand(uint32_t sender_timestamp, char* command, char* reply) {
  if (strcmp(command, "gw stats") == 0) {
    int tracked = 0;
    for (int i = 0; i < MAX_TRACKED_DEVICES; i++) {
      if (_devices[i].inUse) tracked++;
    }
    sprintf(reply, "tracked=%d fwd=%lu poolfail=%lu chan=%s",
            tracked, (unsigned long)_totalForwarded, (unsigned long)_packetPoolFailures,
            _channelReady ? "ok" : "OFF");
    return true;
  }
  if (strcmp(command, "gw now") == 0) {   // force-forward everything pending
    for (int i = 0; i < MAX_TRACKED_DEVICES; i++) {
      if (_devices[i].inUse) _devices[i].everForwarded = false;
    }
    _lastSendMillis = 0;
    strcpy(reply, "flush scheduled");
    return true;
  }
  return false;
}
