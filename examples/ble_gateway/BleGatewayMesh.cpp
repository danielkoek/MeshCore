#include "BleGatewayMesh.h"

#include <base64.hpp>

bool BleGatewayMesh::BeginChannel() {
  memset(_channel.secret, 0, sizeof(_channel.secret));
  int len = decode_base64((unsigned char*)PLANTPAL_CHANNEL_PSK, strlen(PLANTPAL_CHANNEL_PSK), _channel.secret);
  if (len != 16 && len != 32) {
    Serial.println("BleGateway: invalid PLANTPAL_CHANNEL_PSK, forwarding disabled");
    return false;
  }
  mesh::Utils::sha256(_channel.hash, sizeof(_channel.hash), _channel.secret, len);
  _channelReady = true;
  _windowStart = millis();
  return true;
}

bool BleGatewayMesh::isDuplicate(const ScannedAdvert& advert) const {
  for (int i = 0; i < _bufferCount; i++) {
    const ScannedAdvert& seen = _buffer[i];
    if (memcmp(seen.mac, advert.mac, 6) == 0 &&
        seen.dataLen == advert.dataLen &&
        memcmp(seen.data, advert.data, advert.dataLen) == 0) {
      return true;
    }
  }
  return false;
}

void BleGatewayMesh::ProcessAdvert(const ScannedAdvert& advert) {
  if (isDuplicate(advert)) return;   // exact repeat already queued this window

  if (_bufferCount >= MAX_BUFFERED_ADVERTS) {
    _bufferFullDrops++;
    return;
  }
  _buffer[_bufferCount++] = advert;
}

void BleGatewayMesh::FlushPending() {
  if (!_channelReady) return;

  unsigned long now = millis();

  if (!_flushing) {
    if (now - _windowStart < (unsigned long)FORWARD_AFTER_SECS * 1000UL) return;   // window not elapsed yet
    _flushing = true;
    _flushCursor = 0;
  }

  if (now - _lastSendMillis < MIN_SEND_GAP_MILLIS) return;   // one send per gap, max

  if (_flushCursor < _bufferCount) {
    if (sendAdvert(_buffer[_flushCursor])) {
      _flushCursor++;
      _totalForwarded++;
      _lastSendMillis = now;
    } else {
      _packetPoolFailures++;   // retry the same entry next call
    }
    return;
  }

  if (_bufferCount == 0 && !_heartbeatSent) {
    // nothing seen this window — send an empty marker so the far end knows
    // the gateway/channel path is alive even with no adverts to report.
    if (sendHeartbeat()) {
      _heartbeatSent = true;
      _lastSendMillis = now;
    } else {
      _packetPoolFailures++;   // retry next call
    }
    return;
  }

  // batch fully sent (or was empty) — reset for the next window
  _bufferCount = 0;
  _heartbeatSent = false;
  _flushing = false;
  _windowStart = now;
}

bool BleGatewayMesh::sendHeartbeat() {
  // GRP_DATA payload = [data_type LE][app_len] followed by app payload:
  // just the format/version byte, no advert records.
  uint8_t blob[4];
  blob[0] = (uint8_t)(PLANTPAL_DATA_TYPE & 0xFF);
  blob[1] = (uint8_t)(PLANTPAL_DATA_TYPE >> 8);
  blob[2] = 1;     // app_len
  blob[3] = 0x00;  // format version 0 == heartbeat/empty marker

  auto pkt = createGroupDatagram(PAYLOAD_TYPE_GRP_DATA, _channel, blob, 4);
  if (pkt == nullptr) return false;

  sendFlood(pkt);
  return true;
}

bool BleGatewayMesh::sendAdvert(const ScannedAdvert& advert) {
  // GRP_DATA payload = [data_type LE][app_len] followed by the app payload
  // documented in the class comment.
  uint8_t blob[3 + 9 + sizeof(advert.data)];
  uint8_t appLen = 9 + advert.dataLen;

  blob[0] = (uint8_t)(PLANTPAL_DATA_TYPE & 0xFF);
  blob[1] = (uint8_t)(PLANTPAL_DATA_TYPE >> 8);
  blob[2] = appLen;
  blob[3] = 0x01;  // format version
  memcpy(&blob[4], advert.mac, 6);
  blob[10] = (uint8_t)advert.rssi;
  blob[11] = advert.dataLen;
  memcpy(&blob[12], advert.data, advert.dataLen);

  auto pkt = createGroupDatagram(PAYLOAD_TYPE_GRP_DATA, _channel, blob, 3 + appLen);
  if (pkt == nullptr) return false;   // packet pool exhausted; retry next flush

  sendFlood(pkt);
  return true;
}

void BleGatewayMesh::onSensorDataRead() {
  float battVoltage = getVoltage(TELEM_CHANNEL_SELF);
  alertIf(battVoltage < 3.4f, _criticalBatt, HIGH_PRI_ALERT, "Gateway battery is critical!");
  alertIf(battVoltage < 3.6f, _lowBatt, LOW_PRI_ALERT, "Gateway battery is low");
}

bool BleGatewayMesh::handleCustomCommand(uint32_t sender_timestamp, char* command, char* reply) {
  if (strcmp(command, "gw stats") == 0) {
    sprintf(reply, "buffered=%d fwd=%lu fulldrop=%lu poolfail=%lu chan=%s",
            _bufferCount, (unsigned long)_totalForwarded, (unsigned long)_bufferFullDrops,
            (unsigned long)_packetPoolFailures, _channelReady ? "ok" : "OFF");
    return true;
  }
  if (strcmp(command, "gw now") == 0) {   // force the current batch out immediately
    _windowStart = 0;
    strcpy(reply, "flush scheduled");
    return true;
  }
  return false;
}
