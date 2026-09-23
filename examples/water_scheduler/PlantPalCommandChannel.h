#pragma once

// Lets a water-scheduler node receive pump on/off/auto commands over the same
// "PlantPal" mesh group channel the ble_gateway uses for BLE-advert
// forwarding, instead of an authenticated admin CLI session per node. It's a
// broadcast, so a target id picks out which node a command is for; the node
// also broadcasts a small ack back on the same channel so the bridge/worker
// can tell the command was actually delivered and applied.
//
// Wire format (after the channel's own MAC-then-decrypt, matching the
// [data_type LE][app_len] GRP_DATA header ble_gateway already uses):
//   Command (data_type 0xFF02): [0]=version(1) [1..6]=target id [7]=action [8..11]=seq (LE)
//   Ack     (data_type 0xFF03): [0]=version(1) [1..6]=sender id [7]=status  [8..11]=echoed seq (LE)
//
// action: 0=off, 1=on, 2=auto     status: 0=ok, 1=error (command rejected by the scheduler)

#include <Mesh.h>
#include <base64.hpp>

#ifndef PLANTPAL_CHANNEL_PSK
  #define PLANTPAL_CHANNEL_PSK "REPLACE-WITH-16-BYTE-PSK-BASE64"
#endif

#define PLANTPAL_PUMP_CMD_DATA_TYPE  0xFF02
#define PLANTPAL_PUMP_ACK_DATA_TYPE  0xFF03
#define PLANTPAL_PUMP_APP_LEN        12   // version + id(6) + action/status(1) + seq(4)

enum PlantPalPumpAction : uint8_t {
  PLANTPAL_PUMP_OFF  = 0,
  PLANTPAL_PUMP_ON   = 1,
  PLANTPAL_PUMP_AUTO = 2,
};

/// This node's stable 6-byte channel id, derived from its own identity — same
/// "first 6 bytes of the pubkey" convention as the BLE MAC field, so it shows
/// up in the same devices list and can be tagged the same way.
inline void getPlantPalSelfId(const mesh::LocalIdentity& self_id, uint8_t out[6]) {
  memcpy(out, self_id.pub_key, 6);
}

/// Derives the shared GroupChannel from PLANTPAL_CHANNEL_PSK. Returns false on
/// a malformed PSK (node keeps running, just doesn't receive/ack commands).
inline bool derivePlantPalChannel(mesh::GroupChannel& channel) {
  memset(channel.secret, 0, sizeof(channel.secret));
  int len = decode_base64((unsigned char*)PLANTPAL_CHANNEL_PSK, strlen(PLANTPAL_CHANNEL_PSK), channel.secret);
  if (len != 16 && len != 32) return false;
  mesh::Utils::sha256(channel.hash, sizeof(channel.hash), channel.secret, len);
  return true;
}

/// Parses a decrypted GRP_DATA blob; returns true and fills the out params if
/// it's a v1 pump command addressed to `self_id_6`.
inline bool decodePlantPalPumpCommand(const uint8_t* data, size_t len, const uint8_t self_id_6[6],
                                       PlantPalPumpAction& out_action, uint32_t& out_seq) {
  if (len < 3) return false;
  uint16_t data_type = data[0] | (data[1] << 8);
  uint8_t app_len = data[2];
  if (data_type != PLANTPAL_PUMP_CMD_DATA_TYPE) return false;
  if (app_len < PLANTPAL_PUMP_APP_LEN || (size_t)(3 + app_len) > len) return false;

  const uint8_t* app = &data[3];
  if (app[0] != 1) return false;                          // unsupported format version
  if (memcmp(&app[1], self_id_6, 6) != 0) return false;    // not addressed to us

  out_action = (PlantPalPumpAction)app[7];
  memcpy(&out_seq, &app[8], 4);
  return true;
}

/// Encodes a v1 ack blob (full GRP_DATA app payload, header included) into
/// `out`, which must be at least 15 bytes. Returns the blob length to send.
inline size_t encodePlantPalPumpAck(uint8_t* out, const uint8_t self_id_6[6], uint8_t status, uint32_t seq) {
  out[0] = (uint8_t)(PLANTPAL_PUMP_ACK_DATA_TYPE & 0xFF);
  out[1] = (uint8_t)(PLANTPAL_PUMP_ACK_DATA_TYPE >> 8);
  out[2] = PLANTPAL_PUMP_APP_LEN;
  out[3] = 1;   // format version
  memcpy(&out[4], self_id_6, 6);
  out[10] = status;
  memcpy(&out[11], &seq, 4);
  return 3 + PLANTPAL_PUMP_APP_LEN;
}
