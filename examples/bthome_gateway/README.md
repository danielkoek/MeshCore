# BTHome gateway (PlantPal)

A MeshCore **sensor** node that additionally bridges **BTHome v2 BLE adverts** onto the
LoRa mesh. Built for the PlantPal allotment: the nRF52 nodes (air, pump, fan,
irrigation, ultrasonic) broadcast BTHome adverts; this gateway picks them up and
relays them to the companion radio at home, where the `plantpal-bridge` C# service
decodes them and forwards to Home Assistant + plantpal.ie.

ESP32-only (uses NimBLE with extended-advertising scanning enabled via
`CONFIG_BT_NIMBLE_EXT_ADV=1`, needed for the air node's extended adverts).

## How it works

- `BleScanner` — never-ending passive NimBLE scan; anything with service data for
  UUID `0xFCD2` (BTHome) is copied into a FreeRTOS queue (NimBLE callbacks run on
  the BLE host task, so the mesh is never touched from there).
- `BthomeGatewayMesh` (extends `SensorMesh`) — keeps the freshest advert per BLE MAC
  and forwards it as an encrypted `GRP_DATA` group datagram on the private
  **PlantPal** channel, at most once per `FORWARD_INTERVAL_SECS` (default 300) per
  device, with a `MIN_SEND_GAP_MILLIS` (default 5 s) global gap so sends never burst.
- Still a full sensor node: telemetry (temp/humidity/battery via the environment
  sensor manager), remote admin CLI, WiFi-OTA trigger — a dead BLE stack degrades to
  a plain sensor node instead of taking the node offline.

## Wire format

Group datagram, `data_type = 0xFF01` (MeshCore dev range). Payload:

| Offset | Size | Meaning                                              |
|--------|------|------------------------------------------------------|
| 0      | 1    | format version (`0x01`)                              |
| 1      | 6    | BLE MAC, display order                               |
| 7      | 1    | RSSI at the gateway (int8, dBm)                      |
| 8      | 1    | N = service data length                              |
| 9      | N    | raw BTHome v2 service data (starts with device-info byte) |

Channel secret: 16-byte PSK (`PLANTPAL_CHANNEL_PSK`, base64) — must match the
"PlantPal" channel the bridge configures on the companion radio.

## Extra CLI commands (serial or remote admin)

- `gw stats` — tracked device count, forwarded total, packet-pool failures, channel state
- `gw now` — forward the freshest advert of every tracked device immediately
