# BLE gateway (PlantPal)

A MeshCore **sensor** node that additionally bridges **any BLE advert** onto the
LoRa mesh. Built for the PlantPal allotment: nearby BLE devices broadcast adverts
(BTHome sensors and anything else in range); this gateway picks them up and relays
them to the companion radio at home, where the `plantpal-bridge` C# service decodes
and forwards them to Home Assistant + plantpal.ie.

nRF52-only (uses the Adafruit Bluefruit BLE stack's central/observer scanner).

## How it works

- `BleScanner` — never-ending passive Bluefruit scan; every advert seen (any
  payload, any device) is copied into a FreeRTOS queue (Bluefruit's scan callback
  runs on its own task, so the mesh is never touched from there).
- `BleGatewayMesh` (extends `SensorMesh`) — buffers distinct adverts (mac + raw
  payload) seen during a `FORWARD_AFTER_SECS` window (default 60s), silently
  dropping exact repeats. When the window elapses it forwards the whole batch as
  encrypted `GRP_DATA` group datagrams on the private **PlantPal** channel, paced
  by a `MIN_SEND_GAP_MILLIS` (default 500ms) gap between sends, then clears the
  buffer and starts a fresh window.
- Still a full sensor node: telemetry (temp/humidity/battery via the environment
  sensor manager), remote admin CLI, WiFi-OTA trigger — a dead BLE stack degrades to
  a plain sensor node instead of taking the node offline.

The per-window batch-and-dedup approach is deliberately simple (exact-byte-match
dedup, fixed window). If the mesh needs to be smarter about it later — e.g. only
resending a device's reading when it actually changes, or adapting the window to
traffic — that can be layered on `BleGatewayMesh` without touching the scanner.

## Wire format

Group datagram, `data_type = 0xFF01` (MeshCore dev range). Payload:

| Offset | Size | Meaning                                              |
|--------|------|-------------------------------------------------------|
| 0      | 1    | format version (`0x01`)                              |
| 1      | 6    | BLE MAC, display order                               |
| 7      | 1    | RSSI at the gateway (int8, dBm)                      |
| 8      | 1    | N = advert payload length                            |
| 9      | N    | raw legacy BLE advertising payload (AD structures)   |

Channel secret: 16-byte PSK (`PLANTPAL_CHANNEL_PSK`, base64) — must match the
"PlantPal" channel the bridge configures on the companion radio.

## Extra CLI commands (serial or remote admin)

- `gw stats` — buffered-in-current-window count, forwarded total, buffer-full
  drops, packet-pool failures, channel state
- `gw now` — flush the current batch immediately instead of waiting out the window
