#pragma once

#include "../SerialEthernetInterface.h"
#include <Arduino.h>
#include <SPI.h>
#include <ETH.h>

/*
 * Companion-protocol TCP interface for a WIZnet W5500 SPI module, using the ESP32
 * Arduino core 3.x native Ethernet driver (ETH.h, real lwIP stack, DHCP handled by
 * the driver). Frames use the same '<' / '>' framing as the serial/wifi interfaces,
 * so clients such as meshcore_py / the Home Assistant integration connect over TCP.
 *
 * Requires arduino-esp32 core >= 3.1 (e.g. the pioarduino platform). The W5500 may
 * share the SPI bus with the LoRa radio (separate CS).
 *
 * Build flags:
 *   ETHERNET_CS_PIN                            W5500 chip-select (required)
 *   ETHERNET_INT_PIN                           W5500 INT gpio (default -1 = polling mode)
 *   ETHERNET_RST_PIN                           W5500 reset gpio (default -1 = not wired)
 *   ETHERNET_SPI_SCK / _MISO / _MOSI           SPI pins (optional, else bus must already be begun)
 *   ETHERNET_SPI_FREQ_MHZ                      default 20
 *   ETHERNET_TCP_PORT                          default 5000
 *   ETHERNET_STATIC_IP/_GATEWAY/_SUBNET/_DNS   optional static config
 */

#ifndef ETHERNET_CS_PIN
  #error "W5500NativeEthernetInterface requires ETHERNET_CS_PIN"
#endif
#ifndef ETHERNET_INT_PIN
  #define ETHERNET_INT_PIN -1
#endif
#ifndef ETHERNET_RST_PIN
  #define ETHERNET_RST_PIN -1
#endif
#ifndef ETHERNET_SPI_FREQ_MHZ
  #define ETHERNET_SPI_FREQ_MHZ 20
#endif

class W5500NativeEthernetInterface : public SerialEthernetInterface {
  NetworkServer server;
  NetworkClient client;
  bool _hw_ok;
  bool _server_started;
  bool _isConnected;

public:
  W5500NativeEthernetInterface() : server(ETHERNET_TCP_PORT) {
    _hw_ok = false;
    _server_started = false;
    _isConnected = false;
  }

  bool begin();
  void loop() override;

  // BaseSerialInterface methods
  bool isConnected() const override;

  int available() override;
  int read() override;
  size_t write(const uint8_t *buf, size_t size) override;
};
