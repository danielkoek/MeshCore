#pragma once

#include "../SerialEthernetInterface.h"
#include <SPI.h>
#include <Ethernet.h>

// On some cores (e.g. ESP32 arduino core 2.x) the Server base class declares
// begin(uint16_t) as pure virtual, which leaves EthernetServer abstract.
// This shim implements it so the class can be instantiated on all cores.
class W5500EthernetServer : public EthernetServer {
public:
  explicit W5500EthernetServer(uint16_t port) : EthernetServer(port) {}
  using EthernetServer::begin;
  void begin(uint16_t port) { EthernetServer::begin(); }
};

/*
 * Companion frame interface over a WIZnet W5500 wired Ethernet module (ESP32),
 * using the chip's hardwired TCP/IP stack via the Arduino Ethernet library.
 *
 * Fully cooperative: all SPI access happens from the main loop (no RTOS
 * tasks, no lwIP, no WiFi/BLE stacks), so the W5500 can safely share the
 * SPI bus with the LoRa radio (separate CS pins).
 *
 * Build flags:
 *   ETHERNET_CS_PIN                                 W5500 chip-select (required)
 *   ETHERNET_SPI_SCK / _MISO / _MOSI                SPI pins (optional, else bus must already be begun)
 *   ETHERNET_DHCP_TIMEOUT_MS, ETHERNET_DHCP_RETRY_MS
 */
class W5500EthernetInterface : public SerialEthernetInterface {
  bool _isConnected;
  bool net_ready;    // have a DHCP lease, server is listening
  bool hw_missing;   // no W5500 detected -> stop retrying
  unsigned long next_dhcp_attempt;
  uint8_t _mac[6];

  W5500EthernetServer server;
  EthernetClient client;

  bool ensureNetwork();

  public:
    W5500EthernetInterface() : server(ETHERNET_TCP_PORT) {
      _isConnected = false;
      net_ready = false;
      hw_missing = false;
      next_dhcp_attempt = 0;
    }

    bool begin();
    void loop() override;

    // BaseSerialInterface methods
    bool isConnected() const override;

    int available() override;
    int read() override;
    size_t write(const uint8_t *buf, size_t size) override;
};
