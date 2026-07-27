#pragma once

#include "../BaseSerialInterface.h"
#include <SPI.h>
#include <Ethernet.h>

// On some cores (e.g. ESP32 arduino core 2.x) the Server base class declares
// begin(uint16_t) as pure virtual, which leaves EthernetServer abstract.
// This shim implements it so the class can be instantiated on all cores.
class MeshEthernetServer : public EthernetServer {
public:
  explicit MeshEthernetServer(uint16_t port) : EthernetServer(port) {}
  using EthernetServer::begin;
  void begin(uint16_t port) { EthernetServer::begin(); }
};

/*
 * Companion frame interface over a WIZnet W5500 wired Ethernet module,
 * using the chip's hardwired TCP/IP stack via the Arduino Ethernet library.
 *
 * Fully cooperative: all SPI access happens from the main loop (no RTOS
 * tasks, no lwIP, no WiFi/BLE stacks), so the W5500 can safely share the
 * SPI bus with the LoRa radio (separate CS pins).
 *
 * NOTE: the application must call SPI.begin(sck, miso, mosi) BEFORE begin().
 */
class SerialEthernetInterface : public BaseSerialInterface {
  bool deviceConnected;
  bool _isEnabled;
  bool net_ready;    // have a DHCP lease, server is listening
  bool hw_missing;   // no W5500 detected -> stop retrying
  unsigned long _last_write;
  unsigned long next_dhcp_attempt;
  uint8_t _mac[6];

  MeshEthernetServer* _server;
  EthernetClient client;

  struct FrameHeader {
    uint8_t type;
    uint16_t length;
  };

  struct Frame {
    uint8_t len;
    uint8_t buf[MAX_FRAME_SIZE];
  };

  FrameHeader received_frame_header;

  #ifndef FRAME_QUEUE_SIZE
    #define FRAME_QUEUE_SIZE  4
  #endif
  int recv_queue_len;
  Frame recv_queue[FRAME_QUEUE_SIZE];
  int send_queue_len;
  Frame send_queue[FRAME_QUEUE_SIZE];

  void clearBuffers() { recv_queue_len = 0; send_queue_len = 0; }
  bool ensureNetwork();

public:
  SerialEthernetInterface() : _server(NULL), client(EthernetClient()) {
    deviceConnected = false;
    _isEnabled = false;
    net_ready = false;
    hw_missing = false;
    _last_write = 0;
    next_dhcp_attempt = 0;
    send_queue_len = recv_queue_len = 0;
    received_frame_header.type = 0;
    received_frame_header.length = 0;
  }

  void begin(uint16_t port, int cs_pin, const uint8_t mac[6]);

  // BaseSerialInterface methods
  void enable() override;
  void disable() override;
  bool isEnabled() const override { return _isEnabled; }

  bool isConnected() const override;
  bool isWriteBusy() const override;

  size_t writeFrame(const uint8_t src[], size_t len) override;
  size_t checkRecvFrame(uint8_t dest[]) override;

  bool hasReceivedFrameHeader();
  void resetReceivedFrameHeader();
};

#if ETH_DEBUG_LOGGING && ARDUINO
  #include <Arduino.h>
  #define ETH_DEBUG_PRINT(F, ...) Serial.printf("ETH: " F, ##__VA_ARGS__)
  #define ETH_DEBUG_PRINTLN(F, ...) Serial.printf("ETH: " F "\n", ##__VA_ARGS__)
#else
  #define ETH_DEBUG_PRINT(...) {}
  #define ETH_DEBUG_PRINTLN(...) {}
#endif
