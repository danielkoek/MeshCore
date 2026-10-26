#include "W5500EthernetInterface.h"
#include <esp_mac.h>

#ifndef ETHERNET_CS_PIN
  #error "W5500EthernetInterface requires ETHERNET_CS_PIN"
#endif
#ifndef ETHERNET_DHCP_TIMEOUT_MS
  #define ETHERNET_DHCP_TIMEOUT_MS   5000    // max time to block waiting on a DHCP lease
#endif
#ifndef ETHERNET_DHCP_RETRY_MS
  #define ETHERNET_DHCP_RETRY_MS    30000    // how often to re-attempt DHCP while unconfigured
#endif

bool W5500EthernetInterface::begin() {
#if defined(ETHERNET_SPI_SCK) && defined(ETHERNET_SPI_MISO) && defined(ETHERNET_SPI_MOSI)
  SPI.begin(ETHERNET_SPI_SCK, ETHERNET_SPI_MISO, ETHERNET_SPI_MOSI);   // may be shared with the radio (separate CS)
#endif

  esp_read_mac(_mac, ESP_MAC_ETH);   // factory-assigned MAC reserved for Ethernet
  ETHERNET_DEBUG_PRINTLN(
      "Ethernet MAC: %02X:%02X:%02X:%02X:%02X:%02X",
      _mac[0], _mac[1], _mac[2], _mac[3], _mac[4], _mac[5]);

  Ethernet.init(ETHERNET_CS_PIN);
  next_dhcp_attempt = millis();
  return ensureNetwork();   // first attempt now; fast when cable + DHCP server present
}

// brings the network up (and keeps it up), cooperatively from the main loop
bool W5500EthernetInterface::ensureNetwork() {
  if (hw_missing) return false;

  if (net_ready) {
    Ethernet.maintain();   // renew/rebind DHCP lease when due (cheap otherwise)
    return true;
  }

  if ((long)(millis() - next_dhcp_attempt) < 0) return false;
  next_dhcp_attempt = millis() + ETHERNET_DHCP_RETRY_MS;

  if (Ethernet.linkStatus() == LinkOFF) {   // no cable -> don't block on DHCP
    ETHERNET_DEBUG_PRINTLN("link is down, waiting for cable");
    return false;
  }

  // Use static IP if build flags are defined, otherwise DHCP
#if defined(ETHERNET_STATIC_IP) && defined(ETHERNET_STATIC_GATEWAY) && defined(ETHERNET_STATIC_SUBNET) && defined(ETHERNET_STATIC_DNS)
  IPAddress ip(ETHERNET_STATIC_IP);
  IPAddress gateway(ETHERNET_STATIC_GATEWAY);
  IPAddress subnet(ETHERNET_STATIC_SUBNET);
  IPAddress dns(ETHERNET_STATIC_DNS);
  Ethernet.begin(_mac, ip, dns, gateway, subnet);
#else
  if (Ethernet.begin(_mac, ETHERNET_DHCP_TIMEOUT_MS, 2000) == 0) {
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      hw_missing = true;   // no point retrying
      ETHERNET_DEBUG_PRINTLN("W5500 not detected on SPI bus!");
    } else {
      ETHERNET_DEBUG_PRINTLN("DHCP failed, will retry");
    }
    return false;
  }
#endif

  ETHERNET_DEBUG_PRINT_IP("IP Address", Ethernet.localIP());
  ETHERNET_DEBUG_PRINT_IP("Subnet Mask", Ethernet.subnetMask());
  ETHERNET_DEBUG_PRINT_IP("Gateway", Ethernet.gatewayIP());

  server.begin();
  net_ready = true;
  ETHERNET_DEBUG_PRINTLN("listening on TCP port: %d", ETHERNET_TCP_PORT);
  return true;
}

int W5500EthernetInterface::available() {
  return client.available();
}

int W5500EthernetInterface::read() {
  return client.read();
}

size_t W5500EthernetInterface::write(const uint8_t *buf, size_t size) {
  return client.write(buf, size);
}

bool W5500EthernetInterface::isConnected() const {
  return _isConnected;
}

void W5500EthernetInterface::loop() {
  if (!ensureNetwork()) {
    _isConnected = false;
    return;
  }

  auto newClient = server.accept();
  if (newClient) {
    ETHERNET_DEBUG_PRINT_IP("New client accepted", newClient.remoteIP());
    if (client) {
      ETHERNET_DEBUG_PRINTLN("Closing previous client");
      client.stop();
    }
    client = newClient;
    onClientConnected();
  }

  _isConnected = client.connected();
}
