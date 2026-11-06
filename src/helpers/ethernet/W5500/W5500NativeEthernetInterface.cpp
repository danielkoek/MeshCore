#include "W5500NativeEthernetInterface.h"

#if defined(ETHERNET_ENABLED) && defined(ETHERNET_USE_W5500_NATIVE)

#if defined(ETHERNET_SHARE_RADIO_SPI)
  #include <target.h>          // declares: extern SPIClass radio_spi;
  #define ETH_SPI_BUS radio_spi
#else
  #define ETH_SPI_BUS SPI
#endif

static volatile bool eth_has_ip = false;

static void onEthEvent(arduino_event_id_t event, arduino_event_info_t info) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      ETHERNET_DEBUG_PRINTLN("Started");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      ETHERNET_DEBUG_PRINTLN("Link up");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      ETHERNET_DEBUG_PRINTLN("IP: %s, listening on TCP port %d", ETH.localIP().toString().c_str(), ETHERNET_TCP_PORT);
      eth_has_ip = true;
      break;
    case ARDUINO_EVENT_ETH_LOST_IP:
    case ARDUINO_EVENT_ETH_DISCONNECTED:
    case ARDUINO_EVENT_ETH_STOP:
      ETHERNET_DEBUG_PRINTLN("Link/IP lost");
      eth_has_ip = false;
      break;
    default:
      break;
  }
}

bool W5500NativeEthernetInterface::begin() {
#if defined(ETHERNET_SHARE_RADIO_SPI)
  // The radio already started this bus. Hand ETH the radio's own SPIClass: a second
  // SPIClass on the same host re-attaches the pins and stops the bus under the radio,
  // after which the W5500 driver stalls.
#elif defined(ETHERNET_SPI_SCK) && defined(ETHERNET_SPI_MISO) && defined(ETHERNET_SPI_MOSI)
  SPI.begin(ETHERNET_SPI_SCK, ETHERNET_SPI_MISO, ETHERNET_SPI_MOSI);
#endif
  Network.onEvent(onEthEvent);
  _hw_ok = ETH.begin(ETH_PHY_W5500, 1, ETHERNET_CS_PIN, ETHERNET_INT_PIN,
                     ETHERNET_RST_PIN, ETH_SPI_BUS, ETHERNET_SPI_FREQ_MHZ);
  if (!_hw_ok) {
    ETHERNET_DEBUG_PRINTLN("begin() failed (W5500 not found?)");
    return false;
  }
#if defined(ETHERNET_STATIC_IP) && defined(ETHERNET_STATIC_GATEWAY) && defined(ETHERNET_STATIC_SUBNET) && defined(ETHERNET_STATIC_DNS)
  ETH.config(IPAddress(ETHERNET_STATIC_IP), IPAddress(ETHERNET_STATIC_GATEWAY),
             IPAddress(ETHERNET_STATIC_SUBNET), IPAddress(ETHERNET_STATIC_DNS));
#endif
  return true;
}

int W5500NativeEthernetInterface::available() {
  return client.available();
}

int W5500NativeEthernetInterface::read() {
  return client.read();
}

size_t W5500NativeEthernetInterface::write(const uint8_t *buf, size_t size) {
  return client.write(buf, size);
}

bool W5500NativeEthernetInterface::isConnected() const {
  return _isConnected;
}

void W5500NativeEthernetInterface::loop() {
  if (_hw_ok && eth_has_ip) {
    if (!_server_started) {   // lwIP keeps the listener alive across link drops
      server.begin();
      server.setNoDelay(true);
      _server_started = true;
    }
    NetworkClient newClient = server.accept();   // accept() only returns newly-accepted sockets
    if (newClient) {
      ETHERNET_DEBUG_PRINTLN("client connected from %s", newClient.remoteIP().toString().c_str());
      if (client) client.stop();
      client = newClient;
      client.setNoDelay(true);
      onClientConnected();
    }
  }
  _isConnected = client.connected();
}

#endif
