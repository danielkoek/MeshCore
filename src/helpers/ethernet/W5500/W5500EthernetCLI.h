#pragma once

#ifdef ETHERNET_ENABLED

/*
 * Text CLI over TCP for repeater/room-server style firmwares, using a WIZnet W5500
 * (ESP32). Same API as helpers/nrf52/EthernetCLI.h, but fully cooperative (no RTOS
 * task) so the W5500 can safely share the SPI bus with the LoRa radio (separate CS).
 *
 * Build flags:
 *   ETHERNET_CS_PIN                                 W5500 chip-select (required)
 *   ETHERNET_SPI_SCK / _MISO / _MOSI                SPI pins (optional, else bus must already be begun)
 *   ETHERNET_TCP_PORT                               default 23
 *   ETHERNET_DHCP_TIMEOUT_MS, ETHERNET_DHCP_RETRY_MS
 *   ETHERNET_STATIC_IP/_GATEWAY/_SUBNET/_DNS        optional static config
 */

#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <esp_mac.h>

#ifndef ETHERNET_CS_PIN
  #error "W5500EthernetCLI requires ETHERNET_CS_PIN"
#endif
#ifndef ETHERNET_TCP_PORT
  #define ETHERNET_TCP_PORT 23  // telnet port for CLI access
#endif
#ifndef ETHERNET_CLI_BANNER
  #define ETHERNET_CLI_BANNER "MeshCore CLI"
#endif
#ifndef ETHERNET_DHCP_TIMEOUT_MS
  #define ETHERNET_DHCP_TIMEOUT_MS   5000    // max time to block waiting on a DHCP lease
#endif
#ifndef ETHERNET_DHCP_RETRY_MS
  #define ETHERNET_DHCP_RETRY_MS    30000    // how often to re-attempt DHCP while unconfigured
#endif

// On some cores (e.g. ESP32 arduino core 2.x) the Server base class declares
// begin(uint16_t) as pure virtual, which leaves EthernetServer abstract.
class W5500CLIServer : public EthernetServer {
public:
  explicit W5500CLIServer(uint16_t port) : EthernetServer(port) {}
  using EthernetServer::begin;
  void begin(uint16_t port) { EthernetServer::begin(); }
};

static W5500CLIServer ethernet_server(ETHERNET_TCP_PORT);
static EthernetClient ethernet_client;
static bool ethernet_running = false;
static bool ethernet_hw_missing = false;
static unsigned long ethernet_next_attempt = 0;
static uint8_t ethernet_mac[6];

static void ethernet_start_task() {
#if defined(ETHERNET_SPI_SCK) && defined(ETHERNET_SPI_MISO) && defined(ETHERNET_SPI_MOSI)
  SPI.begin(ETHERNET_SPI_SCK, ETHERNET_SPI_MISO, ETHERNET_SPI_MOSI);   // may be shared with the radio (separate CS)
#endif
  esp_read_mac(ethernet_mac, ESP_MAC_ETH);   // factory-assigned MAC reserved for Ethernet
  Serial.printf("ETH: MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
      ethernet_mac[0], ethernet_mac[1], ethernet_mac[2], ethernet_mac[3], ethernet_mac[4], ethernet_mac[5]);
  Ethernet.init(ETHERNET_CS_PIN);
  ethernet_next_attempt = millis();
}

// brings the network up (and keeps it up), cooperatively from the main loop
static bool ethernet_ensure_network() {
  if (ethernet_hw_missing) return false;

  if (ethernet_running) {
    Ethernet.maintain();   // renew/rebind DHCP lease when due (cheap otherwise)
    return true;
  }

  if ((long)(millis() - ethernet_next_attempt) < 0) return false;
  ethernet_next_attempt = millis() + ETHERNET_DHCP_RETRY_MS;

  if (Ethernet.linkStatus() == LinkOFF) {   // no cable -> don't block on DHCP
    Serial.println("ETH: Cable not connected, will retry");
    return false;
  }

#if defined(ETHERNET_STATIC_IP) && defined(ETHERNET_STATIC_GATEWAY) && defined(ETHERNET_STATIC_SUBNET) && defined(ETHERNET_STATIC_DNS)
  IPAddress ip(ETHERNET_STATIC_IP);
  IPAddress gateway(ETHERNET_STATIC_GATEWAY);
  IPAddress subnet(ETHERNET_STATIC_SUBNET);
  IPAddress dns(ETHERNET_STATIC_DNS);
  Ethernet.begin(ethernet_mac, ip, dns, gateway, subnet);
#else
  Serial.println("ETH: Attempting DHCP...");
  if (Ethernet.begin(ethernet_mac, ETHERNET_DHCP_TIMEOUT_MS, 2000) == 0) {
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      ethernet_hw_missing = true;   // no point retrying
      Serial.println("ETH: Hardware not found, giving up");
    } else {
      Serial.println("ETH: DHCP failed, will retry");
    }
    return false;
  }
#endif

  IPAddress ip = Ethernet.localIP();
  Serial.printf("ETH: IP: %u.%u.%u.%u\n", ip[0], ip[1], ip[2], ip[3]);
  Serial.printf("ETH: Listening on TCP port %d\n", ETHERNET_TCP_PORT);
  ethernet_server.begin();
  ethernet_running = true;
  return true;
}

// Format ethernet status into reply buffer. Returns true if command was handled.
static bool ethernet_handle_command(const char* command, char* reply) {
  if (strcmp(command, "eth.status") == 0) {
    if (!ethernet_running) {
      strcpy(reply, ethernet_hw_missing ? "ETH: no hardware" : "ETH: not connected");
    } else {
      IPAddress ip = Ethernet.localIP();
      sprintf(reply, "ETH: %u.%u.%u.%u:%d", ip[0], ip[1], ip[2], ip[3], ETHERNET_TCP_PORT);
    }
    return true;
  }
  return false;
}

// Call from loop() to maintain DHCP and check for new clients
static void ethernet_loop_maintain() {
  if (!ethernet_ensure_network()) return;

  // accept() (not available()) so we only see newly-accepted sockets
  auto newClient = ethernet_server.accept();
  if (newClient) {
    if (ethernet_client) ethernet_client.stop();
    ethernet_client = newClient;
    IPAddress ip = ethernet_client.remoteIP();
    Serial.printf("ETH: Client connected from %u.%u.%u.%u\n", ip[0], ip[1], ip[2], ip[3]);
    ethernet_client.println(ETHERNET_CLI_BANNER);
  }
}

// Read a line from the Ethernet client into the command buffer.
// Returns true when a complete line is ready to process (command is null-terminated).
// The caller should process the command and then reset ethernet_command[0] = 0.
static bool ethernet_read_line(char* ethernet_command, size_t buf_size) {
  if (!ethernet_running || !ethernet_client || !ethernet_client.connected()) return false;

  int elen = strlen(ethernet_command);
  while (ethernet_client.available() && elen < (int)buf_size - 1) {
    char c = ethernet_client.read();
    if (c == '\n' && elen == 0) continue;  // ignore leading LF (from CR+LF)
    if (c == '\r' || c == '\n') { ethernet_command[elen++] = '\r'; break; }
    ethernet_command[elen++] = c;
    ethernet_command[elen] = 0;
  }
  if (elen == (int)buf_size - 1) {  // buffer full: treat as a completed line
    ethernet_command[buf_size - 2] = '\r';
    ethernet_command[buf_size - 1] = 0;
  }

  if (elen > 0 && ethernet_command[elen - 1] == '\r') {
    ethernet_command[elen - 1] = 0;
    ethernet_client.println();
    return true;
  }
  return false;
}

// Send a reply to the Ethernet client
static void ethernet_send_reply(const char* reply) {
  if (reply[0]) {
    ethernet_client.print("  -> "); ethernet_client.println(reply);
  }
}

#endif // ETHERNET_ENABLED
