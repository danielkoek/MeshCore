#pragma once

#ifdef ETHERNET_ENABLED

/*
 * Text CLI over TCP for repeater/room-server style firmwares, using the ESP32
 * Arduino core 3.x native Ethernet driver (ETH.h) with a WIZnet W5500 SPI module.
 * Same API as helpers/nrf52/EthernetCLI.h, backed by the real lwIP stack.
 *
 * Requires arduino-esp32 core >= 3.1 (e.g. the pioarduino platform). The W5500
 * may share the SPI bus with the LoRa radio (separate CS): the driver is handed
 * the global SPI object and every transfer runs inside an SPI transaction, so
 * the two devices interleave safely.
 *
 * Build flags:
 *   ETHERNET_CS_PIN                            W5500 chip-select (required)
 *   ETHERNET_INT_PIN                           W5500 INT gpio (default -1 = polling mode)
 *   ETHERNET_RST_PIN                           W5500 reset gpio (default -1 = not wired)
 *   ETHERNET_SPI_SCK / _MISO / _MOSI           SPI pins (optional, else bus must already be begun)
 *   ETHERNET_SPI_FREQ_MHZ                      default 20
 *   ETHERNET_TCP_PORT                          default 23
 *   ETHERNET_STATIC_IP/_GATEWAY/_SUBNET/_DNS   optional static config
 */

#include <Arduino.h>
#include <SPI.h>
#include <ETH.h>

#ifndef ETHERNET_CS_PIN
  #error "W5500NativeEthernetCLI requires ETHERNET_CS_PIN"
#endif
#ifndef ETHERNET_INT_PIN
  #define ETHERNET_INT_PIN -1   // -1 = INT not wired, driver polls the W5500 instead
#endif
#ifndef ETHERNET_RST_PIN
  #define ETHERNET_RST_PIN -1
#endif
#ifndef ETHERNET_SPI_FREQ_MHZ
  #define ETHERNET_SPI_FREQ_MHZ 20
#endif
#ifndef ETHERNET_TCP_PORT
  #define ETHERNET_TCP_PORT 23  // telnet port for CLI access
#endif
#ifndef ETHERNET_CLI_BANNER
  #define ETHERNET_CLI_BANNER "MeshCore CLI"
#endif

static NetworkServer ethernet_server(ETHERNET_TCP_PORT);
static NetworkClient ethernet_client;
static bool ethernet_hw_ok = false;          // ETH.begin() succeeded
static volatile bool ethernet_has_ip = false;
static bool ethernet_server_started = false;

static void ethernet_on_event(arduino_event_id_t event, arduino_event_info_t info) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("ETH: Started");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:
      Serial.println("ETH: Link up");
      break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.printf("ETH: IP: %s\n", ETH.localIP().toString().c_str());
      Serial.printf("ETH: Listening on TCP port %d\n", ETHERNET_TCP_PORT);
      ethernet_has_ip = true;
      break;
    case ARDUINO_EVENT_ETH_LOST_IP:
      Serial.println("ETH: Lost IP");
      ethernet_has_ip = false;
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH: Link down");
      ethernet_has_ip = false;
      break;
    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("ETH: Stopped");
      ethernet_has_ip = false;
      break;
    default:
      break;
  }
}

static void ethernet_start_task() {
#if defined(ETHERNET_SPI_SCK) && defined(ETHERNET_SPI_MISO) && defined(ETHERNET_SPI_MOSI)
  SPI.begin(ETHERNET_SPI_SCK, ETHERNET_SPI_MISO, ETHERNET_SPI_MOSI);   // may be shared with the radio (separate CS)
#endif
  Network.onEvent(ethernet_on_event);
  ethernet_hw_ok = ETH.begin(ETH_PHY_W5500, 1, ETHERNET_CS_PIN, ETHERNET_INT_PIN,
                             ETHERNET_RST_PIN, SPI, ETHERNET_SPI_FREQ_MHZ);
  if (!ethernet_hw_ok) {
    Serial.println("ETH: begin() failed (W5500 not found?)");
    return;
  }
  Serial.printf("ETH: MAC: %s\n", ETH.macAddress().c_str());
#if defined(ETHERNET_STATIC_IP) && defined(ETHERNET_STATIC_GATEWAY) && defined(ETHERNET_STATIC_SUBNET) && defined(ETHERNET_STATIC_DNS)
  ETH.config(IPAddress(ETHERNET_STATIC_IP), IPAddress(ETHERNET_STATIC_GATEWAY),
             IPAddress(ETHERNET_STATIC_SUBNET), IPAddress(ETHERNET_STATIC_DNS));
#endif
  // DHCP, link detection and retries are handled by the driver/lwIP from here on.
}

// Format ethernet status into reply buffer. Returns true if command was handled.
static bool ethernet_handle_command(const char* command, char* reply) {
  if (strcmp(command, "eth.status") == 0) {
    if (!ethernet_hw_ok) {
      strcpy(reply, "ETH: no hardware");
    } else if (!ethernet_has_ip) {
      strcpy(reply, ETH.linkUp() ? "ETH: waiting for IP" : "ETH: not connected");
    } else {
      sprintf(reply, "ETH: %s:%d", ETH.localIP().toString().c_str(), ETHERNET_TCP_PORT);
    }
    return true;
  }
  return false;
}

// Call from loop() to start the server once the network is up and check for new clients
static void ethernet_loop_maintain() {
  if (!ethernet_has_ip) return;

  if (!ethernet_server_started) {   // lwIP keeps the listener alive across link drops
    ethernet_server.begin();
    ethernet_server.setNoDelay(true);
    ethernet_server_started = true;
  }

  // accept() (not available()) so we only see newly-accepted sockets
  NetworkClient newClient = ethernet_server.accept();
  if (newClient) {
    if (ethernet_client) ethernet_client.stop();
    ethernet_client = newClient;
    Serial.printf("ETH: Client connected from %s\n", ethernet_client.remoteIP().toString().c_str());
    ethernet_client.println(ETHERNET_CLI_BANNER);
  }
}

// Read a line from the Ethernet client into the command buffer.
// Returns true when a complete line is ready to process (command is null-terminated).
// The caller should process the command and then reset ethernet_command[0] = 0.
static bool ethernet_read_line(char* ethernet_command, size_t buf_size) {
  if (!ethernet_client || !ethernet_client.connected()) return false;

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
