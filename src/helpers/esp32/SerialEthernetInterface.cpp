// NOTE: whole file is guarded, as some envs compile helpers/esp32/*.cpp via
// wildcard without having the Ethernet library as a dependency
#ifdef USE_ETH_W5500

#include "SerialEthernetInterface.h"

#ifndef ETH_DHCP_TIMEOUT_MS
  #define ETH_DHCP_TIMEOUT_MS   5000    // max time to block waiting on a DHCP lease
#endif
#ifndef ETH_DHCP_RETRY_MS
  #define ETH_DHCP_RETRY_MS    30000    // how often to re-attempt DHCP while unconfigured
#endif

void SerialEthernetInterface::begin(uint16_t port, int cs_pin, const uint8_t mac[6]) {
  memcpy(_mac, mac, 6);
  Ethernet.init(cs_pin);   // W5500 chip-select (SPI bus itself is begun by the app)
  _server = new MeshEthernetServer(port);
  next_dhcp_attempt = millis();
  ensureNetwork();   // first attempt now; fast when cable + DHCP server present
}

// brings the network up (and keeps it up), cooperatively from the main loop
bool SerialEthernetInterface::ensureNetwork() {
  if (hw_missing) return false;

  if (net_ready) {
    Ethernet.maintain();   // renew/rebind DHCP lease when due (cheap otherwise)
    return true;
  }

  if ((long)(millis() - next_dhcp_attempt) < 0) return false;
  next_dhcp_attempt = millis() + ETH_DHCP_RETRY_MS;

  if (Ethernet.linkStatus() == LinkOFF) {   // no cable -> don't block on DHCP
    ETH_DEBUG_PRINTLN("link is down, waiting for cable");
    return false;
  }

  if (Ethernet.begin((uint8_t *)_mac, ETH_DHCP_TIMEOUT_MS, 2000) == 0) {
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      hw_missing = true;   // no point retrying
      ETH_DEBUG_PRINTLN("W5500 not detected on SPI bus!");
    } else {
      ETH_DEBUG_PRINTLN("DHCP failed, will retry");
    }
    return false;
  }

  _server->begin();
  net_ready = true;
#if ETH_DEBUG_LOGGING
  Serial.print("ETH: IP address: ");
  Serial.println(Ethernet.localIP());
#endif
  return true;
}

// ---------- public methods
void SerialEthernetInterface::enable() {
  if (_isEnabled) return;

  _isEnabled = true;
  clearBuffers();
}

void SerialEthernetInterface::disable() {
  _isEnabled = false;
}

size_t SerialEthernetInterface::writeFrame(const uint8_t src[], size_t len) {
  if (len > MAX_FRAME_SIZE) {
    ETH_DEBUG_PRINTLN("writeFrame(), frame too big, len=%d\n", len);
    return 0;
  }

  if (deviceConnected && len > 0) {
    if (send_queue_len >= FRAME_QUEUE_SIZE) {
      ETH_DEBUG_PRINTLN("writeFrame(), send_queue is full!");
      return 0;
    }

    send_queue[send_queue_len].len = len;  // add to send queue
    memcpy(send_queue[send_queue_len].buf, src, len);
    send_queue_len++;

    return len;
  }
  return 0;
}

bool SerialEthernetInterface::isWriteBusy() const {
  return false;
}

bool SerialEthernetInterface::hasReceivedFrameHeader() {
  return received_frame_header.type != 0 && received_frame_header.length != 0;
}

void SerialEthernetInterface::resetReceivedFrameHeader() {
  received_frame_header.type = 0;
  received_frame_header.length = 0;
}

size_t SerialEthernetInterface::checkRecvFrame(uint8_t dest[]) {
  if (!ensureNetwork()) return 0;

  // check if new client connected
  EthernetClient newClient = _server->accept();
  if (newClient) {

    // disconnect existing client
    deviceConnected = false;
    client.stop();

    // switch active connection to new client
    client = newClient;

    // forget received frame header
    resetReceivedFrameHeader();

  }

  if (client.connected()) {
    if (!deviceConnected) {
      ETH_DEBUG_PRINTLN("Got connection");
      deviceConnected = true;
    }
  } else {
    if (deviceConnected) {
      deviceConnected = false;
      ETH_DEBUG_PRINTLN("Disconnected");
    }
  }

  if (deviceConnected) {
    if (send_queue_len > 0) {   // first, check send queue

      _last_write = millis();
      int len = send_queue[0].len;

      uint8_t pkt[3+len]; // use same header as serial interface so client can delimit frames
      pkt[0] = '>';
      pkt[1] = (len & 0xFF);  // LSB
      pkt[2] = (len >> 8);    // MSB
      memcpy(&pkt[3], send_queue[0].buf, send_queue[0].len);
      client.write(pkt, 3 + len);
      send_queue_len--;
      for (int i = 0; i < send_queue_len; i++) {   // delete top item from queue
        send_queue[i] = send_queue[i + 1];
      }
    } else {

      // check if we are waiting for a frame header
      if(!hasReceivedFrameHeader()){

        // make sure we have received enough bytes for a frame header
        // 3 bytes frame header = (1 byte frame type) + (2 bytes frame length as unsigned 16-bit little endian)
        int frame_header_length = 3;
        if(client.available() >= frame_header_length){

          // read frame header
          client.readBytes(&received_frame_header.type, 1);
          client.readBytes((uint8_t*)&received_frame_header.length, 2);

        }

      }

      // check if we have received a frame header
      if(hasReceivedFrameHeader()){

        // make sure we have received enough bytes for the required frame length
        int available = client.available();
        int frame_type = received_frame_header.type;
        int frame_length = received_frame_header.length;
        if(frame_length > available){
          ETH_DEBUG_PRINTLN("Waiting for %d more bytes", frame_length - available);
          return 0;
        }

        // skip frames that are larger than MAX_FRAME_SIZE
        if(frame_length > MAX_FRAME_SIZE){
          ETH_DEBUG_PRINTLN("Skipping frame: length=%d is larger than MAX_FRAME_SIZE=%d", frame_length, MAX_FRAME_SIZE);
          while(frame_length > 0){
            uint8_t skip[1];
            int skipped = client.read(skip, 1);
            frame_length -= skipped;
          }
          resetReceivedFrameHeader();
          return 0;
        }

        // skip frames that are not expected type
        // '<' is 0x3c which indicates a frame sent from app to radio
        if(frame_type != '<'){
          ETH_DEBUG_PRINTLN("Skipping frame: type=0x%x is unexpected", frame_type);
          while(frame_length > 0){
            uint8_t skip[1];
            int skipped = client.read(skip, 1);
            frame_length -= skipped;
          }
          resetReceivedFrameHeader();
          return 0;
        }

        // read frame data to provided buffer
        client.readBytes(dest, frame_length);

        // ready for next frame
        resetReceivedFrameHeader();
        return frame_length;

      }

    }
  }

  return 0;
}

bool SerialEthernetInterface::isConnected() const {
  return deviceConnected;
}

#endif  // USE_ETH_W5500
