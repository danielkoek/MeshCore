// Water Scheduler – main.cpp
//
// A MeshCore sensor node that controls a water system via a relay on GPIO 2
// (D1 on XIAO S3).  It:
//   • reads an INA3221 power monitor (auto-detected by EnvironmentSensorManager)
//   • uses a DS3231 RTC for accurate timekeeping (auto-detected by AutoDiscoverRTCClock)
//   • exposes a weekly relay schedule via CLI commands over the mesh
//   • supports a manual override (on / off / auto) that survives reboots
//
// CLI commands (send as a chat message to this node after logging in):
//   relay on                  – force relay ON  (persists)
//   relay off                 – force relay OFF (persists)
//   relay auto                – return to schedule (persists)
//   relay status              – report current relay state + mode + entry count
//   schedule add daily 06:00 on  – every day at 06:00 turn ON
//   schedule add daily 22:00 off – every day at 22:00 turn OFF
//   schedule add 1 08:00 on      – Monday at 08:00 turn ON
//   schedule del 0               – delete entry #0
//   schedule list                – list all schedule entries
//   schedule clear               – delete all entries
//
// The same commands also work on the serial terminal (115200 baud).
// INA3221 telemetry is reported automatically via the sensor framework.

#include "SensorMesh.h"    // from examples/simple_sensor/
#include "WaterScheduler.h"

// ---------------------------------------------------------------------------
// MyMesh – extends SensorMesh with water-scheduler behaviour
// ---------------------------------------------------------------------------
class MyMesh : public SensorMesh {
public:
  MyMesh(mesh::MainBoard& board, mesh::Radio& radio, mesh::MillisecondClock& ms,
         mesh::RNG& rng, mesh::RTCClock& rtc, mesh::MeshTables& tables)
    : SensorMesh(board, radio, ms, rng, rtc, tables) {}

  // Expose scheduler loop so main loop() can call it
  void loopScheduler() {
    _scheduler.loop(getRTCClock());
  }

  // Expose scheduler begin after filesystem is ready
  void beginScheduler(FILESYSTEM* fs) {
    _scheduler.begin(fs, getRTCClock(), PIN_RELAY);
  }

  bool getRelayState() const { return _scheduler.getRelayState(); }

protected:
  /* ======================= custom sensor / relay logic =================== */

  WaterScheduler _scheduler;

  // Called by SensorMesh on each periodic sensor read cycle.
  // The INA3221 channels are reported automatically by EnvironmentSensorManager.
  // Use this hook for alerting or time-series recording if needed.
  void onSensorDataRead() override {
    // Nothing extra needed — relay state is queryable via "relay status" command.
  }

  // Not recording time series for this application; return 0 series.
  int querySeriesData(uint32_t /*start_secs_ago*/, uint32_t /*end_secs_ago*/,
                      MinMaxAvg* /*dest*/, int /*max_num*/) override {
    return 0;
  }

  // Intercept custom CLI commands before SensorMesh handles them.
  bool handleCustomCommand(uint32_t sender_timestamp,
                           char* command, char* reply) override {
    return _scheduler.handleCommand(sender_timestamp, command, reply);
  }

  /* ======================================================================= */
};

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
StdRNG          fast_rng;
SimpleMeshTables tables;

MyMesh the_mesh(board, radio_driver, *new ArduinoMillis(), fast_rng, rtc_clock, tables);

static char command[160];

void halt() { while (1) ; }

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  board.begin();

  if (!radio_init()) {
    MESH_DEBUG_PRINTLN("Radio init failed!");
    halt();
  }

  fast_rng.begin(radio_driver.getRngSeed());

  // Mount filesystem and load identity
  SPIFFS.begin(true);
  FILESYSTEM* fs = &SPIFFS;

  IdentityStore store(SPIFFS, "/identity");
  if (!store.load("_main", the_mesh.self_id)) {
    MESH_DEBUG_PRINTLN("Generating new keypair");
    the_mesh.self_id = radio_new_identity();
    int count = 0;
    while (count < 10 &&
           (the_mesh.self_id.pub_key[0] == 0x00 ||
            the_mesh.self_id.pub_key[0] == 0xFF)) {
      the_mesh.self_id = radio_new_identity();
      count++;
    }
    store.save("_main", the_mesh.self_id);
  }

  Serial.print("Water Scheduler ID: ");
  mesh::Utils::printHex(Serial, the_mesh.self_id.pub_key, PUB_KEY_SIZE);
  Serial.println();

  command[0] = 0;

  // Initialise environment sensors (INA3221, BME280, etc. – whatever is present)
  sensors.begin();

  // Initialise the scheduler (loads schedule + override from SPIFFS)
  the_mesh.beginScheduler(fs);

  the_mesh.begin(fs);

  // Broadcast initial advertisement
#if ENABLE_ADVERT_ON_BOOT == 1
  the_mesh.sendSelfAdvertisement(16000, false);
#endif

  Serial.println("Water Scheduler ready.");
  Serial.println("Commands: relay on|off|auto|status  |  schedule add|del|list|clear");
}

// ---------------------------------------------------------------------------
// loop()
// ---------------------------------------------------------------------------
void loop() {
  // Read serial commands
  int len = strlen(command);
  while (Serial.available() && len < (int)sizeof(command) - 1) {
    char c = Serial.read();
    if (c != '\n') {
      command[len++] = c;
      command[len]   = 0;
      Serial.print(c);
    }
    if (c == '\r') break;
  }
  if (len == (int)sizeof(command) - 1) {
    command[sizeof(command) - 1] = '\r';   // force flush
  }

  if (len > 0 && command[len - 1] == '\r') {
    Serial.print('\n');
    command[len - 1] = 0;   // strip CR
    char reply[160];
    reply[0] = 0;
    the_mesh.handleCommand(0, command, reply);
    if (reply[0]) {
      Serial.print("  -> ");
      Serial.println(reply);
    }
    command[0] = 0;
  }

  the_mesh.loop();
  the_mesh.loopScheduler();   // check relay schedule every minute
  sensors.loop();
  rtc_clock.tick();
}
