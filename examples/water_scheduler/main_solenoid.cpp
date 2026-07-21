// Water Scheduler with Latching Solenoid – main_solenoid.cpp
//
// A MeshCore sensor node for XIAO nRF52 that controls a latching solenoid valve
// using a TB6612FNG I2C motor driver. It:
//   • reads a DS3231 RTC for accurate timekeeping
//   • exposes a weekly solenoid schedule via CLI commands over the mesh
//   • supports a manual override (open / close / auto) that survives reboots
//   • sends directional motor pulses to the solenoid (500ms pulse = one toggle)
//
// CLI commands (send as a chat message to this node after logging in):
//   solenoid open                  – force solenoid OPEN  (persists)
//   solenoid close                 – force solenoid CLOSED (persists)
//   solenoid auto                  – return to schedule (persists)
//   solenoid status                – report current solenoid state + mode + entry count
//   schedule add daily 06:00 open  – every day at 06:00 open valve
//   schedule add daily 22:00 close – every day at 22:00 close valve
//   schedule add 1 08:00 open      – Monday at 08:00 open valve
//   schedule del 0                 – delete entry #0
//   schedule list                  – list all schedule entries
//   schedule clear                 – delete all entries
//
// The same commands also work on the serial terminal (115200 baud).
//
// Hardware:
//   - XIAO nRF52 microcontroller
//   - TB6612FNG motor driver module (I2C, default address 0x14)
//   - Latching solenoid valve connected to motor outputs (CH_A or CH_B)
//   - DS3231 RTC for timekeeping (optional, node still works with system time)
//
// Motor Driver (I2C):
//   - Positive speed → motor clockwise (OPEN solenoid)
//   - Negative speed → motor counter-clockwise (CLOSE solenoid)
//   - Pulse for SOLENOID_PULSE_DURATION ms (typically 500ms) then stop

#include "SensorMesh.h"                    // from examples/simple_sensor/
#include "WaterSchedulerSolenoid.h"
#include <Wire.h>
#include <helpers/ui/MomentaryButton.h>

// Debug helper – prints only when MESH_DEBUG is defined
#ifdef MESH_DEBUG
  #define SOL_DEBUG(msg)       do { Serial.print("[SOL] "); Serial.println(msg); } while(0)
  #define SOL_DEBUG2(a,b)      do { Serial.print("[SOL] "); Serial.print(a); Serial.println(b); } while(0)
#else
  #define SOL_DEBUG(msg)
  #define SOL_DEBUG2(a,b)
#endif

// ---------------------------------------------------------------------------
// MyMesh – extends SensorMesh with solenoid scheduler behaviour
// ---------------------------------------------------------------------------
class MyMesh : public SensorMesh {
public:
  MyMesh(mesh::MainBoard& board, mesh::Radio& radio, mesh::MillisecondClock& ms,
         mesh::RNG& rng, mesh::RTCClock& rtc, mesh::MeshTables& tables)
    : SensorMesh(board, radio, ms, rng, rtc, tables) {}

  void loopScheduler() {
    _scheduler.loop(getRTCClock());
  }

  void beginScheduler(FILESYSTEM* fs) {
    _scheduler.begin(fs, getRTCClock());
  }

  bool getSolenoidState() const { return _scheduler.getSolenoidState(); }

  void toggleSolenoid() {
    char cmd[20];
    char reply[160];
    reply[0] = 0;
    if (_scheduler.getSolenoidState()) {
      strcpy(cmd, "solenoid close");
    } else {
      strcpy(cmd, "solenoid open");
    }
    _scheduler.handleCommand(0, cmd, reply);
    SOL_DEBUG2("Button toggle: ", reply);
  }

protected:
  WaterSchedulerSolenoid _scheduler;

  void onSensorDataRead() override {
    // Custom logic for sensor reads if needed
  }

  int querySeriesData(uint32_t /*start_secs_ago*/, uint32_t /*end_secs_ago*/,
                      MinMaxAvg* /*dest*/, int /*max_num*/) override {
    return 0;
  }

  bool handleCustomCommand(uint32_t sender_timestamp,
                           char* command, char* reply) override {
    return _scheduler.handleCommand(sender_timestamp, command, reply);
  }
};

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
StdRNG          fast_rng;
SimpleMeshTables tables;

MyMesh the_mesh(board, radio_driver, *new ArduinoMillis(), fast_rng, rtc_clock, tables);

#ifdef PIN_USER_BTN
MomentaryButton user_btn(PIN_USER_BTN, 1000, true, true);  // 1s long-press, active-LOW, pullup
#endif

static char command[160];

void halt() { while (1) ; }

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  board.begin();   // also calls Wire.setPins() + Wire.begin() for remapped I2C
  SOL_DEBUG("Board init done");

  if (!radio_init()) {
    SOL_DEBUG("ERROR: Radio init failed!");
    halt();
  }
  SOL_DEBUG("Radio init done");

  fast_rng.begin(radio_driver.getRngSeed());

  // Mount filesystem and load identity
  InternalFS.begin();
  FILESYSTEM* fs = &InternalFS;
  SOL_DEBUG("Filesystem mounted");

  IdentityStore store(InternalFS, "");   // nRF52 uses empty path (no subdirectory)
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

  Serial.print("Solenoid Scheduler ID: ");
  mesh::Utils::printHex(Serial, the_mesh.self_id.pub_key, PUB_KEY_SIZE);
  Serial.println();

  command[0] = 0;

  // NOTE: Wire.begin() already called by board.begin() with remapped pins (D6=SCL, D7=SDA).
  // Do NOT call Wire.begin() again here – it would reset to default pins (D4/D5).

  // Scan I2C bus for motor driver
#ifdef MESH_DEBUG
  SOL_DEBUG("I2C scan for motor driver...");
  Wire.beginTransmission(MOTOR_I2C_ADDR);
  uint8_t i2c_err = Wire.endTransmission();
  if (i2c_err == 0) {
    SOL_DEBUG2("Motor driver found at 0x", String(MOTOR_I2C_ADDR, HEX));
  } else {
    SOL_DEBUG2("Motor driver NOT found at 0x", String(MOTOR_I2C_ADDR, HEX));
    SOL_DEBUG2("  I2C error code: ", i2c_err);
  }
#endif

  // Initialise environment sensors (if any)
  sensors.begin();
  SOL_DEBUG("Sensors init done");

  // Initialise the solenoid scheduler (uses direct I2C commands)
  the_mesh.beginScheduler(fs);
  SOL_DEBUG("Scheduler init done");

  the_mesh.begin(fs);
  SOL_DEBUG("Mesh begin done");

#if ENABLE_ADVERT_ON_BOOT == 1
  the_mesh.sendSelfAdvertisement(16000, false);
#endif

#ifdef PIN_USER_BTN
  user_btn.begin();
  SOL_DEBUG("User button init done");
#endif

  Serial.println("Solenoid Scheduler ready.");
  Serial.println("Commands: solenoid open|close|auto|status  |  schedule add|del|list|clear");
  Serial.println("Button: 1x=flood advert, 2x=toggle solenoid");
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
  the_mesh.loopScheduler();   // check solenoid schedule every minute
  sensors.loop();
  rtc_clock.tick();

#ifdef PIN_USER_BTN
  int btn_ev = user_btn.check();
  if (btn_ev == BUTTON_EVENT_CLICK) {
    SOL_DEBUG("Button: single press -> flood advert");
    the_mesh.sendSelfAdvertisement(500, true);   // flood so the app can find us
    Serial.println("  -> Flood advert sent");
  } else if (btn_ev == BUTTON_EVENT_DOUBLE_CLICK) {
    SOL_DEBUG("Button: double press -> toggle solenoid");
    the_mesh.toggleSolenoid();
    Serial.print("  -> Solenoid toggled: ");
    Serial.println(the_mesh.getSolenoidState() ? "OPEN" : "CLOSED");
  }
#endif
}
