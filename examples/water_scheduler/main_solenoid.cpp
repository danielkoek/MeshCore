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
#include <Grove_Motor_Driver_TB6612FNG.h>

// ---------------------------------------------------------------------------
// MyMesh – extends SensorMesh with solenoid scheduler behaviour
// ---------------------------------------------------------------------------
class MyMesh : public SensorMesh {
public:
  MyMesh(mesh::MainBoard& board, mesh::Radio& radio, mesh::MillisecondClock& ms,
         mesh::RNG& rng, mesh::RTCClock& rtc, mesh::MeshTables& tables)
    : SensorMesh(board, radio, ms, rng, rtc, tables), _motor(nullptr) {}

  void loopScheduler() {
    _scheduler.loop(getRTCClock());
  }

  void beginScheduler(FILESYSTEM* fs, MotorDriver* motor) {
    _motor = motor;
    _scheduler.begin(fs, getRTCClock(), motor);
  }

  bool getSolenoidState() const { return _scheduler.getSolenoidState(); }

protected:
  WaterSchedulerSolenoid _scheduler;
  MotorDriver* _motor;

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

static char command[160];
static MotorDriver motor_driver;

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
  InternalFS.begin();
  FILESYSTEM* fs = &InternalFS;

  IdentityStore store(*fs, "/identity");
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

  // Initialise I2C motor driver
  Wire.begin();
  motor_driver.init(MOTOR_I2C_ADDR);
  motor_driver.notStandby();

  // Initialise environment sensors (if any)
  sensors.begin();

  // Initialise the solenoid scheduler with motor driver
  the_mesh.beginScheduler(fs, &motor_driver);

  the_mesh.begin(fs);

#if ENABLE_ADVERT_ON_BOOT == 1
  the_mesh.sendSelfAdvertisement(16000, false);
#endif

  Serial.println("Solenoid Scheduler ready.");
  Serial.println("Commands: solenoid open|close|auto|status  |  schedule add|del|list|clear");
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
}
