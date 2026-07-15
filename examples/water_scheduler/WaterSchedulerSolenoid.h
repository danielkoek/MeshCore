#pragma once

#include <Arduino.h>
#include <Mesh.h>
#include <helpers/IdentityStore.h>

#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  #include <InternalFileSystem.h>
#elif defined(RP2040_PLATFORM)
  #include <LittleFS.h>
#elif defined(ESP32)
  #include <SPIFFS.h>
#endif

#ifndef MOTOR_I2C_ADDR
  #define MOTOR_I2C_ADDR 0x14   // TB6612FNG default I2C address
#endif
// Solenoid is driven on both channels in parallel (matches validated .ino).
// Single-channel wiring may replace this later.
#ifndef MOTOR_CHANNEL_A
  #define MOTOR_CHANNEL_A 0  // MOTOR_CHA
#endif
#ifndef MOTOR_CHANNEL_B
  #define MOTOR_CHANNEL_B 1  // MOTOR_CHB
#endif

#define MAX_SCHEDULE_ENTRIES      16
#define SCHED_FILE                "/solenoid_sched"
#define OVERRIDE_FILE             "/solenoid_ovrd"
#define OVERRIDE_DURATION_SECS    1800
#define SOLENOID_PULSE_DURATION   500    // milliseconds to drive motor
#define SOLENOID_SPEED            255    // Full speed (0-255)

// day_of_week: 0=Sun, 1=Mon, 2=Tue, 3=Wed, 4=Thu, 5=Fri, 6=Sat, 7=Daily
struct ScheduleEntry {
  uint8_t day_of_week;  // 0-6 = specific day, 7 = every day
  uint8_t hour;         // 0-23
  uint8_t minute;       // 0-59
  uint8_t action;       // 0 = CLOSE, 1 = OPEN
  uint8_t enabled;      // 1 = active
};

enum OverrideMode : uint8_t {
  OVERRIDE_AUTO  = 0,   // follow the schedule
  OVERRIDE_OPEN  = 1,   // force solenoid open
  OVERRIDE_CLOSE = 2    // force solenoid closed
};

// ---------------------------------------------------------------------------
// WaterSchedulerSolenoid
//
// Manages a weekly open/close schedule for a latching solenoid valve.
// Uses TB6612FNG motor driver to send directional pulses.
// State (schedule entries + override) is persisted to flash.
//
// CLI commands (via handleCommand):
//   solenoid open              - force solenoid OPEN (persists override)
//   solenoid close             - force solenoid CLOSED (persists override)
//   solenoid auto              - return to schedule (persists mode)
//   solenoid status            - report current state
//   schedule add <day|daily> <HH:MM> <open|close>
//                         - add an entry  (day: 0=Sun..6=Sat, or 'daily')
//   schedule del <index>  - delete entry by index
//   schedule list         - list all entries
//   schedule clear        - remove all entries
// ---------------------------------------------------------------------------
class WaterSchedulerSolenoid {
public:
  WaterSchedulerSolenoid() : _fs(nullptr),
                             _count(0), _rtc(nullptr), _override(OVERRIDE_AUTO),
                             _override_expiry_unix(0), _override_expiry_millis(0),
                             _solenoid_open(false), _last_minute(-1),
                             _pulse_end_millis(0), _pulse_active(false) {}

  // Call once after filesystem is mounted.
  void begin(FILESYSTEM* fs, mesh::RTCClock* rtc);

  // Call every loop iteration, passing the shared RTC clock.
  void loop(mesh::RTCClock* rtc);

  // Handle a CLI command string. Returns true if the command was consumed.
  // sender_timestamp == 0 when called from the serial terminal.
  bool handleCommand(uint32_t sender_timestamp, char* command, char* reply);

  bool        getSolenoidState() const { return _solenoid_open; }
  OverrideMode getOverrideMode()  const { return _override; }

private:
  FILESYSTEM*   _fs;
  ScheduleEntry _entries[MAX_SCHEDULE_ENTRIES];
  int           _count;
  mesh::RTCClock* _rtc;
  OverrideMode  _override;
  uint32_t      _override_expiry_unix;
  uint32_t      _override_expiry_millis;
  bool          _solenoid_open;
  int           _last_minute;
  uint32_t      _pulse_end_millis;   // when current pulse should stop
  bool          _pulse_active;       // pulse in progress

  void motorDrive(bool open);                     // send pulse in specified direction
  void motorStop();                               // stop motor
  void updatePulse();                             // check if pulse should stop
  void setSolenoid(bool open);                    // command solenoid to open/close
  void setOverrideExpiry();
  void expireOverrideIfNeeded();
  void checkSchedule(uint8_t dow, uint8_t hour, uint8_t minute);
  void restoreState(mesh::RTCClock* rtc);
  void saveSchedule();
  void loadSchedule();
  void saveOverride();
  void loadOverride();

  static const char* dayName(uint8_t dow);
};
