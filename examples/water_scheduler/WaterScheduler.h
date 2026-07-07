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

#ifndef PIN_RELAY
  #define PIN_RELAY 2    // D1 on XIAO S3 = GPIO 2
#endif

#define MAX_SCHEDULE_ENTRIES   16
#define SCHED_FILE             "/water_sched"
#define OVERRIDE_FILE          "/water_ovrd"
#define OVERRIDE_DURATION_SECS 1800

// day_of_week: 0=Sun, 1=Mon, 2=Tue, 3=Wed, 4=Thu, 5=Fri, 6=Sat, 7=Daily
struct ScheduleEntry {
  uint8_t day_of_week;  // 0-6 = specific day, 7 = every day
  uint8_t hour;         // 0-23
  uint8_t minute;       // 0-59
  uint8_t action;       // 0 = OFF, 1 = ON
  uint8_t enabled;      // 1 = active
};

enum OverrideMode : uint8_t {
  OVERRIDE_AUTO = 0,   // follow the schedule
  OVERRIDE_ON   = 1,   // force relay on
  OVERRIDE_OFF  = 2    // force relay off
};

// ---------------------------------------------------------------------------
// WaterScheduler
//
// Manages a weekly on/off schedule for a single relay output pin.
// State (schedule entries + override) is persisted to SPIFFS.
//
// CLI commands (via handleCommand):
//   relay on              - force relay ON (persists override)
//   relay off             - force relay OFF (persists override)
//   relay auto            - return to schedule (persists mode)
//   relay status          - report current state
//   schedule add <day|daily> <HH:MM> <on|off>
//                         - add an entry  (day: 0=Sun..6=Sat, or 'daily')
//   schedule del <index>  - delete entry by index
//   schedule list         - list all entries
//   schedule clear        - remove all entries
// ---------------------------------------------------------------------------
class WaterScheduler {
public:
  WaterScheduler() : _fs(nullptr), _relay_pin(PIN_RELAY), _count(0),
                     _rtc(nullptr), _override(OVERRIDE_AUTO),
                     _override_expiry_unix(0), _override_expiry_millis(0),
                     _relay_state(false),
                     _last_minute(-1) {}

  // Call once after SPIFFS is mounted.
  void begin(FILESYSTEM* fs, mesh::RTCClock* rtc, uint8_t relay_pin = PIN_RELAY);

  // Call every loop iteration, passing the shared RTC clock.
  void loop(mesh::RTCClock* rtc);

  // Handle a CLI command string. Returns true if the command was consumed.
  // sender_timestamp == 0 when called from the serial terminal.
  bool handleCommand(uint32_t sender_timestamp, char* command, char* reply);

  bool        getRelayState()   const { return _relay_state; }
  OverrideMode getOverrideMode() const { return _override; }

private:
  FILESYSTEM*   _fs;
  uint8_t       _relay_pin;
  ScheduleEntry _entries[MAX_SCHEDULE_ENTRIES];
  int           _count;
  mesh::RTCClock* _rtc;
  OverrideMode  _override;
  uint32_t      _override_expiry_unix;
  uint32_t      _override_expiry_millis;
  bool          _relay_state;
  int           _last_minute;   // last minute-of-day we evaluated (prevents re-fire)

  void setRelay(bool on);
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
