#include "WaterSchedulerSolenoid.h"
#include <RTClib.h>
#include <Wire.h>

// TB6612FNG motor driver I2C commands
const uint8_t CMD_BRAKE = 0x00;
const uint8_t CMD_STOP = 0x01;
const uint8_t CMD_CW = 0x02;   // Clockwise
const uint8_t CMD_CCW = 0x03;  // Counter-clockwise
const uint8_t CMD_STANDBY = 0x04;

// File mode constants (from Adafruit_LittleFS)
#ifndef FILE_O_READ
  #define FILE_O_READ  0x00
#endif
#ifndef FILE_O_WRITE
  #define FILE_O_WRITE 0x02
#endif

static const char* const DAY_NAMES[] = {
  "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Daily"
};

const char* WaterSchedulerSolenoid::dayName(uint8_t dow) {
  if (dow <= 7) return DAY_NAMES[dow];
  return "?";
}

// ---------------------------------------------------------------------------
// begin()  – initialise I2C motor driver and load persisted state
// ---------------------------------------------------------------------------
void WaterSchedulerSolenoid::begin(FILESYSTEM* fs, mesh::RTCClock* rtc) {
  _fs = fs;
  _rtc = rtc;

  // NOTE: board defaults to running mode on power-up. Sending NOT_STANDBY
  // (0x05) leaves the board acking I2C but ignoring CW/CCW writes, so only
  // send CMD_STANDBY (0x04) as a no-op init, then ensure motor is stopped.
  Wire.beginTransmission(MOTOR_I2C_ADDR);
  Wire.write(CMD_STANDBY);
  Wire.write((uint8_t)0);
  Wire.endTransmission();
  delay(10);

  motorStop();

  _solenoid_open = false;
  _pulse_active = false;
  _pulse_end_millis = 0;
  _last_minute = -1;

  loadSchedule();
  loadOverride();
}

// ---------------------------------------------------------------------------
// motorDrive()  – send directional pulse to solenoid via I2C motor driver
// Drives both channels in parallel (matches validated .ino test sketch).
// open=true: CCW (backward) to OPEN valve
// open=false: CW (forward) to CLOSE valve
// ---------------------------------------------------------------------------
void WaterSchedulerSolenoid::motorDrive(bool open) {
  uint8_t cmd = open ? CMD_CCW : CMD_CW;
  uint8_t speed = SOLENOID_SPEED;

  Wire.beginTransmission(MOTOR_I2C_ADDR);
  Wire.write(cmd);
  Wire.write((uint8_t)MOTOR_CHANNEL_A);
  Wire.write(speed);
  Wire.endTransmission();

  Wire.beginTransmission(MOTOR_I2C_ADDR);
  Wire.write(cmd);
  Wire.write((uint8_t)MOTOR_CHANNEL_B);
  Wire.write(speed);
  Wire.endTransmission();

  _pulse_active = true;
  _pulse_end_millis = millis() + SOLENOID_PULSE_DURATION;
}

// ---------------------------------------------------------------------------
// motorStop()  – stop motor via I2C (both channels)
// ---------------------------------------------------------------------------
void WaterSchedulerSolenoid::motorStop() {
  Wire.beginTransmission(MOTOR_I2C_ADDR);
  Wire.write(CMD_STOP);
  Wire.write((uint8_t)MOTOR_CHANNEL_A);
  Wire.endTransmission();

  Wire.beginTransmission(MOTOR_I2C_ADDR);
  Wire.write(CMD_STOP);
  Wire.write((uint8_t)MOTOR_CHANNEL_B);
  Wire.endTransmission();

  _pulse_active = false;
}

// ---------------------------------------------------------------------------
// updatePulse()  – check if pulse duration has elapsed and stop if so
// ---------------------------------------------------------------------------
void WaterSchedulerSolenoid::updatePulse() {
  if (!_pulse_active) return;

  if ((int32_t)(millis() - _pulse_end_millis) >= 0) {
    motorStop();
  }
}

// ---------------------------------------------------------------------------
// setSolenoid()  – command solenoid to a new state
// If already in target state, does nothing.
// If target state differs, sends pulse.
// ---------------------------------------------------------------------------
void WaterSchedulerSolenoid::setSolenoid(bool open) {
  if (_solenoid_open == open) return;  // already in target state

  _solenoid_open = open;
  motorDrive(open);
}

void WaterSchedulerSolenoid::setOverrideExpiry() {
  _override_expiry_millis = millis() + (OVERRIDE_DURATION_SECS * 1000UL);
  _override_expiry_unix = 0;

  if (_rtc) {
    uint32_t now_unix = _rtc->getCurrentTime();
    if (now_unix >= 100000UL) {
      _override_expiry_unix = now_unix + OVERRIDE_DURATION_SECS;
    }
  }
}

void WaterSchedulerSolenoid::expireOverrideIfNeeded() {
  if (_override == OVERRIDE_AUTO) return;

  bool expired = false;

  if (_override_expiry_unix != 0 && _rtc) {
    uint32_t now_unix = _rtc->getCurrentTime();
    if (now_unix >= _override_expiry_unix) {
      expired = true;
    }
  }

  if (!expired && _override_expiry_millis != 0
      && (int32_t)(millis() - _override_expiry_millis) >= 0) {
    expired = true;
  }

  if (!expired) return;

  _override = OVERRIDE_AUTO;
  _override_expiry_unix = 0;
  _override_expiry_millis = 0;
  _last_minute = -1;
  saveOverride();
}

// ---------------------------------------------------------------------------
// loop()  – called every Arduino loop iteration
// ---------------------------------------------------------------------------
void WaterSchedulerSolenoid::loop(mesh::RTCClock* rtc) {
  if (rtc) {
    _rtc = rtc;
  }

  // Check if pulse duration has elapsed
  updatePulse();

  expireOverrideIfNeeded();

  if (!rtc) return;

  uint32_t now_unix = rtc->getCurrentTime();
  if (now_unix < 100000UL) return;    // RTC not set yet

  DateTime dt(now_unix);
  int current_minute = (int)dt.hour() * 60 + (int)dt.minute();

  if (_last_minute == -1) {
    // First valid call after boot: restore the solenoid to expected state
    restoreState(rtc);
    _last_minute = current_minute;
    return;
  }

  if (current_minute != _last_minute) {
    _last_minute = current_minute;
    if (_override == OVERRIDE_AUTO) {
      checkSchedule(dt.dayOfTheWeek(), dt.hour(), dt.minute());
    }
  }
}

// ---------------------------------------------------------------------------
// checkSchedule()  – test all active entries against current moment
// ---------------------------------------------------------------------------
void WaterSchedulerSolenoid::checkSchedule(uint8_t dow, uint8_t hour, uint8_t minute) {
  for (int i = 0; i < _count; i++) {
    const ScheduleEntry& e = _entries[i];
    if (!e.enabled)                         continue;
    if (e.hour != hour || e.minute != minute) continue;
    if (e.day_of_week != 7 && e.day_of_week != dow) continue;
    // Match: apply action
    setSolenoid(e.action == 1);
  }
}

// ---------------------------------------------------------------------------
// restoreState()  – determine expected solenoid state at boot
//
// Walks backwards through the weekly calendar to find most recent
// scheduled action, then applies it.
// ---------------------------------------------------------------------------
void WaterSchedulerSolenoid::restoreState(mesh::RTCClock* rtc) {
  if (_override == OVERRIDE_OPEN)  { setSolenoid(true);  return; }
  if (_override == OVERRIDE_CLOSE) { setSolenoid(false); return; }
  if (_count == 0)                 { setSolenoid(false); return; }

  uint32_t now_unix = rtc->getCurrentTime();
  DateTime dt(now_unix);

  // Current position in the week expressed in minutes (0 = Sun 00:00)
  int cur_week_min = (int)dt.dayOfTheWeek() * 1440
                   + (int)dt.hour()         * 60
                   + (int)dt.minute();

  int     best_offset = 7 * 1440 + 1;   // sentinel (> one full week)
  uint8_t best_action = 0;              // default to CLOSE if nothing found

  for (int i = 0; i < _count; i++) {
    const ScheduleEntry& e = _entries[i];
    if (!e.enabled) continue;

    int entry_min = (int)e.hour * 60 + (int)e.minute;
    int d_start   = (e.day_of_week == 7) ? 0 : (int)e.day_of_week;
    int d_end     = (e.day_of_week == 7) ? 6 : (int)e.day_of_week;

    for (int d = d_start; d <= d_end; d++) {
      int entry_week_min = d * 1440 + entry_min;
      int ago = cur_week_min - entry_week_min;
      if (ago < 0) ago += 7 * 1440;   // wrap around week boundary

      if (ago < best_offset) {
        best_offset = ago;
        best_action = e.action;
      }
    }
  }

  setSolenoid(best_action == 1);
}

// ---------------------------------------------------------------------------
// Persistence helpers
// ---------------------------------------------------------------------------
void WaterSchedulerSolenoid::saveSchedule() {
  if (!_fs) return;
  File f = _fs->open(SCHED_FILE, FILE_O_WRITE);
  if (!f) return;
  uint8_t cnt = (uint8_t)_count;
  f.write(&cnt, 1);
  if (_count > 0) {
    f.write(reinterpret_cast<uint8_t*>(_entries),
            _count * sizeof(ScheduleEntry));
  }
  f.close();
}

void WaterSchedulerSolenoid::loadSchedule() {
  _count = 0;
  if (!_fs || !_fs->exists(SCHED_FILE)) return;

  File f = _fs->open(SCHED_FILE, FILE_O_READ);
  if (!f) return;

  uint8_t cnt = 0;
  if (f.read(&cnt, 1) == 1 && cnt <= MAX_SCHEDULE_ENTRIES) {
    size_t expected = cnt * sizeof(ScheduleEntry);
    if (f.read(reinterpret_cast<uint8_t*>(_entries), expected) == expected) {
      _count = (int)cnt;
    }
  }
  f.close();
}

void WaterSchedulerSolenoid::saveOverride() {
  if (!_fs) return;
  File f = _fs->open(OVERRIDE_FILE, FILE_O_WRITE);
  if (!f) return;
  uint8_t v = (uint8_t)_override;
  f.write(&v, 1);
  f.write(reinterpret_cast<uint8_t*>(&_override_expiry_unix), sizeof(_override_expiry_unix));
  f.close();
}

void WaterSchedulerSolenoid::loadOverride() {
  _override = OVERRIDE_AUTO;
  _override_expiry_unix = 0;
  _override_expiry_millis = 0;
  if (!_fs || !_fs->exists(OVERRIDE_FILE)) return;

  File f = _fs->open(OVERRIDE_FILE, FILE_O_READ);
  if (!f) return;
  uint8_t v = 0;
  if (f.read(&v, 1) == 1 && v <= 2) {
    _override = (OverrideMode)v;
    if (f.read(reinterpret_cast<uint8_t*>(&_override_expiry_unix), sizeof(_override_expiry_unix))
          != sizeof(_override_expiry_unix)) {
      _override_expiry_unix = 0;
    }
  }
  f.close();

  if (_override != OVERRIDE_AUTO && _override_expiry_unix == 0) {
    _override = OVERRIDE_AUTO;
  }
}

// ---------------------------------------------------------------------------
// handleCommand()  – CLI dispatch
// ---------------------------------------------------------------------------
bool WaterSchedulerSolenoid::handleCommand(uint32_t sender_timestamp, char* command, char* reply) {
  // ---- solenoid open ----
  if (strcmp(command, "solenoid open") == 0) {
    _override = OVERRIDE_OPEN;
    setOverrideExpiry();
    saveOverride();
    setSolenoid(true);
    strcpy(reply, "Solenoid FORCED OPEN for 30 minutes");
    return true;
  }

  // ---- solenoid close ----
  if (strcmp(command, "solenoid close") == 0) {
    _override = OVERRIDE_CLOSE;
    setOverrideExpiry();
    saveOverride();
    setSolenoid(false);
    strcpy(reply, "Solenoid FORCED CLOSED for 30 minutes");
    return true;
  }

  // ---- solenoid auto ----
  if (strcmp(command, "solenoid auto") == 0) {
    _override = OVERRIDE_AUTO;
    _override_expiry_unix = 0;
    _override_expiry_millis = 0;
    saveOverride();
    _last_minute = -1;   // trigger re-evaluate on next loop tick
    strcpy(reply, "Solenoid set to AUTO (schedule)");
    return true;
  }

  // ---- solenoid status ----
  if (strcmp(command, "solenoid status") == 0) {
    const char* mode_str = (_override == OVERRIDE_OPEN)  ? "FORCED OPEN"  :
                           (_override == OVERRIDE_CLOSE) ? "FORCED CLOSED" : "AUTO";
    snprintf(reply, 160, "Solenoid: %s | Mode: %s | Entries: %d",
             _solenoid_open ? "OPEN" : "CLOSED", mode_str, _count);
    return true;
  }

  // ---- schedule list ----
  if (strcmp(command, "schedule list") == 0) {
    if (_count == 0) {
      strcpy(reply, "No schedule entries");
    } else {
      if (sender_timestamp == 0) {
        Serial.printf("Schedule (%d entries):\r\n", _count);
        for (int i = 0; i < _count; i++) {
          const ScheduleEntry& e = _entries[i];
          Serial.printf("  [%d] %s %02d:%02d %s\r\n",
                        i, dayName(e.day_of_week),
                        e.hour, e.minute,
                        e.action ? "OPEN" : "CLOSE");
        }
      }
      // Compact reply: "N: DdHHMMa DdHHMMa ..."
      int ofs = snprintf(reply, 160, "%d:", _count);
      for (int i = 0; i < _count && ofs < 152; i++) {
        const ScheduleEntry& e = _entries[i];
        char day_ch = (e.day_of_week == 7) ? 'D' : (char)('0' + e.day_of_week);
        ofs += snprintf(reply + ofs, 160 - ofs, " %c%02d%02d%c",
                        day_ch, e.hour, e.minute,
                        e.action ? '+' : '-');
      }
    }
    return true;
  }

  // ---- schedule clear ----
  if (strcmp(command, "schedule clear") == 0) {
    _count = 0;
    saveSchedule();
    strcpy(reply, "Schedule cleared");
    return true;
  }

  // ---- schedule add <day|daily> <HH:MM> <open|close> ----
  if (memcmp(command, "schedule add ", 13) == 0) {
    if (_count >= MAX_SCHEDULE_ENTRIES) {
      strcpy(reply, "Err: schedule full (max 16)");
      return true;
    }

    char* p = command + 13;
    int day_val;

    if (memcmp(p, "daily ", 6) == 0) {
      day_val = 7;
      p += 6;
    } else {
      char* sp = strchr(p, ' ');
      if (!sp) { strcpy(reply, "Err: usage: schedule add <day|daily> <HH:MM> <open|close>"); return true; }
      *sp = '\0';
      day_val = atoi(p);
      if (day_val < 0 || day_val > 7) { strcpy(reply, "Err: day must be 0-6 or 'daily'"); return true; }
      p = sp + 1;
    }

    int hh = -1, mm = -1;
    char action_str[8] = {0};
    if (sscanf(p, "%d:%d %7s", &hh, &mm, action_str) != 3
        || hh < 0 || hh > 23
        || mm < 0 || mm > 59) {
      strcpy(reply, "Err: usage: schedule add <day|daily> <HH:MM> <open|close>");
      return true;
    }

    // Accept "open"/"OPEN" / "close"/"CLOSE"
    bool is_open = (strcmp(action_str, "open") == 0 || strcmp(action_str, "OPEN") == 0);

    ScheduleEntry& e = _entries[_count++];
    e.day_of_week = (uint8_t)day_val;
    e.hour        = (uint8_t)hh;
    e.minute      = (uint8_t)mm;
    e.action      = is_open ? 1 : 0;
    e.enabled     = 1;

    saveSchedule();
    snprintf(reply, 160, "Added [%d] %s %02d:%02d %s",
             _count - 1, dayName(e.day_of_week),
             e.hour, e.minute,
             e.action ? "OPEN" : "CLOSE");
    return true;
  }

  // ---- schedule del <index> ----
  if (memcmp(command, "schedule del ", 13) == 0) {
    int idx = atoi(command + 13);
    if (idx < 0 || idx >= _count) {
      snprintf(reply, 160, "Err: index %d out of range (0-%d)", idx, _count - 1);
    } else {
      for (int i = idx; i < _count - 1; i++) {
        _entries[i] = _entries[i + 1];
      }
      _count--;
      saveSchedule();
      snprintf(reply, 160, "Deleted entry %d (%d remaining)", idx, _count);
    }
    return true;
  }

  return false;   // command not handled by this module
}
