#include "WaterScheduler.h"
#include <RTClib.h>   // DateTime from RTClib (already a dependency)

// ---------------------------------------------------------------------------
// Day name table (index 7 = Daily)
// ---------------------------------------------------------------------------
static const char* const DAY_NAMES[] = {
  "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Daily"
};

const char* WaterScheduler::dayName(uint8_t dow) {
  if (dow <= 7) return DAY_NAMES[dow];
  return "?";
}

// ---------------------------------------------------------------------------
// begin()  – initialise GPIO and load persisted state
// ---------------------------------------------------------------------------
void WaterScheduler::begin(FILESYSTEM* fs, mesh::RTCClock* rtc, uint8_t relay_pin) {
  _fs        = fs;
  _rtc       = rtc;
  _relay_pin = relay_pin;

  pinMode(_relay_pin, OUTPUT);
  digitalWrite(_relay_pin, LOW);
  _relay_state = false;
  _last_minute = -1;

  loadSchedule();
  loadOverride();
}

// ---------------------------------------------------------------------------
// setRelay()  – drive the GPIO and record state
// ---------------------------------------------------------------------------
void WaterScheduler::setRelay(bool on) {
  _relay_state = on;
  digitalWrite(_relay_pin, on ? HIGH : LOW);
}

void WaterScheduler::setOverrideExpiry() {
  _override_expiry_millis = millis() + (OVERRIDE_DURATION_SECS * 1000UL);
  _override_expiry_unix = 0;

  if (_rtc) {
    uint32_t now_unix = _rtc->getCurrentTime();
    if (now_unix >= 100000UL) {
      _override_expiry_unix = now_unix + OVERRIDE_DURATION_SECS;
    }
  }
}

void WaterScheduler::expireOverrideIfNeeded() {
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
void WaterScheduler::loop(mesh::RTCClock* rtc) {
  if (rtc) {
    _rtc = rtc;
  }

  expireOverrideIfNeeded();

  if (!rtc) return;

  uint32_t now_unix = rtc->getCurrentTime();
  if (now_unix < 100000UL) return;    // RTC not set yet (epoch near zero)

  DateTime dt(now_unix);
  int current_minute = (int)dt.hour() * 60 + (int)dt.minute();

  if (_last_minute == -1) {
    // First valid call after boot: restore the relay to the expected state
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
// checkSchedule()  – test all active entries against the current moment
// ---------------------------------------------------------------------------
void WaterScheduler::checkSchedule(uint8_t dow, uint8_t hour, uint8_t minute) {
  for (int i = 0; i < _count; i++) {
    const ScheduleEntry& e = _entries[i];
    if (!e.enabled)                         continue;
    if (e.hour != hour || e.minute != minute) continue;
    if (e.day_of_week != 7 && e.day_of_week != dow) continue;
    // Match: apply action
    setRelay(e.action == 1);
  }
}

// ---------------------------------------------------------------------------
// restoreState()  – determine the expected relay state at boot
//
// Walks backwards through the weekly calendar to find the most recently
// scheduled action, then applies it.  Override modes bypass this logic.
// ---------------------------------------------------------------------------
void WaterScheduler::restoreState(mesh::RTCClock* rtc) {
  if (_override == OVERRIDE_ON)  { setRelay(true);  return; }
  if (_override == OVERRIDE_OFF) { setRelay(false); return; }
  if (_count == 0)               { setRelay(false); return; }

  uint32_t now_unix = rtc->getCurrentTime();
  DateTime dt(now_unix);

  // Current position in the week expressed in minutes (0 = Sun 00:00)
  int cur_week_min = (int)dt.dayOfTheWeek() * 1440
                   + (int)dt.hour()         * 60
                   + (int)dt.minute();

  int     best_offset = 7 * 1440 + 1;   // sentinel  (> one full week)
  uint8_t best_action = 0;              // default to OFF if nothing found

  for (int i = 0; i < _count; i++) {
    const ScheduleEntry& e = _entries[i];
    if (!e.enabled) continue;

    int entry_min = (int)e.hour * 60 + (int)e.minute;
    int d_start   = (e.day_of_week == 7) ? 0 : (int)e.day_of_week;
    int d_end     = (e.day_of_week == 7) ? 6 : (int)e.day_of_week;

    for (int d = d_start; d <= d_end; d++) {
      int entry_week_min = d * 1440 + entry_min;
      int ago = cur_week_min - entry_week_min;
      if (ago < 0) ago += 7 * 1440;   // wrap around the week boundary

      if (ago < best_offset) {
        best_offset = ago;
        best_action = e.action;
      }
    }
  }

  setRelay(best_action == 1);
}

// ---------------------------------------------------------------------------
// Persistence helpers
// ---------------------------------------------------------------------------
void WaterScheduler::saveSchedule() {
  if (!_fs) return;
  File f = _fs->open(SCHED_FILE, FILE_WRITE);
  if (!f) return;
  uint8_t cnt = (uint8_t)_count;
  f.write(&cnt, 1);
  if (_count > 0) {
    f.write(reinterpret_cast<uint8_t*>(_entries),
            _count * sizeof(ScheduleEntry));
  }
  f.close();
}

void WaterScheduler::loadSchedule() {
  _count = 0;
  if (!_fs || !_fs->exists(SCHED_FILE)) return;

  File f = _fs->open(SCHED_FILE, FILE_READ);
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

void WaterScheduler::saveOverride() {
  if (!_fs) return;
  File f = _fs->open(OVERRIDE_FILE, FILE_WRITE);
  if (!f) return;
  uint8_t v = (uint8_t)_override;
  f.write(&v, 1);
  f.write(reinterpret_cast<uint8_t*>(&_override_expiry_unix), sizeof(_override_expiry_unix));
  f.close();
}

void WaterScheduler::loadOverride() {
  _override = OVERRIDE_AUTO;
  _override_expiry_unix = 0;
  _override_expiry_millis = 0;
  if (!_fs || !_fs->exists(OVERRIDE_FILE)) return;

  File f = _fs->open(OVERRIDE_FILE, FILE_READ);
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
//
// Returns true if the command was consumed (even if it produced an error).
// reply buffer is 160 bytes (same as CommonCLI convention).
// ---------------------------------------------------------------------------
bool WaterScheduler::handleCommand(uint32_t sender_timestamp, char* command, char* reply) {
  // ---- relay on ----
  if (strcmp(command, "relay on") == 0) {
    _override = OVERRIDE_ON;
    setOverrideExpiry();
    saveOverride();
    setRelay(true);
    strcpy(reply, "Relay FORCED ON for 30 minutes");
    return true;
  }

  // ---- relay off ----
  if (strcmp(command, "relay off") == 0) {
    _override = OVERRIDE_OFF;
    setOverrideExpiry();
    saveOverride();
    setRelay(false);
    strcpy(reply, "Relay FORCED OFF for 30 minutes");
    return true;
  }

  // ---- relay auto ----
  if (strcmp(command, "relay auto") == 0) {
    _override = OVERRIDE_AUTO;
    _override_expiry_unix = 0;
    _override_expiry_millis = 0;
    saveOverride();
    _last_minute = -1;   // trigger a re-evaluate on next loop tick
    strcpy(reply, "Relay set to AUTO (schedule)");
    return true;
  }

  // ---- relay status ----
  if (strcmp(command, "relay status") == 0) {
    const char* mode_str = (_override == OVERRIDE_ON)  ? "FORCED ON"  :
                           (_override == OVERRIDE_OFF) ? "FORCED OFF" : "AUTO";
    snprintf(reply, 160, "Relay: %s | Mode: %s | Entries: %d",
             _relay_state ? "ON" : "OFF", mode_str, _count);
    return true;
  }

  // ---- schedule list ----
  if (strcmp(command, "schedule list") == 0) {
    if (_count == 0) {
      strcpy(reply, "No schedule entries");
    } else {
      // When called from serial terminal print full list there too
      if (sender_timestamp == 0) {
        Serial.printf("Schedule (%d entries):\r\n", _count);
        for (int i = 0; i < _count; i++) {
          const ScheduleEntry& e = _entries[i];
          Serial.printf("  [%d] %s %02d:%02d %s\r\n",
                        i, dayName(e.day_of_week),
                        e.hour, e.minute,
                        e.action ? "ON" : "OFF");
        }
      }
      // Compact reply that fits in 160 chars:  "N: DdHHMMa DdHHMMa ..."
      // Each token = 9 chars, so 16 entries = 3+16*9 = 147 chars.
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

  // ---- schedule add <day|daily> <HH:MM> <on|off> ----
  // Examples:
  //   schedule add daily 06:00 on
  //   schedule add 1 07:30 on        (Monday)
  //   schedule add 0 22:00 off       (Sunday)
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
      if (!sp) { strcpy(reply, "Err: usage: schedule add <day|daily> <HH:MM> <on|off>"); return true; }
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
      strcpy(reply, "Err: usage: schedule add <day|daily> <HH:MM> <on|off>");
      return true;
    }

    // Accept "on"/"ON" / "off"/"OFF"
    bool is_on = (strcmp(action_str, "on") == 0 || strcmp(action_str, "ON") == 0);

    ScheduleEntry& e = _entries[_count++];
    e.day_of_week = (uint8_t)day_val;
    e.hour        = (uint8_t)hh;
    e.minute      = (uint8_t)mm;
    e.action      = is_on ? 1 : 0;
    e.enabled     = 1;

    saveSchedule();
    snprintf(reply, 160, "Added [%d] %s %02d:%02d %s",
             _count - 1, dayName(e.day_of_week),
             e.hour, e.minute,
             e.action ? "ON" : "OFF");
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
