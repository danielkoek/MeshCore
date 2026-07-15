# Latching Solenoid Water Scheduler

A MeshCore-based water scheduler using a **latching solenoid valve** with **TB6612FNG I2C motor driver** for directional pulse control. Runs on **XIAO nRF52** with mesh networking and CLI-based scheduling.

## Key Differences from Relay Scheduler

| Feature | Relay Scheduler | Solenoid Scheduler |
|---------|-----------------|-------------------|
| Platform | XIAO S3 (ESP32) | XIAO nRF52 |
| Output Device | Relay (on/off) | Latching solenoid |
| Control | Simple HIGH/LOW | Directional motor pulses (I2C) |
| Pulse Duration | N/A | 500ms (configurable) |
| Motor Driver | N/A | TB6612FNG (I2C, addr 0x14) |
| Interface | GPIO | I2C (SDA/SCL) |

## Hardware Wiring

### TB6612FNG Motor Driver → XIAO nRF52 (I2C)

| TB6612FNG Pin | XIAO nRF52 Pin | Function |
|---------------|---|----------|
| SDA | D7 | I2C data |
| SCL | D6 | I2C clock |
| GND | GND | Ground |
| VCC | 5V | Power (check driver spec) |
| MOTOR_OUT_A | Solenoid pin 1 | Motor output A |
| MOTOR_OUT_B | Solenoid pin 2 | Motor output B |

**Note:** TB6612FNG I2C address is **0x14** (default). Adjust with:
```ini
-D MOTOR_I2C_ADDR=0x14
```

Both channels are driven in parallel (0=MOTOR_CHA, 1=MOTOR_CHB), can be changed:
```ini
-D MOTOR_CHANNEL_A=0
-D MOTOR_CHANNEL_B=1
```

## Solenoid Pulse Logic

Raw I2C commands to both channels of the TB6612FNG:

**OPEN solenoid:**
- Send CCW command (counter-clockwise, full speed) to CHA and CHB
- Pulse for `SOLENOID_PULSE_DURATION` ms (default 500ms)
- Latching solenoid clicks open and stays open

**CLOSE solenoid:**
- Send CW command (clockwise, full speed) to CHA and CHB
- Pulse for `SOLENOID_PULSE_DURATION` ms (default 500ms)
- Latching solenoid clicks closed and stays closed

## Building & Flashing

```bash
# Build for solenoid scheduler
pio run -e solenoid_water_scheduler

# Upload to device
pio run -e solenoid_water_scheduler -t upload

# Monitor serial output
pio device monitor -b 115200
```

## CLI Commands

All commands work via serial terminal (115200 baud) or over mesh chat.

### Solenoid Control

```
solenoid open              # Force solenoid OPEN (persists 30 min)
solenoid close             # Force solenoid CLOSED (persists 30 min)
solenoid auto              # Return to schedule
solenoid status            # Show current state + mode + schedule entry count
```

### Schedule Management

```
schedule add daily 06:00 open    # Every day at 06:00 open valve
schedule add daily 22:00 close   # Every day at 22:00 close valve
schedule add 1 08:00 open        # Monday at 08:00 open (0=Sun, 1=Mon, ..., 6=Sat)
schedule add 0 12:00 close       # Sunday at 12:00 close

schedule list                     # Show all entries (compact on mesh, full on serial)
schedule del 0                    # Delete entry #0
schedule clear                    # Delete all entries
```

## State Persistence

State is saved to internal flash:
- `/solenoid_sched` – schedule entries
- `/solenoid_ovrd` – override mode + expiry timestamp

On boot, the scheduler restores the solenoid to the expected state based on the most recent scheduled action in the past week.

## Configuration

### Override Duration

Edit `WaterSchedulerSolenoid.h`:

```cpp
#define OVERRIDE_DURATION_SECS 1800  // 30 minutes
```

### Pulse Duration

Edit `WaterSchedulerSolenoid.h`:

```cpp
#define SOLENOID_PULSE_DURATION 500  // milliseconds
```

Adjust if your solenoid requires a longer or shorter pulse.

### Max Schedule Entries

```cpp
#define MAX_SCHEDULE_ENTRIES 16  // Maximum 16 scheduled actions
```

## RTC Synchronization

The scheduler uses the RTC clock (auto-discovered) for accurate timekeeping. Without an RTC, the device will fall back to system time (less reliable).

For best results, add a **DS3231 RTC** module via I2C:
- SCL → D6
- SDA → D7
- GND → GND
- 5V → VCC

## Troubleshooting

### Solenoid doesn't respond to commands
- Check TB6612FNG wiring and voltage
- Verify PIN_MOTOR_PWM/DIR_A/DIR_B are configured correctly in platformio.local.ini
- Confirm PWM pin can deliver enough current (solenoid draws 100-500mA during pulse)

### Schedule doesn't trigger at expected time
- Verify RTC is set correctly (`solenoid status` will show RTC state)
- Check time zone and day-of-week (0=Sunday)
- Ensure schedule entry is enabled and not deleted

### State doesn't persist across reboot
- Verify internal flash filesystem is mounted
- Check `/solenoid_sched` and `/solenoid_ovrd` files exist on device

## Example Script

Set up watering 6am-10pm with a one-hour on/off cycle:

```
schedule clear
schedule add daily 06:00 open     # Turn on at 6am
schedule add daily 07:00 close    # Turn off at 7am
schedule add daily 08:00 open     # Turn on at 8am
schedule add daily 09:00 close    # Turn off at 9am
... (repeat for 6am-10pm)
schedule list
```

## Performance Notes

- Schedules checked once per minute
- Pulse overhead: ~500ms per solenoid state change
- No impact on mesh messaging during pulse (non-blocking)
- Maximum 16 schedule entries (configurable)

## See Also

- [WaterScheduler (relay-based)](../water_scheduler/WaterScheduler.h) – Original scheduler for relay outputs
- [SensorMesh](../simple_sensor/SensorMesh.h) – Base mesh framework
- TB6612FNG datasheet: https://www.pololu.com/file/0J86/TB6612FNG.pdf
