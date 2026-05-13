# CHAdeMO DC Fast Charge Controller — RP2040

A modular, safety-focused CHAdeMO protocol implementation for the Raspberry Pi Pico (RP2040), designed for both EV (vehicle) and EVSE (charging station) applications.

## Features

- **Dual-role support**: Compile-time selection of VEHICLE or STATION role
- **Complete CHAdeMO state machine**: Idle → Plug Detect → Handshake → Parameter Check → Insulation Test → Pre-charge → Charging → Shutdown → Stopped
- **Safety-first design**: Watchdog timer, failsafe GPIO defaults, sequenced emergency shutdown
- **Dual CAN buses**: Two independent MCP2515 controllers on SPI0 and SPI1
- **Hardware abstraction**: Clean HAL for easy porting to other MCUs
- **Dynamic current control**: 20A/sec slew rate limiting per CHAdeMO spec
- **Extensive fault detection**: 15 fault codes including CAN timeout, voltage/current mismatch, overtemperature, ESTOP
- **Open-source foundations**: CAN structures verified against real-world implementations (furdog/chademo, jamiejones85/ESP32-Chademo, Isaac96/CHAdeMOSoftware)

## Architecture

```
main.c              — 100Hz main loop, event handling, diagnostics
chademo_config.h    — Compile-time role selection, GPIO pin mapping, protocol constants
chademo_hal.h/.c    — Pico SDK GPIO and SPI-CAN abstraction layer
chademo_can.h/.c    — CHAdeMO CAN frame structures (0x100-0x109), pack/unpack functions
chademo_fsm.h/.c    — Core non-blocking state machine (vehicle + station logic)
flash.sh            — Auto-detect and flash script for RP2040/RP2350
CMakeLists.txt      — Pico SDK build configuration
```

## Hardware Requirements

### MCU
- Raspberry Pi Pico (RP2040)

### CAN Interface
- 2x MCP2515 CAN controller modules with TJA1051 transceivers
- **Crystal frequency**: Most modules use 8MHz (set `MCP2515_OSC_MHZ` in config). Some use 16MHz.
- **Bit rate**: 500 kbps (CHAdeMO standard)

### GPIO Mapping

| Function | GPIO | Description |
|----------|------|-------------|
| **CAN1 SPI0** |||
| SCK | GP2 | SPI clock |
| TX (MOSI) | GP3 | SPI data out |
| RX (MISO) | GP4 | SPI data in |
| INT | GP5 | MCP2515 interrupt |
| CS | GP6 | Chip select |
| **Charger Outputs** |||
| SS1 | GP7 | Charge sequence signal 1 |
| SS2 | GP8 | Charge sequence signal 2 |
| **Charger Input** |||
| DCP (in) | GP9 | DC Present from vehicle |
| **CAN2 SPI1** |||
| SCK | GP10 | SPI clock |
| TX (MOSI) | GP11 | SPI data out |
| RX (MISO) | GP12 | SPI data in |
| INT | GP13 | MCP2515 interrupt |
| CS | GP14 | Chip select |
| **Common** |||
| CONTACTOR | GP15 | HV contactor coil driver |
| **Vehicle Inputs** |||
| SS1 (in) | GP16 | Lock signal 1 from charger |
| SS2 (in) | GP17 | Lock signal 2 from charger |
| PP | GP18 | Proximity pilot |
| **Vehicle Output** |||
| DCP (out) | GP19 | DC Present to charger |

## Building

### Prerequisites
- [Pico SDK](https://github.com/raspberrypi/pico-sdk) installed
- ARM GCC toolchain (`gcc-arm-none-eabi`)
- CMake 3.13+
- Linux: `udisks2` (for auto-mounting) or `sudo` fallback

### Build Steps

```bash
# Set Pico SDK path
export PICO_SDK_PATH=/path/to/pico-sdk

# Create build directory
mkdir build && cd build

# Configure for VEHICLE role
cmake .. -DCHADEMO_ROLE=VEHICLE
make -j4

# Or configure for STATION role
cmake .. -DCHADEMO_ROLE=STATION
make -j4
```

### Flashing to the Pico

The project includes `flash.sh` -- an auto-detection script that finds a Pico in BOOTSEL mode (RP2040 or RP2350), mounts it if needed, and copies the correct `.uf2`.

#### Method 1: `make flash` (recommended)
```bash
# Put the Pico into BOOTSEL mode (hold BOOTSEL, plug in USB), then:
make flash
```

#### Method 2: Run the script directly
```bash
# From the build directory:
../flash.sh

# Or specify build dir and role explicitly:
../flash.sh ./build VEHICLE
```

#### Manual flash (fallback)
```bash
# Hold BOOTSEL while connecting USB, then copy the UF2:
cp chademo_controller_vehicle.uf2 /media/$USER/RPI-RP2/   # RP2040
# or
cp chademo_controller_vehicle.uf2 /media/$USER/RP2350/   # RP2350
```

### Debug Output
The firmware uses USB CDC for `printf()` output. Connect to the Pico's USB port and open a serial terminal at 115200 baud to see diagnostic messages.

```bash
# Linux
minicom -D /dev/ttyACM0 -b 115200

# Or use screen
screen /dev/ttyACM0 115200
```

## State Machine

```
                    +---------+
         +--------->|  IDLE   |<------------------+
         |          +----+----+                     |
         |               | plug detected            |
         |               v                          |
         |         +---------+                      |
         |         | PLUG    |                      |
         |         |DETECTED |                      |
         |         +----+----+                      |
         |              | CAN handshake            |
         |              v                          |
         |         +---------+                      |
         |         |HANDSHAKE|                      |
         |         +----+----+                      |
         |              | params OK                |
         |              v                          |
         |         +---------+      +-------------+ |
         |         | PARAM   |----->| INSULATION  | |
         |         | CHECK   |      | TEST (EVSE) | |
         |         +----+----+      +------+------+ |
         |              |                |          |
         |              v                v          |
         |         +---------+      +---------+     |
         |         |PRECHARGE|<-----+PRECHARGE|     |
         |         +----+----+      +----+----+     |
         |              | contactors close         |
         |              v                          |
         |         +---------+      +-------------+ |
         +---------| CHARGING|----->|   SHUTDOWN  | |
          fault    +----+----+      +------+------+ |
                      | fault              |         |
                      v                    v         |
               +-------------+    +-------------+   |
               |FAULT_SHUTDOWN|   |WAIT_ZERO_CUR|   |
               +------+------+   +------+------+   |
                      |                   |          |
                      v                   v          |
               +-------------+    +-------------+   |
               |   STOPPED   |<---+CONTACTOR_OPEN|  |
               +------+------+    +-------------+   |
                      | unplug                     |
                      +----------------------------+
```

## CAN Message Reference

### Vehicle transmits (EV → EVSE)

| ID | Name | Period | Content |
|----|------|--------|---------|
| 0x100 | Battery Specs | 100ms | Max battery voltage, pack capacity |
| 0x101 | Charge Timing | 100ms | Max charge time, estimated time, pack kWh |
| 0x102 | Control Request | 100ms | Protocol version, target V/I, fault flags, status, SOC |

### Charger transmits (EVSE → EV)

| ID | Name | Period | Content |
|----|------|--------|---------|
| 0x108 | Available Output | 100ms | Weld detection support, max output V/I, threshold voltage |
| 0x109 | System Status | 100ms | Protocol version, present V/I, status flags, remaining time |

## Safety Features

1. **Fail-safe boot**: Contactor is OPEN and all control signals are de-asserted before any initialization
2. **Watchdog timer**: Resets MCU if main loop hangs >100ms
3. **Sequenced emergency shutdown**: 4-phase fault handling (zero current → open contactor → de-assert controls → safe state)
4. **CAN timeout**: Fault if no frames received for >1 second
5. **Voltage/current mismatch**: Abort if measured values deviate >12.5% from reported values
6. **Pre-charge timeout**: Abort if voltage matching takes >10 seconds
7. **Plug debounce**: 200ms mechanical debounce on all plug detect inputs

## Integrating with Your Hardware

### 1. Replace simulated values with real sensors

In `main.c`, function `process_application_logic()`:

```c
// Replace these simulation lines:
chademo_fsm_set_measured_voltage(&g_fsm_ctx, sim_voltage);
chademo_fsm_set_measured_current(&g_fsm_ctx, sim_current);

// With actual ADC reads:
chademo_fsm_set_measured_voltage(&g_fsm_ctx, adc_read_voltage());
chademo_fsm_set_measured_current(&g_fsm_ctx, adc_read_current());
chademo_fsm_set_battery_soc(&g_fsm_ctx, bms_get_soc());
chademo_fsm_set_battery_temp(&g_fsm_ctx, bms_get_max_temp());
```

### 2. Add BMS CAN communication

If your BMS communicates via CAN (e.g., Orion BMS, REC), use CAN2 (SPI1) to receive BMS frames and feed the data into the FSM using the setter functions.

### 3. Control the power supply (EVSE role)

In the CHARGING state (EVSE), read `ctx->rx.h102.charging_current_request_A` and `ctx->rx.h102.target_battery_voltage_V` to set your PSU's voltage and current setpoints.

## References

This implementation incorporates knowledge from these excellent open-source CHAdeMO projects:

| Project | Author | Role | Key Contribution |
|---------|--------|------|-----------------|
| [furdog/chademo](https://github.com/furdog/chademo) | furdog | EVSE | Hardware-agnostic EVSE state machine, CAN frame structures per IEEE 2030.1.1 |
| [ESP32-Chademo](https://github.com/jamiejones85/ESP32-Chademo) | jamiejones85 | EV | Real-world EV-side implementation with dynamic current control |
| [CHAdeMOSoftware](https://github.com/Isaac96/CHAdeMOSoftware) | Isaac96 | EV | Arduino Due port, JLD505-based, tested on converted EVs |
| [ccs32clara-chademo](https://github.com/osexpert/ccs32clara-chademo) | osexpert | Adapter | CCS-to-CHAdeMO adapter, detailed timing notes |

## License

This firmware is provided as-is for educational and development purposes. The CHAdeMO protocol is subject to patents and trade secrets held by the CHAdeMO Association. Commercial use requires appropriate licensing.

## Disclaimer

**WARNING**: This software controls high-voltage DC equipment. Improper use can cause electric shock, fire, or death. The authors assume no liability for any damages. This code has not been certified for safety-critical applications. Thorough testing with appropriate safety equipment is mandatory before any real-world deployment.
