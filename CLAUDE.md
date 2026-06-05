# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

TI MSPM0G3507 bare-metal firmware for a line-following robot car. Built with Code Composer Studio (CCS) Theia, TI armclang compiler, MSPM0 SDK 2.10.00.04.

## Build / Flash / Debug

- **Build:** `Ctrl+B` in CCS Theia, or run `make` inside `Debug/`. Generated makefile is at `Debug/makefile`.
- **Clean:** `make clean` in `Debug/`.
- **Flash & Debug:** CCS Theia launch config at `.theia/launch.json`. Use XDS110 debug probe.
- **SysConfig:** Peripheral configuration is in `empty.syscfg`. Opening it in CCS regenerates `ti_msp_dl_config.c` / `ti_msp_dl_config.h` (in `Debug/`). These files are auto-generated — do not hand-edit them.

## Architecture

### Main loop (`main.c`)
1. `SYSCFG_DL_init()` — clocks, GPIO, timers, I2C, PWM, UART from SysConfig.
2. Init OLED (I2C1), motors (2× DC with TB6612 + quadrature encoders), MPU6050 gyro (I2C0).
3. Enable interrupts: encoder edges, buttons (KEY_1 on PB7, KEY_4 on PB6).
4. Loop: read MPU6050 yaw → calibrate → display yaw on OLED → `status_run(yaw)`.

### State machine (`user_driver/status.c`)
`sys_status` (enum `system_status_t`) controls operating mode:

| Mode | Enum | Behavior |
|------|------|----------|
| 0 | `STATUS_IDLE` | Motors off, wait for button |
| 1 | `STATUS_DIST` | Straight 1750 encoder pulses, then hard brake |
| 2 | `STATUS_LINE_TRACK_2` | Straight until line detected → track line, gyro recovery when off-line |
| 3 | `STATUS_LINE_TRACK` | Full competition: S3_STRAIGHT1 → S3_ALIGN1 → S3_CURVE1 → S3_STRAIGHT2 → S3_ALIGN2 → S3_CURVE2, loops with lap counting |

- `status_cycle_next()` — button toggles mode (wraps around `STATUS_COUNT`).
- `status_toggle_start()` — button toggles start/stop for current mode.
- `status_run(yaw)` — called every main loop iteration.

### Motor control (`user_driver/motor.c`)
- **PID Timer ISR** (`MOTOR_PID_INST_IRQHandler`): fires every 30ms. Calculates speed from encoder counts, applies low-pass filter, runs motor PID (positional: Kp=12, Ki=2).
- **Speed units:** mm/s. Encoder: 260 lines, tire radius 48mm.
- `target_speed_1` = right motor, `target_speed_2` = left motor. Upper layers write these; ISR reads them.
- **Hard brake:** reverses motor direction at 100% PWM for 8 ticks (~240ms), then shorts windings.
- **Mode 2 dispatch:** `question2_run()` runs inside ISR at 20Hz — switches between straight-nav and line-tracking based on sensor state.

### Line tracker (`user_driver/tracker.c`)
7-channel IR sensor array (L0–R0). `tracker_value[]` = 1 when over black line.
- `compute_error()` calculates weighted position (-1.0 to +1.0).
- `tracker_pid(base_speed, pid)` applies PID correction to `target_speed_1/2`.
- `track_line()` uses default PID (Kp=950, Ki=10, Kd=400) at BASE_SPEED 500.

### Gyro-stabilized straight (`user_driver/straight.c`)
Uses MPU6050 yaw for heading correction with a separate PID (Kp=2, Ki=0.3).
- Ramps speed up to `STRAIGHT_BASE_SPEED` (800).
- `straight_enc_acc` accumulates total encoder pulses (never reset).
- Two API layers: ISR-pathed (`straight_begin/run`, sets `straight_active`, has exit) and main-loop-pathed (`straight_nav_begin/run/update_ref`, no exit, used by mode 2).

### MPU6050 (`user_driver/mpu_port.c`, `user_driver/mpu_nav.c`, `user_driver/inv_mpu*.c`)
- InvenSense DMP driver (`inv_mpu.c` + `inv_mpu_dmp_motion_driver.c`) provides quaternion-based yaw.
- `mpu_nav.c` wraps calibration: auto-calibrates first 50 samples, or `mpu_reset_zero()` for instant zero.
- `g_yaw` = calibrated yaw, updated in main loop. `g_raw_yaw` = raw reading, also available.

### PID utilities (`user_driver/pid_utils.c`)
Reusable positional PID with integral clamping. Also provides `normalize_angle()` (to [-180,180]) and `clamp_value()`.

## Key timing constraints
- **Main loop** must read MPU6050 and update OLED fast enough to not miss yaw samples. Do NOT add blocking delays in the main loop.
- **PID ISR** runs every 30ms. Motor speed calculation and PID must complete within this window.
- **SysTick** at 1ms provides `sys_tick_ms` for timeouts.
- MPU6050 DMP reads are blocking I2C — the `while(DMP_Read_Data(...))` spin-waits for DMP FIFO.

## Pin map (quick reference)

| Peripheral | Pins |
|------------|------|
| PWM (motors) | PA12 (right), PA13 (left) |
| Motor direction | PA8/PA9 (right), PA7/PA18 (left) |
| Encoders | PA22/PA21 (right), PB19/PB20 (left) |
| I2C OLED | PB2 (SCL), PB3 (SDA) |
| I2C MPU6050 | PA1 (SCL), PA0 (SDA) |
| Line tracker | PB8(L1), PA17(L2), PB9(MID), PA2(R1), PA24(R2), PA25(R0), PA15(L0) |
| Buttons | PB7 (KEY_1), PB6 (KEY_4) |
| UART debug | PA28 (TX), PA31 (RX) |

## Development notes

- `volatile` globals shared between ISR and main loop: `target_speed_1/2`, `speed_1/2`, `tracker_value[]`, `tracking_active`, `straight_enc_acc`, `g_yaw`, `g_raw_yaw`, `sys_tick_ms`, encoder counters.
- `extern volatile uint32_t sys_tick_ms` is declared in `main.c` (SysTick ISR) and used across modules for timing — don't redeclare it as non-volatile.
- The `Debug/` directory is in `.gitignore` but `.ccsproject`, `.cproject`, `.project`, `.settings/` are also gitignored — the canonical project spec is `empty.syscfg`. To open the project in CCS, import the `empty.syscfg` or the `.project` file.
- `user_driver/` contains all custom code. Third-party MPU6050 DMP code (`inv_mpu.c`, `inv_mpu_dmp_motion_driver.c`, `dmpKey.h`, `dmpmap.h`) should be treated as vendored — avoid modifying.
