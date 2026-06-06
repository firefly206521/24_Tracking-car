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
2. Init OLED (I2C1), buzzer, motors (2× DC with TB6612 + quadrature encoders), MPU6050 gyro (I2C0).
3. Enable interrupts: encoder edges, buttons (KEY_1 on PB7, KEY_4 on PB6), GPIOA/GPIOB shared lines.
4. Loop: read MPU6050 yaw → calibrate → display yaw on OLED → `status_run(yaw)`.

### State machine (`user_driver/status.c`)
`sys_status` (enum `system_status_t`) controls operating mode:

| Mode | Enum | Behavior |
|------|------|----------|
| 0 | `STATUS_IDLE` | Motors off, wait for button |
| 1 | `STATUS_DIST` | Straight 1650 encoder pulses (avg of L/R), then hard brake + buzzer beep |
| 2 | `STATUS_LINE_TRACK_2` | Straight until line detected → track line (uses `pid_line`, ISR-dispatched via `question2_run`) |
| 3 | `STATUS_LINE_TRACK` | Full Q3 course: straight→align→curve→straight→align→curve, loops with gyro hemisphere-reversal counting. Stops after `S3_REVERSALS_Q3` (2) reversals when off-line. Uses `pid_line`. |
| 4 | `STATUS_LINE_TRACK_2ND` | Same state machine as mode 3, but stops after `S3_REVERSALS_Q4` (8) reversals. Uses `pid_line_q4`. |

Modes 3/4 share a 7-state sub-machine: `S3_STRAIGHT1 → S3_ALIGN1 → S3_CURVE1 → S3_STRAIGHT2 → S3_ALIGN2 → S3_CURVE2 → (loop)`. Key mechanisms:
- **Gyro-stabilized straight:** ramps to `S3_BASE_SPEED` (600), PID-corrects against a pre-turned reference yaw. On timeout (190 ticks without line), drops speed to 200 and starts a force-turn timer (`S3_FORCE_TURN_DELAY_1/2` ms) that sets a forced yaw target beyond horizontal — guarantees the car turns even if line is never seen.
- **Hemisphere reversal counting:** tracks whether yaw relative to init yaw is in front (≤90°) or rear (>90°). Each hemisphere flip increments `s3_reversal_count`. Stop condition: `reversal_count >= target AND off-line`.
- **Line debounce:** `S3_LINE_DEBOUNCE` (3) consecutive matching readings before `s3_on_line` changes state.
- **Align boost:** PID correction is multiplied by `S3_ALIGN_BOOST` (1.5×) during alignment phases for faster heading correction.
- **Curve tracking:** calls `tracker_pid()` at reduced speed (300, ramping up), with pre-loaded integral bias (`S3_TRACK_I_INIT` = -20) to help the car turn into the curve.

- `status_cycle_next()` — KEY_1 cycles mode (wraps around `STATUS_COUNT`), resets all init flags and PID state.
- `status_toggle_start()` — KEY_4 toggles start/stop, zeros yaw reference via `mpu_reset_zero()`.

### Motor control (`user_driver/motor.c`)
- **PID Timer ISR** (`MOTOR_PID_INST_IRQHandler`): fires every 30ms. Calculates speed from encoder counts (260 lines, tire radius 48mm), applies low-pass filter, runs motor PID (positional: Kp=12, Ki=2). Writes PWM duty via `DL_PWM_setCaptureCompareValue`.
- **Speed units:** mm/s. `target_speed_1` = right motor, `target_speed_2` = left motor.
- **Hard brake:** reverses motor direction at 100% PWM for 8 ISR ticks (~240ms), then shorts windings (`motor_brake`). `motor_hard_brake_reset()` clears the brake state machine.
- **Mode 2 dispatch:** `question2_run()` runs inside ISR at 20Hz — switches between `straight_run()` and `track_line()` based on `tracking_active` and line sensor state.
- **Encoder IRQ routing:** `PA21` (encoder1 rising edge) and `PA22` (encoder1 direction) are in GPIOA. `PB19` (encoder2 rising edge) and `PB20` (encoder2 direction) are in GPIOB. All share `GROUP1_IRQHandler` in `key.c`.

### Interrupt architecture
All GPIO interrupts share a single handler: `GROUP1_IRQHandler` in `key.c`. It switches on `DL_GPIO_getPendingInterrupt(GPIOB)` for KEY_1, KEY_4, and encoder2, then on `DL_GPIO_getPendingInterrupt(GPIOA)` for encoder1. Button debounce is 150ms. The NVIC enables `MOTOR_EC1A_IIDX`, `MOTOR_EC2A_IIDX`, `KEY_KEY_1_IIDX`, `GPIO_MULTIPLE_GPIOB_INT_IRQN`, and `MOTOR_GPIOA_INT_IRQN`.

### Line tracker (`user_driver/tracker.c`)
7-channel IR sensor array (L0–R0). `tracker_value[]` = 1 when over black line (active-low GPIO).
- `compute_error()` — weighted centroid (-1.0 to +1.0). Returns `last_error` when no sensors see the line.
- `tracker_pid(base_speed, pid)` — applies PID correction to `target_speed_1/2`, respects MIN_SPEED (50) and MAX_SPEED (1200).
- Two separate PID instances: `pid_line` (Kp=900, Ki=10, Kd=400) for Q3, `pid_line_q4` (Kp=850, Ki=10, Kd=600) for Q4.
- `track_line()` — convenience wrapper that uses default `pid_line` at `BASE_SPEED` 500.

### Gyro-stabilized straight (`user_driver/straight.c`)
Uses MPU6050 yaw for heading correction with a separate PID (Kp=2, Ki=0.3).
- Ramps speed up to `STRAIGHT_BASE_SPEED` (800).
- `straight_enc_acc` accumulates total encoder pulses (never reset by user code — only zeroed on `straight_begin`).
- Two API layers: ISR-pathed (`straight_begin/run`, sets `straight_active`, has exit at `STRAIGHT_DIST_MAX`) and main-loop-pathed (`straight_nav_begin/run/update_ref`, no exit, used by mode 2 ISR dispatch).
- `straight_line_detected()` — returns 1 if any tracker sensor sees the line.
- `straight_get_distance()` — returns `straight_enc_acc`, used by mode 1 to check pulse count.
- `straight_force_stop()` — zeros target speeds and clears `straight_active`.

### MPU6050 (`user_driver/mpu_port.c`, `user_driver/mpu_nav.c`, `user_driver/inv_mpu*.c`)
- InvenSense DMP driver (`inv_mpu.c` + `inv_mpu_dmp_motion_driver.c`) provides quaternion-based yaw.
- `mpu_nav.c` wraps calibration: auto-calibrates first 50 samples, or `mpu_reset_zero()` for instant zero.
- `g_yaw` = calibrated yaw, updated in main loop. `g_raw_yaw` = raw reading.

### PID utilities (`user_driver/pid_utils.c`)
Reusable positional PID with integral clamping (`integral_max`). `pid_compute()` returns output, `pid_reset()` zeros error/integral. Also provides `normalize_angle()` (to [-180,180]) and `clamp_value()`.

### Buzzzer (`user_driver/buzzer.c`)
Active-low buzzer + LED. `buzzer_beep()` activates buzzer and LED; they auto-off after 500ms via `buzzer_tick()`. Note: `buzzer_tick()` is defined but not currently called from main loop — buzzer/LED currently latch on indefinitely after beep (intended?).

### Idle state (`user_driver/idle.c`)
`stay_idle()` — zeroes target speeds, disables motor directions, resets encoders/integrals, clears `tracking_active`, calls `straight_force_stop()`, updates OLED.

### `change` variable
Global `uint8_t change` (in `motor.c`) counts line-edge transitions. Incremented in `status_run` when debounced line state changes, each time triggering `buzzer_beep()`.

## Key timing constraints
- **Main loop** must read MPU6050 and update OLED fast enough to not miss yaw samples. Do NOT add blocking delays in the main loop.
- **PID ISR** runs every 30ms. Motor speed calculation and PID must complete within this window. `question2_run()` is called at 20Hz inside this ISR.
- **SysTick** at 1ms provides `sys_tick_ms` for timeouts.
- MPU6050 DMP reads are blocking I2C — the `while(DMP_Read_Data(...))` spin-waits for DMP FIFO.

## Pin map (quick reference)

| Peripheral | Pins |
|------------|------|
| PWM (motors) | PA12 (right), PA13 (left) |
| Motor direction | PA8/PA9 (right), PA7/PA18 (left) |
| Motor STBY | PB24 |
| Encoders | PA21 (right A), PA22 (right B), PB19 (left A), PB20 (left B) |
| I2C OLED | PB2 (SCL), PB3 (SDA) |
| I2C MPU6050 | PA1 (SCL), PA0 (SDA) |
| Line tracker | PB8(L1), PA17(L2), PB9(MID), PA2(R1), PA24(R2), PA25(R0), PA15(L0) |
| Buttons | PB7 (KEY_1 — cycle mode), PB6 (KEY_4 — start/stop) |
| Buzzer | `buzzer_BUZZER_PIN` (SysConfig-configured) |
| UART debug | PA28 (TX), PA31 (RX) |

## Development notes

- `volatile` globals shared between ISR and main loop: `target_speed_1/2`, `speed_1/2`, `tracker_value[]`, `tracking_active`, `straight_enc_acc`, `g_yaw`, `g_raw_yaw`, `sys_tick_ms`, `change`, encoder counters.
- `extern volatile uint32_t sys_tick_ms` is declared in `main.c` (SysTick ISR) and used across modules for timing — don't redeclare it as non-volatile.
- The `Debug/` directory is in `.gitignore`. `.ccsproject`, `.cproject`, `.project`, `.settings/` are also gitignored — the canonical project spec is `empty.syscfg`. To open the project in CCS, import the `empty.syscfg` or the `.project` file.
- `user_driver/` contains all custom code. Third-party MPU6050 DMP code (`inv_mpu.c`, `inv_mpu_dmp_motion_driver.c`, `dmpKey.h`, `dmpmap.h`) should be treated as vendored — avoid modifying.
- `GROUP1_IRQHandler` in `key.c` handles ALL GPIO interrupts (buttons + both encoders). If adding new GPIO interrupts, add cases here — do not create a separate handler.
