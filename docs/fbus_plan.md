# FBUS servo-bus driver for PX4 — implementation plan

Branch `fbus`, based on upstream tag `v1.18.0-beta2` (83c4f4e). Target board: px4_fmu-v6x.

Goal: drive FrSky Xact servos (W56xx) over FBUS (F.Port v2) from PX4 as a first-class
actuator output driver — like the DroneCAN servo path, not the PWM pins — with per-servo
current / voltage / temperature telemetry logged and streamed over MAVLink.

## 1. Protocol background

FBUS is a single-wire half-duplex inverted UART bus at 460800 baud. The master transmits,
at a fixed frame rate, one CONTROL frame (16 x 11-bit channels, SBUS packing) followed
back-to-back by one DOWNLINK slot that either polls one sensor (round-robin) or idles.
A polled servo answers 500-3000 us later with a 10-byte uplink (current 0.1 A, voltage
0.1 V, temperature 1 degC packed in appId 0x6800+servoId). Servo configuration
(physical ID, servo ID, range, center, direction, data rate) is a read/write/save
exchange on the same bus, one servo connected at a time.

A working master implementation (faithful Rotorflight port, bench-validated on a
Teensy 4.0 + Xact W5651) exists in the MFCA firmware repo: `lib/fbus/FBusMaster.{h,cpp}`
plus protocol/evaluation docs in `docs/fbus/`. That code is the porting source for the
protocol core here.

Key protocol facts that shape the design:

- Channel values are microseconds with the standard FrSky SBUS line
  (us = sbus / 1.6 + 880), clamped to 1000..2000 us. Neutral 1520 us, 10 us/deg in
  1520us/333Hz mode. Servo internal update limit: 333 Hz (1520 us mode).
- There is NO disarm on the bus: a servo arms on its first CONTROL frame and never
  returns to limp. Silence keeps an un-armed servo limp; a real disarm after arming
  requires cutting the servo power rail.
- Control and telemetry share the same frame: no listen-only mode.

## 2. What the PX4 v1.18 audit established

- No FBUS/F.Port anything in-tree; no FMU-side SBUS output driver either
  (`sbus1_output` is px4io-firmware only). This driver is the first serial servo bus
  on the FMU side.
- Output framework: inherit `OutputModuleInterface`, own a
  `MixingOutput(param_prefix, N, *this, policy, false, false)`; declare a
  `actuator_output` group in `module.yaml`. Everything else — per-channel
  FUNC/DIS/MIN/CENT/MAX/FAIL/REV params, QGC Actuators UI, actuator_test,
  arming/lockdown/termination handling, -1..1 -> us scaling — is generated/handled.
  The driver implements `updateOutputs(float outputs[16], num, has_updates)` receiving
  values already scaled to us.
- Scheduling: `SchedulingPolicy::Auto` = `actuator_servos` uORB callback (rate-loop
  rate, variable). For the deterministic FBUS frame rate use
  `SchedulingPolicy::Disabled` + `ScheduleOnInterval()` (in-tree precedent:
  `pca9685_pwm_out`).
- Templates: `src/drivers/actuators/vertiq_io` (serial_config + actuator_output +
  round-robin poll embedded in the control frame + disarm-behavior params),
  `src/drivers/actuators/voxl_esc` (hot path: write frame, then non-blocking read of
  the PREVIOUS cycle's reply — the pipelined half-duplex turnaround; `device::Serial`;
  `px4::serial_port_to_wq()`), `src/drivers/dshot/DShotTelemetry` (baud-aware
  early-out, online debounce, adaptive skip of absent responders with disarmed retry).
  Known vertiq_io defects not to copy: FIONREAD into a uint8_t (OOB write),
  online/armed flags never cleared.
- Servo feedback does not exist in PX4: no ServoStatus message, and even the DroneCAN
  servo controller has no status consumer. `esc_status` is the only per-actuator
  telemetry topic and the MAVLink ESC streams filter it to motor functions 101-112.
- Physical layer is ready: `device::Serial` supports 460800, `setSingleWireMode()`
  (TIOCSSINGLEWIRE), `setInvertedMode()` (TIOCSINVERT; no in-tree caller yet) and
  `setSwapRxTxMode()`. fmu-v6x defconfig enables `STM32H7_USART_SINGLEWIRE / INVERT /
  SWAP` chip-wide: every UART qualifies.
- fmu-v6x ports: GPS1=ttyS0, TEL3=ttyS1, console=ttyS2, EXT2=ttyS3(UART4),
  TEL2=ttyS4(UART5, DMA, 10 KB TX buffer — recommended), RC+PX4IO=ttyS5 (avoid),
  TEL1=ttyS6, GPS2=ttyS7. The bus lives on the port's TX pin (STM32 half-duplex).
- Echo warning: the Teensy LPUART disconnects its receiver while transmitting (no
  echo), so the Teensy port removed Rotorflight's echo filter. STM32 half-duplex
  (HDSEL) loops TX into RX — the echo is expected to come back. The PX4 port
  re-introduces the transmitted-byte-count echo filter, made adaptive via raw
  rx/tx byte counters (rawRx ~= tx -> echo present; rawRx ~= 0 -> no echo).

## 3. Architecture

New driver `src/drivers/actuators/fbus`:

- `FbusServo` : `ModuleBase` + `OutputModuleInterface`, work queue
  `px4::serial_port_to_wq(device)`.
- `MixingOutput{"FBUS_SV", 8, *this, SchedulingPolicy::Disabled, false, false}` +
  `ScheduleOnInterval(1e6 / FBUS_RATE)`; default rate 500 Hz. `update()` reads the
  latest `actuator_servos` sample each cycle.
- `FbusProtocol.{hpp,cpp}`: platform-free protocol core ported from the Teensy
  FBusMaster — frame build/parse, FrSky CRC, phyId check bits, discovery scan
  0x00..0x1A, round-robin poll, Xact config read/write/save, adaptive echo filter.
  Unit-testable on native.
- Serial: `device::Serial` open sequence (voxl2_io pattern) + `setBaudrate(460800)` +
  `setSingleWireMode()` + `setInvertedMode(true)`. Port selected at runtime via the
  generated `FBUS_CFG` param (`serial_config` section in module.yaml); no default port.
- `updateOutputs()`: map channels with `isFunctionSet()`, clamp to 1000..2000 us,
  hand to the protocol core; write frame, then non-blocking read of the previous
  reply (voxl_esc pipeline).
- module.yaml `actuator_output`: prefix `FBUS_SV`, 8 channels, standard_params in us
  including `center` (3-point MIN/CENT/MAX interpolation — the DroneCAN servo path
  does not even have that).

Safety semantics:

- Bus SILENT until the first cycle where the system is armed, prearmed, or an
  actuator test is active. Servos on the pad stay limp; an FC reboot cannot re-arm
  them silently. Once the bus has started it keeps transmitting (disarmed ->
  `FBUS_SV_DISx`, default 1500 us; termination -> `FBUS_SV_FAILx`).
- Documented limitation: no limp over FBUS. A hard disarm needs a MOSFET on the
  servo rail. Single-wire bus = single point of failure for all attached servos.
- Servo-side `Range` (90/120/180 deg) is provisioned into each servo as a hard
  travel guard independent of the FC.

## 4. Telemetry (decision: proper topic from the start)

- New uORB messages `msg/ServoReport.msg` + `msg/ServoStatus.msg` (8 slots):
  per-slot `actuator_function` (Servo 201..215), position echo (last commanded us),
  voltage V, current A, temperature degC, online flag, error counters
  (crc_errors, timeouts). Container: servo_count, online/armed bitmasks,
  connection type.
- Health: dshot-style — online after 100 ms of clean replies, offline after 500 ms
  staleness, adaptive skip of absent IDs with disarmed retry every 3 s.
- Logging: add `servo_status` to `logged_topics.cpp` (default topics, ~10 Hz interval).
- MAVLink: there is no standard servo-telemetry message. Add a custom `SERVO_STATUS`
  message: fork `mavlink/mavlink`, add the message to a dialect, point the
  `src/modules/mavlink/mavlink` submodule of this branch to the fork, add a
  `MavlinkStreamServoStatus` in `src/modules/mavlink/streams/`. The custom QGC build
  and the pymavlink-based GCS consume the same XML, so the message decodes in the
  MAVLink Inspector. (Stock QGC would show it as unknown — acceptable.)

## 5. Phases

- Phase 0 — environment: branch `fbus` checked out in the WSL clone, submodules
  synced to v1.18, user-local ARM GNU toolchain 13.3, reference build
  `make px4_fmu-v6x_default` green. This plan committed.
- Phase 1 — protocol core: port `FbusProtocol` from the Teensy FBusMaster + gtest
  unit tests (11-bit pack/unpack, CRC, phyId check bits, echo filter, poll/response
  state machine). Runs on native; CI-able.
- Phase 2 — output driver: module skeleton (module.yaml serial_config +
  actuator_output, Kconfig `DRIVERS_ACTUATORS_FBUS`, CMakeLists), 500 Hz loop,
  UART bring-up single-wire+inverted, control frames on the wire, silent-until-arm
  gate. Milestone: an Xact follows the QGC actuator test on TELEM2.
- Phase 3 — telemetry: discovery + round-robin poll, `servo_status` publication,
  health tracking, logger entry. Milestone: V/I/T per servo visible in the ulog.
- Phase 4 — MAVLink: mavlink fork + custom SERVO_STATUS + stream; visible in the
  MAVLink Inspector of the custom QGC.
- Phase 5 — safety polish + docs: failsafe/termination values on the bus, boot/arming
  event reporting, driver README, provisioning convention (physId 0x0C.., servoId =
  channel, Range minimal, data rate 100 ms — done on the Teensy bench or via
  optional `fbus cfg` CLI verbs, disarmed only).
- Phase 6 — bench validation: Xact W5651 on a v6x TELEM port; verify echo behavior,
  frame timing at 500 Hz, telemetry rates, us-equivalence (1520 = neutral) vs PWM.

## 6. Risks / open points

- STM32 half-duplex echo behavior under NuttX is reasoned, not yet measured — the
  adaptive filter covers both outcomes; verify at Phase 6.
- `setInvertedMode()` has no in-tree user yet; if the NuttX ioctl path misbehaves,
  fall back to the raw `TIOCSINVERT` ioctl (SbusRc pattern).
- Frame rate 500 Hz vs servo limit 333 Hz: the servo resamples; if bench shows
  aliasing artifacts, drop `FBUS_RATE` to 333.
- Custom MAVLink message = permanent submodule divergence from upstream; contained
  by keeping the message in one dialect file and the fork rebased on the same
  mavlink SHA the branch pins.
