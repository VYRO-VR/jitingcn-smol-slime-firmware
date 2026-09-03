# Receiver contract: yaw drift firmware changes

Hand-off for `VYRO-VR/SlimeVR-Tracker-nRF-Receiver`. This describes the tracker-side
changes made in `VYRO-VR/jitingcn-smol-slime-firmware` for Phase 1b and Phase 2 of
`yaw-drift-handoff.md`, and exactly what the receiver has to implement to match.

Everything here is **additive**. No existing packet length, sub-packet layout or
composite framing changes, so the receiver update can ship independently of the
tracker firmware and an un-updated receiver keeps working.

---

## 1. New PONG remote commands (host → tracker)

Two new flags in PONG `data[7]`, alongside the existing ones in
`src/connection/esb.h`:

| Flag | Value | Meaning |
|---|---|---|
| `ESB_PONG_FLAG_MAG_HOLD` | `0x27` | Stop the fusion filter trusting the magnetometer |
| `ESB_PONG_FLAG_MAG_UNHOLD` | `0x28` | Resume normal magnetometer fusion |

Neither carries any PONG payload bytes — `data[3..11]` are untouched, so the
normal time-sync path applies. The values sit outside `0x30-0x33`, which the OTA
range check executes immediately; these go through the standard ~1500 ms deferred
execution like every other command.

Acknowledgement is the existing generic echo: the tracker returns the same value
in PING `data[7]` until the receiver sends `NORMAL` again. No new ack handling.

**What the receiver needs:**
- Emit both flags in PONG.
- Expose them over the HID command protocol, next to the existing sens-auto opcode
  (`rcv_cmd_remote_sens_auto_hid`, `rcv_cmd.c:481`).
- Expose them over the receiver's serial console, e.g. `send <id> mag hold on|off`.

**What the hold does on the tracker** (context, not receiver work): the fusion
backend skips the magnetometer update entirely — no heading correction, no
new-field adoption — and online mag calibration stops collecting samples and
stops committing to flash. Unlike `MAG_ON`/`MAG_OFF` there is no fusion restart,
no orientation glitch and no NVS write, and the hold is **not persisted**: a
reboot clears it. This is the actuator Phase 3 (server-side cross-tracker mag
corroboration) should use — never `setMag`.

Side effect worth knowing about at the host: while held, the tracker reports mag
disturbance continuously, so the existing temperature-byte sign-flip
(`connection_update_sensor_temp`) shows "disturbed" for the duration of the hold.

---

## 2. New uplink sub-packet type 6 (tracker → host)

Reports the progress and outcome of a gyro sensitivity auto-calibration
(`sens auto`), which previously existed only as `printk` output on the tracker's
own serial console. This is what replaces the timeout-based inference described
under "Known gap" in `yaw-drift-handoff.md`.

### Framing

**The tracker sends type 6 as a standalone packet only. It is never placed inside
a composite (`0xFE`) frame.** A receiver that does not know a sub-packet type
cannot skip it and would mis-offset every following sub-packet in the composite;
a standalone 17-byte ESB frame is self-delimiting, so an un-updated receiver just
drops it. Keep it that way — do not add type 6 to the composite length table
expecting the tracker to piggyback it.

The frame is the usual legacy 16-byte layout, `data[0] = 6`, `data[1] = tracker id`,
payload from `data[2]`, `data[15] = 0`:

```
| b0 | b1 | b2    | b3     | b4   | b5  | b6..b7    | b8..b9   | b10..b15 |
| 6  | id | phase | result | axis | seq | scale_q12 | progress | resv (0) |
```

Payload length (the `SUB_DATA_LEN_SENS_CAL` value) is **7**. `scale_q12` and
`progress` are little-endian `uint16`.

### Fields

- `phase` — `enum sens_cal_phase`
- `result` — `enum sens_cal_result`, `NONE` while a run is still in progress
- `axis` — `0 = X`, `1 = Y`, `2 = Z`
- `seq` — incremented once per completed run; use it to tell a fresh result from
  a repeat of the previous one
- `scale_q12` — computed scale in Q12: `scale = scale_q12 / 4096.0`. `0` means
  not applicable. Reported even for rejected runs, so the host can show *why* a
  run was rejected.
- `progress` — integrated absolute rotation in whole degrees, saturating at
  65535. Divide by 360 for the live turn counter. Measured by the uncalibrated
  gyro, so it is good to roughly 1% — fine as a progress dial.

### Enum values (wire contract — never renumber, only append)

```
enum sens_cal_phase {
    SENS_CAL_PHASE_IDLE       = 0,
    SENS_CAL_PHASE_HOLD_STILL = 1,  /* LED long pattern */
    SENS_CAL_PHASE_BIAS       = 2,
    SENS_CAL_PHASE_ARMED      = 3,  /* LED flashing: spin now */
    SENS_CAL_PHASE_RECORDING  = 4,  /* LED solid */
    SENS_CAL_PHASE_DONE       = 5,  /* terminal, see result */
};

enum sens_cal_result {
    SENS_CAL_RESULT_NONE            = 0,  /* still running */
    SENS_CAL_RESULT_OK              = 1,
    SENS_CAL_RESULT_INVALID_PARAMS  = 2,
    SENS_CAL_RESULT_NOT_STILL       = 3,
    SENS_CAL_RESULT_GYRO_TIMEOUT    = 4,
    SENS_CAL_RESULT_NO_BIAS_SAMPLES = 5,
    SENS_CAL_RESULT_NO_SPIN         = 6,
    SENS_CAL_RESULT_SPIN_TIMEOUT    = 7,
    SENS_CAL_RESULT_ANGLE_TOO_SMALL = 8,
    SENS_CAL_RESULT_INVALID_SCALE   = 9,
    SENS_CAL_RESULT_OFF_AXIS        = 10,
    SENS_CAL_RESULT_SCALE_RANGE     = 11,
    SENS_CAL_RESULT_NO_RETAINED     = 12,
};
```

### Cadence

2 Hz while a run is in progress, plus a 10 s linger after a terminal result so a
host that was not listening at the moment of completion still sees the outcome.
Nothing is sent when idle. Because a calibration spin keeps quaternion data ready
on nearly every pass, the report preempts one quat frame rather than waiting for
an idle slot — expect a negligible gap in the fusion stream during a run.

**What the receiver needs:** recognise standalone packet type 6, parse the 7-byte
payload, and forward it to the host over HID.

---

## 3. How Preflight consumes this

- `phase` drives the on-screen phase machine (hold still → bias → spin now →
  recording → complete), mirroring the tracker's LED patterns.
- `progress / 360` drives the live turn counter ("7.2 / 10 turns").
- `result` plus `seq` gives pass/fail and the computed scale for the completion
  screen, replacing the phase-timeout inference.
- The verification spin stays pure GUI math — it measures what the firmware
  cannot: the residual error after the correction is applied.

Start a run with the existing `ESB_PONG_FLAG_SENS_AUTO` (`0x24`), which already
carries the axis in PONG `data[3]` and a big-endian `uint16` revolution count in
`data[4..5]`. Use 10 revolutions, not the default.
