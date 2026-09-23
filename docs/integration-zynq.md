# Zynq acquisition and attitude integration

This guide connects board-specific acquisition to the C99 estimator. The library contains no SPI, FPGA, interrupt, storage, or scheduler implementation. Its public contract is in `include/imu/imu.h`, `imu_types.h`, and `imu_sensor_port.h`. The caller owns one persistent `imu_t`, its configuration, calibration record, input frame, and output storage. Serialize calls that touch the same `imu_t`.

## Timing and sensor setup

Run a precise host timer at **485 Hz** (nominal period `1/485 s = 2.061855670 ms`). Timestamp the common register-latch/reference event with a monotonic `uint64_t` nanosecond clock. The period is a scheduling target: `imu_process` derives each `dt` from successive timestamps. Do not use wall-clock time, a hard-coded `dt`, or an adjusted timestamp that has already had filter delay subtracted.

Each ADXRS453 updates from its own internal clock at `f0/32`, typically **about** 485 Hz. This internal rate is not phase locked to the Zynq 485 Hz trigger. Assert all three gyro chip selects simultaneously to latch their rate registers at a common instant. Three parallel MISO lines and a shared clock can read the latched words concurrently. Simultaneous register latch **does not synchronize the three ADXRS453 internal MEMS clocks**. If hardware reads them sequentially, retain each CS assertion time in `imu_acquisition_t.gyro_latch_timestamp_ns[]` and correct the resulting skew in the acquisition adapter before assembling the common-frame `imu_sample_t`; that core sample cannot encode three separate gyro latch epochs. Do not claim three-axis simultaneous physical samples.

The ADXRS453 uses a **one-exchange response pipeline** (Rev. B, pp. 14–17, Figures 28–29): exchange `A_k` asserts CS at host time `t_k`, latches rate data, and transmits the response to the **previous** command. After the complete 32-clock command and CS deassertion, exchange `B_k` returns the data and status latched for `A_k`, while itself latching another set. Allow at least the specified 0.1 µs sequential-transfer delay between CS deassertion and reassertion. The response to the device's first command is `0x00000001`; discard it as pipeline priming, never as a zero-rate sample. If `A_k` and `B_k` are both issued every host tick, discard or separately account for `A_k`'s response to `B_(k-1)`; use only `B_k`'s response for frame `t_k`. The three `B_k` response words must echo the three `A_k` sensor-data command sequence tags (`SQ[2:0]`), have valid response parity (`P0` for bits 31:16 and `P1` for all 32 bits), `ST[1:0]=01` for valid sensor data, and no indicated fault. Check the SPI/request/data-unavailable error response and PLL, power, self-test, and other relevant fault bits before setting `IMU_SENSOR_DATA_VALID`; map failures into the public status flags. Sequence echo checks command/response association; it does **not** synchronize internal MEMS sampling. A driver using one exchange per tick must instead queue the *prior* latch epoch and pair that returned rate with acceleration from the same prior epoch, never with the current tick's accelerometer or timestamp.

Configure ADXL355 `ODR_LPF=0x3` (**500 Hz/125 Hz**) and `EXT_SYNC=10`, `EXT_CLK=0` (external sync with interpolation and the internal clock), and drive its SYNC input from the precise **485 Hz** Zynq timing source. With default `INT_MAP[7:4]=0000`, route **SYNC to pin 14** (the multiplexed DRDY pin) and the active-high **DRDY output to pin 13** (the multiplexed INT2 pin); do not use the ordinary internal-sync DRDY pin assumption. At `ODR_LPF=0x3`, SYNC-to-DRDY latency is **22 internal oscillator cycles** (Rev. D, pp. 32–34, Tables 13–14). Wait for the DRDY corresponding to the current SYNC edge before reading the three-axis sample, and associate that reading with that edge's host epoch. DRDY clears on a FIFO or X/Y/Z data-register read, or autoclears about halfway to the next output set; avoid treating a repeated register read as a fresh sample. After the **first** SYNC edge (and after any resynchronization), the first **one to three** DRDY/sample slots may be lost or corrupted within one group delay. Conservatively discard the first **three** slots, consuming any ready data to clear DRDY while marking acceleration not fresh; do not begin normal alignment on them. Disable the high-pass filter so dc gravity remains available. Verify the configured registers, pins, DRDY edges, and SYNC waveform on the board. The accelerometer is synchronized to the host signal; the gyros are not.

Use measured board signal-chain delays where available. Nominal starting values are **4.00 ms** for each ADXRS453 rate path and **5.27 ms** for the ADXL355 interpolation/decimation path. Set `config.gyro_delay_s[0..2]` and `config.accel_delay_s` before `imu_init`. The difference, about 1.27 ms with these nominal values, is within the default `max_delay_compensation_s=0.01`. The library applies a bounded constant-rate rotation approximation between acceleration and gyro ages. Its defaults leave all delays at zero, so explicitly configure them; do not subtract the same delay in the driver. The latch timestamps remain in the same monotonic epoch as `sample.timestamp_ns`; the adapter must account for sequential-read skew. These group delays describe signal filtering, while per-device latch times describe acquisition timing.

## Build and frame contract

Compile the `imu` CMake library for the Zynq target as C99 with the target floating-point ABI. Allocate the `imu_t` in persistent application storage; avoid copying it in the scheduler. The library allocates no heap storage. Supply one complete `imu_sample_t` per host frame, including all temperatures, statuses, and freshness fields. Gyro rates are in **rad/s** and acceleration is in **m/s²**. Temperatures are in °C. Driver conversion from device words to SI units precedes calibration in the library. A level stationary sensor in the FRD body frame reports specific force approximately `[0, 0, -9.80665] m/s²`; feeding `[0, 0, +g]` reverses the attitude observation.

Body axes are **FRD**: x forward, y right, z down. Navigation axes are **NED**: north, east, down. `q_nb` is a scalar-first Hamilton quaternion that rotates body vectors into NED. `roll_rad`, `pitch_rad`, and `yaw_rad` use right-handed Euler signs from that quaternion. Check sensor mounting, ADXRS453 axis polarity, ADXL355 axis polarity, and the board-to-body rotation with known positive rotations before flight. Yaw is **relative and drifting** without a heading sensor; it is not north referenced.

The callback below shows the two-exchange gyro schedule. The first three-CS assertion is the common rate-latch/reference event; the second exchange returns those latched rate words. `decode_*` represents board code that fills **every** rate/acceleration, temperature, status, and freshness field, and retains per-device latch timestamps. Keep a command sequence tag for each of the three gyros. The raw output of an acquisition port with return `IMU_STATUS_SENSOR_DEGRADED` still supplies a complete frame; other acquisition errors leave its output unchanged and require the caller to form an explicitly degraded frame or skip the estimator call. The illustrative `publish_only_if_status_allows` follows the policy in the next section.

```c
on_485_hz_tick(uint64_t timestamp_ns) {
    imu_sample_t sample = {0};
    imu_output_t output;
    imu_status_t status;
    gyro_response_t prior_response[3], latched_response[3];
    uint8_t latch_sequence[3], next_sequence[3];

    /* Exchange A_k: simultaneous CS falling edges latch rate at timestamp_ns.
       MISO here belongs to the prior command; first-ever MISO is 0x00000001. */
    choose_sequence_tags(latch_sequence);
    issue_three_sensor_data_commands(latch_sequence, prior_response);
    discard_or_account_for_prior_response(prior_response);
    wait_at_least_0_1_microsecond_after_cs_high();
    /* Exchange B_k: MISO now returns A_k's latched rate and SQ tags. */
    choose_sequence_tags(next_sequence);
    issue_three_sensor_data_commands(next_sequence, latched_response);
    decode_and_check_gyro_responses(latched_response, latch_sequence,
                                    sample.gyro_rad_s, sample.gyro_status,
                                    sample.gyro_sample_fresh);
    /* SYNC_k occurred at timestamp_ns. DRDY_k follows it by 22 oscillator
       cycles; the adapter consumes but rejects the first three startup slots. */
    if (wait_for_matching_adxl355_drdy_before_next_tick(timestamp_ns))
        read_and_classify_synced_adxl355(sample.accel_m_s2,
                                        &sample.accel_status,
                                        &sample.accel_sample_fresh);
    else
        mark_accel_not_fresh(&sample);
    decode_temperatures_and_accel_freshness(&sample);
    sample.timestamp_ns = timestamp_ns;
    status = imu_process(&imu, &sample, &output);
    publish_only_if_status_allows(status, &output);
}
```

In production, capture `timestamp_ns` at the actual timer/first-CS/SYNC reference event, not after SPI reads or DRDY. Preserve all gyro and accelerometer latch/ready times in an `imu_acquisition_t` when implementing `imu_sensor_port_t`. The accelerometer reading must be the sample made ready for that same SYNC epoch; if its matching DRDY is missed, mark acceleration not fresh or skip the frame. Do not pair a delayed gyro response with a later accelerometer sample. It is acceptable for a newly read gyro register word to be *stale* because its internal sensor has not updated since the previous host tick; mark its freshness false. A new SPI transaction or incremented SPI command sequence is not proof of a new MEMS measurement. Never duplicate a value while marking it fresh. If the board cannot identify a new internal gyro update, define and validate a conservative freshness strategy from hardware observations; do not invent data-ready behavior the ADXRS453 lacks.

`IMU_SENSOR_DATA_VALID` must be set **and** all bits in `IMU_SENSOR_FAULT_MASK` clear, independently for each channel, before a fresh reading is usable. Propagate the device/adapter diagnostics exactly:

| Input bit | Driver meaning |
| --- | --- |
| `DATA_VALID` | Positive indication that the decoded value is valid. |
| `PARITY_ERROR` | SPI parity check failed. |
| `SEQUENCE_ERROR` | Gyro command/response sequence check failed. |
| `PLL_ERROR` | Sensor clock/PLL diagnostic failed. |
| `POWER_ERROR` | Supply or power diagnostic failed. |
| `SELF_TEST_ERROR` | Self-test/health diagnostic failed. |
| `FIFO_ERROR` | Accelerometer FIFO integrity/overrun failed. |
| `OVERRANGE` | Sensor measurement outside valid range. |

Do not use the gyro sequence bit as a clock-synchronization indicator. Nonfinite decoded numbers are invalid; reject or mark the frame unusable before estimation. `gyro_sample_fresh[3]` and `accel_sample_fresh` refer to newly acquired **sensor measurements**. A stale accelerometer does not update gravity qualification. An unhealthy gyro axis prevents propagation using that frame; missing time is consumed once and covariance grows instead of replaying the last rate. `imu_output_t` echoes the channel status words.

## Startup and publication

1. Load a versioned calibration record, verify its serialized checksum in the storage layer, set `checksum_valid`, and validate its temperature range and validity marker. Calibrate individual gyro scales/orthogonality and mounting, temperature-dependent gyro bias, accelerometer scale/nonorthogonality/offset, and board-to-FRD rotation. The transform is `body_rotation * (scale_alignment * raw - bias(T))`. Bias polynomials use the record reference temperature and coefficients in SI units. The library validates the record at `imu_init`; a bad record blocks normal startup. Set `allow_uncalibrated=true` only for an explicit diagnostic mode, and observe `IMU_HEALTH_UNCALIBRATED` in output. If temperature extrapolation is permitted, output marks out-of-range use with `IMU_HEALTH_TEMPERATURE_EXTRAPOLATED`.
2. Call `imu_config_defaults`, set measured delay values, calibrated noise/quality parameters and optional premeasured notches, then select initialization. `IMU_INIT_QUATERNION` accepts a known `initial_q_nb`; even then, `imu_process` waits for a healthy, fresh frame before reporting initialized. `IMU_INIT_STATIONARY` requires a static dwell followed by `alignment_samples` consecutive qualified frames; it averages gravity for roll/pitch and gyro readings for initial bias. Caller-supplied `initial_yaw_rad` sets only a relative yaw origin. A single accelerometer sample is never a high-confidence alignment.
3. Call `imu_init(&imu, &config, &calibration)` and require `true`. Feed complete frames with strictly increasing nanosecond timestamps. The first accepted frame establishes the time epoch and integrates no nominal interval. During alignment, expect `IMU_STATUS_NOT_INITIALIZED` and `IMU_FUSION_ALIGNING`. Do not publish attitude until `output.initialized` is true. If persistent bad timing or a numerical fault warrants restart, quiesce the consumer and call `imu_reset`, then repeat alignment; a hard numerical fault latches until reset/reinitialization.
4. On each returned status, publish only an output actually committed by `imu_process`. `IMU_STATUS_OK` with `initialized=true` is the normal attitude update. `IMU_STATUS_SENSOR_DEGRADED` or `IMU_STATUS_TIME_GAP` may carry a committed initialized output; a bad accelerometer can leave valid gyro propagation intact, whereas a gyro dropout or time gap freezes unobserved attitude motion and grows uncertainty. Publish degraded output only with its status/health flags under an application-defined degraded-data policy. `IMU_STATUS_NOT_INITIALIZED` is alignment data, not flight attitude. `IMU_STATUS_BAD_ARGUMENT`, `IMU_STATUS_NONFINITE`, and `IMU_STATUS_TIME_REVERSED` do not commit a new state/output; never publish an uninitialized local `output` on those returns. `IMU_STATUS_NUMERICAL_FAULT` marks invalid output and requires controlled reset. `IMU_STATUS_AIDING_UNAVAILABLE` is the magnetometer-stub result, not a normal `imu_process` frame result. If a time step exceeds `max_dt_s` (default 20 ms), `TIME_GAP` takes precedence and the interval is consumed without integrating unobserved rotation; a later fresh frame integrates only its own interval. A step below `min_dt_s` (default 0.5 ms) returns `BAD_ARGUMENT` without changing state.

The output includes `fusion_mode` (`INVALID`, `ALIGNING`, `GYRO_ONLY`, `GRAVITY`, or `STATIC`), `accel_weight`, bias-corrected angular rate, estimated residual gyro bias, covariance-derived one-sigma roll/pitch/yaw uncertainty, `status`, `timing_status`, `calibration_valid`, and these health bits:

| Output bit | Interpretation |
| --- | --- |
| `IMU_HEALTH_GYRO_DEGRADED` | One or more gyro measurements unusable. |
| `IMU_HEALTH_ACCEL_DEGRADED` | Accelerometer observation unusable or rejected. |
| `IMU_HEALTH_TIMING_DEGRADED` | Input interval exceeded configured maximum. |
| `IMU_HEALTH_UNCALIBRATED` | Explicit diagnostic calibration fallback. |
| `IMU_HEALTH_NUMERICAL_FAULT` | Latched estimator numerical fault. |
| `IMU_HEALTH_HEADING_UNOBSERVABLE` | Absolute heading is unavailable. |
| `IMU_HEALTH_TEMPERATURE_EXTRAPOLATED` | A sensor temperature is outside the calibrated range under allowed extrapolation. |
| `IMU_HEALTH_UNCERTAINTY_UNAVAILABLE` | `attitude_std_rad` contains finite `FLT_MAX` sentinels, not standard deviations. |

`imu_update_magnetometer` currently validates pointers and finite field components, then returns `IMU_STATUS_AIDING_UNAVAILABLE` without changing estimator state. Neither a fresh magnetic sample nor a healthy status enables heading aiding. Treat yaw and its uncertainty as relative until a future implemented heading observation is explicitly verified.

## Bring-up checks

Log the frame and individual latch timestamps, raw converted SI readings, freshness/status, estimator return, output health and fusion mode, and config/calibration version. Confirm a stationary level sign test, known-axis rotations, measured timer jitter and acquisition skew, actual ADXL355 register state and SYNC edge, and the temperature range. Replay a captured log with `tools/imu_replay`; without independent attitude truth, replay cannot establish dynamic accuracy. Measure execution time, memory/stack headroom, and board temperature behavior on the Zynq target before enabling a flight-control consumer. The current host verification result and its failing combined-scenario gate are recorded in `docs/verification-report.md`.
