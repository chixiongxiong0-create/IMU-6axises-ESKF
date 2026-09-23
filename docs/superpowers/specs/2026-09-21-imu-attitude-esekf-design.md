# ADXRS453 + ADXL355 6-Axis IMU Attitude Estimator Design

## 1. Purpose and Scope

This project implements a deterministic C99 attitude-estimation library for an IMU made from three orthogonally mounted ADXRS453 single-axis gyroscopes and one ADXL355 three-axis accelerometer.

The target platforms include aggressive FPV aircraft, three-axis gimbals on off-road climbing vehicles, and other high-vibration systems. The primary goals are maximum practical roll/pitch accuracy, robust behavior under vibration and linear acceleration, explicit uncertainty reporting, and clean future integration of a magnetometer.

The implementation contains only sensor-independent algorithm code and placeholder acquisition interfaces. SPI drivers, FPGA logic, interrupts, and board support are outside this project.

Performance is guaranteed only while each ADXRS453 remains within its specified useful angular-rate range. Gyroscope saturation recovery is out of scope.

## 2. Observable Quantities and Coordinate Conventions

The body frame is FRD: x forward, y right, and z down. The navigation frame is NED: north, east, and down. The unit quaternion `q_nb` rotates vectors from the body frame into the navigation frame. Euler output uses right-handed roll, pitch, and yaw derived from `q_nb`.

With no magnetic, visual, GNSS, or other heading observation, gravity makes roll and pitch observable but does not make absolute yaw observable. The library therefore:

- estimates roll and pitch using gyroscope propagation and qualified gravity observations;
- reports yaw as a relative angle that accumulates drift;
- propagates yaw uncertainty instead of applying a fictitious heading constraint; and
- exposes a magnetometer-update interface for a later heading observation implementation.

## 3. Sensor Constraints and Timing

### 3.1 ADXRS453

Each ADXRS453 produces temperature-compensated rate data at an internal update rate of `f0 / 32`, typically about 485 Hz. Its rate path has an approximately 77.5 Hz low-pass corner and approximately 4 ms group delay. The sensor has no external sampling clock or data-ready signal. Its SPI sequence bits echo command sequencing; they do not synchronize the three internal sensor clocks.

The Zynq must assert the three gyroscope chip-select signals at the same instant and, where hardware permits, clock three independent MISO lines in parallel. This gives a common register-latch time but does not make the three internal MEMS sampling phases identical. If parallel acquisition is unavailable, the acquisition layer records each chip-select assertion time separately.

### 3.2 ADXL355

The ADXL355 is configured with its internal 500 Hz output data rate and 125 Hz low-pass corner. It operates in external synchronization with interpolation (`EXT_SYNC = 10`) and receives a precise 485 Hz synchronization signal from the Zynq. In this configuration, the combined interpolation/decimation delay is approximately 5.27 ms.

The accelerometer high-pass filter remains disabled because attitude estimation requires the dc gravity component.

### 3.3 System Time Base

The Zynq provides a monotonic nanosecond time base and a strict 485 Hz algorithm trigger. The nominal period is approximately 2.061856 ms. The estimator always calculates `dt` from timestamps and never substitutes a hard-coded nominal period.

Every input reports its register-latch timestamp, freshness, and sensor status. The estimator associates measurements with their estimated physical measurement times after accounting for configured signal-chain delay. The initial implementation uses bounded delay compensation and covariance propagation; it does not claim that the ADXRS453 internal sampling phases are externally synchronized.

## 4. Selected Estimator

### 4.1 Algorithm Choice

The primary estimator is a multiplicative error-state extended Kalman filter (ESKF). An adaptive nonlinear complementary filter is implemented as a benchmark using the same calibrated samples and observation-quality logic.

The ESKF is selected because it explicitly represents gyroscope-bias uncertainty, handles variable time intervals and observation dropouts, provides covariance-derived confidence, and supports an independent future magnetometer update. A UKF or direct four-component quaternion EKF adds complexity without addressing the dominant failure mode: accelerometer contamination by platform acceleration and vibration.

### 4.2 Nominal and Error States

The nominal state is:

```text
x = { q_nb, b_g }
```

where `q_nb` is the body-to-navigation unit quaternion and `b_g` is the residual three-axis gyroscope bias in rad/s.

The six-dimensional error state is:

```text
delta_x = [ delta_theta, delta_b_g ]^T
```

The covariance is a 6 x 6 symmetric matrix. A three-component small-angle correction is injected multiplicatively into the nominal quaternion after each measurement update, after which the error state is reset.

Accelerometer bias is deliberately not estimated online. With only a six-axis IMU, sustained linear acceleration and accelerometer bias cannot be reliably separated. Accelerometer offset, scale, nonorthogonality, temperature response, and mounting alignment are handled by calibration. A future navigation ESKF with external velocity or position observations may add accelerometer bias, velocity, and position states.

### 4.3 Propagation

At each valid gyroscope frame, the estimator:

1. applies calibration, axis alignment, and temperature compensation;
2. subtracts the current residual bias estimate;
3. integrates the quaternion with an exponential-map increment;
4. normalizes the quaternion with finite-value and norm checks; and
5. propagates covariance using configured rate-noise density and bias random-walk density.

The production code uses `float`. Intermediate expressions are arranged to minimize cancellation, and all normalization and covariance operations have explicit numerical guards.

## 5. Accelerometer Observation and Bias Correction

On every fresh accelerometer sample, the estimator predicts gravity in the body frame and compares its direction with the calibrated specific-force direction. The direction residual is based on their cross product. This observation corrects roll, pitch, and the gyroscope-bias components that are observable through gravity. It never supplies an absolute yaw constraint.

Accelerometer updates are not restricted to complete rest. They are applied whenever the data is sufficiently gravity-dominated. The library assigns a continuous observation-quality weight from:

- acceleration-magnitude departure from standard gravity;
- high-frequency energy over a fixed sample window;
- acceleration jerk;
- angle between measured specific force and predicted gravity;
- normalized innovation squared (NIS);
- freshness, overrange, FIFO, and sensor-health flags; and
- persistence and hysteresis timers.

The quality weight continuously scales the acceleration measurement covariance. Small weights produce weak corrections; a weight below the configured rejection threshold skips the update. Weight changes are slew-limited so recovery cannot cause an attitude jump.

Persistent centripetal acceleration can resemble tilted gravity even when acceleration magnitude is near one g. A six-axis IMU cannot resolve every such case. The estimator therefore uses all quality indicators, favors temporary gyroscope-only propagation when evidence conflicts, and reports reduced confidence. It does not absorb persistent acceleration into an online accelerometer-bias state.

## 6. Static Detection and Zero-Angular-Rate Update

Static detection is separate from ordinary gravity qualification. It requires all of the following for a configurable dwell time with entry and exit hysteresis:

- low calibrated angular-rate magnitude and variance;
- acceleration magnitude near standard gravity;
- low acceleration variance and jerk;
- low vibration energy; and
- valid sensor status throughout the dwell interval.

While static, a zero-angular-rate update directly observes all three gyroscope-bias components. Static entry, exit, and bias-update rate are limited to prevent vehicle vibration or slow motion from being misclassified as zero rate.

## 7. Vibration Rejection

Vibration robustness has three layers:

1. **Sensor signal paths.** The ADXRS453 internal rate filter and ADXL355 500 Hz/125 Hz decimation filter provide the primary antialias filtering.
2. **Optional deterministic notch filters.** A configurable bank of second-order notches may target measured motor or structural frequencies. Notches are bypassed by default and are never adapted blindly in flight.
3. **Observation qualification.** Windowed vibration energy, jerk, innovation angle, and NIS reduce or reject contaminated gravity updates.

Mechanical design remains part of the system requirement. The gyroscopes must be mounted near a stiff support, and the board should place its first system resonance above 800 Hz as recommended by the ADXRS453 data sheet. ADXL355 vibration rectification and mechanical headroom cannot be repaired in software.

## 8. Calibration

Calibration is applied before estimation and consists of:

- three ADXRS453 scalar scale factors and their assembled three-axis alignment matrix;
- gyroscope mounting-angle and cross-axis correction;
- a temperature-dependent bias model for each gyroscope;
- ADXL355 3 x 3 scale/nonorthogonality/alignment matrix;
- ADXL355 offset and temperature model; and
- the board-IMU-to-body FRD rotation.

Calibration records carry a version, validity marker, temperature range, and checksum supplied by the storage layer. Invalid calibration prevents normal estimator startup unless the caller explicitly selects a diagnostic uncalibrated mode. Diagnostic mode is always marked in output health flags.

## 9. Software Architecture

The library is C99, uses `float`, allocates no dynamic memory, has deterministic execution, and has no operating-system dependency.

Planned modules are:

- `imu.h/.c`: public lifecycle and processing API;
- `imu_types.h`: configuration, samples, output, and health flags;
- `imu_calibration.h/.c`: calibration and temperature compensation;
- `imu_esekf.h/.c`: quaternion propagation and 6 x 6 ESKF;
- `imu_complementary.h/.c`: benchmark adaptive complementary estimator;
- `imu_motion_quality.h/.c`: vibration, jerk, innovation, and static detection;
- `imu_biquad.h/.c`: optional notch filters;
- `imu_math.h/.c`: fixed-size vector, matrix, and quaternion operations; and
- `imu_sensor_port.h`: acquisition-layer placeholder contract.

The core public contract is conceptually:

```c
typedef struct {
    uint64_t timestamp_ns;
    float gyro_rad_s[3];
    float gyro_temperature_c[3];
    uint32_t gyro_status[3];
    float accel_m_s2[3];
    float accel_temperature_c;
    uint32_t accel_status;
    bool gyro_sample_fresh[3];
    bool accel_sample_fresh;
} imu_sample_t;

bool imu_init(
    imu_t *imu,
    const imu_config_t *config,
    const imu_calibration_t *calibration);

imu_status_t imu_process(
    imu_t *imu,
    const imu_sample_t *sample,
    imu_output_t *output);

imu_status_t imu_update_magnetometer(
    imu_t *imu,
    const imu_mag_sample_t *sample);
```

The acquisition layer converts device words to SI units and exposes parity, sequence, self-test, power, PLL, FIFO, overrange, freshness, and timing status. Vendor SPI framing is not part of the estimator.

The output contains the normalized quaternion, FRD/NED Euler angles, calibrated bias-corrected angular rate, estimated gyroscope bias, acceleration-quality score, covariance-derived roll/pitch/yaw uncertainty, current fusion mode, timing status, calibration status, and sensor-health flags.

## 10. Error Handling and Numerical Safety

The estimator rejects or explicitly handles:

- nonmonotonic timestamps;
- time steps outside configured bounds;
- missing, stale, or repeated sensor data;
- nonfinite samples or calibration values;
- invalid quaternion norm;
- singular or nonfinite innovation covariance;
- covariance loss of symmetry or positive-semidefinite behavior; and
- sensor parity, sequence, self-test, power, PLL, FIFO, and overrange faults.

A rejected accelerometer observation does not stop valid gyroscope propagation. A rejected gyroscope frame does not reuse it as if it were new; the output is marked degraded and uncertainty grows. Hard numerical faults move the estimator into an explicit invalid state requiring controlled reinitialization.

## 11. Initialization

Initialization accepts either:

- a known initial quaternion supplied by the application; or
- a stationary alignment request that averages qualified accelerometer data to initialize roll and pitch, sets yaw to a caller-provided value (zero by default), and estimates initial gyroscope bias from the three rate channels.

Initialization does not complete until calibration is valid, timestamps are monotonic, sensors report healthy data, and the selected alignment method has sufficient samples. No single accelerometer sample is used as a high-confidence initial attitude.

## 12. Magnetometer Extension

The magnetometer API accepts a timestamped, calibrated magnetic vector and health information. Until a magnetometer model is configured, the function reports that heading aiding is unavailable and does not modify state.

A later implementation will add magnetic-field magnitude/direction qualification, hard/soft-iron calibration inputs, magnetic-disturbance rejection, and a yaw-sensitive ESKF observation. It will not change gyroscope propagation or the existing sample-processing ABI.

## 13. Verification and Acceptance

### 13.1 Unit and Numerical Tests

Tests cover quaternion algebra, exponential-map integration, coordinate conventions, fixed-size matrix operations, covariance prediction/update, calibration, biquads, quality scoring, static detection, initialization, timestamps, fault flags, nonfinite inputs, and recovery behavior.

All test builds enable strict compiler warnings. Host builds use available sanitizers and floating-point checks. Tests verify quaternion normalization, finite outputs, covariance symmetry, and covariance positive-semidefinite behavior within floating-point tolerance.

### 13.2 Deterministic Truth Simulation

Synthetic trajectories include static arbitrary attitudes, constant and compound rotations, rapid motion up to but not beyond the guaranteed gyroscope range, linear acceleration, sustained turns, centripetal acceleration, vibration sweeps, broadband vibration, shocks, temperature-driven bias changes, observation dropout, and observation recovery.

Each scenario fixes its random seed and specifies roll/pitch/yaw error, peak transient, recovery time, bias error, and consistency thresholds before results are evaluated.

### 13.3 Monte Carlo Verification

Monte Carlo trials use the data-sheet noise densities and configurable bias processes. Reports include attitude RMS and peak error, bias-estimation error, NIS distribution, covariance consistency, recovery time, and unobservable yaw drift. The report never presents yaw as absolute heading without an external heading observation.

### 13.4 Complementary-Filter Comparison

The ESKF and adaptive complementary filter receive identical calibrated samples, timestamps, notch settings, and observation-quality decisions. The comparison measures static tilt noise, dynamic roll/pitch RMS, erroneous correction during acceleration, vibration-induced attitude error, recovery time, response to bias change, CPU time, and memory.

The ESKF is accepted as the default only if it improves roll/pitch RMS in the combined high-vibration and bias-change scenarios without worse numerical failures or discontinuities. If the evidence does not show that improvement, the results retain the complementary filter as a supported alternative rather than changing the evaluation metric.

### 13.5 Hardware Log Replay

A host replay tool accepts timestamped CSV or binary captures from the Zynq. Absolute dynamic-accuracy claims require an external truth source such as a precision rate table, optical tracker, or higher-grade reference IMU. Logs without truth may validate repeatability, static stability, timing, and statistical consistency but not absolute dynamic accuracy.

## 14. Deliverables

The implementation phase will deliver:

- the portable C99 estimator library;
- the adaptive complementary benchmark;
- placeholder acquisition interfaces;
- configuration and calibration structures;
- unit, simulation, Monte Carlo, and replay tests;
- a host-side comparison report generator; and
- integration documentation for the 485 Hz Zynq task.

## 15. Source Documents

- Analog Devices, *ADXL354/ADXL355 Low Noise, Low Drift, Low Power, 3-Axis MEMS Accelerometers*, Rev. D, `adxl354_adxl355.pdf`.
- Analog Devices, *ADXRS453 High Performance, Digital Output Gyroscope*, Rev. B, `ADXRS453.pdf`.
