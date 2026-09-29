"""Generate deterministic, synthetic 485 Hz IMU input and truth for 3D replay.

Run from the repository root:
    py -m tools.generate_demo_imu RAW.csv TRUTH.csv
"""

import argparse
import csv
import math
import random
from pathlib import Path


RATE_HZ = 485
DURATION_S = 12
GRAVITY = 9.80665
RAW_HEADER = (
    "timestamp_ns,gx,gy,gz,tgx,tgy,tgz,gstatus_x,gstatus_y,gstatus_z,"
    "gfresh_x,gfresh_y,gfresh_z,ax,ay,az,ta,astatus,afresh"
).split(",")
TRUTH_HEADER = (
    "timestamp_ns,qw,qx,qy,qz,accel_weight,fusion_mode,health_flags,status"
).split(",")


def _motion(t: float) -> tuple[float, float, float]:
    """Body angular rate in rad/s, with a stationary alignment prelude."""
    if t < 1.5:
        return (0.0, 0.0, 0.0)
    if t < 5.0:
        u = t - 1.5
        return (0.60 * math.sin(0.8 * u),
                0.40 * math.sin(0.7 * u + 0.3), 0.65)
    if t < 7.0:
        u = t - 5.0
        return (-0.45, 0.35 * math.cos(1.1 * u), -0.45)
    if t < 9.5:
        u = t - 7.0
        return (0.20 + 0.32 * math.sin(math.tau * 4.0 * u),
                0.23 * math.sin(math.tau * 3.3 * u), 0.30)
    u = t - 9.5
    return (0.22 * math.cos(1.2 * u), -0.18 * math.sin(1.1 * u),
            0.12 * math.cos(0.9 * u))


def _linear_accel(t: float) -> tuple[float, float, float]:
    if not 7.0 <= t < 9.5:
        return (0.0, 0.0, 0.0)
    u = t - 7.0
    return (0.8 + 1.2 * math.sin(math.tau * 2.0 * u),
            0.5 * math.cos(math.tau * 1.5 * u), 0.15)


def _multiply(a: tuple[float, ...], b: tuple[float, ...]) -> tuple[float, ...]:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw*bw - ax*bx - ay*by - az*bz,
            aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw)


def _advance(q: tuple[float, ...], omega: tuple[float, ...], dt: float) -> tuple[float, ...]:
    magnitude = math.sqrt(sum(component * component for component in omega))
    half_angle = 0.5 * magnitude * dt
    scale = math.sin(half_angle) / magnitude if magnitude else 0.5 * dt
    delta = (math.cos(half_angle), *(component * scale for component in omega))
    product = _multiply(q, delta)
    norm = math.sqrt(sum(component * component for component in product))
    return tuple(component / norm for component in product)


def _specific_force(q: tuple[float, ...], t: float) -> tuple[float, float, float]:
    """Rotate navigation-frame gravity specific force into the FRD body."""
    conjugate = (q[0], -q[1], -q[2], -q[3])
    rotated = _multiply(_multiply(conjugate, (0.0, 0.0, 0.0, -GRAVITY)), q)
    linear = _linear_accel(t)
    vibration = (0.0, 0.0, 0.0)
    if 7.0 <= t < 9.5:
        u = t - 7.0
        vibration = (0.22 * math.sin(math.tau * 37 * u),
                     0.18 * math.sin(math.tau * 43 * u + 0.4),
                     0.25 * math.sin(math.tau * 31 * u + 0.8))
    return tuple(rotated[i + 1] + linear[i] + vibration[i] for i in range(3))


def generate_demo(raw_path: str | Path, truth_path: str | Path, *, seed: int = 20260928) -> int:
    """Write a 12 s raw-sensor CSV and matching truth-attitude CSV."""
    raw_path, truth_path = Path(raw_path), Path(truth_path)
    if raw_path.resolve() == truth_path.resolve():
        raise ValueError("raw and truth output paths must differ")
    raw_path.parent.mkdir(parents=True, exist_ok=True)
    truth_path.parent.mkdir(parents=True, exist_ok=True)
    rng = random.Random(seed)
    q: tuple[float, ...] = (1.0, 0.0, 0.0, 0.0)
    count = DURATION_S * RATE_HZ + 1
    with raw_path.open("w", newline="", encoding="utf-8") as raw_file, \
         truth_path.open("w", newline="", encoding="utf-8") as truth_file:
        raw, truth = csv.writer(raw_file), csv.writer(truth_file)
        raw.writerow(RAW_HEADER)
        truth.writerow(TRUTH_HEADER)
        for frame in range(count):
            t = frame / RATE_HZ
            if frame:
                q = _advance(q, _motion((frame - 0.5) / RATE_HZ), 1.0 / RATE_HZ)
            timestamp = 1_000_000_000 + frame * 1_000_000_000 // RATE_HZ
            omega = _motion(t)
            force = _specific_force(q, t)
            gyro_bias = (0.008, -0.006, 0.010)
            gyro = [omega[i] + gyro_bias[i] + rng.gauss(0.0, 0.003)
                    for i in range(3)]
            accel = [force[i] + rng.gauss(0.0, 0.04) for i in range(3)]
            temperature = 25.0 + 1.5 * math.sin(0.15 * t)
            raw.writerow([timestamp, *[f"{v:.9g}" for v in gyro],
                          *[f"{temperature:.6g}"] * 3, 1, 1, 1,
                          1, 1, 1, *[f"{v:.9g}" for v in accel],
                          f"{temperature:.6g}", 1, 1])
            truth.writerow([timestamp, *[f"{v:.10g}" for v in q],
                            1, 3, 0, 0])
    return count


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("raw_csv", type=Path)
    parser.add_argument("truth_csv", type=Path)
    parser.add_argument("--seed", type=int, default=20260928)
    args = parser.parse_args()
    count = generate_demo(args.raw_csv, args.truth_csv, seed=args.seed)
    print(f"generated {count} synthetic 485 Hz frames: {args.raw_csv}, {args.truth_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
