"""Validated, dependency-free adapter for imu_replay output CSV files."""

import csv
import math
from dataclasses import dataclass
from pathlib import Path


_HEADER = (
    "timestamp_ns", "qw", "qx", "qy", "qz", "accel_weight",
    "fusion_mode", "health_flags", "status",
)
_UINT64_MAX = (1 << 64) - 1
_UINT32_MAX = (1 << 32) - 1


@dataclass(frozen=True)
class AttitudeSample:
    timestamp_ns: int
    q_nb: tuple[float, float, float, float]
    accel_weight: float
    fusion_mode: int
    health_flags: int
    status: int


@dataclass(frozen=True)
class AttitudeTrack:
    samples: tuple[AttitudeSample, ...]
    hold_ns: int
    name: str


def _invalid(path: Path, line: int, reason: str) -> ValueError:
    return ValueError(f"{path}: line {line}: {reason}")


def _unsigned(text: str, limit: int, path: Path, line: int, name: str) -> int:
    if not text or not all("0" <= char <= "9" for char in text):
        raise _invalid(path, line, f"invalid {name}")
    value = 0
    for char in text:
        digit = ord(char) - ord("0")
        if value > (limit - digit) // 10:
            raise _invalid(path, line, f"{name} exceeds range")
        value = 10 * value + digit
    return value


def _finite(text: str, path: Path, line: int, name: str) -> float:
    if not text or text != text.strip() or "_" in text:
        raise _invalid(path, line, f"invalid {name}")
    try:
        value = float(text)
    except ValueError as error:
        raise _invalid(path, line, f"invalid {name}") from error
    if not math.isfinite(value):
        raise _invalid(path, line, f"nonfinite {name}")
    return value


def _parse_sample(fields: list[str], path: Path, line: int) -> AttitudeSample:
    if len(fields) != len(_HEADER):
        raise _invalid(path, line, f"expected {len(_HEADER)} columns, got {len(fields)}")
    timestamp = _unsigned(fields[0], _UINT64_MAX, path, line, "timestamp_ns")
    quaternion = tuple(
        _finite(fields[index], path, line, _HEADER[index]) for index in range(1, 5)
    )
    scale = max(abs(component) for component in quaternion)
    if scale == 0:
        raise _invalid(path, line, "zero quaternion")
    scaled = tuple(component / scale for component in quaternion)
    magnitude = math.sqrt(sum(component * component for component in scaled))
    q_nb = tuple(component / magnitude for component in scaled)
    weight = _finite(fields[5], path, line, "accel_weight")
    if not 0 <= weight <= 1:
        raise _invalid(path, line, "accel_weight outside [0, 1]")
    mode = _unsigned(fields[6], _UINT32_MAX, path, line, "fusion_mode")
    health = _unsigned(fields[7], _UINT32_MAX, path, line, "health_flags")
    status = _unsigned(fields[8], _UINT32_MAX, path, line, "status")
    return AttitudeSample(timestamp, q_nb, weight, mode, health, status)


def load_replay_csv(path: str | Path) -> AttitudeTrack:
    """Load one replay track, rejecting malformed or out-of-range values."""
    source = Path(path)
    samples: list[AttitudeSample] = []
    try:
        with source.open("r", encoding="utf-8", newline="") as stream:
            reader = csv.reader(stream, strict=True)
            try:
                try:
                    header = next(reader)
                except StopIteration as error:
                    raise _invalid(source, 1, "missing CSV header") from error
                if tuple(header) != _HEADER:
                    raise _invalid(source, 1, "unexpected CSV header")
                for fields in reader:
                    line = reader.line_num
                    sample = _parse_sample(fields, source, line)
                    if samples and sample.timestamp_ns <= samples[-1].timestamp_ns:
                        raise _invalid(source, line, "nonmonotonic timestamp_ns")
                    samples.append(sample)
            except csv.Error as error:
                raise _invalid(source, max(reader.line_num, 1), str(error)) from error
    except (OSError, UnicodeError) as error:
        raise ValueError(f"{source}: {error}") from error
    if not samples:
        raise _invalid(source, 2, "missing data rows")
    if len(samples) == 1:
        hold_ns = 0
    else:
        diffs = sorted(
            samples[index].timestamp_ns - samples[index - 1].timestamp_ns
            for index in range(1, len(samples))
        )
        count = len(diffs)
        double_median = (2 * diffs[count // 2] if count % 2 else
                         diffs[count // 2 - 1] + diffs[count // 2])
        hold_ns = min(double_median, 100_000_000)
    return AttitudeTrack(tuple(samples), hold_ns, source.name)
