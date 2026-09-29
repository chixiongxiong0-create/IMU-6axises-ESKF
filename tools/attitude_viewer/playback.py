"""Renderer-independent replay timing and sample selection."""

from bisect import bisect_right
from fractions import Fraction

from .data import AttitudeSample, AttitudeTrack


_SPEEDS = {0.25: Fraction(1, 4), 0.5: Fraction(1, 2), 1.0: Fraction(1),
           2.0: Fraction(2), 4.0: Fraction(4)}


class ReplayController:
    """Map one integer-nanosecond clock onto one or two replay tracks."""

    def __init__(self, tracks: tuple[AttitudeTrack, ...]):
        if len(tracks) not in (1, 2) or any(not track.samples for track in tracks):
            raise ValueError("expected one or two nonempty tracks")
        self.tracks = tracks
        self._timestamps = tuple(
            tuple(sample.timestamp_ns for sample in track.samples) for track in tracks
        )
        self.start_ns = min(timestamps[0] for timestamps in self._timestamps)
        self.end_ns = max(timestamps[-1] for timestamps in self._timestamps)
        self.position_ns = self.start_ns
        self.speed = 1.0
        self.playing = False
        self.base_position_ns = self.position_ns
        self.base_wall_ns: int | None = None
        self._last_wall_ns: int | None = None

    def set_position_ns(self, position_ns: int) -> None:
        self.position_ns = max(self.start_ns, min(position_ns, self.end_ns))
        self.base_position_ns = self.position_ns
        if self.playing:
            self.base_wall_ns = self._last_wall_ns
        if self.position_ns == self.end_ns:
            self.playing = False

    def set_speed(self, speed: float) -> None:
        if isinstance(speed, bool) or speed not in _SPEEDS:
            raise ValueError("speed must be 0.25, 0.5, 1, 2, or 4")
        self.base_position_ns = self.position_ns
        if self.playing:
            self.base_wall_ns = self._last_wall_ns
        self.speed = float(speed)

    def play(self, now_ns: int) -> None:
        if self.position_ns == self.end_ns:
            return
        if not self.playing:
            self.playing = True
            self.base_position_ns = self.position_ns
            self.base_wall_ns = now_ns
            self._last_wall_ns = now_ns

    def pause(self, now_ns: int) -> None:
        if self.playing:
            self.advance(now_ns)
            self.playing = False

    def advance(self, now_ns: int) -> None:
        if not self.playing:
            return
        assert self.base_wall_ns is not None
        elapsed_ns = max(0, now_ns - self.base_wall_ns)
        rate = _SPEEDS[self.speed]
        increment_ns = elapsed_ns * rate.numerator // rate.denominator
        self.position_ns = min(self.base_position_ns + increment_ns, self.end_ns)
        self._last_wall_ns = now_ns
        if self.position_ns == self.end_ns:
            self.playing = False

    def selected_samples(self) -> tuple[AttitudeSample | None, ...]:
        selected: list[AttitudeSample | None] = []
        for track, timestamps in zip(self.tracks, self._timestamps):
            if self.position_ns > timestamps[-1]:
                selected.append(None)
                continue
            index = bisect_right(timestamps, self.position_ns) - 1
            if index < 0 or self.position_ns - timestamps[index] > track.hold_ns:
                selected.append(None)
            else:
                selected.append(track.samples[index])
        return tuple(selected)
