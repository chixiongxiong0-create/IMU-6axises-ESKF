import unittest

from tools.attitude_viewer.data import AttitudeSample, AttitudeTrack
from tools.attitude_viewer.playback import ReplayController


ORIGIN = 1 << 63


def sample(timestamp_ns):
    return AttitudeSample(timestamp_ns, (1.0, 0.0, 0.0, 0.0), 0.5, 0, 0, 0)


def track(name, hold_ns, *timestamps):
    return AttitudeTrack(tuple(sample(timestamp) for timestamp in timestamps), hold_ns, name)


class ReplayControllerTests(unittest.TestCase):
    def test_dual_union_range_and_exact_nanosecond_seek(self):
        first = track("first", 0, ORIGIN, ORIGIN + 2_000_000)
        second = track("second", 0, ORIGIN + 1_000_000, ORIGIN + 3_000_000)
        controller = ReplayController((first, second))
        self.assertEqual((controller.start_ns, controller.end_ns), (ORIGIN, ORIGIN + 3_000_000))
        self.assertEqual(controller.position_ns, ORIGIN)
        self.assertEqual(controller.selected_samples(), (first.samples[0], None))
        controller.set_position_ns(ORIGIN + 2_000_000)
        self.assertEqual(controller.position_ns, ORIGIN + 2_000_000)
        self.assertEqual(controller.selected_samples(), (first.samples[1], None))
        controller.set_position_ns(ORIGIN + 3_000_000)
        self.assertEqual(controller.selected_samples(), (None, second.samples[1]))

    def test_sample_hold_expires_one_nanosecond_after_limit(self):
        first = track("first", 10, 100, 200)
        controller = ReplayController((first,))
        controller.set_position_ns(110)
        self.assertEqual(controller.selected_samples(), (first.samples[0],))
        controller.set_position_ns(111)
        self.assertEqual(controller.selected_samples(), (None,))

    def test_before_first_and_after_last_are_no_data(self):
        first = track("first", 50, 100, 200)
        second = track("second", 50, 50, 250)
        controller = ReplayController((first, second))
        self.assertEqual(controller.selected_samples(), (None, second.samples[0]))
        controller.set_position_ns(250)
        self.assertEqual(controller.selected_samples(), (None, second.samples[1]))

    def test_speed_change_preserves_position_and_rebases_elapsed(self):
        controller = ReplayController((track("first", 0, 100, 10_000),))
        controller.play(1_000)
        controller.advance(1_100)
        self.assertEqual(controller.position_ns, 200)
        controller.set_speed(4.0)
        self.assertEqual(controller.position_ns, 200)
        controller.advance(1_200)
        self.assertEqual(controller.position_ns, 600)

    def test_quarter_and_four_times_speed_use_integer_elapsed(self):
        controller = ReplayController((track("first", 0, ORIGIN, ORIGIN + 10_000),))
        controller.set_speed(0.25)
        controller.play(1_000)
        controller.advance(1_007)
        self.assertEqual(controller.position_ns, ORIGIN + 1)
        controller.advance(1_008)
        self.assertEqual(controller.position_ns, ORIGIN + 2)
        controller.set_speed(4.0)
        controller.advance(1_010)
        self.assertEqual(controller.position_ns, ORIGIN + 10)

    def test_end_clamps_and_pauses_without_loop(self):
        controller = ReplayController((track("first", 0, 100, 200),))
        controller.play(1_000)
        controller.advance(1_500)
        self.assertEqual(controller.position_ns, 200)
        self.assertFalse(controller.playing)
        controller.advance(2_000)
        self.assertEqual(controller.position_ns, 200)

    def test_pause_freezes_position_and_seek_is_clamped(self):
        controller = ReplayController((track("first", 0, 100, 200),))
        controller.play(1_000)
        controller.pause(1_025)
        self.assertEqual(controller.position_ns, 125)
        controller.advance(2_000)
        self.assertEqual(controller.position_ns, 125)
        controller.set_position_ns(-1)
        self.assertEqual(controller.position_ns, 100)
        controller.set_position_ns(1_000)
        self.assertEqual(controller.position_ns, 200)

    def test_rejects_invalid_track_count_and_speed(self):
        first = track("first", 0, 100)
        for tracks in ((), (first, first, first)):
            with self.subTest(tracks=tracks), self.assertRaises(ValueError):
                ReplayController(tracks)
        controller = ReplayController((first,))
        for speed in (0, 0.3, 8):
            with self.subTest(speed=speed), self.assertRaises(ValueError):
                controller.set_speed(speed)


if __name__ == "__main__":
    unittest.main()
