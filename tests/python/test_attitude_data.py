import tempfile
import unittest
from pathlib import Path

from tools.attitude_viewer.data import AttitudeSample, AttitudeTrack, load_replay_csv


HEADER = "timestamp_ns,qw,qx,qy,qz,accel_weight,fusion_mode,health_flags,status"
ROW = "1000000,1,0,0,0,0.5,2,3,4"


class ReplayCsvTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / "replay.csv"

    def load(self, *rows, header=HEADER):
        self.path.write_text("\n".join((header, *rows)) + "\n", encoding="utf-8")
        return load_replay_csv(self.path)

    def assert_bad(self, line, *rows, header=HEADER):
        with self.assertRaises(ValueError) as raised:
            self.load(*rows, header=header)
        self.assertIn(str(self.path), str(raised.exception))
        self.assertIn(f"line {line}", str(raised.exception))

    def test_exact_header_and_one_row(self):
        track = self.load(ROW)
        self.assertIsInstance(track, AttitudeTrack)
        self.assertEqual(track.name, self.path.name)
        self.assertEqual(track.hold_ns, 0)
        self.assertEqual(track.samples, (AttitudeSample(1000000, (1.0, 0.0, 0.0, 0.0), 0.5, 2, 3, 4),))

    def test_two_rows_and_normalized_quaternion(self):
        track = self.load(ROW, "3000000,2,0,0,0,1,0,4294967295,4294967295")
        self.assertEqual(len(track.samples), 2)
        self.assertEqual(track.samples[1].q_nb, (1.0, 0.0, 0.0, 0.0))
        self.assertEqual(track.samples[1].health_flags, 4294967295)
        self.assertEqual(track.hold_ns, 4000000)

    def test_maximum_uint64_timestamp(self):
        track = self.load("18446744073709551615,1,0,0,0,0,0,0,0")
        self.assertEqual(track.samples[0].timestamp_ns, 18446744073709551615)

    def test_hold_uses_twice_median_positive_delta_and_caps_at_100_ms(self):
        track = self.load(
            "0,1,0,0,0,0,0,0,0",
            "2000000,1,0,0,0,0,0,0,0",
            "6000000,1,0,0,0,0,0,0,0",
            "14000000,1,0,0,0,0,0,0,0",
        )
        self.assertEqual(track.hold_ns, 8000000)
        capped = self.load("0,1,0,0,0,0,0,0,0", "100000000,1,0,0,0,0,0,0,0")
        self.assertEqual(capped.hold_ns, 100000000)

    def test_rejects_nonmonotonic_and_overflow_timestamps(self):
        for bad in ("1000000,1,0,0,0,0,0,0,0", "999999,1,0,0,0,0,0,0,0", "18446744073709551616,1,0,0,0,0,0,0,0"):
            with self.subTest(bad=bad):
                self.assert_bad(3, ROW, bad)

    def test_rejects_nonfinite_and_zero_quaternion(self):
        for quaternion in ("nan,0,0,0", "0,0,0,0", "inf,0,0,0"):
            with self.subTest(quaternion=quaternion):
                self.assert_bad(2, f"0,{quaternion},0,0,0,0")

    def test_rejects_accel_weight_outside_unit_interval(self):
        for weight in ("-0.01", "1.01", "nan"):
            with self.subTest(weight=weight):
                self.assert_bad(2, f"0,1,0,0,0,{weight},0,0,0")

    def test_rejects_negative_and_overflow_uint32_fields(self):
        for index in (6, 7, 8):
            for value in ("-1", "4294967296"):
                with self.subTest(index=index, value=value):
                    fields = ["0", "1", "0", "0", "0", "0", "0", "0", "0"]
                    fields[index] = value
                    self.assert_bad(2, ",".join(fields))

    def test_rejects_wrong_header_column_count_and_empty_data(self):
        self.assert_bad(1, ROW, header=HEADER + ",extra")
        self.assert_bad(2, "0,1,0,0,0,0,0,0")
        self.assert_bad(2, "0,1,0,0,0,0,0,0,0,extra")
        self.assert_bad(2)

    def test_file_errors_include_path(self):
        with self.assertRaises(ValueError) as raised:
            load_replay_csv(self.path)
        self.assertIn(str(self.path), str(raised.exception))

    def test_very_long_integer_reports_path_and_line(self):
        self.assert_bad(2, "9" * 5000 + ",1,0,0,0,0,0,0,0")

    def test_malformed_csv_reports_path_and_line(self):
        self.assert_bad(2, '0,"1,0,0,0,0,0,0,0')


if __name__ == "__main__":
    unittest.main()
