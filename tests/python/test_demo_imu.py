"""Behavior tests for the reproducible multi-axis IMU demonstration."""

import csv
import tempfile
import unittest
from pathlib import Path

from tools.attitude_viewer.data import load_replay_csv
from tools.generate_demo_imu import generate_demo


class DemoImuTests(unittest.TestCase):
    def test_generates_replayable_multiaxis_motion_and_truth(self):
        with tempfile.TemporaryDirectory() as directory:
            raw = Path(directory) / "raw.csv"
            truth = Path(directory) / "truth.csv"
            count = generate_demo(raw, truth)
            self.assertEqual(count, 12 * 485 + 1)

            with raw.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), count)
            self.assertEqual(
                list(rows[0]),
                "timestamp_ns,gx,gy,gz,tgx,tgy,tgz,gstatus_x,gstatus_y,"
                "gstatus_z,gfresh_x,gfresh_y,gfresh_z,ax,ay,az,ta,astatus,afresh".split(","),
            )
            self.assertEqual(rows[0]["timestamp_ns"], "1000000000")
            self.assertEqual(rows[-1]["timestamp_ns"], "13000000000")
            self.assertEqual(rows[0]["gstatus_x"], "1")
            self.assertEqual(rows[0]["gfresh_x"], "1")
            self.assertEqual(rows[0]["astatus"], "1")
            self.assertEqual(rows[0]["afresh"], "1")
            self.assertAlmostEqual(float(rows[0]["az"]), -9.80665, delta=0.3)

            active = rows[4 * 485]
            self.assertGreater(abs(float(active["gx"])), 0.05)
            self.assertGreater(abs(float(active["gy"])), 0.05)
            self.assertGreater(abs(float(active["gz"])), 0.05)
            accelerated = rows[8 * 485]
            self.assertGreater(
                abs(float(accelerated["ax"])) + abs(float(accelerated["ay"])),
                0.5,
            )

            track = load_replay_csv(truth)
            self.assertEqual(len(track.samples), count)
            self.assertEqual(track.samples[0].q_nb, (1.0, 0.0, 0.0, 0.0))
            self.assertGreater(abs(track.samples[4 * 485].q_nb[1]), 0.02)
            self.assertGreater(abs(track.samples[4 * 485].q_nb[2]), 0.02)

    def test_same_seed_produces_identical_files(self):
        with tempfile.TemporaryDirectory() as directory:
            paths = [Path(directory) / name for name in ("a.csv", "b.csv", "c.csv", "d.csv")]
            generate_demo(paths[0], paths[1], seed=47)
            generate_demo(paths[2], paths[3], seed=47)
            self.assertEqual(paths[0].read_bytes(), paths[2].read_bytes())
            self.assertEqual(paths[1].read_bytes(), paths[3].read_bytes())


if __name__ == "__main__":
    unittest.main()
