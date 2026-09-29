import unittest
import subprocess
import sys
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

from PySide6.QtWidgets import QApplication, QWidget

from tools.attitude_viewer.data import AttitudeSample, AttitudeTrack
from tools.attitude_viewer.app import AttitudeWindow, load_tracks, parse_paths


class FakeScene(QWidget):
    def __init__(self, parent, layout="single"):
        super().__init__(parent)
        self.layout_mode = layout
        self.samples = ()
        self.timestamp_ns = None
        self.reset_count = 0
        self.source_names = ()

    def set_source_names(self, names):
        self.source_names = names

    def set_layout(self, layout):
        self.layout_mode = layout

    def show_samples(self, samples, timestamp_ns):
        self.samples = samples
        self.timestamp_ns = timestamp_ns

    def reset_cameras(self):
        self.reset_count += 1


def track(name, *samples):
    return AttitudeTrack(tuple(samples), 200_000_000, name)


def sample(timestamp_ns, status=0, health=0, mode=0):
    return AttitudeSample(timestamp_ns, (1.0, 0.0, 0.0, 0.0), 0.5, mode, health, status)


class AppTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.now = 10_000_000_000
        self.first = track("A.csv", sample(1_000_000_000), sample(5_000_000_000))

    def make_window(self, *tracks):
        window = AttitudeWindow(tuple(tracks), scene_factory=FakeScene,
                                clock_ns=lambda: self.now)
        self.addCleanup(window.close)
        return window

    def test_cli_rejects_more_than_two_files(self):
        with self.assertRaisesRegex(ValueError, "at most two"):
            parse_paths(["a.csv", "b.csv", "c.csv"])
        self.assertEqual(parse_paths(["a.csv"]), (Path("a.csv"),))

    def test_missing_gui_dependency_reports_install_command(self):
        result = subprocess.run(
            [sys.executable, "-S", "-m", "tools.attitude_viewer", "example.csv"],
            capture_output=True, text=True, check=False,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("pip install PyVista PyVistaQt PySide6", result.stderr)

    def test_bad_file_diagnostic_identifies_path_and_line(self):
        with TemporaryDirectory() as directory:
            bad = Path(directory) / "bad.csv"
            bad.write_text("wrong,header\n", encoding="utf-8")
            with self.assertRaises(ValueError) as caught:
                load_tracks((bad,))
            self.assertIn(str(bad), str(caught.exception))
            self.assertIn("line 1", str(caught.exception))
            self.assertEqual(bad.read_text(encoding="utf-8"), "wrong,header\n")

    def test_play_pause_scrub_and_speed_control_rebase_clock(self):
        window = self.make_window(self.first)
        window.speed_combo.setCurrentText("2×")
        self.assertEqual(window.controller.speed, 2.0)
        window.play_button.click()
        self.now += 250_000_000
        window._on_tick()
        self.assertEqual(window.controller.position_ns, 1_500_000_000)
        window.timeline.sliderPressed.emit()
        self.assertFalse(window.controller.playing)
        window.timeline.setValue(window.timeline.maximum() // 2)
        self.assertEqual(window.controller.position_ns, 3_000_000_000)
        window.timeline.sliderReleased.emit()
        self.assertTrue(window.controller.playing)
        window.speed_combo.setCurrentText("0.5×")
        self.now += 200_000_000
        window._on_tick()
        self.assertEqual(window.controller.position_ns, 3_100_000_000)
        window.play_button.click()
        self.assertFalse(window.controller.playing)
        self.assertIn("host reference", window.time_label.text().lower())

    def test_two_layouts_preserve_position_and_speed(self):
        second = track("B.csv", sample(2_000_000_000), sample(6_000_000_000))
        window = self.make_window(self.first, second)
        window.controller.set_position_ns(3_000_000_000)
        window.speed_combo.setCurrentText("2×")
        window.layout_combo.setCurrentText("Overlay")
        self.assertEqual(window.scene.layout_mode, "overlay")
        self.assertEqual(window.controller.position_ns, 3_000_000_000)
        self.assertEqual(window.controller.speed, 2.0)
        window.layout_combo.setCurrentText("Side by side")
        self.assertEqual(window.scene.layout_mode, "side_by_side")
        self.assertEqual(window.controller.position_ns, 3_000_000_000)
        window.reset_button.click()
        self.assertEqual(window.scene.reset_count, 1)

    def test_no_data_and_degraded_flags_are_visible(self):
        first = AttitudeTrack((sample(100, status=3, health=0x12, mode=4),), 0, "A.csv")
        second = track("B.csv", sample(200))
        window = self.make_window(first, second)
        self.assertIn("DEGRADED", window.state_labels[0].text())
        self.assertIn("status=3", window.state_labels[0].text())
        self.assertIn("health=0x12", window.state_labels[0].text())
        self.assertIn("mode=4", window.state_labels[0].text())
        self.assertIn("No data", window.state_labels[1].text())
        window.controller.set_position_ns(200)
        window._refresh()
        self.assertIn("No data", window.state_labels[0].text())
        self.assertIn("OK", window.state_labels[1].text())

    def test_replacing_csv_keeps_selected_playback_speed(self):
        window = self.make_window(self.first)
        self.assertEqual(window.scene.source_names, ("A.csv",))
        window.speed_combo.setCurrentText("4×")
        with TemporaryDirectory() as directory:
            replacement = Path(directory) / "replacement.csv"
            replacement.write_text(
                "timestamp_ns,qw,qx,qy,qz,accel_weight,fusion_mode,health_flags,status\n"
                "200,1,0,0,0,1,0,0,0\n"
                "300,1,0,0,0,1,0,0,0\n",
                encoding="utf-8",
            )
            with patch("tools.attitude_viewer.app.QFileDialog.getOpenFileNames",
                       return_value=([str(replacement)], "")):
                window.open_button.click()
        self.assertEqual(window.controller.tracks[0].name, "replacement.csv")
        self.assertEqual(window.scene.source_names, ("replacement.csv",))
        self.assertEqual(window.speed_combo.currentText(), "4×")
        self.assertEqual(window.controller.speed, 4.0)


if __name__ == "__main__":
    unittest.main()
