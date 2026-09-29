"""Qt controls for viewing one or two imu_replay attitude CSV files."""

import sys
import time
from pathlib import Path
from typing import Callable

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QApplication, QComboBox, QFileDialog, QHBoxLayout, QLabel, QMainWindow,
    QMessageBox, QPushButton, QSlider, QVBoxLayout, QWidget,
)

from .data import AttitudeTrack, load_replay_csv
from .playback import ReplayController


def parse_paths(arguments: list[str]) -> tuple[Path, ...]:
    """Accept zero, one, or two positional CSV paths."""
    if len(arguments) > 2:
        raise ValueError("expected at most two CSV files")
    return tuple(Path(argument) for argument in arguments)


def load_tracks(paths: tuple[Path, ...]) -> tuple[AttitudeTrack, ...]:
    """Load all paths before changing the displayed replay."""
    return tuple(load_replay_csv(path) for path in paths)


class AttitudeWindow(QMainWindow):
    """A bounded-rate GUI over the renderer-independent replay controller."""

    _TIMELINE_STEPS = 1_000_000
    _LAYOUTS = {"Single": "single", "Side by side": "side_by_side", "Overlay": "overlay"}
    _SPEEDS = {"0.25×": 0.25, "0.5×": 0.5, "1×": 1.0, "2×": 2.0, "4×": 4.0}

    def __init__(
        self,
        tracks: tuple[AttitudeTrack, ...],
        scene_factory: Callable | None = None,
        clock_ns: Callable[[], int] = time.monotonic_ns,
    ):
        super().__init__()
        if scene_factory is None:
            from .scene import AttitudeScene
            scene_factory = AttitudeScene
        self._scene_factory = scene_factory
        self._clock_ns = clock_ns
        self.controller = ReplayController(tracks)
        self._resume_after_scrub = False
        self.setWindowTitle("IMU attitude replay")
        self.resize(1150, 720)

        central = QWidget(self)
        self.setCentralWidget(central)
        column = QVBoxLayout(central)
        controls = QHBoxLayout()
        column.addLayout(controls)

        self.open_button = QPushButton("Open CSV…", self)
        self.open_button.clicked.connect(self._choose_files)
        controls.addWidget(self.open_button)

        self.layout_combo = QComboBox(self)
        self.layout_combo.currentTextChanged.connect(self._change_layout)
        controls.addWidget(self.layout_combo)

        self.reset_button = QPushButton("Reset camera", self)
        controls.addWidget(self.reset_button)

        self.play_button = QPushButton("Play", self)
        self.play_button.clicked.connect(self._toggle_play)
        controls.addWidget(self.play_button)

        self.speed_combo = QComboBox(self)
        self.speed_combo.addItems(self._SPEEDS)
        self.speed_combo.setCurrentText("1×")
        self.speed_combo.currentTextChanged.connect(self._change_speed)
        controls.addWidget(self.speed_combo)

        self.timeline = QSlider(Qt.Orientation.Horizontal, self)
        self.timeline.setRange(0, self._TIMELINE_STEPS)
        self.timeline.sliderPressed.connect(self._scrub_started)
        self.timeline.valueChanged.connect(self._scrub_changed)
        self.timeline.sliderReleased.connect(self._scrub_finished)
        column.addWidget(self.timeline)

        self.time_label = QLabel(self)
        column.addWidget(self.time_label)
        self.state_labels = (QLabel(self), QLabel(self))
        for label in self.state_labels:
            column.addWidget(label)
        self.scene = self._scene_factory(central, layout="single" if len(tracks) == 1 else "side_by_side")
        column.addWidget(self.scene, stretch=1)
        self.scene.set_source_names(tuple(track.name for track in tracks))
        self.reset_button.clicked.connect(self.scene.reset_cameras)
        self._set_layout_choices(len(tracks))

        self._timer = QTimer(self)
        self._timer.setInterval(25)  # 40 Hz, independently of sensor CSV row rate.
        self._timer.timeout.connect(self._on_tick)
        self._timer.start()
        self._refresh()

    def _set_layout_choices(self, count: int) -> None:
        self.layout_combo.blockSignals(True)
        self.layout_combo.clear()
        if count == 1:
            self.layout_combo.addItem("Single")
        else:
            self.layout_combo.addItems(("Side by side", "Overlay"))
        self.layout_combo.blockSignals(False)
        self.scene.set_layout(self._LAYOUTS[self.layout_combo.currentText()])

    def _choose_files(self) -> None:
        paths, _ = QFileDialog.getOpenFileNames(
            self, "Choose one or two imu_replay CSV files", "", "CSV files (*.csv);;All files (*)"
        )
        if not paths:
            return
        try:
            parsed = parse_paths(paths)
            tracks = load_tracks(parsed)
        except ValueError as error:
            QMessageBox.critical(self, "Unable to open replay", str(error))
            return
        replacement = ReplayController(tracks)
        replacement.set_speed(self._SPEEDS[self.speed_combo.currentText()])
        self.controller = replacement
        self.scene.set_source_names(tuple(track.name for track in tracks))
        self._set_layout_choices(len(tracks))
        self._refresh()

    def _change_layout(self, label: str) -> None:
        if label:
            self.scene.set_layout(self._LAYOUTS[label])
            self._refresh()

    def _change_speed(self, label: str) -> None:
        if label:
            self.controller.advance(self._clock_ns())
            self.controller.set_speed(self._SPEEDS[label])
            self._refresh()

    def _toggle_play(self) -> None:
        now_ns = self._clock_ns()
        if self.controller.playing:
            self.controller.pause(now_ns)
        else:
            self.controller.play(now_ns)
        self._refresh()

    def _scrub_started(self) -> None:
        self._resume_after_scrub = self.controller.playing
        self.controller.pause(self._clock_ns())
        self._refresh()

    def _scrub_changed(self, value: int) -> None:
        duration = self.controller.end_ns - self.controller.start_ns
        position = self.controller.start_ns + duration * value // self._TIMELINE_STEPS
        self.controller.set_position_ns(position)
        self._refresh()

    def _scrub_finished(self) -> None:
        if self._resume_after_scrub:
            self.controller.play(self._clock_ns())
        self._resume_after_scrub = False
        self._refresh()

    def _on_tick(self) -> None:
        self.controller.advance(self._clock_ns())
        self._refresh()

    def _refresh(self) -> None:
        controller = self.controller
        samples = controller.selected_samples()
        self.scene.show_samples(samples, controller.position_ns)
        duration = controller.end_ns - controller.start_ns
        value = (controller.position_ns - controller.start_ns) * self._TIMELINE_STEPS // duration if duration else 0
        self.timeline.blockSignals(True)
        self.timeline.setValue(value)
        self.timeline.blockSignals(False)
        self.play_button.setText("Pause" if controller.playing else "Play")
        self.time_label.setText(
            f"Host reference timestamp_ns: {controller.position_ns}  "
            f"({(controller.position_ns - controller.start_ns) / 1e9:.3f} s from start)"
        )
        for index, label in enumerate(self.state_labels):
            if index >= len(samples):
                label.hide()
                continue
            label.show()
            name = controller.tracks[index].name
            sample = samples[index]
            if sample is None:
                label.setText(f"{name}: No data")
                label.setStyleSheet("color: #d97706; font-weight: bold")
            else:
                degraded = sample.status != 0 or sample.health_flags != 0
                prefix = "DEGRADED" if degraded else "OK"
                label.setText(
                    f"{name}: {prefix}  status={sample.status}  "
                    f"health=0x{sample.health_flags:x}  mode={sample.fusion_mode}  "
                    f"accel_weight={sample.accel_weight:.3f}"
                )
                label.setStyleSheet("color: #c62828; font-weight: bold" if degraded else "")

    def closeEvent(self, event) -> None:
        self._timer.stop()
        self.scene.close()
        super().closeEvent(event)


def main(arguments: list[str] | None = None) -> int:
    """Launch the viewer; return a CLI status for invalid arguments or files."""
    if arguments is None:
        arguments = sys.argv[1:]
    try:
        paths = parse_paths(arguments)
    except ValueError as error:
        print(f"attitude viewer: {error}", file=sys.stderr)
        return 2
    app = QApplication.instance() or QApplication(sys.argv[:1])
    if not paths:
        selected, _ = QFileDialog.getOpenFileNames(
            None, "Choose one or two imu_replay CSV files", "", "CSV files (*.csv);;All files (*)"
        )
        if not selected:
            return 0
        try:
            paths = parse_paths(selected)
        except ValueError as error:
            QMessageBox.critical(None, "Unable to open replay", str(error))
            return 2
    try:
        tracks = load_tracks(paths)
        window = AttitudeWindow(tracks)
    except (ValueError, ImportError) as error:
        message = str(error)
        if isinstance(error, ImportError):
            message += "\nInstall dependencies: py -m pip install PyVista PyVistaQt PySide6"
        print(f"attitude viewer: {message}", file=sys.stderr)
        QMessageBox.critical(None, "Unable to open replay", message)
        return 1
    window.show()
    return app.exec()
