"""Qt/VTK attitude scene. Coordinates are NED; local body geometry is FRD."""

from dataclasses import dataclass
from typing import Literal

import numpy as np
import pyvista as pv
from PySide6.QtCore import QEvent
from PySide6.QtGui import QCursor
from PySide6.QtWidgets import QHBoxLayout, QToolTip, QWidget
from pyvistaqt import QtInteractor
from vtkmodules.vtkRenderingCore import vtkPropPicker

from .data import AttitudeSample
from .geometry import rotation_matrix


Layout = Literal["single", "side_by_side", "overlay"]
_STREAM_COLORS = ("#37c9ec", "#ffae42")
_AXIS_COLORS = ("#ec5b5b", "#68cb73", "#6d91f2")
_CAMERA = ((4.0, -6.0, -4.0), (0.0, 0.0, 0.0), (0.0, 0.0, -1.0))
_REFERENCE_REACH = 3.0


def overlay_offset(index: int) -> tuple[float, float, float]:
    """Fixed east-west separation for display; it is not an estimated position."""
    if index not in (0, 1):
        raise ValueError("overlay supports exactly two streams")
    return (0.0, -0.9 if index == 0 else 0.9, 0.0)


def actor_transform(
    sample: AttitudeSample, offset: tuple[float, float, float]
) -> tuple[tuple[float, float, float, float], ...]:
    """Homogeneous FRD-to-NED transform, with a display-only translation."""
    rotation = rotation_matrix(sample.q_nb)
    return tuple(
        tuple(rotation[row]) + (offset[row],) for row in range(3)
    ) + ((0.0, 0.0, 0.0, 1.0),)


@dataclass
class _View:
    interactor: QtInteractor
    indices: tuple[int, ...]
    body_actors: dict[int, list]
    no_data: dict[int, object]
    caption: object
    picker: vtkPropPicker


class AttitudeScene(QWidget):
    """One or two synchronized render views; playback remains with the caller."""

    def __init__(self, parent: QWidget, layout: Layout = "single"):
        super().__init__(parent)
        self._row = QHBoxLayout(self)
        self._row.setContentsMargins(0, 0, 0, 0)
        self._row.setSpacing(4)
        self._views: list[_View] = []
        self._layout_mode: Layout = "single"
        self._samples: tuple[AttitudeSample | None, ...] = ()
        self._timestamp_ns = 0
        self._closed = False
        self._source_names: tuple[str, ...] = ()
        self._body_mesh, self._nose_mesh = self._make_body_meshes()
        self.set_layout(layout)

    @staticmethod
    def _make_body_meshes():
        hull = pv.Cylinder(center=(0, 0, 0), direction=(1, 0, 0), radius=0.12, height=0.75)
        wing = pv.Box(bounds=(-0.12, 0.16, -0.55, 0.55, -0.025, 0.025))
        tail = pv.Box(bounds=(-0.35, -0.2, -0.26, 0.26, -0.025, 0.025))
        nose = pv.Cone(center=(0.56, 0, 0), direction=(1, 0, 0), height=0.4, radius=0.18)
        return hull.merge(wing).merge(tail), nose

    def _build_view(self, indices: tuple[int, ...]) -> _View:
        plotter = QtInteractor(parent=self, auto_update=False)
        self._row.addWidget(plotter)
        plotter.set_background("#111827")
        for label, direction, color in zip("ned", ((1, 0, 0), (0, 1, 0), (0, 0, 1)), _AXIS_COLORS):
            positive = tuple(_REFERENCE_REACH * component for component in direction)
            negative = tuple(-component for component in positive)
            plotter.add_mesh(pv.Line((0, 0, 0), positive), name=f"ned-positive-{label}",
                             color=color, line_width=3, opacity=0.9, pickable=False)
            plotter.add_mesh(pv.Line(negative, (0, 0, 0)), name=f"ned-negative-{label}",
                             color=color, line_width=2, opacity=0.3, pickable=False)
            tip_start = tuple((_REFERENCE_REACH - 0.25) * component for component in direction)
            plotter.add_mesh(pv.Arrow(start=tip_start, direction=direction, scale=0.25),
                             color=color, opacity=0.9, pickable=False)
        plotter.add_point_labels(
            np.array(((3.2, 0, 0), (0, 3.2, 0), (0, 0, 3.2))),
            ["N", "E", "D"],
            text_color="white", font_size=16, point_size=0,
            show_points=False, shape=None, always_visible=True,
        )
        body_actors: dict[int, list] = {}
        no_data: dict[int, object] = {}
        picker = vtkPropPicker()
        picker.PickFromListOn()
        for index in indices:
            actors = [
                plotter.add_mesh(self._body_mesh, color=_STREAM_COLORS[index], smooth_shading=True),
                plotter.add_mesh(self._nose_mesh, color="white", smooth_shading=True),
            ]
            for direction, color in zip(((1, 0, 0), (0, 1, 0), (0, 0, 1)), _AXIS_COLORS):
                axis = pv.Arrow(start=(0, 0, 0), direction=direction, scale=0.95, shaft_radius=0.025)
                actors.append(plotter.add_mesh(axis, color=color, pickable=False))
            body_actors[index] = actors
            for actor in actors[:2]:
                picker.AddPickList(actor)
            no_data_actor = plotter.add_text(
                f"{'A' if index == 0 else 'B'}: No data",
                position=(12, 46 + 27 * index), font_size=13, color="white",
            )
            no_data_actor.SetVisibility(False)
            no_data[index] = no_data_actor
        if indices == (0, 1):
            caption_text = "A cyan / B amber; offset display only | NED fixed; FRD body axes"
        else:
            caption_text = f"{'A cyan' if indices == (0,) else 'B amber'} | NED fixed; FRD body axes"
        caption = plotter.add_text(caption_text, position="lower_left", font_size=11, color="white")
        self._set_camera(plotter)
        view = _View(plotter, indices, body_actors, no_data, caption, picker)
        plotter.setMouseTracking(True)
        plotter.installEventFilter(self)
        plotter.interactor.AddObserver("MouseMoveEvent", lambda _obj, _event: self._hover(view))
        return view

    @staticmethod
    def _set_camera(plotter: QtInteractor) -> None:
        plotter.camera_position = _CAMERA
        plotter.enable_parallel_projection()
        plotter.camera.parallel_scale = 4.0

    def set_source_names(self, names: tuple[str, ...]) -> None:
        self._source_names = tuple(names)
        QToolTip.hideText()

    def _source_for_actor(self, view: _View, picked: object) -> str | None:
        if picked is None:
            return None
        for index in view.indices:
            if index < len(self._source_names):
                for actor in view.body_actors[index][:2]:
                    if picked == actor and actor.GetVisibility():
                        return self._source_names[index]
        return None

    def _hover(self, view: _View) -> None:
        if self._closed:
            return
        x, y = view.interactor.interactor.GetEventPosition()
        view.picker.Pick(x, y, 0, view.interactor.renderer)
        name = self._source_for_actor(view, view.picker.GetActor())
        if name:
            QToolTip.showText(QCursor.pos(), name, view.interactor)
        else:
            QToolTip.hideText()

    def eventFilter(self, watched, event) -> bool:
        if event.type() == QEvent.Type.Leave and any(
            watched is view.interactor for view in self._views
        ):
            QToolTip.hideText()
        return super().eventFilter(watched, event)

    def set_layout(self, layout: Layout) -> None:
        if self._closed:
            raise RuntimeError("scene is closed")
        if layout not in ("single", "side_by_side", "overlay"):
            raise ValueError(f"unknown scene layout: {layout}")
        if self._views and layout == self._layout_mode:
            return
        QToolTip.hideText()
        for view in self._views:
            view.interactor.removeEventFilter(self)
            self._row.removeWidget(view.interactor)
            view.interactor.close()
            view.interactor.setParent(None)
            view.interactor.deleteLater()
        self._views.clear()
        self._layout_mode = layout
        groups = ((0,), (1,)) if layout == "side_by_side" else ((0, 1),) if layout == "overlay" else ((0,),)
        for indices in groups:
            self._views.append(self._build_view(indices))
        self.show_samples(self._samples, self._timestamp_ns)

    def show_samples(self, samples: tuple[AttitudeSample | None, ...], timestamp_ns: int) -> None:
        if self._closed:
            raise RuntimeError("scene is closed")
        self._samples = tuple(samples)
        self._timestamp_ns = timestamp_ns
        for view in self._views:
            for index in view.indices:
                sample = samples[index] if index < len(samples) else None
                visible = sample is not None
                for actor in view.body_actors[index]:
                    actor.SetVisibility(visible)
                    if sample is not None:
                        offset = overlay_offset(index) if self._layout_mode == "overlay" else (0.0, 0.0, 0.0)
                        actor.user_matrix = np.asarray(actor_transform(sample, offset))
                view.no_data[index].SetVisibility(not visible)
            view.interactor.render()

    def reset_cameras(self) -> None:
        for view in self._views:
            self._set_camera(view.interactor)
            view.interactor.render()

    def close(self) -> bool:
        if not self._closed:
            QToolTip.hideText()
            self._closed = True
            for view in self._views:
                view.interactor.removeEventFilter(self)
                self._row.removeWidget(view.interactor)
                view.interactor.close()
                view.interactor.setParent(None)
                view.interactor.deleteLater()
            self._views.clear()
        return super().close()
