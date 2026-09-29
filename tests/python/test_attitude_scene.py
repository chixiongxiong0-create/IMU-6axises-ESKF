import math
import unittest
from unittest.mock import patch

from tools.attitude_viewer.data import AttitudeSample, AttitudeTrack
from tools.attitude_viewer.playback import ReplayController
from tools.attitude_viewer.scene import AttitudeScene, actor_transform, overlay_offset


def sample(quaternion=(1.0, 0.0, 0.0, 0.0)):
    return AttitudeSample(100, quaternion, 1.0, 0, 0, 0)


class SceneGeometryTests(unittest.TestCase):
    def test_yaw_transform_points_nose_east(self):
        sine = math.sqrt(0.5)
        matrix = actor_transform(sample((sine, 0.0, 0.0, sine)), (0.0, 0.0, 0.0))
        nose = tuple(matrix[row][0] for row in range(3))
        for actual, expected in zip(nose, (0.0, 1.0, 0.0)):
            self.assertAlmostEqual(actual, expected)

    def test_overlay_offsets_are_fixed_and_symmetric(self):
        first = overlay_offset(0)
        second = overlay_offset(1)
        self.assertEqual(first, (0.0, -0.9, 0.0))
        self.assertEqual(second, (0.0, 0.9, 0.0))
        self.assertEqual(actor_transform(sample(), first)[1][3], -0.9)
        self.assertEqual(actor_transform(sample(), second)[1][3], 0.9)


class SceneWidgetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PySide6.QtWidgets import QApplication

        cls.app = QApplication.instance() or QApplication([])

    def test_missing_sample_hides_body_and_overlay_marks_offsets_as_display_only(self):
        from PySide6.QtWidgets import QWidget

        parent = QWidget()
        scene = AttitudeScene(parent, layout="overlay")
        try:
            scene.show_samples((sample(), sample()), 100)
            self.assertTrue(scene._views[0].body_actors[1][0].GetVisibility())
            scene.show_samples((sample(), None), 101)
            self.assertFalse(scene._views[0].body_actors[1][0].GetVisibility())
            self.assertIn("display only", scene._views[0].caption.GetText(0).lower())
            self.assertIn("No data", scene._views[0].no_data[1].GetInput())
        finally:
            scene.close()
            parent.close()

    def test_layout_change_preserves_controller_clock_and_speed(self):
        from PySide6.QtWidgets import QWidget

        track = AttitudeTrack((sample(),), 0, "one")
        controller = ReplayController((track, track))
        controller.set_speed(2.0)
        controller.set_position_ns(100)
        parent = QWidget()
        scene = AttitudeScene(parent, layout="side_by_side")
        try:
            scene.show_samples(controller.selected_samples(), controller.position_ns)
            scene.set_layout("overlay")
            self.assertEqual(controller.position_ns, 100)
            self.assertEqual(controller.speed, 2.0)
            self.assertEqual(len(scene._views), 1)
        finally:
            scene.close()
            parent.close()

    def test_reference_axes_span_both_sides_of_scene(self):
        from PySide6.QtWidgets import QWidget

        parent = QWidget()
        scene = AttitudeScene(parent)
        try:
            actors = scene._views[0].interactor.actors
            for axis in "ned":
                positive = actors[f"ned-positive-{axis}"]
                negative = actors[f"ned-negative-{axis}"]
                self.assertGreater(positive.GetLength(), 2.5)
                self.assertGreater(negative.GetLength(), 2.5)
                self.assertLess(negative.GetProperty().GetOpacity(),
                                positive.GetProperty().GetOpacity())
        finally:
            scene.close()
            parent.close()

    def test_body_actor_maps_to_source_name_but_body_axes_do_not(self):
        from PySide6.QtWidgets import QWidget

        parent = QWidget()
        scene = AttitudeScene(parent, layout="overlay")
        try:
            scene.set_source_names(("truth.csv", "estimate.csv"))
            scene.show_samples((sample(), sample()), 100)
            view = scene._views[0]
            self.assertEqual(scene._source_for_actor(view, view.body_actors[0][0]), "truth.csv")
            self.assertEqual(scene._source_for_actor(view, view.body_actors[1][1]), "estimate.csv")
            self.assertIsNone(scene._source_for_actor(view, view.body_actors[0][2]))
        finally:
            scene.close()
            parent.close()

    def test_picker_selects_visible_body_instead_of_reference_axis(self):
        from PySide6.QtWidgets import QWidget

        parent = QWidget()
        scene = AttitudeScene(parent)
        try:
            scene.resize(640, 480)
            parent.show()
            scene.show()
            self.app.processEvents()
            scene.set_source_names(("flight.csv",))
            scene.show_samples((sample(),), 100)
            view = scene._views[0]
            renderer = view.interactor.renderer
            renderer.SetWorldPoint(0.0, 0.0, 0.0, 1.0)
            renderer.WorldToDisplay()
            x, y, _ = renderer.GetDisplayPoint()
            view.picker.Pick(int(x), int(y), 0, renderer)
            self.assertEqual(scene._source_for_actor(view, view.picker.GetActor()), "flight.csv")
            view.interactor.interactor.SetEventInformation(int(x), int(y), 0, 0, "0", 0, None)
            with patch("tools.attitude_viewer.scene.QToolTip.showText") as show_tip:
                scene._hover(view)
                self.assertEqual(show_tip.call_args.args[1], "flight.csv")
        finally:
            scene.close()
            parent.close()


if __name__ == "__main__":
    unittest.main()
