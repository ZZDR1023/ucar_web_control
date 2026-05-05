import json
import tempfile
import unittest
from pathlib import Path

from web_panel.patrol import PatrolManager


class PatrolManagerTest(unittest.TestCase):
    def make_manager(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        root = Path(tmp.name)
        return PatrolManager(
            root / "patrol_route.json",
            routes_path=root / "patrol_routes.json",
            runs_path=root / "patrol_runs.json",
        )

    def test_add_point_persists_route_without_starting_patrol(self):
        manager = self.make_manager()

        point = manager.add_point("door", 1.2, -0.4, 0.5)

        self.assertEqual(point["name"], "door")
        self.assertEqual(manager.snapshot()["state"]["mode"], "idle")
        with open(manager.route_path, "r", encoding="utf-8") as fh:
            saved = json.load(fh)
        self.assertEqual(saved["points"][0]["name"], "door")
        self.assertAlmostEqual(saved["points"][0]["x"], 1.2)

    def test_add_point_rejects_empty_name_and_non_finite_coordinates(self):
        manager = self.make_manager()

        with self.assertRaises(ValueError):
            manager.add_point("", 0.0, 0.0, 0.0)
        with self.assertRaises(ValueError):
            manager.add_point("bad", float("nan"), 0.0, 0.0)

    def test_delete_point_persists_remaining_route(self):
        manager = self.make_manager()
        manager.add_point("a", 0.0, 0.0, 0.0)
        manager.add_point("b", 1.0, 0.0, 0.0)

        deleted = manager.delete_point(0)

        self.assertEqual(deleted["name"], "a")
        self.assertEqual([p["name"] for p in manager.snapshot()["points"]], ["b"])
        with open(manager.route_path, "r", encoding="utf-8") as fh:
            saved = json.load(fh)
        self.assertEqual([p["name"] for p in saved["points"]], ["b"])

    def test_start_requires_at_least_one_point(self):
        manager = self.make_manager()

        with self.assertRaises(ValueError):
            manager.start()
        self.assertEqual(manager.snapshot()["state"]["mode"], "idle")

    def test_start_pause_resume_and_stop_keep_current_index(self):
        manager = self.make_manager()
        manager.add_point("door", 1.0, 0.0, 0.0)
        manager.add_point("desk", 2.0, 0.0, 0.0)

        current = manager.start()
        self.assertEqual(current["name"], "door")
        self.assertEqual(manager.snapshot()["state"]["mode"], "running")
        self.assertEqual(manager.snapshot()["state"]["current_index"], 0)

        manager.pause("operator paused")
        paused = manager.snapshot()["state"]
        self.assertEqual(paused["mode"], "paused")
        self.assertEqual(paused["current_index"], 0)

        resumed = manager.resume()
        self.assertEqual(resumed["name"], "door")
        self.assertEqual(manager.snapshot()["state"]["mode"], "running")

        manager.stop("operator stopped")
        stopped = manager.snapshot()["state"]
        self.assertEqual(stopped["mode"], "stopped")
        self.assertEqual(stopped["current_index"], 0)
        self.assertEqual(len(manager.snapshot()["points"]), 2)

    def test_blocked_state_requires_explicit_resume(self):
        manager = self.make_manager()
        manager.add_point("door", 1.0, 0.0, 0.0)
        manager.start()

        manager.handle_blocked(True)
        self.assertEqual(manager.snapshot()["state"]["mode"], "blocked")

        manager.handle_blocked(False)
        self.assertEqual(manager.snapshot()["state"]["mode"], "paused")

        current = manager.resume()
        self.assertEqual(current["name"], "door")
        self.assertEqual(manager.snapshot()["state"]["mode"], "running")

    def test_reaching_points_advances_and_finishes(self):
        manager = self.make_manager()
        manager.add_point("door", 1.0, 0.0, 0.0)
        manager.add_point("desk", 2.0, 0.0, 0.0)
        manager.start()

        next_point = manager.mark_current_reached(capture_path="capture1.jpg")
        state = manager.snapshot()["state"]
        self.assertEqual(next_point["name"], "desk")
        self.assertEqual(state["mode"], "running")
        self.assertEqual(state["current_index"], 1)
        self.assertEqual(state["captures"][0]["path"], "capture1.jpg")

        final = manager.mark_current_reached(capture_path="capture2.jpg")
        state = manager.snapshot()["state"]
        self.assertIsNone(final)
        self.assertEqual(state["mode"], "finished")
        self.assertEqual(state["current_index"], 1)
        self.assertEqual(state["captures"][-1]["point_name"], "desk")

    def test_save_and_load_named_routes(self):
        manager = self.make_manager()
        manager.add_point("door", 1.0, 0.0, 0.0)
        manager.add_point("desk", 2.0, 0.0, 1.57)

        route = manager.save_named_route("morning")
        self.assertEqual(route["name"], "morning")
        self.assertEqual(len(route["points"]), 2)
        self.assertEqual(manager.snapshot()["current_route_name"], "morning")

        manager.delete_point(1)
        self.assertEqual(len(manager.snapshot()["points"]), 1)

        loaded = manager.load_named_route("morning")
        self.assertEqual(len(loaded["points"]), 2)
        self.assertEqual(manager.snapshot()["points"][1]["name"], "desk")
        self.assertEqual(manager.list_routes()[0]["name"], "morning")

    def test_route_name_is_required_when_saving_named_route(self):
        manager = self.make_manager()
        manager.add_point("door", 1.0, 0.0, 0.0)

        with self.assertRaises(ValueError):
            manager.save_named_route("")

    def test_start_creates_run_and_captures_are_recorded(self):
        manager = self.make_manager()
        manager.add_point("door", 1.0, 0.0, 0.0)
        manager.save_named_route("morning")

        manager.start()
        self.assertEqual(manager.snapshot()["state"]["run"]["route_name"], "morning")

        manager.mark_current_reached(capture_path="/tmp/door.jpg")
        snapshot = manager.snapshot()
        run = snapshot["state"]["run"]
        self.assertEqual(run["status"], "finished")
        self.assertEqual(run["captures"][0]["path"], "/tmp/door.jpg")
        self.assertEqual(run["captures"][0]["point_name"], "door")
        self.assertEqual(manager.list_runs()[0]["id"], run["id"])

    def test_stop_updates_active_run_status(self):
        manager = self.make_manager()
        manager.add_point("door", 1.0, 0.0, 0.0)
        manager.save_named_route("morning")
        manager.start()

        manager.stop("operator stopped")

        run = manager.snapshot()["state"]["run"]
        self.assertEqual(run["status"], "stopped")
        self.assertEqual(manager.list_runs()[0]["status"], "stopped")


if __name__ == "__main__":
    unittest.main()
