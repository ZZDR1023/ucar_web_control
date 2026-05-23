import json
import tempfile
import unittest
from pathlib import Path

from web_panel.line_record import LineFollowRecorder


class LineFollowRecorderTest(unittest.TestCase):
    def make_recorder(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        return LineFollowRecorder(Path(tmp.name))

    def test_start_creates_session_directories_and_manifest(self):
        recorder = self.make_recorder()

        session = recorder.start("manual lap", {"line_color": "black"})

        self.assertTrue(session["active"])
        self.assertTrue((Path(session["path"]) / "frames").is_dir())
        self.assertTrue((Path(session["path"]) / "debug").is_dir())
        with open(Path(session["path"]) / "manifest.json", "r", encoding="utf-8") as fh:
            manifest = json.load(fh)
        self.assertEqual(manifest["name"], "manual lap")
        self.assertEqual(manifest["config"]["line_color"], "black")

    def test_record_sample_writes_images_and_metadata(self):
        recorder = self.make_recorder()
        recorder.start("sample", {"line_color": "black"})

        saved = recorder.record_sample(
            b"frame-bytes",
            b"debug-bytes",
            {
                "cmd": {"linear_x": 0.1, "angular_z": 0.2},
                "line_follow": {"result": {"offset": 0.1}},
            },
        )

        self.assertTrue(saved)
        snapshot = recorder.snapshot()
        self.assertEqual(snapshot["sample_count"], 1)
        root = Path(snapshot["path"])
        self.assertEqual((root / "frames" / "frame_000001.jpg").read_bytes(), b"frame-bytes")
        self.assertEqual((root / "debug" / "debug_000001.jpg").read_bytes(), b"debug-bytes")
        with open(root / "metadata.jsonl", "r", encoding="utf-8") as fh:
            row = json.loads(fh.readline())
        self.assertEqual(row["index"], 1)
        self.assertEqual(row["frame"], "frames/frame_000001.jpg")
        self.assertEqual(row["debug"], "debug/debug_000001.jpg")
        self.assertEqual(row["cmd"]["linear_x"], 0.1)

    def test_record_sample_is_ignored_when_not_active(self):
        recorder = self.make_recorder()

        saved = recorder.record_sample(b"frame", b"debug", {"cmd": {}})

        self.assertFalse(saved)
        self.assertFalse(recorder.snapshot()["active"])

    def test_stop_marks_session_inactive(self):
        recorder = self.make_recorder()
        recorder.start("sample", {})

        stopped = recorder.stop()

        self.assertFalse(stopped["active"])
        self.assertIsNotNone(stopped["ended_at"])
        self.assertFalse(recorder.snapshot()["active"])


if __name__ == "__main__":
    unittest.main()
