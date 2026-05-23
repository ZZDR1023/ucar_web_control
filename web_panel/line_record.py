"""视觉巡线样本记录工具。"""

import json
import re
import threading
import time
from pathlib import Path


def safe_session_name(name):
    text = str(name or "").strip()
    if not text:
        return "manual_line_follow"
    text = re.sub(r"[^0-9A-Za-z_.\-\u4e00-\u9fff]+", "_", text)
    text = text.strip("._-")
    return text[:48] or "manual_line_follow"


class LineFollowRecorder:
    def __init__(self, root_dir):
        self.root_dir = Path(root_dir)
        self.lock = threading.Lock()
        self.active = False
        self.session = None
        self.sample_count = 0

    def start(self, name="", config=None):
        with self.lock:
            if self.active:
                return self.snapshot_unlocked()
            now = time.time()
            stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(now))
            session_name = "{}_{}".format(stamp, safe_session_name(name))
            path = self.root_dir / session_name
            (path / "frames").mkdir(parents=True, exist_ok=False)
            (path / "debug").mkdir(parents=True, exist_ok=True)
            self.sample_count = 0
            self.session = {
                "active": True,
                "name": str(name or "").strip() or "manual_line_follow",
                "path": str(path),
                "started_at": now,
                "ended_at": None,
                "sample_count": 0,
                "config": dict(config or {}),
            }
            self.active = True
            self._write_manifest_unlocked()
            return self.snapshot_unlocked()

    def stop(self):
        with self.lock:
            if self.session is None:
                return self.snapshot_unlocked()
            self.active = False
            self.session["active"] = False
            self.session["ended_at"] = time.time()
            self.session["sample_count"] = self.sample_count
            self._write_manifest_unlocked()
            return self.snapshot_unlocked()

    def snapshot(self):
        with self.lock:
            return self.snapshot_unlocked()

    def snapshot_unlocked(self):
        if self.session is None:
            return {
                "active": False,
                "name": "",
                "path": "",
                "started_at": None,
                "ended_at": None,
                "sample_count": 0,
            }
        snapshot = dict(self.session)
        snapshot["active"] = self.active
        snapshot["sample_count"] = self.sample_count
        return snapshot

    def record_sample(self, frame_jpeg, debug_jpeg, metadata):
        if not frame_jpeg:
            return False
        with self.lock:
            if not self.active or self.session is None:
                return False
            self.sample_count += 1
            index = self.sample_count
            root = Path(self.session["path"])
            frame_rel = "frames/frame_{:06d}.jpg".format(index)
            debug_rel = "debug/debug_{:06d}.jpg".format(index)
            (root / frame_rel).write_bytes(frame_jpeg)
            if debug_jpeg:
                (root / debug_rel).write_bytes(debug_jpeg)
            else:
                debug_rel = None
            row = dict(metadata or {})
            row["index"] = index
            row["stamp"] = time.time()
            row["frame"] = frame_rel
            row["debug"] = debug_rel
            with open(root / "metadata.jsonl", "a", encoding="utf-8") as fh:
                fh.write(json.dumps(row, ensure_ascii=False, sort_keys=True) + "\n")
            self.session["sample_count"] = self.sample_count
            return True

    def _write_manifest_unlocked(self):
        if self.session is None:
            return
        path = Path(self.session["path"]) / "manifest.json"
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(self.snapshot_unlocked(), fh, ensure_ascii=False, indent=2, sort_keys=True)
