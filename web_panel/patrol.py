#!/usr/bin/env python3
"""Patrol route persistence and execution state for the UCAR Web panel."""

import json
import math
import re
import threading
import time
from pathlib import Path


class PatrolManager:
    def __init__(self, route_path):
        self.route_path = Path(route_path)
        self.lock = threading.RLock()
        self.points = []
        self.state = {
            "mode": "idle",
            "current_index": None,
            "current_point": None,
            "last_event": "not started",
            "captures": [],
        }
        self.load_route()

    def load_route(self):
        with self.lock:
            if not self.route_path.exists():
                self.points = []
                return []
            with open(self.route_path, "r", encoding="utf-8") as fh:
                data = json.load(fh)
            self.points = [self._normalize_point(point) for point in data.get("points", [])]
            return list(self.points)

    def save_route(self):
        with self.lock:
            self.route_path.parent.mkdir(parents=True, exist_ok=True)
            payload = {"points": self.points}
            tmp_path = self.route_path.with_suffix(self.route_path.suffix + ".tmp")
            with open(tmp_path, "w", encoding="utf-8") as fh:
                json.dump(payload, fh, ensure_ascii=False, indent=2)
                fh.write("\n")
            tmp_path.replace(self.route_path)
            return payload

    def add_point(self, name, x, y, yaw):
        with self.lock:
            point = self._normalize_point({"name": name, "x": x, "y": y, "yaw": yaw})
            self.points.append(point)
            self.save_route()
            self.state["last_event"] = "added patrol point {}".format(point["name"])
            return dict(point)

    def delete_point(self, index):
        with self.lock:
            index = self._validate_index(index)
            deleted = self.points.pop(index)
            if self.state["current_index"] == index:
                self.state["current_point"] = None
            elif self.state["current_index"] is not None and self.state["current_index"] > index:
                self.state["current_index"] -= 1
            self.save_route()
            self.state["last_event"] = "deleted patrol point {}".format(deleted["name"])
            return dict(deleted)

    def start(self, start_index=0):
        with self.lock:
            if not self.points:
                raise ValueError("patrol route is empty")
            index = self._validate_index(start_index)
            self.state["mode"] = "running"
            self.state["current_index"] = index
            self.state["current_point"] = dict(self.points[index])
            self.state["last_event"] = "patrol started at {}".format(self.points[index]["name"])
            return dict(self.points[index])

    def pause(self, reason="patrol paused"):
        with self.lock:
            if self.state["mode"] in ("running", "blocked"):
                self.state["mode"] = "paused"
            self.state["last_event"] = reason
            return self.snapshot()["state"]

    def resume(self):
        with self.lock:
            if not self.points:
                raise ValueError("patrol route is empty")
            index = self.state["current_index"]
            if index is None:
                index = 0
            index = self._validate_index(index)
            self.state["mode"] = "running"
            self.state["current_index"] = index
            self.state["current_point"] = dict(self.points[index])
            self.state["last_event"] = "patrol resumed at {}".format(self.points[index]["name"])
            return dict(self.points[index])

    def stop(self, reason="patrol stopped"):
        with self.lock:
            self.state["mode"] = "stopped"
            self.state["last_event"] = reason
            return self.snapshot()["state"]

    def handle_blocked(self, blocked):
        with self.lock:
            if blocked and self.state["mode"] == "running":
                self.state["mode"] = "blocked"
                self.state["last_event"] = "patrol blocked by safety layer"
            elif not blocked and self.state["mode"] == "blocked":
                self.state["mode"] = "paused"
                self.state["last_event"] = "obstacle cleared; resume required"
            return self.snapshot()["state"]

    def mark_current_reached(self, capture_path=None, capture_error=None):
        with self.lock:
            index = self.state["current_index"]
            if index is None:
                raise ValueError("patrol is not active")
            index = self._validate_index(index)
            point = self.points[index]
            capture = {
                "time": time.strftime("%Y-%m-%d %H:%M:%S"),
                "point_index": index,
                "point_name": point["name"],
                "path": capture_path,
                "error": capture_error,
            }
            self.state["captures"].append(capture)
            if index + 1 >= len(self.points):
                self.state["mode"] = "finished"
                self.state["current_point"] = dict(point)
                self.state["last_event"] = "patrol finished"
                return None
            next_index = index + 1
            self.state["mode"] = "running"
            self.state["current_index"] = next_index
            self.state["current_point"] = dict(self.points[next_index])
            self.state["last_event"] = "patrol advanced to {}".format(self.points[next_index]["name"])
            return dict(self.points[next_index])

    def snapshot(self):
        with self.lock:
            return {
                "points": [dict(point) for point in self.points],
                "state": {
                    "mode": self.state["mode"],
                    "current_index": self.state["current_index"],
                    "current_point": dict(self.state["current_point"]) if self.state["current_point"] else None,
                    "last_event": self.state["last_event"],
                    "captures": [dict(capture) for capture in self.state["captures"]],
                },
            }

    def _validate_index(self, index):
        try:
            parsed = int(index)
        except (TypeError, ValueError):
            raise ValueError("invalid patrol point index")
        if parsed < 0 or parsed >= len(self.points):
            raise ValueError("patrol point index out of range")
        return parsed

    def _normalize_point(self, point):
        name = str(point.get("name", "")).strip()
        if not name:
            raise ValueError("patrol point name is required")
        x = self._finite_float(point.get("x"), "x")
        y = self._finite_float(point.get("y"), "y")
        yaw = self._finite_float(point.get("yaw"), "yaw")
        return {"name": name[:40], "x": x, "y": y, "yaw": yaw}

    def _finite_float(self, value, field):
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            raise ValueError("{} must be a finite number".format(field))
        if not math.isfinite(parsed):
            raise ValueError("{} must be a finite number".format(field))
        return parsed


def safe_name(name):
    cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(name).strip())
    return cleaned.strip("._") or "point"
