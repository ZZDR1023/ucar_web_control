#!/usr/bin/env python3
"""Patrol route persistence and execution state for the UCAR Web panel."""

import json
import math
import re
import threading
import time
from pathlib import Path


class PatrolManager:
    def __init__(self, route_path, routes_path=None, runs_path=None):
        self.route_path = Path(route_path)
        self.routes_path = Path(routes_path) if routes_path else self.route_path.with_name("patrol_routes.json")
        self.runs_path = Path(runs_path) if runs_path else self.route_path.with_name("patrol_runs.json")
        self.lock = threading.RLock()
        self.points = []
        self.current_route_name = ""
        self.routes = []
        self.runs = []
        self.current_run = None
        self.state = {
            "mode": "idle",
            "current_index": None,
            "current_point": None,
            "last_event": "not started",
            "captures": [],
        }
        self.load_route()
        self.load_routes()
        self.load_runs()

    def load_route(self):
        with self.lock:
            if not self.route_path.exists():
                self.points = []
                return []
            data = self._read_json(self.route_path, {"points": []})
            self.points = [self._normalize_point(point) for point in data.get("points", [])]
            self.current_route_name = str(data.get("route_name", "")).strip()
            return list(self.points)

    def save_route(self):
        with self.lock:
            payload = {"route_name": self.current_route_name, "points": self.points}
            self._write_json(self.route_path, payload)
            return payload

    def load_routes(self):
        with self.lock:
            data = self._read_json(self.routes_path, {"routes": []})
            routes = []
            for route in data.get("routes", []):
                name = self._normalize_route_name(route.get("name", ""))
                points = [self._normalize_point(point) for point in route.get("points", [])]
                routes.append({
                    "name": name,
                    "points": points,
                    "updated_at": route.get("updated_at") or self._now_text(),
                })
            self.routes = routes
            return list(self.routes)

    def save_routes(self):
        with self.lock:
            payload = {"routes": self.routes}
            self._write_json(self.routes_path, payload)
            return payload

    def save_named_route(self, name):
        with self.lock:
            route_name = self._normalize_route_name(name)
            route = {
                "name": route_name,
                "points": [dict(point) for point in self.points],
                "updated_at": self._now_text(),
            }
            replaced = False
            for idx, existing in enumerate(self.routes):
                if existing["name"] == route_name:
                    self.routes[idx] = route
                    replaced = True
                    break
            if not replaced:
                self.routes.append(route)
            self.current_route_name = route_name
            self.save_route()
            self.save_routes()
            self.state["last_event"] = "saved route {}".format(route_name)
            return dict(route)

    def load_named_route(self, name):
        with self.lock:
            route_name = self._normalize_route_name(name)
            for route in self.routes:
                if route["name"] == route_name:
                    self.points = [dict(point) for point in route["points"]]
                    self.current_route_name = route_name
                    self.state["mode"] = "idle"
                    self.state["current_index"] = None
                    self.state["current_point"] = None
                    self.state["captures"] = []
                    self.state["last_event"] = "loaded route {}".format(route_name)
                    self.current_run = None
                    self.save_route()
                    return {"name": route_name, "points": [dict(point) for point in self.points]}
            raise ValueError("patrol route not found")

    def list_routes(self):
        with self.lock:
            return [
                {
                    "name": route["name"],
                    "updated_at": route["updated_at"],
                    "point_count": len(route["points"]),
                    "points": [dict(point) for point in route["points"]],
                }
                for route in self.routes
            ]

    def load_runs(self):
        with self.lock:
            data = self._read_json(self.runs_path, {"runs": []})
            self.runs = list(data.get("runs", []))
            return list(self.runs)

    def save_runs(self):
        with self.lock:
            self.runs = self.runs[-50:]
            payload = {"runs": self.runs}
            self._write_json(self.runs_path, payload)
            return payload

    def list_runs(self):
        with self.lock:
            return [dict(run) for run in reversed(self.runs)]

    def get_run(self, run_id):
        with self.lock:
            for run in self.runs:
                if run.get("id") == run_id:
                    return dict(run)
            raise ValueError("patrol run not found")

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
            self.state["captures"] = []
            self.state["last_event"] = "patrol started at {}".format(self.points[index]["name"])
            self._start_run()
            return dict(self.points[index])

    def pause(self, reason="patrol paused"):
        with self.lock:
            if self.state["mode"] in ("running", "blocked"):
                self.state["mode"] = "paused"
                self._update_run_status("paused")
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
            self._update_run_status("running")
            return dict(self.points[index])

    def stop(self, reason="patrol stopped"):
        with self.lock:
            self.state["mode"] = "stopped"
            self.state["last_event"] = reason
            self._finish_run("stopped")
            return self.snapshot()["state"]

    def handle_blocked(self, blocked):
        with self.lock:
            if blocked and self.state["mode"] == "running":
                self.state["mode"] = "blocked"
                self.state["last_event"] = "patrol blocked by safety layer"
                self._update_run_status("blocked")
            elif not blocked and self.state["mode"] == "blocked":
                self.state["mode"] = "paused"
                self.state["last_event"] = "obstacle cleared; resume required"
                self._update_run_status("paused")
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
            self._record_capture(capture)
            if index + 1 >= len(self.points):
                self.state["mode"] = "finished"
                self.state["current_point"] = dict(point)
                self.state["last_event"] = "patrol finished"
                self._finish_run("finished")
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
                "current_route_name": self.current_route_name,
                "routes": self.list_routes(),
                "runs": self.list_runs(),
                "state": {
                    "mode": self.state["mode"],
                    "current_index": self.state["current_index"],
                    "current_point": dict(self.state["current_point"]) if self.state["current_point"] else None,
                    "last_event": self.state["last_event"],
                    "captures": [dict(capture) for capture in self.state["captures"]],
                    "run": dict(self.current_run) if self.current_run else None,
                },
            }

    def _start_run(self):
        run = {
            "id": time.strftime("%Y%m%d_%H%M%S"),
            "route_name": self.current_route_name or "未命名路线",
            "started_at": self._now_text(),
            "ended_at": None,
            "status": "running",
            "points": [dict(point) for point in self.points],
            "captures": [],
            "last_event": self.state["last_event"],
        }
        self.current_run = run
        self.runs.append(run)
        self.save_runs()
        return run

    def _update_run_status(self, status):
        if not self.current_run:
            return
        self.current_run["status"] = status
        self.current_run["last_event"] = self.state["last_event"]
        self.save_runs()

    def _finish_run(self, status):
        if not self.current_run:
            return
        self.current_run["status"] = status
        self.current_run["ended_at"] = self._now_text()
        self.current_run["last_event"] = self.state["last_event"]
        self.save_runs()

    def _record_capture(self, capture):
        if not self.current_run:
            return
        self.current_run.setdefault("captures", []).append(dict(capture))
        self.current_run["last_event"] = "captured {}".format(capture["point_name"])
        self.save_runs()

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

    def _normalize_route_name(self, name):
        route_name = str(name).strip()
        if not route_name:
            raise ValueError("patrol route name is required")
        return route_name[:60]

    def _finite_float(self, value, field):
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            raise ValueError("{} must be a finite number".format(field))
        if not math.isfinite(parsed):
            raise ValueError("{} must be a finite number".format(field))
        return parsed

    def _read_json(self, path, default):
        if not path.exists():
            return default
        with open(path, "r", encoding="utf-8") as fh:
            return json.load(fh)

    def _write_json(self, path, payload):
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp_path = path.with_suffix(path.suffix + ".tmp")
        with open(tmp_path, "w", encoding="utf-8") as fh:
            json.dump(payload, fh, ensure_ascii=False, indent=2)
            fh.write("\n")
        tmp_path.replace(path)

    def _now_text(self):
        return time.strftime("%Y-%m-%d %H:%M:%S")


def safe_name(name):
    cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(name).strip())
    return cleaned.strip("._") or "point"
