"""Vision line-following helpers for the UCAR Web panel."""

import math
import time

import numpy as np


SUPPORTED_COLORS = ("black", "white", "yellow")


def clamp(value, low, high):
    try:
        value = float(value)
    except (TypeError, ValueError):
        value = low
    if not math.isfinite(value):
        value = low
    return max(low, min(high, value))


class LineFollowConfig:
    def __init__(
        self,
        line_color="yellow",
        roi_top=0.45,
        roi_bottom=0.95,
        linear_speed=0.06,
        angular_gain=0.45,
        min_area=250.0,
        black_threshold=170.0,
        enabled=False,
    ):
        self.line_color = line_color if line_color in SUPPORTED_COLORS else "black"
        self.roi_top = clamp(roi_top, 0.0, 0.95)
        self.roi_bottom = clamp(roi_bottom, 0.05, 1.0)
        if self.roi_bottom <= self.roi_top:
            self.roi_bottom = min(1.0, self.roi_top + 0.10)
            if self.roi_bottom <= self.roi_top:
                self.roi_top = max(0.0, self.roi_bottom - 0.10)
        self.linear_speed = clamp(linear_speed, 0.0, 0.10)
        self.angular_gain = clamp(angular_gain, 0.0, 0.60)
        self.min_area = clamp(min_area, 20.0, 20000.0)
        self.black_threshold = clamp(black_threshold, 60.0, 230.0)
        self.enabled = bool(enabled)

    @classmethod
    def from_dict(cls, data, current=None):
        data = data or {}
        base = current or cls()
        return cls(
            line_color=data.get("line_color", base.line_color),
            roi_top=data.get("roi_top", base.roi_top),
            roi_bottom=data.get("roi_bottom", base.roi_bottom),
            linear_speed=data.get("linear_speed", base.linear_speed),
            angular_gain=data.get("angular_gain", base.angular_gain),
            min_area=data.get("min_area", base.min_area),
            black_threshold=data.get("black_threshold", base.black_threshold),
            enabled=data.get("enabled", base.enabled),
        )

    def to_dict(self):
        return {
            "line_color": self.line_color,
            "roi_top": self.roi_top,
            "roi_bottom": self.roi_bottom,
            "linear_speed": self.linear_speed,
            "angular_gain": self.angular_gain,
            "min_area": self.min_area,
            "black_threshold": self.black_threshold,
            "enabled": self.enabled,
        }


class LineFollowResult:
    def __init__(
        self,
        detected=False,
        offset=0.0,
        center_x=None,
        center_y=None,
        area=0.0,
        linear_x=0.0,
        angular_z=0.0,
        forbidden_blocked=False,
        forbidden_area=0.0,
        message="no frame",
        stamp=None,
    ):
        self.detected = bool(detected)
        self.offset = float(offset)
        self.center_x = center_x
        self.center_y = center_y
        self.area = float(area)
        self.linear_x = float(linear_x)
        self.angular_z = float(angular_z)
        self.forbidden_blocked = bool(forbidden_blocked)
        self.forbidden_area = float(forbidden_area)
        self.message = message
        self.stamp = stamp if stamp is not None else time.time()

    def to_dict(self):
        return {
            "detected": self.detected,
            "offset": self.offset,
            "center_x": self.center_x,
            "center_y": self.center_y,
            "area": self.area,
            "linear_x": self.linear_x,
            "angular_z": self.angular_z,
            "forbidden_blocked": self.forbidden_blocked,
            "forbidden_area": self.forbidden_area,
            "message": self.message,
            "stamp": self.stamp,
        }


class LineFollower:
    def __init__(self, config=None):
        self.config = config or LineFollowConfig()
        self.last_result = LineFollowResult()
        self.last_debug_frame = None

    def update_config(self, config):
        self.config = config

    def process(self, frame_bgr):
        if frame_bgr is None:
            return self._store_result(LineFollowResult(message="no camera frame"))
        frame = np.asarray(frame_bgr)
        if frame.ndim != 3 or frame.shape[2] < 3 or frame.shape[0] < 10 or frame.shape[1] < 10:
            return self._store_result(LineFollowResult(message="invalid frame"))

        height, width = frame.shape[:2]
        y1 = int(height * self.config.roi_top)
        y2 = int(height * self.config.roi_bottom)
        y1 = max(0, min(height - 1, y1))
        y2 = max(y1 + 1, min(height, y2))
        roi = frame[y1:y2, :, :3]
        mask = self._mask_for_color(roi, self.config.line_color)
        forbidden_mask = self._solid_forbidden_mask(roi)
        if self.config.line_color == "black":
            line_mask, path_points = self._trace_black_path(mask)
        else:
            line_mask = self._extract_line_region(mask)
            path_points = self._points_from_mask(line_mask)
        ys, xs = np.nonzero(line_mask)
        area = float(xs.size)
        forbidden_blocked = self._path_touches_forbidden(path_points, forbidden_mask)
        forbidden_area = float(forbidden_mask.sum())

        debug = self._make_debug_frame(frame, line_mask, y1, y2, None, None, forbidden_mask)
        if area < self.config.min_area:
            result = LineFollowResult(
                detected=False,
                area=area,
                linear_x=0.0,
                angular_z=0.0,
                forbidden_blocked=forbidden_blocked,
                forbidden_area=forbidden_area,
                message="未检测到足够面积的路线",
            )
            self.last_debug_frame = debug
            return self._store_result(result)

        center_x = float(xs.mean())
        center_y = float(ys.mean() + y1)
        offset = (center_x - (width / 2.0)) / (width / 2.0)
        angular_z = -offset * self.config.angular_gain
        angular_z = max(-0.60, min(0.60, angular_z))
        linear_x = 0.0 if forbidden_blocked else self.config.linear_speed
        message = "红/黄实线过近，已禁止巡线控制" if forbidden_blocked else "已检测到路线"
        result = LineFollowResult(
            detected=True,
            offset=offset,
            center_x=center_x,
            center_y=center_y,
            area=area,
            linear_x=linear_x,
            angular_z=angular_z,
            forbidden_blocked=forbidden_blocked,
            forbidden_area=forbidden_area,
            message=message,
        )
        self.last_debug_frame = self._make_debug_frame(frame, line_mask, y1, y2, center_x, center_y, forbidden_mask)
        return self._store_result(result)

    def _store_result(self, result):
        self.last_result = result
        return result

    def _mask_for_color(self, roi, color):
        b = roi[:, :, 0].astype(np.int16)
        g = roi[:, :, 1].astype(np.int16)
        r = roi[:, :, 2].astype(np.int16)
        if color == "white":
            mask = (r > 190) & (g > 190) & (b > 190) & (np.maximum.reduce([r, g, b]) - np.minimum.reduce([r, g, b]) < 45)
        elif color == "yellow":
            mask = (r > 90) & (g > 85) & (b < 135) & (r > b + 20) & (g > b + 15) & (np.abs(r - g) < 110)
        else:
            bright = np.maximum.reduce([r, g, b])
            chroma = bright - np.minimum.reduce([r, g, b])
            mask = (bright < self.config.black_threshold) & (chroma < 55)
        return mask

    def _extract_line_region(self, mask):
        component = self._largest_component(mask)
        if component is not None:
            return component
        return self._extract_line_columns(mask)

    def _trace_black_path(self, mask):
        height, width = mask.shape[:2]
        selected = np.zeros_like(mask, dtype=bool)
        points = []
        prev_x = width / 2.0
        max_jump = max(24.0, width * 0.28)
        min_run_width = 3
        for y in range(height - 1, -1, -1):
            runs = self._row_runs(mask[y], min_run_width)
            if not runs:
                continue
            scored = []
            for start, end in runs:
                center = (start + end - 1) / 2.0
                scored.append((abs(center - prev_x), center, start, end))
            distance, center, start, end = min(scored, key=lambda item: item[0])
            if not points and distance > max_jump:
                continue
            if points and distance > max_jump:
                continue
            selected[y, start:end] = True
            points.append((center, y))
            prev_x = center
        min_path_rows = max(30, int(height * 0.25))
        if len(points) < min_path_rows:
            return np.zeros_like(mask, dtype=bool), []
        return selected, points

    def _row_runs(self, row, min_width):
        runs = []
        start = None
        for idx, value in enumerate(row):
            if value and start is None:
                start = idx
            elif not value and start is not None:
                if idx - start >= min_width:
                    runs.append((start, idx))
                start = None
        if start is not None and len(row) - start >= min_width:
            runs.append((start, len(row)))
        return runs

    def _points_from_mask(self, mask):
        points = []
        ys, xs = np.nonzero(mask)
        if ys.size == 0:
            return points
        for y in np.unique(ys):
            row_xs = xs[ys == y]
            points.append((float(row_xs.mean()), int(y)))
        return points

    def _solid_forbidden_mask(self, roi):
        b = roi[:, :, 0].astype(np.int16)
        g = roi[:, :, 1].astype(np.int16)
        r = roi[:, :, 2].astype(np.int16)
        red = (r > 120) & (r > g + 35) & (r > b + 35)
        yellow = (r > 90) & (g > 85) & (b < 140) & (r > b + 20) & (g > b + 15) & (np.abs(r - g) < 120)
        return self._solid_components(red | yellow)

    def _solid_components(self, mask):
        solid = np.zeros_like(mask, dtype=bool)
        for component in self._component_masks(mask):
            ys, xs = np.nonzero(component)
            if ys.size == 0:
                continue
            width = int(xs.max() - xs.min() + 1)
            height = int(ys.max() - ys.min() + 1)
            area = int(ys.size)
            if area >= 120 and max(width, height) >= 45:
                solid |= component
        return solid

    def _component_masks(self, mask):
        try:
            import cv2
            labels_count, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype("uint8"), 8)
            components = []
            for label in range(1, labels_count):
                if stats[label, cv2.CC_STAT_AREA] > 0:
                    components.append(labels == label)
            return components
        except Exception:
            return self._component_masks_fallback(mask)

    def _component_masks_fallback(self, mask):
        visited = np.zeros_like(mask, dtype=bool)
        components = []
        height, width = mask.shape[:2]
        for y, x in zip(*np.nonzero(mask)):
            if visited[y, x]:
                continue
            stack = [(int(y), int(x))]
            visited[y, x] = True
            coords = []
            while stack:
                cy, cx = stack.pop()
                coords.append((cy, cx))
                for ny in range(max(0, cy - 1), min(height, cy + 2)):
                    for nx in range(max(0, cx - 1), min(width, cx + 2)):
                        if not visited[ny, nx] and mask[ny, nx]:
                            visited[ny, nx] = True
                            stack.append((ny, nx))
            component = np.zeros_like(mask, dtype=bool)
            if coords:
                cy, cx = zip(*coords)
                component[list(cy), list(cx)] = True
                components.append(component)
        return components

    def _path_touches_forbidden(self, points, forbidden_mask):
        if not points or not np.any(forbidden_mask):
            return False
        height, width = forbidden_mask.shape[:2]
        contact_band_start = int(height * 0.94)
        contact_mask = np.zeros_like(forbidden_mask, dtype=bool)
        contact_mask[contact_band_start:, :] = forbidden_mask[contact_band_start:, :]
        if not np.any(contact_mask):
            return False
        for x, y in points:
            if y < contact_band_start:
                continue
            xi = int(round(x))
            yi = int(round(y))
            y0 = max(0, yi - 8)
            y1 = min(height, yi + 9)
            x0 = max(0, xi - 14)
            x1 = min(width, xi + 15)
            if np.any(contact_mask[y0:y1, x0:x1]):
                return True
        return False

    def _largest_component(self, mask):
        try:
            import cv2
            labels_count, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype("uint8"), 8)
        except Exception:
            return None
        if labels_count <= 1:
            return np.zeros_like(mask, dtype=bool)
        areas = stats[1:, cv2.CC_STAT_AREA]
        best = int(np.argmax(areas)) + 1
        return labels == best

    def _extract_line_columns(self, mask):
        roi_height = mask.shape[0]
        column_counts = mask.sum(axis=0)
        min_column_pixels = max(6, int(roi_height * 0.28))
        good_columns = column_counts >= min_column_pixels
        if not np.any(good_columns):
            return np.zeros_like(mask, dtype=bool)
        return mask & good_columns.reshape(1, -1)

    def _make_debug_frame(self, frame, mask, y1, y2, center_x, center_y, forbidden_mask=None):
        debug = frame[:, :, :3].copy()
        height, width = debug.shape[:2]
        overlay = debug[y1:y2]
        if forbidden_mask is not None and forbidden_mask.shape[:2] == overlay.shape[:2]:
            overlay[forbidden_mask] = (255, 0, 255)
        if mask.shape[:2] == overlay.shape[:2]:
            overlay[mask] = (0, 0, 255)
        self._draw_horizontal(debug, y1, (255, 160, 0))
        self._draw_horizontal(debug, max(y1, y2 - 1), (255, 160, 0))
        self._draw_vertical(debug, int(width / 2), (255, 255, 255))
        if center_x is not None and center_y is not None:
            self._draw_cross(debug, int(center_x), int(center_y), (0, 255, 0))
        return debug

    def _draw_horizontal(self, frame, y, color):
        y = max(0, min(frame.shape[0] - 1, int(y)))
        frame[y:y + 2, :, :] = color

    def _draw_vertical(self, frame, x, color):
        x = max(0, min(frame.shape[1] - 1, int(x)))
        frame[:, x:x + 2, :] = color

    def _draw_cross(self, frame, x, y, color):
        y0 = max(0, y - 8)
        y1 = min(frame.shape[0], y + 9)
        x0 = max(0, x - 8)
        x1 = min(frame.shape[1], x + 9)
        frame[y0:y1, x:x + 2, :] = color
        frame[y:y + 2, x0:x1, :] = color


def encode_jpeg(frame_bgr, quality=80):
    frame = np.asarray(frame_bgr)
    try:
        import cv2
        ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
        if ok:
            return jpeg.tobytes()
    except Exception:
        pass
    from PIL import Image
    image = Image.fromarray(frame[:, :, ::-1])
    import io
    buf = io.BytesIO()
    image.save(buf, format="JPEG", quality=int(quality))
    return buf.getvalue()


def decode_jpeg(jpeg_bytes):
    if not jpeg_bytes:
        return None
    try:
        import cv2
        arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
        return cv2.imdecode(arr, cv2.IMREAD_COLOR)
    except Exception:
        from PIL import Image
        import io
        image = Image.open(io.BytesIO(jpeg_bytes)).convert("RGB")
        rgb = np.asarray(image)
        return rgb[:, :, ::-1].copy()
