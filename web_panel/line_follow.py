"""Vision line-following helpers for the UCAR Web panel."""

import math
import time

import numpy as np


SUPPORTED_COLORS = ("black", "white", "yellow")
HSV_DEFAULTS = {
    "black": (0.0, 179.0, 0.0, 80.0, 0.0, 200.0),
    "white": (0.0, 179.0, 0.0, 45.0, 190.0, 255.0),
    "yellow": (15.0, 45.0, 55.0, 255.0, 60.0, 255.0),
}


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
        line_color="black",
        roi_top=0.45,
        roi_bottom=0.95,
        linear_speed=0.06,
        angular_gain=0.60,
        min_area=250.0,
        h_min=None,
        h_max=None,
        s_min=None,
        s_max=None,
        v_min=None,
        v_max=None,
        black_threshold=None,
        enabled=False,
    ):
        self.line_color = line_color if line_color in SUPPORTED_COLORS else "black"
        defaults = HSV_DEFAULTS[self.line_color]
        if black_threshold is not None and v_max is None and self.line_color == "black":
            v_max = black_threshold
        self.roi_top = clamp(roi_top, 0.0, 0.95)
        self.roi_bottom = clamp(roi_bottom, 0.05, 1.0)
        if self.roi_bottom <= self.roi_top:
            self.roi_bottom = min(1.0, self.roi_top + 0.10)
            if self.roi_bottom <= self.roi_top:
                self.roi_top = max(0.0, self.roi_bottom - 0.10)
        self.linear_speed = clamp(linear_speed, 0.0, 0.10)
        self.angular_gain = clamp(angular_gain, 0.0, 0.60)
        self.min_area = clamp(min_area, 20.0, 20000.0)
        self.h_min = clamp(defaults[0] if h_min is None else h_min, 0.0, 179.0)
        self.h_max = clamp(defaults[1] if h_max is None else h_max, 0.0, 179.0)
        self.s_min = clamp(defaults[2] if s_min is None else s_min, 0.0, 255.0)
        self.s_max = clamp(defaults[3] if s_max is None else s_max, 0.0, 255.0)
        self.v_min = clamp(defaults[4] if v_min is None else v_min, 0.0, 255.0)
        self.v_max = clamp(defaults[5] if v_max is None else v_max, 0.0, 255.0)
        self.enabled = bool(enabled)

    @classmethod
    def from_dict(cls, data, current=None):
        data = data or {}
        base = current or cls()
        line_color = data.get("line_color", base.line_color)
        color_changed = current is not None and line_color != base.line_color
        defaults = HSV_DEFAULTS[line_color if line_color in SUPPORTED_COLORS else "black"]
        h_min = data.get("h_min", defaults[0] if color_changed else base.h_min)
        h_max = data.get("h_max", defaults[1] if color_changed else base.h_max)
        s_min = data.get("s_min", defaults[2] if color_changed else base.s_min)
        s_max = data.get("s_max", defaults[3] if color_changed else base.s_max)
        v_min = data.get("v_min", defaults[4] if color_changed else base.v_min)
        v_max = data.get("v_max", defaults[5] if color_changed else base.v_max)
        if "black_threshold" in data and "v_max" not in data and line_color == "black":
            v_max = data.get("black_threshold")
        return cls(
            line_color=line_color,
            roi_top=data.get("roi_top", base.roi_top),
            roi_bottom=data.get("roi_bottom", base.roi_bottom),
            linear_speed=data.get("linear_speed", base.linear_speed),
            angular_gain=data.get("angular_gain", base.angular_gain),
            min_area=data.get("min_area", base.min_area),
            h_min=h_min,
            h_max=h_max,
            s_min=s_min,
            s_max=s_max,
            v_min=v_min,
            v_max=v_max,
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
            "h_min": self.h_min,
            "h_max": self.h_max,
            "s_min": self.s_min,
            "s_max": self.s_max,
            "v_min": self.v_min,
            "v_max": self.v_max,
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
        self.search_angular_speed = 0.24
        self.steering_deadband = 0.07
        self.slow_turn_offset = 0.14
        self.turn_in_place_offset = 0.30
        self.max_follow_angular_speed = 0.42
        self.last_stable_center_x = None
        self.last_stable_offset = 0.0
        self.last_turn_sign = 0.0
        self.opposite_turn_frames = 0
        self.last_path_area = 0.0
        self.no_path_turn_sign = 0.0
        self.no_path_turn_flips = 0
        self.branch_search_sign = 0.0
        self.branch_search_stable_frames = 0
        self.branch_search_release_frames = 3

    def update_config(self, config):
        if config.line_color != self.config.line_color:
            self._reset_prediction()
        self.config = config

    def process(self, frame_bgr):
        if frame_bgr is None:
            return self._store_result(LineFollowResult(message="no camera frame"))
        frame = np.asarray(frame_bgr)
        if frame.ndim != 3 or frame.shape[2] < 3 or frame.shape[0] < 10 or frame.shape[1] < 10:
            return self._store_result(LineFollowResult(message="invalid frame"))

        height, width = frame.shape[:2]
        frame_hsv = self._bgr_to_hsv(frame[:, :, :3])
        y1 = int(height * self.config.roi_top)
        y2 = int(height * self.config.roi_bottom)
        y1 = max(0, min(height - 1, y1))
        y2 = max(y1 + 1, min(height, y2))
        roi = frame[y1:y2, :, :3]
        roi_hsv = frame_hsv[y1:y2, :, :]
        mask = self._mask_for_color(roi_hsv)
        forbidden_mask = self._solid_forbidden_mask(roi_hsv)
        forbidden_contact_mask = self._solid_forbidden_mask(frame_hsv[y1:, :, :])
        if self.config.line_color == "black":
            marking_mask = self._non_drivable_marking_mask(roi_hsv)
            mask = mask & ~marking_mask
            line_mask, path_points = self._trace_black_path(mask, self._predicted_path_center(width))
            if not path_points and np.any(mask):
                contrast_mask = self._black_contrast_mask(roi_hsv, mask) & ~marking_mask
                line_mask, path_points = self._trace_black_path(contrast_mask, self._predicted_path_center(width))
        else:
            line_mask = self._extract_line_region(mask)
            path_points = self._points_from_mask(line_mask)
        ys, xs = np.nonzero(line_mask)
        area = float(xs.size)
        contact_band_start = int((y2 - y1) * 0.94)
        forbidden_blocked = self._path_touches_forbidden(path_points, forbidden_contact_mask, contact_band_start)
        forbidden_avoid_angular = self._forbidden_avoidance_angular_z(forbidden_contact_mask, contact_band_start)
        forbidden_area = float(forbidden_mask.sum())
        near_field_seen = self._near_field_seen(line_mask)

        debug = self._make_debug_frame(frame, line_mask, y1, y2, None, None, forbidden_mask)
        if area < self.config.min_area:
            if forbidden_blocked:
                angular_z = 0.0
            else:
                angular_z = self._branch_search_angular_z(self._search_angular_z())
            result = LineFollowResult(
                detected=False,
                area=area,
                linear_x=0.0,
                angular_z=angular_z,
                forbidden_blocked=forbidden_blocked,
                forbidden_area=forbidden_area,
                message="前方禁压线可见且未检测到可走路线，停车等待" if forbidden_blocked else "未检测到足够面积的路线，原地搜索",
            )
            self.last_debug_frame = debug
            return self._store_result(result)

        center_x, roi_center_y, curve_offset = self._target_center_from_channel(line_mask, path_points, forbidden_mask)
        if center_x is None:
            center_x = float(xs.mean())
            roi_center_y = float(ys.mean())
        center_y = float(roi_center_y + y1)
        offset = (center_x - (width / 2.0)) / (width / 2.0)
        offset = self._stabilized_offset(offset, curve_offset, forbidden_blocked, near_field_seen)
        center_x = (width / 2.0) + offset * (width / 2.0)
        lower_branch_seen = self._lower_branch_seen(area, roi_center_y, y2 - y1, near_field_seen)
        area_drop_turn_sign = self._shrinking_path_turn_sign(area, offset, near_field_seen)
        stable_near_path = near_field_seen and not forbidden_blocked and abs(offset) <= self.slow_turn_offset
        linear_x, angular_z = self._motion_for_offset(offset, forbidden_blocked, near_field_seen)
        if lower_branch_seen and not near_field_seen:
            self._activate_branch_search(self._angular_sign(angular_z) or self.last_turn_sign or 1.0)
        if abs(offset) >= self.turn_in_place_offset and not forbidden_blocked and not near_field_seen:
            self._activate_branch_search(self._angular_sign(angular_z) or self.last_turn_sign or 1.0)
        if area_drop_turn_sign != 0.0 and not forbidden_blocked:
            self._activate_branch_search(area_drop_turn_sign)
        if self.branch_search_sign != 0.0:
            self._maybe_release_branch_search(stable_near_path)
        branch_search_active = self.branch_search_sign != 0.0
        if not near_field_seen or branch_search_active:
            angular_z = self._branch_search_angular_z(angular_z)
            linear_x = 0.0
        if lower_branch_seen and linear_x <= 0.0 and abs(angular_z) > 0.01:
            linear_x = 0.0
        if area_drop_turn_sign != 0.0 and not forbidden_blocked and not branch_search_active:
            angular_z = area_drop_turn_sign * max(abs(angular_z), self.max_follow_angular_speed)
            linear_x = 0.0
        if forbidden_blocked and forbidden_avoid_angular is not None:
            if branch_search_active:
                linear_x = 0.0
                angular_z = self._branch_search_angular_z(angular_z)
            elif lower_branch_seen and self._angular_sign_conflicts(angular_z, forbidden_avoid_angular):
                linear_x = 0.0
            else:
                linear_x = 0.0
                angular_z = forbidden_avoid_angular
        if forbidden_blocked and linear_x > 0.0:
            message = "分叉路径可见，低速脱离禁压边界"
        elif forbidden_blocked:
            message = "红/黄实线过近，已禁止巡线控制"
        elif not near_field_seen:
            message = "近场未检测到路线，原地转向对准"
        else:
            message = "已检测到路线"
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
        if result.detected and not result.forbidden_blocked and result.center_x is not None:
            if self.branch_search_sign != 0.0:
                self.last_turn_sign = self.branch_search_sign
                return result
            self.last_stable_center_x = float(result.center_x)
            self.last_stable_offset = float(result.offset)
            turn_sign = self._turn_sign_for_offset(result.offset)
            if turn_sign != 0.0:
                self.last_turn_sign = turn_sign
            elif abs(result.angular_z) > 0.01:
                self.last_turn_sign = 1.0 if result.angular_z > 0 else -1.0
            self.last_path_area = max(0.0, float(result.area))
        return result

    def _reset_prediction(self):
        self.last_stable_center_x = None
        self.last_stable_offset = 0.0
        self.last_turn_sign = 0.0
        self.opposite_turn_frames = 0
        self.last_path_area = 0.0
        self._reset_branch_search()

    def _reset_branch_search(self):
        self.no_path_turn_sign = 0.0
        self.no_path_turn_flips = 0
        self.branch_search_sign = 0.0
        self.branch_search_stable_frames = 0

    def _predicted_path_center(self, width):
        if self.last_stable_center_x is None:
            return None
        return max(0.0, min(float(width - 1), float(self.last_stable_center_x)))

    def _turn_sign_for_offset(self, offset):
        if abs(offset) <= self.steering_deadband:
            return 0.0
        return 1.0 if offset < 0.0 else -1.0

    def _stabilized_offset(self, offset, curve_offset, forbidden_blocked, near_field_seen):
        if forbidden_blocked:
            return offset
        if near_field_seen:
            offset = self._apply_curve_offset(offset, curve_offset)
        desired_turn = self._turn_sign_for_offset(offset)
        if not near_field_seen or desired_turn == 0.0 or self.last_turn_sign == 0.0:
            if desired_turn == 0.0:
                self.opposite_turn_frames = 0
            return offset
        if desired_turn == self.last_turn_sign:
            self.opposite_turn_frames = 0
            return offset

        self.opposite_turn_frames += 1
        curve_turn = self._turn_sign_for_offset(curve_offset)
        strong_reverse = abs(offset) >= 0.55
        curve_supports_reverse = curve_turn == desired_turn
        if strong_reverse or curve_supports_reverse or self.opposite_turn_frames >= 3:
            return offset

        held_offset = -self.last_turn_sign * max(self.steering_deadband + 0.02, min(abs(offset), 0.16))
        return held_offset

    def _apply_curve_offset(self, offset, curve_offset):
        if abs(curve_offset) <= self.steering_deadband:
            return offset
        curve_turn = self._turn_sign_for_offset(curve_offset)
        offset_turn = self._turn_sign_for_offset(offset)
        if offset_turn != 0.0 and offset_turn != curve_turn:
            return offset
        hinted = curve_offset * 0.60
        if abs(offset) >= abs(hinted):
            return offset
        return hinted

    def _search_angular_z(self):
        if self.last_result.detected and abs(self.last_result.offset) > 0.05:
            direction = -1.0 if self.last_result.offset > 0 else 1.0
        elif abs(self.last_result.angular_z) > 0.01:
            direction = 1.0 if self.last_result.angular_z > 0 else -1.0
        else:
            direction = 1.0
        return direction * self.search_angular_speed

    def _branch_search_angular_z(self, angular_z):
        sign = self._angular_sign(angular_z)
        if sign == 0.0:
            sign = self.branch_search_sign or self.last_turn_sign or 1.0
        if self.branch_search_sign != 0.0:
            return self._branch_search_speed() * self.branch_search_sign
        if self.no_path_turn_sign != 0.0 and sign != self.no_path_turn_sign:
            self.no_path_turn_flips += 1
            if self.no_path_turn_flips >= 2:
                self._activate_branch_search(sign)
        self.no_path_turn_sign = sign
        if self.branch_search_sign != 0.0:
            return self._branch_search_speed() * self.branch_search_sign
        return abs(angular_z if abs(angular_z) > 0.01 else self.search_angular_speed) * sign

    def _branch_search_speed(self):
        return clamp(self.config.angular_gain * 0.80, 0.14, self.max_follow_angular_speed)

    def _activate_branch_search(self, sign):
        sign = 1.0 if sign >= 0.0 else -1.0
        if self.branch_search_sign == 0.0:
            self.branch_search_sign = sign
        self.branch_search_stable_frames = 0

    def _maybe_release_branch_search(self, stable_near_path):
        if not stable_near_path:
            self.branch_search_stable_frames = 0
            return
        self.branch_search_stable_frames += 1
        if self.branch_search_stable_frames >= self.branch_search_release_frames:
            self._reset_branch_search()

    def _angular_sign(self, angular_z):
        if abs(angular_z) <= 0.01:
            return 0.0
        return 1.0 if angular_z > 0.0 else -1.0

    def _shrinking_path_turn_sign(self, area, offset, near_field_seen):
        if not near_field_seen or self.last_turn_sign == 0.0:
            return 0.0
        if self.last_path_area < self.config.min_area * 4.0:
            return 0.0
        if area > self.last_path_area * 0.72:
            return 0.0
        if abs(offset) > self.turn_in_place_offset:
            return 0.0
        return self.last_turn_sign

    def _motion_for_offset(self, offset, forbidden_blocked, near_field_seen=True):
        abs_offset = abs(offset)
        if abs_offset <= self.steering_deadband:
            angular_z = 0.0
        else:
            angular_z = -offset * self.config.angular_gain
            angular_z = max(-self.max_follow_angular_speed, min(self.max_follow_angular_speed, angular_z))
            if abs_offset >= self.turn_in_place_offset:
                angular_z = self._turn_sign_for_offset(offset) * self.max_follow_angular_speed

        if not near_field_seen:
            if abs(angular_z) <= 0.01:
                angular_z = self._search_angular_z()
            return 0.0, angular_z
        if forbidden_blocked:
            return 0.0, angular_z
        if abs_offset >= self.turn_in_place_offset:
            return 0.0, angular_z
        if abs_offset >= self.slow_turn_offset:
            return self.config.linear_speed * 0.45, angular_z
        return self.config.linear_speed, angular_z

    def _angular_sign_conflicts(self, first, second):
        if abs(first) <= 0.01 or abs(second) <= 0.01:
            return False
        return (first > 0.0) != (second > 0.0)

    def _lower_branch_seen(self, area, roi_center_y, roi_height, near_field_seen):
        if near_field_seen or roi_height <= 0:
            return False
        if area < self.config.min_area * 3.0:
            return False
        return float(roi_center_y) >= float(roi_height) * 0.55

    def _near_field_seen(self, line_mask):
        height, width = line_mask.shape[:2]
        if height <= 0 or width <= 0:
            return False
        start = max(0, int(height * 0.82))
        near = line_mask[start:, :]
        if not np.any(near):
            return False
        min_width = max(6, int(width * 0.025))
        min_rows = max(3, int(height * 0.03))
        rows_with_path = 0
        for row in near:
            if self._row_runs(row, min_width):
                rows_with_path += 1
        return rows_with_path >= min_rows

    def _bgr_to_hsv(self, frame_bgr):
        try:
            import cv2
            return cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        except Exception:
            return self._bgr_to_hsv_fallback(frame_bgr)

    def _bgr_to_hsv_fallback(self, frame_bgr):
        bgr = frame_bgr.astype(np.float32) / 255.0
        b = bgr[:, :, 0]
        g = bgr[:, :, 1]
        r = bgr[:, :, 2]
        maxc = np.maximum.reduce([r, g, b])
        minc = np.minimum.reduce([r, g, b])
        delta = maxc - minc
        hue = np.zeros_like(maxc)
        nonzero = delta > 1e-6
        rmax = nonzero & (maxc == r)
        gmax = nonzero & (maxc == g)
        bmax = nonzero & (maxc == b)
        hue[rmax] = ((g[rmax] - b[rmax]) / delta[rmax]) % 6.0
        hue[gmax] = ((b[gmax] - r[gmax]) / delta[gmax]) + 2.0
        hue[bmax] = ((r[bmax] - g[bmax]) / delta[bmax]) + 4.0
        hue = hue * 30.0
        saturation = np.zeros_like(maxc)
        valid_value = maxc > 1e-6
        saturation[valid_value] = delta[valid_value] / maxc[valid_value]
        hsv = np.zeros_like(frame_bgr, dtype=np.uint8)
        hsv[:, :, 0] = np.clip(hue, 0, 179).astype(np.uint8)
        hsv[:, :, 1] = np.clip(saturation * 255.0, 0, 255).astype(np.uint8)
        hsv[:, :, 2] = np.clip(maxc * 255.0, 0, 255).astype(np.uint8)
        return hsv

    def _mask_for_color(self, hsv_roi):
        mask = self._hsv_range_mask(
            hsv_roi,
            self.config.h_min,
            self.config.h_max,
            self.config.s_min,
            self.config.s_max,
            self.config.v_min,
            self.config.v_max,
        )
        return mask

    def _black_contrast_mask(self, hsv_roi, base_mask):
        value = hsv_roi[:, :, 2]
        candidate_values = value[base_mask]
        if candidate_values.size == 0:
            return np.zeros_like(base_mask, dtype=bool)
        low_value = float(np.percentile(candidate_values, 5))
        high_value = float(np.percentile(candidate_values, 95))
        if high_value - low_value < 20.0:
            return np.zeros_like(base_mask, dtype=bool)
        contrast_margin = min(50.0, max(12.0, (high_value - low_value) * 0.55))
        threshold = min(self.config.v_max, low_value + contrast_margin)
        return base_mask & (value <= threshold)

    def _hsv_range_mask(self, hsv, h_min, h_max, s_min, s_max, v_min, v_max):
        try:
            import cv2
            lower = np.array([int(h_min), int(s_min), int(v_min)], dtype=np.uint8)
            upper = np.array([int(h_max), int(s_max), int(v_max)], dtype=np.uint8)
            if h_min <= h_max:
                return cv2.inRange(hsv, lower, upper) > 0
            lower_a = np.array([0, int(s_min), int(v_min)], dtype=np.uint8)
            upper_a = np.array([int(h_max), int(s_max), int(v_max)], dtype=np.uint8)
            lower_b = np.array([int(h_min), int(s_min), int(v_min)], dtype=np.uint8)
            upper_b = np.array([179, int(s_max), int(v_max)], dtype=np.uint8)
            return (cv2.inRange(hsv, lower_a, upper_a) > 0) | (cv2.inRange(hsv, lower_b, upper_b) > 0)
        except Exception:
            h = hsv[:, :, 0].astype(np.float32)
            s = hsv[:, :, 1].astype(np.float32)
            v = hsv[:, :, 2].astype(np.float32)
            if h_min <= h_max:
                hue_mask = (h >= h_min) & (h <= h_max)
            else:
                hue_mask = (h <= h_max) | (h >= h_min)
            return hue_mask & (s >= s_min) & (s <= s_max) & (v >= v_min) & (v <= v_max)

    def _extract_line_region(self, mask):
        component = self._largest_component(mask)
        if component is not None:
            return component
        return self._extract_line_columns(mask)

    def _trace_black_path(self, mask, preferred_x=None):
        height, width = mask.shape[:2]
        selected = np.zeros_like(mask, dtype=bool)
        points = []
        prev_x = preferred_x if preferred_x is not None else width / 2.0
        max_jump = max(24.0, width * 0.28)
        min_run_width = max(6, int(width * 0.01))
        near_start = int(height * 0.82)
        bounded_rows = 0
        for y in range(height - 1, -1, -1):
            runs = self._row_runs(mask[y], min_run_width)
            if not runs:
                continue
            runs = self._filter_unbounded_runs(runs, width, allow_full_width=y >= near_start)
            if not runs:
                continue
            scored = []
            for start, end in runs:
                center = (start + end - 1) / 2.0
                unbounded = self._is_unbounded_run(start, end, width)
                scored.append((abs(center - prev_x), center, start, end, unbounded))
            distance, center, start, end, unbounded = min(scored, key=lambda item: item[0])
            if not points and distance > max_jump:
                seed = self._wide_initial_seed_run(scored, width)
                if seed is None:
                    continue
                distance, center, start, end, unbounded = seed
            if points and distance > max_jump:
                continue
            selected[y, start:end] = True
            points.append((center, y))
            if not unbounded:
                bounded_rows += 1
            prev_x = center
        min_path_rows = max(30, int(height * 0.25))
        if len(points) < min_path_rows or bounded_rows <= 0:
            return np.zeros_like(mask, dtype=bool), []
        return selected, points

    def _wide_initial_seed_run(self, scored_runs, width):
        min_seed_width = max(32, int(width * 0.12))
        candidates = []
        for distance, center, start, end, unbounded in scored_runs:
            run_width = end - start
            if unbounded or run_width < min_seed_width:
                continue
            candidates.append((run_width, -distance, distance, center, start, end, unbounded))
        if not candidates:
            return None
        _, _, distance, center, start, end, unbounded = max(candidates, key=lambda item: (item[0], item[1]))
        return distance, center, start, end, unbounded

    def _filter_unbounded_runs(self, runs, width, allow_full_width=False):
        filtered = []
        for start, end in runs:
            if self._is_unbounded_run(start, end, width) and not allow_full_width:
                continue
            filtered.append((start, end))
        return filtered

    def _is_unbounded_run(self, start, end, width):
        return end - start >= int(width * 0.88)

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

    def _target_center_from_channel(self, line_mask, path_points, forbidden_mask):
        height, width = line_mask.shape[:2]
        rows = [int(round(y)) for _, y in path_points]
        if not rows:
            rows = [int(y) for y in np.unique(np.nonzero(line_mask)[0])]
        lane_centers = []
        path_centers = []
        for y in rows:
            if y < 0 or y >= height:
                continue
            road_xs = np.nonzero(line_mask[y])[0]
            if road_xs.size == 0:
                continue
            road_min = int(road_xs.min())
            road_max = int(road_xs.max())
            road_center = (road_min + road_max) / 2.0
            path_centers.append((road_center, float(y), self._row_weight(y, height)))
            left_edge, right_edge = self._forbidden_channel_edges(forbidden_mask[y], road_center)
            if left_edge is None or right_edge is None:
                continue
            if right_edge - left_edge < max(36, int(width * 0.18)):
                continue
            lane_center = (left_edge + right_edge) / 2.0
            lane_centers.append((lane_center, float(y), self._row_weight(y, height)))
        if lane_centers:
            center_x, center_y = self._weighted_center(lane_centers)
            return center_x, center_y, self._curve_offset_from_samples(lane_centers, width)
        if path_centers:
            center_x, center_y = self._weighted_center(path_centers)
            return center_x, center_y, 0.0
        return None, None, 0.0

    def _curve_offset_from_samples(self, samples, width):
        if len(samples) < 8 or width <= 0:
            return 0.0
        ordered = sorted(samples, key=lambda item: item[1])
        take = max(3, len(ordered) // 4)
        far = ordered[:take]
        near = ordered[-take:]
        far_center = sum(x for x, _, _ in far) / float(len(far))
        near_center = sum(x for x, _, _ in near) / float(len(near))
        delta = far_center - near_center
        if abs(delta) < max(10.0, width * 0.035):
            return 0.0
        curve_offset = delta / (width / 2.0)
        return max(-0.18, min(0.18, curve_offset))

    def _forbidden_channel_edges(self, forbidden_row, road_center):
        runs = self._row_runs(forbidden_row, 2)
        left_edges = []
        right_edges = []
        for start, end in runs:
            center = (start + end - 1) / 2.0
            if center < road_center:
                left_edges.append(end - 1)
            elif center > road_center:
                right_edges.append(start)
        left_edge = max(left_edges) if left_edges else None
        right_edge = min(right_edges) if right_edges else None
        return left_edge, right_edge

    def _row_weight(self, y, height):
        if height <= 1:
            return 1.0
        near_field = float(y) / float(height - 1)
        return max(0.01, near_field ** 4)

    def _weighted_center(self, samples):
        total_weight = sum(weight for _, _, weight in samples)
        if total_weight <= 0:
            return None, None
        center_x = sum(x * weight for x, _, weight in samples) / total_weight
        center_y = sum(y * weight for _, y, weight in samples) / total_weight
        return center_x, center_y

    def _solid_forbidden_mask(self, roi):
        return self._solid_components(self._forbidden_color_mask(roi))

    def _forbidden_color_mask(self, roi):
        red = self._hsv_range_mask(roi, 170, 10, 60, 255, 80, 255)
        yellow = self._hsv_range_mask(roi, 15, 45, 55, 255, 60, 255)
        return red | yellow

    def _non_drivable_marking_mask(self, roi):
        s = roi[:, :, 1].astype(np.int16)
        v = roi[:, :, 2].astype(np.int16)
        red_yellow = self._forbidden_color_mask(roi)
        blue = self._hsv_range_mask(roi, 95, 130, 50, 255, 80, 255)
        white = (v > 205) & (s < 45)
        saturated = (s > 85) & (v > 110)
        return red_yellow | blue | white | saturated

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

    def _path_touches_forbidden(self, points, forbidden_mask, contact_band_start=None):
        if not np.any(forbidden_mask):
            return False
        height, width = forbidden_mask.shape[:2]
        if contact_band_start is None:
            contact_band_start = int(height * 0.94)
        contact_band_start = max(0, min(height - 1, int(contact_band_start)))
        contact_mask = np.zeros_like(forbidden_mask, dtype=bool)
        contact_mask[contact_band_start:, :] = forbidden_mask[contact_band_start:, :]
        if not np.any(contact_mask):
            return False
        if not points:
            return self._center_contact_touches_forbidden(contact_mask)
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

    def _center_contact_touches_forbidden(self, contact_mask):
        height, width = contact_mask.shape[:2]
        vehicle_half_width = max(18, int(width * 0.14))
        center = width // 2
        x0 = max(0, center - vehicle_half_width)
        x1 = min(width, center + vehicle_half_width + 1)
        return bool(np.any(contact_mask[:, x0:x1]))

    def _forbidden_avoidance_angular_z(self, forbidden_mask, contact_band_start=None):
        if not np.any(forbidden_mask):
            return None
        height, width = forbidden_mask.shape[:2]
        if contact_band_start is None:
            contact_band_start = int(height * 0.94)
        contact_band_start = max(0, min(height - 1, int(contact_band_start)))
        ys, xs = np.nonzero(forbidden_mask[contact_band_start:, :])
        if xs.size == 0:
            return None
        center = (width - 1) / 2.0
        contact_offset = float(xs.mean()) - center
        side_deadband = max(8.0, width * 0.04)
        if contact_offset > side_deadband:
            return abs(self.search_angular_speed)
        if contact_offset < -side_deadband:
            return -abs(self.search_angular_speed)
        angular_z = self._search_angular_z()
        if abs(angular_z) <= 0.01:
            return abs(self.search_angular_speed)
        return angular_z

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
