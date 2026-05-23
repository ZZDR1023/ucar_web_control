"""语音控制命令解析。

浏览器或其他语音识别前端只需要把识别文本发给后端；本模块负责把
自然语言短句转换成明确、可测试的机器人动作。
"""

import re
import math


STOP_WORDS = ("停止", "停下", "停车", "刹车", "急停", "别动", "不要动")

MOTION_COMMANDS = (
    (("前进", "向前", "往前", "走"), 0.10, 0.0, "前进"),
    (("后退", "倒退", "向后", "往后"), -0.08, 0.0, "后退"),
    (("左转", "向左", "往左"), 0.0, 0.45, "左转"),
    (("右转", "向右", "往右"), 0.0, -0.45, "右转"),
)

MOVE_SPEED = 0.10
BACK_SPEED = -0.08
TURN_SPEED = 0.45
DEFAULT_TURN_DEGREES = 90.0
MAX_STEP_SECONDS = 15.0
MAX_DISTANCE_METERS = 1.5
MAX_TURN_DEGREES = 180.0

CHINESE_NUMBERS = {
    "零": 0.0,
    "一": 1.0,
    "二": 2.0,
    "两": 2.0,
    "三": 3.0,
    "四": 4.0,
    "五": 5.0,
}

CHINESE_ORDINALS = {
    1: "一",
    2: "二",
    3: "三",
    4: "四",
    5: "五",
    6: "六",
    7: "七",
    8: "八",
    9: "九",
    10: "十",
}


def normalize_text(text):
    return re.sub(r"\s+", "", str(text or "").strip().lower())


def _base_response(text, action, ok=True, message=""):
    return {
        "ok": ok,
        "action": action,
        "text": text,
        "message": message,
    }


def _match_location(text, locations):
    if not locations:
        return None
    for point in locations:
        names = [point.get("name", "")]
        names.extend(point.get("aliases", []) or [])
        normalized_names = [normalize_text(name) for name in names if name]
        if any(name and name in text for name in normalized_names):
            return point
    return None


def _number_value(value, default=None):
    if value is None or value == "":
        return default
    if value in CHINESE_NUMBERS:
        return CHINESE_NUMBERS[value]
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _clamp(value, lower, upper):
    return max(lower, min(upper, value))


def build_route_point_aliases(route_name, zero_based_index, include_generic=False):
    route = normalize_text(route_name)
    if not route and not include_generic:
        return []
    number = int(zero_based_index) + 1
    tokens = [str(number)]
    if number in CHINESE_ORDINALS:
        tokens.append(CHINESE_ORDINALS[number])
    aliases = []
    for token in tokens:
        if route:
            aliases.extend([
                "{}中的{}号巡逻点".format(route, token),
                "{}的{}号巡逻点".format(route, token),
                "{}{}号巡逻点".format(route, token),
                "{}第{}个点".format(route, token),
                "{}第{}号点".format(route, token),
            ])
        if include_generic:
            aliases.extend([
                "{}号巡逻点".format(token),
                "{}号点".format(token),
                "第{}个点".format(token),
                "第{}号点".format(token),
            ])
    return aliases


def odom_target_for_step(step, move_scale=1.0, turn_scale=1.0):
    kind = step.get("kind")
    if kind == "move":
        distance = float(step.get("distance_m", 0.0) or 0.0)
        return {"distance_m": max(0.0, distance * max(0.1, float(move_scale)))}
    if kind == "turn":
        angle_deg = float(step.get("angle_deg", 0.0) or 0.0)
        return {"angle_rad": math.radians(max(0.0, angle_deg * max(0.1, float(turn_scale))))}
    return {}


def _move_step(direction, distance):
    distance = _clamp(float(distance), 0.05, MAX_DISTANCE_METERS)
    linear_x = MOVE_SPEED if direction == "forward" else BACK_SPEED
    duration_s = _clamp(distance / abs(linear_x), 0.3, MAX_STEP_SECONDS)
    label = "前进{:.2f}米".format(distance) if direction == "forward" else "后退{:.2f}米".format(distance)
    return {
        "kind": "move",
        "label": label,
        "linear_x": linear_x,
        "angular_z": 0.0,
        "distance_m": distance,
        "duration_s": duration_s,
    }


def _turn_step(direction, angle):
    angle = _clamp(float(angle), 10.0, MAX_TURN_DEGREES)
    angular_z = TURN_SPEED if direction == "left" else -TURN_SPEED
    duration_s = _clamp(math.radians(angle) / abs(angular_z), 0.3, MAX_STEP_SECONDS)
    label = "左转{:.0f}度".format(angle) if direction == "left" else "右转{:.0f}度".format(angle)
    return {
        "kind": "turn",
        "label": label,
        "linear_x": 0.0,
        "angular_z": angular_z,
        "angle_deg": angle,
        "duration_s": duration_s,
    }


def _parse_motion_step(part):
    number = r"(\d+(?:\.\d+)?|[零一二两三四五])?"
    if any(word in part for word in ("前进", "向前", "往前")):
        match = re.search(number + r"(?:米|m)", part)
        if match:
            return _move_step("forward", _number_value(match.group(1), 1.0))
        if "秒" in part:
            seconds = _number_value(re.search(number + r"秒", part).group(1), 1.0)
            step = _move_step("forward", MOVE_SPEED * seconds)
            step["duration_s"] = _clamp(seconds, 0.3, MAX_STEP_SECONDS)
            step["label"] = "前进{:.1f}秒".format(step["duration_s"])
            return step
    if any(word in part for word in ("后退", "倒退", "向后", "往后")):
        match = re.search(number + r"(?:米|m)", part)
        if match:
            return _move_step("back", _number_value(match.group(1), 0.5))
        if "秒" in part:
            seconds = _number_value(re.search(number + r"秒", part).group(1), 1.0)
            step = _move_step("back", abs(BACK_SPEED) * seconds)
            step["duration_s"] = _clamp(seconds, 0.3, MAX_STEP_SECONDS)
            step["label"] = "后退{:.1f}秒".format(step["duration_s"])
            return step
    if any(word in part for word in ("左转", "向左", "往左")):
        match = re.search(number + r"(?:度|°)", part)
        return _turn_step("left", _number_value(match.group(1), DEFAULT_TURN_DEGREES) if match else DEFAULT_TURN_DEGREES)
    if any(word in part for word in ("右转", "向右", "往右")):
        match = re.search(number + r"(?:度|°)", part)
        return _turn_step("right", _number_value(match.group(1), DEFAULT_TURN_DEGREES) if match else DEFAULT_TURN_DEGREES)
    return None


def _parse_sequence(text):
    has_sequence_word = any(word in text for word in ("再", "然后", "接着", "并且"))
    has_quantity = re.search(r"(\d+(?:\.\d+)?|[一二两三四五])(?:米|m|度|°|秒)", text) is not None
    if not has_sequence_word and not has_quantity:
        return None
    parts = [part for part in re.split(r"(?:然后|接着|并且|再|，|,)", text) if part]
    steps = []
    for part in parts:
        step = _parse_motion_step(part)
        if step:
            steps.append(step)
    if not steps:
        return None
    result = _base_response(text, "sequence", True, "执行动作序列：" + "，".join(step["label"] for step in steps))
    result["steps"] = steps
    result["duration_s"] = sum(step["duration_s"] for step in steps)
    return result


def parse_voice_command(text, locations=None):
    normalized = normalize_text(text)
    if not normalized:
        return _base_response(normalized, "unknown", False, "没有识别到语音内容")

    if "巡逻" in normalized:
        if any(word in normalized for word in ("开始", "启动", "执行")):
            return _base_response(normalized, "patrol_start", True, "开始巡逻")
        if any(word in normalized for word in ("暂停", "等一下", "等下")):
            return _base_response(normalized, "patrol_pause", True, "暂停巡逻")
        if any(word in normalized for word in ("继续", "恢复")):
            return _base_response(normalized, "patrol_resume", True, "恢复巡逻")
        if any(word in normalized for word in ("结束", "停止", "取消")):
            return _base_response(normalized, "patrol_stop", True, "停止巡逻")

    if any(word in normalized for word in ("跟着我", "跟随我", "人体跟随", "人员跟随")):
        if any(word in normalized for word in ("结束", "停止", "取消", "关闭")):
            return _base_response(normalized, "person_follow_stop", True, "停止人体跟随")
        return _base_response(normalized, "person_follow_start", True, "开始人体跟随")
    if "跟随" in normalized and any(word in normalized for word in ("结束", "停止", "取消", "关闭")):
        return _base_response(normalized, "person_follow_stop", True, "停止人体跟随")

    if any(word in normalized for word in STOP_WORDS):
        result = _base_response(normalized, "stop", True, "停止")
        result.update({"linear_x": 0.0, "angular_z": 0.0})
        return result

    sequence = _parse_sequence(normalized)
    if sequence is not None:
        return sequence

    if any(word in normalized for word in ("去", "到", "前往", "导航到")):
        point = _match_location(normalized, locations or [])
        if point is not None:
            result = _base_response(normalized, "goal", True, "导航到{}".format(point.get("name", "目标点")))
            result.update({
                "target_name": point.get("name", ""),
                "x": float(point.get("x", 0.0)),
                "y": float(point.get("y", 0.0)),
                "yaw": float(point.get("yaw", 0.0) or 0.0),
            })
            return result
        return _base_response(normalized, "unknown", False, "未找到对应导航点")

    for words, linear_x, angular_z, label in MOTION_COMMANDS:
        if any(word in normalized for word in words):
            result = _base_response(normalized, "cmd_vel", True, label)
            result.update({"linear_x": linear_x, "angular_z": angular_z})
            return result

    return _base_response(normalized, "unknown", False, "未识别的语音命令")
