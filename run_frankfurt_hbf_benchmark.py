# -*- coding: utf-8 -*-
"""
法兰克福中央车站 QEA-NS 与 CP-SAT 对比实验。
"""

from __future__ import annotations

import argparse
import csv
import importlib.util
import random
import sys
import time
from collections import deque
from pathlib import Path

import numpy as np


STATION_NAME = "法兰克福中央车站"
SCHEME_PLAN = "原计划复核"
SCHEME_DISTURBED = "扰动基准"
SCHEME_QEA = "QEA-NS"
SCHEME_CP_SAT = "CP-SAT"

MACRO_INBOUND_CONFLICT = "进站咽喉冲突"
MACRO_TRACK_CONFLICT = "股道占用冲突"
MACRO_OUTBOUND_CONFLICT = "出站咽喉冲突"
MACRO_CONFLICT_METRICS = [
    "进站咽喉冲突项",
    "股道占用冲突项",
    "出站咽喉冲突项",
]

LOCAL_DISTURBANCE_EVENTS_PER_SCENARIO = (2, 2)
LOCAL_SOURCE_TRAIN_COUNT_RANGE = (1, 3)
LOCAL_WEIBULL_DELAY_SCALE = 2.8
LOCAL_WEIBULL_DELAY_CAP = 4
LOCAL_TRACK_BLOCK_RANGE = (3, 5)
LOCAL_MARKOV_DEVICE_MAX_DURATION = 5
LOCAL_PROPAGATION_MAX_ROUNDS = 3
LOCAL_RECOVERY_LONG_WAIT_THRESHOLD = 10
DELAY_CHOICES = tuple(range(0, 21))
DEFAULT_INPUT_NAME = "frankfurt_hbf_gtfs_schedule.csv"
ACTIVE_TARGET_SIZE = 30
OPTIMIZE_ALL_TRAINS = True
MAX_OPTIONS_PER_TRAIN = len(DELAY_CHOICES) * 3
NEIGHBORHOOD_ACTIVE_START = 60
NEIGHBORHOOD_ACTIVE_STEP = 5
NEIGHBORHOOD_ACTIVE_CAP = 139
SUBPROBLEM_SEED = 20260622
QEA_DETERMINISTIC_GENERATION_CAP = 300
ENABLE_QEA_PROJECTION = True
QEA_PROJECTION_ACTIVE_CAP = 75
QEA_PROJECTION_MAX_RATIO = 0.75
QEA_PROJECTION_EDGE_FACTOR = 5
QEA_PROJECTION_TIME_WINDOW = 30
QEA_PROJECTION_DEVIATION_PENALTY = 8
CP_OBJECTIVE_WEIGHTS = {
    "conflict": 10000000,
    "delay": 1000,
    "max_delay": 10,
    "track_change": 1,
    "adjusted_train": 1,
}
OBJECTIVE_REFERENCE_ASSIGNMENT_IDS = None
OBJECTIVE_OPTIONS_BY_TRAIN = None
OBJECTIVE_REFERENCE_DELAY_BY_TRAIN = None
QEA_CANDIDATE_POOL = []
SOURCE_TRAIN_COUNT_OVERRIDE = None


EMBEDDED_STATION_RESOURCE_SOURCE = 'from __future__ import annotations\n\nimport csv\nimport re\nimport math\nimport random\nimport statistics\nimport time\nimport xml.etree.ElementTree as ET\nfrom collections import Counter, defaultdict\nfrom dataclasses import dataclass\nfrom html import escape\nfrom pathlib import Path\nfrom typing import Iterable\n\nfrom PIL import Image, ImageColor, ImageDraw, ImageFont\n\n\nENCODINGS = ("utf-8-sig", "utf-8", "gb18030", "gbk", "utf-16")\nBICUBIC_RESAMPLE = (\n    Image.Resampling.BICUBIC if hasattr(Image, "Resampling") else Image.BICUBIC\n)\nDEFAULT_INPUT_PATTERNS = (\n    "frankfurt_hbf_gtfs_schedule.csv",\n    "*schedule*.csv",\n    "*gtfs*.csv",\n)\nCANONICAL_TRACKS = tuple(str(index) for index in range(1, 22))\n\nFIELD_TRAIN_NO = "车次"\nFIELD_TRIP_ID = "trip_id"\nFIELD_DIRECTION = "方向"\nFIELD_PLANNED_TRACK = "计划股道"\nFIELD_PREV_STATION = "上一站"\nFIELD_NEXT_STATION = "下一站"\nFIELD_IN_THROAT = "接车侧咽喉"\nFIELD_OUT_THROAT = "发车侧咽喉"\nFIELD_ARRIVAL = "计划到达时刻"\nFIELD_DEPARTURE = "计划出发时刻"\nFIELD_DWELL = "停留时间(分钟)"\nFIELD_AC = "inbound_operation_duration_min"\nFIELD_C = "minimum_track_occupation_min"\nFIELD_CE = "outbound_operation_duration_min"\n\nTRACK_ZONE_ORDER = {\n    "西侧低位场": 0,\n    "中西到发场": 1,\n    "中央到发场": 2,\n    "中东到发场": 3,\n    "南侧到发场": 4,\n    "其他股道带": 5,\n}\n\nTHROAT_PARTITION_ORDER = {\n    "西咽喉": ("W1", "W2", "W3", "W4", "W5", "W6"),\n    "东咽喉": ("E1", "E2", "E3", "E4", "E5", "E6"),\n    "南咽喉": ("S1", "S2", "S3", "S4", "S5"),\n    "始发端": ("O1", "O2", "O3", "O4"),\n    "终到端": ("T1", "T2", "T3", "T4"),\n}\n\nSOURCE_PRIORITY = {\n    "原计划工程进路": 0,\n    "同路径历史进路": 1,\n    "同方向同咽喉进路": 2,\n    "同方向工程进路": 3,\n    "邻接股道工程推断": 4,\n    "恢复扩展咽喉组合": 5,\n    "恢复扩展进路变体": 6,\n    "原计划冲突股道保留": 0,\n    "时隙空余股道候选": 1,\n}\n\nDEFAULT_MODEL_WEIGHTS = {\n    "safety_weight": 1.35,\n    "stability_weight": 1.10,\n}\n\nFREE_SPACE_SEARCH_ENABLED = True\nFREE_SPACE_TRACK_LIMIT = 10\nFREE_SPACE_RICH_TRACK_LIMIT = 16\nFREE_SPACE_SEED_LIMIT = 3\nFREE_SPACE_RICH_SEED_LIMIT = 6\nFREE_SPACE_OPTION_LIMIT = 36\nFREE_SPACE_RICH_OPTION_LIMIT = 72\nFREE_SPACE_VARIANT_LIMIT = 2\nFREE_SPACE_RICH_VARIANT_LIMIT = 4\n\nDEFAULT_DISTURBANCE_WINDOW = (8 * 60, 9 * 60)\nINPUT_TIME_WINDOW = (8 * 60, 10 * 60)\nDISTURBANCE_TIME_MAX_COUNT = 2\nDISTURBANCE_SOURCE_TRAIN_RANGE = (1, 3)\nDISTURBANCE_SPATIAL_RESOURCE_RANGE = (1, 3)\nDISTURBANCE_TIME_CONTRIBUTION_THRESHOLD = 0.80\nDISTURBANCE_SOURCE_CONTRIBUTION_THRESHOLD = 0.75\nDISTURBANCE_RESOURCE_CONTRIBUTION_THRESHOLD = 0.75\nDISTURBANCE_TRAIN_LOOKAROUND_MIN = 3\nDISTURBANCE_LOAD_WEIGHTS = (0.5, 0.5)\nDISTURBANCE_TENSION_WEIGHTS = (0.3, 0.5, 0.2)\nDISTURBANCE_TRACK_BLOCK_RANGE = (3, 5)\nDISTURBANCE_SWITCH_SLOWDOWN_RANGE = (0.05, 0.15)\nDISTURBANCE_PROPAGATION_MAX_ROUNDS = 3\nDISTURBANCE_PROPAGATION_HORIZON_BUFFER = 10\nDISTURBANCE_MAX_PROPAGATED_DELAY = (\n    15  # 最大允许等待/恢复阈值；用于触发说明，不再截断真实等待。\n)\nDISTURBANCE_REARRANGEMENT_FRONTIER_LIMIT = 10\nDISTURBANCE_LOCAL_WAIT_CAP = 5\nDISTURBANCE_LOCAL_CONFLICT_WINDOW = 35\nDISTURBANCE_RISK_WEIGHTS = {\n    "train": 0.35,\n    "route": 0.25,\n    "switch": 0.25,\n    "throat": 0.15,\n}\nSHAPLEY_APPROX_SAMPLES = 96\nWEIBULL_DELAY_SHAPE = 1.65\nWEIBULL_DELAY_SCALE = 2.8\nWEIBULL_DELAY_CAP = 4\nMARKOV_DEVICE_MAX_DURATION = 5\nRECOVERY_LONG_WAIT_THRESHOLD = 10\nRECOVERY_AGGRESSIVE_MODE = True\nRECOVERY_DISABLE_STABILITY_COST = True\nRECOVERY_AGGRESSIVE_TRACK_LIMIT = 0\nRECOVERY_AGGRESSIVE_OPTION_LIMIT = 96\nRECOVERY_DELAY_ABSORPTION_THRESHOLD = 3\nRECOVERY_DELAY_ABSORPTION_PASSES = 4\nRECOVERY_AGGRESSIVE_OUTER_LOOPS = 6\nRECOVERY_AGGRESSIVE_REFINEMENT_PASSES = 2\nRESOURCE_WINDOW_CACHE_LIMIT = 500_000\nPAIR_CONFLICT_CACHE_LIMIT = 300_000\n\nRESOURCE_WINDOW_CACHE = {}\nPAIR_CONFLICT_DETAIL_CACHE = {}\nPAIR_CONFLICT_DETAIL_COARSE_CACHE = {}\n\nHARD_CONFLICT_PRIORITY = 1_000_000_000_000.0\nHARD_RISK_PRIORITY = 1_000_000.0\n\nCONFLICT_PENALTY = {\n    "股道占用冲突": (4200.0, 160.0),\n    "始发终到作业冲突": (3200.0, 120.0),\n    "接车进路锁闭冲突": (1500.0, 60.0),\n    "发车进路锁闭冲突": (1500.0, 60.0),\n    "道岔组冲突": (1950.0, 75.0),\n    "进路交叉冲突": (1750.0, 70.0),\n    "防护带冲突": (1100.0, 35.0),\n    "咽喉能力冲突": (650.0, 20.0),\n}\n\n\n@dataclass(frozen=True)\nclass TrainRecord:\n    index: int\n    record_id: str\n    train_no: str\n    trip_id: str\n    direction: str\n    planned_track: str\n    prev_station: str\n    next_station: str\n    in_throat: str\n    out_throat: str\n    arrival_text: str\n    departure_text: str\n    arrival_min: int\n    departure_min: int\n    dwell_min: int\n    ab_min: int\n    b_min: int\n    bc_min: int\n    c_min: int\n    cd_min: int\n    d_min: int\n    de_min: int\n    inbound_start: int\n    inbound_end: int\n    dwell_start: int\n    dwell_end: int\n    outbound_start: int\n    outbound_end: int\n\n    @property\n    def route_signature(self) -> tuple[str, str, str]:\n        return (\n            self.direction.strip(),\n            self.prev_station.strip(),\n            self.next_station.strip(),\n        )\n\n    @property\n    def exact_route_key(self) -> tuple[str, str, str, str, str, str]:\n        return (\n            self.direction.strip(),\n            self.prev_station.strip(),\n            self.next_station.strip(),\n            self.planned_track.strip(),\n            self.in_throat.strip(),\n            self.out_throat.strip(),\n        )\n\n    @property\n    def operation_type(self) -> str:\n        is_origin = (not self.prev_station.strip()) or self.in_throat == "始发端"\n        is_terminal = (not self.next_station.strip()) or self.out_throat == "终到端"\n        if is_origin and is_terminal:\n            return "始发终到"\n        if is_origin:\n            return "始发"\n        if is_terminal:\n            return "终到"\n        return "通过"\n\n    @property\n    def window_start(self) -> int:\n        return self.inbound_start\n\n    @property\n    def window_end(self) -> int:\n        return self.outbound_end\n\n\n@dataclass(frozen=True)\nclass ResourceWindow:\n    train_id: str\n    option_id: str\n    route_code: str\n    stage: str\n    resource_name: str\n    resource_category: str\n    movement_family: str\n    throat: str\n    partition: str\n    channel: str\n    zone: str\n    track: str\n    start_min: int\n    end_min: int\n\n\ndef shift_resource_window(resource: ResourceWindow, shift: int) -> ResourceWindow:\n    return ResourceWindow(\n        train_id=resource.train_id,\n        option_id=resource.option_id,\n        route_code=resource.route_code,\n        stage=resource.stage,\n        resource_name=resource.resource_name,\n        resource_category=resource.resource_category,\n        movement_family=resource.movement_family,\n        throat=resource.throat,\n        partition=resource.partition,\n        channel=resource.channel,\n        zone=resource.zone,\n        track=resource.track,\n        start_min=resource.start_min + shift,\n        end_min=resource.end_min + shift,\n    )\n\n\ndef retime_resource_window(\n    resource: ResourceWindow,\n    original_train: TrainRecord,\n    current_train: TrainRecord,\n) -> ResourceWindow:\n    arrival_shift = current_train.arrival_min - original_train.arrival_min\n    departure_shift = current_train.departure_min - original_train.departure_min\n    if resource.stage == "C":\n        start_min = current_train.dwell_start\n        end_min = current_train.dwell_end\n    elif resource.stage in {"C-D", "D", "D-E"}:\n        start_min = resource.start_min + departure_shift\n        end_min = resource.end_min + departure_shift\n    else:\n        start_min = resource.start_min + arrival_shift\n        end_min = resource.end_min + arrival_shift\n    return ResourceWindow(\n        train_id=resource.train_id,\n        option_id=resource.option_id,\n        route_code=resource.route_code,\n        stage=resource.stage,\n        resource_name=resource.resource_name,\n        resource_category=resource.resource_category,\n        movement_family=resource.movement_family,\n        throat=resource.throat,\n        partition=resource.partition,\n        channel=resource.channel,\n        zone=resource.zone,\n        track=resource.track,\n        start_min=start_min,\n        end_min=end_min,\n    )\n\n\n@dataclass(frozen=True)\nclass CandidateSeed:\n    track: str\n    in_throat: str\n    out_throat: str\n    source_level: str\n    support_count: int\n    note: str\n\n\n@dataclass(frozen=True)\nclass CandidateRoutePlan:\n    option_id: str\n    train_id: str\n    track: str\n    in_throat: str\n    out_throat: str\n    in_partition: str\n    out_partition: str\n    inbound_channel: str\n    outbound_channel: str\n    route_variant: str\n    route_code: str\n    inbound_route_code: str\n    outbound_route_code: str\n    route_family: str\n    source_level: str\n    support_count: int\n    route_score: float\n    candidate_rank: int\n    operation_type: str\n    delay_risk_cost: float\n    stability_cost: float\n    balance_reward: float\n    linear_cost: float\n    note: str\n    resources: tuple[ResourceWindow, ...]\n\n\n@dataclass(frozen=True)\nclass PairConflictDetail:\n    conflict_level: str\n    conflict_type: str\n    resource_name: str\n    overlap_min: int\n    description: str\n\n\n@dataclass(frozen=True)\nclass ConflictEntry:\n    scheme: str\n    conflict_level: str\n    conflict_type: str\n    train1: str\n    train2: str\n    route1: str\n    route2: str\n    resource: str\n    overlap_min: int\n    interval1: str\n    interval2: str\n    track1: str\n    track2: str\n    description: str\n\n\n@dataclass\n\n\n@dataclass(frozen=True)\nclass DisturbanceTarget:\n    resource_type: str\n    resource_key: str\n    display_name: str\n    throat: str\n    partition: str\n    tension_score: float\n    intensity_value: float\n    intensity_unit: str\n    note: str\n    duration_min: int = 0\n\n\n@dataclass(frozen=True)\nclass DisturbanceEvent:\n    disturbance_minute: int\n    selected_train_load: int\n    selected_switch_load: int\n    selected_load_score: float\n    source_train_ids: tuple[str, ...]\n    source_train_delays: dict[str, int]\n    source_train_scores: dict[str, float]\n    spatial_targets: tuple[DisturbanceTarget, ...]\n\n\n@dataclass(frozen=True)\nclass DisturbanceScenario:\n    window_start_min: int\n    window_end_min: int\n    disturbance_minute: int\n    selected_train_load: int\n    selected_switch_load: int\n    selected_load_score: float\n    source_train_ids: tuple[str, ...]\n    source_train_delays: dict[str, int]\n    source_train_scores: dict[str, float]\n    spatial_targets: tuple[DisturbanceTarget, ...]\n    track_scores: dict[str, float]\n    partition_scores: dict[str, float]\n    propagation_rounds: int\n    disturbance_events: tuple[DisturbanceEvent, ...] = tuple()\n\n\ndef train_timing_cache_key(train: TrainRecord) -> tuple[object, ...]:\n    return (\n        train.record_id,\n        train.arrival_min,\n        train.departure_min,\n        train.ab_min,\n        train.b_min,\n        train.bc_min,\n        train.c_min,\n        train.cd_min,\n        train.d_min,\n        train.de_min,\n        train.inbound_start,\n        train.inbound_end,\n        train.dwell_start,\n        train.dwell_end,\n        train.outbound_start,\n        train.outbound_end,\n    )\n\n\ndef resource_signature(resource: ResourceWindow) -> tuple[object, ...]:\n    return (\n        resource.stage,\n        resource.resource_name,\n        resource.resource_category,\n        resource.movement_family,\n        resource.throat,\n        resource.partition,\n        resource.channel,\n        resource.zone,\n        resource.track,\n        resource.start_min,\n        resource.end_min,\n    )\n\n\ndef option_resource_signature(\n    option: CandidateRoutePlan,\n) -> tuple[tuple[object, ...], ...]:\n    return tuple(resource_signature(resource) for resource in option.resources)\n\n\ndef candidate_equivalence_signature(option: CandidateRoutePlan) -> tuple[object, ...]:\n    return (\n        option.track,\n        option.in_throat,\n        option.out_throat,\n        option.in_partition,\n        option.out_partition,\n        option.inbound_channel,\n        option.outbound_channel,\n        option.route_variant,\n        option_resource_signature(option),\n    )\n\n\ndef candidate_quality_key(option: CandidateRoutePlan) -> tuple[object, ...]:\n    return (\n        option.linear_cost,\n        option.delay_risk_cost,\n        option.stability_cost,\n        -option.balance_reward,\n        SOURCE_PRIORITY.get(option.source_level, 99),\n        -option.support_count,\n        option.option_id,\n    )\n\n\ndef deduplicate_equivalent_candidate_plans(\n    candidate_plans: list[CandidateRoutePlan],\n) -> list[CandidateRoutePlan]:\n    best_by_signature: dict[tuple[object, ...], CandidateRoutePlan] = {}\n    order: list[tuple[object, ...]] = []\n    for option in candidate_plans:\n        signature = candidate_equivalence_signature(option)\n        old_option = best_by_signature.get(signature)\n        if old_option is None:\n            best_by_signature[signature] = option\n            order.append(signature)\n            continue\n        if candidate_quality_key(option) < candidate_quality_key(old_option):\n            best_by_signature[signature] = option\n    return [best_by_signature[signature] for signature in order]\n\n\ndef option_pair_cache_key(\n    option_a: CandidateRoutePlan,\n    option_b: CandidateRoutePlan,\n) -> tuple[object, ...]:\n    return (option_resource_signature(option_a), option_resource_signature(option_b))\n\n\n\n\ndef detect_encoding(path: Path) -> str:\n    for encoding in ENCODINGS:\n        try:\n            with path.open("r", encoding=encoding, newline="") as handle:\n                next(csv.reader(handle), None)\n            return encoding\n        except UnicodeDecodeError:\n            continue\n    raise ValueError(f"无法识别文件编码：{path}")\n\n\ndef find_input_file(base_dir: Path, explicit: str) -> Path:\n    if explicit:\n        path = Path(explicit).expanduser()\n        if not path.is_absolute():\n            path = (base_dir / path).resolve()\n        if not path.exists():\n            raise FileNotFoundError(f"未找到输入文件：{path}")\n        return path\n\n    for pattern in DEFAULT_INPUT_PATTERNS:\n        matched = sorted(base_dir.glob(pattern))\n        if matched:\n            return matched[0]\n\n    csv_files = sorted(\n        base_dir.glob("*.csv"), key=lambda item: item.stat().st_size, reverse=True\n    )\n    if csv_files:\n        return csv_files[0]\n    raise FileNotFoundError(f"目录 {base_dir} 下没有找到 CSV 文件。")\n\n\ndef parse_minutes(value: str) -> int:\n    text = (value or "").strip()\n    if not text:\n        return 0\n    return int(round(float(text)))\n\n\ndef parse_clock_to_minutes(value: str) -> int:\n    text = (value or "").strip()\n    if not text:\n        return 0\n    hour, minute, second = (int(part) for part in text.split(":"))\n    return hour * 60 + minute + (1 if second else 0)\n\n\ndef format_minutes_as_clock(total_minutes: int) -> str:\n    minutes = total_minutes % (24 * 60)\n    hour = minutes // 60\n    minute = minutes % 60\n    return f"{hour:02d}:{minute:02d}"\n\n\ndef format_interval(start: int, end: int) -> str:\n    return f"{format_minutes_as_clock(start)}-{format_minutes_as_clock(end)}"\n\n\ndef overlap_minutes(start_a: int, end_a: int, start_b: int, end_b: int) -> int:\n    return max(0, min(end_a, end_b) - max(start_a, start_b))\n\n\ndef safe_ratio(numerator: float, denominator: float) -> float:\n    return numerator / denominator if denominator else 0.0\n\n\ndef track_sort_key(track: str) -> tuple[int, str]:\n    text = (track or "").strip()\n    return (int(text) if text.isdigit() else 10**9, text)\n\n\ndef track_distance(track_a: str, track_b: str) -> int:\n    if track_a.isdigit() and track_b.isdigit():\n        return abs(int(track_a) - int(track_b))\n    return 0 if track_a == track_b else 99\n\n\ndef classify_track_zone(track: str) -> str:\n    if not track.isdigit():\n        return "其他股道带"\n    number = int(track)\n    if number <= 5:\n        return "西侧低位场"\n    if number <= 10:\n        return "中西到发场"\n    if number <= 15:\n        return "中央到发场"\n    if number <= 18:\n        return "中东到发场"\n    return "南侧到发场"\n\n\ndef zone_distance(zone_a: str, zone_b: str) -> int:\n    return abs(TRACK_ZONE_ORDER.get(zone_a, 99) - TRACK_ZONE_ORDER.get(zone_b, 99))\n\n\ndef throat_zone_conflicts(throat: str, zone_a: str, zone_b: str) -> bool:\n    if not zone_a or not zone_b:\n        return False\n    gap = zone_distance(zone_a, zone_b)\n    if throat in {"始发端", "终到端"}:\n        return gap == 0\n    if throat in {"西咽喉", "东咽喉"}:\n        return gap <= 1\n    if throat == "南咽喉":\n        if "南侧到发场" in {zone_a, zone_b}:\n            return gap <= 2\n        return gap <= 1\n    return gap == 0\n\n\ndef flank_zone_label(zone: str) -> str:\n    mapping = {\n        "西侧低位场": "西低位防护带",\n        "中西到发场": "中西防护带",\n        "中央到发场": "中央防护带",\n        "中东到发场": "中东防护带",\n        "南侧到发场": "南场防护带",\n        "其他股道带": "其他防护带",\n    }\n    return mapping.get(zone, "其他防护带")\n\n\ndef direction_code(direction: str) -> str:\n    return {"上行": "UP", "下行": "DN"}.get(direction, "UK")\n\n\ndef throat_code(throat: str) -> str:\n    return {\n        "西咽喉": "W",\n        "东咽喉": "E",\n        "南咽喉": "S",\n        "始发端": "O",\n        "终到端": "T",\n    }.get(throat, "U")\n\n\ndef zone_code(zone: str) -> str:\n    return {\n        "西侧低位场": "Z0",\n        "中西到发场": "Z1",\n        "中央到发场": "Z2",\n        "中东到发场": "Z3",\n        "南侧到发场": "Z4",\n        "其他股道带": "ZU",\n    }.get(zone, "ZU")\n\n\ndef track_adjacency_score(track_a: str, track_b: str) -> int:\n    if track_a == track_b:\n        return 0\n    if not (track_a.isdigit() and track_b.isdigit()):\n        return 99\n    distance = abs(int(track_a) - int(track_b))\n    zone_gap = zone_distance(classify_track_zone(track_a), classify_track_zone(track_b))\n    if zone_gap == 0:\n        if distance == 1:\n            return 1\n        if distance == 2:\n            return 2\n        if distance <= 4:\n            return 3\n        return 5\n    if zone_gap == 1:\n        if distance <= 2:\n            return 3\n        if distance <= 5:\n            return 4\n        return 6\n    if zone_gap == 2:\n        return 6 + min(distance, 4)\n    return 99\n\n\ndef throat_partition(track: str, throat: str) -> str:\n    """\n    咽喉分区映射函数\n\n    通用站场咽喉分区划分说明：\n    - 西咽喉：分为6个咽喉区段（W1-W6），对应不同的进路分组\n    - 东咽喉：分为6个咽喉区段（E1-E6）\n    - 南咽喉：分为5个咽喉区段（S1-S5），为辅助咽喉\n    - 始发端/终到端：分为4个区段\n    - 分区划分基于咽喉进路的空间拓扑关系，相邻股道通常共享咽喉区段资源\n    """\n    if not track.isdigit():\n        return ""\n    number = int(track)\n    if throat == "西咽喉":\n        if number <= 3:\n            return "W1"\n        if number <= 6:\n            return "W2"\n        if number <= 10:\n            return "W3"\n        if number <= 14:\n            return "W4"\n        if number <= 18:\n            return "W5"\n        return "W6"\n    if throat == "东咽喉":\n        if number <= 4:\n            return "E1"\n        if number <= 8:\n            return "E2"\n        if number <= 12:\n            return "E3"\n        if number <= 16:\n            return "E4"\n        if number <= 19:\n            return "E5"\n        return "E6"\n    if throat == "南咽喉":\n        if number <= 2:\n            return "S1"\n        if number <= 6:\n            return "S2"\n        if number <= 12:\n            return "S3"\n        if number <= 18:\n            return "S4"\n        return "S5"\n    if throat == "始发端":\n        if number <= 7:\n            return "O1"\n        if number <= 12:\n            return "O2"\n        if number <= 16:\n            return "O3"\n        return "O4"\n    if throat == "终到端":\n        if number <= 7:\n            return "T1"\n        if number <= 12:\n            return "T2"\n        if number <= 16:\n            return "T3"\n        return "T4"\n    return ""\n\n\ndef throat_partition_distance(throat: str, track_a: str, track_b: str) -> int:\n    part_a = throat_partition(track_a, throat)\n    part_b = throat_partition(track_b, throat)\n    if not part_a or not part_b:\n        return 99\n    order = THROAT_PARTITION_ORDER.get(throat, ())\n    if not order:\n        return 99\n    return abs(order.index(part_a) - order.index(part_b))\n\n\ndef throat_partition_relation(\n    throat: str, track_a: str, track_b: str\n) -> tuple[str, str, str] | None:\n    """\n    判断两股道在指定咽喉的资源竞争关系\n\n    参数:\n        throat: 咽喉名称\n        track_a, track_b: 股道编号\n\n    返回:\n        (分区A, 分区B, 关系级别) 或 None\n        关系级别:\n        - hard: 同一咽喉分区（不可同时使用）\n        - soft: 相邻咽喉分区（空间接近，存在潜在竞争）\n    """\n    part_a = throat_partition(track_a, throat)\n    part_b = throat_partition(track_b, throat)\n    partition_gap = throat_partition_distance(throat, track_a, track_b)\n    adjacency = track_adjacency_score(track_a, track_b)\n    if partition_gap == 99:\n        return None\n    if partition_gap == 0:\n        return (part_a, part_b, "hard")\n    if throat in {"始发端", "终到端"}:\n        if adjacency <= 1:\n            return (part_a, part_b, "soft")\n        return None\n    if throat in {"西咽喉", "东咽喉"}:\n        if partition_gap == 1 and adjacency <= 3:\n            return (part_a, part_b, "soft")\n        return None\n    if throat == "南咽喉":\n        if partition_gap == 1 and adjacency <= 3:\n            return (part_a, part_b, "soft")\n        if partition_gap == 2 and adjacency <= 1:\n            return (part_a, part_b, "soft")\n        return None\n    return None\n\n\ndef throat_partition_conflicts(throat: str, track_a: str, track_b: str) -> bool:\n    return throat_partition_relation(throat, track_a, track_b) is not None\n\n\ndef channel_variants(\n    track: str, throat: str, movement: str, operation_type: str\n) -> list[str]:\n    partition = throat_partition(track, throat)\n    if not partition:\n        return [""]\n    variants = [f"{partition}-P"]\n    if throat in {"西咽喉", "东咽喉"}:\n        variants.append(f"{partition}-R")\n    elif throat == "南咽喉":\n        variants.append(f"{partition}-S")\n    elif throat in {"始发端", "终到端"} and operation_type in {\n        "始发",\n        "终到",\n        "始发终到",\n    }:\n        variants.append(f"{partition}-Y")\n    return variants\n\n\ndef topology_note_for_track(track: str) -> str:\n    if not track.isdigit():\n        return "未知拓扑"\n    partition_notes = [\n        f"西咽喉={throat_partition(track, \'西咽喉\')}",\n        f"东咽喉={throat_partition(track, \'东咽喉\')}",\n        f"南咽喉={throat_partition(track, \'南咽喉\')}",\n    ]\n    return "；".join(partition_notes)\n\n\ndef load_trains(csv_path: Path, encoding: str) -> list[TrainRecord]:\n    with csv_path.open("r", encoding=encoding, newline="") as handle:\n        reader = csv.DictReader(handle)\n        rows = list(reader)\n\n    required_fields = {\n        FIELD_TRAIN_NO,\n        FIELD_TRIP_ID,\n        FIELD_DIRECTION,\n        FIELD_PLANNED_TRACK,\n        FIELD_PREV_STATION,\n        FIELD_NEXT_STATION,\n        FIELD_IN_THROAT,\n        FIELD_OUT_THROAT,\n        FIELD_ARRIVAL,\n        FIELD_DEPARTURE,\n        FIELD_DWELL,\n        FIELD_AC,\n        FIELD_C,\n        FIELD_CE,\n    }\n    missing_fields = sorted(required_fields.difference(reader.fieldnames or []))\n    if missing_fields:\n        raise KeyError(f"输入文件缺少字段：{\', \'.join(missing_fields)}")\n\n    trains: list[TrainRecord] = []\n    input_start, input_end = INPUT_TIME_WINDOW\n    for row in rows:\n        arrival_min = parse_clock_to_minutes(row[FIELD_ARRIVAL])\n        departure_min = parse_clock_to_minutes(row[FIELD_DEPARTURE])\n        dwell_min = parse_minutes(row[FIELD_DWELL])\n        c_min = parse_minutes(row[FIELD_C])\n        dwell_end = max(departure_min, arrival_min + max(dwell_min, c_min))\n        ac_min = parse_minutes(row[FIELD_AC])\n        ce_min = parse_minutes(row[FIELD_CE])\n        # 把 A-C 拆分为 A-B / B / B-C 三段，按固定比例（0.45 / 0.22 / 余数），保证三段之和 = ac_min\n        ab_min = int(round(ac_min * 0.45))\n        b_min = int(round(ac_min * 0.22))\n        bc_min = max(0, ac_min - ab_min - b_min)\n        # 把 C-E 拆分为 C-D / D / D-E 三段，按固定比例（0.30 / 0.30 / 余数），保证三段之和 = ce_min\n        cd_min = int(round(ce_min * 0.30))\n        d_min = int(round(ce_min * 0.30))\n        de_min = max(0, ce_min - cd_min - d_min)\n        inbound_start = arrival_min - ab_min - b_min - bc_min\n        outbound_end = departure_min + cd_min + d_min + de_min\n        if overlap_minutes(inbound_start, outbound_end, input_start, input_end) <= 0:\n            continue\n\n        index = len(trains) + 1\n\n        trains.append(\n            TrainRecord(\n                index=index,\n                record_id=f"R{index:03d}",\n                train_no=(row[FIELD_TRAIN_NO] or "").strip(),\n                trip_id=(row[FIELD_TRIP_ID] or "").strip(),\n                direction=(row[FIELD_DIRECTION] or "").strip(),\n                planned_track=(row[FIELD_PLANNED_TRACK] or "").strip(),\n                prev_station=(row[FIELD_PREV_STATION] or "").strip(),\n                next_station=(row[FIELD_NEXT_STATION] or "").strip(),\n                in_throat=(row[FIELD_IN_THROAT] or "").strip(),\n                out_throat=(row[FIELD_OUT_THROAT] or "").strip(),\n                arrival_text=(row[FIELD_ARRIVAL] or "").strip(),\n                departure_text=(row[FIELD_DEPARTURE] or "").strip(),\n                arrival_min=arrival_min,\n                departure_min=departure_min,\n                dwell_min=dwell_min,\n                ab_min=ab_min,\n                b_min=b_min,\n                bc_min=bc_min,\n                c_min=c_min,\n                cd_min=cd_min,\n                d_min=d_min,\n                de_min=de_min,\n                inbound_start=inbound_start,\n                inbound_end=arrival_min,\n                dwell_start=arrival_min,\n                dwell_end=dwell_end,\n                outbound_start=departure_min,\n                outbound_end=outbound_end,\n            )\n        )\n\n    if not trains:\n        raise ValueError("输入数据在 08:00-10:00 时间窗内没有可用列车。")\n\n    return trains\n\n\ndef rebuild_train_record(\n    train: TrainRecord,\n    arrival_shift: int = 0,\n    departure_shift: int = 0,\n    ab_delta: int = 0,\n    b_delta: int = 0,\n    bc_delta: int = 0,\n    cd_delta: int = 0,\n    d_delta: int = 0,\n    de_delta: int = 0,\n) -> TrainRecord:\n    arrival_min = max(0, train.arrival_min + arrival_shift)\n    departure_min = max(arrival_min, train.departure_min + departure_shift)\n    ab_min = max(0, train.ab_min + ab_delta)\n    b_min = max(0, train.b_min + b_delta)\n    bc_min = max(0, train.bc_min + bc_delta)\n    cd_min = max(0, train.cd_min + cd_delta)\n    d_min = max(0, train.d_min + d_delta)\n    de_min = max(0, train.de_min + de_delta)\n    dwell_end = max(departure_min, arrival_min + max(train.dwell_min, train.c_min))\n    return TrainRecord(\n        index=train.index,\n        record_id=train.record_id,\n        train_no=train.train_no,\n        trip_id=train.trip_id,\n        direction=train.direction,\n        planned_track=train.planned_track,\n        prev_station=train.prev_station,\n        next_station=train.next_station,\n        in_throat=train.in_throat,\n        out_throat=train.out_throat,\n        arrival_text=f"{format_minutes_as_clock(arrival_min)}:00",\n        departure_text=f"{format_minutes_as_clock(departure_min)}:00",\n        arrival_min=arrival_min,\n        departure_min=departure_min,\n        dwell_min=train.dwell_min,\n        ab_min=ab_min,\n        b_min=b_min,\n        bc_min=bc_min,\n        c_min=train.c_min,\n        cd_min=cd_min,\n        d_min=d_min,\n        de_min=de_min,\n        inbound_start=arrival_min - ab_min - b_min - bc_min,\n        inbound_end=arrival_min,\n        dwell_start=arrival_min,\n        dwell_end=dwell_end,\n        outbound_start=departure_min,\n        outbound_end=departure_min + cd_min + d_min + de_min,\n    )\n\n\ndef busiest_hour_window(trains: list[TrainRecord]) -> tuple[int, int]:\n    start_min = min(train.arrival_min for train in trains)\n    end_min = max(train.departure_min for train in trains)\n    best_start = start_min\n    best_count = -1\n    for minute in range(start_min, end_min + 1):\n        count = sum(\n            1\n            for train in trains\n            if minute <= train.arrival_min < minute + 60\n            or minute <= train.departure_min < minute + 60\n        )\n        if count > best_count:\n            best_count = count\n            best_start = minute\n    return best_start, best_start + 60\n\n\ndef normalize_score_map(values: dict[str, float]) -> dict[str, float]:\n    if not values:\n        return {}\n    minimum = min(values.values())\n    maximum = max(values.values())\n    if math.isclose(maximum, minimum, rel_tol=1e-9, abs_tol=1e-9):\n        return {key: (1.0 if value > 0 else 0.0) for key, value in values.items()}\n    return {\n        key: safe_ratio(value - minimum, maximum - minimum)\n        for key, value in values.items()\n    }\n\n\ndef weighted_sample_without_replacement(\n    rng: random.Random,\n    items: list[object],\n    sample_size: int,\n    weight_func,\n) -> list[object]:\n    pool = list(items)\n    selected: list[object] = []\n    while pool and len(selected) < sample_size:\n        weights = [max(0.0, float(weight_func(item))) for item in pool]\n        total_weight = sum(weights)\n        if total_weight <= 1e-9:\n            index = rng.randrange(len(pool))\n        else:\n            threshold = rng.random() * total_weight\n            cumulative = 0.0\n            index = len(pool) - 1\n            for idx, weight in enumerate(weights):\n                cumulative += weight\n                if cumulative >= threshold:\n                    index = idx\n                    break\n        selected.append(pool.pop(index))\n    return selected\n\n\ndef contribution_limited_count(\n    scores: list[float],\n    threshold: float,\n    max_count: int,\n    min_count: int = 1,\n) -> int:\n    if not scores:\n        return 0\n    ordered_scores = sorted((max(0.0, float(score)) for score in scores), reverse=True)\n    positive_total = sum(ordered_scores)\n    if positive_total <= 1e-12:\n        return max(1, min(min_count, max_count, len(ordered_scores)))\n    cumulative = 0.0\n    selected_count = len(ordered_scores)\n    for index, score in enumerate(ordered_scores, start=1):\n        cumulative += score\n        if cumulative / positive_total >= threshold:\n            selected_count = index\n            break\n    return min(max(selected_count, min_count), max_count, len(ordered_scores))\n\n\ndef build_planned_resource_windows_by_train(\n    trains: list[TrainRecord],\n) -> dict[str, tuple[ResourceWindow, ...]]:\n    return {\n        train.record_id: build_resource_windows(\n            train=train,\n            option_id=f"{train.record_id}:PLAN",\n            route_code="PLAN",\n            track=train.planned_track,\n            in_throat=train.in_throat,\n            out_throat=train.out_throat,\n            inbound_channel="PLAN-IN",\n            outbound_channel="PLAN-OUT",\n        )\n        for train in trains\n    }\n\n\ndef disturbance_analysis_window(trains: list[TrainRecord]) -> tuple[int, int]:\n    default_start, default_end = DEFAULT_DISTURBANCE_WINDOW\n    if any(\n        overlap_minutes(\n            train.window_start, train.window_end, default_start, default_end\n        )\n        > 0\n        for train in trains\n    ):\n        return default_start, default_end\n    return busiest_hour_window(trains)\n\n\ndef clip_resource_window(\n    resource: ResourceWindow, start_min: int, end_min: int\n) -> ResourceWindow:\n    return ResourceWindow(\n        train_id=resource.train_id,\n        option_id=resource.option_id,\n        route_code=resource.route_code,\n        stage=resource.stage,\n        resource_name=resource.resource_name,\n        resource_category=resource.resource_category,\n        movement_family=resource.movement_family,\n        throat=resource.throat,\n        partition=resource.partition,\n        channel=resource.channel,\n        zone=resource.zone,\n        track=resource.track,\n        start_min=max(resource.start_min, start_min),\n        end_min=min(resource.end_min, end_min),\n    )\n\n\ndef build_disturbance_load_profile(\n    trains: list[TrainRecord],\n    resources_by_train: dict[str, tuple[ResourceWindow, ...]],\n    window_start: int,\n    window_end: int,\n) -> list[dict[str, float]]:\n    route_resources = [\n        resource\n        for resources in resources_by_train.values()\n        for resource in resources\n        if resource.resource_category in {"route_lock", "flank_protection"}\n    ]\n    switch_resources = [\n        resource\n        for resources in resources_by_train.values()\n        for resource in resources\n        if resource.resource_category == "switch_ladder"\n    ]\n    throat_resources = [\n        resource\n        for resources in resources_by_train.values()\n        for resource in resources\n        if resource.resource_category == "throat_capacity"\n    ]\n    rows: list[dict[str, float]] = []\n    train_counts: list[int] = []\n    route_counts: list[int] = []\n    switch_counts: list[int] = []\n    throat_counts: list[int] = []\n    for minute in range(window_start, window_end):\n        train_count = sum(\n            1\n            for train in trains\n            if overlap_minutes(\n                train.inbound_start, train.inbound_end, minute, minute + 1\n            )\n            > 0\n            or overlap_minutes(\n                train.outbound_start, train.outbound_end, minute, minute + 1\n            )\n            > 0\n        )\n        switch_count = sum(\n            1\n            for resource in switch_resources\n            if overlap_minutes(resource.start_min, resource.end_min, minute, minute + 1)\n            > 0\n        )\n        route_count = sum(\n            1\n            for resource in route_resources\n            if overlap_minutes(resource.start_min, resource.end_min, minute, minute + 1)\n            > 0\n        )\n        throat_count = sum(\n            1\n            for resource in throat_resources\n            if overlap_minutes(resource.start_min, resource.end_min, minute, minute + 1)\n            > 0\n        )\n        train_counts.append(train_count)\n        route_counts.append(route_count)\n        switch_counts.append(switch_count)\n        throat_counts.append(throat_count)\n        rows.append(\n            {\n                "分钟": float(minute),\n                "列车作业数": float(train_count),\n                "进路锁闭作业数": float(route_count),\n                "道岔组作业数": float(switch_count),\n                "咽喉能力作业数": float(throat_count),\n            }\n        )\n\n    max_train_count = max(train_counts, default=0)\n    max_route_count = max(route_counts, default=0)\n    max_switch_count = max(switch_counts, default=0)\n    max_throat_count = max(throat_counts, default=0)\n    raw_intensities: list[float] = []\n    for row in rows:\n        train_norm = safe_ratio(row["列车作业数"], max_train_count)\n        route_norm = safe_ratio(row["进路锁闭作业数"], max_route_count)\n        switch_norm = safe_ratio(row["道岔组作业数"], max_switch_count)\n        throat_norm = safe_ratio(row["咽喉能力作业数"], max_throat_count)\n        raw_intensity = (\n            DISTURBANCE_RISK_WEIGHTS["train"] * train_norm\n            + DISTURBANCE_RISK_WEIGHTS["route"] * route_norm\n            + DISTURBANCE_RISK_WEIGHTS["switch"] * switch_norm\n            + DISTURBANCE_RISK_WEIGHTS["throat"] * throat_norm\n        )\n        row["列车负荷归一值"] = train_norm\n        row["进路负荷归一值"] = route_norm\n        row["道岔组负荷归一值"] = switch_norm\n        row["咽喉负荷归一值"] = throat_norm\n        row["时变负荷强度"] = raw_intensity\n        raw_intensities.append(raw_intensity)\n    max_intensity = max(raw_intensities, default=0.0)\n    intensity_sum = sum(raw_intensities)\n    for row in rows:\n        normalized_intensity = safe_ratio(row["时变负荷强度"], max_intensity)\n        row["负荷指数"] = normalized_intensity\n        row["NHPP扰动概率"] = safe_ratio(row["时变负荷强度"], intensity_sum)\n    return rows\n\n\ndef count_overlapping_window_pairs(windows: list[ResourceWindow]) -> float:\n    if len(windows) < 2:\n        return 0.0\n    ordered = sorted(\n        windows,\n        key=lambda resource: (resource.start_min, resource.end_min, resource.train_id),\n    )\n    count = 0\n    for left_index, left in enumerate(ordered):\n        for right in ordered[left_index + 1 :]:\n            if right.start_min >= left.end_min:\n                break\n            if (\n                overlap_minutes(\n                    left.start_min, left.end_min, right.start_min, right.end_min\n                )\n                > 0\n            ):\n                count += 1\n    return float(count)\n\n\ndef entropy_weight_topsis_scores(\n    item_metrics: dict[str, dict[str, float]],\n    metric_names: list[str],\n) -> tuple[dict[str, float], dict[str, float]]:\n    if not item_metrics:\n        return {}, {\n            metric: safe_ratio(1.0, len(metric_names)) for metric in metric_names\n        }\n    item_keys = list(item_metrics)\n    n_items = len(item_keys)\n    if n_items == 1:\n        return {item_keys[0]: 1.0}, {\n            metric: safe_ratio(1.0, len(metric_names)) for metric in metric_names\n        }\n\n    entropy_values: dict[str, float] = {}\n    for metric in metric_names:\n        column = [\n            max(0.0, float(item_metrics[item].get(metric, 0.0))) for item in item_keys\n        ]\n        column_sum = sum(column)\n        if column_sum <= 1e-12:\n            entropy_values[metric] = 1.0\n            continue\n        entropy = 0.0\n        for value in column:\n            if value <= 0.0:\n                continue\n            p_value = value / column_sum\n            entropy -= p_value * math.log(p_value)\n        entropy_values[metric] = entropy / math.log(n_items)\n\n    diversities = {\n        metric: max(0.0, 1.0 - entropy_values[metric]) for metric in metric_names\n    }\n    diversity_sum = sum(diversities.values())\n    if diversity_sum <= 1e-12:\n        weights = {metric: 1.0 / len(metric_names) for metric in metric_names}\n    else:\n        weights = {\n            metric: diversities[metric] / diversity_sum for metric in metric_names\n        }\n\n    weighted_matrix: dict[str, list[float]] = {}\n    for item in item_keys:\n        values: list[float] = []\n        for metric in metric_names:\n            denominator = math.sqrt(\n                sum(\n                    max(0.0, float(item_metrics[other].get(metric, 0.0))) ** 2\n                    for other in item_keys\n                )\n            )\n            normalized = safe_ratio(\n                max(0.0, float(item_metrics[item].get(metric, 0.0))), denominator\n            )\n            values.append(normalized * weights[metric])\n        weighted_matrix[item] = values\n\n    positive_ideal = [\n        max(weighted_matrix[item][idx] for item in item_keys)\n        for idx in range(len(metric_names))\n    ]\n    negative_ideal = [\n        min(weighted_matrix[item][idx] for item in item_keys)\n        for idx in range(len(metric_names))\n    ]\n    scores: dict[str, float] = {}\n    for item in item_keys:\n        values = weighted_matrix[item]\n        d_positive = math.sqrt(\n            sum(\n                (values[idx] - positive_ideal[idx]) ** 2\n                for idx in range(len(metric_names))\n            )\n        )\n        d_negative = math.sqrt(\n            sum(\n                (values[idx] - negative_ideal[idx]) ** 2\n                for idx in range(len(metric_names))\n            )\n        )\n        scores[item] = safe_ratio(d_negative, d_positive + d_negative)\n    return scores, weights\n\n\ndef build_track_tension_scores(\n    trains: list[TrainRecord],\n    resources_by_train: dict[str, tuple[ResourceWindow, ...]],\n    window_start: int,\n    window_end: int,\n) -> dict[str, dict[str, float]]:\n    usage_counts: defaultdict[str, float] = defaultdict(float)\n    occupancy_minutes: defaultdict[str, float] = defaultdict(float)\n    clipped_windows_by_track: defaultdict[str, list[ResourceWindow]] = defaultdict(list)\n    for train in trains:\n        if (\n            overlap_minutes(\n                train.dwell_start, train.dwell_end, window_start, window_end\n            )\n            > 0\n        ):\n            usage_counts[train.planned_track] += 1.0\n    for resources in resources_by_train.values():\n        for resource in resources:\n            if resource.resource_category != "track" or not resource.track:\n                continue\n            overlap = overlap_minutes(\n                resource.start_min, resource.end_min, window_start, window_end\n            )\n            if overlap <= 0:\n                continue\n            occupancy_minutes[resource.track] += float(overlap)\n            clipped_windows_by_track[resource.track].append(\n                clip_resource_window(resource, window_start, window_end)\n            )\n\n    track_keys = set(CANONICAL_TRACKS)\n    track_keys.update(usage_counts)\n    track_keys.update(occupancy_minutes)\n    peak_concurrency: dict[str, float] = {}\n    for track in track_keys:\n        peak, _ = max_resource_concurrency(\n            clipped_windows_by_track.get(track, []),\n            key_func=lambda item: item.track,\n        )\n        peak_concurrency[track] = float(peak)\n\n    item_metrics = {\n        track: {\n            "使用次数": usage_counts.get(track, 0.0),\n            "占用总时长": occupancy_minutes.get(track, 0.0),\n            "峰值并发": peak_concurrency.get(track, 0.0),\n            "冲突参与次数": count_overlapping_window_pairs(\n                clipped_windows_by_track.get(track, [])\n            ),\n        }\n        for track in track_keys\n    }\n    topsis_scores, entropy_weights = entropy_weight_topsis_scores(\n        item_metrics,\n        ["使用次数", "占用总时长", "峰值并发", "冲突参与次数"],\n    )\n\n    stats: dict[str, dict[str, float]] = {}\n    for track in sorted(track_keys, key=track_sort_key):\n        stats[track] = {\n            "使用次数": usage_counts.get(track, 0.0),\n            "占用总时长": occupancy_minutes.get(track, 0.0),\n            "峰值并发": peak_concurrency.get(track, 0.0),\n            "冲突参与次数": item_metrics[track]["冲突参与次数"],\n            "紧张度评分": topsis_scores.get(track, 0.0),\n            "熵权_使用次数": entropy_weights.get("使用次数", 0.0),\n            "熵权_占用总时长": entropy_weights.get("占用总时长", 0.0),\n            "熵权_峰值并发": entropy_weights.get("峰值并发", 0.0),\n            "熵权_冲突参与次数": entropy_weights.get("冲突参与次数", 0.0),\n        }\n    return stats\n\n\ndef build_partition_tension_scores(\n    resources_by_train: dict[str, tuple[ResourceWindow, ...]],\n    window_start: int,\n    window_end: int,\n) -> dict[str, dict[str, float]]:\n    usage_counts: defaultdict[str, float] = defaultdict(float)\n    occupancy_minutes: defaultdict[str, float] = defaultdict(float)\n    clipped_windows_by_partition: defaultdict[str, list[ResourceWindow]] = defaultdict(\n        list\n    )\n    for train_id, resources in resources_by_train.items():\n        counted_partitions: set[str] = set()\n        for resource in resources:\n            if resource.resource_category not in {\n                "route_lock",\n                "switch_ladder",\n                "flank_protection",\n            }:\n                continue\n            if not resource.throat or not resource.partition:\n                continue\n            overlap = overlap_minutes(\n                resource.start_min, resource.end_min, window_start, window_end\n            )\n            if overlap <= 0:\n                continue\n            partition_key = f"{resource.throat}-{resource.partition}"\n            occupancy_minutes[partition_key] += float(overlap)\n            clipped_windows_by_partition[partition_key].append(\n                clip_resource_window(resource, window_start, window_end)\n            )\n            counted_partitions.add(partition_key)\n        for partition_key in counted_partitions:\n            usage_counts[partition_key] += 1.0\n\n    partition_keys = set(usage_counts) | set(occupancy_minutes)\n    peak_concurrency: dict[str, float] = {}\n    for partition_key in partition_keys:\n        peak, _ = max_resource_concurrency(\n            clipped_windows_by_partition.get(partition_key, []),\n            key_func=lambda item: f"{item.throat}-{item.partition}",\n        )\n        peak_concurrency[partition_key] = float(peak)\n\n    item_metrics = {\n        key: {\n            "使用次数": usage_counts.get(key, 0.0),\n            "占用总时长": occupancy_minutes.get(key, 0.0),\n            "峰值并发": peak_concurrency.get(key, 0.0),\n            "冲突参与次数": count_overlapping_window_pairs(\n                clipped_windows_by_partition.get(key, [])\n            ),\n        }\n        for key in partition_keys\n    }\n    topsis_scores, entropy_weights = entropy_weight_topsis_scores(\n        item_metrics,\n        ["使用次数", "占用总时长", "峰值并发", "冲突参与次数"],\n    )\n\n    stats: dict[str, dict[str, float]] = {}\n    for partition_key in sorted(partition_keys):\n        stats[partition_key] = {\n            "使用次数": usage_counts.get(partition_key, 0.0),\n            "占用总时长": occupancy_minutes.get(partition_key, 0.0),\n            "峰值并发": peak_concurrency.get(partition_key, 0.0),\n            "冲突参与次数": item_metrics[partition_key]["冲突参与次数"],\n            "紧张度评分": topsis_scores.get(partition_key, 0.0),\n            "熵权_使用次数": entropy_weights.get("使用次数", 0.0),\n            "熵权_占用总时长": entropy_weights.get("占用总时长", 0.0),\n            "熵权_峰值并发": entropy_weights.get("峰值并发", 0.0),\n            "熵权_冲突参与次数": entropy_weights.get("冲突参与次数", 0.0),\n        }\n    return stats\n\n\ndef select_disturbance_minute(\n    load_profile: list[dict[str, float]],\n    rng: random.Random,\n) -> dict[str, float]:\n    if not load_profile:\n        raise ValueError("扰动负荷序列为空，无法抽取扰动时刻。")\n    probability_sum = sum(float(row.get("NHPP扰动概率", 0.0)) for row in load_profile)\n    if probability_sum <= 1e-12:\n        peak_row = first_peak_load_row(load_profile)\n        if peak_row is None:\n            raise ValueError("未能从扰动负荷序列中识别峰值时刻。")\n        return peak_row\n    threshold = rng.random() * probability_sum\n    cumulative = 0.0\n    for row in load_profile:\n        cumulative += float(row.get("NHPP扰动概率", 0.0))\n        if cumulative >= threshold:\n            return row\n    return load_profile[-1]\n\n\ndef select_disturbance_load_rows(\n    load_profile: list[dict[str, float]],\n    rng: random.Random,\n) -> tuple[dict[str, float], ...]:\n    if not load_profile:\n        raise ValueError("扰动负荷序列为空，无法抽取扰动时刻。")\n    sorted_rows = sorted(\n        load_profile,\n        key=lambda row: (\n            -float(row.get("NHPP扰动概率", 0.0)),\n            -float(row.get("负荷指数", 0.0)),\n            int(round(row["分钟"])),\n        ),\n    )\n    candidate_count = contribution_limited_count(\n        [\n            float(row.get("NHPP扰动概率", 0.0)) + 0.01 * float(row.get("负荷指数", 0.0))\n            for row in sorted_rows\n        ],\n        DISTURBANCE_TIME_CONTRIBUTION_THRESHOLD,\n        DISTURBANCE_TIME_MAX_COUNT,\n    )\n    if candidate_count <= 0:\n        selected = [select_disturbance_minute(load_profile, rng)]\n    else:\n        selected = sorted_rows[:candidate_count]\n    unique_by_minute: dict[int, dict[str, float]] = {}\n    for row in selected:\n        unique_by_minute.setdefault(int(round(row["分钟"])), row)\n    return tuple(\n        sorted(unique_by_minute.values(), key=lambda row: int(round(row["分钟"])))\n    )\n\n\ndef train_shapley_resource_score(\n    train: TrainRecord,\n    resources_by_train: dict[str, tuple[ResourceWindow, ...]],\n    track_scores: dict[str, float],\n    partition_scores: dict[str, float],\n) -> float:\n    partitions = {\n        f"{resource.throat}-{resource.partition}"\n        for resource in resources_by_train[train.record_id]\n        if resource.throat and resource.partition\n    }\n    return track_scores.get(train.planned_track, 0.0) + sum(\n        partition_scores.get(partition_key, 0.0) for partition_key in partitions\n    )\n\n\ndef approximate_source_train_shapley_scores(\n    candidates: list[TrainRecord],\n    resources_by_train: dict[str, tuple[ResourceWindow, ...]],\n    disturbance_minute: int,\n    track_scores: dict[str, float],\n    partition_scores: dict[str, float],\n    rng: random.Random,\n) -> dict[str, float]:\n    if not candidates:\n        return {}\n    resource_scores = {\n        train.record_id: train_shapley_resource_score(\n            train, resources_by_train, track_scores, partition_scores\n        )\n        for train in candidates\n    }\n    proximity_scores = {\n        train.record_id: 1.0\n        / (\n            1.0\n            + min(\n                abs(train.arrival_min - disturbance_minute),\n                abs(train.departure_min - disturbance_minute),\n                abs(train.window_start - disturbance_minute),\n                abs(train.window_end - disturbance_minute),\n            )\n        )\n        for train in candidates\n    }\n    overlap_neighbors: dict[str, set[str]] = {\n        train.record_id: set() for train in candidates\n    }\n    downstream_neighbors: dict[str, set[str]] = {\n        train.record_id: set() for train in candidates\n    }\n    for train in candidates:\n        for other in candidates:\n            if other.record_id == train.record_id:\n                continue\n            if (\n                overlap_minutes(\n                    train.window_start,\n                    train.window_end,\n                    other.window_start,\n                    other.window_end,\n                )\n                > 0\n            ):\n                overlap_neighbors[train.record_id].add(other.record_id)\n            if (\n                other.window_start >= train.window_start\n                and overlap_minutes(\n                    train.window_start,\n                    train.window_end + 20,\n                    other.window_start,\n                    other.window_end,\n                )\n                > 0\n            ):\n                downstream_neighbors[train.record_id].add(other.record_id)\n\n    def coalition_value(coalition: set[str]) -> float:\n        if not coalition:\n            return 0.0\n        base_value = sum(\n            resource_scores.get(train_id, 0.0) + proximity_scores.get(train_id, 0.0)\n            for train_id in coalition\n        )\n        overlap_value = (\n            sum(len(overlap_neighbors[train_id] & coalition) for train_id in coalition)\n            * 0.25\n        )\n        downstream_value = (\n            sum(\n                len(downstream_neighbors[train_id] - coalition)\n                for train_id in coalition\n            )\n            * 0.15\n        )\n        return base_value + overlap_value + downstream_value\n\n    shapley_scores = {train.record_id: 0.0 for train in candidates}\n    sample_count = max(\n        1, min(SHAPLEY_APPROX_SAMPLES, math.factorial(min(len(candidates), 7)))\n    )\n    candidate_ids = [train.record_id for train in candidates]\n    for _ in range(sample_count):\n        permutation = list(candidate_ids)\n        rng.shuffle(permutation)\n        coalition: set[str] = set()\n        current_value = 0.0\n        for train_id in permutation:\n            new_coalition = set(coalition)\n            new_coalition.add(train_id)\n            new_value = coalition_value(new_coalition)\n            shapley_scores[train_id] += new_value - current_value\n            coalition = new_coalition\n            current_value = new_value\n    return {\n        train_id: max(0.05, score / sample_count)\n        for train_id, score in shapley_scores.items()\n    }\n\n\ndef select_source_train_ids(\n    trains: list[TrainRecord],\n    resources_by_train: dict[str, tuple[ResourceWindow, ...]],\n    disturbance_minute: int,\n    track_scores: dict[str, float],\n    partition_scores: dict[str, float],\n    rng: random.Random,\n) -> tuple[tuple[str, ...], dict[str, float]]:\n    candidate_start = disturbance_minute - DISTURBANCE_TRAIN_LOOKAROUND_MIN\n    candidate_end = disturbance_minute + DISTURBANCE_TRAIN_LOOKAROUND_MIN + 1\n    candidates = [\n        train\n        for train in trains\n        if overlap_minutes(\n            train.window_start, train.window_end, candidate_start, candidate_end\n        )\n        > 0\n    ]\n    candidate_scores = approximate_source_train_shapley_scores(\n        candidates,\n        resources_by_train,\n        disturbance_minute,\n        track_scores,\n        partition_scores,\n        rng,\n    )\n\n    if not candidate_scores:\n        fallback_trains = sorted(\n            trains,\n            key=lambda item: (\n                min(\n                    abs(item.arrival_min - disturbance_minute),\n                    abs(item.departure_min - disturbance_minute),\n                ),\n                item.index,\n            ),\n        )\n        candidate_scores = {\n            train.record_id: 1.0 / (rank + 1)\n            for rank, train in enumerate(\n                fallback_trains[: max(DISTURBANCE_SOURCE_TRAIN_RANGE)]\n            )\n        }\n\n    selected = sorted(\n        candidate_scores,\n        key=lambda train_id: (-candidate_scores[train_id], train_id),\n    )\n    min_count, max_count = DISTURBANCE_SOURCE_TRAIN_RANGE\n    sample_size = contribution_limited_count(\n        [candidate_scores[train_id] for train_id in selected],\n        DISTURBANCE_SOURCE_CONTRIBUTION_THRESHOLD,\n        max_count,\n        min_count,\n    )\n    return tuple(selected[:sample_size]), candidate_scores\n\n\ndef select_spatial_targets(\n    track_stats: dict[str, dict[str, float]],\n    partition_stats: dict[str, dict[str, float]],\n    rng: random.Random,\n) -> tuple[DisturbanceTarget, ...]:\n    pool: list[tuple[str, str, float]] = []\n    for track, stat in track_stats.items():\n        if stat.get("使用次数", 0.0) > 0 or stat.get("占用总时长", 0.0) > 0:\n            pool.append(("track", track, float(stat.get("紧张度评分", 0.0))))\n    for partition_key, stat in partition_stats.items():\n        if stat.get("使用次数", 0.0) > 0 or stat.get("占用总时长", 0.0) > 0:\n            pool.append(\n                ("partition", partition_key, float(stat.get("紧张度评分", 0.0)))\n            )\n    if not pool:\n        return tuple()\n\n    ordered_pool = sorted(pool, key=lambda item: (-item[2], item[0], item[1]))\n    min_count, max_count = DISTURBANCE_SPATIAL_RESOURCE_RANGE\n    sample_size = contribution_limited_count(\n        [item[2] for item in ordered_pool],\n        DISTURBANCE_RESOURCE_CONTRIBUTION_THRESHOLD,\n        max_count,\n        min_count,\n    )\n    selected = ordered_pool[:sample_size]\n    targets: list[DisturbanceTarget] = []\n    for resource_type, resource_key, score in selected:\n        if resource_type == "track":\n            duration = sample_markov_device_duration(rng, score, resource_type)\n            targets.append(\n                DisturbanceTarget(\n                    resource_type="track",\n                    resource_key=resource_key,\n                    display_name=f"股道{resource_key}",\n                    throat="",\n                    partition="",\n                    tension_score=score,\n                    intensity_value=float(duration),\n                    intensity_unit="分钟",\n                    note="股道临时封锁时长由马尔可夫状态转移生成。",\n                    duration_min=duration,\n                )\n            )\n        else:\n            throat, partition = resource_key.split("-", 1)\n            min_ratio, max_ratio = DISTURBANCE_SWITCH_SLOWDOWN_RANGE\n            slowdown_ratio = min_ratio + (max_ratio - min_ratio) * score\n            duration = sample_markov_device_duration(rng, score, resource_type)\n            targets.append(\n                DisturbanceTarget(\n                    resource_type="partition",\n                    resource_key=resource_key,\n                    display_name=f"{throat}-{partition}",\n                    throat=throat,\n                    partition=partition,\n                    tension_score=score,\n                    intensity_value=slowdown_ratio,\n                    intensity_unit="比例",\n                    note=f"道岔组/分区退化持续{duration}分钟，由马尔可夫状态转移生成。",\n                    duration_min=duration,\n                )\n            )\n    return tuple(targets)\n\n\ndef sample_source_delay_minutes(rng: random.Random) -> int:\n    sampled = rng.weibullvariate(WEIBULL_DELAY_SCALE, WEIBULL_DELAY_SHAPE)\n    return max(1, min(WEIBULL_DELAY_CAP, int(round(sampled))))\n\n\ndef weighted_state_transition(\n    rng: random.Random,\n    transitions: list[tuple[str, float]],\n) -> str:\n    total = sum(max(0.0, weight) for _, weight in transitions)\n    if total <= 1e-12:\n        return transitions[-1][0]\n    threshold = rng.random() * total\n    cumulative = 0.0\n    for state, weight in transitions:\n        cumulative += max(0.0, weight)\n        if cumulative >= threshold:\n            return state\n    return transitions[-1][0]\n\n\ndef sample_markov_device_duration(\n    rng: random.Random,\n    tension_score: float,\n    resource_type: str,\n) -> int:\n    min_duration = DISTURBANCE_TRACK_BLOCK_RANGE[0] if resource_type == "track" else 2\n    score = max(0.0, min(1.0, tension_score))\n    state = "F" if resource_type == "track" else "D"\n    duration = 0\n    for _ in range(MARKOV_DEVICE_MAX_DURATION):\n        if state in {"D", "F", "R"}:\n            duration += 1\n        if state == "D":\n            state = weighted_state_transition(\n                rng,\n                [\n                    ("D", 0.35 + 0.20 * score),\n                    ("F", 0.15 + 0.20 * score),\n                    ("R", 0.35),\n                    ("N", 0.15),\n                ],\n            )\n        elif state == "F":\n            state = weighted_state_transition(\n                rng,\n                [\n                    ("F", 0.35 + 0.25 * score),\n                    ("R", 0.50 - 0.10 * score),\n                    ("D", 0.10),\n                    ("N", 0.05),\n                ],\n            )\n        elif state == "R":\n            state = weighted_state_transition(\n                rng,\n                [("N", 0.65), ("R", 0.25), ("D", 0.10)],\n            )\n        else:\n            if duration >= min_duration:\n                break\n            state = "R"\n    return max(min_duration, min(MARKOV_DEVICE_MAX_DURATION, duration))\n\n\ndef scenario_events(scenario: DisturbanceScenario) -> tuple[DisturbanceEvent, ...]:\n    if scenario.disturbance_events:\n        return scenario.disturbance_events\n    return (\n        DisturbanceEvent(\n            disturbance_minute=scenario.disturbance_minute,\n            selected_train_load=scenario.selected_train_load,\n            selected_switch_load=scenario.selected_switch_load,\n            selected_load_score=scenario.selected_load_score,\n            source_train_ids=scenario.source_train_ids,\n            source_train_delays=scenario.source_train_delays,\n            source_train_scores=scenario.source_train_scores,\n            spatial_targets=scenario.spatial_targets,\n        ),\n    )\n\n\ndef scenario_disturbance_minutes(scenario: DisturbanceScenario) -> tuple[int, ...]:\n    minutes = tuple(event.disturbance_minute for event in scenario_events(scenario))\n    return tuple(sorted(dict.fromkeys(minutes))) or (scenario.disturbance_minute,)\n\n\ndef format_disturbance_minutes(scenario: DisturbanceScenario) -> str:\n    return "、".join(\n        format_minutes_as_clock(minute)\n        for minute in scenario_disturbance_minutes(scenario)\n    )\n\n\ndef scenario_effect_start_minute(scenario: DisturbanceScenario) -> int:\n    return min(scenario_disturbance_minutes(scenario))\n\n\ndef disturbance_resource_group(resource: ResourceWindow) -> str:\n    if (\n        resource.resource_category in {"track", "track_access", "terminal_op"}\n        and resource.track\n    ):\n        return f"track:{resource.track}"\n    if (\n        resource.resource_category\n        in {"route_lock", "switch_ladder", "flank_protection"}\n        and resource.throat\n        and resource.partition\n    ):\n        return f"partition:{resource.throat}-{resource.partition}"\n    if resource.resource_category == "throat_capacity" and resource.throat:\n        return f"throat:{resource.throat}"\n    return ""\n\n\ndef disturbance_delay_scope(resource: ResourceWindow) -> str:\n    if resource.stage in {"C-D", "D", "D-E"}:\n        return "departure"\n    return "both"\n\n\ndef apply_initial_disturbance_effects(\n    trains: list[TrainRecord],\n    resources_by_train: dict[str, tuple[ResourceWindow, ...]],\n    disturbance_minute: int,\n    analysis_window_end: int,\n    source_train_delays: dict[str, int],\n    spatial_targets: tuple[DisturbanceTarget, ...],\n) -> tuple[list[TrainRecord], dict[str, list[str]], list[dict[str, object]]]:\n    current_by_id = {train.record_id: train for train in trains}\n    delay_notes: defaultdict[str, list[str]] = defaultdict(list)\n    closure_blocks: list[dict[str, object]] = []\n\n    for train in trains:\n        direct_delay = source_train_delays.get(train.record_id, 0)\n        if direct_delay <= 0:\n            continue\n        current_by_id[train.record_id] = rebuild_train_record(\n            current_by_id[train.record_id],\n            arrival_shift=direct_delay,\n            departure_shift=direct_delay,\n        )\n        delay_notes[train.record_id].append(\n            f"Shapley源头列车，Weibull初始晚点+{direct_delay}分"\n        )\n\n    for target in spatial_targets:\n        target_duration = (\n            target.duration_min\n            if target.duration_min > 0\n            else int(round(target.intensity_value))\n            if target.resource_type == "track"\n            else analysis_window_end - disturbance_minute\n        )\n        target_end_min = min(\n            analysis_window_end, disturbance_minute + max(1, target_duration)\n        )\n        if target.resource_type == "track":\n            closure_blocks.append(\n                {\n                    "group_key": f"track:{target.resource_key}",\n                    "start_min": disturbance_minute,\n                    "end_min": target_end_min,\n                    "display_name": target.display_name,\n                }\n            )\n            continue\n\n        for train in trains:\n            arrival_extra = 0\n            departure_extra = 0\n            for resource in resources_by_train[train.record_id]:\n                if resource.resource_category != "switch_ladder":\n                    continue\n                if (\n                    resource.throat != target.throat\n                    or resource.partition != target.partition\n                ):\n                    continue\n                if (\n                    overlap_minutes(\n                        resource.start_min,\n                        resource.end_min,\n                        disturbance_minute,\n                        target_end_min,\n                    )\n                    <= 0\n                ):\n                    continue\n                if resource.stage == "B":\n                    arrival_extra += max(\n                        1, int(math.ceil(train.b_min * target.intensity_value))\n                    )\n                elif resource.stage == "C-D":\n                    departure_extra += max(\n                        1, int(math.ceil(train.cd_min * target.intensity_value))\n                    )\n            if arrival_extra <= 0 and departure_extra <= 0:\n                continue\n            current_by_id[train.record_id] = rebuild_train_record(\n                current_by_id[train.record_id],\n                arrival_shift=arrival_extra,\n                departure_shift=arrival_extra + departure_extra,\n            )\n            if arrival_extra > 0:\n                delay_notes[train.record_id].append(\n                    f"{target.display_name}接车作业放缓+{arrival_extra}分"\n                )\n            if departure_extra > 0:\n                delay_notes[train.record_id].append(\n                    f"{target.display_name}发车作业放缓+{departure_extra}分"\n                )\n\n    disturbed_trains = [current_by_id[train.record_id] for train in trains]\n    return (\n        disturbed_trains,\n        {train_id: list(notes) for train_id, notes in delay_notes.items()},\n        closure_blocks,\n    )\n\n\ndef apply_disturbance_events(\n    trains: list[TrainRecord],\n    resources_by_train: dict[str, tuple[ResourceWindow, ...]],\n    analysis_window_end: int,\n    events: tuple[DisturbanceEvent, ...],\n) -> tuple[list[TrainRecord], dict[str, list[str]], list[dict[str, object]]]:\n    current_trains = list(trains)\n    merged_notes: dict[str, list[str]] = {}\n    closure_blocks: list[dict[str, object]] = []\n    for event in sorted(events, key=lambda item: item.disturbance_minute):\n        original_by_id = {train.record_id: train for train in trains}\n        current_by_id = {train.record_id: train for train in current_trains}\n        shifted_resources_by_train = {}\n        for train_id, resources in resources_by_train.items():\n            original_train = original_by_id.get(train_id)\n            current_train = current_by_id.get(train_id)\n            if original_train is None or current_train is None:\n                shifted_resources_by_train[train_id] = resources\n            else:\n                shifted_resources_by_train[train_id] = tuple(\n                    retime_resource_window(resource, original_train, current_train)\n                    for resource in resources\n                )\n        current_trains, event_notes, event_blocks = apply_initial_disturbance_effects(\n            trains=current_trains,\n            resources_by_train=shifted_resources_by_train,\n            disturbance_minute=event.disturbance_minute,\n            analysis_window_end=analysis_window_end,\n            source_train_delays=event.source_train_delays,\n            spatial_targets=event.spatial_targets,\n        )\n        for train_id, notes in event_notes.items():\n            stamped_notes = [\n                f"{format_minutes_as_clock(event.disturbance_minute)} {note}"\n                for note in notes\n            ]\n            merged_notes.setdefault(train_id, []).extend(stamped_notes)\n        closure_blocks.extend(event_blocks)\n    return (\n        current_trains,\n        {\n            train_id: unique_preserve_order(notes)\n            for train_id, notes in merged_notes.items()\n        },\n        closure_blocks,\n    )\n\n\ndef build_local_rearranged_assignment(\n    trains: list[TrainRecord],\n    library: dict[str, object],\n    max_route_candidates: int,\n) -> tuple[dict[str, CandidateRoutePlan], dict[str, list[str]]]:\n    options_by_train, train_map = build_candidate_route_plans(\n        trains=trains,\n        library=library,\n        max_route_candidates=max_route_candidates,\n    )\n    raw_pair_costs, hard_counts, hard_risks = build_pairwise_conflict_maps(\n        trains=trains,\n        options_by_train=options_by_train,\n        mode="full",\n        hard_soft_enabled=True,\n        use_variant_penalty=False,\n    )\n    pair_costs = build_weighted_pair_costs(\n        raw_pair_costs, DEFAULT_MODEL_WEIGHTS["safety_weight"]\n    )\n    linear_costs = build_weighted_linear_costs(\n        options_by_train,\n        DEFAULT_MODEL_WEIGHTS["safety_weight"],\n        DEFAULT_MODEL_WEIGHTS["stability_weight"],\n    )\n    train_order = build_train_order(trains, options_by_train)\n    selected_option_ids = build_planned_assignment_ids(trains, options_by_train)\n    selected_option_ids = greedy_descent(\n        selected_option_ids,\n        train_order,\n        options_by_train,\n        linear_costs,\n        pair_costs,\n        hard_counts,\n        hard_risks,\n        train_map,\n        max_passes=4,\n    )\n    selected_option_ids = repair_hard_conflicts(\n        trains=trains,\n        train_map=train_map,\n        options_by_train=options_by_train,\n        selected_option_ids=selected_option_ids,\n        linear_costs=linear_costs,\n        pair_costs=pair_costs,\n        hard_counts=hard_counts,\n        hard_risks=hard_risks,\n    )\n    assignment = build_assignment_lookup(selected_option_ids, options_by_train)\n    notes: dict[str, list[str]] = {}\n    for train in trains:\n        option = assignment[train.record_id]\n        if (\n            option.track != train.planned_track\n            or option.in_throat != train.in_throat\n            or option.out_throat != train.out_throat\n        ):\n            notes[train.record_id] = [\n                (\n                    f"局部重排进站:股道{train.planned_track}->{option.track};"\n                    f"咽喉{train.in_throat}/{train.out_throat}->{option.in_throat}/{option.out_throat}"\n                )\n            ]\n    return assignment, notes\n\n\ndef propagate_planned_disturbance(\n    trains: list[TrainRecord],\n    assignment: dict[str, CandidateRoutePlan],\n    closure_blocks: list[dict[str, object]],\n    delay_notes: dict[str, list[str]],\n    library: dict[str, object],\n    max_route_candidates: int,\n    disturbance_minute: int,\n) -> tuple[list[TrainRecord], dict[str, CandidateRoutePlan], int]:\n    current_by_id = {train.record_id: train for train in trains}\n    current_assignment = dict(assignment)\n    propagation_rounds = 0\n    active_train_ids = set(delay_notes)\n    current_trains = [\n        current_by_id[train.record_id]\n        for train in sorted(current_by_id.values(), key=lambda item: item.index)\n    ]\n    current_assignment = rebuild_assignment_for_trains(\n        current_trains, current_assignment\n    )\n    active_train_ids.update(\n        closure_impacted_train_ids(\n            current_assignment, closure_blocks, disturbance_minute\n        )\n    )\n    if not active_train_ids:\n        final_assignment = rebuild_assignment_for_trains(\n            current_trains, current_assignment\n        )\n        return current_trains, final_assignment, propagation_rounds\n\n    for _ in range(DISTURBANCE_PROPAGATION_MAX_ROUNDS):\n        current_trains = [\n            current_by_id[train.record_id]\n            for train in sorted(current_by_id.values(), key=lambda item: item.index)\n        ]\n        current_assignment = rebuild_assignment_for_trains(\n            current_trains, current_assignment\n        )\n        train_map = {train.record_id: train for train in current_trains}\n        conflicts = collect_local_chain_conflicts(\n            current_trains, current_assignment, active_train_ids, "扰动传播重构"\n        )\n        frontier = local_conflict_frontier(\n            conflicts, active_train_ids, disturbance_minute, train_map\n        )\n        frontier.update(\n            closure_impacted_train_ids(\n                current_assignment, closure_blocks, disturbance_minute\n            )\n        )\n        frontier = {\n            train_id\n            for train_id in frontier\n            if train_id in train_map\n            and train_map[train_id].window_end > disturbance_minute\n        }\n        if not frontier:\n            break\n\n        propagation_rounds += 1\n        reroute_options_by_train, _ = build_candidate_route_plans(\n            trains=current_trains,\n            library=library,\n            max_route_candidates=max(4, max_route_candidates),\n            rich_candidate_train_ids=frontier,\n            recovery_aggressive_train_ids=frontier,\n        )\n        linear_costs = build_weighted_linear_costs(\n            reroute_options_by_train,\n            DEFAULT_MODEL_WEIGHTS["safety_weight"],\n            DEFAULT_MODEL_WEIGHTS["stability_weight"],\n        )\n        changed_train_ids: set[str] = set()\n        for train_id in sorted(\n            frontier,\n            key=lambda item: (\n                -sum(\n                    1\n                    for conflict in conflicts\n                    if item in conflict_pair_train_ids(conflict)\n                ),\n                train_map[item].arrival_min,\n                train_map[item].index,\n            ),\n        )[:DISTURBANCE_REARRANGEMENT_FRONTIER_LIMIT]:\n            reroute_option = choose_local_rearrangement_option(\n                train_id,\n                train_map,\n                reroute_options_by_train,\n                current_assignment,\n                closure_blocks,\n                frontier,\n                linear_costs,\n            )\n            if reroute_option is not None:\n                previous_option = current_assignment[train_id]\n                current_assignment[train_id] = reroute_option\n                delay_notes.setdefault(train_id, []).append(\n                    f"传播链资源重排:股道{previous_option.track}->{reroute_option.track};"\n                    f"咽喉{previous_option.in_throat}/{previous_option.out_throat}"\n                    f"->{reroute_option.in_throat}/{reroute_option.out_throat}"\n                )\n                changed_train_ids.add(train_id)\n                continue\n\n            if apply_local_small_wait(\n                train_id,\n                current_by_id,\n                current_assignment,\n                closure_blocks,\n                frontier,\n                delay_notes,\n            ):\n                changed_train_ids.add(train_id)\n\n        if not changed_train_ids:\n            break\n        active_train_ids.update(changed_train_ids)\n        for conflict in conflicts:\n            left_id, right_id = conflict_pair_train_ids(conflict)\n            if left_id in changed_train_ids or right_id in changed_train_ids:\n                active_train_ids.add(left_id)\n                active_train_ids.add(right_id)\n\n    disturbed_trains = [\n        current_by_id[train.record_id]\n        for train in sorted(current_by_id.values(), key=lambda item: item.index)\n    ]\n    final_assignment = rebuild_assignment_for_trains(\n        disturbed_trains, current_assignment\n    )\n    return disturbed_trains, final_assignment, propagation_rounds\n\n\ndef option_overlaps_closure(\n    option: CandidateRoutePlan, closure_blocks: list[dict[str, object]]\n) -> bool:\n    for resource in option.resources:\n        group_key = disturbance_resource_group(resource)\n        if not group_key:\n            continue\n        for block in closure_blocks:\n            if str(block["group_key"]) != group_key:\n                continue\n            if (\n                overlap_minutes(\n                    resource.start_min,\n                    resource.end_min,\n                    int(block["start_min"]),\n                    int(block["end_min"]),\n                )\n                > 0\n            ):\n                return True\n    return False\n\n\ndef option_has_resource_conflict(\n    train_id: str,\n    option: CandidateRoutePlan,\n    assignment: dict[str, CandidateRoutePlan],\n) -> bool:\n    for other_train_id, other_option in assignment.items():\n        if other_train_id == train_id:\n            continue\n        if collect_pair_conflict_details(option, other_option):\n            return True\n    return False\n\n\ndef find_propagation_reroute_option(\n    train_id: str,\n    options_by_train: dict[str, list[CandidateRoutePlan]],\n    assignment: dict[str, CandidateRoutePlan],\n    closure_blocks: list[dict[str, object]],\n) -> CandidateRoutePlan | None:\n    current_option = assignment[train_id]\n    for option in options_by_train.get(train_id, []):\n        if option.option_id == current_option.option_id:\n            continue\n        if option_resource_signature(option) == option_resource_signature(\n            current_option\n        ):\n            continue\n        if option_overlaps_closure(option, closure_blocks):\n            continue\n        if option_has_resource_conflict(train_id, option, assignment):\n            continue\n        return option\n    return None\n\n\ndef conflict_pair_train_ids(conflict: ConflictEntry) -> tuple[str, str]:\n    return conflict.train1.split("-", 1)[0], conflict.train2.split("-", 1)[0]\n\n\ndef closure_impacted_train_ids(\n    assignment: dict[str, CandidateRoutePlan],\n    closure_blocks: list[dict[str, object]],\n    disturbance_minute: int,\n) -> set[str]:\n    impacted: set[str] = set()\n    if not closure_blocks:\n        return impacted\n    for train_id, option in assignment.items():\n        if option_overlaps_closure(option, closure_blocks):\n            impacted.add(train_id)\n            continue\n        if any(\n            resource.end_min > disturbance_minute\n            and any(\n                str(block["group_key"]) == disturbance_resource_group(resource)\n                and resource.start_min\n                <= int(block["end_min"]) + DISTURBANCE_TRAIN_LOOKAROUND_MIN\n                and resource.end_min\n                >= int(block["start_min"]) - DISTURBANCE_TRAIN_LOOKAROUND_MIN\n                for block in closure_blocks\n            )\n            for resource in option.resources\n        ):\n            impacted.add(train_id)\n    return impacted\n\n\ndef local_conflict_frontier(\n    conflicts: list[ConflictEntry],\n    active_train_ids: set[str],\n    disturbance_minute: int,\n    train_map: dict[str, TrainRecord],\n) -> set[str]:\n    frontier: set[str] = set(active_train_ids)\n    for conflict in conflicts:\n        left_id, right_id = conflict_pair_train_ids(conflict)\n        if left_id not in train_map or right_id not in train_map:\n            continue\n        if (\n            train_map[left_id].window_end <= disturbance_minute\n            and train_map[right_id].window_end <= disturbance_minute\n        ):\n            continue\n        if left_id in active_train_ids or right_id in active_train_ids:\n            frontier.add(left_id)\n            frontier.add(right_id)\n    return frontier\n\n\ndef collect_local_chain_conflicts(\n    trains: list[TrainRecord],\n    assignment: dict[str, CandidateRoutePlan],\n    active_train_ids: set[str],\n    scheme: str,\n) -> list[ConflictEntry]:\n    if not active_train_ids:\n        return []\n    train_map = {train.record_id: train for train in trains}\n    candidate_ids: set[str] = set(active_train_ids)\n    for active_id in active_train_ids:\n        active_train = train_map.get(active_id)\n        if active_train is None:\n            continue\n        for train in trains:\n            if train.record_id == active_id:\n                continue\n            if (\n                abs(train.window_start - active_train.window_start)\n                <= DISTURBANCE_LOCAL_CONFLICT_WINDOW\n            ):\n                candidate_ids.add(train.record_id)\n    candidate_trains = [\n        train_map[train_id]\n        for train_id in candidate_ids\n        if train_id in train_map and train_id in assignment\n    ]\n    local_conflicts: list[ConflictEntry] = []\n    for left_index, train_a in enumerate(candidate_trains):\n        option_a = assignment[train_a.record_id]\n        for right_index in range(left_index + 1, len(candidate_trains)):\n            train_b = candidate_trains[right_index]\n            if (\n                train_a.record_id not in active_train_ids\n                and train_b.record_id not in active_train_ids\n            ):\n                continue\n            option_b = assignment[train_b.record_id]\n            detail_list = collect_pair_conflict_details(option_a, option_b)\n            for detail in detail_list:\n                local_conflicts.append(\n                    ConflictEntry(\n                        scheme=scheme,\n                        conflict_level=detail.conflict_level,\n                        conflict_type=detail.conflict_type,\n                        train1=f"{train_a.record_id}-{train_a.train_no}",\n                        train2=f"{train_b.record_id}-{train_b.train_no}",\n                        route1=option_a.route_code,\n                        route2=option_b.route_code,\n                        resource=detail.resource_name,\n                        overlap_min=detail.overlap_min,\n                        interval1=format_interval(\n                            train_a.window_start, train_a.window_end\n                        ),\n                        interval2=format_interval(\n                            train_b.window_start, train_b.window_end\n                        ),\n                        track1=option_a.track,\n                        track2=option_b.track,\n                        description=detail.description,\n                    )\n                )\n    return local_conflicts\n\n\ndef choose_local_rearrangement_option(\n    train_id: str,\n    train_map: dict[str, TrainRecord],\n    options_by_train: dict[str, list[CandidateRoutePlan]],\n    assignment: dict[str, CandidateRoutePlan],\n    closure_blocks: list[dict[str, object]],\n    active_train_ids: set[str],\n    linear_costs: dict[str, float],\n) -> CandidateRoutePlan | None:\n    current_option = assignment[train_id]\n    best_option = current_option\n    best_key: tuple[int, int, int, float, int, str] | None = None\n    train = train_map[train_id]\n    for option in options_by_train.get(train_id, []):\n        if option.option_id != current_option.option_id and option.candidate_rank > 18:\n            continue\n        candidate_option = rebuild_option_for_train(train, option)\n        if option_overlaps_closure(candidate_option, closure_blocks):\n            continue\n        local_conflict_count = 0\n        local_overlap_total = 0\n        global_conflict_count = 0\n        for other_train_id, other_option in assignment.items():\n            if other_train_id == train_id:\n                continue\n            details = collect_pair_conflict_details(candidate_option, other_option)\n            if not details:\n                continue\n            detail_count = len(details)\n            overlap_total = sum(detail.overlap_min for detail in details)\n            global_conflict_count += detail_count\n            if other_train_id in active_train_ids:\n                local_conflict_count += detail_count\n                local_overlap_total += overlap_total\n        change_penalty = (\n            0\n            if assignment_option_signature(candidate_option)\n            != assignment_option_signature(current_option)\n            else 1\n        )\n        key = (\n            local_conflict_count,\n            local_overlap_total,\n            global_conflict_count,\n            linear_costs.get(candidate_option.option_id, candidate_option.linear_cost),\n            change_penalty,\n            candidate_option.option_id,\n        )\n        if best_key is None or key < best_key:\n            best_key = key\n            best_option = candidate_option\n    if assignment_option_signature(best_option) == assignment_option_signature(\n        current_option\n    ):\n        return None\n    return best_option\n\n\ndef apply_local_small_wait(\n    train_id: str,\n    current_by_id: dict[str, TrainRecord],\n    assignment: dict[str, CandidateRoutePlan],\n    closure_blocks: list[dict[str, object]],\n    active_train_ids: set[str],\n    delay_notes: dict[str, list[str]],\n) -> bool:\n    train = current_by_id[train_id]\n    current_option = assignment[train_id]\n    best_train: TrainRecord | None = None\n    best_option: CandidateRoutePlan | None = None\n    best_key: tuple[int, int, int] | None = None\n    current_conflict_count = 0\n    current_overlap_total = 0\n    if option_overlaps_closure(current_option, closure_blocks):\n        current_conflict_count += 1\n        current_overlap_total += 1\n    for other_train_id in active_train_ids:\n        if other_train_id == train_id or other_train_id not in assignment:\n            continue\n        if (\n            other_train_id in current_by_id\n            and abs(current_by_id[other_train_id].window_start - train.window_start)\n            > 30\n        ):\n            continue\n        details = collect_pair_conflict_details(\n            current_option, assignment[other_train_id]\n        )\n        current_conflict_count += len(details)\n        current_overlap_total += sum(detail.overlap_min for detail in details)\n    current_key = (current_conflict_count, current_overlap_total, 0)\n    if current_conflict_count <= 0:\n        return False\n    for shift in range(1, DISTURBANCE_LOCAL_WAIT_CAP + 1):\n        candidate_train = rebuild_train_record(\n            train, arrival_shift=shift, departure_shift=shift\n        )\n        candidate_option = rebuild_option_for_train(candidate_train, current_option)\n        if option_overlaps_closure(candidate_option, closure_blocks):\n            continue\n        local_conflict_count = 0\n        local_overlap_total = 0\n        for other_train_id in active_train_ids:\n            if other_train_id == train_id or other_train_id not in assignment:\n                continue\n            if (\n                other_train_id in current_by_id\n                and abs(\n                    current_by_id[other_train_id].window_start\n                    - candidate_train.window_start\n                )\n                > 30\n            ):\n                continue\n            details = collect_pair_conflict_details(\n                candidate_option, assignment[other_train_id]\n            )\n            local_conflict_count += len(details)\n            local_overlap_total += sum(detail.overlap_min for detail in details)\n        key = (local_conflict_count, local_overlap_total, shift)\n        if best_key is None or key < best_key:\n            best_key = key\n            best_train = candidate_train\n            best_option = candidate_option\n        if local_conflict_count == 0:\n            break\n    if best_train is None or best_option is None:\n        return False\n    if best_key is None or best_key >= current_key:\n        return False\n    current_by_id[train_id] = best_train\n    assignment[train_id] = best_option\n    delay_notes.setdefault(train_id, []).append(f"局部资源链小幅顺延+{best_key[2]}分")\n    return True\n\n\ndef unique_preserve_order(items: Iterable[str]) -> list[str]:\n    seen: set[str] = set()\n    ordered: list[str] = []\n    for item in items:\n        text = str(item).strip()\n        if not text or text in seen:\n            continue\n        seen.add(text)\n        ordered.append(text)\n    return ordered\n\n\ndef first_peak_load_row(\n    load_profile: list[dict[str, float]],\n) -> dict[str, float] | None:\n    if not load_profile:\n        return None\n    peak_value = max(float(row["负荷指数"]) for row in load_profile)\n    for row in load_profile:\n        if math.isclose(float(row["负荷指数"]), peak_value, rel_tol=1e-9, abs_tol=1e-9):\n            return row\n    return load_profile[0]\n\n\ndef build_disturbance_scenario(\n    trains: list[TrainRecord],\n    library: dict[str, object],\n    seed: int,\n    max_route_candidates: int,\n) -> tuple[\n    DisturbanceScenario, list[TrainRecord], dict[str, list[str]], list[dict[str, float]]\n]:\n    rng = random.Random(seed + 701)\n    resources_by_train = build_planned_resource_windows_by_train(trains)\n    window_start, window_end = disturbance_analysis_window(trains)\n    load_profile = build_disturbance_load_profile(\n        trains, resources_by_train, window_start, window_end\n    )\n    track_stats = build_track_tension_scores(\n        trains, resources_by_train, window_start, window_end\n    )\n    partition_stats = build_partition_tension_scores(\n        resources_by_train, window_start, window_end\n    )\n    track_score_map = {\n        track: float(stat.get("紧张度评分", 0.0)) for track, stat in track_stats.items()\n    }\n    partition_score_map = {\n        key: float(stat.get("紧张度评分", 0.0)) for key, stat in partition_stats.items()\n    }\n    events: list[DisturbanceEvent] = []\n    for selected_load in select_disturbance_load_rows(load_profile, rng):\n        event_minute = int(round(selected_load["分钟"]))\n        source_train_ids, source_train_scores = select_source_train_ids(\n            trains=trains,\n            resources_by_train=resources_by_train,\n            disturbance_minute=event_minute,\n            track_scores=track_score_map,\n            partition_scores=partition_score_map,\n            rng=rng,\n        )\n        source_train_delays = {\n            train_id: sample_source_delay_minutes(rng) for train_id in source_train_ids\n        }\n        spatial_targets = select_spatial_targets(track_stats, partition_stats, rng)\n        events.append(\n            DisturbanceEvent(\n                disturbance_minute=event_minute,\n                selected_train_load=int(round(selected_load["列车作业数"])),\n                selected_switch_load=int(round(selected_load["道岔组作业数"])),\n                selected_load_score=float(selected_load["负荷指数"]),\n                source_train_ids=source_train_ids,\n                source_train_delays=source_train_delays,\n                source_train_scores=source_train_scores,\n                spatial_targets=spatial_targets,\n            )\n        )\n    primary_event = max(events, key=lambda item: item.selected_load_score)\n    source_train_ids = tuple(\n        dict.fromkeys(\n            train_id for event in events for train_id in event.source_train_ids\n        )\n    )\n    source_train_delays: dict[str, int] = {}\n    source_train_scores: dict[str, float] = {}\n    spatial_targets = tuple(\n        target for event in events for target in event.spatial_targets\n    )\n    for event in events:\n        for train_id, delay in event.source_train_delays.items():\n            source_train_delays[train_id] = max(\n                source_train_delays.get(train_id, 0), delay\n            )\n        for train_id, score in event.source_train_scores.items():\n            source_train_scores[train_id] = max(\n                source_train_scores.get(train_id, 0.0), score\n            )\n    disturbed_trains, delay_notes, closure_blocks = apply_disturbance_events(\n        trains=trains,\n        resources_by_train=resources_by_train,\n        analysis_window_end=window_end,\n        events=tuple(events),\n    )\n    options_by_train, _ = build_candidate_route_plans(\n        trains=trains,\n        library=library,\n        max_route_candidates=max_route_candidates,\n    )\n    planned_assignment = build_plan_assignment(trains, options_by_train)\n    disturbed_assignment = rebuild_assignment_for_trains(\n        disturbed_trains, planned_assignment\n    )\n    disturbed_trains, _, propagation_rounds = propagate_planned_disturbance(\n        disturbed_trains,\n        disturbed_assignment,\n        closure_blocks,\n        delay_notes,\n        library,\n        max_route_candidates,\n        min(event.disturbance_minute for event in events),\n    )\n    normalized_delay_notes = {\n        train_id: unique_preserve_order(notes)\n        for train_id, notes in delay_notes.items()\n    }\n    scenario = DisturbanceScenario(\n        window_start_min=window_start,\n        window_end_min=window_end,\n        disturbance_minute=primary_event.disturbance_minute,\n        selected_train_load=primary_event.selected_train_load,\n        selected_switch_load=primary_event.selected_switch_load,\n        selected_load_score=primary_event.selected_load_score,\n        source_train_ids=source_train_ids,\n        source_train_delays=source_train_delays,\n        source_train_scores=source_train_scores,\n        spatial_targets=spatial_targets,\n        track_scores=track_score_map,\n        partition_scores=partition_score_map,\n        propagation_rounds=propagation_rounds,\n        disturbance_events=tuple(events),\n    )\n    return scenario, disturbed_trains, normalized_delay_notes, load_profile\n\n\ndef generate_disturbance_scenario(\n    trains: list[TrainRecord],\n    seed: int,\n) -> tuple[DisturbanceScenario, list[dict[str, float]]]:\n    rng = random.Random(seed + 701)\n    resources_by_train = build_planned_resource_windows_by_train(trains)\n    window_start, window_end = disturbance_analysis_window(trains)\n    load_profile = build_disturbance_load_profile(\n        trains, resources_by_train, window_start, window_end\n    )\n    track_stats = build_track_tension_scores(\n        trains, resources_by_train, window_start, window_end\n    )\n    partition_stats = build_partition_tension_scores(\n        resources_by_train, window_start, window_end\n    )\n    track_score_map = {\n        track: float(stat.get("紧张度评分", 0.0)) for track, stat in track_stats.items()\n    }\n    partition_score_map = {\n        key: float(stat.get("紧张度评分", 0.0)) for key, stat in partition_stats.items()\n    }\n    events: list[DisturbanceEvent] = []\n    for selected_load in select_disturbance_load_rows(load_profile, rng):\n        event_minute = int(round(selected_load["分钟"]))\n        source_train_ids, source_train_scores = select_source_train_ids(\n            trains=trains,\n            resources_by_train=resources_by_train,\n            disturbance_minute=event_minute,\n            track_scores=track_score_map,\n            partition_scores=partition_score_map,\n            rng=rng,\n        )\n        source_train_delays = {\n            train_id: sample_source_delay_minutes(rng) for train_id in source_train_ids\n        }\n        spatial_targets = select_spatial_targets(track_stats, partition_stats, rng)\n        events.append(\n            DisturbanceEvent(\n                disturbance_minute=event_minute,\n                selected_train_load=int(round(selected_load["列车作业数"])),\n                selected_switch_load=int(round(selected_load["道岔组作业数"])),\n                selected_load_score=float(selected_load["负荷指数"]),\n                source_train_ids=source_train_ids,\n                source_train_delays=source_train_delays,\n                source_train_scores=source_train_scores,\n                spatial_targets=spatial_targets,\n            )\n        )\n    primary_event = max(events, key=lambda item: item.selected_load_score)\n    merged_source_ids = tuple(\n        dict.fromkeys(\n            train_id for event in events for train_id in event.source_train_ids\n        )\n    )\n    merged_delays: dict[str, int] = {}\n    merged_scores: dict[str, float] = {}\n    merged_targets = tuple(\n        target for event in events for target in event.spatial_targets\n    )\n    for event in events:\n        for train_id, delay in event.source_train_delays.items():\n            merged_delays[train_id] = max(merged_delays.get(train_id, 0), delay)\n        for train_id, score in event.source_train_scores.items():\n            merged_scores[train_id] = max(merged_scores.get(train_id, 0.0), score)\n    scenario = DisturbanceScenario(\n        window_start_min=window_start,\n        window_end_min=window_end,\n        disturbance_minute=primary_event.disturbance_minute,\n        selected_train_load=primary_event.selected_train_load,\n        selected_switch_load=primary_event.selected_switch_load,\n        selected_load_score=primary_event.selected_load_score,\n        source_train_ids=merged_source_ids,\n        source_train_delays=merged_delays,\n        source_train_scores=merged_scores,\n        spatial_targets=merged_targets,\n        track_scores=track_score_map,\n        partition_scores=partition_score_map,\n        propagation_rounds=0,\n        disturbance_events=tuple(events),\n    )\n    return scenario, load_profile\n\n\ndef scenario_with_propagation_rounds(\n    scenario: DisturbanceScenario,\n    propagation_rounds: int,\n) -> DisturbanceScenario:\n    return DisturbanceScenario(\n        window_start_min=scenario.window_start_min,\n        window_end_min=scenario.window_end_min,\n        disturbance_minute=scenario.disturbance_minute,\n        selected_train_load=scenario.selected_train_load,\n        selected_switch_load=scenario.selected_switch_load,\n        selected_load_score=scenario.selected_load_score,\n        source_train_ids=scenario.source_train_ids,\n        source_train_delays=scenario.source_train_delays,\n        source_train_scores=scenario.source_train_scores,\n        spatial_targets=scenario.spatial_targets,\n        track_scores=scenario.track_scores,\n        partition_scores=scenario.partition_scores,\n        propagation_rounds=propagation_rounds,\n        disturbance_events=scenario.disturbance_events,\n    )\n\n\ndef compute_delay_statistics(\n    reference_trains: list[TrainRecord],\n    current_trains: list[TrainRecord],\n) -> dict[str, object]:\n    reference_map = {train.record_id: train for train in reference_trains}\n    delay_by_train: dict[str, int] = {}\n    positive_delays: list[int] = []\n    for train in current_trains:\n        reference = reference_map[train.record_id]\n        delay = max(\n            0,\n            train.arrival_min - reference.arrival_min,\n            train.departure_min - reference.departure_min,\n        )\n        delay_by_train[train.record_id] = delay\n        if delay > 0:\n            positive_delays.append(delay)\n    total_delay = sum(positive_delays)\n    return {\n        "delay_by_train": delay_by_train,\n        "总晚点时长": total_delay,\n        "平均晚点时长": safe_ratio(total_delay, len(positive_delays)),\n        "最大晚点时长": max(positive_delays, default=0),\n        "受影响列车数": len(positive_delays),\n        "高晚点列车数": sum(1 for delay in positive_delays if delay >= 15),\n    }\n\n\nHARD_LOCK_CONFLICT_TYPES = frozenset({"接车进路锁闭冲突", "发车进路锁闭冲突"})\n\n\ndef build_hard_conflict_breakdown(conflicts: list[ConflictEntry]) -> dict[str, int]:\n    hard_total = 0\n    track_hard = 0\n    route_lock_hard = 0\n    for conflict in conflicts:\n        if conflict.conflict_level != "硬冲突":\n            continue\n        hard_total += 1\n        if conflict.conflict_type == "股道占用冲突":\n            track_hard += 1\n        if conflict.conflict_type in HARD_LOCK_CONFLICT_TYPES:\n            route_lock_hard += 1\n    return {\n        "硬冲突项": hard_total,\n        "股道占用硬冲突项": track_hard,\n        "进路锁闭硬冲突项": route_lock_hard,\n    }\n\n\ndef is_strict_safe_conflicts(conflicts: list[ConflictEntry]) -> bool:\n    stats = build_hard_conflict_breakdown(conflicts)\n    return (\n        stats["硬冲突项"] == 0\n        and stats["股道占用硬冲突项"] == 0\n        and stats["进路锁闭硬冲突项"] == 0\n    )\n\n\ndef merge_note_maps(*note_maps: dict[str, list[str]]) -> dict[str, list[str]]:\n    merged: dict[str, list[str]] = {}\n    for note_map in note_maps:\n        for train_id, notes in note_map.items():\n            merged.setdefault(train_id, []).extend(notes)\n    return {\n        train_id: unique_preserve_order(notes) for train_id, notes in merged.items()\n    }\n\n\ndef assignment_digest(\n    assignment: dict[str, CandidateRoutePlan],\n) -> tuple[tuple[str, tuple[str, str, str, str, str, str]], ...]:\n    return tuple(\n        sorted(\n            (\n                train_id,\n                assignment_option_signature(option),\n            )\n            for train_id, option in assignment.items()\n        )\n    )\n\n\ndef assignment_option_signature(\n    option: CandidateRoutePlan,\n) -> tuple[str, str, str, str, str, str]:\n    return (\n        option.track,\n        option.in_throat,\n        option.out_throat,\n        option.inbound_channel,\n        option.outbound_channel,\n        option.route_variant,\n    )\n\n\ndef project_assignment_to_option_ids(\n    trains: list[TrainRecord],\n    options_by_train: dict[str, list[CandidateRoutePlan]],\n    assignment: dict[str, CandidateRoutePlan] | None,\n) -> dict[str, str]:\n    selected: dict[str, str] = {}\n    for train in trains:\n        options = options_by_train[train.record_id]\n        matched_option_id: str | None = None\n        previous = assignment.get(train.record_id) if assignment is not None else None\n        if previous is not None:\n            previous_signature = assignment_option_signature(previous)\n            matched_option_id = next(\n                (\n                    option.option_id\n                    for option in options\n                    if assignment_option_signature(option) == previous_signature\n                ),\n                None,\n            )\n            if matched_option_id is None:\n                matched_option_id = next(\n                    (\n                        option.option_id\n                        for option in options\n                        if (option.track, option.in_throat, option.out_throat)\n                        == (previous.track, previous.in_throat, previous.out_throat)\n                    ),\n                    None,\n                )\n        if matched_option_id is None:\n            matched_option_id = next(\n                (\n                    option.option_id\n                    for option in options\n                    if option.track == train.planned_track\n                    and option.in_throat == train.in_throat\n                    and option.out_throat == train.out_throat\n                ),\n                options[0].option_id,\n            )\n        selected[train.record_id] = matched_option_id\n    return selected\n\n\ndef build_delay_priority_train_order(\n    trains: list[TrainRecord],\n    options_by_train: dict[str, list[CandidateRoutePlan]],\n    delay_by_train: dict[str, int],\n) -> list[str]:\n    overlap_count = {train.record_id: 0 for train in trains}\n    for left_index, train_a in enumerate(trains):\n        for right_index in range(left_index + 1, len(trains)):\n            train_b = trains[right_index]\n            if (\n                overlap_minutes(\n                    train_a.window_start,\n                    train_a.window_end,\n                    train_b.window_start,\n                    train_b.window_end,\n                )\n                > 0\n            ):\n                overlap_count[train_a.record_id] += 1\n                overlap_count[train_b.record_id] += 1\n    ordered = sorted(\n        trains,\n        key=lambda train: (\n            -delay_by_train.get(train.record_id, 0),\n            -overlap_count[train.record_id],\n            len(options_by_train[train.record_id]),\n            train.arrival_min,\n            train.index,\n        ),\n    )\n    return [train.record_id for train in ordered]\n\n\ndef build_propagation_criticality_scores(\n    trains: list[TrainRecord],\n    delay_by_train: dict[str, int],\n    library: dict[str, object],\n) -> dict[str, float]:\n    scores: dict[str, float] = {}\n    for train in trains:\n        later_impacted = sum(\n            1\n            for other in trains\n            if other.record_id != train.record_id\n            and other.window_start >= train.window_start\n            and overlap_minutes(\n                train.window_start,\n                train.window_end + 25,\n                other.window_start,\n                other.window_end,\n            )\n            > 0\n        )\n        bottleneck_score = library["throat_usage_counts"].get(\n            train.in_throat, 0\n        ) + library["throat_usage_counts"].get(train.out_throat, 0)\n        partition_score = 0.0\n        if train.in_throat:\n            partition_score += max(\n                0,\n                6\n                - THROAT_PARTITION_ORDER.get(train.in_throat, ()).index(\n                    throat_partition(train.planned_track, train.in_throat)\n                )\n                if throat_partition(train.planned_track, train.in_throat)\n                in THROAT_PARTITION_ORDER.get(train.in_throat, ())\n                else 0,\n            )\n        if train.out_throat:\n            partition_score += max(\n                0,\n                6\n                - THROAT_PARTITION_ORDER.get(train.out_throat, ()).index(\n                    throat_partition(train.planned_track, train.out_throat)\n                )\n                if throat_partition(train.planned_track, train.out_throat)\n                in THROAT_PARTITION_ORDER.get(train.out_throat, ())\n                else 0,\n            )\n        overlap_score = sum(\n            1\n            for other in trains\n            if other.record_id != train.record_id\n            and overlap_minutes(\n                train.window_start,\n                train.window_end,\n                other.window_start,\n                other.window_end,\n            )\n            > 0\n        )\n        delay_score = delay_by_train.get(train.record_id, 0)\n        scores[train.record_id] = (\n            later_impacted * 5.0\n            + bottleneck_score * 1.8\n            + delay_score * 1.2\n            + overlap_score * 0.8\n            + partition_score * 0.5\n        )\n    return scores\n\n\ndef build_recovery_focus_clusters(\n    trains: list[TrainRecord],\n    delay_by_train: dict[str, int],\n    criticality_scores: dict[str, float] | None = None,\n) -> list[list[str]]:\n    positive_trains = [\n        train for train in trains if delay_by_train.get(train.record_id, 0) > 0\n    ]\n    if not positive_trains:\n        return []\n\n    seed_trains = [\n        train\n        for train in positive_trains\n        if delay_by_train.get(train.record_id, 0) >= RECOVERY_LONG_WAIT_THRESHOLD\n    ]\n    if not seed_trains:\n        seed_trains = sorted(\n            positive_trains,\n            key=lambda train: (\n                -(criticality_scores or {}).get(train.record_id, 0.0),\n                -delay_by_train.get(train.record_id, 0),\n                train.index,\n            ),\n        )[:6]\n\n    clusters: list[list[str]] = []\n    assigned: set[str] = set()\n    for seed in sorted(\n        seed_trains,\n        key=lambda train: (\n            -(criticality_scores or {}).get(train.record_id, 0.0),\n            -delay_by_train.get(train.record_id, 0),\n            train.index,\n        ),\n    ):\n        if seed.record_id in assigned:\n            continue\n        cluster_members = [\n            train\n            for train in trains\n            if (\n                overlap_minutes(\n                    seed.window_start - 15,\n                    seed.window_end + 15,\n                    train.window_start,\n                    train.window_end,\n                )\n                > 0\n                or min(\n                    abs(seed.arrival_min - train.arrival_min),\n                    abs(seed.departure_min - train.departure_min),\n                )\n                <= 20\n                or seed.in_throat == train.in_throat\n                or seed.out_throat == train.out_throat\n            )\n        ]\n        cluster_members = sorted(\n            cluster_members,\n            key=lambda train: (\n                -(criticality_scores or {}).get(train.record_id, 0.0),\n                -delay_by_train.get(train.record_id, 0),\n                train.arrival_min,\n                train.index,\n            ),\n        )[:12]\n        cluster_ids = [train.record_id for train in cluster_members]\n        for train_id in cluster_ids:\n            assigned.add(train_id)\n        clusters.append(cluster_ids)\n    return clusters\n\n\ndef optimize_train_subset(\n    selected_option_ids: dict[str, str],\n    target_train_ids: list[str],\n    options_by_train: dict[str, list[CandidateRoutePlan]],\n    linear_costs: dict[str, float],\n    pair_costs: dict[str, dict[str, float]],\n    hard_counts: dict[str, dict[str, int]],\n    hard_risks: dict[str, dict[str, float]],\n    train_map: dict[str, TrainRecord],\n    delay_by_train: dict[str, int],\n    priority_scores: dict[str, float] | None = None,\n    passes: int = 3,\n) -> dict[str, str]:\n    ordered_targets = sorted(\n        target_train_ids,\n        key=lambda train_id: (\n            -(priority_scores or {}).get(train_id, 0.0),\n            -delay_by_train.get(train_id, 0),\n            train_map[train_id].arrival_min,\n            train_map[train_id].index,\n        ),\n    )\n    for _ in range(max(1, passes)):\n        changed = False\n        for train_id in ordered_targets:\n            others = {\n                other_train_id: option_id\n                for other_train_id, option_id in selected_option_ids.items()\n                if other_train_id != train_id\n            }\n            current_option_id = selected_option_ids[train_id]\n            best_option_id = current_option_id\n            best_cost = compute_option_cost(\n                train_id,\n                current_option_id,\n                others,\n                linear_costs,\n                pair_costs,\n                hard_counts,\n                hard_risks,\n            )\n            for option in options_by_train[train_id]:\n                option_id = option.option_id\n                if any(\n                    hard_counts.get(option_id, {}).get(other_option_id, 0) > 0\n                    for other_option_id in others.values()\n                ):\n                    continue\n                option_cost = compute_option_cost(\n                    train_id,\n                    option_id,\n                    others,\n                    linear_costs,\n                    pair_costs,\n                    hard_counts,\n                    hard_risks,\n                )\n                if option_cost + 1e-9 < best_cost:\n                    best_cost = option_cost\n                    best_option_id = option_id\n            if best_option_id != current_option_id:\n                selected_option_ids[train_id] = best_option_id\n                changed = True\n        if not changed:\n            break\n    return selected_option_ids\n\n\ndef build_delay_aware_linear_costs(\n    reference_trains: list[TrainRecord],\n    options_by_train: dict[str, list[CandidateRoutePlan]],\n    delay_by_train: dict[str, int],\n) -> dict[str, float]:\n    reference_map = {train.record_id: train for train in reference_trains}\n    linear_costs: dict[str, float] = {}\n    for train_id, options in options_by_train.items():\n        delay = delay_by_train.get(train_id, 0)\n        stability_factor = max(0.12, 1.0 - min(delay, 24) / 28.0)\n        reference_train = reference_map[train_id]\n        for option in options:\n            residual = (\n                option.linear_cost\n                - option.delay_risk_cost * 6.0\n                - option.stability_cost * 2.4\n                + option.balance_reward\n            )\n            plan_unchanged = (\n                option.track == reference_train.planned_track\n                and option.in_throat == reference_train.in_throat\n                and option.out_throat == reference_train.out_throat\n            )\n            change_reward = 0.0\n            hold_penalty = 0.0\n            if delay > 0 and not plan_unchanged:\n                change_reward = min(75.0, delay * 4.0)\n            if delay > 0 and plan_unchanged:\n                hold_penalty = min(120.0, delay * 6.0)\n            stability_component = (\n                option.stability_cost\n                * 2.4\n                * DEFAULT_MODEL_WEIGHTS["stability_weight"]\n                * stability_factor\n            )\n            delay_risk_multiplier = 6.0\n            if RECOVERY_AGGRESSIVE_MODE and delay > 0:\n                delay_risk_multiplier = 2.2\n            if RECOVERY_DISABLE_STABILITY_COST and delay > 0:\n                stability_component = 0.0\n            linear_costs[option.option_id] = max(\n                0.0,\n                option.delay_risk_cost\n                * delay_risk_multiplier\n                * DEFAULT_MODEL_WEIGHTS["safety_weight"]\n                + stability_component\n                - option.balance_reward\n                + residual\n                + hold_penalty\n                - change_reward,\n            )\n    return linear_costs\n\n\ndef build_delay_aware_pair_costs(\n    pair_costs: dict[str, dict[str, float]],\n    options_by_train: dict[str, list[CandidateRoutePlan]],\n    delay_by_train: dict[str, int],\n) -> dict[str, dict[str, float]]:\n    option_to_train = {\n        option.option_id: train_id\n        for train_id, options in options_by_train.items()\n        for option in options\n    }\n    max_delay = max(delay_by_train.values(), default=0)\n    if max_delay <= 0:\n        return pair_costs\n    adjusted: dict[str, dict[str, float]] = {}\n    for option_id, neighbors in pair_costs.items():\n        adjusted_neighbors: dict[str, float] = {}\n        for other_id, value in neighbors.items():\n            delay_factor = max(\n                delay_by_train.get(option_to_train[option_id], 0),\n                delay_by_train.get(option_to_train[other_id], 0),\n            )\n            multiplier = 1.0 + 1.2 * safe_ratio(delay_factor, max_delay)\n            adjusted_neighbors[other_id] = value * multiplier\n        adjusted[option_id] = adjusted_neighbors\n    return adjusted\n\n\ndef build_assignment_change_notes(\n    previous_assignment: dict[str, CandidateRoutePlan] | None,\n    current_assignment: dict[str, CandidateRoutePlan],\n) -> dict[str, list[str]]:\n    notes: dict[str, list[str]] = {}\n    if previous_assignment is None:\n        return notes\n    for train_id, option in current_assignment.items():\n        previous = previous_assignment.get(train_id)\n        if previous is None:\n            continue\n        if assignment_option_signature(previous) == assignment_option_signature(option):\n            continue\n        notes[train_id] = [\n            (\n                f"迭代重排进站:股道{previous.track}->{option.track};"\n                f"咽喉{previous.in_throat}/{previous.out_throat}->{option.in_throat}/{option.out_throat}"\n            )\n        ]\n    return notes\n\n\ndef second_dispatch_high_delay_trains(\n    current_trains: list[TrainRecord],\n    delay_by_train: dict[str, int],\n    selected_option_ids: dict[str, str],\n    options_by_train: dict[str, list[CandidateRoutePlan]],\n    linear_costs: dict[str, float],\n    pair_costs: dict[str, dict[str, float]],\n    hard_counts: dict[str, dict[str, int]],\n    hard_risks: dict[str, dict[str, float]],\n) -> dict[str, str]:\n    high_delay_train_ids = sorted(\n        [\n            train.record_id\n            for train in current_trains\n            if delay_by_train.get(train.record_id, 0) >= 15\n        ],\n        key=lambda train_id: (\n            -delay_by_train.get(train_id, 0),\n            train_id,\n        ),\n    )\n    for train_id in high_delay_train_ids:\n        others = {\n            other_train_id: option_id\n            for other_train_id, option_id in selected_option_ids.items()\n            if other_train_id != train_id\n        }\n        current_option_id = selected_option_ids[train_id]\n        best_option_id = current_option_id\n        best_cost = compute_option_cost(\n            train_id,\n            current_option_id,\n            others,\n            linear_costs,\n            pair_costs,\n            hard_counts,\n            hard_risks,\n        )\n        for option in options_by_train[train_id]:\n            option_id = option.option_id\n            if any(\n                hard_counts.get(option_id, {}).get(other_option_id, 0) > 0\n                for other_option_id in others.values()\n            ):\n                continue\n            option_cost = compute_option_cost(\n                train_id,\n                option_id,\n                others,\n                linear_costs,\n                pair_costs,\n                hard_counts,\n                hard_risks,\n            )\n            if option_cost + 1e-9 < best_cost:\n                best_cost = option_cost\n                best_option_id = option_id\n        selected_option_ids[train_id] = best_option_id\n    return selected_option_ids\n\n\ndef delay_absorption_local_search(\n    selected_option_ids: dict[str, str],\n    options_by_train: dict[str, list[CandidateRoutePlan]],\n    linear_costs: dict[str, float],\n    pair_costs: dict[str, dict[str, float]],\n    hard_counts: dict[str, dict[str, int]],\n    hard_risks: dict[str, dict[str, float]],\n    train_map: dict[str, TrainRecord],\n    delay_by_train: dict[str, int],\n    priority_scores: dict[str, float] | None = None,\n    passes: int = RECOVERY_DELAY_ABSORPTION_PASSES,\n) -> dict[str, str]:\n    target_train_ids = sorted(\n        [\n            train_id\n            for train_id, delay in delay_by_train.items()\n            if delay >= RECOVERY_DELAY_ABSORPTION_THRESHOLD\n            and train_id in selected_option_ids\n        ],\n        key=lambda train_id: (\n            -delay_by_train.get(train_id, 0),\n            -(priority_scores or {}).get(train_id, 0.0),\n            train_map[train_id].arrival_min,\n            train_map[train_id].index,\n        ),\n    )\n    if not target_train_ids:\n        return selected_option_ids\n\n    option_lookup = {\n        option.option_id: option\n        for options in options_by_train.values()\n        for option in options\n    }\n\n    for _ in range(max(1, passes)):\n        changed = False\n        for train_id in target_train_ids:\n            others = {\n                other_train_id: option_id\n                for other_train_id, option_id in selected_option_ids.items()\n                if other_train_id != train_id\n            }\n            train = train_map[train_id]\n            current_option_id = selected_option_ids[train_id]\n            current_option = option_lookup[current_option_id]\n            best_option_id = current_option_id\n            best_signature = (\n                compute_option_cost(\n                    train_id,\n                    current_option_id,\n                    others,\n                    linear_costs,\n                    pair_costs,\n                    hard_counts,\n                    hard_risks,\n                ),\n                current_option.track == train.planned_track\n                and current_option.in_throat == train.in_throat\n                and current_option.out_throat == train.out_throat,\n                current_option.delay_risk_cost,\n                current_option.option_id,\n            )\n            for option in options_by_train[train_id]:\n                option_id = option.option_id\n                if option_id == current_option_id:\n                    continue\n                if any(\n                    hard_counts.get(option_id, {}).get(other_option_id, 0) > 0\n                    for other_option_id in others.values()\n                ):\n                    continue\n                signature = (\n                    compute_option_cost(\n                        train_id,\n                        option_id,\n                        others,\n                        linear_costs,\n                        pair_costs,\n                        hard_counts,\n                        hard_risks,\n                    ),\n                    option.track == train.planned_track\n                    and option.in_throat == train.in_throat\n                    and option.out_throat == train.out_throat,\n                    option.delay_risk_cost,\n                    option.option_id,\n                )\n                if signature < best_signature:\n                    best_signature = signature\n                    best_option_id = option_id\n            if best_option_id != current_option_id:\n                selected_option_ids[train_id] = best_option_id\n                changed = True\n        if not changed:\n            break\n    return selected_option_ids\n\n\ndef optimize_recovery_assignment(\n    reference_trains: list[TrainRecord],\n    current_trains: list[TrainRecord],\n    library: dict[str, object],\n    current_assignment: dict[str, CandidateRoutePlan] | None,\n    prioritize_delay: bool,\n    max_route_candidates: int,\n) -> tuple[\n    dict[str, CandidateRoutePlan],\n    dict[str, list[CandidateRoutePlan]],\n    dict[str, list[str]],\n]:\n    delay_stats = compute_delay_statistics(reference_trains, current_trains)\n    delay_by_train = delay_stats["delay_by_train"]\n    criticality_scores = (\n        build_propagation_criticality_scores(current_trains, delay_by_train, library)\n        if prioritize_delay\n        else {}\n    )\n    focus_clusters = (\n        build_recovery_focus_clusters(\n            current_trains, delay_by_train, criticality_scores=criticality_scores\n        )\n        if prioritize_delay\n        else []\n    )\n    long_wait_train_ids = {\n        train_id\n        for train_id, delay in delay_by_train.items()\n        if delay >= RECOVERY_LONG_WAIT_THRESHOLD\n    }\n    delayed_train_ids = {\n        train_id\n        for train_id, delay in delay_by_train.items()\n        if delay >= RECOVERY_DELAY_ABSORPTION_THRESHOLD\n    }\n    rich_candidate_train_ids = (\n        (\n            {train_id for cluster in focus_clusters for train_id in cluster}\n            | long_wait_train_ids\n            | delayed_train_ids\n        )\n        if prioritize_delay\n        else {train.record_id for train in current_trains}\n    )\n    candidate_timing_trains = (\n        reference_trains\n        if (prioritize_delay and RECOVERY_AGGRESSIVE_MODE)\n        else current_trains\n    )\n    recovery_candidate_limit = max_route_candidates\n    if prioritize_delay:\n        recovery_candidate_limit = max(\n            max_route_candidates, RECOVERY_AGGRESSIVE_OPTION_LIMIT\n        )\n    options_by_train, train_map = build_candidate_route_plans(\n        trains=candidate_timing_trains,\n        library=library,\n        max_route_candidates=recovery_candidate_limit,\n        rich_candidate_train_ids=rich_candidate_train_ids,\n        recovery_aggressive_train_ids=delayed_train_ids if prioritize_delay else None,\n    )\n    raw_pair_costs, hard_counts, hard_risks = build_pairwise_conflict_maps(\n        trains=candidate_timing_trains,\n        options_by_train=options_by_train,\n        mode="full",\n        hard_soft_enabled=True,\n        use_variant_penalty=False,\n    )\n    if prioritize_delay:\n        linear_costs = build_delay_aware_linear_costs(\n            reference_trains, options_by_train, delay_by_train\n        )\n        pair_costs = build_delay_aware_pair_costs(\n            raw_pair_costs, options_by_train, delay_by_train\n        )\n        train_order = sorted(\n            [train.record_id for train in candidate_timing_trains],\n            key=lambda train_id: (\n                -criticality_scores.get(train_id, 0.0),\n                -delay_by_train.get(train_id, 0),\n                train_map[train_id].arrival_min,\n                train_map[train_id].index,\n            ),\n        )\n    else:\n        linear_costs = build_weighted_linear_costs(\n            options_by_train,\n            DEFAULT_MODEL_WEIGHTS["safety_weight"],\n            DEFAULT_MODEL_WEIGHTS["stability_weight"],\n        )\n        pair_costs = build_weighted_pair_costs(\n            raw_pair_costs, DEFAULT_MODEL_WEIGHTS["safety_weight"]\n        )\n        train_order = build_train_order(candidate_timing_trains, options_by_train)\n    selected_option_ids = project_assignment_to_option_ids(\n        candidate_timing_trains, options_by_train, current_assignment\n    )\n    if prioritize_delay and focus_clusters:\n        for cluster in focus_clusters:\n            selected_option_ids = optimize_train_subset(\n                selected_option_ids,\n                cluster,\n                options_by_train,\n                linear_costs,\n                pair_costs,\n                hard_counts,\n                hard_risks,\n                train_map,\n                delay_by_train,\n                priority_scores=criticality_scores,\n                passes=4,\n            )\n        selected_option_ids = greedy_descent(\n            selected_option_ids,\n            train_order,\n            options_by_train,\n            linear_costs,\n            pair_costs,\n            hard_counts,\n            hard_risks,\n            train_map,\n            max_passes=3,\n            prefer_reassignment_train_ids=delayed_train_ids,\n        )\n    else:\n        selected_option_ids = greedy_descent(\n            selected_option_ids,\n            train_order,\n            options_by_train,\n            linear_costs,\n            pair_costs,\n            hard_counts,\n            hard_risks,\n            train_map,\n            max_passes=8,\n        )\n    if prioritize_delay:\n        selected_option_ids = delay_absorption_local_search(\n            selected_option_ids,\n            options_by_train,\n            linear_costs,\n            pair_costs,\n            hard_counts,\n            hard_risks,\n            train_map,\n            delay_by_train,\n            priority_scores=criticality_scores,\n        )\n    selected_option_ids = conflict_driven_joint_local_search(\n        selected_option_ids,\n        options_by_train,\n        linear_costs,\n        pair_costs,\n        hard_counts,\n        hard_risks,\n        train_map,\n        max_rounds=6 if prioritize_delay else 4,\n        prefer_reassignment_train_ids=delayed_train_ids if prioritize_delay else None,\n    )\n    selected_option_ids = repair_hard_conflicts(\n        trains=candidate_timing_trains,\n        train_map=train_map,\n        options_by_train=options_by_train,\n        selected_option_ids=selected_option_ids,\n        linear_costs=linear_costs,\n        pair_costs=pair_costs,\n        hard_counts=hard_counts,\n        hard_risks=hard_risks,\n    )\n    if prioritize_delay:\n        selected_option_ids = delay_absorption_local_search(\n            selected_option_ids,\n            options_by_train,\n            linear_costs,\n            pair_costs,\n            hard_counts,\n            hard_risks,\n            train_map,\n            delay_by_train,\n            priority_scores=criticality_scores,\n            passes=2,\n        )\n        for cluster in focus_clusters[:4]:\n            selected_option_ids = optimize_train_subset(\n                selected_option_ids,\n                cluster,\n                options_by_train,\n                linear_costs,\n                pair_costs,\n                hard_counts,\n                hard_risks,\n                train_map,\n                delay_by_train,\n                priority_scores=criticality_scores,\n                passes=2,\n            )\n        selected_option_ids = repair_hard_conflicts(\n            trains=candidate_timing_trains,\n            train_map=train_map,\n            options_by_train=options_by_train,\n            selected_option_ids=selected_option_ids,\n            linear_costs=linear_costs,\n            pair_costs=pair_costs,\n            hard_counts=hard_counts,\n            hard_risks=hard_risks,\n        )\n        selected_option_ids = delay_absorption_local_search(\n            selected_option_ids,\n            options_by_train,\n            linear_costs,\n            pair_costs,\n            hard_counts,\n            hard_risks,\n            train_map,\n            delay_by_train,\n            priority_scores=criticality_scores,\n            passes=2,\n        )\n    assignment = build_assignment_lookup(selected_option_ids, options_by_train)\n    return (\n        assignment,\n        options_by_train,\n        build_assignment_change_notes(current_assignment, assignment),\n    )\n\n\ndef enforce_strict_safety_on_realized_trains(\n    realized_trains: list[TrainRecord],\n    library: dict[str, object],\n    current_assignment: dict[str, CandidateRoutePlan],\n    max_route_candidates: int,\n    disturbance_minute: int | None = None,\n    reference_trains: list[TrainRecord] | None = None,\n    max_safety_wait_rounds: int = 160,\n) -> tuple[\n    list[TrainRecord],\n    dict[str, CandidateRoutePlan],\n    list[ConflictEntry],\n    dict[str, list[str]],\n]:\n    current_trains = list(realized_trains)\n    reference_map = {\n        train.record_id: train for train in (reference_trains or realized_trains)\n    }\n    options_by_train, train_map = build_candidate_route_plans(\n        trains=current_trains,\n        library=library,\n        max_route_candidates=max_route_candidates,\n    )\n    raw_pair_costs, hard_counts, hard_risks = build_pairwise_conflict_maps(\n        trains=current_trains,\n        options_by_train=options_by_train,\n        mode="full",\n        hard_soft_enabled=True,\n        use_variant_penalty=False,\n    )\n    linear_costs = build_weighted_linear_costs(\n        options_by_train,\n        DEFAULT_MODEL_WEIGHTS["safety_weight"],\n        DEFAULT_MODEL_WEIGHTS["stability_weight"],\n    )\n    pair_costs = build_weighted_pair_costs(\n        raw_pair_costs, DEFAULT_MODEL_WEIGHTS["safety_weight"]\n    )\n    selected_option_ids = project_assignment_to_option_ids(\n        current_trains, options_by_train, current_assignment\n    )\n    train_order = build_train_order(current_trains, options_by_train)\n    selected_option_ids = greedy_descent(\n        selected_option_ids,\n        train_order,\n        options_by_train,\n        linear_costs,\n        pair_costs,\n        hard_counts,\n        hard_risks,\n        train_map,\n        max_passes=6,\n    )\n    selected_option_ids = conflict_driven_joint_local_search(\n        selected_option_ids,\n        options_by_train,\n        linear_costs,\n        pair_costs,\n        hard_counts,\n        hard_risks,\n        train_map,\n        max_rounds=6,\n    )\n    selected_option_ids = repair_hard_conflicts(\n        trains=current_trains,\n        train_map=train_map,\n        options_by_train=options_by_train,\n        selected_option_ids=selected_option_ids,\n        linear_costs=linear_costs,\n        pair_costs=pair_costs,\n        hard_counts=hard_counts,\n        hard_risks=hard_risks,\n    )\n    assignment = build_assignment_lookup(selected_option_ids, options_by_train)\n    notes = build_assignment_change_notes(current_assignment, assignment)\n\n    current_by_id = {train.record_id: train for train in current_trains}\n\n    def causal_conflicts(conflicts: list[ConflictEntry]) -> list[ConflictEntry]:\n        if disturbance_minute is None:\n            return conflicts\n        return [\n            conflict\n            for conflict in conflicts\n            if current_by_id[conflict.train1.split("-", 1)[0]].window_end\n            > disturbance_minute\n            or current_by_id[conflict.train2.split("-", 1)[0]].window_end\n            > disturbance_minute\n        ]\n\n    def timetable_objective(\n        candidate_by_id: dict[str, TrainRecord],\n    ) -> tuple[int, int, int, int]:\n        delays: list[int] = []\n        for train_id, train in candidate_by_id.items():\n            reference = reference_map.get(train_id)\n            if reference is None:\n                continue\n            delay = max(\n                0,\n                train.arrival_min - reference.arrival_min,\n                train.departure_min - reference.departure_min,\n            )\n            if delay > 0:\n                delays.append(delay)\n        return (\n            sum(delays),\n            max(delays, default=0),\n            sum(1 for delay in delays if delay >= 15),\n            len(delays),\n        )\n\n    def choose_delay_action(\n        train1_id: str, train2_id: str, base_shift: int\n    ) -> tuple[str, int] | None:\n        candidates: list[tuple[str, int]] = []\n        for train_id, other_id in ((train1_id, train2_id), (train2_id, train1_id)):\n            train = current_by_id[train_id]\n            if (\n                disturbance_minute is not None\n                and train.window_end <= disturbance_minute\n            ):\n                continue\n            other = current_by_id[other_id]\n            required_shift = base_shift\n            if train.window_start <= other.window_start:\n                required_shift = max(\n                    required_shift, other.window_end - train.window_start + 1\n                )\n            candidates.append((train_id, required_shift))\n        if not candidates:\n            return None\n        scored: list[tuple[tuple[int, int, int, int], int, str, int]] = []\n        for train_id, shift in candidates:\n            candidate_by_id = dict(current_by_id)\n            candidate_by_id[train_id] = rebuild_train_record(\n                candidate_by_id[train_id],\n                arrival_shift=shift,\n                departure_shift=shift,\n            )\n            scored.append(\n                (\n                    timetable_objective(candidate_by_id),\n                    current_by_id[train_id].index,\n                    train_id,\n                    shift,\n                )\n            )\n        _, _, train_id, shift = min(scored, key=lambda item: (item[0], item[1]))\n        return train_id, shift\n\n    for _ in range(max_safety_wait_rounds):\n        current_trains = [\n            current_by_id[train.record_id]\n            for train in sorted(current_by_id.values(), key=lambda item: item.index)\n        ]\n        assignment = rebuild_assignment_for_trains(current_trains, assignment)\n        conflicts = causal_conflicts(\n            collect_conflicts(current_trains, assignment, "扰动优化")\n        )\n        if is_strict_safe_conflicts(conflicts):\n            return current_trains, assignment, conflicts, notes\n\n        hard_conflicts = [\n            conflict for conflict in conflicts if conflict.conflict_level == "硬冲突"\n        ]\n        if disturbance_minute is not None:\n            hard_conflicts = [\n                conflict\n                for conflict in hard_conflicts\n                if current_by_id[conflict.train1.split("-", 1)[0]].window_end\n                > disturbance_minute\n                or current_by_id[conflict.train2.split("-", 1)[0]].window_end\n                > disturbance_minute\n            ]\n        if not hard_conflicts:\n            return current_trains, assignment, conflicts, notes\n\n        shift_by_train: defaultdict[str, int] = defaultdict(int)\n        for target in hard_conflicts:\n            train1_id = target.train1.split("-", 1)[0]\n            train2_id = target.train2.split("-", 1)[0]\n            shift = max(1, target.overlap_min + 1)\n            delay_action = choose_delay_action(train1_id, train2_id, shift)\n            if delay_action is None:\n                continue\n            delay_train_id, shift = delay_action\n            shift_by_train[delay_train_id] = max(\n                shift_by_train[delay_train_id],\n                shift,\n            )\n\n        if not shift_by_train:\n            return current_trains, assignment, conflicts, notes\n\n        for delay_train_id, shift in shift_by_train.items():\n            current_by_id[delay_train_id] = rebuild_train_record(\n                current_by_id[delay_train_id],\n                arrival_shift=shift,\n                departure_shift=shift,\n            )\n            notes.setdefault(delay_train_id, []).append(f"安全等待+{shift}分")\n\n    final_trains = [\n        current_by_id[train.record_id]\n        for train in sorted(current_by_id.values(), key=lambda item: item.index)\n    ]\n    assignment = rebuild_assignment_for_trains(final_trains, assignment)\n    conflicts = causal_conflicts(\n        collect_conflicts(final_trains, assignment, "扰动优化")\n    )\n    return (\n        final_trains,\n        assignment,\n        conflicts,\n        {\n            train_id: unique_preserve_order(note_list)\n            for train_id, note_list in notes.items()\n        },\n    )\n\n\ndef enforce_static_safety_on_assignment(\n    trains: list[TrainRecord],\n    library: dict[str, object],\n    current_assignment: dict[str, CandidateRoutePlan],\n    max_route_candidates: int,\n) -> tuple[\n    list[TrainRecord],\n    dict[str, CandidateRoutePlan],\n    list[ConflictEntry],\n    dict[str, list[str]],\n]:\n    working_assignment = dict(current_assignment)\n    merged_notes: dict[str, list[str]] = {}\n\n    for _ in range(4):\n        candidate_assignment, _, candidate_notes = optimize_recovery_assignment(\n            reference_trains=trains,\n            current_trains=trains,\n            library=library,\n            current_assignment=working_assignment,\n            prioritize_delay=False,\n            max_route_candidates=max_route_candidates,\n        )\n        conflicts = collect_conflicts(trains, candidate_assignment, "优化方案")\n        merged_notes = merge_note_maps(merged_notes, candidate_notes)\n        if is_strict_safe_conflicts(conflicts):\n            return trains, candidate_assignment, conflicts, merged_notes\n        if assignment_digest(candidate_assignment) == assignment_digest(\n            working_assignment\n        ):\n            working_assignment = candidate_assignment\n            break\n        working_assignment = candidate_assignment\n\n    final_conflicts = collect_conflicts(trains, working_assignment, "优化方案")\n    if is_strict_safe_conflicts(final_conflicts):\n        return trains, working_assignment, final_conflicts, merged_notes\n\n    safe_trains, safe_assignment, safe_conflicts, safety_notes = (\n        enforce_strict_safety_on_realized_trains(\n            realized_trains=trains,\n            library=library,\n            current_assignment=working_assignment,\n            max_route_candidates=max_route_candidates,\n            disturbance_minute=None,\n            reference_trains=trains,\n            max_safety_wait_rounds=24,\n        )\n    )\n    merged_notes = merge_note_maps(merged_notes, safety_notes)\n    return safe_trains, safe_assignment, safe_conflicts, merged_notes\n\n\ndef simulate_scenario_for_assignment(\n    reference_trains: list[TrainRecord],\n    assignment: dict[str, CandidateRoutePlan],\n    scenario: DisturbanceScenario,\n    library: dict[str, object],\n    max_route_candidates: int,\n) -> tuple[list[TrainRecord], dict[str, CandidateRoutePlan], dict[str, list[str]], int]:\n    synchronized_assignment = rebuild_assignment_for_trains(\n        reference_trains, assignment\n    )\n    resources_by_train = {\n        train_id: option.resources\n        for train_id, option in synchronized_assignment.items()\n    }\n    disturbed_trains, delay_notes, closure_blocks = apply_disturbance_events(\n        trains=reference_trains,\n        resources_by_train=resources_by_train,\n        analysis_window_end=scenario.window_end_min,\n        events=scenario_events(scenario),\n    )\n    disturbed_trains, realized_assignment, propagation_rounds = (\n        propagate_planned_disturbance(\n            disturbed_trains,\n            synchronized_assignment,\n            closure_blocks,\n            delay_notes,\n            library,\n            max_route_candidates,\n            scenario_effect_start_minute(scenario),\n        )\n    )\n    normalized_notes = {\n        train_id: unique_preserve_order(notes)\n        for train_id, notes in delay_notes.items()\n    }\n    return disturbed_trains, realized_assignment, normalized_notes, propagation_rounds\n\n\ndef recovery_objective(\n    reference_trains: list[TrainRecord],\n    realized_trains: list[TrainRecord],\n    realized_assignment: dict[str, CandidateRoutePlan],\n    conflicts: list[ConflictEntry],\n) -> tuple[int, int, int, int, int, int, int, int]:\n    metrics = compute_assignment_metrics(\n        realized_trains, realized_assignment, conflicts\n    )\n    delay_stats = compute_delay_statistics(reference_trains, realized_trains)\n    hard_stats = build_hard_conflict_breakdown(conflicts)\n    return (\n        0 if is_strict_safe_conflicts(conflicts) else 1,\n        int(hard_stats["硬冲突项"]),\n        int(hard_stats["股道占用硬冲突项"]),\n        int(hard_stats["进路锁闭硬冲突项"]),\n        int(delay_stats["总晚点时长"]),\n        int(delay_stats["最大晚点时长"]),\n        int(delay_stats["高晚点列车数"]),\n        int(metrics["软冲突项"]),\n        int(metrics["总重叠分钟"]),\n    )\n\n\ndef aggressive_delay_absorption_refinement(\n    reference_trains: list[TrainRecord],\n    library: dict[str, object],\n    scenario: DisturbanceScenario,\n    safe_bundle: tuple[\n        list[TrainRecord],\n        dict[str, CandidateRoutePlan],\n        list[ConflictEntry],\n        dict[str, list[str]],\n        int,\n    ],\n    max_route_candidates: int,\n) -> tuple[\n    list[TrainRecord],\n    dict[str, CandidateRoutePlan],\n    list[ConflictEntry],\n    dict[str, list[str]],\n    int,\n]:\n    best_bundle = safe_bundle\n    best_objective = recovery_objective(\n        reference_trains, best_bundle[0], best_bundle[1], best_bundle[2]\n    )\n\n    for _ in range(max(0, RECOVERY_AGGRESSIVE_REFINEMENT_PASSES)):\n        candidate_assignment, _, candidate_change_notes = optimize_recovery_assignment(\n            reference_trains=reference_trains,\n            current_trains=best_bundle[0],\n            library=library,\n            current_assignment=best_bundle[1],\n            prioritize_delay=True,\n            max_route_candidates=max_route_candidates,\n        )\n        if assignment_digest(candidate_assignment) == assignment_digest(best_bundle[1]):\n            break\n\n        realized_trains, realized_assignment, realized_notes, realized_rounds = (\n            simulate_scenario_for_assignment(\n                reference_trains=reference_trains,\n                assignment=candidate_assignment,\n                scenario=scenario,\n                library=library,\n                max_route_candidates=max_route_candidates,\n            )\n        )\n        safe_trains, safe_assignment, realized_conflicts, safety_notes = (\n            enforce_strict_safety_on_realized_trains(\n                realized_trains,\n                library,\n                realized_assignment,\n                max_route_candidates,\n                scenario.disturbance_minute,\n                reference_trains,\n            )\n        )\n        objective = recovery_objective(\n            reference_trains, safe_trains, safe_assignment, realized_conflicts\n        )\n        if objective[0] != 0 or objective >= best_objective:\n            break\n\n        merged_notes = merge_note_maps(\n            best_bundle[3], candidate_change_notes, realized_notes, safety_notes\n        )\n        best_objective = objective\n        best_bundle = (\n            safe_trains,\n            safe_assignment,\n            realized_conflicts,\n            merged_notes,\n            realized_rounds,\n        )\n\n    return best_bundle\n\n\ndef iterative_recovery_schedule(\n    reference_trains: list[TrainRecord],\n    library: dict[str, object],\n    scenario: DisturbanceScenario,\n    initial_assignment: dict[str, CandidateRoutePlan],\n    max_route_candidates: int,\n) -> tuple[\n    list[TrainRecord],\n    dict[str, CandidateRoutePlan],\n    list[ConflictEntry],\n    dict[str, list[str]],\n    int,\n]:\n    current_assignment = initial_assignment\n    current_change_notes: dict[str, list[str]] = {}\n    best_safe_bundle: (\n        tuple[\n            list[TrainRecord],\n            dict[str, CandidateRoutePlan],\n            list[ConflictEntry],\n            dict[str, list[str]],\n            int,\n        ]\n        | None\n    ) = None\n    best_safe_objective: tuple[int, int, int, int, int, int, int, int] | None = None\n    last_bundle: (\n        tuple[\n            list[TrainRecord],\n            dict[str, CandidateRoutePlan],\n            list[ConflictEntry],\n            dict[str, list[str]],\n            int,\n        ]\n        | None\n    ) = None\n    safe_reached = False\n\n    for _ in range(RECOVERY_AGGRESSIVE_OUTER_LOOPS if RECOVERY_AGGRESSIVE_MODE else 3):\n        realized_trains, realized_assignment, realized_notes, realized_rounds = (\n            simulate_scenario_for_assignment(\n                reference_trains,\n                current_assignment,\n                scenario,\n                library,\n                max_route_candidates,\n            )\n        )\n        safe_trains, safe_assignment, realized_conflicts, safety_notes = (\n            enforce_strict_safety_on_realized_trains(\n                realized_trains,\n                library,\n                realized_assignment,\n                max_route_candidates,\n                scenario.disturbance_minute,\n                reference_trains,\n            )\n        )\n        merged_notes = merge_note_maps(\n            current_change_notes, realized_notes, safety_notes\n        )\n        objective = recovery_objective(\n            reference_trains,\n            safe_trains,\n            safe_assignment,\n            realized_conflicts,\n        )\n        last_bundle = (\n            safe_trains,\n            safe_assignment,\n            realized_conflicts,\n            merged_notes,\n            realized_rounds,\n        )\n\n        if objective[0] == 0:\n            if best_safe_objective is None or objective < best_safe_objective:\n                best_safe_objective = objective\n                best_safe_bundle = last_bundle\n            safe_reached = True\n            candidate_assignment, _, candidate_change_notes = (\n                optimize_recovery_assignment(\n                    reference_trains=reference_trains,\n                    current_trains=safe_trains,\n                    library=library,\n                    current_assignment=safe_assignment,\n                    prioritize_delay=True,\n                    max_route_candidates=max_route_candidates,\n                )\n            )\n            if not candidate_change_notes:\n                break\n            if assignment_digest(candidate_assignment) == assignment_digest(\n                safe_assignment\n            ):\n                break\n            current_assignment = candidate_assignment\n            current_change_notes = candidate_change_notes\n            continue\n\n        candidate_assignment, _, candidate_change_notes = optimize_recovery_assignment(\n            reference_trains=reference_trains,\n            current_trains=safe_trains,\n            library=library,\n            current_assignment=safe_assignment,\n            prioritize_delay=False,\n            max_route_candidates=max_route_candidates,\n        )\n        if assignment_digest(candidate_assignment) == assignment_digest(\n            safe_assignment\n        ):\n            break\n        current_assignment = candidate_assignment\n        current_change_notes = candidate_change_notes\n\n    if best_safe_bundle is not None:\n        return (\n            aggressive_delay_absorption_refinement(\n                reference_trains,\n                library,\n                scenario,\n                best_safe_bundle,\n                max_route_candidates,\n            )\n            if RECOVERY_AGGRESSIVE_MODE\n            else best_safe_bundle\n        )\n    if safe_reached and best_safe_bundle is not None:\n        return (\n            aggressive_delay_absorption_refinement(\n                reference_trains,\n                library,\n                scenario,\n                best_safe_bundle,\n                max_route_candidates,\n            )\n            if RECOVERY_AGGRESSIVE_MODE\n            else best_safe_bundle\n        )\n    if best_safe_bundle is not None:\n        return (\n            aggressive_delay_absorption_refinement(\n                reference_trains,\n                library,\n                scenario,\n                best_safe_bundle,\n                max_route_candidates,\n            )\n            if RECOVERY_AGGRESSIVE_MODE\n            else best_safe_bundle\n        )\n    assert last_bundle is not None\n    return last_bundle\n\n\ndef format_disturbance_target(target: DisturbanceTarget) -> str:\n    if target.intensity_unit == "比例":\n        intensity_text = f"{target.intensity_value * 100:.1f}%"\n        if target.duration_min > 0:\n            intensity_text = f"{intensity_text}/{target.duration_min}分钟"\n    else:\n        intensity_text = f"{int(round(target.intensity_value))}{target.intensity_unit}"\n    return f"{target.display_name}({intensity_text})"\n\n\ndef build_disturbance_description(\n    scenario: DisturbanceScenario,\n    trains: list[TrainRecord],\n) -> str:\n    train_map = {train.record_id: train for train in trains}\n    event_texts = []\n    for event in scenario_events(scenario):\n        source_text = (\n            "、".join(\n                f"{train_map[train_id].train_no or train_id}+{event.source_train_delays.get(train_id, 0)}分"\n                for train_id in event.source_train_ids\n                if train_id in train_map\n            )\n            or "无"\n        )\n        target_text = (\n            "、".join(\n                format_disturbance_target(target) for target in event.spatial_targets\n            )\n            or "无"\n        )\n        event_texts.append(\n            f"{format_minutes_as_clock(event.disturbance_minute)}: 源头列车 {source_text}; 空间资源 {target_text}"\n        )\n    event_text = "；".join(event_texts) or "无"\n    return (\n        f"高峰窗口 {format_minutes_as_clock(scenario.window_start_min)}-{format_minutes_as_clock(scenario.window_end_min)} "\n        f"内按时变负荷强度函数和NHPP抽取扰动时刻 {format_disturbance_minutes(scenario)}；"\n        f"各扰动时刻源头列车由近似Shapley值识别、空间资源由熵权-TOPSIS紧张度与马尔可夫状态转移生成：{event_text}；"\n        f"扰动影响沿受影响列车资源链重构传播，共传播 {scenario.propagation_rounds} 轮。"\n    )\n\n\ndef build_disturbance_load_rows(\n    load_profile: list[dict[str, float]],\n    scenario: DisturbanceScenario,\n) -> list[dict[str, object]]:\n    peak_row = first_peak_load_row(load_profile)\n    peak_minute = (\n        int(round(peak_row["分钟"]))\n        if peak_row is not None\n        else scenario.disturbance_minute\n    )\n    disturbance_minutes = set(scenario_disturbance_minutes(scenario))\n    rows: list[dict[str, object]] = []\n    for row in load_profile:\n        minute = int(round(row["分钟"]))\n        rows.append(\n            {\n                "分钟": minute,\n                "时刻": format_minutes_as_clock(minute),\n                "列车作业数": int(round(row["列车作业数"])),\n                "进路锁闭作业数": int(round(row.get("进路锁闭作业数", 0.0))),\n                "道岔组作业数": int(round(row["道岔组作业数"])),\n                "咽喉能力作业数": int(round(row.get("咽喉能力作业数", 0.0))),\n                "列车负荷归一值": f"{row[\'列车负荷归一值\']:.4f}",\n                "进路负荷归一值": f"{row.get(\'进路负荷归一值\', 0.0):.4f}",\n                "道岔组负荷归一值": f"{row[\'道岔组负荷归一值\']:.4f}",\n                "咽喉负荷归一值": f"{row.get(\'咽喉负荷归一值\', 0.0):.4f}",\n                "时变负荷强度": f"{row.get(\'时变负荷强度\', row[\'负荷指数\']):.4f}",\n                "NHPP扰动概率": f"{row.get(\'NHPP扰动概率\', 0.0):.4f}",\n                "负荷指数": f"{row[\'负荷指数\']:.4f}",\n                "是否峰值首次时刻": "是" if minute == peak_minute else "否",\n                "是否抽中扰动": "是" if minute in disturbance_minutes else "否",\n                "是否扰动时刻": "是" if minute in disturbance_minutes else "否",\n            }\n        )\n    return rows\n\n\ndef build_disturbance_impact_rows(\n    original_trains: list[TrainRecord],\n    disturbed_trains: list[TrainRecord],\n    scenario: DisturbanceScenario,\n    delay_notes: dict[str, list[str]],\n) -> tuple[list[dict[str, object]], dict[str, object]]:\n    original_map = {train.record_id: train for train in original_trains}\n    disturbed_map = {train.record_id: train for train in disturbed_trains}\n    affected_train_ids: list[str] = []\n    rows: list[dict[str, object]] = []\n    counted_delays: list[int] = []\n    effect_start_minute = scenario_effect_start_minute(scenario)\n\n    for train in original_trains:\n        if train.window_end <= effect_start_minute:\n            continue\n        disturbed = disturbed_map[train.record_id]\n        arrival_delay = max(0, disturbed.arrival_min - train.arrival_min)\n        departure_delay = max(0, disturbed.departure_min - train.departure_min)\n        counted_delay = max(arrival_delay, departure_delay)\n        notes = unique_preserve_order(delay_notes.get(train.record_id, []))\n        if counted_delay <= 0 and not notes:\n            continue\n        affected_train_ids.append(train.record_id)\n        counted_delays.append(counted_delay)\n        rows.append(\n            {\n                "场景": "风险驱动短暂扰动",\n                "扰动时刻": format_disturbance_minutes(scenario),\n                "列车记录ID": train.record_id,\n                "车次": train.train_no,\n                "trip_id": train.trip_id,\n                "原计划股道": train.planned_track,\n                "原到达时刻": format_minutes_as_clock(train.arrival_min),\n                "扰动后到达时刻": format_minutes_as_clock(disturbed.arrival_min),\n                "到达晚点(分)": arrival_delay,\n                "原出发时刻": format_minutes_as_clock(train.departure_min),\n                "扰动后出发时刻": format_minutes_as_clock(disturbed.departure_min),\n                "出发晚点(分)": departure_delay,\n                "统计晚点(分)": counted_delay,\n                "影响说明": "；".join(notes),\n            }\n        )\n\n    affected_resources = (\n        build_planned_resource_windows_by_train(\n            [disturbed_map[train_id] for train_id in affected_train_ids]\n        )\n        if affected_train_ids\n        else {}\n    )\n    affected_tracks = {\n        original_map[train_id].planned_track\n        for train_id in affected_train_ids\n        if original_map[train_id].planned_track\n    }\n    affected_throats = {\n        throat\n        for train_id in affected_train_ids\n        for throat in (\n            original_map[train_id].in_throat,\n            original_map[train_id].out_throat,\n        )\n        if throat\n    }\n    affected_partitions = {\n        f"{resource.throat}-{resource.partition}"\n        for resources in affected_resources.values()\n        for resource in resources\n        if resource.resource_category\n        in {"route_lock", "switch_ladder", "flank_protection"}\n        and resource.throat\n        and resource.partition\n    }\n    summary = {\n        "总晚点时长": sum(counted_delays),\n        "平均晚点时长": safe_ratio(sum(counted_delays), len(counted_delays)),\n        "最大晚点时长": max(counted_delays, default=0),\n        "初始扰动列车数": len(scenario.source_train_ids),\n        "受影响列车数": len(affected_train_ids),\n        "受影响股道数": len(affected_tracks),\n        "受影响咽喉数": len(affected_throats),\n        "受影响分区/道岔组数": len(affected_partitions),\n        "affected_train_ids": tuple(affected_train_ids),\n    }\n    return rows, summary\n\n\ndef build_disturbance_summary_rows(\n    trains: list[TrainRecord],\n    scenario: DisturbanceScenario,\n    load_profile: list[dict[str, float]],\n    impact_summary: dict[str, object],\n) -> list[dict[str, object]]:\n    train_map = {train.record_id: train for train in trains}\n    peak_row = first_peak_load_row(load_profile)\n    peak_minute = (\n        int(round(peak_row["分钟"]))\n        if peak_row is not None\n        else scenario.disturbance_minute\n    )\n    source_text = (\n        "；".join(\n            "、".join(\n                f"{train_map[train_id].train_no or train_id}+{event.source_train_delays.get(train_id, 0)}分"\n                for train_id in event.source_train_ids\n                if train_id in train_map\n            )\n            or "无"\n            for event in scenario_events(scenario)\n        )\n        or "无"\n    )\n    target_text = (\n        "；".join(\n            "、".join(\n                format_disturbance_target(target) for target in event.spatial_targets\n            )\n            or "无"\n            for event in scenario_events(scenario)\n        )\n        or "无"\n    )\n    return [\n        {\n            "场景": "风险驱动短暂扰动",\n            "高峰窗口": f"{format_minutes_as_clock(scenario.window_start_min)}-{format_minutes_as_clock(scenario.window_end_min)}",\n            "峰值首次时刻": format_minutes_as_clock(peak_minute),\n            "扰动时刻": format_disturbance_minutes(scenario),\n            "列车作业数": scenario.selected_train_load,\n            "道岔组作业数": scenario.selected_switch_load,\n            "负荷指数": f"{scenario.selected_load_score:.4f}",\n            "源头列车": source_text,\n            "空间扰动资源": target_text,\n            "传播轮次": scenario.propagation_rounds,\n            "总晚点时长": impact_summary["总晚点时长"],\n            "平均晚点时长": f"{impact_summary[\'平均晚点时长\']:.2f}",\n            "最大晚点时长": impact_summary["最大晚点时长"],\n            "初始扰动列车数": impact_summary["初始扰动列车数"],\n            "受影响列车数": impact_summary["受影响列车数"],\n            "受影响股道数": impact_summary["受影响股道数"],\n            "受影响咽喉数": impact_summary["受影响咽喉数"],\n            "受影响分区/道岔组数": impact_summary["受影响分区/道岔组数"],\n            "说明": build_disturbance_description(scenario, trains),\n        }\n    ]\n\n\ndef build_route_library(trains: Iterable[TrainRecord]) -> dict[str, object]:\n    templates_by_signature: defaultdict[\n        tuple[str, str, str], Counter[tuple[str, str, str]]\n    ] = defaultdict(Counter)\n    templates_by_direction: defaultdict[str, Counter[tuple[str, str, str]]] = (\n        defaultdict(Counter)\n    )\n    templates_by_track_direction: defaultdict[\n        tuple[str, str], Counter[tuple[str, str]]\n    ] = defaultdict(Counter)\n    templates_by_family: defaultdict[tuple[str, str, str], Counter[str]] = defaultdict(\n        Counter\n    )\n    tracks_by_signature: defaultdict[tuple[str, str, str], Counter[str]] = defaultdict(\n        Counter\n    )\n    tracks_by_direction: defaultdict[str, Counter[str]] = defaultdict(Counter)\n    global_tracks: Counter[str] = Counter()\n    track_usage_minutes: Counter[str] = Counter()\n    throat_usage_counts: Counter[str] = Counter()\n\n    for train in trains:\n        template = (train.planned_track, train.in_throat, train.out_throat)\n        templates_by_signature[train.route_signature][template] += 1\n        templates_by_direction[train.direction][template] += 1\n        templates_by_track_direction[(train.direction, train.planned_track)][\n            (train.in_throat, train.out_throat)\n        ] += 1\n        templates_by_family[(train.direction, train.in_throat, train.out_throat)][\n            train.planned_track\n        ] += 1\n        tracks_by_signature[train.route_signature][train.planned_track] += 1\n        tracks_by_direction[train.direction][train.planned_track] += 1\n        global_tracks[train.planned_track] += 1\n        track_usage_minutes[train.planned_track] += train.dwell_end - train.dwell_start\n        throat_usage_counts[train.in_throat] += 1\n        throat_usage_counts[train.out_throat] += 1\n\n    return {\n        "templates_by_signature": templates_by_signature,\n        "templates_by_direction": templates_by_direction,\n        "templates_by_track_direction": templates_by_track_direction,\n        "templates_by_family": templates_by_family,\n        "tracks_by_signature": tracks_by_signature,\n        "tracks_by_direction": tracks_by_direction,\n        "observed_track_counts": global_tracks,\n        "track_usage_minutes": track_usage_minutes,\n        "throat_usage_counts": throat_usage_counts,\n        "all_tracks_sorted": sorted(\n            set(global_tracks.keys()).union(CANONICAL_TRACKS), key=track_sort_key\n        ),\n    }\n\n\ndef infer_throat_pair(\n    train: TrainRecord, track: str, library: dict[str, object]\n) -> tuple[str, str]:\n    signature_counter = library["templates_by_signature"].get(\n        train.route_signature, Counter()\n    )\n    by_track = Counter()\n    for (candidate_track, in_throat, out_throat), count in signature_counter.items():\n        if candidate_track == track:\n            by_track[(in_throat, out_throat)] += count\n    if by_track:\n        return by_track.most_common(1)[0][0]\n\n    direction_track_counter = library["templates_by_track_direction"].get(\n        (train.direction, track), Counter()\n    )\n    if direction_track_counter:\n        return direction_track_counter.most_common(1)[0][0]\n\n    if track == train.planned_track:\n        return (train.in_throat, train.out_throat)\n\n    if train.direction == "上行":\n        if train.prev_station in {"Forest-Est", "Hal", "Ruisbroek"}:\n            return ("南咽喉", "东咽喉")\n        if train.operation_type == "始发":\n            return ("始发端", "东咽喉")\n        return ("西咽喉", "东咽喉")\n\n    if train.next_station in {"Forest-Est", "Ruisbroek", "Hal"}:\n        return ("东咽喉", "南咽喉")\n    if train.operation_type == "终到":\n        return ("东咽喉", "终到端")\n    return ("东咽喉", "西咽喉")\n\n\ndef infer_throat_pair_candidates(\n    train: TrainRecord,\n    track: str,\n    library: dict[str, object],\n    rich_candidates: bool,\n) -> list[CandidateSeed]:\n    seeds: dict[tuple[str, str, str], CandidateSeed] = {}\n    signature_counter = library["templates_by_signature"].get(\n        train.route_signature, Counter()\n    )\n    for (\n        candidate_track,\n        in_throat,\n        out_throat,\n    ), count in signature_counter.most_common(6 if rich_candidates else 2):\n        if candidate_track != track:\n            continue\n        add_candidate_seed(\n            seeds,\n            track,\n            in_throat,\n            out_throat,\n            "同路径历史进路",\n            int(count),\n            "基于相同路径样本生成的咽喉组合候选。",\n        )\n\n    direction_track_counter = library["templates_by_track_direction"].get(\n        (train.direction, track), Counter()\n    )\n    for (in_throat, out_throat), count in direction_track_counter.most_common(\n        4 if rich_candidates else 1\n    ):\n        add_candidate_seed(\n            seeds,\n            track,\n            in_throat,\n            out_throat,\n            "同方向工程进路",\n            int(count),\n            "基于同方向同股道样本生成的咽喉组合候选。",\n        )\n\n    heuristic_pair = infer_throat_pair(train, track, library)\n    add_candidate_seed(\n        seeds,\n        track,\n        heuristic_pair[0],\n        heuristic_pair[1],\n        "邻接股道工程推断",\n        0,\n        "基于股道拓扑推断的咽喉组合候选。",\n    )\n\n    if track == train.planned_track:\n        add_candidate_seed(\n            seeds,\n            track,\n            train.in_throat,\n            train.out_throat,\n            "原计划工程进路",\n            0,\n            "保留原计划股道及咽喉组合。",\n        )\n\n    if rich_candidates:\n        inbound_candidates = unique_preserve_order(\n            [train.in_throat, heuristic_pair[0]]\n            + [seed.in_throat for seed in seeds.values()]\n        )\n        outbound_candidates = unique_preserve_order(\n            [train.out_throat, heuristic_pair[1]]\n            + [seed.out_throat for seed in seeds.values()]\n        )\n        for in_throat in inbound_candidates[:3]:\n            for out_throat in outbound_candidates[:3]:\n                add_candidate_seed(\n                    seeds,\n                    track,\n                    in_throat,\n                    out_throat,\n                    "恢复扩展咽喉组合",\n                    0,\n                    "在恢复阶段扩展的可行进出站咽喉组合。",\n                )\n\n    ordered = sorted(\n        seeds.values(),\n        key=lambda seed: (\n            SOURCE_PRIORITY.get(seed.source_level, 99),\n            -seed.support_count,\n            track_distance(seed.track, train.planned_track),\n            seed.in_throat != train.in_throat,\n            seed.out_throat != train.out_throat,\n            seed.track,\n            seed.in_throat,\n            seed.out_throat,\n        ),\n    )\n    return ordered\n\n\ndef candidate_bottleneck_penalty(\n    option: CandidateRoutePlan,\n    library: dict[str, object],\n) -> float:\n    track_minutes = float(library["track_usage_minutes"].get(option.track, 0))\n    throat_load = float(\n        library["throat_usage_counts"].get(option.in_throat, 0)\n    ) + float(library["throat_usage_counts"].get(option.out_throat, 0))\n    partition_penalty = 0.0\n    if option.in_partition:\n        partition_penalty += 2.0\n    if option.out_partition:\n        partition_penalty += 2.0\n    route_variant_penalty = 1.5 if option.route_variant != "标准通道" else 0.0\n    return (\n        track_minutes * 0.015\n        + throat_load * 0.25\n        + partition_penalty\n        + route_variant_penalty\n    )\n\n\ndef screen_candidate_route_plans(\n    train: TrainRecord,\n    candidate_plans: list[CandidateRoutePlan],\n    library: dict[str, object],\n    max_route_candidates: int,\n    rich_candidates: bool,\n    recovery_aggressive: bool = False,\n) -> list[CandidateRoutePlan]:\n    if not candidate_plans:\n        return []\n    if recovery_aggressive:\n        ordered = sorted(\n            candidate_plans,\n            key=lambda option: (\n                candidate_bottleneck_penalty(option, library),\n                option.delay_risk_cost,\n                option.route_variant != "标准通道",\n                SOURCE_PRIORITY.get(option.source_level, 99),\n                option.option_id,\n            ),\n        )\n        limit = max(RECOVERY_AGGRESSIVE_OPTION_LIMIT, max_route_candidates)\n        return ordered[:limit] if limit > 0 else ordered\n    ordered = sorted(\n        candidate_plans,\n        key=lambda option: (\n            candidate_bottleneck_penalty(option, library),\n            option.linear_cost,\n            option.track != train.planned_track,\n            option.route_variant != "标准通道",\n            option.option_id,\n        ),\n    )\n    if FREE_SPACE_SEARCH_ENABLED:\n        limit = (\n            FREE_SPACE_RICH_OPTION_LIMIT if rich_candidates else FREE_SPACE_OPTION_LIMIT\n        )\n        return ordered[:limit] if limit > 0 else ordered\n    if not rich_candidates:\n        limit = max_route_candidates if max_route_candidates > 0 else len(ordered)\n        return ordered[:limit]\n\n    per_track_limit = 3\n    per_family_limit = 2\n    track_counts: defaultdict[str, int] = defaultdict(int)\n    family_counts: defaultdict[str, int] = defaultdict(int)\n    screened: list[CandidateRoutePlan] = []\n    limit = max_route_candidates * 3 if max_route_candidates > 0 else len(ordered)\n    for option in ordered:\n        if track_counts[option.track] >= per_track_limit:\n            continue\n        if family_counts[option.route_family] >= per_family_limit:\n            continue\n        screened.append(option)\n        track_counts[option.track] += 1\n        family_counts[option.route_family] += 1\n        if len(screened) >= limit:\n            break\n    return screened if screened else ordered[: max(1, min(len(ordered), limit))]\n\n\ndef add_candidate_seed(\n    seeds: dict[tuple[str, str, str], CandidateSeed],\n    track: str,\n    in_throat: str,\n    out_throat: str,\n    source_level: str,\n    support_count: int,\n    note: str,\n) -> None:\n    key = (track, in_throat, out_throat)\n    new_seed = CandidateSeed(\n        track=track,\n        in_throat=in_throat,\n        out_throat=out_throat,\n        source_level=source_level,\n        support_count=support_count,\n        note=note,\n    )\n    old_seed = seeds.get(key)\n    if old_seed is None:\n        seeds[key] = new_seed\n        return\n    old_priority = SOURCE_PRIORITY.get(old_seed.source_level, 99)\n    new_priority = SOURCE_PRIORITY.get(source_level, 99)\n    if (new_priority, -support_count) < (old_priority, -old_seed.support_count):\n        seeds[key] = new_seed\n\n\ndef track_occupancy_interval(train: TrainRecord) -> tuple[int, int]:\n    start_min = train.inbound_end - train.bc_min\n    end_min = max(train.dwell_end, train.outbound_start + train.cd_min)\n    return start_min, end_min\n\n\ndef build_track_occupancy_index(\n    trains: Iterable[TrainRecord],\n) -> dict[str, list[tuple[int, int, str]]]:\n    occupancy_by_track: defaultdict[str, list[tuple[int, int, str]]] = defaultdict(list)\n    for train in trains:\n        start_min, end_min = track_occupancy_interval(train)\n        occupancy_by_track[train.planned_track].append(\n            (start_min, end_min, train.record_id)\n        )\n    for track in occupancy_by_track:\n        occupancy_by_track[track].sort()\n    return dict(occupancy_by_track)\n\n\ndef is_track_idle_for_train(\n    train: TrainRecord,\n    track: str,\n    occupancy_by_track: dict[str, list[tuple[int, int, str]]],\n) -> bool:\n    candidate_start, candidate_end = track_occupancy_interval(train)\n    for occupied_start, occupied_end, occupied_train_id in occupancy_by_track.get(\n        track, []\n    ):\n        if occupied_train_id == train.record_id:\n            continue\n        if (\n            overlap_minutes(\n                candidate_start, candidate_end, occupied_start, occupied_end\n            )\n            > 0\n        ):\n            return False\n    return True\n\n\ndef build_candidate_route_plans(\n    trains: list[TrainRecord],\n    library: dict[str, object],\n    max_route_candidates: int,\n    rich_candidate_train_ids: set[str] | None = None,\n    recovery_aggressive_train_ids: set[str] | None = None,\n) -> tuple[dict[str, list[CandidateRoutePlan]], dict[str, TrainRecord]]:\n    train_map = {train.record_id: train for train in trains}\n    all_tracks: list[str] = library["all_tracks_sorted"]\n    occupancy_by_track = build_track_occupancy_index(trains)\n    track_usage_counts = {\n        track: len(windows) for track, windows in occupancy_by_track.items()\n    }\n    track_usage_minutes = {\n        track: sum(max(0, end_min - start_min) for start_min, end_min, _ in windows)\n        for track, windows in occupancy_by_track.items()\n    }\n    options_by_train: dict[str, list[CandidateRoutePlan]] = {}\n    variant_cache: dict[tuple[object, ...], tuple[CandidateRoutePlan, ...]] = {}\n\n    def build_variants_cached(\n        train: TrainRecord,\n        seed: CandidateSeed,\n        rich_variants: bool,\n    ) -> list[CandidateRoutePlan]:\n        cache_key = (\n            train_timing_cache_key(train),\n            train.direction,\n            train.operation_type,\n            train.planned_track,\n            train.in_throat,\n            train.out_throat,\n            seed.track,\n            seed.in_throat,\n            seed.out_throat,\n            seed.source_level,\n            seed.support_count,\n            seed.note,\n            rich_variants,\n        )\n        cached = variant_cache.get(cache_key)\n        if cached is None:\n            cached = tuple(\n                build_candidate_route_plan_variants(\n                    train, seed, library, rich_variants=rich_variants\n                )\n            )\n            variant_cache[cache_key] = cached\n        return list(cached)\n\n    for train in trains:\n        recovery_aggressive = (\n            RECOVERY_AGGRESSIVE_MODE\n            and recovery_aggressive_train_ids is not None\n            and train.record_id in recovery_aggressive_train_ids\n        )\n        rich_candidates = (\n            recovery_aggressive\n            or FREE_SPACE_SEARCH_ENABLED\n            or (\n                rich_candidate_train_ids is not None\n                and train.record_id in rich_candidate_train_ids\n            )\n        )\n        if recovery_aggressive:\n            candidate_tracks = list(all_tracks)\n        else:\n            candidate_tracks = [\n                track\n                for track in all_tracks\n                if is_track_idle_for_train(train, track, occupancy_by_track)\n            ]\n        planned_track_idle = train.planned_track in candidate_tracks\n        if not planned_track_idle:\n            candidate_tracks = [train.planned_track] + [\n                track for track in candidate_tracks if track != train.planned_track\n            ]\n        if recovery_aggressive:\n            candidate_tracks = sorted(\n                set(candidate_tracks),\n                key=lambda track: (\n                    track_usage_minutes.get(track, 0),\n                    track_usage_counts.get(track, 0),\n                    track_distance(track, train.planned_track),\n                    track_sort_key(track),\n                ),\n            )\n        else:\n            candidate_tracks = sorted(\n                set(candidate_tracks),\n                key=lambda track: (\n                    0 if track == train.planned_track else 1,\n                    track_usage_minutes.get(track, 0),\n                    track_usage_counts.get(track, 0),\n                    track_distance(track, train.planned_track),\n                    track_sort_key(track),\n                ),\n            )\n        if FREE_SPACE_SEARCH_ENABLED:\n            track_limit = (\n                RECOVERY_AGGRESSIVE_TRACK_LIMIT\n                if recovery_aggressive\n                else (\n                    FREE_SPACE_RICH_TRACK_LIMIT\n                    if rich_candidates\n                    else FREE_SPACE_TRACK_LIMIT\n                )\n            )\n            if track_limit > 0 and len(candidate_tracks) > track_limit:\n                candidate_tracks = candidate_tracks[:track_limit]\n        elif (\n            not rich_candidates\n            and max_route_candidates > 0\n            and len(candidate_tracks) > max_route_candidates\n        ):\n            candidate_tracks = candidate_tracks[:max_route_candidates]\n\n        selected_plans: list[CandidateRoutePlan] = []\n        for track in candidate_tracks:\n            if not rich_candidates:\n                in_throat, out_throat = (\n                    (train.in_throat, train.out_throat)\n                    if track == train.planned_track\n                    else infer_throat_pair(train, track, library)\n                )\n                if track == train.planned_track and not planned_track_idle:\n                    seed = CandidateSeed(\n                        track=track,\n                        in_throat=in_throat,\n                        out_throat=out_throat,\n                        source_level="原计划冲突股道保留",\n                        support_count=0,\n                        note="原计划股道在当前时段并非空余，仅为保留原计划对照方案而加入。",\n                    )\n                else:\n                    seed = CandidateSeed(\n                        track=track,\n                        in_throat=in_throat,\n                        out_throat=out_throat,\n                        source_level="时隙空余股道候选",\n                        support_count=0,\n                        note="按该列车占用时段筛出的空余股道全量候选，不再使用历史样本与近邻裁剪。",\n                    )\n                selected_plans.extend(\n                    build_variants_cached(\n                        train,\n                        seed,\n                        False,\n                    )\n                )\n                continue\n            seeds = infer_throat_pair_candidates(\n                train, track, library, rich_candidates=rich_candidates\n            )\n            if FREE_SPACE_SEARCH_ENABLED:\n                seed_limit = (\n                    0\n                    if recovery_aggressive\n                    else (\n                        FREE_SPACE_RICH_SEED_LIMIT\n                        if rich_candidates\n                        else FREE_SPACE_SEED_LIMIT\n                    )\n                )\n                if seed_limit > 0:\n                    seeds = seeds[:seed_limit]\n            if track == train.planned_track and not planned_track_idle:\n                seed_map = {\n                    (seed.track, seed.in_throat, seed.out_throat): seed\n                    for seed in seeds\n                }\n                add_candidate_seed(\n                    seed_map,\n                    track,\n                    train.in_throat,\n                    train.out_throat,\n                    "原计划冲突股道保留",\n                    0,\n                    "原计划股道在当前时段并非空余，仅为保留原计划对照方案而加入。",\n                )\n                seeds = list(seed_map.values())\n            if not seeds:\n                heuristic_in, heuristic_out = infer_throat_pair(train, track, library)\n                if track == train.planned_track and not planned_track_idle:\n                    seeds = [\n                        CandidateSeed(\n                            track=track,\n                            in_throat=train.in_throat,\n                            out_throat=train.out_throat,\n                            source_level="原计划冲突股道保留",\n                            support_count=0,\n                            note="原计划股道在当前时段并非空余，仅为保留原计划对照方案而加入。",\n                        )\n                    ]\n                else:\n                    seeds = [\n                        CandidateSeed(\n                            track=track,\n                            in_throat=heuristic_in,\n                            out_throat=heuristic_out,\n                            source_level="时隙空余股道候选",\n                            support_count=0,\n                            note="按该列车占用时段筛出的空余股道候选。",\n                        )\n                    ]\n            for seed in seeds:\n                selected_plans.extend(\n                    build_variants_cached(\n                        train,\n                        seed,\n                        rich_candidates,\n                    )\n                )\n\n        selected_plans = deduplicate_equivalent_candidate_plans(selected_plans)\n        selected_plans = screen_candidate_route_plans(\n            train,\n            selected_plans,\n            library,\n            max_route_candidates,\n            rich_candidates=rich_candidates,\n            recovery_aggressive=recovery_aggressive,\n        )\n\n        option_list: list[CandidateRoutePlan] = []\n        for rank, option in enumerate(selected_plans, start=1):\n            option_list.append(\n                CandidateRoutePlan(\n                    option_id=f"x_{train.index:03d}_{rank:02d}",\n                    train_id=option.train_id,\n                    track=option.track,\n                    in_throat=option.in_throat,\n                    out_throat=option.out_throat,\n                    in_partition=option.in_partition,\n                    out_partition=option.out_partition,\n                    inbound_channel=option.inbound_channel,\n                    outbound_channel=option.outbound_channel,\n                    route_variant=option.route_variant,\n                    route_code=option.route_code.replace(\n                        option.option_id, f"x_{train.index:03d}_{rank:02d}"\n                    )\n                    if option.option_id in option.route_code\n                    else option.route_code,\n                    inbound_route_code=option.inbound_route_code,\n                    outbound_route_code=option.outbound_route_code,\n                    route_family=option.route_family,\n                    source_level=option.source_level,\n                    support_count=option.support_count,\n                    route_score=option.route_score,\n                    candidate_rank=rank,\n                    operation_type=option.operation_type,\n                    delay_risk_cost=option.delay_risk_cost,\n                    stability_cost=option.stability_cost,\n                    balance_reward=option.balance_reward,\n                    linear_cost=option.linear_cost,\n                    note=option.note,\n                    resources=tuple(\n                        ResourceWindow(\n                            train_id=resource.train_id,\n                            option_id=f"x_{train.index:03d}_{rank:02d}",\n                            route_code=resource.route_code,\n                            stage=resource.stage,\n                            resource_name=resource.resource_name,\n                            resource_category=resource.resource_category,\n                            movement_family=resource.movement_family,\n                            throat=resource.throat,\n                            partition=resource.partition,\n                            channel=resource.channel,\n                            zone=resource.zone,\n                            track=resource.track,\n                            start_min=resource.start_min,\n                            end_min=resource.end_min,\n                        )\n                        for resource in option.resources\n                    ),\n                )\n            )\n\n        options_by_train[train.record_id] = option_list\n\n    return options_by_train, train_map\n\n\ndef build_candidate_route_plan_variants(\n    train: TrainRecord,\n    seed: CandidateSeed,\n    library: dict[str, object],\n    rich_variants: bool = False,\n) -> list[CandidateRoutePlan]:\n    zone = classify_track_zone(seed.track)\n    track_counts: Counter[str] = library["observed_track_counts"]\n    track_usage_minutes: Counter[str] = library["track_usage_minutes"]\n    throat_usage_counts: Counter[str] = library["throat_usage_counts"]\n    average_track_usage = sum(track_usage_minutes.values()) / max(\n        1, len(CANONICAL_TRACKS)\n    )\n    adjacency_penalty = (\n        max(0, track_adjacency_score(seed.track, train.planned_track) - 1) * 6\n    )\n    in_partition = throat_partition(seed.track, seed.in_throat)\n    out_partition = throat_partition(seed.track, seed.out_throat)\n    in_partition_shift = 0 if seed.in_throat == train.in_throat else 1\n    out_partition_shift = 0 if seed.out_throat == train.out_throat else 1\n    in_partition_shift += max(\n        0, throat_partition_distance(seed.in_throat, seed.track, train.planned_track)\n    )\n    out_partition_shift += max(\n        0, throat_partition_distance(seed.out_throat, seed.track, train.planned_track)\n    )\n    throat_penalty = (\n        10\n        if (seed.in_throat, seed.out_throat) != (train.in_throat, train.out_throat)\n        else 0\n    )\n    congestion_penalty = (\n        0.08 * throat_usage_counts.get(seed.in_throat, 0)\n        + 0.08 * throat_usage_counts.get(seed.out_throat, 0)\n        + 0.03 * track_counts.get(seed.track, 0)\n    )\n    operation_penalty = 0.0\n    if (\n        train.operation_type in {"始发", "终到", "始发终到"}\n        and seed.track != train.planned_track\n    ):\n        operation_penalty += 8.0\n    delay_risk_cost = (\n        adjacency_penalty\n        + 4.0 * (in_partition_shift + out_partition_shift)\n        + throat_penalty\n        + congestion_penalty\n        + operation_penalty\n    )\n\n    change_penalty = 28 if seed.track != train.planned_track else 0\n    distance_penalty = 6 * track_distance(seed.track, train.planned_track)\n    source_penalty = {\n        "原计划工程进路": 0,\n        "同路径历史进路": 4,\n        "同方向同咽喉进路": 10,\n        "同方向工程进路": 16,\n        "邻接股道工程推断": 24,\n        "原计划冲突股道保留": 0,\n        "时隙空余股道候选": 0,\n    }.get(seed.source_level, 0)\n    stability_cost = (\n        change_penalty\n        + distance_penalty\n        + source_penalty\n        + max(0, track_adjacency_score(seed.track, train.planned_track) - 1) * 4\n    )\n    underuse_bonus = max(\n        0.0, average_track_usage - track_usage_minutes.get(seed.track, 0)\n    )\n    balance_reward = min(18.0, underuse_bonus / max(1.0, average_track_usage) * 18.0)\n    support_discount = min(22, seed.support_count * 1.2)\n    inbound_channels = channel_variants(\n        seed.track, seed.in_throat, "接车", train.operation_type\n    )\n    outbound_channels = channel_variants(\n        seed.track, seed.out_throat, "发车", train.operation_type\n    )\n    candidate_plans: list[CandidateRoutePlan] = []\n    variant_index = 1\n    for inbound_channel in inbound_channels:\n        for outbound_channel in outbound_channels:\n            route_variant = (\n                "标准通道"\n                if inbound_channel.endswith("-P") and outbound_channel.endswith("-P")\n                else "疏解通道"\n            )\n            variant_penalty = 0.0 if route_variant == "标准通道" else 6.0\n            linear_cost = max(\n                0.0,\n                delay_risk_cost * 6.0\n                + stability_cost * 2.4\n                + variant_penalty\n                - balance_reward\n                - support_discount,\n            )\n            route_score = seed.support_count * 6 - linear_cost\n            option_id = f"x_{train.index:03d}_TMP{variant_index:02d}"\n            route_code = (\n                f"RTE-{train.record_id}-{direction_code(train.direction)}-"\n                f"{throat_code(seed.in_throat)}{seed.track}-{throat_code(seed.out_throat)}-{variant_index}"\n            )\n            inbound_route_code = (\n                f"IN-{direction_code(train.direction)}-{inbound_channel}-{seed.track}"\n            )\n            outbound_route_code = (\n                f"OUT-{direction_code(train.direction)}-{outbound_channel}-{seed.track}"\n            )\n            route_family = f"{train.direction}_{seed.in_throat}进_{seed.out_throat}出_{zone}_{route_variant}"\n            resources = build_resource_windows(\n                train=train,\n                option_id=option_id,\n                route_code=route_code,\n                track=seed.track,\n                in_throat=seed.in_throat,\n                out_throat=seed.out_throat,\n                inbound_channel=inbound_channel,\n                outbound_channel=outbound_channel,\n            )\n            note = (\n                f"{seed.note} 支撑样本数={seed.support_count}。"\n                f" 延误风险代理={delay_risk_cost:.2f}；稳定性代价={stability_cost:.2f}；"\n                f"均衡奖励={balance_reward:.2f}；通道变体={route_variant}。"\n            )\n            candidate_plans.append(\n                CandidateRoutePlan(\n                    option_id=option_id,\n                    train_id=train.record_id,\n                    track=seed.track,\n                    in_throat=seed.in_throat,\n                    out_throat=seed.out_throat,\n                    in_partition=in_partition,\n                    out_partition=out_partition,\n                    inbound_channel=inbound_channel,\n                    outbound_channel=outbound_channel,\n                    route_variant=route_variant,\n                    route_code=route_code,\n                    inbound_route_code=inbound_route_code,\n                    outbound_route_code=outbound_route_code,\n                    route_family=route_family,\n                    source_level=seed.source_level,\n                    support_count=seed.support_count,\n                    route_score=route_score,\n                    candidate_rank=variant_index,\n                    operation_type=train.operation_type,\n                    delay_risk_cost=delay_risk_cost,\n                    stability_cost=stability_cost,\n                    balance_reward=balance_reward,\n                    linear_cost=linear_cost,\n                    note=note,\n                    resources=resources,\n                )\n            )\n            variant_index += 1\n    if not candidate_plans:\n        return []\n    ordered = sorted(\n        candidate_plans,\n        key=lambda option: (\n            candidate_bottleneck_penalty(option, library),\n            option.linear_cost,\n            option.route_variant != "标准通道",\n            option.inbound_channel,\n            option.outbound_channel,\n            option.option_id,\n        ),\n    )\n    if FREE_SPACE_SEARCH_ENABLED:\n        variant_limit = (\n            FREE_SPACE_RICH_VARIANT_LIMIT if rich_variants else FREE_SPACE_VARIANT_LIMIT\n        )\n        return ordered[:variant_limit] if variant_limit > 0 else ordered\n    if rich_variants:\n        return ordered[: min(3, len(ordered))]\n    return [ordered[0]]\n\n\ndef build_resource_windows(\n    train: TrainRecord,\n    option_id: str,\n    route_code: str,\n    track: str,\n    in_throat: str,\n    out_throat: str,\n    inbound_channel: str,\n    outbound_channel: str,\n) -> tuple[ResourceWindow, ...]:\n    cache_key = (\n        train_timing_cache_key(train),\n        option_id,\n        route_code,\n        track,\n        in_throat,\n        out_throat,\n        inbound_channel,\n        outbound_channel,\n    )\n    cached = RESOURCE_WINDOW_CACHE.get(cache_key)\n    if cached is not None:\n        return cached\n\n    zone = classify_track_zone(track)\n    in_partition = throat_partition(track, in_throat)\n    out_partition = throat_partition(track, out_throat)\n    windows: list[ResourceWindow] = []\n\n    def add_window(\n        stage: str,\n        resource_name: str,\n        resource_category: str,\n        movement_family: str,\n        start_min: int,\n        end_min: int,\n        throat: str = "",\n        partition: str = "",\n        channel: str = "",\n        resource_track: str = "",\n    ) -> None:\n        if end_min <= start_min:\n            return\n        windows.append(\n            ResourceWindow(\n                train_id=train.record_id,\n                option_id=option_id,\n                route_code=route_code,\n                stage=stage,\n                resource_name=resource_name,\n                resource_category=resource_category,\n                movement_family=movement_family,\n                throat=throat,\n                partition=partition,\n                channel=channel,\n                zone=zone,\n                track=resource_track,\n                start_min=start_min,\n                end_min=end_min,\n            )\n        )\n\n    ab_end = train.inbound_start + train.ab_min\n    b_end = ab_end + train.b_min\n    bc_end = train.inbound_end\n    cd_end = train.outbound_start + train.cd_min\n    d_end = cd_end + train.d_min\n    de_end = train.outbound_end\n\n    add_window(\n        "A-B",\n        f"入口咽喉:{in_throat}",\n        "throat_capacity",\n        "接车",\n        train.inbound_start,\n        ab_end,\n        throat=in_throat,\n        partition=in_partition,\n        resource_track=track,\n    )\n    add_window(\n        "A-B",\n        f"接车进路锁闭:{in_throat}:{in_partition or zone}",\n        "route_lock",\n        "接车",\n        train.inbound_start,\n        ab_end,\n        throat=in_throat,\n        partition=in_partition,\n        channel=inbound_channel,\n        resource_track=track,\n    )\n    add_window(\n        "B",\n        f"接车道岔组:{in_throat}:{in_partition or zone}",\n        "switch_ladder",\n        "接车",\n        ab_end,\n        b_end,\n        throat=in_throat,\n        partition=in_partition,\n        channel=inbound_channel,\n        resource_track=track,\n    )\n    add_window(\n        "B-C",\n        f"接车防护带:{in_throat}:{in_partition or flank_zone_label(zone)}",\n        "flank_protection",\n        "接车",\n        b_end,\n        bc_end,\n        throat=in_throat,\n        partition=in_partition,\n        channel=inbound_channel,\n        resource_track=track,\n    )\n    add_window(\n        "B-C",\n        f"股道接入:{track}",\n        "track_access",\n        "接车",\n        b_end,\n        bc_end,\n        throat=in_throat,\n        resource_track=track,\n    )\n    add_window(\n        "C",\n        f"股道占用:{track}",\n        "track",\n        "股道",\n        train.dwell_start,\n        train.dwell_end,\n        resource_track=track,\n    )\n    if train.operation_type in {"始发", "始发终到"}:\n        add_window(\n            "C",\n            f"始发作业:{track}",\n            "terminal_op",\n            "始发",\n            train.dwell_start,\n            train.dwell_end,\n            resource_track=track,\n        )\n    if train.operation_type in {"终到", "始发终到"}:\n        add_window(\n            "C",\n            f"终到作业:{track}",\n            "terminal_op",\n            "终到",\n            train.dwell_start,\n            train.dwell_end,\n            resource_track=track,\n        )\n    add_window(\n        "C-D",\n        f"发车准备:{track}",\n        "track_access",\n        "发车",\n        train.outbound_start,\n        cd_end,\n        throat=out_throat,\n        partition=out_partition,\n        channel=outbound_channel,\n        resource_track=track,\n    )\n    add_window(\n        "C-D",\n        f"发车道岔组:{out_throat}:{out_partition or zone}",\n        "switch_ladder",\n        "发车",\n        train.outbound_start,\n        cd_end,\n        throat=out_throat,\n        partition=out_partition,\n        channel=outbound_channel,\n        resource_track=track,\n    )\n    add_window(\n        "D",\n        f"发车进路锁闭:{out_throat}:{out_partition or zone}",\n        "route_lock",\n        "发车",\n        cd_end,\n        d_end,\n        throat=out_throat,\n        partition=out_partition,\n        channel=outbound_channel,\n        resource_track=track,\n    )\n    add_window(\n        "D-E",\n        f"发车防护带:{out_throat}:{out_partition or flank_zone_label(zone)}",\n        "flank_protection",\n        "发车",\n        d_end,\n        de_end,\n        throat=out_throat,\n        partition=out_partition,\n        channel=outbound_channel,\n        resource_track=track,\n    )\n    add_window(\n        "D-E",\n        f"出口咽喉:{out_throat}",\n        "throat_capacity",\n        "发车",\n        d_end,\n        de_end,\n        throat=out_throat,\n        partition=out_partition,\n        resource_track=track,\n    )\n\n    result = tuple(windows)\n    if len(RESOURCE_WINDOW_CACHE) < RESOURCE_WINDOW_CACHE_LIMIT:\n        RESOURCE_WINDOW_CACHE[cache_key] = result\n    return result\n\n\ndef classify_window_conflict(\n    window_a: ResourceWindow, window_b: ResourceWindow\n) -> PairConflictDetail | None:\n    overlap = overlap_minutes(\n        window_a.start_min, window_a.end_min, window_b.start_min, window_b.end_min\n    )\n    if overlap <= 0:\n        return None\n\n    categories = {window_a.resource_category, window_b.resource_category}\n    if window_a.track and window_b.track and window_a.track == window_b.track:\n        if "terminal_op" in categories:\n            return PairConflictDetail(\n                conflict_level="硬冲突",\n                conflict_type="始发终到作业冲突",\n                resource_name=f"股道 {window_a.track}",\n                overlap_min=overlap,\n                description="同一股道上的始发/终到作业时段发生重叠。",\n            )\n        if categories & {"track", "track_access"}:\n            return PairConflictDetail(\n                conflict_level="硬冲突",\n                conflict_type="股道占用冲突",\n                resource_name=f"股道 {window_a.track}",\n                overlap_min=overlap,\n                description="同一股道的接入、停留或发车准备区间发生重叠。",\n            )\n\n    relation = None\n    if window_a.throat and window_a.throat == window_b.throat:\n        relation = throat_partition_relation(\n            window_a.throat, window_a.track, window_b.track\n        )\n    if relation:\n        partition_a, partition_b, relation_level = relation\n        zone_label = (\n            partition_a\n            if partition_a == partition_b\n            else f"{partition_a}/{partition_b}"\n        )\n        movement_pair = (window_a.movement_family, window_b.movement_family)\n        if "switch_ladder" in categories:\n            return PairConflictDetail(\n                conflict_level="硬冲突",\n                conflict_type="道岔组冲突",\n                resource_name=f"{window_a.throat}-{zone_label}",\n                overlap_min=overlap,\n                description="同一咽喉分区或相邻分区的道岔组占用发生重叠。",\n            )\n        if movement_pair == ("接车", "接车"):\n            if "route_lock" in categories:\n                return PairConflictDetail(\n                    conflict_level="硬冲突",\n                    conflict_type="接车进路锁闭冲突",\n                    resource_name=f"{window_a.throat}-{zone_label}",\n                    overlap_min=overlap,\n                    description="同一接车咽喉分区的进路锁闭区间发生重叠。",\n                )\n            if "flank_protection" in categories:\n                return PairConflictDetail(\n                    conflict_level="硬冲突",\n                    conflict_type="防护带冲突",\n                    resource_name=f"{window_a.throat}-{zone_label}",\n                    overlap_min=overlap,\n                    description="接车防护带锁闭区间发生重叠。",\n                )\n            return PairConflictDetail(\n                conflict_level="硬冲突",\n                conflict_type="咽喉能力冲突",\n                resource_name=f"{window_a.throat}-{zone_label}",\n                overlap_min=overlap,\n                description="接车入口咽喉能力区间发生重叠。",\n            )\n        if movement_pair == ("发车", "发车"):\n            if "route_lock" in categories:\n                return PairConflictDetail(\n                    conflict_level="硬冲突",\n                    conflict_type="发车进路锁闭冲突",\n                    resource_name=f"{window_a.throat}-{zone_label}",\n                    overlap_min=overlap,\n                    description="同一发车咽喉分区的进路锁闭区间发生重叠。",\n                )\n            if "flank_protection" in categories:\n                return PairConflictDetail(\n                    conflict_level="硬冲突",\n                    conflict_type="防护带冲突",\n                    resource_name=f"{window_a.throat}-{zone_label}",\n                    overlap_min=overlap,\n                    description="发车防护带锁闭区间发生重叠。",\n                )\n            return PairConflictDetail(\n                conflict_level="硬冲突",\n                conflict_type="咽喉能力冲突",\n                resource_name=f"{window_a.throat}-{zone_label}",\n                overlap_min=overlap,\n                description="发车出口咽喉能力区间发生重叠。",\n            )\n\n        if "route_lock" in categories:\n            return PairConflictDetail(\n                conflict_level="硬冲突",\n                conflict_type="进路交叉冲突",\n                resource_name=f"{window_a.throat}-{zone_label}",\n                overlap_min=overlap,\n                description="接发或发接方向在同一咽喉分区的锁闭区内发生交叉冲突。",\n            )\n        if "flank_protection" in categories:\n            return PairConflictDetail(\n                conflict_level="硬冲突",\n                conflict_type="防护带冲突",\n                resource_name=f"{window_a.throat}-{zone_label}",\n                overlap_min=overlap,\n                description="接发进路共享同一防护带资源。",\n            )\n        return PairConflictDetail(\n            conflict_level="硬冲突",\n            conflict_type="咽喉能力冲突",\n            resource_name=f"{window_a.throat}-{zone_label}",\n            overlap_min=overlap,\n            description="接发进路在同一咽喉能力区发生重叠。",\n        )\n\n    return None\n\n\ndef collect_pair_conflict_details(\n    option_a: CandidateRoutePlan,\n    option_b: CandidateRoutePlan,\n) -> list[PairConflictDetail]:\n    cache_key = option_pair_cache_key(option_a, option_b)\n    cached = PAIR_CONFLICT_DETAIL_CACHE.get(cache_key)\n    if cached is not None:\n        return list(cached)\n\n    aggregate: dict[tuple[str, str], PairConflictDetail] = {}\n    for window_a in option_a.resources:\n        for window_b in option_b.resources:\n            detail = classify_window_conflict(window_a, window_b)\n            if detail is None:\n                continue\n            key = (detail.conflict_type, detail.resource_name)\n            old_detail = aggregate.get(key)\n            if old_detail is None or detail.overlap_min > old_detail.overlap_min:\n                aggregate[key] = detail\n    result = tuple(aggregate.values())\n    if len(PAIR_CONFLICT_DETAIL_CACHE) < PAIR_CONFLICT_CACHE_LIMIT:\n        PAIR_CONFLICT_DETAIL_CACHE[cache_key] = result\n    return list(result)\n\n\ndef detail_penalty(detail: PairConflictDetail) -> float:\n    base, slope = CONFLICT_PENALTY[detail.conflict_type]\n    multiplier = 12.0 if detail.conflict_level == "硬冲突" else 1.0\n    return multiplier * (base + slope * detail.overlap_min)\n\n\ndef detail_penalty_variant(\n    detail: PairConflictDetail, hard_soft_enabled: bool = True\n) -> float:\n    base, slope = CONFLICT_PENALTY[detail.conflict_type]\n    multiplier = (\n        20.0 if (hard_soft_enabled and detail.conflict_level == "硬冲突") else 1.0\n    )\n    return multiplier * (base + slope * detail.overlap_min)\n\n\ndef conflict_entry_risk(conflict: ConflictEntry) -> float:\n    base, slope = CONFLICT_PENALTY[conflict.conflict_type]\n    multiplier = 20.0 if conflict.conflict_level == "硬冲突" else 1.0\n    return multiplier * (base + slope * conflict.overlap_min)\n\n\ndef build_weighted_linear_costs(\n    options_by_train: dict[str, list[CandidateRoutePlan]],\n    safety_weight: float,\n    stability_weight: float,\n) -> dict[str, float]:\n    linear_costs: dict[str, float] = {}\n    for options in options_by_train.values():\n        for option in options:\n            residual = (\n                option.linear_cost\n                - option.delay_risk_cost * 6.0\n                - option.stability_cost * 2.4\n                + option.balance_reward\n            )\n            linear_costs[option.option_id] = max(\n                0.0,\n                option.delay_risk_cost * 6.0 * safety_weight\n                + option.stability_cost * 2.4 * stability_weight\n                - option.balance_reward\n                + residual,\n            )\n    return linear_costs\n\n\ndef build_weighted_pair_costs(\n    pair_costs: dict[str, dict[str, float]],\n    safety_weight: float,\n) -> dict[str, dict[str, float]]:\n    if abs(safety_weight - 1.0) <= 1e-9:\n        return pair_costs\n    return {\n        option_id: {\n            other_id: value * safety_weight for other_id, value in neighbors.items()\n        }\n        for option_id, neighbors in pair_costs.items()\n    }\n\n\ndef simplify_options_for_ablation(\n    options_by_train: dict[str, list[CandidateRoutePlan]],\n) -> dict[str, list[CandidateRoutePlan]]:\n    simplified: dict[str, list[CandidateRoutePlan]] = {}\n    for train_id, options in options_by_train.items():\n        chosen: dict[tuple[str, str, str], CandidateRoutePlan] = {}\n        for option in sorted(\n            options,\n            key=lambda item: (\n                item.route_variant != "标准通道",\n                item.linear_cost,\n                item.candidate_rank,\n                item.option_id,\n            ),\n        ):\n            key = (option.track, option.in_throat, option.out_throat)\n            if key not in chosen:\n                chosen[key] = option\n        simplified[train_id] = list(chosen.values())\n    return simplified\n\n\ndef classify_window_conflict_coarse(\n    window_a: ResourceWindow, window_b: ResourceWindow\n) -> PairConflictDetail | None:\n    overlap = overlap_minutes(\n        window_a.start_min, window_a.end_min, window_b.start_min, window_b.end_min\n    )\n    if overlap <= 0:\n        return None\n\n    categories = {window_a.resource_category, window_b.resource_category}\n    if window_a.track and window_b.track and window_a.track == window_b.track:\n        if "terminal_op" in categories:\n            return PairConflictDetail(\n                "硬冲突",\n                "始发终到作业冲突",\n                f"股道 {window_a.track}",\n                overlap,\n                "同一股道上的始发/终到作业时段发生重叠。",\n            )\n        if categories & {"track", "track_access"}:\n            return PairConflictDetail(\n                "硬冲突",\n                "股道占用冲突",\n                f"股道 {window_a.track}",\n                overlap,\n                "同一股道的接入、停留或发车准备区间发生重叠。",\n            )\n\n    if (\n        window_a.throat\n        and window_a.throat == window_b.throat\n        and throat_zone_conflicts(window_a.throat, window_a.zone, window_b.zone)\n    ):\n        zone_label = (\n            window_a.zone\n            if window_a.zone == window_b.zone\n            else f"{window_a.zone}/{window_b.zone}"\n        )\n        movement_pair = (window_a.movement_family, window_b.movement_family)\n        if "switch_ladder" in categories:\n            return PairConflictDetail(\n                "硬冲突",\n                "道岔组冲突",\n                f"{window_a.throat}-{zone_label}",\n                overlap,\n                "同一粗分区或相邻粗分区的道岔组占用发生重叠。",\n            )\n        if movement_pair == ("接车", "接车"):\n            if "route_lock" in categories:\n                return PairConflictDetail(\n                    "硬冲突",\n                    "接车进路锁闭冲突",\n                    f"{window_a.throat}-{zone_label}",\n                    overlap,\n                    "接车咽喉粗分区的进路锁闭区间发生重叠。",\n                )\n            if "flank_protection" in categories:\n                return PairConflictDetail(\n                    "硬冲突",\n                    "防护带冲突",\n                    f"{window_a.throat}-{zone_label}",\n                    overlap,\n                    "接车防护带锁闭区间发生重叠。",\n                )\n            return PairConflictDetail(\n                "硬冲突",\n                "咽喉能力冲突",\n                f"{window_a.throat}-{zone_label}",\n                overlap,\n                "接车入口咽喉能力区间发生重叠。",\n            )\n        if movement_pair == ("发车", "发车"):\n            if "route_lock" in categories:\n                return PairConflictDetail(\n                    "硬冲突",\n                    "发车进路锁闭冲突",\n                    f"{window_a.throat}-{zone_label}",\n                    overlap,\n                    "发车咽喉粗分区的进路锁闭区间发生重叠。",\n                )\n            if "flank_protection" in categories:\n                return PairConflictDetail(\n                    "硬冲突",\n                    "防护带冲突",\n                    f"{window_a.throat}-{zone_label}",\n                    overlap,\n                    "发车防护带锁闭区间发生重叠。",\n                )\n            return PairConflictDetail(\n                "硬冲突",\n                "咽喉能力冲突",\n                f"{window_a.throat}-{zone_label}",\n                overlap,\n                "发车出口咽喉能力区间发生重叠。",\n            )\n        if "route_lock" in categories:\n            return PairConflictDetail(\n                "硬冲突",\n                "进路交叉冲突",\n                f"{window_a.throat}-{zone_label}",\n                overlap,\n                "接发或发接方向在粗分区锁闭区内发生交叉冲突。",\n            )\n        if "flank_protection" in categories:\n            return PairConflictDetail(\n                "硬冲突",\n                "防护带冲突",\n                f"{window_a.throat}-{zone_label}",\n                overlap,\n                "接发进路共享同一粗分区防护带资源。",\n            )\n        return PairConflictDetail(\n            "硬冲突",\n            "咽喉能力冲突",\n            f"{window_a.throat}-{zone_label}",\n            overlap,\n            "接发进路在同一粗分区咽喉能力区发生重叠。",\n        )\n    return None\n\n\ndef collect_pair_conflict_details_coarse(\n    option_a: CandidateRoutePlan,\n    option_b: CandidateRoutePlan,\n) -> list[PairConflictDetail]:\n    cache_key = option_pair_cache_key(option_a, option_b)\n    cached = PAIR_CONFLICT_DETAIL_COARSE_CACHE.get(cache_key)\n    if cached is not None:\n        return list(cached)\n\n    aggregate: dict[tuple[str, str], PairConflictDetail] = {}\n    for window_a in option_a.resources:\n        for window_b in option_b.resources:\n            detail = classify_window_conflict_coarse(window_a, window_b)\n            if detail is None:\n                continue\n            key = (detail.conflict_type, detail.resource_name)\n            old_detail = aggregate.get(key)\n            if old_detail is None or detail.overlap_min > old_detail.overlap_min:\n                aggregate[key] = detail\n    result = tuple(aggregate.values())\n    if len(PAIR_CONFLICT_DETAIL_COARSE_CACHE) < PAIR_CONFLICT_CACHE_LIMIT:\n        PAIR_CONFLICT_DETAIL_COARSE_CACHE[cache_key] = result\n    return list(result)\n\n\ndef build_pair_penalties_variant(\n    trains: list[TrainRecord],\n    options_by_train: dict[str, list[CandidateRoutePlan]],\n    mode: str,\n    hard_soft_enabled: bool = True,\n) -> dict[str, dict[str, float]]:\n    pair_costs, _, _ = build_pairwise_conflict_maps(\n        trains=trains,\n        options_by_train=options_by_train,\n        mode=mode,\n        hard_soft_enabled=hard_soft_enabled,\n        use_variant_penalty=True,\n    )\n    return pair_costs\n\n\ndef conflict_metric_bundle(conflicts: list[ConflictEntry]) -> dict[str, int]:\n    statistics_bundle = build_conflict_statistics(conflicts)\n    return {\n        "总冲突项": statistics_bundle["总冲突项"],\n        "硬冲突项": statistics_bundle["硬冲突项"],\n        "软冲突项": statistics_bundle["软冲突项"],\n        "联锁类冲突项": statistics_bundle["联锁类冲突项"],\n        "股道冲突项": statistics_bundle["股道占用冲突项"],\n        "股道占用冲突项": statistics_bundle["股道占用冲突项"],\n        "始发终到作业冲突项": statistics_bundle["始发终到作业冲突项"],\n        "进路交叉冲突项": statistics_bundle["进路交叉冲突项"],\n    }\n\n\ndef rebuild_option_for_train(\n    train: TrainRecord, option: CandidateRoutePlan\n) -> CandidateRoutePlan:\n    rebuilt_resources = build_resource_windows(\n        train=train,\n        option_id=option.option_id,\n        route_code=option.route_code,\n        track=option.track,\n        in_throat=option.in_throat,\n        out_throat=option.out_throat,\n        inbound_channel=option.inbound_channel,\n        outbound_channel=option.outbound_channel,\n    )\n    return CandidateRoutePlan(\n        option_id=option.option_id,\n        train_id=option.train_id,\n        track=option.track,\n        in_throat=option.in_throat,\n        out_throat=option.out_throat,\n        in_partition=option.in_partition,\n        out_partition=option.out_partition,\n        inbound_channel=option.inbound_channel,\n        outbound_channel=option.outbound_channel,\n        route_variant=option.route_variant,\n        route_code=option.route_code,\n        inbound_route_code=option.inbound_route_code,\n        outbound_route_code=option.outbound_route_code,\n        route_family=option.route_family,\n        source_level=option.source_level,\n        support_count=option.support_count,\n        route_score=option.route_score,\n        candidate_rank=option.candidate_rank,\n        operation_type=option.operation_type,\n        delay_risk_cost=option.delay_risk_cost,\n        stability_cost=option.stability_cost,\n        balance_reward=option.balance_reward,\n        linear_cost=option.linear_cost,\n        note=option.note,\n        resources=rebuilt_resources,\n    )\n\n\ndef rebuild_assignment_for_trains(\n    trains: list[TrainRecord],\n    assignment: dict[str, CandidateRoutePlan],\n) -> dict[str, CandidateRoutePlan]:\n    train_map = {train.record_id: train for train in trains}\n    return {\n        train_id: rebuild_option_for_train(train_map[train_id], option)\n        for train_id, option in assignment.items()\n    }\n\n\ndef rebuild_options_by_train(\n    trains: list[TrainRecord],\n    options_by_train: dict[str, list[CandidateRoutePlan]],\n) -> dict[str, list[CandidateRoutePlan]]:\n    train_map = {train.record_id: train for train in trains}\n    return {\n        train_id: [\n            rebuild_option_for_train(train_map[train_id], option) for option in options\n        ]\n        for train_id, options in options_by_train.items()\n    }\n\n\ndef build_pairwise_conflict_maps(\n    trains: list[TrainRecord],\n    options_by_train: dict[str, list[CandidateRoutePlan]],\n    mode: str = "full",\n    hard_soft_enabled: bool = True,\n    use_variant_penalty: bool = True,\n) -> tuple[\n    dict[str, dict[str, float]], dict[str, dict[str, int]], dict[str, dict[str, float]]\n]:\n    options_by_train = {\n        train_id: sorted(\n            options,\n            key=lambda option: (\n                option.candidate_rank,\n                option.linear_cost,\n                option.option_id,\n            ),\n        )[:40]\n        for train_id, options in options_by_train.items()\n    }\n    pair_costs: dict[str, dict[str, float]] = defaultdict(dict)\n    hard_counts: dict[str, dict[str, int]] = defaultdict(dict)\n    hard_risks: dict[str, dict[str, float]] = defaultdict(dict)\n\n    for left_index, train_a in enumerate(trains):\n        for right_index in range(left_index + 1, len(trains)):\n            train_b = trains[right_index]\n            broad_overlap = overlap_minutes(\n                train_a.window_start,\n                train_a.window_end,\n                train_b.window_start,\n                train_b.window_end,\n            )\n            if broad_overlap <= 0:\n                continue\n            for option_a in options_by_train[train_a.record_id]:\n                for option_b in options_by_train[train_b.record_id]:\n                    full_details = collect_pair_conflict_details(option_a, option_b)\n                    hard_details = [\n                        detail\n                        for detail in full_details\n                        if detail.conflict_level == "硬冲突"\n                    ]\n                    if hard_details:\n                        count = len(hard_details)\n                        risk = sum(detail_penalty(detail) for detail in hard_details)\n                        hard_counts[option_a.option_id][option_b.option_id] = count\n                        hard_counts[option_b.option_id][option_a.option_id] = count\n                        hard_risks[option_a.option_id][option_b.option_id] = risk\n                        hard_risks[option_b.option_id][option_a.option_id] = risk\n\n                    if mode == "coarse_partition":\n                        pair_details = collect_pair_conflict_details_coarse(\n                            option_a, option_b\n                        )\n                    elif mode == "track_only":\n                        pair_details = [\n                            detail\n                            for detail in full_details\n                            if detail.conflict_type\n                            in {"股道占用冲突", "始发终到作业冲突"}\n                        ]\n                    else:\n                        pair_details = full_details\n                    if not pair_details:\n                        continue\n                    if use_variant_penalty:\n                        penalty = sum(\n                            detail_penalty_variant(\n                                detail, hard_soft_enabled=hard_soft_enabled\n                            )\n                            for detail in pair_details\n                        )\n                    else:\n                        penalty = sum(detail_penalty(detail) for detail in pair_details)\n                    pair_costs[option_a.option_id][option_b.option_id] = penalty\n                    pair_costs[option_b.option_id][option_a.option_id] = penalty\n\n    return dict(pair_costs), dict(hard_counts), dict(hard_risks)\n\n\ndef build_pair_penalties(\n    trains: list[TrainRecord],\n    options_by_train: dict[str, list[CandidateRoutePlan]],\n) -> dict[str, dict[str, float]]:\n    pair_costs, _, _ = build_pairwise_conflict_maps(\n        trains=trains,\n        options_by_train=options_by_train,\n        mode="full",\n        hard_soft_enabled=True,\n        use_variant_penalty=False,\n    )\n    return pair_costs\n\n\ndef build_hard_conflict_maps(\n    trains: list[TrainRecord],\n    options_by_train: dict[str, list[CandidateRoutePlan]],\n) -> tuple[dict[str, dict[str, int]], dict[str, dict[str, float]]]:\n    _, hard_counts, hard_risks = build_pairwise_conflict_maps(\n        trains=trains,\n        options_by_train=options_by_train,\n        mode="full",\n        hard_soft_enabled=True,\n    )\n    return hard_counts, hard_risks\n\n\ndef compute_option_cost(\n    train_id: str,\n    option_id: str,\n    selected: dict[str, str],\n    linear_costs: dict[str, float],\n    pair_costs: dict[str, dict[str, float]],\n    hard_counts: dict[str, dict[str, int]],\n    hard_risks: dict[str, dict[str, float]],\n) -> float:\n    total = linear_costs[option_id]\n    neighbors = pair_costs.get(option_id, {})\n    hard_count_total = 0\n    hard_risk_total = 0.0\n    for other_train_id, other_option_id in selected.items():\n        if other_train_id == train_id:\n            continue\n        total += neighbors.get(other_option_id, 0.0)\n        hard_count_total += hard_counts.get(option_id, {}).get(other_option_id, 0)\n        hard_risk_total += hard_risks.get(option_id, {}).get(other_option_id, 0.0)\n    return (\n        hard_count_total * HARD_CONFLICT_PRIORITY\n        + hard_risk_total * HARD_RISK_PRIORITY\n        + total\n    )\n\n\ndef assignment_energy(\n    selected: dict[str, str],\n    linear_costs: dict[str, float],\n    pair_costs: dict[str, dict[str, float]],\n    hard_counts: dict[str, dict[str, int]],\n    hard_risks: dict[str, dict[str, float]],\n) -> float:\n    total = 0.0\n    hard_count_total = 0\n    hard_risk_total = 0.0\n    train_ids = sorted(selected)\n    for index, train_id in enumerate(train_ids):\n        option_id = selected[train_id]\n        total += linear_costs[option_id]\n        for other_id in train_ids[index + 1 :]:\n            other_option_id = selected[other_id]\n            total += pair_costs.get(option_id, {}).get(other_option_id, 0.0)\n            hard_count_total += hard_counts.get(option_id, {}).get(other_option_id, 0)\n            hard_risk_total += hard_risks.get(option_id, {}).get(other_option_id, 0.0)\n    return (\n        hard_count_total * HARD_CONFLICT_PRIORITY\n        + hard_risk_total * HARD_RISK_PRIORITY\n        + total\n    )\n\n\ndef build_train_order(\n    trains: list[TrainRecord], options_by_train: dict[str, list[CandidateRoutePlan]]\n) -> list[str]:\n    overlap_count = {train.record_id: 0 for train in trains}\n    for left_index, train_a in enumerate(trains):\n        for right_index in range(left_index + 1, len(trains)):\n            train_b = trains[right_index]\n            if (\n                overlap_minutes(\n                    train_a.window_start,\n                    train_a.window_end,\n                    train_b.window_start,\n                    train_b.window_end,\n                )\n                > 0\n            ):\n                overlap_count[train_a.record_id] += 1\n                overlap_count[train_b.record_id] += 1\n    ordered = sorted(\n        trains,\n        key=lambda train: (\n            -overlap_count[train.record_id],\n            len(options_by_train[train.record_id]),\n            train.arrival_min,\n            train.index,\n        ),\n    )\n    return [train.record_id for train in ordered]\n\n\ndef build_high_pressure_train_ids(\n    trains: list[TrainRecord], limit: int = 18\n) -> set[str]:\n    overlap_count = {train.record_id: 0 for train in trains}\n    for left_index, train_a in enumerate(trains):\n        for right_index in range(left_index + 1, len(trains)):\n            train_b = trains[right_index]\n            if (\n                overlap_minutes(\n                    train_a.window_start,\n                    train_a.window_end,\n                    train_b.window_start,\n                    train_b.window_end,\n                )\n                > 0\n            ):\n                overlap_count[train_a.record_id] += 1\n                overlap_count[train_b.record_id] += 1\n    ordered = sorted(\n        trains,\n        key=lambda train: (\n            -overlap_count[train.record_id],\n            train.arrival_min,\n            train.index,\n        ),\n    )\n    return {train.record_id for train in ordered[: max(0, limit)]}\n\n\ndef softmax(values: list[float]) -> list[float]:\n    peak = max(values)\n    exps = [math.exp(max(-50.0, min(50.0, value - peak))) for value in values]\n    total = sum(exps)\n    if total <= 0:\n        return [1.0 / len(values)] * len(values)\n    return [item / total for item in exps]\n\n\ndef sample_index(probabilities: list[float], rng: random.Random) -> int:\n    threshold = rng.random()\n    cumulative = 0.0\n    for index, probability in enumerate(probabilities):\n        cumulative += probability\n        if threshold <= cumulative:\n            return index\n    return len(probabilities) - 1\n\n\ndef build_linear_norms(\n    options_by_train: dict[str, list[CandidateRoutePlan]],\n) -> dict[str, list[float]]:\n    normalized: dict[str, list[float]] = {}\n    for train_id, options in options_by_train.items():\n        values = [option.linear_cost for option in options]\n        max_value = max(values) if values else 1.0\n        denominator = max(1.0, max_value)\n        normalized[train_id] = [value / denominator for value in values]\n    return normalized\n\n\ndef build_assignment_lookup(\n    selected_option_ids: dict[str, str],\n    options_by_train: dict[str, list[CandidateRoutePlan]],\n) -> dict[str, CandidateRoutePlan]:\n    option_lookup = {\n        option.option_id: option\n        for options in options_by_train.values()\n        for option in options\n    }\n    return {\n        train_id: option_lookup[option_id]\n        for train_id, option_id in selected_option_ids.items()\n    }\n\n\ndef greedy_descent(\n    selected: dict[str, str],\n    train_order: list[str],\n    options_by_train: dict[str, list[CandidateRoutePlan]],\n    linear_costs: dict[str, float],\n    pair_costs: dict[str, dict[str, float]],\n    hard_counts: dict[str, dict[str, int]],\n    hard_risks: dict[str, dict[str, float]],\n    train_map: dict[str, TrainRecord],\n    max_passes: int = 12,\n    prefer_reassignment_train_ids: set[str] | None = None,\n) -> dict[str, str]:\n    option_lookup = {\n        option.option_id: option\n        for options in options_by_train.values()\n        for option in options\n    }\n    for _ in range(max(1, max_passes)):\n        changed = False\n        for train_id in train_order:\n            current_option_id = selected[train_id]\n            best_option_id = current_option_id\n            best_cost = compute_option_cost(\n                train_id,\n                current_option_id,\n                selected,\n                linear_costs,\n                pair_costs,\n                hard_counts,\n                hard_risks,\n            )\n\n            for option in options_by_train[train_id]:\n                if option.option_id == current_option_id:\n                    continue\n                candidate_cost = compute_option_cost(\n                    train_id,\n                    option.option_id,\n                    selected,\n                    linear_costs,\n                    pair_costs,\n                    hard_counts,\n                    hard_risks,\n                )\n                if candidate_cost + 1e-9 < best_cost:\n                    best_cost = candidate_cost\n                    best_option_id = option.option_id\n                elif abs(candidate_cost - best_cost) <= 1e-9:\n                    best_option = option_lookup[best_option_id]\n                    train = train_map[train_id]\n                    if (\n                        prefer_reassignment_train_ids\n                        and train_id in prefer_reassignment_train_ids\n                    ):\n                        candidate_key = (\n                            option.track == train.planned_track\n                            and option.in_throat == train.in_throat\n                            and option.out_throat == train.out_throat,\n                            option.delay_risk_cost,\n                            SOURCE_PRIORITY.get(option.source_level, 99),\n                            option.option_id,\n                        )\n                        best_key = (\n                            best_option.track == train.planned_track\n                            and best_option.in_throat == train.in_throat\n                            and best_option.out_throat == train.out_throat,\n                            best_option.delay_risk_cost,\n                            SOURCE_PRIORITY.get(best_option.source_level, 99),\n                            best_option.option_id,\n                        )\n                    else:\n                        candidate_key = (\n                            track_distance(option.track, train.planned_track),\n                            option.track != train.planned_track,\n                            SOURCE_PRIORITY.get(option.source_level, 99),\n                            option.option_id,\n                        )\n                        best_key = (\n                            track_distance(best_option.track, train.planned_track),\n                            best_option.track != train.planned_track,\n                            SOURCE_PRIORITY.get(best_option.source_level, 99),\n                            best_option.option_id,\n                        )\n                    if candidate_key < best_key:\n                        best_option_id = option.option_id\n\n            if best_option_id != current_option_id:\n                selected[train_id] = best_option_id\n                changed = True\n        if not changed:\n            break\n    return selected\n\n\ndef conflict_driven_joint_local_search(\n    selected: dict[str, str],\n    options_by_train: dict[str, list[CandidateRoutePlan]],\n    linear_costs: dict[str, float],\n    pair_costs: dict[str, dict[str, float]],\n    hard_counts: dict[str, dict[str, int]],\n    hard_risks: dict[str, dict[str, float]],\n    train_map: dict[str, TrainRecord],\n    max_rounds: int = 8,\n    prefer_reassignment_train_ids: set[str] | None = None,\n) -> dict[str, str]:\n    option_to_train = {\n        option.option_id: train_id\n        for train_id, options in options_by_train.items()\n        for option in options\n    }\n    option_lookup = {\n        option.option_id: option\n        for options in options_by_train.values()\n        for option in options\n    }\n\n    def train_pressure(\n        train_id: str, selected_now: dict[str, str]\n    ) -> tuple[int, float]:\n        option_id = selected_now[train_id]\n        hard_total = 0\n        risk_total = 0.0\n        for other_train_id, other_option_id in selected_now.items():\n            if other_train_id == train_id:\n                continue\n            hard_total += hard_counts.get(option_id, {}).get(other_option_id, 0)\n            risk_total += pair_costs.get(option_id, {}).get(other_option_id, 0.0)\n        return hard_total, risk_total\n\n    for _ in range(max(1, max_rounds)):\n        ordered_trains = sorted(\n            selected,\n            key=lambda train_id: (\n                -train_pressure(train_id, selected)[0],\n                -train_pressure(train_id, selected)[1],\n                train_map[train_id].arrival_min,\n                train_map[train_id].index,\n            ),\n        )\n        changed = False\n        for train_id in ordered_trains:\n            current_option_id = selected[train_id]\n            current_signature = (\n                train_pressure(train_id, selected)[0],\n                compute_option_cost(\n                    train_id,\n                    current_option_id,\n                    selected,\n                    linear_costs,\n                    pair_costs,\n                    hard_counts,\n                    hard_risks,\n                ),\n            )\n            best_option_id = current_option_id\n            best_signature = current_signature\n            for option in options_by_train[train_id]:\n                option_id = option.option_id\n                if option_id == current_option_id:\n                    continue\n                selected[train_id] = option_id\n                signature = (\n                    train_pressure(train_id, selected)[0],\n                    compute_option_cost(\n                        train_id,\n                        option_id,\n                        selected,\n                        linear_costs,\n                        pair_costs,\n                        hard_counts,\n                        hard_risks,\n                    ),\n                )\n                if signature < best_signature:\n                    best_signature = signature\n                    best_option_id = option_id\n                elif signature == best_signature:\n                    candidate = option_lookup[option_id]\n                    incumbent = option_lookup[best_option_id]\n                    train = train_map[train_id]\n                    if (\n                        prefer_reassignment_train_ids\n                        and train_id in prefer_reassignment_train_ids\n                    ):\n                        candidate_key = (\n                            candidate.track == train.planned_track\n                            and candidate.in_throat == train.in_throat\n                            and candidate.out_throat == train.out_throat,\n                            candidate.delay_risk_cost,\n                            candidate.option_id,\n                        )\n                        incumbent_key = (\n                            incumbent.track == train.planned_track\n                            and incumbent.in_throat == train.in_throat\n                            and incumbent.out_throat == train.out_throat,\n                            incumbent.delay_risk_cost,\n                            incumbent.option_id,\n                        )\n                    else:\n                        candidate_key = (\n                            candidate.delay_risk_cost,\n                            track_distance(candidate.track, train.planned_track),\n                            candidate.option_id,\n                        )\n                        incumbent_key = (\n                            incumbent.delay_risk_cost,\n                            track_distance(incumbent.track, train.planned_track),\n                            incumbent.option_id,\n                        )\n                    if candidate_key < incumbent_key:\n                        best_option_id = option_id\n            selected[train_id] = best_option_id\n            if best_option_id != current_option_id:\n                changed = True\n\n        if not changed:\n            break\n\n    return selected\n\n\ndef build_preference_logits(\n    assignment: dict[str, str],\n    options_by_train: dict[str, list[CandidateRoutePlan]],\n    hot_value: float,\n    cold_value: float,\n) -> dict[str, list[float]]:\n    return {\n        train_id: [\n            hot_value if option.option_id == assignment[train_id] else cold_value\n            for option in options_by_train[train_id]\n        ]\n        for train_id in assignment\n    }\n\n\n\n\n\n\ndef build_planned_assignment_ids(\n    trains: list[TrainRecord],\n    options_by_train: dict[str, list[CandidateRoutePlan]],\n) -> dict[str, str]:\n    assignment: dict[str, str] = {}\n    for train in trains:\n        planned_option = next(\n            (\n                option\n                for option in options_by_train[train.record_id]\n                if option.track == train.planned_track\n                and option.in_throat == train.in_throat\n                and option.out_throat == train.out_throat\n            ),\n            options_by_train[train.record_id][0],\n        )\n        assignment[train.record_id] = planned_option.option_id\n    return assignment\n\n\n\n\ndef collect_conflicts(\n    trains: list[TrainRecord],\n    assignment: dict[str, CandidateRoutePlan],\n    scheme: str,\n) -> list[ConflictEntry]:\n    conflicts: list[ConflictEntry] = []\n    for left_index, train_a in enumerate(trains):\n        option_a = assignment[train_a.record_id]\n        for right_index in range(left_index + 1, len(trains)):\n            train_b = trains[right_index]\n            option_b = assignment[train_b.record_id]\n            detail_list = collect_pair_conflict_details(option_a, option_b)\n            for detail in detail_list:\n                conflicts.append(\n                    ConflictEntry(\n                        scheme=scheme,\n                        conflict_level=detail.conflict_level,\n                        conflict_type=detail.conflict_type,\n                        train1=f"{train_a.record_id}-{train_a.train_no}",\n                        train2=f"{train_b.record_id}-{train_b.train_no}",\n                        route1=option_a.route_code,\n                        route2=option_b.route_code,\n                        resource=detail.resource_name,\n                        overlap_min=detail.overlap_min,\n                        interval1=format_interval(\n                            train_a.window_start, train_a.window_end\n                        ),\n                        interval2=format_interval(\n                            train_b.window_start, train_b.window_end\n                        ),\n                        track1=option_a.track,\n                        track2=option_b.track,\n                        description=detail.description,\n                    )\n                )\n    return conflicts\n\n\ndef summarize_conflicts(conflicts: list[ConflictEntry]) -> dict[str, dict[str, int]]:\n    summary: dict[str, dict[str, int]] = defaultdict(\n        lambda: {"数量": 0, "重叠分钟合计": 0}\n    )\n    for conflict in conflicts:\n        summary[conflict.conflict_type]["数量"] += 1\n        summary[conflict.conflict_type]["重叠分钟合计"] += conflict.overlap_min\n    return dict(summary)\n\n\nNON_INTERLOCKING_CONFLICT_TYPES = frozenset({"股道占用冲突", "始发终到作业冲突"})\n\n\ndef build_conflict_statistics(conflicts: list[ConflictEntry]) -> dict[str, int]:\n    summary = summarize_conflicts(conflicts)\n    hard_conflicts = [\n        conflict for conflict in conflicts if conflict.conflict_level == "硬冲突"\n    ]\n    soft_conflicts = [\n        conflict for conflict in conflicts if conflict.conflict_level == "软冲突"\n    ]\n\n    def stat_value(conflict_type: str, field: str) -> int:\n        return summary.get(conflict_type, {"数量": 0, "重叠分钟合计": 0})[field]\n\n    return {\n        "总冲突项": len(conflicts),\n        "总重叠分钟": sum(conflict.overlap_min for conflict in conflicts),\n        "硬冲突项": len(hard_conflicts),\n        "硬冲突重叠分钟": sum(conflict.overlap_min for conflict in hard_conflicts),\n        "软冲突项": len(soft_conflicts),\n        "软冲突重叠分钟": sum(conflict.overlap_min for conflict in soft_conflicts),\n        "联锁类冲突项": sum(\n            value["数量"]\n            for key, value in summary.items()\n            if key not in NON_INTERLOCKING_CONFLICT_TYPES\n        ),\n        "联锁类重叠分钟": sum(\n            value["重叠分钟合计"]\n            for key, value in summary.items()\n            if key not in NON_INTERLOCKING_CONFLICT_TYPES\n        ),\n        "股道占用冲突项": stat_value("股道占用冲突", "数量"),\n        "股道占用重叠分钟": stat_value("股道占用冲突", "重叠分钟合计"),\n        "始发终到作业冲突项": stat_value("始发终到作业冲突", "数量"),\n        "始发终到作业重叠分钟": stat_value("始发终到作业冲突", "重叠分钟合计"),\n        "进路交叉冲突项": stat_value("进路交叉冲突", "数量"),\n        "进路交叉重叠分钟": stat_value("进路交叉冲突", "重叠分钟合计"),\n    }\n\n\ndef build_qubo_rows(\n    trains: list[TrainRecord],\n    options_by_train: dict[str, list[CandidateRoutePlan]],\n    pair_costs: dict[str, dict[str, float]],\n) -> tuple[list[dict[str, object]], list[dict[str, object]]]:\n    one_hot_penalty = 6500.0\n    variable_rows: list[dict[str, object]] = []\n    qubo_rows: list[dict[str, object]] = []\n\n    for train in trains:\n        options = options_by_train[train.record_id]\n        for option in options:\n            variable_rows.append(\n                {\n                    "变量名": option.option_id,\n                    "列车记录ID": train.record_id,\n                    "车次": train.train_no,\n                    "trip_id": train.trip_id,\n                    "候选股道": option.track,\n                    "接车侧咽喉": option.in_throat,\n                    "发车侧咽喉": option.out_throat,\n                    "接车分区": option.in_partition,\n                    "发车分区": option.out_partition,\n                    "接车通道": option.inbound_channel,\n                    "发车通道": option.outbound_channel,\n                    "进路变体": option.route_variant,\n                    "进路代码": option.route_code,\n                    "进路族": option.route_family,\n                    "候选来源": option.source_level,\n                    "支撑样本数": option.support_count,\n                    "作业类型": option.operation_type,\n                    "延误风险代理": f"{option.delay_risk_cost:.2f}",\n                    "稳定性代价": f"{option.stability_cost:.2f}",\n                    "均衡奖励": f"{option.balance_reward:.2f}",\n                    "线性代价": f"{option.linear_cost:.2f}",\n                    "说明": option.note,\n                }\n            )\n            qubo_rows.append(\n                {\n                    "变量1": option.option_id,\n                    "变量2": option.option_id,\n                    "系数": f"{(-one_hot_penalty + option.linear_cost):.2f}",\n                    "系数类型": "线性项",\n                    "说明": "唯一分配约束展开后的线性项 + 进路变更代价项。",\n                }\n            )\n\n        for left_index in range(len(options)):\n            for right_index in range(left_index + 1, len(options)):\n                qubo_rows.append(\n                    {\n                        "变量1": options[left_index].option_id,\n                        "变量2": options[right_index].option_id,\n                        "系数": f"{(2 * one_hot_penalty):.2f}",\n                        "系数类型": "同车互斥项",\n                        "说明": f"{train.record_id}-{train.train_no} 只能选择一条候选工程进路。",\n                    }\n                )\n\n    option_to_train = {\n        option.option_id: train_id\n        for train_id, options in options_by_train.items()\n        for option in options\n    }\n    emitted_pairs: set[tuple[str, str]] = set()\n    for option_id, neighbors in pair_costs.items():\n        for other_option_id, penalty in neighbors.items():\n            if option_to_train[option_id] == option_to_train[other_option_id]:\n                continue\n            key = tuple(sorted((option_id, other_option_id)))\n            if key in emitted_pairs:\n                continue\n            emitted_pairs.add(key)\n            qubo_rows.append(\n                {\n                    "变量1": key[0],\n                    "变量2": key[1],\n                    "系数": f"{penalty:.2f}",\n                    "系数类型": "联锁冲突惩罚项",\n                    "说明": "两条工程进路同时取值时，联锁资源冲突将以惩罚项写入 QUBO。",\n                }\n            )\n\n    return variable_rows, qubo_rows\n\n\ndef resolve_output_path(path: Path) -> Path:\n    if not path.exists():\n        return path\n    for index in range(1, 100):\n        candidate = path.with_name(f"{path.stem}_新版{index}{path.suffix}")\n        if not candidate.exists():\n            return candidate\n    raise RuntimeError(f"无法为输出文件生成可写的新文件名：{path}")\n\n\ndef write_csv(path: Path, rows: list[dict[str, object]], fieldnames: list[str]) -> Path:\n    target = path\n    while True:\n        try:\n            with target.open("w", encoding="utf-8-sig", newline="") as handle:\n                writer = csv.DictWriter(\n                    handle, fieldnames=fieldnames, extrasaction="ignore"\n                )\n                writer.writeheader()\n                writer.writerows(\n                    [\n                        {\n                            field: "" if row.get(field) is None else row.get(field)\n                            for field in fieldnames\n                        }\n                        for row in rows\n                    ]\n                )\n            return target\n        except PermissionError:\n            target = resolve_output_path(target)\n\n\nSVG_COLORS = (\n    "#1f77b4",\n    "#ff7f0e",\n    "#2ca02c",\n    "#d62728",\n    "#9467bd",\n    "#8c564b",\n    "#e377c2",\n    "#7f7f7f",\n)\nSVG_FONT_FAMILY = "\'Microsoft YaHei\', \'SimHei\', Arial, sans-serif"\n\n\ndef write_text_file(path: Path, content: str) -> Path:\n    target = path\n    while True:\n        try:\n            with target.open("w", encoding="utf-8") as handle:\n                handle.write(content)\n            return target\n        except PermissionError:\n            target = resolve_output_path(target)\n\n\ndef load_chart_font(size: int, bold: bool = False) -> ImageFont.ImageFont:\n    candidate_paths = []\n    if bold:\n        candidate_paths.extend(\n            [\n                Path(r"C:\\Windows\\Fonts\\msyhbd.ttc"),\n                Path(r"C:\\Windows\\Fonts\\simhei.ttf"),\n                Path(r"C:\\Windows\\Fonts\\arialbd.ttf"),\n            ]\n        )\n    candidate_paths.extend(\n        [\n            Path(r"C:\\Windows\\Fonts\\msyh.ttc"),\n            Path(r"C:\\Windows\\Fonts\\simhei.ttf"),\n            Path(r"C:\\Windows\\Fonts\\arial.ttf"),\n        ]\n    )\n    for font_path in candidate_paths:\n        if font_path.exists():\n            try:\n                return ImageFont.truetype(str(font_path), size=size)\n            except OSError:\n                continue\n    return ImageFont.load_default()\n\n\ndef svg_number(value: str | None, default: float = 0.0) -> float:\n    if not value:\n        return default\n    match = re.search(r"[-+]?\\d+(?:\\.\\d+)?", value)\n    return float(match.group(0)) if match else default\n\n\ndef svg_color(value: str | None, default: str = "#000000") -> tuple[int, int, int, int]:\n    if not value or value.lower() == "none":\n        if default.lower() == "none":\n            return (0, 0, 0, 0)\n        value = default\n    rgb = ImageColor.getrgb(value)\n    return (rgb[0], rgb[1], rgb[2], 255)\n\n\ndef image_text_size(\n    draw: ImageDraw.ImageDraw, text: str, font: ImageFont.ImageFont\n) -> tuple[int, int]:\n    bbox = draw.textbbox((0, 0), text, font=font)\n    return max(1, bbox[2] - bbox[0]), max(1, bbox[3] - bbox[1])\n\n\ndef draw_svg_text(\n    image: Image.Image,\n    x: float,\n    y: float,\n    text: str,\n    size: int,\n    anchor: str,\n    weight: str,\n    fill: str,\n    rotation: float = 0.0,\n) -> None:\n    if not text:\n        return\n    font = load_chart_font(size, bold=(weight == "bold"))\n    scratch = Image.new("RGBA", (8, 8), (255, 255, 255, 0))\n    scratch_draw = ImageDraw.Draw(scratch)\n    text_w, text_h = image_text_size(scratch_draw, text, font)\n    if anchor == "middle":\n        left = x - text_w / 2\n    elif anchor == "end":\n        left = x - text_w\n    else:\n        left = x\n    top = y - text_h * 0.85\n    if abs(rotation) <= 1e-6:\n        ImageDraw.Draw(image).text((left, top), text, font=font, fill=svg_color(fill))\n        return\n    padding = 8\n    text_image = Image.new(\n        "RGBA", (text_w + padding * 2, text_h + padding * 2), (255, 255, 255, 0)\n    )\n    ImageDraw.Draw(text_image).text(\n        (padding, padding), text, font=font, fill=svg_color(fill)\n    )\n    rotation_center_x = left + text_w / 2\n    rotation_center_y = top + text_h / 2\n    rotated = text_image.rotate(-rotation, expand=True, resample=BICUBIC_RESAMPLE)\n    paste_x = int(round(rotation_center_x - rotated.width / 2))\n    paste_y = int(round(rotation_center_y - rotated.height / 2))\n    image.alpha_composite(rotated, (paste_x, paste_y))\n\n\ndef svg_file_to_png(svg_path: Path, png_path: Path) -> Path:\n    tree = ET.parse(svg_path)\n    root = tree.getroot()\n    width = int(round(svg_number(root.attrib.get("width"), 1200)))\n    height = int(round(svg_number(root.attrib.get("height"), 800)))\n    image = Image.new("RGBA", (width, height), (255, 255, 255, 255))\n    draw = ImageDraw.Draw(image)\n    namespace_suffix = "}"\n    for node in root:\n        tag = node.tag.split(namespace_suffix)[-1]\n        if tag == "rect":\n            x = svg_number(node.attrib.get("x"))\n            y = svg_number(node.attrib.get("y"))\n            w = svg_number(node.attrib.get("width"))\n            h = svg_number(node.attrib.get("height"))\n            rx = int(round(svg_number(node.attrib.get("rx"), 0.0)))\n            fill = svg_color(node.attrib.get("fill"), "#ffffff")\n            stroke = node.attrib.get("stroke")\n            stroke_fill = svg_color(stroke, "none") if stroke else None\n            stroke_width = (\n                int(round(svg_number(node.attrib.get("stroke-width"), 1.0)))\n                if stroke\n                else 0\n            )\n            xy = [x, y, x + w, y + h]\n            if rx > 0 and hasattr(draw, "rounded_rectangle"):\n                draw.rounded_rectangle(\n                    xy, radius=rx, fill=fill, outline=stroke_fill, width=stroke_width\n                )\n            else:\n                draw.rectangle(xy, fill=fill, outline=stroke_fill, width=stroke_width)\n        elif tag == "line":\n            x1 = svg_number(node.attrib.get("x1"))\n            y1 = svg_number(node.attrib.get("y1"))\n            x2 = svg_number(node.attrib.get("x2"))\n            y2 = svg_number(node.attrib.get("y2"))\n            stroke = svg_color(node.attrib.get("stroke"), "#000000")\n            stroke_width = int(round(svg_number(node.attrib.get("stroke-width"), 1.0)))\n            draw.line((x1, y1, x2, y2), fill=stroke, width=stroke_width)\n        elif tag == "text":\n            x = svg_number(node.attrib.get("x"))\n            y = svg_number(node.attrib.get("y"))\n            size = int(round(svg_number(node.attrib.get("font-size"), 14)))\n            anchor = node.attrib.get("text-anchor", "start")\n            weight = node.attrib.get("font-weight", "normal")\n            fill = node.attrib.get("fill", "#1f2937")\n            transform = node.attrib.get("transform", "")\n            rotation = 0.0\n            rotate_match = re.search(r"rotate\\(([-+]?\\d+(?:\\.\\d+)?)", transform)\n            if rotate_match:\n                rotation = float(rotate_match.group(1))\n            draw_svg_text(\n                image,\n                x,\n                y,\n                node.text or "",\n                size,\n                anchor,\n                weight,\n                fill,\n                rotation=rotation,\n            )\n    target = png_path\n    while True:\n        try:\n            image.save(target, format="PNG")\n            return target\n        except PermissionError:\n            target = resolve_output_path(target)\n\n\ndef extract_visual_number(value: object) -> float | None:\n    if value is None:\n        return None\n    if isinstance(value, (int, float)):\n        return float(value)\n    text = str(value).strip()\n    if not text or text == "--":\n        return None\n    match = re.search(r"[-+]?\\d+(?:\\.\\d+)?", text.replace(",", ""))\n    if not match:\n        return None\n    return float(match.group(0))\n\n\ndef nice_axis_max(max_value: float) -> float:\n    if max_value <= 0:\n        return 1.0\n    magnitude = 10 ** math.floor(math.log10(max_value))\n    normalized = max_value / magnitude\n    if normalized <= 1:\n        return 1.0 * magnitude\n    if normalized <= 2:\n        return 2.0 * magnitude\n    if normalized <= 5:\n        return 5.0 * magnitude\n    return 10.0 * magnitude\n\n\ndef svg_text(\n    x: float,\n    y: float,\n    text: str,\n    size: int = 14,\n    anchor: str = "start",\n    weight: str = "normal",\n    fill: str = "#1f2937",\n) -> str:\n    return (\n        f\'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" text-anchor="{anchor}" \'\n        f\'font-family="{SVG_FONT_FAMILY}" font-weight="{weight}" fill="{fill}">{escape(text)}</text>\'\n    )\n\n\ndef svg_grouped_bar_panel(\n    x: float,\n    y: float,\n    width: float,\n    height: float,\n    title: str,\n    categories: list[str],\n    series: list[tuple[str, list[float]]],\n) -> str:\n    left_margin = 78\n    right_margin = 24\n    top_margin = 52\n    bottom_margin = 76\n    plot_x = x + left_margin\n    plot_y = y + top_margin\n    plot_w = width - left_margin - right_margin\n    plot_h = height - top_margin - bottom_margin\n    max_value = max((value for _, values in series for value in values), default=0.0)\n    axis_max = nice_axis_max(max_value * 1.1 if max_value > 0 else 1.0)\n    elements = [\n        f\'<rect x="{x:.1f}" y="{y:.1f}" width="{width:.1f}" height="{height:.1f}" rx="14" fill="#ffffff" stroke="#d7dee8"/>\',\n        svg_text(x + 18, y + 30, title, size=18, weight="bold"),\n    ]\n    for tick_index in range(5):\n        tick_value = axis_max * tick_index / 4\n        tick_y = plot_y + plot_h - plot_h * tick_index / 4\n        elements.append(\n            f\'<line x1="{plot_x:.1f}" y1="{tick_y:.1f}" x2="{plot_x + plot_w:.1f}" y2="{tick_y:.1f}" stroke="#e5e7eb" stroke-width="1"/>\'\n        )\n        elements.append(\n            svg_text(\n                plot_x - 10,\n                tick_y + 5,\n                f"{tick_value:.0f}",\n                size=11,\n                anchor="end",\n                fill="#6b7280",\n            )\n        )\n    elements.append(\n        f\'<line x1="{plot_x:.1f}" y1="{plot_y + plot_h:.1f}" x2="{plot_x + plot_w:.1f}" y2="{plot_y + plot_h:.1f}" stroke="#374151" stroke-width="1.2"/>\'\n    )\n    series_count = max(1, len(series))\n    category_slot = plot_w / max(1, len(categories))\n    bar_group_width = category_slot * 0.72\n    bar_width = min(28.0, bar_group_width / series_count)\n    for category_index, category in enumerate(categories):\n        base_x = (\n            plot_x\n            + category_slot * category_index\n            + (category_slot - bar_group_width) / 2\n        )\n        label_x = plot_x + category_slot * (category_index + 0.5)\n        for series_index, (_, values) in enumerate(series):\n            value = values[category_index]\n            bar_height = 0.0 if axis_max <= 0 else plot_h * value / axis_max\n            bar_x = base_x + series_index * bar_width\n            bar_y = plot_y + plot_h - bar_height\n            color = SVG_COLORS[series_index % len(SVG_COLORS)]\n            elements.append(\n                f\'<rect x="{bar_x:.1f}" y="{bar_y:.1f}" width="{max(4.0, bar_width - 4):.1f}" height="{bar_height:.1f}" fill="{color}" rx="3"/>\'\n            )\n        elements.append(\n            f\'<text x="{label_x:.1f}" y="{plot_y + plot_h + 22:.1f}" font-size="11" text-anchor="end" \'\n            f\'font-family="{SVG_FONT_FAMILY}" fill="#374151" transform="rotate(-25 {label_x:.1f},{plot_y + plot_h + 22:.1f})">{escape(category)}</text>\'\n        )\n    legend_x = x + 18\n    legend_y = y + height - 20\n    for series_index, (series_name, _) in enumerate(series):\n        color = SVG_COLORS[series_index % len(SVG_COLORS)]\n        item_x = legend_x + series_index * 170\n        elements.append(\n            f\'<rect x="{item_x:.1f}" y="{legend_y - 10:.1f}" width="12" height="12" fill="{color}" rx="2"/>\'\n        )\n        elements.append(\n            svg_text(item_x + 18, legend_y, series_name, size=11, fill="#4b5563")\n        )\n    return "".join(elements)\n\n\ndef build_svg_document(\n    title: str, subtitle: str, panels: list[str], width: int, height: int\n) -> str:\n    return (\n        f\'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">\'\n        f\'<rect width="{width}" height="{height}" fill="#f8fafc"/>\'\n        f"{svg_text(32, 40, title, size=24, weight=\'bold\')}"\n        f"{svg_text(32, 66, subtitle, size=12, fill=\'#6b7280\')}"\n        f"{\'\'.join(panels)}"\n        "</svg>"\n    )\n\n\ndef build_comparison_visualization(rows: list[dict[str, object]]) -> str | None:\n    if not rows:\n        return None\n    categories = [str(row["方法名称"]) for row in rows]\n    feasibility_series = [\n        (\n            "总冲突项",\n            [\n                extract_visual_number(row["第一部分：可行性结果-总冲突项"]) or 0.0\n                for row in rows\n            ],\n        ),\n        (\n            "硬冲突项",\n            [\n                extract_visual_number(row["第一部分：可行性结果-硬冲突项"]) or 0.0\n                for row in rows\n            ],\n        ),\n        (\n            "软冲突项",\n            [\n                extract_visual_number(row["第一部分：可行性结果-软冲突项"]) or 0.0\n                for row in rows\n            ],\n        ),\n        (\n            "联锁类冲突项",\n            [\n                extract_visual_number(row["第一部分：可行性结果-联锁类冲突项"]) or 0.0\n                for row in rows\n            ],\n        ),\n    ]\n    detail_series = [\n        (\n            "股道占用冲突项",\n            [\n                extract_visual_number(row["第一部分：可行性结果-股道占用冲突项"]) or 0.0\n                for row in rows\n            ],\n        ),\n        (\n            "始发终到作业冲突项",\n            [\n                extract_visual_number(row["第一部分：可行性结果-始发终到作业冲突项"])\n                or 0.0\n                for row in rows\n            ],\n        ),\n        (\n            "进路交叉冲突项",\n            [\n                extract_visual_number(row["第一部分：可行性结果-进路交叉冲突项"]) or 0.0\n                for row in rows\n            ],\n        ),\n    ]\n    risk_series = [\n        (\n            "风险总分",\n            [\n                extract_visual_number(row["第一部分：可行性结果-加权冲突风险总分"])\n                or 0.0\n                for row in rows\n            ],\n        ),\n    ]\n    superiority_series = [\n        (\n            "股道利用标准差",\n            [\n                extract_visual_number(row["第二部分：优越性结果-股道利用标准差"]) or 0.0\n                for row in rows\n            ],\n        ),\n        (\n            "高峰股道并发",\n            [\n                extract_visual_number(row["第二部分：优越性结果-高峰股道并发"]) or 0.0\n                for row in rows\n            ],\n        ),\n        (\n            "高峰联锁并发",\n            [\n                extract_visual_number(row["第二部分：优越性结果-高峰联锁并发"]) or 0.0\n                for row in rows\n            ],\n        ),\n    ]\n    rate_series = [\n        (\n            "原计划保留率(%)",\n            [\n                extract_visual_number(row["第二部分：优越性结果-原计划保留率"]) or 0.0\n                for row in rows\n            ],\n        ),\n        (\n            "股道变更率(%)",\n            [\n                extract_visual_number(row["第二部分：优越性结果-股道变更率"]) or 0.0\n                for row in rows\n            ],\n        ),\n        (\n            "最繁忙股道占用强度",\n            [\n                extract_visual_number(row["第二部分：优越性结果-最繁忙股道占用强度"])\n                or 0.0\n                for row in rows\n            ],\n        ),\n        (\n            "最繁忙咽喉占用强度",\n            [\n                extract_visual_number(row["第二部分：优越性结果-最繁忙咽喉占用强度"])\n                or 0.0\n                for row in rows\n            ],\n        ),\n    ]\n    panels = [\n        svg_grouped_bar_panel(\n            28,\n            92,\n            640,\n            320,\n            "第一部分：可行性结果（主指标）",\n            categories,\n            feasibility_series,\n        ),\n        svg_grouped_bar_panel(\n            700,\n            92,\n            640,\n            320,\n            "第一部分：可行性结果（细分冲突）",\n            categories,\n            detail_series,\n        ),\n        svg_grouped_bar_panel(\n            28,\n            442,\n            640,\n            320,\n            "第一部分：可行性结果（风险总分）",\n            categories,\n            risk_series,\n        ),\n        svg_grouped_bar_panel(\n            700,\n            442,\n            640,\n            320,\n            "第二部分：优越性结果（均衡、并发、比例）",\n            categories,\n            superiority_series + rate_series,\n        ),\n    ]\n    return build_svg_document(\n        "通用站场 对比实验可视化",\n        "仅对比实验部分，分为可行性与优越性两部分。",\n        panels,\n        1368,\n        790,\n    )\n\n\ndef build_robustness_visualization(rows: list[dict[str, object]]) -> str | None:\n    if not rows:\n        return None\n    optimized_label = (\n        "恢复优化" if any(row.get("方案") == "恢复优化" for row in rows) else "优化方案"\n    )\n    base_rows = [row for row in rows if row.get("方案") in {"原计划", optimized_label}]\n    if not base_rows:\n        return None\n    scenario_names = []\n    for row in base_rows:\n        scenario_name = str(row["场景"])\n        if scenario_name not in scenario_names:\n            scenario_names.append(scenario_name)\n    original_map = {\n        str(row["场景"]): row for row in base_rows if row.get("方案") == "原计划"\n    }\n    optimized_map = {\n        str(row["场景"]): row for row in base_rows if row.get("方案") == optimized_label\n    }\n    panels = [\n        svg_grouped_bar_panel(\n            28,\n            92,\n            640,\n            300,\n            "鲁棒性：总冲突与硬冲突",\n            scenario_names,\n            [\n                (\n                    "原计划总冲突",\n                    [\n                        extract_visual_number(original_map[name]["总冲突项"]) or 0.0\n                        for name in scenario_names\n                    ],\n                ),\n                (\n                    f"{optimized_label}总冲突",\n                    [\n                        extract_visual_number(optimized_map[name]["总冲突项"]) or 0.0\n                        for name in scenario_names\n                    ],\n                ),\n                (\n                    "原计划硬冲突",\n                    [\n                        extract_visual_number(original_map[name]["硬冲突项"]) or 0.0\n                        for name in scenario_names\n                    ],\n                ),\n                (\n                    f"{optimized_label}硬冲突",\n                    [\n                        extract_visual_number(optimized_map[name]["硬冲突项"]) or 0.0\n                        for name in scenario_names\n                    ],\n                ),\n            ],\n        ),\n        svg_grouped_bar_panel(\n            700,\n            92,\n            640,\n            300,\n            "鲁棒性：软冲突与交叉冲突",\n            scenario_names,\n            [\n                (\n                    "原计划软冲突",\n                    [\n                        extract_visual_number(original_map[name]["软冲突项"]) or 0.0\n                        for name in scenario_names\n                    ],\n                ),\n                (\n                    f"{optimized_label}软冲突",\n                    [\n                        extract_visual_number(optimized_map[name]["软冲突项"]) or 0.0\n                        for name in scenario_names\n                    ],\n                ),\n                (\n                    "原计划进路交叉",\n                    [\n                        extract_visual_number(original_map[name]["进路交叉冲突项"])\n                        or 0.0\n                        for name in scenario_names\n                    ],\n                ),\n                (\n                    f"{optimized_label}进路交叉",\n                    [\n                        extract_visual_number(optimized_map[name]["进路交叉冲突项"])\n                        or 0.0\n                        for name in scenario_names\n                    ],\n                ),\n            ],\n        ),\n        svg_grouped_bar_panel(\n            28,\n            420,\n            1312,\n            300,\n            "鲁棒性：总重叠分钟与保留率",\n            scenario_names,\n            [\n                (\n                    "原计划总重叠分钟",\n                    [\n                        extract_visual_number(original_map[name]["总重叠分钟"]) or 0.0\n                        for name in scenario_names\n                    ],\n                ),\n                (\n                    f"{optimized_label}总重叠分钟",\n                    [\n                        extract_visual_number(optimized_map[name]["总重叠分钟"]) or 0.0\n                        for name in scenario_names\n                    ],\n                ),\n                (\n                    f"{optimized_label}保留率(%)",\n                    [\n                        extract_visual_number(optimized_map[name]["原计划保留率"])\n                        or 0.0\n                        for name in scenario_names\n                    ],\n                ),\n            ],\n        ),\n    ]\n    return build_svg_document(\n        "通用站场 鲁棒性分析可视化",\n        f"展示各扰动场景下原计划与{optimized_label}的冲突表现。",\n        panels,\n        1368,\n        750,\n    )\n\n\ndef build_optimization_effect_visualization(\n    rows: list[dict[str, object]],\n) -> str | None:\n    if not rows:\n        return None\n    panels: list[str] = []\n    module_names: list[str] = []\n    for row in rows:\n        module = str(row["优化模块"])\n        if module not in module_names:\n            module_names.append(module)\n    panel_y = 92\n    for module in module_names:\n        module_rows = [row for row in rows if row.get("优化模块") == module]\n        categories = [str(row["评价指标"]) for row in module_rows]\n        baseline_values = [\n            extract_visual_number(row["原计划结果"]) or 0.0 for row in module_rows\n        ]\n        optimized_values = [\n            extract_visual_number(row["非扰动优化结果"]) or 0.0 for row in module_rows\n        ]\n        disturbed_values = [\n            extract_visual_number(row["扰动恢复优化结果"]) or 0.0 for row in module_rows\n        ]\n        panels.append(\n            svg_grouped_bar_panel(\n                28,\n                panel_y,\n                1312,\n                220,\n                f"{module}：原计划/非扰动优化/扰动恢复优化",\n                categories,\n                [\n                    ("原计划结果", baseline_values),\n                    ("非扰动优化结果", optimized_values),\n                    ("扰动恢复优化结果", disturbed_values),\n                ],\n            )\n        )\n        panel_y += 240\n    return build_svg_document(\n        "通用站场 优化效果评价可视化",\n        "按模块展示原计划、非扰动优化与扰动恢复优化三类结果。",\n        panels,\n        1368,\n        max(340, 110 + len(panels) * 240),\n    )\n\n\ndef build_resource_balance_visualization(rows: list[dict[str, object]]) -> str | None:\n    if not rows:\n        return None\n    track_rows = [row for row in rows if row.get("资源类别") == "股道"]\n    throat_rows = [row for row in rows if row.get("资源类别") == "咽喉"]\n    balance_row = next((row for row in rows if row.get("资源类别") == "均衡性"), None)\n    top_track_rows = sorted(\n        track_rows,\n        key=lambda row: max(\n            extract_visual_number(row["原始占用总时长(分钟)"]) or 0.0,\n            extract_visual_number(row["非扰动优化占用总时长(分钟)"]) or 0.0,\n            extract_visual_number(row["扰动后恢复占用总时长(分钟)"]) or 0.0,\n        ),\n        reverse=True,\n    )[:8]\n    panels = [\n        svg_grouped_bar_panel(\n            28,\n            92,\n            640,\n            300,\n            "资源均衡：重点股道占用总时长",\n            [str(row["资源名称"]) for row in top_track_rows],\n            [\n                (\n                    "原始占用总时长",\n                    [\n                        extract_visual_number(row["原始占用总时长(分钟)"]) or 0.0\n                        for row in top_track_rows\n                    ],\n                ),\n                (\n                    "非扰动优化占用总时长",\n                    [\n                        extract_visual_number(row["非扰动优化占用总时长(分钟)"]) or 0.0\n                        for row in top_track_rows\n                    ],\n                ),\n                (\n                    "扰动后恢复占用总时长",\n                    [\n                        extract_visual_number(row["扰动后恢复占用总时长(分钟)"]) or 0.0\n                        for row in top_track_rows\n                    ],\n                ),\n            ],\n        ),\n        svg_grouped_bar_panel(\n            700,\n            92,\n            640,\n            300,\n            "资源均衡：咽喉占用强度",\n            [str(row["资源名称"]) for row in throat_rows],\n            [\n                (\n                    "原始占用强度",\n                    [\n                        extract_visual_number(row["原始占用强度"]) or 0.0\n                        for row in throat_rows\n                    ],\n                ),\n                (\n                    "非扰动优化占用强度",\n                    [\n                        extract_visual_number(row["非扰动优化占用强度"]) or 0.0\n                        for row in throat_rows\n                    ],\n                ),\n                (\n                    "扰动后恢复占用强度",\n                    [\n                        extract_visual_number(row["扰动后恢复占用强度"]) or 0.0\n                        for row in throat_rows\n                    ],\n                ),\n            ],\n        ),\n        svg_grouped_bar_panel(\n            28,\n            420,\n            640,\n            300,\n            "资源均衡：咽喉峰值并发",\n            [str(row["资源名称"]) for row in throat_rows],\n            [\n                (\n                    "原始峰值并发",\n                    [\n                        extract_visual_number(row["原始峰值并发"]) or 0.0\n                        for row in throat_rows\n                    ],\n                ),\n                (\n                    "非扰动优化峰值并发",\n                    [\n                        extract_visual_number(row["非扰动优化峰值并发"]) or 0.0\n                        for row in throat_rows\n                    ],\n                ),\n                (\n                    "扰动后恢复峰值并发",\n                    [\n                        extract_visual_number(row["扰动后恢复峰值并发"]) or 0.0\n                        for row in throat_rows\n                    ],\n                ),\n            ],\n        ),\n    ]\n    if balance_row is not None:\n        panels.append(\n            svg_grouped_bar_panel(\n                700,\n                420,\n                640,\n                300,\n                "资源均衡：股道利用标准差",\n                [str(balance_row["资源名称"])],\n                [\n                    (\n                        "原始均衡性",\n                        [extract_visual_number(balance_row["原始均衡性"]) or 0.0],\n                    ),\n                    (\n                        "非扰动优化均衡性",\n                        [extract_visual_number(balance_row["非扰动优化均衡性"]) or 0.0],\n                    ),\n                    (\n                        "扰动后恢复均衡性",\n                        [extract_visual_number(balance_row["扰动后恢复均衡性"]) or 0.0],\n                    ),\n                ],\n            )\n        )\n    return build_svg_document(\n        "通用站场 资源均衡评价可视化",\n        "展示原始数据、非扰动优化与扰动后恢复优化下的股道与咽喉资源变化。",\n        panels,\n        1368,\n        750,\n    )\n\n\ndef build_plan_assignment(\n    trains: list[TrainRecord],\n    options_by_train: dict[str, list[CandidateRoutePlan]],\n) -> dict[str, CandidateRoutePlan]:\n    assignment: dict[str, CandidateRoutePlan] = {}\n    for train in trains:\n        selected = next(\n            (\n                option\n                for option in options_by_train[train.record_id]\n                if option.track == train.planned_track\n                and option.in_throat == train.in_throat\n                and option.out_throat == train.out_throat\n            ),\n            options_by_train[train.record_id][0],\n        )\n        assignment[train.record_id] = selected\n    return assignment\n\n\ndef hard_conflict_count_for_assignment(conflicts: list[ConflictEntry]) -> int:\n    return sum(1 for conflict in conflicts if conflict.conflict_level == "硬冲突")\n\n\ndef repair_hard_conflicts(\n    trains: list[TrainRecord],\n    train_map: dict[str, TrainRecord],\n    options_by_train: dict[str, list[CandidateRoutePlan]],\n    selected_option_ids: dict[str, str],\n    linear_costs: dict[str, float],\n    pair_costs: dict[str, dict[str, float]],\n    hard_counts: dict[str, dict[str, int]],\n    hard_risks: dict[str, dict[str, float]],\n) -> dict[str, str]:\n    rng = random.Random(2026)\n    planned_ids = build_planned_assignment_ids(trains, options_by_train)\n\n    def total_hard_count(selected: dict[str, str]) -> int:\n        train_ids = sorted(selected)\n        total = 0\n        for index, train_id in enumerate(train_ids):\n            option_id = selected[train_id]\n            for other_id in train_ids[index + 1 :]:\n                total += hard_counts.get(option_id, {}).get(selected[other_id], 0)\n        return total\n\n    def train_hard_count(train_id: str, selected: dict[str, str]) -> int:\n        option_id = selected[train_id]\n        total = 0\n        for other_id, other_option_id in selected.items():\n            if other_id == train_id:\n                continue\n            total += hard_counts.get(option_id, {}).get(other_option_id, 0)\n        return total\n\n    best_selected = dict(selected_option_ids)\n    best_hard = total_hard_count(best_selected)\n    if best_hard == 0:\n        return best_selected\n\n    for restart in range(6):\n        selected = dict(best_selected if restart == 0 else planned_ids)\n        if restart > 1:\n            for train_id, options in options_by_train.items():\n                selected[train_id] = rng.choice(options).option_id\n\n        for _ in range(400):\n            current_hard = total_hard_count(selected)\n            if current_hard == 0:\n                return selected\n            if current_hard < best_hard:\n                best_hard = current_hard\n                best_selected = dict(selected)\n\n            train_scores = {\n                train_id: train_hard_count(train_id, selected) for train_id in selected\n            }\n            candidate_trains = [\n                train_id\n                for train_id, score in train_scores.items()\n                if score == max(train_scores.values()) and score > 0\n            ]\n            if not candidate_trains:\n                break\n            target_train_id = rng.choice(candidate_trains)\n            current_option_id = selected[target_train_id]\n\n            best_option_id = current_option_id\n            best_signature = (\n                train_scores[target_train_id],\n                compute_option_cost(\n                    target_train_id,\n                    current_option_id,\n                    selected,\n                    linear_costs,\n                    pair_costs,\n                    hard_counts,\n                    hard_risks,\n                ),\n            )\n            candidates: list[tuple[str, tuple[float, float]]] = []\n            for option in options_by_train[target_train_id]:\n                option_id = option.option_id\n                selected[target_train_id] = option_id\n                signature = (\n                    train_hard_count(target_train_id, selected),\n                    compute_option_cost(\n                        target_train_id,\n                        option_id,\n                        selected,\n                        linear_costs,\n                        pair_costs,\n                        hard_counts,\n                        hard_risks,\n                    ),\n                )\n                candidates.append((option_id, signature))\n                if signature < best_signature:\n                    best_signature = signature\n                    best_option_id = option_id\n            selected[target_train_id] = best_option_id\n            if best_option_id == current_option_id:\n                equal_best = [\n                    option_id\n                    for option_id, signature in candidates\n                    if signature == best_signature and option_id != current_option_id\n                ]\n                if equal_best:\n                    selected[target_train_id] = rng.choice(equal_best)\n                else:\n                    random_train = rng.choice(list(selected.keys()))\n                    selected[random_train] = rng.choice(\n                        options_by_train[random_train]\n                    ).option_id\n\n    selected = dict(best_selected)\n\n    def current_hard_graph(selected_now: dict[str, str]) -> dict[str, set[str]]:\n        graph: dict[str, set[str]] = defaultdict(set)\n        train_ids = sorted(selected_now)\n        for index, train_id in enumerate(train_ids):\n            option_id = selected_now[train_id]\n            for other_id in train_ids[index + 1 :]:\n                other_option_id = selected_now[other_id]\n                if hard_counts.get(option_id, {}).get(other_option_id, 0) > 0:\n                    graph[train_id].add(other_id)\n                    graph[other_id].add(train_id)\n        return graph\n\n    def connected_components(graph: dict[str, set[str]]) -> list[list[str]]:\n        visited: set[str] = set()\n        components: list[list[str]] = []\n        for node in graph:\n            if node in visited:\n                continue\n            stack = [node]\n            component: list[str] = []\n            visited.add(node)\n            while stack:\n                current = stack.pop()\n                component.append(current)\n                for nxt in graph.get(current, set()):\n                    if nxt not in visited:\n                        visited.add(nxt)\n                        stack.append(nxt)\n            components.append(component)\n        return components\n\n    def solve_component(\n        component: list[str], selected_now: dict[str, str]\n    ) -> dict[str, str] | None:\n        component_set = set(component)\n        fixed_selected = {\n            train_id: option_id\n            for train_id, option_id in selected_now.items()\n            if train_id not in component_set\n        }\n\n        def feasible_options(train_id: str, partial: dict[str, str]) -> list[str]:\n            feasible: list[str] = []\n            for option in options_by_train[train_id]:\n                option_id = option.option_id\n                bad = False\n                for other_id, other_option_id in fixed_selected.items():\n                    if hard_counts.get(option_id, {}).get(other_option_id, 0) > 0:\n                        bad = True\n                        break\n                if bad:\n                    continue\n                for other_id, other_option_id in partial.items():\n                    if hard_counts.get(option_id, {}).get(other_option_id, 0) > 0:\n                        bad = True\n                        break\n                if not bad:\n                    feasible.append(option_id)\n            feasible.sort(\n                key=lambda option_id: compute_option_cost(\n                    train_id,\n                    option_id,\n                    {**fixed_selected, **partial},\n                    linear_costs,\n                    pair_costs,\n                    hard_counts,\n                    hard_risks,\n                )\n            )\n            return feasible\n\n        def backtrack(partial: dict[str, str]) -> dict[str, str] | None:\n            if len(partial) == len(component):\n                return dict(partial)\n            unassigned = [train_id for train_id in component if train_id not in partial]\n            option_lists = [\n                (train_id, feasible_options(train_id, partial))\n                for train_id in unassigned\n            ]\n            if any(not opts for _, opts in option_lists):\n                return None\n            train_id, options_list = min(\n                option_lists, key=lambda item: (len(item[1]), item[0])\n            )\n            for option_id in options_list:\n                partial[train_id] = option_id\n                solved = backtrack(partial)\n                if solved is not None:\n                    return solved\n                partial.pop(train_id, None)\n            return None\n\n        return backtrack({})\n\n    for _ in range(3):\n        graph = current_hard_graph(selected)\n        if not graph:\n            break\n        progress = False\n        for component in connected_components(graph):\n            if len(component) > 18:\n                continue\n            solved = solve_component(component, selected)\n            if solved is None:\n                continue\n            for train_id, option_id in solved.items():\n                selected[train_id] = option_id\n            progress = True\n        if not progress:\n            break\n\n    return selected\n\n\ndef build_result_rows(\n    original_trains: list[TrainRecord],\n    original_assignment: dict[str, CandidateRoutePlan],\n    optimized_assignment: dict[str, CandidateRoutePlan],\n    disturbed_trains: list[TrainRecord],\n    recovered_assignment: dict[str, CandidateRoutePlan],\n    options_by_train: dict[str, list[CandidateRoutePlan]],\n    disturbed_options_by_train: dict[str, list[CandidateRoutePlan]],\n) -> list[dict[str, object]]:\n    disturbed_map = {train.record_id: train for train in disturbed_trains}\n    rows: list[dict[str, object]] = []\n    for train in original_trains:\n        original_option = original_assignment[train.record_id]\n        optimized_option = optimized_assignment[train.record_id]\n        disturbed_train = disturbed_map[train.record_id]\n        recovered_option = recovered_assignment[train.record_id]\n        rows.append(\n            {\n                "序号": train.index,\n                "列车记录ID": train.record_id,\n                "车次": train.train_no,\n                "trip_id": train.trip_id,\n                "方向": train.direction,\n                "上一站": train.prev_station,\n                "下一站": train.next_station,\n                "作业类型": train.operation_type,\n                "原始到达时刻": train.arrival_text,\n                "原始出发时刻": train.departure_text,\n                "原始股道": train.planned_track,\n                "原始接车侧咽喉": train.in_throat,\n                "原始发车侧咽喉": train.out_throat,\n                "原始接车分区": original_option.in_partition,\n                "原始发车分区": original_option.out_partition,\n                "原始接车通道": original_option.inbound_channel,\n                "原始发车通道": original_option.outbound_channel,\n                "原始进路变体": original_option.route_variant,\n                "原始进路代码": original_option.route_code,\n                "原始接车进路代码": original_option.inbound_route_code,\n                "原始发车进路代码": original_option.outbound_route_code,\n                "原始进路族": original_option.route_family,\n                "原始进站阶段区间": format_interval(\n                    train.inbound_start, train.inbound_end\n                ),\n                "原始股道停留区间": format_interval(train.dwell_start, train.dwell_end),\n                "原始出站阶段区间": format_interval(\n                    train.outbound_start, train.outbound_end\n                ),\n                "非扰动优化股道": optimized_option.track,\n                "非扰动优化是否变更": "是"\n                if optimized_option.track != train.planned_track\n                else "否",\n                "非扰动优化接车侧咽喉": optimized_option.in_throat,\n                "非扰动优化发车侧咽喉": optimized_option.out_throat,\n                "非扰动优化接车分区": optimized_option.in_partition,\n                "非扰动优化发车分区": optimized_option.out_partition,\n                "非扰动优化接车通道": optimized_option.inbound_channel,\n                "非扰动优化发车通道": optimized_option.outbound_channel,\n                "非扰动优化进路变体": optimized_option.route_variant,\n                "非扰动优化进路代码": optimized_option.route_code,\n                "非扰动优化接车进路代码": optimized_option.inbound_route_code,\n                "非扰动优化发车进路代码": optimized_option.outbound_route_code,\n                "非扰动优化进路族": optimized_option.route_family,\n                "非扰动优化候选来源": optimized_option.source_level,\n                "非扰动优化支撑样本数": optimized_option.support_count,\n                "非扰动优化候选进路数": len(options_by_train[train.record_id]),\n                "非扰动优化延误风险代理": f"{optimized_option.delay_risk_cost:.2f}",\n                "非扰动优化稳定性代价": f"{optimized_option.stability_cost:.2f}",\n                "非扰动优化均衡奖励": f"{optimized_option.balance_reward:.2f}",\n                "非扰动优化线性代价": f"{optimized_option.linear_cost:.2f}",\n                "非扰动优化说明": optimized_option.note,\n                "扰动后恢复到达时刻": disturbed_train.arrival_text,\n                "扰动后恢复出发时刻": disturbed_train.departure_text,\n                "相对原始到达晚点(分)": max(\n                    0, disturbed_train.arrival_min - train.arrival_min\n                ),\n                "相对原始出发晚点(分)": max(\n                    0, disturbed_train.departure_min - train.departure_min\n                ),\n                "扰动后恢复股道": recovered_option.track,\n                "扰动后恢复是否变更": "是"\n                if recovered_option.track != train.planned_track\n                else "否",\n                "扰动后恢复接车侧咽喉": recovered_option.in_throat,\n                "扰动后恢复发车侧咽喉": recovered_option.out_throat,\n                "扰动后恢复接车分区": recovered_option.in_partition,\n                "扰动后恢复发车分区": recovered_option.out_partition,\n                "扰动后恢复接车通道": recovered_option.inbound_channel,\n                "扰动后恢复发车通道": recovered_option.outbound_channel,\n                "扰动后恢复进路变体": recovered_option.route_variant,\n                "扰动后恢复进路代码": recovered_option.route_code,\n                "扰动后恢复接车进路代码": recovered_option.inbound_route_code,\n                "扰动后恢复发车进路代码": recovered_option.outbound_route_code,\n                "扰动后恢复进路族": recovered_option.route_family,\n                "扰动后恢复候选来源": recovered_option.source_level,\n                "扰动后恢复支撑样本数": recovered_option.support_count,\n                "扰动后恢复进站阶段区间": format_interval(\n                    disturbed_train.inbound_start, disturbed_train.inbound_end\n                ),\n                "扰动后恢复股道停留区间": format_interval(\n                    disturbed_train.dwell_start, disturbed_train.dwell_end\n                ),\n                "扰动后恢复出站阶段区间": format_interval(\n                    disturbed_train.outbound_start, disturbed_train.outbound_end\n                ),\n                "扰动后恢复候选进路数": len(\n                    disturbed_options_by_train[train.record_id]\n                ),\n                "扰动后恢复延误风险代理": f"{recovered_option.delay_risk_cost:.2f}",\n                "扰动后恢复稳定性代价": f"{recovered_option.stability_cost:.2f}",\n                "扰动后恢复均衡奖励": f"{recovered_option.balance_reward:.2f}",\n                "扰动后恢复线性代价": f"{recovered_option.linear_cost:.2f}",\n                "扰动后恢复说明": recovered_option.note,\n            }\n        )\n    return rows\n\n\ndef build_conflict_result_rows(\n    plan_conflicts: list[ConflictEntry],\n    optimized_conflicts: list[ConflictEntry],\n    recovered_conflicts: list[ConflictEntry],\n) -> list[dict[str, object]]:\n    rows: list[dict[str, object]] = []\n    conflict_type_order = {name: index for index, name in enumerate(CONFLICT_PENALTY)}\n\n    def add_row(\n        scheme: str,\n        section: str,\n        level: str,\n        conflict_type: str,\n        count: int,\n        minutes: int,\n        parent: str,\n        addable: str,\n        formula: str,\n        note: str,\n    ) -> None:\n        rows.append(\n            {\n                "方案": scheme,\n                "统计分组": section,\n                "冲突层级": level,\n                "冲突类型": conflict_type,\n                "冲突数量": count,\n                "重叠分钟合计": minutes,\n                "上级口径": parent,\n                "是否参与加总": addable,\n                "加总/包含关系": formula,\n                "结果说明": note,\n            }\n        )\n\n    scheme_conflict_sets = [\n        ("原计划结果", plan_conflicts),\n        ("非扰动优化结果", optimized_conflicts),\n        ("扰动恢复优化结果", recovered_conflicts),\n    ]\n    for scheme, conflicts in scheme_conflict_sets:\n        conflict_stats = build_conflict_statistics(conflicts)\n        by_level_type: defaultdict[tuple[str, str], dict[str, int]] = defaultdict(\n            lambda: {"数量": 0, "重叠分钟合计": 0}\n        )\n        for conflict in conflicts:\n            key = (conflict.conflict_level, conflict.conflict_type)\n            by_level_type[key]["数量"] += 1\n            by_level_type[key]["重叠分钟合计"] += conflict.overlap_min\n        add_row(\n            scheme,\n            "A-总量口径",\n            "总体",\n            "总冲突项",\n            conflict_stats["总冲突项"],\n            conflict_stats["总重叠分钟"],\n            "无",\n            "总项",\n            "总冲突项 = 安全冲突项 = 股道占用冲突项 + 始发终到作业冲突项 + 联锁类冲突项；软冲突项为兼容字段，当前安全口径下应为0",\n            "总冲突项是最高层安全冲突口径，不能再与子项重复相加。",\n        )\n        add_row(\n            scheme,\n            "B-按软硬层级",\n            "硬冲突",\n            "硬冲突项",\n            conflict_stats["硬冲突项"],\n            conflict_stats["硬冲突重叠分钟"],\n            "总冲突项",\n            "安全冲突项",\n            "当前安全口径下：硬冲突项 = 总冲突项，软冲突项 = 0",\n            "已清零。"\n            if conflict_stats["硬冲突项"] == 0\n            else "未清零，所有安全冲突均需清零。",\n        )\n        add_row(\n            scheme,\n            "B-按软硬层级",\n            "软冲突",\n            "软冲突项",\n            conflict_stats["软冲突项"],\n            conflict_stats["软冲突重叠分钟"],\n            "总冲突项",\n            "兼容字段，不作为独立安全层级",\n            "当前安全口径下所有冲突均按硬冲突处理，软冲突项应为0",\n            "兼容旧表头保留；若非0，说明仍存在未硬化的冲突判定。",\n        )\n        add_row(\n            scheme,\n            "C-按资源大类",\n            "资源大类",\n            "股道占用冲突项",\n            conflict_stats["股道占用冲突项"],\n            conflict_stats["股道占用重叠分钟"],\n            "总冲突项",\n            "可与始发终到作业冲突项、联锁类冲突项相加",\n            "总冲突项 = 股道占用冲突项 + 始发终到作业冲突项 + 联锁类冲突项",\n            "股道停留、接入或发车准备区间重叠。",\n        )\n        add_row(\n            scheme,\n            "C-按资源大类",\n            "资源大类",\n            "始发终到作业冲突项",\n            conflict_stats["始发终到作业冲突项"],\n            conflict_stats["始发终到作业重叠分钟"],\n            "总冲突项",\n            "可与股道占用冲突项、联锁类冲突项相加",\n            "总冲突项 = 股道占用冲突项 + 始发终到作业冲突项 + 联锁类冲突项",\n            "始发/终到作业重叠，单列用于与股道占用和联锁类对账。",\n        )\n        add_row(\n            scheme,\n            "C-按资源大类",\n            "资源大类",\n            "联锁类冲突项",\n            conflict_stats["联锁类冲突项"],\n            conflict_stats["联锁类重叠分钟"],\n            "总冲突项",\n            "可与股道占用冲突项、始发终到作业冲突项相加",\n            "联锁类冲突项 = 接车进路锁闭 + 发车进路锁闭 + 道岔组 + 进路交叉 + 防护带 + 咽喉能力",\n            "联锁类冲突项已包含进路交叉等子项，不能再与子项重复相加。",\n        )\n        add_row(\n            scheme,\n            "D-联锁子类",\n            "联锁子项",\n            "进路交叉冲突项",\n            conflict_stats["进路交叉冲突项"],\n            conflict_stats["进路交叉重叠分钟"],\n            "联锁类冲突项",\n            "不可与联锁类冲突项重复相加",\n            "进路交叉冲突项 ⊂ 联锁类冲突项",\n            "该项属于联锁类冲突子项，用于细分查看。",\n        )\n        for (level, conflict_type), stat in sorted(\n            by_level_type.items(),\n            key=lambda item: (\n                0 if item[0][0] == "硬冲突" else 1,\n                conflict_type_order.get(item[0][1], len(conflict_type_order)),\n                item[0][1],\n            ),\n        ):\n            parent = (\n                "股道占用冲突项"\n                if conflict_type == "股道占用冲突"\n                else "始发终到作业冲突项"\n                if conflict_type == "始发终到作业冲突"\n                else "联锁类冲突项"\n            )\n            add_row(\n                scheme,\n                "E-软硬×类型明细",\n                level,\n                conflict_type,\n                stat["数量"],\n                stat["重叠分钟合计"],\n                parent,\n                "明细项，不与上级重复相加",\n                f"{conflict_type}({level}) 属于 {parent} 的明细拆分；当前安全口径下应全部为硬冲突",\n                "所有安全冲突均必须清零。",\n            )\n    return rows\n\n\ndef max_resource_concurrency(\n    resources: list[ResourceWindow],\n    key_func,\n) -> tuple[int, str]:\n    events_by_key: defaultdict[str, list[tuple[int, int]]] = defaultdict(list)\n    for resource in resources:\n        key = key_func(resource)\n        if not key:\n            continue\n        events_by_key[key].append((resource.start_min, 1))\n        events_by_key[key].append((resource.end_min, -1))\n\n    best_concurrency = 0\n    best_key = ""\n    for key, events in events_by_key.items():\n        active = 0\n        for _, delta in sorted(events, key=lambda item: (item[0], item[1])):\n            active += delta\n            if active > best_concurrency:\n                best_concurrency = active\n                best_key = key\n    return best_concurrency, best_key\n\n\ndef compute_assignment_metrics(\n    trains: list[TrainRecord],\n    assignment: dict[str, CandidateRoutePlan],\n    conflicts: list[ConflictEntry],\n) -> dict[str, object]:\n    selected_options = list(assignment.values())\n    track_minutes: defaultdict[str, int] = defaultdict(int)\n    track_shifts: list[int] = []\n    for train in trains:\n        option = assignment[train.record_id]\n        track_minutes[option.track] += train.dwell_end - train.dwell_start\n        if option.track != train.planned_track:\n            track_shifts.append(track_distance(option.track, train.planned_track))\n\n    balance_std = (\n        statistics.pstdev(track_minutes.values()) if len(track_minutes) > 1 else 0.0\n    )\n    exact_like_count = sum(\n        1\n        for option in selected_options\n        if option.source_level in {"原计划工程进路", "同路径历史进路"}\n    )\n    plan_retention_count = sum(\n        1\n        for train in trains\n        if assignment[train.record_id].track == train.planned_track\n        and assignment[train.record_id].in_throat == train.in_throat\n        and assignment[train.record_id].out_throat == train.out_throat\n    )\n    inferred_count = sum(\n        1 for option in selected_options if "推断" in option.source_level\n    )\n    changed_count = sum(\n        1\n        for train in trains\n        if assignment[train.record_id].track != train.planned_track\n    )\n\n    resources = [\n        resource for option in selected_options for resource in option.resources\n    ]\n    horizon_start = min(train.window_start for train in trains)\n    horizon_end = max(train.window_end for train in trains)\n    horizon = max(1, horizon_end - horizon_start)\n    throat_minutes: defaultdict[str, int] = defaultdict(int)\n    for resource in resources:\n        if resource.throat:\n            throat_minutes[resource.throat] += resource.end_min - resource.start_min\n    peak_track, peak_track_resource = max_resource_concurrency(\n        [\n            resource\n            for resource in resources\n            if resource.resource_category in {"track", "track_access", "terminal_op"}\n        ],\n        key_func=lambda resource: resource.track,\n    )\n    peak_interlocking, peak_interlocking_resource = max_resource_concurrency(\n        [\n            resource\n            for resource in resources\n            if resource.resource_category\n            in {"route_lock", "switch_ladder", "flank_protection", "throat_capacity"}\n        ],\n        key_func=lambda resource: resource.throat,\n    )\n    conflict_stats = build_conflict_statistics(conflicts)\n\n    return {\n        "股道变更列车数": changed_count,\n        "股道变更率": safe_ratio(changed_count, len(trains)),\n        "原计划保留列车数": plan_retention_count,\n        "原计划保留率": safe_ratio(plan_retention_count, len(trains)),\n        "平均股道偏移": sum(track_shifts) / len(track_shifts) if track_shifts else 0.0,\n        "最大股道偏移": max(track_shifts) if track_shifts else 0,\n        "精确历史进路匹配率": exact_like_count / max(1, len(selected_options)),\n        "工程推断进路使用率": inferred_count / max(1, len(selected_options)),\n        "零样本进路数": sum(\n            1 for option in selected_options if option.support_count <= 0\n        ),\n        "平均支撑样本数": sum(option.support_count for option in selected_options)\n        / max(1, len(selected_options)),\n        "支撑样本总量": sum(option.support_count for option in selected_options),\n        "平均延误风险代理": sum(option.delay_risk_cost for option in selected_options)\n        / max(1, len(selected_options)),\n        "总延误风险代理": sum(option.delay_risk_cost for option in selected_options),\n        "总稳定性代价": sum(option.stability_cost for option in selected_options),\n        "总均衡奖励": sum(option.balance_reward for option in selected_options),\n        "股道利用标准差": balance_std,\n        "最繁忙股道占用强度": max(\n            (minutes / horizon for minutes in track_minutes.values()), default=0.0\n        ),\n        "最繁忙咽喉占用强度": max(\n            (minutes / horizon for minutes in throat_minutes.values()), default=0.0\n        ),\n        "高峰股道并发": peak_track,\n        "高峰股道资源": peak_track_resource,\n        "高峰联锁并发": peak_interlocking,\n        "高峰联锁资源": peak_interlocking_resource,\n        "总冲突项": conflict_stats["总冲突项"],\n        "总重叠分钟": conflict_stats["总重叠分钟"],\n        "联锁类冲突项": conflict_stats["联锁类冲突项"],\n        "股道占用冲突项": conflict_stats["股道占用冲突项"],\n        "始发终到作业冲突项": conflict_stats["始发终到作业冲突项"],\n        "进路交叉冲突项": conflict_stats["进路交叉冲突项"],\n        "硬冲突项": conflict_stats["硬冲突项"],\n        "软冲突项": conflict_stats["软冲突项"],\n        "硬冲突重叠分钟": conflict_stats["硬冲突重叠分钟"],\n        "加权冲突风险总分": sum(\n            conflict_entry_risk(conflict) for conflict in conflicts\n        ),\n        "总线性代价": sum(option.linear_cost for option in selected_options),\n    }\n\n\ndef run_model_variant(\n    variant_name: str,\n    target_trains: list[TrainRecord],\n    reference_trains: list[TrainRecord],\n    max_route_candidates: int,\n    particles: int,\n    iterations: int,\n    shots: int,\n    seed: int,\n    pair_mode: str = "full",\n    hard_soft_enabled: bool = True,\n    simplify_candidates: bool = False,\n    safety_weight: float | None = None,\n    stability_weight: float | None = None,\n) -> tuple[dict[str, object], dict[str, CandidateRoutePlan], list[ConflictEntry]]:\n    started = time.perf_counter()\n    if safety_weight is None:\n        safety_weight = DEFAULT_MODEL_WEIGHTS["safety_weight"]\n    if stability_weight is None:\n        stability_weight = DEFAULT_MODEL_WEIGHTS["stability_weight"]\n    library = build_route_library(reference_trains)\n    pressure_train_ids = build_high_pressure_train_ids(target_trains)\n    options_by_train, train_map = build_candidate_route_plans(\n        target_trains,\n        library,\n        max_route_candidates,\n        rich_candidate_train_ids=pressure_train_ids,\n    )\n    if simplify_candidates:\n        options_by_train = simplify_options_for_ablation(options_by_train)\n    raw_pair_costs, hard_counts, hard_risks = build_pairwise_conflict_maps(\n        trains=target_trains,\n        options_by_train=options_by_train,\n        mode=pair_mode,\n        hard_soft_enabled=hard_soft_enabled,\n        use_variant_penalty=True,\n    )\n    pair_costs = build_weighted_pair_costs(raw_pair_costs, safety_weight)\n    linear_costs = build_weighted_linear_costs(\n        options_by_train, safety_weight, stability_weight\n    )\n    if variant_name == "原计划":\n        assignment = build_plan_assignment(target_trains, options_by_train)\n    else:\n        option_ids, _ = quantum_particle_qaoa_solve(\n            trains=target_trains,\n            train_map=train_map,\n            options_by_train=options_by_train,\n            linear_costs=linear_costs,\n            pair_costs=pair_costs,\n            hard_counts=hard_counts,\n            hard_risks=hard_risks,\n            particles=particles,\n            iterations=iterations,\n            shots=shots,\n            seed=seed,\n        )\n        option_ids = repair_hard_conflicts(\n            trains=target_trains,\n            train_map=train_map,\n            options_by_train=options_by_train,\n            selected_option_ids=option_ids,\n            linear_costs=linear_costs,\n            pair_costs=pair_costs,\n            hard_counts=hard_counts,\n            hard_risks=hard_risks,\n        )\n        assignment = build_assignment_lookup(option_ids, options_by_train)\n    conflicts = collect_conflicts(target_trains, assignment, variant_name)\n    metrics = compute_assignment_metrics(target_trains, assignment, conflicts)\n    bundle = conflict_metric_bundle(conflicts)\n    elapsed = time.perf_counter() - started\n    result = {\n        "方法名称": variant_name,\n        "总冲突项": bundle["总冲突项"],\n        "总重叠分钟": metrics["总重叠分钟"],\n        "硬冲突项": metrics["硬冲突项"],\n        "软冲突项": metrics["软冲突项"],\n        "联锁冲突项": bundle["联锁类冲突项"],\n        "股道冲突项": bundle["股道冲突项"],\n        "股道占用冲突项": metrics["股道占用冲突项"],\n        "始发终到作业冲突项": metrics["始发终到作业冲突项"],\n        "进路交叉冲突项": bundle["进路交叉冲突项"],\n        "加权冲突风险总分": metrics["加权冲突风险总分"],\n        "股道利用标准差": metrics["股道利用标准差"],\n        "最繁忙股道占用强度": metrics["最繁忙股道占用强度"],\n        "最繁忙咽喉占用强度": metrics["最繁忙咽喉占用强度"],\n        "高峰股道并发": metrics["高峰股道并发"],\n        "高峰联锁并发": metrics["高峰联锁并发"],\n        "股道变更率": metrics["股道变更率"],\n        "原计划保留率": metrics["原计划保留率"],\n        "运行时间秒": elapsed,\n    }\n    return result, assignment, conflicts\n\n\ndef format_ratio_text(value: float) -> str:\n    return f"{value * 100:.2f}%"\n\n\ndef format_relative_change_text(before: float, after: float) -> str:\n    if math.isclose(before, 0.0, rel_tol=1e-9, abs_tol=1e-9):\n        return (\n            "0.00%"\n            if math.isclose(after, 0.0, rel_tol=1e-9, abs_tol=1e-9)\n            else "+100.00%"\n        )\n    return f"{safe_ratio(after - before, abs(before)) * 100:+.2f}%"\n\n\ndef format_signed_int_text(value: int) -> str:\n    return f"{int(value):+d}"\n\n\ndef format_signed_score_text(value: float) -> str:\n    return f"{value:+.0f}"\n\n\ndef format_signed_percentage_point_text(value: float) -> str:\n    return f"{value * 100:+.2f}个百分点"\n\n\ndef conflict_relation_note(extra_note: str = "") -> str:\n    base_note = (\n        "总冲突项 = 硬冲突项 + 软冲突项；"\n        "联锁类冲突项 = 总冲突项 - 股道占用冲突项 - 始发终到作业冲突项；"\n        "进路交叉冲突项属于联锁类冲突子项。"\n    )\n    if extra_note:\n        return f"{base_note}{extra_note}"\n    return base_note\n\n\ndef build_relative_model_conclusion(\n    result: dict[str, object],\n    reference_result: dict[str, object],\n    reference_name: str,\n) -> str:\n    if result["方法名称"] == reference_name:\n        return f"{reference_name}基准。"\n    return (\n        f"相对{reference_name}：总冲突{int(result[\'总冲突项\']) - int(reference_result[\'总冲突项\']):+d}，"\n        f"硬冲突{int(result[\'硬冲突项\']) - int(reference_result[\'硬冲突项\']):+d}，"\n        f"软冲突{int(result[\'软冲突项\']) - int(reference_result[\'软冲突项\']):+d}，"\n        f"联锁类{int(result[\'联锁冲突项\']) - int(reference_result[\'联锁冲突项\']):+d}，"\n        f"风险总分{float(result[\'加权冲突风险总分\']) - float(reference_result[\'加权冲突风险总分\']):+.0f}。"\n    )\n\n\ndef build_comparison_rows(\n    results: list[dict[str, object]], full_model_name: str\n) -> list[dict[str, object]]:\n    rows: list[dict[str, object]] = []\n    model_notes = {\n        "原计划": "不做优化的现实参照方案。",\n        "股道层基线模型": "仅处理股道层可行性，不引入联锁约束。",\n        "粗联锁模型": "在股道层基础上加入粗粒度联锁约束。",\n        "完整模型": "采用完整联锁建模并兼顾可行性与优越性目标。",\n    }\n    for result in results:\n        rows.append(\n            {\n                "实验类型": "对比实验",\n                "方法名称": result["方法名称"],\n                "模型说明": model_notes.get(result["方法名称"], ""),\n                "第一部分：可行性结果-总冲突项": result["总冲突项"],\n                "第一部分：可行性结果-硬冲突项": result["硬冲突项"],\n                "第一部分：可行性结果-软冲突项": result["软冲突项"],\n                "第一部分：可行性结果-联锁类冲突项": result["联锁冲突项"],\n                "第一部分：可行性结果-股道占用冲突项": result["股道占用冲突项"],\n                "第一部分：可行性结果-始发终到作业冲突项": result["始发终到作业冲突项"],\n                "第一部分：可行性结果-进路交叉冲突项": result["进路交叉冲突项"],\n                "第一部分：可行性结果-加权冲突风险总分": f"{result[\'加权冲突风险总分\']:.0f}",\n                "第二部分：优越性结果-股道利用标准差": f"{result[\'股道利用标准差\']:.2f}",\n                "第二部分：优越性结果-最繁忙股道占用强度": f"{result[\'最繁忙股道占用强度\']:.4f}",\n                "第二部分：优越性结果-最繁忙咽喉占用强度": f"{result[\'最繁忙咽喉占用强度\']:.4f}",\n                "第二部分：优越性结果-高峰股道并发": int(result["高峰股道并发"]),\n                "第二部分：优越性结果-高峰联锁并发": int(result["高峰联锁并发"]),\n                "第二部分：优越性结果-原计划保留率": format_ratio_text(\n                    result["原计划保留率"]\n                ),\n                "第二部分：优越性结果-股道变更率": format_ratio_text(\n                    result["股道变更率"]\n                ),\n                "运行时间(秒)": f"{result[\'运行时间秒\']:.2f}",\n            }\n        )\n    return rows\n\n\ndef derive_recovery_solver_settings(\n    particles: int,\n    iterations: int,\n    shots: int,\n) -> tuple[int, int, int]:\n    return (\n        max(4, min(particles, max(4, particles - 2))),\n        max(5, min(iterations, max(5, iterations - 2))),\n        max(1, min(shots, max(1, shots - 1))),\n    )\n\n\ndef build_robustness_rows(\n    original_trains: list[TrainRecord],\n    scenario: DisturbanceScenario,\n    impact_summary: dict[str, object],\n    plan_metrics: dict[str, object],\n    recovered_metrics: dict[str, object],\n) -> list[dict[str, object]]:\n    scenario_name = "风险驱动短暂扰动"\n    scenario_desc = build_disturbance_description(scenario, original_trains)\n    rows: list[dict[str, object]] = []\n    for scheme, metrics in [("原计划", plan_metrics), ("恢复优化", recovered_metrics)]:\n        rows.append(\n            {\n                "场景": scenario_name,\n                "说明": scenario_desc,\n                "方案": scheme,\n                "总冲突项": metrics["总冲突项"],\n                "硬冲突项": metrics["硬冲突项"],\n                "软冲突项": metrics["软冲突项"],\n                "联锁类冲突项": metrics["联锁类冲突项"],\n                "股道占用冲突项": metrics["股道占用冲突项"],\n                "始发终到作业冲突项": metrics["始发终到作业冲突项"],\n                "进路交叉冲突项": metrics["进路交叉冲突项"],\n                "总重叠分钟": metrics["总重叠分钟"],\n                "原计划保留率": format_ratio_text(metrics["原计划保留率"]),\n                "总晚点时长": impact_summary["总晚点时长"],\n                "平均晚点时长": f"{impact_summary[\'平均晚点时长\']:.2f}",\n                "最大晚点时长": impact_summary["最大晚点时长"],\n                "初始扰动列车数": impact_summary["初始扰动列车数"],\n                "受影响列车数": impact_summary["受影响列车数"],\n                "受影响股道数": impact_summary["受影响股道数"],\n                "受影响咽喉数": impact_summary["受影响咽喉数"],\n                "受影响分区/道岔组数": impact_summary["受影响分区/道岔组数"],\n                "扰动时刻": format_disturbance_minutes(scenario),\n                "传播轮次": scenario.propagation_rounds,\n                "指标关系说明": conflict_relation_note(),\n            }\n        )\n    rows.append(\n        {\n            "场景": scenario_name,\n            "说明": scenario_desc,\n            "方案": "改善量(原计划-恢复优化)",\n            "总冲突项": plan_metrics["总冲突项"] - recovered_metrics["总冲突项"],\n            "硬冲突项": plan_metrics["硬冲突项"] - recovered_metrics["硬冲突项"],\n            "软冲突项": plan_metrics["软冲突项"] - recovered_metrics["软冲突项"],\n            "联锁类冲突项": plan_metrics["联锁类冲突项"]\n            - recovered_metrics["联锁类冲突项"],\n            "股道占用冲突项": plan_metrics["股道占用冲突项"]\n            - recovered_metrics["股道占用冲突项"],\n            "始发终到作业冲突项": plan_metrics["始发终到作业冲突项"]\n            - recovered_metrics["始发终到作业冲突项"],\n            "进路交叉冲突项": plan_metrics["进路交叉冲突项"]\n            - recovered_metrics["进路交叉冲突项"],\n            "总重叠分钟": plan_metrics["总重叠分钟"] - recovered_metrics["总重叠分钟"],\n            "原计划保留率": "",\n            "总晚点时长": impact_summary["总晚点时长"],\n            "平均晚点时长": f"{impact_summary[\'平均晚点时长\']:.2f}",\n            "最大晚点时长": impact_summary["最大晚点时长"],\n            "初始扰动列车数": impact_summary["初始扰动列车数"],\n            "受影响列车数": impact_summary["受影响列车数"],\n            "受影响股道数": impact_summary["受影响股道数"],\n            "受影响咽喉数": impact_summary["受影响咽喉数"],\n            "受影响分区/道岔组数": impact_summary["受影响分区/道岔组数"],\n            "扰动时刻": format_disturbance_minutes(scenario),\n            "传播轮次": scenario.propagation_rounds,\n            "指标关系说明": conflict_relation_note(\n                " 改善量按原计划-恢复优化计算；正值表示恢复优化后该指标更少。"\n            ),\n        }\n    )\n    return rows\n\n\ndef summarize_assignment_resources(\n    trains: list[TrainRecord],\n    assignment: dict[str, CandidateRoutePlan],\n) -> tuple[\n    dict[str, dict[str, float]],\n    dict[str, dict[str, float]],\n    dict[str, dict[str, float]],\n    Counter[str],\n]:\n    track_stats: defaultdict[str, dict[str, float]] = defaultdict(\n        lambda: {"使用次数": 0.0, "占用总时长": 0.0, "占用强度": 0.0, "峰值并发": 0.0}\n    )\n    throat_stats: defaultdict[str, dict[str, float]] = defaultdict(\n        lambda: {"使用次数": 0.0, "占用总时长": 0.0, "占用强度": 0.0, "峰值并发": 0.0}\n    )\n    partition_stats: defaultdict[str, dict[str, float]] = defaultdict(\n        lambda: {"使用次数": 0.0, "占用总时长": 0.0}\n    )\n    route_family_stats: Counter[str] = Counter()\n    track_windows: list[ResourceWindow] = []\n    throat_resources: list[ResourceWindow] = []\n\n    train_map = {train.record_id: train for train in trains}\n    for train_id, option in assignment.items():\n        train = train_map[train_id]\n        track_stats[option.track]["使用次数"] += 1\n        track_stats[option.track]["占用总时长"] += train.dwell_end - train.dwell_start\n        route_family_stats[option.route_family] += 1\n        track_windows.append(\n            ResourceWindow(\n                train_id=train.record_id,\n                option_id=option.option_id,\n                route_code=option.route_code,\n                stage="C",\n                resource_name=f"股道占用:{option.track}",\n                resource_category="track",\n                movement_family="股道",\n                throat="",\n                partition="",\n                channel="",\n                zone=classify_track_zone(option.track),\n                track=option.track,\n                start_min=train.dwell_start,\n                end_min=train.dwell_end,\n            )\n        )\n        for resource in option.resources:\n            if resource.throat:\n                if resource.throat in {"始发端", "终到端"}:\n                    continue\n                throat_stats[resource.throat]["使用次数"] += 1\n                throat_stats[resource.throat]["占用总时长"] += (\n                    resource.end_min - resource.start_min\n                )\n                throat_resources.append(resource)\n            if resource.partition and resource.throat not in {"始发端", "终到端"}:\n                partition_key = f"{resource.throat}-{resource.partition}"\n                partition_stats[partition_key]["使用次数"] += 1\n                partition_stats[partition_key]["占用总时长"] += (\n                    resource.end_min - resource.start_min\n                )\n\n        endpoint_windows: list[tuple[str, int, int]] = []\n        if train.operation_type in {"始发", "始发终到"}:\n            endpoint_windows.append(("始发端", train.inbound_start, train.inbound_end))\n        if train.operation_type in {"终到", "始发终到"}:\n            endpoint_windows.append(\n                ("终到端", train.outbound_start, train.outbound_end)\n            )\n        for endpoint_throat, start_min, end_min in endpoint_windows:\n            throat_stats[endpoint_throat]["使用次数"] += 1\n            endpoint_duration = max(0, end_min - start_min)\n            throat_stats[endpoint_throat]["占用总时长"] += endpoint_duration\n            if endpoint_duration > 0:\n                throat_resources.append(\n                    ResourceWindow(\n                        train_id=train.record_id,\n                        option_id=option.option_id,\n                        route_code=option.route_code,\n                        stage="endpoint",\n                        resource_name=f"端点咽喉:{endpoint_throat}",\n                        resource_category="throat_capacity",\n                        movement_family="端点",\n                        throat=endpoint_throat,\n                        partition=throat_partition(option.track, endpoint_throat),\n                        channel="",\n                        zone=classify_track_zone(option.track),\n                        track=option.track,\n                        start_min=start_min,\n                        end_min=end_min,\n                    )\n                )\n\n    horizon_start = min(train.window_start for train in trains)\n    horizon_end = max(train.window_end for train in trains)\n    horizon = max(1, horizon_end - horizon_start)\n    for track, stat in track_stats.items():\n        stat["占用强度"] = stat["占用总时长"] / horizon\n        peak, _ = max_resource_concurrency(\n            [resource for resource in track_windows if resource.track == track],\n            key_func=lambda resource: resource.track,\n        )\n        stat["峰值并发"] = peak\n    for throat, stat in throat_stats.items():\n        stat["占用强度"] = stat["占用总时长"] / horizon\n        peak, _ = max_resource_concurrency(\n            [resource for resource in throat_resources if resource.throat == throat],\n            key_func=lambda resource: resource.throat,\n        )\n        stat["峰值并发"] = peak\n\n    return (\n        dict(track_stats),\n        dict(throat_stats),\n        dict(partition_stats),\n        route_family_stats,\n    )\n\n\ndef hhi_from_values(values: list[float]) -> float:\n    total = sum(values)\n    if total <= 0:\n        return 0.0\n    return sum((value / total) ** 2 for value in values if value > 0)\n\n\ndef max_share_from_counter(counter: Counter[str]) -> float:\n    total = sum(counter.values())\n    if total <= 0:\n        return 0.0\n    return max(counter.values()) / total\n\n\ndef format_degree(\n    baseline_value: float,\n    optimized_value: float,\n    lower_is_better: bool,\n) -> str:\n    if abs(optimized_value - baseline_value) <= 1e-9:\n        return "基本持平"\n    if abs(baseline_value) <= 1e-9:\n        return "由0变为当前值"\n    if lower_is_better:\n        ratio = safe_ratio(baseline_value - optimized_value, abs(baseline_value))\n        return f"{\'下降\' if ratio >= 0 else \'上升\'}{abs(ratio) * 100:.2f}%"\n    ratio = safe_ratio(optimized_value - baseline_value, abs(baseline_value))\n    return f"{\'提升\' if ratio >= 0 else \'下降\'}{abs(ratio) * 100:.2f}%"\n\n\ndef build_optimization_effect_rows(\n    original_trains: list[TrainRecord],\n    plan_assignment: dict[str, CandidateRoutePlan],\n    optimized_trains: list[TrainRecord],\n    optimized_assignment: dict[str, CandidateRoutePlan],\n    plan_conflicts: list[ConflictEntry],\n    optimized_conflicts: list[ConflictEntry],\n    recovered_trains: list[TrainRecord],\n    recovered_assignment: dict[str, CandidateRoutePlan],\n    recovered_conflicts: list[ConflictEntry],\n) -> list[dict[str, object]]:\n    baseline = compute_assignment_metrics(\n        original_trains, plan_assignment, plan_conflicts\n    )\n    optimized = compute_assignment_metrics(\n        optimized_trains, optimized_assignment, optimized_conflicts\n    )\n    disturbed_optimized = compute_assignment_metrics(\n        recovered_trains, recovered_assignment, recovered_conflicts\n    )\n    baseline_delay = compute_delay_statistics(original_trains, original_trains)\n    optimized_delay = compute_delay_statistics(original_trains, optimized_trains)\n    disturbed_delay = compute_delay_statistics(original_trains, recovered_trains)\n    plan_tracks, plan_throats, plan_partitions, plan_route_families = (\n        summarize_assignment_resources(original_trains, plan_assignment)\n    )\n    opt_tracks, opt_throats, opt_partitions, opt_route_families = (\n        summarize_assignment_resources(optimized_trains, optimized_assignment)\n    )\n    (\n        disturbed_tracks,\n        disturbed_throats,\n        disturbed_partitions,\n        disturbed_route_families,\n    ) = summarize_assignment_resources(recovered_trains, recovered_assignment)\n    rows: list[dict[str, object]] = []\n\n    def build_metric_conclusion(\n        metric: str,\n        baseline_value: float,\n        optimized_value: float,\n        disturbed_value: float,\n        lower_is_better: bool,\n    ) -> str:\n        if lower_is_better:\n            if disturbed_value + 1e-9 < optimized_value:\n                return f"{metric}在扰动恢复优化后低于非扰动优化结果。"\n            if math.isclose(\n                disturbed_value, optimized_value, rel_tol=1e-9, abs_tol=1e-9\n            ):\n                return f"{metric}在扰动恢复优化后与非扰动优化结果基本持平。"\n            return f"{metric}在扰动恢复优化后高于非扰动优化结果，反映扰动恢复代价。"\n        if disturbed_value > optimized_value + 1e-9:\n            return f"{metric}在扰动恢复优化后高于非扰动优化结果。"\n        if math.isclose(disturbed_value, optimized_value, rel_tol=1e-9, abs_tol=1e-9):\n            return f"{metric}在扰动恢复优化后与非扰动优化结果基本持平。"\n        return f"{metric}在扰动恢复优化后低于非扰动优化结果，反映扰动恢复约束影响。"\n\n    def effect_metric_relation(\n        module: str, metric: str, lower_is_better: bool\n    ) -> tuple[str, str, str, str, str, str]:\n        direction = "最小化" if lower_is_better else "最大化"\n        if metric == "总冲突项":\n            return (\n                "A-冲突总量口径",\n                "总项",\n                "无",\n                "总项",\n                "总冲突项 = 硬冲突项 + 软冲突项 = 股道占用冲突项 + 始发终到作业冲突项 + 联锁类冲突项",\n                direction,\n            )\n        if metric in {"硬冲突项", "软冲突项"}:\n            return (\n                "B-按软硬层级",\n                "冲突层级项",\n                "总冲突项",\n                "硬冲突项与软冲突项可相加",\n                "总冲突项 = 硬冲突项 + 软冲突项",\n                direction,\n            )\n        if metric in {"股道占用冲突项", "始发终到作业冲突项", "联锁类冲突项"}:\n            return (\n                "C-按资源大类",\n                "资源大类项",\n                "总冲突项",\n                "股道占用冲突项、始发终到作业冲突项、联锁类冲突项可相加",\n                "总冲突项 = 股道占用冲突项 + 始发终到作业冲突项 + 联锁类冲突项",\n                direction,\n            )\n        if metric == "进路交叉冲突项":\n            return (\n                "D-联锁子类",\n                "联锁子项",\n                "联锁类冲突项",\n                "不可与联锁类冲突项重复相加",\n                "进路交叉冲突项 ⊂ 联锁类冲突项",\n                direction,\n            )\n        if module == "资源优化":\n            return (\n                "E-资源优化指标",\n                "评价指标",\n                "资源利用状态",\n                "不参与冲突数量加总",\n                "用于评价股道/联锁资源并发与均衡，不属于冲突项求和口径",\n                direction,\n            )\n        if module == "结构优化":\n            return (\n                "F-结构优化指标",\n                "评价指标",\n                "资源分配结构",\n                "不参与冲突数量加总",\n                "用于评价资源集中度、瓶颈占用和进路族集中度，不属于冲突项求和口径",\n                direction,\n            )\n        if module == "晚点恢复":\n            return (\n                "G-晚点恢复指标",\n                "评价指标",\n                "列车时刻恢复",\n                "不参与冲突数量加总",\n                "总晚点、平均晚点、最大晚点分别从不同角度评价晚点恢复，三者不可相加",\n                direction,\n            )\n        return ("Z-其他指标", "评价指标", "无", "不参与加总", "无", direction)\n\n    def add_summary_row(\n        module: str,\n        metric: str,\n        baseline_value: float,\n        optimized_value: float,\n        disturbed_value: float,\n        unit: str,\n        lower_is_better: bool,\n        conclusion: str | None = None,\n    ) -> None:\n        if unit == "比例":\n            baseline_text = f"{baseline_value * 100:.2f}%"\n            optimized_text = f"{optimized_value * 100:.2f}%"\n            disturbed_text = f"{disturbed_value * 100:.2f}%"\n        elif unit == "HHI":\n            baseline_text = f"{baseline_value:.4f}"\n            optimized_text = f"{optimized_value:.4f}"\n            disturbed_text = f"{disturbed_value:.4f}"\n        elif unit == "项":\n            baseline_text = f"{int(round(baseline_value))}项"\n            optimized_text = f"{int(round(optimized_value))}项"\n            disturbed_text = f"{int(round(disturbed_value))}项"\n        elif unit == "分钟":\n            baseline_text = f"{baseline_value:.2f}分钟"\n            optimized_text = f"{optimized_value:.2f}分钟"\n            disturbed_text = f"{disturbed_value:.2f}分钟"\n        elif unit == "列":\n            baseline_text = f"{int(round(baseline_value))}列"\n            optimized_text = f"{int(round(optimized_value))}列"\n            disturbed_text = f"{int(round(disturbed_value))}列"\n        else:\n            baseline_text = f"{baseline_value:.2f}"\n            optimized_text = f"{optimized_value:.2f}"\n            disturbed_text = f"{disturbed_value:.2f}"\n        group, level, parent, addable, formula, direction = effect_metric_relation(\n            module, metric, lower_is_better\n        )\n        rows.append(\n            {\n                "优化模块": module,\n                "统计分组": group,\n                "指标层级": level,\n                "评价指标": metric,\n                "原计划结果": baseline_text,\n                "非扰动优化结果": optimized_text,\n                "扰动恢复优化结果": disturbed_text,\n                "上级口径": parent,\n                "是否参与加总": addable,\n                "加总/包含关系": formula,\n                "优化方向": direction,\n                "非扰动优化相对原计划优化程度": format_degree(\n                    baseline_value, optimized_value, lower_is_better\n                ),\n                "扰动恢复优化相对非扰动优化变化程度": format_degree(\n                    optimized_value, disturbed_value, lower_is_better\n                ),\n                "评价结论": conclusion\n                or build_metric_conclusion(\n                    metric,\n                    baseline_value,\n                    optimized_value,\n                    disturbed_value,\n                    lower_is_better,\n                ),\n            }\n        )\n\n    add_summary_row(\n        "冲突优化",\n        "总冲突项",\n        float(baseline["总冲突项"]),\n        float(optimized["总冲突项"]),\n        float(disturbed_optimized["总冲突项"]),\n        "项",\n        True,\n    )\n    add_summary_row(\n        "冲突优化",\n        "硬冲突项",\n        float(baseline["硬冲突项"]),\n        float(optimized["硬冲突项"]),\n        float(disturbed_optimized["硬冲突项"]),\n        "项",\n        True,\n    )\n    add_summary_row(\n        "冲突优化",\n        "软冲突项",\n        float(baseline["软冲突项"]),\n        float(optimized["软冲突项"]),\n        float(disturbed_optimized["软冲突项"]),\n        "项",\n        True,\n    )\n    add_summary_row(\n        "冲突优化",\n        "联锁类冲突项",\n        float(baseline["联锁类冲突项"]),\n        float(optimized["联锁类冲突项"]),\n        float(disturbed_optimized["联锁类冲突项"]),\n        "项",\n        True,\n    )\n    add_summary_row(\n        "冲突优化",\n        "股道占用冲突项",\n        float(baseline["股道占用冲突项"]),\n        float(optimized["股道占用冲突项"]),\n        float(disturbed_optimized["股道占用冲突项"]),\n        "项",\n        True,\n    )\n    add_summary_row(\n        "冲突优化",\n        "始发终到作业冲突项",\n        float(baseline["始发终到作业冲突项"]),\n        float(optimized["始发终到作业冲突项"]),\n        float(disturbed_optimized["始发终到作业冲突项"]),\n        "项",\n        True,\n    )\n    add_summary_row(\n        "冲突优化",\n        "进路交叉冲突项",\n        float(baseline["进路交叉冲突项"]),\n        float(optimized["进路交叉冲突项"]),\n        float(disturbed_optimized["进路交叉冲突项"]),\n        "项",\n        True,\n        "该项属于联锁类冲突子项，用于细分查看。",\n    )\n\n    add_summary_row(\n        "资源优化",\n        "股道利用标准差",\n        float(baseline["股道利用标准差"]),\n        float(optimized["股道利用标准差"]),\n        float(disturbed_optimized["股道利用标准差"]),\n        "分钟",\n        True,\n    )\n    add_summary_row(\n        "资源优化",\n        "高峰联锁并发",\n        float(baseline["高峰联锁并发"]),\n        float(optimized["高峰联锁并发"]),\n        float(disturbed_optimized["高峰联锁并发"]),\n        "列",\n        True,\n    )\n    add_summary_row(\n        "资源优化",\n        "高峰股道并发",\n        float(baseline["高峰股道并发"]),\n        float(optimized["高峰股道并发"]),\n        float(disturbed_optimized["高峰股道并发"]),\n        "列",\n        True,\n    )\n\n    plan_track_hhi = hhi_from_values(\n        [stat["占用总时长"] for stat in plan_tracks.values()]\n    )\n    opt_track_hhi = hhi_from_values(\n        [stat["占用总时长"] for stat in opt_tracks.values()]\n    )\n    disturbed_track_hhi = hhi_from_values(\n        [stat["占用总时长"] for stat in disturbed_tracks.values()]\n    )\n    plan_bottleneck = max(\n        (stat["占用总时长"] for stat in plan_partitions.values()), default=0.0\n    )\n    opt_bottleneck = max(\n        (stat["占用总时长"] for stat in opt_partitions.values()), default=0.0\n    )\n    disturbed_bottleneck = max(\n        (stat["占用总时长"] for stat in disturbed_partitions.values()), default=0.0\n    )\n    plan_route_concentration = max_share_from_counter(plan_route_families)\n    opt_route_concentration = max_share_from_counter(opt_route_families)\n    disturbed_route_concentration = max_share_from_counter(disturbed_route_families)\n    add_summary_row(\n        "结构优化",\n        "股道分配集中度",\n        plan_track_hhi,\n        opt_track_hhi,\n        disturbed_track_hhi,\n        "HHI",\n        True,\n    )\n    add_summary_row(\n        "结构优化",\n        "局部瓶颈区占用时长",\n        plan_bottleneck,\n        opt_bottleneck,\n        disturbed_bottleneck,\n        "分钟",\n        True,\n    )\n    add_summary_row(\n        "结构优化",\n        "最大进路集中度",\n        plan_route_concentration,\n        opt_route_concentration,\n        disturbed_route_concentration,\n        "比例",\n        True,\n    )\n    add_summary_row(\n        "晚点恢复",\n        "总晚点时长",\n        float(baseline_delay["总晚点时长"]),\n        float(optimized_delay["总晚点时长"]),\n        float(disturbed_delay["总晚点时长"]),\n        "分钟",\n        True,\n    )\n    add_summary_row(\n        "晚点恢复",\n        "平均晚点时长",\n        float(baseline_delay["平均晚点时长"]),\n        float(optimized_delay["平均晚点时长"]),\n        float(disturbed_delay["平均晚点时长"]),\n        "分钟",\n        True,\n    )\n    add_summary_row(\n        "晚点恢复",\n        "最大晚点时长",\n        float(baseline_delay["最大晚点时长"]),\n        float(optimized_delay["最大晚点时长"]),\n        float(disturbed_delay["最大晚点时长"]),\n        "分钟",\n        True,\n    )\n    return rows\n\n\ndef build_resource_balance_rows(\n    original_trains: list[TrainRecord],\n    original_assignment: dict[str, CandidateRoutePlan],\n    optimized_trains: list[TrainRecord],\n    optimized_assignment: dict[str, CandidateRoutePlan],\n    disturbed_trains: list[TrainRecord],\n    recovered_assignment: dict[str, CandidateRoutePlan],\n) -> list[dict[str, object]]:\n    rows: list[dict[str, object]] = []\n    original_tracks, original_throats, _, _ = summarize_assignment_resources(\n        original_trains, original_assignment\n    )\n    optimized_tracks, optimized_throats, _, _ = summarize_assignment_resources(\n        optimized_trains, optimized_assignment\n    )\n    recovered_tracks, recovered_throats, _, _ = summarize_assignment_resources(\n        disturbed_trains, recovered_assignment\n    )\n    original_track_values = [stat["占用总时长"] for stat in original_tracks.values()]\n    optimized_track_values = [stat["占用总时长"] for stat in optimized_tracks.values()]\n    recovered_track_values = [stat["占用总时长"] for stat in recovered_tracks.values()]\n    original_track_balance_std = (\n        statistics.pstdev(original_track_values)\n        if len(original_track_values) > 1\n        else 0.0\n    )\n    optimized_track_balance_std = (\n        statistics.pstdev(optimized_track_values)\n        if len(optimized_track_values) > 1\n        else 0.0\n    )\n    recovered_track_balance_std = (\n        statistics.pstdev(recovered_track_values)\n        if len(recovered_track_values) > 1\n        else 0.0\n    )\n    rows.append(\n        {\n            "资源类别": "均衡性",\n            "资源名称": "股道利用标准差",\n            "原始使用次数": "--",\n            "非扰动优化使用次数": "--",\n            "扰动后恢复使用次数": "--",\n            "非扰动优化使用次数变化": "--",\n            "非扰动优化使用次数变化率": "--",\n            "扰动后恢复使用次数变化": "--",\n            "扰动后恢复使用次数变化率": "--",\n            "原始占用总时长(分钟)": "--",\n            "非扰动优化占用总时长(分钟)": "--",\n            "扰动后恢复占用总时长(分钟)": "--",\n            "非扰动优化占用时长变化(分钟)": "--",\n            "非扰动优化占用时长变化率": "--",\n            "扰动后恢复占用时长变化(分钟)": "--",\n            "扰动后恢复占用时长变化率": "--",\n            "原始占用强度": "--",\n            "非扰动优化占用强度": "--",\n            "扰动后恢复占用强度": "--",\n            "非扰动优化占用强度变化": "--",\n            "非扰动优化占用强度变化率": "--",\n            "扰动后恢复占用强度变化": "--",\n            "扰动后恢复占用强度变化率": "--",\n            "原始峰值并发": "--",\n            "非扰动优化峰值并发": "--",\n            "扰动后恢复峰值并发": "--",\n            "非扰动优化峰值并发变化": "--",\n            "非扰动优化峰值并发变化率": "--",\n            "扰动后恢复峰值并发变化": "--",\n            "扰动后恢复峰值并发变化率": "--",\n            "原始均衡性": f"{original_track_balance_std:.2f}",\n            "非扰动优化均衡性": f"{optimized_track_balance_std:.2f}",\n            "扰动后恢复均衡性": f"{recovered_track_balance_std:.2f}",\n            "非扰动优化均衡性变化": f"{optimized_track_balance_std - original_track_balance_std:+.2f}",\n            "非扰动优化均衡性优化率": format_degree(\n                original_track_balance_std, optimized_track_balance_std, True\n            ),\n            "扰动后恢复均衡性变化": f"{recovered_track_balance_std - optimized_track_balance_std:+.2f}",\n            "扰动后恢复均衡性优化率": format_degree(\n                optimized_track_balance_std, recovered_track_balance_std, True\n            ),\n            "优化说明": "数值越低表示股道分配越均衡，用于补充整体均衡性评价。",\n        }\n    )\n\n    def make_balance_row(\n        category: str,\n        name: str,\n        original_use: object,\n        optimized_use: object,\n        recovered_use: object,\n        original_minutes: object,\n        optimized_minutes: object,\n        recovered_minutes: object,\n        original_intensity: object,\n        optimized_intensity: object,\n        recovered_intensity: object,\n        original_peak: object,\n        optimized_peak: object,\n        recovered_peak: object,\n        note: str,\n    ) -> dict[str, object]:\n        numeric_fields = all(\n            isinstance(value, (int, float))\n            for value in [\n                original_use,\n                optimized_use,\n                recovered_use,\n                original_minutes,\n                optimized_minutes,\n                recovered_minutes,\n                original_intensity,\n                optimized_intensity,\n                recovered_intensity,\n                original_peak,\n                optimized_peak,\n                recovered_peak,\n            ]\n        )\n        if not numeric_fields:\n            return {\n                "资源类别": category,\n                "资源名称": name,\n                "原始使用次数": original_use,\n                "非扰动优化使用次数": optimized_use,\n                "扰动后恢复使用次数": recovered_use,\n                "非扰动优化使用次数变化": "--",\n                "非扰动优化使用次数变化率": "--",\n                "扰动后恢复使用次数变化": "--",\n                "扰动后恢复使用次数变化率": "--",\n                "原始占用总时长(分钟)": original_minutes,\n                "非扰动优化占用总时长(分钟)": optimized_minutes,\n                "扰动后恢复占用总时长(分钟)": recovered_minutes,\n                "非扰动优化占用时长变化(分钟)": "--",\n                "非扰动优化占用时长变化率": "--",\n                "扰动后恢复占用时长变化(分钟)": "--",\n                "扰动后恢复占用时长变化率": "--",\n                "原始占用强度": original_intensity,\n                "非扰动优化占用强度": optimized_intensity,\n                "扰动后恢复占用强度": recovered_intensity,\n                "非扰动优化占用强度变化": "--",\n                "非扰动优化占用强度变化率": "--",\n                "扰动后恢复占用强度变化": "--",\n                "扰动后恢复占用强度变化率": "--",\n                "原始峰值并发": original_peak,\n                "非扰动优化峰值并发": optimized_peak,\n                "扰动后恢复峰值并发": recovered_peak,\n                "非扰动优化峰值并发变化": "--",\n                "非扰动优化峰值并发变化率": "--",\n                "扰动后恢复峰值并发变化": "--",\n                "扰动后恢复峰值并发变化率": "--",\n                "原始均衡性": "--",\n                "非扰动优化均衡性": "--",\n                "扰动后恢复均衡性": "--",\n                "非扰动优化均衡性变化": "--",\n                "非扰动优化均衡性优化率": "--",\n                "扰动后恢复均衡性变化": "--",\n                "扰动后恢复均衡性优化率": "--",\n                "优化说明": note,\n            }\n\n        return {\n            "资源类别": category,\n            "资源名称": name,\n            "原始使用次数": int(round(float(original_use))),\n            "非扰动优化使用次数": int(round(float(optimized_use))),\n            "扰动后恢复使用次数": int(round(float(recovered_use))),\n            "非扰动优化使用次数变化": int(\n                round(float(optimized_use) - float(original_use))\n            ),\n            "非扰动优化使用次数变化率": format_relative_change_text(\n                float(original_use), float(optimized_use)\n            ),\n            "扰动后恢复使用次数变化": int(\n                round(float(recovered_use) - float(optimized_use))\n            ),\n            "扰动后恢复使用次数变化率": format_relative_change_text(\n                float(optimized_use), float(recovered_use)\n            ),\n            "原始占用总时长(分钟)": f"{float(original_minutes):.2f}",\n            "非扰动优化占用总时长(分钟)": f"{float(optimized_minutes):.2f}",\n            "扰动后恢复占用总时长(分钟)": f"{float(recovered_minutes):.2f}",\n            "非扰动优化占用时长变化(分钟)": f"{float(optimized_minutes) - float(original_minutes):+.2f}",\n            "非扰动优化占用时长变化率": format_relative_change_text(\n                float(original_minutes), float(optimized_minutes)\n            ),\n            "扰动后恢复占用时长变化(分钟)": f"{float(recovered_minutes) - float(optimized_minutes):+.2f}",\n            "扰动后恢复占用时长变化率": format_relative_change_text(\n                float(optimized_minutes), float(recovered_minutes)\n            ),\n            "原始占用强度": f"{float(original_intensity):.4f}",\n            "非扰动优化占用强度": f"{float(optimized_intensity):.4f}",\n            "扰动后恢复占用强度": f"{float(recovered_intensity):.4f}",\n            "非扰动优化占用强度变化": f"{float(optimized_intensity) - float(original_intensity):+.4f}",\n            "非扰动优化占用强度变化率": format_relative_change_text(\n                float(original_intensity), float(optimized_intensity)\n            ),\n            "扰动后恢复占用强度变化": f"{float(recovered_intensity) - float(optimized_intensity):+.4f}",\n            "扰动后恢复占用强度变化率": format_relative_change_text(\n                float(optimized_intensity), float(recovered_intensity)\n            ),\n            "原始峰值并发": int(round(float(original_peak))),\n            "非扰动优化峰值并发": int(round(float(optimized_peak))),\n            "扰动后恢复峰值并发": int(round(float(recovered_peak))),\n            "非扰动优化峰值并发变化": int(\n                round(float(optimized_peak) - float(original_peak))\n            ),\n            "非扰动优化峰值并发变化率": format_relative_change_text(\n                float(original_peak), float(optimized_peak)\n            ),\n            "扰动后恢复峰值并发变化": int(\n                round(float(recovered_peak) - float(optimized_peak))\n            ),\n            "扰动后恢复峰值并发变化率": format_relative_change_text(\n                float(optimized_peak), float(recovered_peak)\n            ),\n            "原始均衡性": "--",\n            "非扰动优化均衡性": "--",\n            "扰动后恢复均衡性": "--",\n            "非扰动优化均衡性变化": "--",\n            "非扰动优化均衡性优化率": "--",\n            "扰动后恢复均衡性变化": "--",\n            "扰动后恢复均衡性优化率": "--",\n            "优化说明": note,\n        }\n\n    for track in CANONICAL_TRACKS:\n        original_stat = original_tracks.get(\n            track,\n            {"使用次数": 0.0, "占用总时长": 0.0, "占用强度": 0.0, "峰值并发": 0.0},\n        )\n        optimized_stat = optimized_tracks.get(\n            track,\n            {"使用次数": 0.0, "占用总时长": 0.0, "占用强度": 0.0, "峰值并发": 0.0},\n        )\n        recovered_stat = recovered_tracks.get(\n            track,\n            {"使用次数": 0.0, "占用总时长": 0.0, "占用强度": 0.0, "峰值并发": 0.0},\n        )\n        rows.append(\n            make_balance_row(\n                "股道",\n                track,\n                original_stat["使用次数"],\n                optimized_stat["使用次数"],\n                recovered_stat["使用次数"],\n                original_stat["占用总时长"],\n                optimized_stat["占用总时长"],\n                recovered_stat["占用总时长"],\n                original_stat["占用强度"],\n                optimized_stat["占用强度"],\n                recovered_stat["占用强度"],\n                original_stat["峰值并发"],\n                optimized_stat["峰值并发"],\n                recovered_stat["峰值并发"],\n                "用于观察股道使用是否更均衡、是否打散集中分配。",\n            )\n        )\n\n    for throat in ["西咽喉", "东咽喉", "南咽喉", "始发端", "终到端"]:\n        original_stat = original_throats.get(\n            throat,\n            {"使用次数": 0.0, "占用总时长": 0.0, "占用强度": 0.0, "峰值并发": 0.0},\n        )\n        optimized_stat = optimized_throats.get(\n            throat,\n            {"使用次数": 0.0, "占用总时长": 0.0, "占用强度": 0.0, "峰值并发": 0.0},\n        )\n        recovered_stat = recovered_throats.get(\n            throat,\n            {"使用次数": 0.0, "占用总时长": 0.0, "占用强度": 0.0, "峰值并发": 0.0},\n        )\n        rows.append(\n            make_balance_row(\n                "咽喉",\n                throat,\n                original_stat["使用次数"],\n                optimized_stat["使用次数"],\n                recovered_stat["使用次数"],\n                original_stat["占用总时长"],\n                optimized_stat["占用总时长"],\n                recovered_stat["占用总时长"],\n                original_stat["占用强度"],\n                optimized_stat["占用强度"],\n                recovered_stat["占用强度"],\n                original_stat["峰值并发"],\n                optimized_stat["峰值并发"],\n                recovered_stat["峰值并发"],\n                "用于观察咽喉资源压力是否下降。",\n            )\n        )\n\n    top_original_tracks = sorted(\n        original_tracks.items(),\n        key=lambda item: item[1].get("占用总时长", 0),\n        reverse=True,\n    )[:3]\n    top_optimized_tracks = sorted(\n        optimized_tracks.items(),\n        key=lambda item: item[1].get("占用总时长", 0),\n        reverse=True,\n    )[:3]\n    top_recovered_tracks = sorted(\n        recovered_tracks.items(),\n        key=lambda item: item[1].get("占用总时长", 0),\n        reverse=True,\n    )[:3]\n    rows.append(\n        make_balance_row(\n            "瓶颈资源",\n            "股道瓶颈前3位",\n            "、".join(\n                f"{name}:{int(stat[\'使用次数\'])}" for name, stat in top_original_tracks\n            ),\n            "、".join(\n                f"{name}:{int(stat[\'使用次数\'])}" for name, stat in top_optimized_tracks\n            ),\n            "、".join(\n                f"{name}:{int(stat[\'使用次数\'])}" for name, stat in top_recovered_tracks\n            ),\n            "、".join(\n                f"{name}:{stat[\'占用总时长\']:.2f}" for name, stat in top_original_tracks\n            ),\n            "、".join(\n                f"{name}:{stat[\'占用总时长\']:.2f}"\n                for name, stat in top_optimized_tracks\n            ),\n            "、".join(\n                f"{name}:{stat[\'占用总时长\']:.2f}"\n                for name, stat in top_recovered_tracks\n            ),\n            "、".join(\n                f"{name}:{stat[\'占用强度\']:.4f}" for name, stat in top_original_tracks\n            ),\n            "、".join(\n                f"{name}:{stat[\'占用强度\']:.4f}" for name, stat in top_optimized_tracks\n            ),\n            "、".join(\n                f"{name}:{stat[\'占用强度\']:.4f}" for name, stat in top_recovered_tracks\n            ),\n            "、".join(\n                f"{name}:{int(stat[\'峰值并发\'])}" for name, stat in top_original_tracks\n            ),\n            "、".join(\n                f"{name}:{int(stat[\'峰值并发\'])}" for name, stat in top_optimized_tracks\n            ),\n            "、".join(\n                f"{name}:{int(stat[\'峰值并发\'])}" for name, stat in top_recovered_tracks\n            ),\n            "用于观察原本过度集中的股道是否被打散。",\n        )\n    )\n\n    top_original_throats = sorted(\n        original_throats.items(),\n        key=lambda item: item[1].get("占用强度", 0),\n        reverse=True,\n    )[:3]\n    top_optimized_throats = sorted(\n        optimized_throats.items(),\n        key=lambda item: item[1].get("占用强度", 0),\n        reverse=True,\n    )[:3]\n    top_recovered_throats = sorted(\n        recovered_throats.items(),\n        key=lambda item: item[1].get("占用强度", 0),\n        reverse=True,\n    )[:3]\n    rows.append(\n        make_balance_row(\n            "瓶颈资源",\n            "咽喉瓶颈前3位",\n            "、".join(\n                f"{name}:{int(stat[\'使用次数\'])}" for name, stat in top_original_throats\n            ),\n            "、".join(\n                f"{name}:{int(stat[\'使用次数\'])}"\n                for name, stat in top_optimized_throats\n            ),\n            "、".join(\n                f"{name}:{int(stat[\'使用次数\'])}"\n                for name, stat in top_recovered_throats\n            ),\n            "、".join(\n                f"{name}:{stat[\'占用总时长\']:.2f}"\n                for name, stat in top_original_throats\n            ),\n            "、".join(\n                f"{name}:{stat[\'占用总时长\']:.2f}"\n                for name, stat in top_optimized_throats\n            ),\n            "、".join(\n                f"{name}:{stat[\'占用总时长\']:.2f}"\n                for name, stat in top_recovered_throats\n            ),\n            "、".join(\n                f"{name}:{stat[\'占用强度\']:.4f}" for name, stat in top_original_throats\n            ),\n            "、".join(\n                f"{name}:{stat[\'占用强度\']:.4f}" for name, stat in top_optimized_throats\n            ),\n            "、".join(\n                f"{name}:{stat[\'占用强度\']:.4f}" for name, stat in top_recovered_throats\n            ),\n            "、".join(\n                f"{name}:{int(stat[\'峰值并发\'])}" for name, stat in top_original_throats\n            ),\n            "、".join(\n                f"{name}:{int(stat[\'峰值并发\'])}"\n                for name, stat in top_optimized_throats\n            ),\n            "、".join(\n                f"{name}:{int(stat[\'峰值并发\'])}"\n                for name, stat in top_recovered_throats\n            ),\n            "用于观察局部瓶颈区压力是否下降。",\n        )\n    )\n    return rows\n\n\n\n\n'
EMBEDDED_FRANKFURT_STATION_SOURCE = '"""法兰克福中央车站基础优化模块兼容层。"""\n\nfrom __future__ import annotations\n\nimport importlib.util\nimport csv\nimport sys\nfrom pathlib import Path\n\n\n_SOURCE = Path(__file__).with_name("_embedded_station_resource_model.py")\n_SPEC = importlib.util.spec_from_file_location("frankfurt_track_base_impl", _SOURCE)\nif _SPEC is None or _SPEC.loader is None:\n    raise RuntimeError(f"无法加载基础优化实现：{_SOURCE}")\n\n_MODULE = importlib.util.module_from_spec(_SPEC)\nsys.modules[_SPEC.name] = _MODULE\n_SPEC.loader.exec_module(_MODULE)\n\n# Frankfurt (Main) Hbf topology used by the Frankfurt experiment.\n# The embedded base module supplies shared data structures and conflict calculations.\n# Frankfurt-specific track and throat resources are overridden before re-export.\nFRANKFURT_TRACKS = tuple(str(index) for index in range(1, 30))\nFRANKFURT_ZONE_ORDER = {\n    "北翼到发场": 0,\n    "中北到发场": 1,\n    "中央到发场": 2,\n    "中南到发场": 3,\n    "南翼到发场": 4,\n    "外侧机走/备用股道": 5,\n    "其他股道带": 6,\n}\nFRANKFURT_PARTITION_ORDER = {\n    "西咽喉": ("W1", "W2", "W3", "W4", "W5", "W6"),\n    "东咽喉": ("E1", "E2", "E3", "E4", "E5", "E6"),\n    "南咽喉": ("S1", "S2", "S3", "S4", "S5", "S6"),\n    "始发端": ("O1", "O2", "O3", "O4", "O5", "O6"),\n    "终到端": ("T1", "T2", "T3", "T4", "T5", "T6"),\n}\n\n\ndef frankfurt_classify_track_zone(track: str) -> str:\n    if not str(track).isdigit():\n        return "其他股道带"\n    number = int(track)\n    if number <= 5:\n        return "北翼到发场"\n    if number <= 10:\n        return "中北到发场"\n    if number <= 16:\n        return "中央到发场"\n    if number <= 21:\n        return "中南到发场"\n    if number <= 26:\n        return "南翼到发场"\n    if number <= 29:\n        return "外侧机走/备用股道"\n    return "其他股道带"\n\n\ndef frankfurt_zone_code(zone: str) -> str:\n    return {\n        "北翼到发场": "FZ1",\n        "中北到发场": "FZ2",\n        "中央到发场": "FZ3",\n        "中南到发场": "FZ4",\n        "南翼到发场": "FZ5",\n        "外侧机走/备用股道": "FZ6",\n        "其他股道带": "FZU",\n    }.get(zone, "FZU")\n\n\ndef frankfurt_flank_zone_label(zone: str) -> str:\n    return {\n        "北翼到发场": "北翼防护带",\n        "中北到发场": "中北防护带",\n        "中央到发场": "中央防护带",\n        "中南到发场": "中南防护带",\n        "南翼到发场": "南翼防护带",\n        "外侧机走/备用股道": "外侧防护带",\n        "其他股道带": "其他防护带",\n    }.get(zone, "其他防护带")\n\n\ndef frankfurt_throat_zone_conflicts(throat: str, zone_a: str, zone_b: str) -> bool:\n    if not zone_a or not zone_b:\n        return False\n    return zone_a == zone_b\n\n\ndef frankfurt_throat_partition(track: str, throat: str) -> str:\n    if not str(track).isdigit():\n        return ""\n    prefix = {\n        "西咽喉": "W",\n        "东咽喉": "E",\n        "南咽喉": "S",\n        "始发端": "O",\n        "终到端": "T",\n    }.get(throat)\n    if prefix is None:\n        return ""\n    number = int(track)\n    if not 1 <= number <= 29:\n        return ""\n    if number <= 5:\n        group = 1\n    elif number <= 10:\n        group = 2\n    elif number <= 16:\n        group = 3\n    elif number <= 21:\n        group = 4\n    elif number <= 26:\n        group = 5\n    else:\n        group = 6\n    return f"{prefix}{group}"\n\n\ndef frankfurt_throat_partition_relation(\n    throat: str, track_a: str, track_b: str\n) -> tuple[str, str, str] | None:\n    """Frankfurt Hbf uses grouped throat corridors in this experiment."""\n    part_a = frankfurt_throat_partition(track_a, throat)\n    part_b = frankfurt_throat_partition(track_b, throat)\n    if not part_a or not part_b:\n        return None\n    order = FRANKFURT_PARTITION_ORDER.get(throat, ())\n    if not order or part_a not in order or part_b not in order:\n        return None\n    if part_a == part_b:\n        return (part_a, part_b, "hard")\n    return None\n\n\ndef _frankfurt_operation_flags(\n    direction: str, in_throat: str, out_throat: str\n) -> tuple[bool, bool]:\n    direction_text = (direction or "").strip()\n    is_origin = direction_text in {"出发", "始发"} or in_throat == "始发端"\n    is_terminal = direction_text in {"到达", "终到"} or out_throat == "终到端"\n    return is_origin, is_terminal\n\n\ndef _frankfurt_normalize_operation_fields(\n    row: dict[str, str],\n) -> tuple[str, str, str, str, str]:\n    direction = (row[_MODULE.FIELD_DIRECTION] or "").strip()\n    prev_station = (row[_MODULE.FIELD_PREV_STATION] or "").strip()\n    next_station = (row[_MODULE.FIELD_NEXT_STATION] or "").strip()\n    in_throat = (row[_MODULE.FIELD_IN_THROAT] or "").strip()\n    out_throat = (row[_MODULE.FIELD_OUT_THROAT] or "").strip()\n    if direction in {"出发", "始发"}:\n        prev_station = ""\n        in_throat = "始发端"\n    if direction in {"到达", "终到"}:\n        next_station = ""\n        out_throat = "终到端"\n    return direction, prev_station, next_station, in_throat, out_throat\n\n\ndef _frankfurt_train_windows(\n    arrival_min: int,\n    departure_min: int,\n    dwell_min: int,\n    c_min: int,\n    ab_min: int,\n    b_min: int,\n    bc_min: int,\n    cd_min: int,\n    d_min: int,\n    de_min: int,\n    is_origin: bool,\n    is_terminal: bool,\n) -> tuple[int, int, int, int, int, int, int, int, int, int, int]:\n    if is_origin:\n        ab_min = b_min = bc_min = 0\n    if is_terminal:\n        cd_min = d_min = de_min = 0\n    dwell_end = max(departure_min, arrival_min + max(dwell_min, c_min))\n    inbound_start = arrival_min if is_origin else arrival_min - ab_min - b_min - bc_min\n    inbound_end = arrival_min\n    dwell_start = arrival_min\n    outbound_start = dwell_end if is_terminal else departure_min\n    outbound_end = dwell_end if is_terminal else departure_min + cd_min + d_min + de_min\n    return (\n        ab_min,\n        b_min,\n        bc_min,\n        cd_min,\n        d_min,\n        de_min,\n        inbound_start,\n        inbound_end,\n        dwell_start,\n        dwell_end,\n        outbound_start,\n        outbound_end,\n    )\n\n\ndef frankfurt_load_trains(csv_path: Path, encoding: str) -> list:\n    with Path(csv_path).open("r", encoding=encoding, newline="") as handle:\n        reader = csv.DictReader(handle)\n        rows = list(reader)\n\n    required_fields = {\n        _MODULE.FIELD_TRAIN_NO,\n        _MODULE.FIELD_TRIP_ID,\n        _MODULE.FIELD_DIRECTION,\n        _MODULE.FIELD_PLANNED_TRACK,\n        _MODULE.FIELD_PREV_STATION,\n        _MODULE.FIELD_NEXT_STATION,\n        _MODULE.FIELD_IN_THROAT,\n        _MODULE.FIELD_OUT_THROAT,\n        _MODULE.FIELD_ARRIVAL,\n        _MODULE.FIELD_DEPARTURE,\n        _MODULE.FIELD_DWELL,\n        _MODULE.FIELD_AC,\n        _MODULE.FIELD_C,\n        _MODULE.FIELD_CE,\n    }\n    missing_fields = sorted(required_fields.difference(reader.fieldnames or []))\n    if missing_fields:\n        raise KeyError(f"输入文件缺少字段：{\', \'.join(missing_fields)}")\n\n    trains = []\n    input_start, input_end = _MODULE.INPUT_TIME_WINDOW\n    for row in rows:\n        direction, prev_station, next_station, in_throat, out_throat = (\n            _frankfurt_normalize_operation_fields(row)\n        )\n        is_origin, is_terminal = _frankfurt_operation_flags(\n            direction, in_throat, out_throat\n        )\n        arrival_min = _MODULE.parse_clock_to_minutes(row[_MODULE.FIELD_ARRIVAL])\n        departure_min = _MODULE.parse_clock_to_minutes(row[_MODULE.FIELD_DEPARTURE])\n        dwell_min = _MODULE.parse_minutes(row[_MODULE.FIELD_DWELL])\n        c_min = _MODULE.parse_minutes(row[_MODULE.FIELD_C])\n        # 读 3 段时长字段，再拆成 6 个内部子段（保持下游代码兼容）\n        ac_min_total = _MODULE.parse_minutes(row[_MODULE.FIELD_AC])\n        ce_min_total = _MODULE.parse_minutes(row[_MODULE.FIELD_CE])\n        ab_split = int(round(ac_min_total * 0.45))\n        b_split = int(round(ac_min_total * 0.22))\n        bc_split = max(0, ac_min_total - ab_split - b_split)\n        cd_split = int(round(ce_min_total * 0.30))\n        d_split = int(round(ce_min_total * 0.30))\n        de_split = max(0, ce_min_total - cd_split - d_split)\n        (\n            ab_min,\n            b_min,\n            bc_min,\n            cd_min,\n            d_min,\n            de_min,\n            inbound_start,\n            inbound_end,\n            dwell_start,\n            dwell_end,\n            outbound_start,\n            outbound_end,\n        ) = _frankfurt_train_windows(\n            arrival_min,\n            departure_min,\n            dwell_min,\n            c_min,\n            ab_split,\n            b_split,\n            bc_split,\n            cd_split,\n            d_split,\n            de_split,\n            is_origin,\n            is_terminal,\n        )\n        if (\n            _MODULE.overlap_minutes(\n                inbound_start, max(outbound_end, dwell_end), input_start, input_end\n            )\n            <= 0\n        ):\n            continue\n\n        index = len(trains) + 1\n        trains.append(\n            _MODULE.TrainRecord(\n                index=index,\n                record_id=f"R{index:03d}",\n                train_no=(row[_MODULE.FIELD_TRAIN_NO] or "").strip(),\n                trip_id=(row[_MODULE.FIELD_TRIP_ID] or "").strip(),\n                direction=direction,\n                planned_track=(row[_MODULE.FIELD_PLANNED_TRACK] or "").strip(),\n                prev_station=prev_station,\n                next_station=next_station,\n                in_throat=in_throat,\n                out_throat=out_throat,\n                arrival_text=(row[_MODULE.FIELD_ARRIVAL] or "").strip(),\n                departure_text=(row[_MODULE.FIELD_DEPARTURE] or "").strip(),\n                arrival_min=arrival_min,\n                departure_min=departure_min,\n                dwell_min=dwell_min,\n                ab_min=ab_min,\n                b_min=b_min,\n                bc_min=bc_min,\n                c_min=c_min,\n                cd_min=cd_min,\n                d_min=d_min,\n                de_min=de_min,\n                inbound_start=inbound_start,\n                inbound_end=inbound_end,\n                dwell_start=dwell_start,\n                dwell_end=dwell_end,\n                outbound_start=outbound_start,\n                outbound_end=outbound_end,\n            )\n        )\n\n    if not trains:\n        raise ValueError("输入数据在 08:00-10:00 时间窗内没有可用列车。")\n    return trains\n\n\ndef frankfurt_rebuild_train_record(\n    train,\n    arrival_shift: int = 0,\n    departure_shift: int = 0,\n    ab_delta: int = 0,\n    b_delta: int = 0,\n    bc_delta: int = 0,\n    cd_delta: int = 0,\n    d_delta: int = 0,\n    de_delta: int = 0,\n):\n    arrival_min = max(0, train.arrival_min + arrival_shift)\n    departure_min = max(arrival_min, train.departure_min + departure_shift)\n    is_origin, is_terminal = _frankfurt_operation_flags(\n        train.direction, train.in_throat, train.out_throat\n    )\n    (\n        ab_min,\n        b_min,\n        bc_min,\n        cd_min,\n        d_min,\n        de_min,\n        inbound_start,\n        inbound_end,\n        dwell_start,\n        dwell_end,\n        outbound_start,\n        outbound_end,\n    ) = _frankfurt_train_windows(\n        arrival_min,\n        departure_min,\n        train.dwell_min,\n        train.c_min,\n        max(0, train.ab_min + ab_delta),\n        max(0, train.b_min + b_delta),\n        max(0, train.bc_min + bc_delta),\n        max(0, train.cd_min + cd_delta),\n        max(0, train.d_min + d_delta),\n        max(0, train.de_min + de_delta),\n        is_origin,\n        is_terminal,\n    )\n    return _MODULE.TrainRecord(\n        index=train.index,\n        record_id=train.record_id,\n        train_no=train.train_no,\n        trip_id=train.trip_id,\n        direction=train.direction,\n        planned_track=train.planned_track,\n        prev_station=train.prev_station,\n        next_station=train.next_station,\n        in_throat=train.in_throat,\n        out_throat=train.out_throat,\n        arrival_text=f"{_MODULE.format_minutes_as_clock(arrival_min)}:00",\n        departure_text=f"{_MODULE.format_minutes_as_clock(departure_min)}:00",\n        arrival_min=arrival_min,\n        departure_min=departure_min,\n        dwell_min=train.dwell_min,\n        ab_min=ab_min,\n        b_min=b_min,\n        bc_min=bc_min,\n        c_min=train.c_min,\n        cd_min=cd_min,\n        d_min=d_min,\n        de_min=de_min,\n        inbound_start=inbound_start,\n        inbound_end=inbound_end,\n        dwell_start=dwell_start,\n        dwell_end=dwell_end,\n        outbound_start=outbound_start,\n        outbound_end=outbound_end,\n    )\n\n\ndef _first_channel(track: str, throat: str, movement: str, operation_type: str) -> str:\n    variants = _MODULE.channel_variants(track, throat, movement, operation_type)\n    return variants[0] if variants else ""\n\n\ndef frankfurt_planned_option(\n    train, option_id: str | None = None, candidate_rank: int = 0\n):\n    option_id = option_id or f"x_{train.index:03d}_00"\n    zone = frankfurt_classify_track_zone(train.planned_track)\n    in_partition = frankfurt_throat_partition(train.planned_track, train.in_throat)\n    out_partition = frankfurt_throat_partition(train.planned_track, train.out_throat)\n    inbound_channel = _first_channel(\n        train.planned_track, train.in_throat, "接车", train.operation_type\n    )\n    outbound_channel = _first_channel(\n        train.planned_track, train.out_throat, "发车", train.operation_type\n    )\n    route_code = (\n        f"PLAN-{train.record_id}-{_MODULE.direction_code(train.direction)}-"\n        f"{_MODULE.throat_code(train.in_throat)}{train.planned_track}-"\n        f"{_MODULE.throat_code(train.out_throat)}"\n    )\n    resources = _MODULE.build_resource_windows(\n        train=train,\n        option_id=option_id,\n        route_code=route_code,\n        track=train.planned_track,\n        in_throat=train.in_throat,\n        out_throat=train.out_throat,\n        inbound_channel=inbound_channel,\n        outbound_channel=outbound_channel,\n    )\n    return _MODULE.CandidateRoutePlan(\n        option_id=option_id,\n        train_id=train.record_id,\n        track=train.planned_track,\n        in_throat=train.in_throat,\n        out_throat=train.out_throat,\n        in_partition=in_partition,\n        out_partition=out_partition,\n        inbound_channel=inbound_channel,\n        outbound_channel=outbound_channel,\n        route_variant="标准通道",\n        route_code=route_code,\n        inbound_route_code=f"IN-{_MODULE.direction_code(train.direction)}-{inbound_channel}-{train.planned_track}",\n        outbound_route_code=f"OUT-{_MODULE.direction_code(train.direction)}-{outbound_channel}-{train.planned_track}",\n        route_family=f"{train.direction}_{train.in_throat}进_{train.out_throat}出_{zone}_标准通道",\n        source_level="原计划工程进路",\n        support_count=0,\n        route_score=0.0,\n        candidate_rank=candidate_rank,\n        operation_type=train.operation_type,\n        delay_risk_cost=0.0,\n        stability_cost=0.0,\n        balance_reward=0.0,\n        linear_cost=0.0,\n        note="法兰克福输入计划锁定候选：扰动发生前按原计划股道和原计划进路执行。",\n        resources=resources,\n    )\n\n\ndef frankfurt_build_candidate_route_plans(*args, **kwargs):\n    return _BASE_BUILD_CANDIDATE_ROUTE_PLANS(*args, **kwargs)\n\n\ndef frankfurt_build_plan_assignment(trains, options_by_train):\n    assignment = {}\n    for train in trains:\n        planned_signature = _MODULE.assignment_option_signature(\n            frankfurt_planned_option(train)\n        )\n        selected = next(\n            (\n                option\n                for option in options_by_train.get(train.record_id, [])\n                if _MODULE.assignment_option_signature(option) == planned_signature\n            ),\n            frankfurt_planned_option(train),\n        )\n        assignment[train.record_id] = selected\n    return assignment\n\n\ndef frankfurt_build_planned_assignment_ids(trains, options_by_train):\n    assignment = {}\n    for train in trains:\n        planned_signature = _MODULE.assignment_option_signature(\n            frankfurt_planned_option(train)\n        )\n        selected = next(\n            (\n                option.option_id\n                for option in options_by_train.get(train.record_id, [])\n                if _MODULE.assignment_option_signature(option) == planned_signature\n            ),\n            options_by_train[train.record_id][0].option_id,\n        )\n        assignment[train.record_id] = selected\n    return assignment\n\n\n_MODULE.CANONICAL_TRACKS = FRANKFURT_TRACKS\n_MODULE.TRACK_ZONE_ORDER = FRANKFURT_ZONE_ORDER\n_MODULE.THROAT_PARTITION_ORDER = FRANKFURT_PARTITION_ORDER\n_MODULE.FREE_SPACE_TRACK_LIMIT = 29\n_MODULE.FREE_SPACE_RICH_TRACK_LIMIT = 29\n_MODULE.FREE_SPACE_OPTION_LIMIT = 96\n_MODULE.FREE_SPACE_RICH_OPTION_LIMIT = 120\n_MODULE.classify_track_zone = frankfurt_classify_track_zone\n_MODULE.zone_code = frankfurt_zone_code\n_MODULE.flank_zone_label = frankfurt_flank_zone_label\n_MODULE.throat_zone_conflicts = frankfurt_throat_zone_conflicts\n_MODULE.throat_partition = frankfurt_throat_partition\n_MODULE.throat_partition_relation = frankfurt_throat_partition_relation\n_MODULE.load_trains = frankfurt_load_trains\n_MODULE.rebuild_train_record = frankfurt_rebuild_train_record\n_BASE_BUILD_CANDIDATE_ROUTE_PLANS = _MODULE.build_candidate_route_plans\n_MODULE.build_candidate_route_plans = frankfurt_build_candidate_route_plans\n_MODULE.build_plan_assignment = frankfurt_build_plan_assignment\n_MODULE.build_planned_assignment_ids = frankfurt_build_planned_assignment_ids\nfor _cache_name in (\n    "RESOURCE_WINDOW_CACHE",\n    "PAIR_CONFLICT_DETAIL_CACHE",\n    "PAIR_CONFLICT_DETAIL_COARSE_CACHE",\n):\n    _cache = getattr(_MODULE, _cache_name, None)\n    if _cache is not None:\n        _cache.clear()\n\nfor _name, _value in vars(_MODULE).items():\n    if not _name.startswith("__"):\n        globals()[_name] = _value\n'
EMBEDDED_EXPERIMENT_CORE_SOURCE = '"""\n法兰克福中央车站到发线运用优化实验\n\n目标：基于 Frankfurt (Main) Hbf 真实 GTFS 高峰数据，比较 QEA-NS\n主优化方法与 CP-SAT 对照组在突发扰动下的到发线/进路恢复能力。\n\n本文件是独立实验入口，不依赖其他实验脚本的实现状态。\n"""\n\nfrom __future__ import annotations\n\nimport argparse\nimport csv\nimport importlib.util\nimport math\nimport random\nimport sys\nimport time\nfrom collections import Counter\nfrom dataclasses import replace\nfrom pathlib import Path\n\ntry:\n    import numpy as np\nexcept ImportError as exc:  # pragma: no cover\n    raise RuntimeError("需要 numpy 才能运行 QEA-NS 实验。") from exc\n\ntry:\n    import openpyxl\n    from openpyxl.styles import Alignment, Border, Font, PatternFill, Side\n    from openpyxl.utils import get_column_letter\n\n    EXCEL_AVAILABLE = True\nexcept ImportError:\n    EXCEL_AVAILABLE = False\n\n\nSTATION_NAME = "法兰克福中央车站"\nSCHEME_PLAN = "原计划复核"\nSCHEME_QEA = "QEA-NS"\nSCHEME_CP_SAT = "CP-SAT"\nSCHEME_DISTURBED = "扰动基准"\nSUBPROBLEM_COST_SCALE = 100\n\n\ndef load_base_module():\n    return _load_embedded_module("frankfurt_track_base", "_embedded_frankfurt_hbf_station_model.py", EMBEDDED_FRANKFURT_STATION_SOURCE)\n\n\nbase = load_base_module()\n\n\n# 法兰克福入口直接比较扰动基准、QEA-NS 与 CP-SAT。\n# 候选生成阶段关闭基础库的强化改进过程。\nbase.RECOVERY_AGGRESSIVE_MODE = False\nbase.RECOVERY_AGGRESSIVE_REFINEMENT_PASSES = 0\nbase.RECOVERY_AGGRESSIVE_OUTER_LOOPS = 3\n\n\ndef parse_args():\n    parser = argparse.ArgumentParser(\n        description="法兰克福中央车站 QEA-NS vs CP-SAT 扰动恢复实验"\n    )\n    parser.add_argument(\n        "--input", type=str, default="frankfurt_hbf_gtfs_schedule.csv"\n    )\n    parser.add_argument("--output-dir", type=str, default="")\n    parser.add_argument("--max-route-candidates", type=int, default=5)\n    parser.add_argument("--seed", type=int, default=42)\n    parser.add_argument("--qea-time-limit", type=float, default=400.0)\n    parser.add_argument("--subproblem-time-limit", type=float, default=5.0)\n    parser.add_argument(\n        "--cp-sat-time-limit",\n        dest="cp_sat_time_limit",\n        type=float,\n        default=None,\n    )\n    parser.add_argument("--qea-pop-size", type=int, default=50)\n    parser.add_argument("--qea-max-generations", type=int, default=500)\n    parser.add_argument("--safety-wait-rounds", type=int, default=160)\n    args = parser.parse_args()\n    if args.cp_sat_time_limit is None:\n        args.cp_sat_time_limit = args.subproblem_time_limit\n    return args\n\n\ndef import_cp_model():\n    try:\n        from ortools.sat.python import cp_model\n\n        return cp_model\n    except Exception:\n        return None\n\n\ndef import_mip_solver():\n    try:\n        from ortools.linear_solver import pywraplp\n\n        return pywraplp\n    except Exception:\n        return None\n\n\ndef option_cost(option_id, linear_costs):\n    return max(0, int(round(linear_costs.get(option_id, 0.0) * SUBPROBLEM_COST_SCALE)))\n\n\ndef pair_cost(\n    option_a, option_b, pair_costs, hard_counts, hard_risks, strict_hard=True\n):\n    soft = int(\n        round(pair_costs.get(option_a, {}).get(option_b, 0.0) * SUBPROBLEM_COST_SCALE)\n    )\n    if strict_hard:\n        return max(0, soft)\n    hard = hard_counts.get(option_a, {}).get(option_b, 0) * 1_000_000_000\n    risk = int(round(hard_risks.get(option_a, {}).get(option_b, 0.0) * 100_000))\n    return max(0, soft + hard + risk)\n\n\ndef solve_subproblem(\n    initial_assignment,\n    active_train_ids,\n    options_by_train,\n    linear_costs,\n    pair_costs_map,\n    hard_counts,\n    hard_risks,\n    time_limit_seconds,\n    strict_hard=True,\n    max_options_per_train=10,\n):\n    cp_model = import_cp_model()\n    if cp_model is None:\n        return None, "ORTools不可用"\n\n    selected = dict(initial_assignment or {})\n    active_train_ids = {tid for tid in active_train_ids if tid in options_by_train}\n    if not active_train_ids:\n        return dict(selected), "空模型"\n\n    if max_options_per_train and max_options_per_train > 0:\n        limited_options_by_train = {}\n        for tid, options in options_by_train.items():\n            current_option_id = selected.get(tid)\n            sorted_options = sorted(\n                options,\n                key=lambda option: (\n                    option.option_id != current_option_id,\n                    option.linear_cost,\n                    option.candidate_rank,\n                    option.option_id,\n                ),\n            )[:max_options_per_train]\n            if current_option_id and all(\n                option.option_id != current_option_id for option in sorted_options\n            ):\n                current = next(\n                    (\n                        option\n                        for option in options\n                        if option.option_id == current_option_id\n                    ),\n                    None,\n                )\n                if current is not None:\n                    sorted_options = [current, *sorted_options[:-1]]\n            limited_options_by_train[tid] = sorted_options\n        options_by_train = limited_options_by_train\n\n    model = cp_model.CpModel()\n    variables = {}\n    objective_terms = []\n\n    for tid in sorted(active_train_ids):\n        train_vars = []\n        for option in options_by_train[tid]:\n            var = model.NewBoolVar(option.option_id)\n            variables[option.option_id] = var\n            train_vars.append(var)\n            cost = option_cost(option.option_id, linear_costs)\n            if cost:\n                objective_terms.append(cost * var)\n            if selected.get(tid) == option.option_id:\n                model.AddHint(var, 1)\n            elif tid in selected:\n                model.AddHint(var, 0)\n        model.AddExactlyOne(train_vars)\n\n    active_order = sorted(active_train_ids)\n    fixed_selected = {\n        tid: option_id\n        for tid, option_id in selected.items()\n        if tid in options_by_train and tid not in active_train_ids\n    }\n    for left_index, left_tid in enumerate(active_order):\n        for right_tid in active_order[left_index + 1 :]:\n            for left_option in options_by_train[left_tid]:\n                left_var = variables[left_option.option_id]\n                for right_option in options_by_train[right_tid]:\n                    right_var = variables[right_option.option_id]\n                    if (\n                        strict_hard\n                        and hard_counts.get(left_option.option_id, {}).get(\n                            right_option.option_id, 0\n                        )\n                        > 0\n                    ):\n                        model.AddBoolOr([left_var.Not(), right_var.Not()])\n                        continue\n                    cost = pair_cost(\n                        left_option.option_id,\n                        right_option.option_id,\n                        pair_costs_map,\n                        hard_counts,\n                        hard_risks,\n                        strict_hard=strict_hard,\n                    )\n                    if cost <= 0:\n                        continue\n                    both = model.NewBoolVar(\n                        f"pair_{left_option.option_id}_{right_option.option_id}"\n                    )\n                    model.Add(both <= left_var)\n                    model.Add(both <= right_var)\n                    model.Add(both >= left_var + right_var - 1)\n                    objective_terms.append(cost * both)\n\n    for tid in active_order:\n        for option in options_by_train[tid]:\n            var = variables[option.option_id]\n            for fixed_tid, fixed_option_id in fixed_selected.items():\n                if fixed_tid == tid:\n                    continue\n                if (\n                    strict_hard\n                    and hard_counts.get(option.option_id, {}).get(fixed_option_id, 0)\n                    > 0\n                ):\n                    model.Add(var == 0)\n                    continue\n                fixed_cost = pair_cost(\n                    option.option_id,\n                    fixed_option_id,\n                    pair_costs_map,\n                    hard_counts,\n                    hard_risks,\n                    strict_hard=strict_hard,\n                )\n                if fixed_cost > 0:\n                    objective_terms.append(fixed_cost * var)\n\n    model.Minimize(sum(objective_terms) if objective_terms else 0)\n    solver = cp_model.CpSolver()\n    if time_limit_seconds is not None and float(time_limit_seconds) > 0:\n        solver.parameters.max_time_in_seconds = max(0.5, float(time_limit_seconds))\n    solver.parameters.num_search_workers = 1\n    status = solver.Solve(model)\n    status_name = solver.StatusName(status)\n    if status not in {cp_model.OPTIMAL, cp_model.FEASIBLE}:\n        return None, status_name\n\n    solved = dict(selected)\n    for tid in active_train_ids:\n        for option in options_by_train[tid]:\n            if solver.BooleanValue(variables[option.option_id]):\n                solved[tid] = option.option_id\n                break\n    return solved, status_name\n\n\ndef solve_mip_full_model(\n    initial_assignment,\n    active_train_ids,\n    options_by_train,\n    linear_costs,\n    pair_costs_map,\n    hard_counts,\n    hard_risks,\n    time_limit_seconds,\n    strict_hard=True,\n    solver_name="CBC",\n):\n    pywraplp = import_mip_solver()\n    if pywraplp is None:\n        return None, "ORTools线性求解器不可用"\n\n    solver_aliases = {\n        "CBC": ["CBC_MIXED_INTEGER_PROGRAMMING", "CBC"],\n        "SCIP": ["SCIP", "SCIP_MIXED_INTEGER_PROGRAMMING"],\n        "HIGHS": ["HIGHS", "HIGHS_MIXED_INTEGER_PROGRAMMING"],\n    }\n    requested_solver = str(solver_name or "CBC").upper()\n    solver = None\n    for alias in solver_aliases.get(requested_solver, [requested_solver]):\n        solver = pywraplp.Solver.CreateSolver(alias)\n        if solver is not None:\n            break\n    if solver is None:\n        return None, f"{requested_solver}求解器不可用"\n\n    selected = dict(initial_assignment or {})\n    active_train_ids = {tid for tid in active_train_ids if tid in options_by_train}\n    if not active_train_ids:\n        return dict(selected), "空模型"\n\n    solver.SetTimeLimit(int(max(0.5, float(time_limit_seconds)) * 1000))\n    variables = {}\n    objective = solver.Objective()\n    objective.SetMinimization()\n\n    for tid in sorted(active_train_ids):\n        train_vars = []\n        for option in options_by_train[tid]:\n            var = solver.IntVar(0, 1, option.option_id)\n            variables[option.option_id] = var\n            train_vars.append(var)\n            cost = option_cost(option.option_id, linear_costs)\n            if cost:\n                objective.SetCoefficient(var, cost)\n        solver.Add(sum(train_vars) == 1)\n\n    active_order = sorted(active_train_ids)\n    pair_counter = 0\n    for left_index, left_tid in enumerate(active_order):\n        for right_tid in active_order[left_index + 1 :]:\n            for left_option in options_by_train[left_tid]:\n                left_var = variables[left_option.option_id]\n                for right_option in options_by_train[right_tid]:\n                    right_var = variables[right_option.option_id]\n                    if (\n                        strict_hard\n                        and hard_counts.get(left_option.option_id, {}).get(\n                            right_option.option_id, 0\n                        )\n                        > 0\n                    ):\n                        solver.Add(left_var + right_var <= 1)\n                        continue\n                    cost = pair_cost(\n                        left_option.option_id,\n                        right_option.option_id,\n                        pair_costs_map,\n                        hard_counts,\n                        hard_risks,\n                        strict_hard=strict_hard,\n                    )\n                    if cost <= 0:\n                        continue\n                    pair_counter += 1\n                    both = solver.IntVar(0, 1, f"p_{pair_counter}")\n                    solver.Add(both <= left_var)\n                    solver.Add(both <= right_var)\n                    solver.Add(both >= left_var + right_var - 1)\n                    objective.SetCoefficient(both, cost)\n\n    if selected and hasattr(solver, "SetHint"):\n        hint_vars = []\n        hint_values = []\n        for tid in active_train_ids:\n            for option in options_by_train[tid]:\n                hint_vars.append(variables[option.option_id])\n                hint_values.append(1 if selected.get(tid) == option.option_id else 0)\n        solver.SetHint(hint_vars, hint_values)\n\n    status = solver.Solve()\n    status_map = {\n        pywraplp.Solver.OPTIMAL: "OPTIMAL",\n        pywraplp.Solver.FEASIBLE: "FEASIBLE",\n        pywraplp.Solver.INFEASIBLE: "INFEASIBLE",\n        pywraplp.Solver.UNBOUNDED: "UNBOUNDED",\n        pywraplp.Solver.ABNORMAL: "ABNORMAL",\n        pywraplp.Solver.NOT_SOLVED: "NOT_SOLVED",\n    }\n    status_name = status_map.get(status, str(status))\n    if status not in {pywraplp.Solver.OPTIMAL, pywraplp.Solver.FEASIBLE}:\n        return None, status_name\n\n    solved = dict(selected)\n    for tid in active_train_ids:\n        best_option = max(\n            options_by_train[tid],\n            key=lambda option: variables[option.option_id].solution_value(),\n        )\n        solved[tid] = best_option.option_id\n    return solved, status_name\n\n\ndef hard_count_for_selection(selected, hard_counts):\n    train_ids = sorted(selected)\n    total = 0\n    for index, train_id in enumerate(train_ids):\n        option_id = selected[train_id]\n        for other_id in train_ids[index + 1 :]:\n            total += hard_counts.get(option_id, {}).get(selected[other_id], 0)\n    return int(total)\n\n\ndef assignment_energy(selected, linear_costs, pair_costs_map, hard_counts, hard_risks):\n    return base.assignment_energy(\n        selected, linear_costs, pair_costs_map, hard_counts, hard_risks\n    )\n\n\ndef assignment_display_energy(selected, linear_costs, pair_costs_map):\n    total = 0.0\n    train_ids = sorted(selected)\n    for index, train_id in enumerate(train_ids):\n        option_id = selected[train_id]\n        total += linear_costs.get(option_id, 0.0)\n        for other_id in train_ids[index + 1 :]:\n            total += pair_costs_map.get(option_id, {}).get(selected[other_id], 0.0)\n    return total\n\n\ndef qea_optimize(\n    trains,\n    options_by_train,\n    linear_costs,\n    pair_costs_map,\n    hard_counts,\n    hard_risks,\n    train_map,\n    time_limit_seconds=400.0,\n    seed=42,\n    pop_size=50,\n    max_generations=500,\n    initial_assignment_ids=None,\n    reference_trains=None,\n    disturbance_minute=None,\n    frozen_train_ids=None,\n    max_safety_wait_rounds=160,\n):\n    rng = random.Random(seed)\n    np_rng = np.random.default_rng(seed)\n    start = time.perf_counter()\n    train_order = base.build_train_order(trains, options_by_train)\n\n    planned = (\n        dict(initial_assignment_ids)\n        if initial_assignment_ids\n        else base.build_planned_assignment_ids(trains, options_by_train)\n    )\n\n    def normalize_assignment(selected):\n        normalized = {}\n        for tid in train_order:\n            valid_option_ids = {option.option_id for option in options_by_train[tid]}\n            option_id = selected.get(tid)\n            if option_id not in valid_option_ids:\n                option_id = options_by_train[tid][0].option_id\n            normalized[tid] = option_id\n        return normalized\n\n    def evaluate(selected):\n        normalized = normalize_assignment(selected)\n        hard = hard_count_for_selection(normalized, hard_counts)\n        energy = assignment_energy(\n            normalized, linear_costs, pair_costs_map, hard_counts, hard_risks\n        )\n        return normalized, hard, energy\n\n    frozen_train_ids = set(frozen_train_ids or set())\n    landing_enabled = reference_trains is not None and disturbance_minute is not None\n    landing_cache = {}\n    landing_eval_rounds = min(max(1, int(max_safety_wait_rounds)), 40)\n\n    def qea_time_exhausted():\n        return time.perf_counter() - start > time_limit_seconds\n\n    def selected_cache_key(selected):\n        normalized = normalize_assignment(selected)\n        return tuple((tid, normalized[tid]) for tid in train_order)\n\n    def selected_assignment(selected):\n        normalized = normalize_assignment(selected)\n        assignment = base.build_assignment_lookup(normalized, options_by_train)\n        return base.rebuild_assignment_for_trains(trains, assignment)\n\n    def raw_hard_conflict_train_counts(selected):\n        assignment = selected_assignment(selected)\n        conflicts = base.collect_conflicts(trains, assignment, SCHEME_QEA)\n        counts = Counter()\n        for conflict in conflicts:\n            if conflict.conflict_level != "硬冲突":\n                continue\n            left_id = train_id_from_conflict_name(conflict.train1)\n            right_id = train_id_from_conflict_name(conflict.train2)\n            if left_id in options_by_train:\n                counts[left_id] += 1\n            if right_id in options_by_train:\n                counts[right_id] += 1\n        return counts\n\n    def landing_objective(selected):\n        normalized = normalize_assignment(selected)\n        key = selected_cache_key(normalized)\n        if key in landing_cache:\n            return landing_cache[key]\n\n        raw_hard = hard_count_for_selection(normalized, hard_counts)\n        display_energy = assignment_display_energy(\n            normalized, linear_costs, pair_costs_map\n        )\n        if raw_hard > 40:\n            result = (\n                (raw_hard, 10**9, 10**9, raw_hard, display_energy),\n                raw_hard,\n                display_energy,\n            )\n            landing_cache[key] = result\n            return result\n        if not landing_enabled:\n            result = (\n                (raw_hard, 10**9, 10**9, raw_hard, display_energy),\n                raw_hard,\n                display_energy,\n            )\n        else:\n            assignment = selected_assignment(normalized)\n            landed_trains, _, landed_conflicts, _, _ = iterative_safety_wait_recovery(\n                reference_trains=reference_trains,\n                disturbed_trains=trains,\n                disturbed_assignment=assignment,\n                options_by_train=options_by_train,\n                disturbance_minute=disturbance_minute,\n                frozen_train_ids=frozen_train_ids,\n                max_rounds=landing_eval_rounds,\n            )\n            conflict_stats = base.build_conflict_statistics(landed_conflicts)\n            delay_stats = base.compute_delay_statistics(reference_trains, landed_trains)\n            result = (\n                (\n                    conflict_stats["硬冲突项"],\n                    delay_stats["总晚点时长"],\n                    delay_stats["最大晚点时长"],\n                    raw_hard,\n                    display_energy,\n                ),\n                raw_hard,\n                display_energy,\n            )\n        if len(landing_cache) > 4000:\n            landing_cache.clear()\n        landing_cache[key] = result\n        return result\n\n    def landing_aware_conflict_repair(\n        selected,\n        max_passes=4,\n        option_limit=12,\n        train_limit=24,\n        max_evaluations=120,\n    ):\n        if not landing_enabled:\n            repaired, repaired_hard, repaired_energy = evaluate(selected)\n            return repaired, repaired_hard, repaired_energy, 0, None\n\n        repaired = normalize_assignment(selected)\n        best_objective, _, _ = landing_objective(repaired)\n        improvements = 0\n        evaluations = 0\n        for _ in range(max(1, max_passes)):\n            if qea_time_exhausted() or evaluations >= max_evaluations:\n                break\n            counts = raw_hard_conflict_train_counts(repaired)\n            if not counts:\n                break\n            improved_this_pass = False\n            ordered_train_ids = sorted(\n                counts,\n                key=lambda tid: (\n                    -counts[tid],\n                    train_map[tid].arrival_min,\n                    train_map[tid].index,\n                ),\n            )[:train_limit]\n            for tid in ordered_train_ids:\n                if qea_time_exhausted() or evaluations >= max_evaluations:\n                    break\n                current_option_id = repaired.get(tid)\n                candidate_options = sorted(\n                    options_by_train[tid],\n                    key=lambda option: (\n                        option.option_id != current_option_id,\n                        linear_costs.get(option.option_id, 0.0),\n                        option.candidate_rank,\n                        option.option_id,\n                    ),\n                )[:option_limit]\n                local_option_id = current_option_id\n                local_objective = best_objective\n                for option in candidate_options:\n                    if option.option_id == current_option_id:\n                        continue\n                    trial = dict(repaired)\n                    trial[tid] = option.option_id\n                    trial_objective, _, _ = landing_objective(trial)\n                    evaluations += 1\n                    if trial_objective < local_objective:\n                        local_objective = trial_objective\n                        local_option_id = option.option_id\n                if local_option_id != current_option_id:\n                    repaired[tid] = local_option_id\n                    best_objective = local_objective\n                    improvements += 1\n                    improved_this_pass = True\n            if not improved_this_pass:\n                break\n        repaired, repaired_hard, repaired_energy = evaluate(repaired)\n        return repaired, repaired_hard, repaired_energy, improvements, best_objective\n\n    best, best_hard, best_energy = evaluate(planned)\n    zero_generation = 0 if best_hard == 0 else -1\n    sample_count = 0\n    refinement_count = 0\n    landing_repair_count = 0\n    raw_conflict_repair_count = 0\n\n    def pair_matrix_value(matrix, train_a, option_a, train_b, option_b):\n        if train_a <= train_b:\n            return matrix.get(option_a, {}).get(option_b, 0)\n        return matrix.get(option_b, {}).get(option_a, 0)\n\n    def single_change_score(\n        selected, train_id, new_option_id, current_hard, current_energy\n    ):\n        old_option_id = selected[train_id]\n        if old_option_id == new_option_id:\n            return current_hard, current_energy\n\n        hard_delta = 0\n        risk_delta = 0.0\n        pair_delta = 0.0\n        for other_id, other_option_id in selected.items():\n            if other_id == train_id:\n                continue\n            old_hard = pair_matrix_value(\n                hard_counts, train_id, old_option_id, other_id, other_option_id\n            )\n            new_hard = pair_matrix_value(\n                hard_counts, train_id, new_option_id, other_id, other_option_id\n            )\n            old_risk = pair_matrix_value(\n                hard_risks, train_id, old_option_id, other_id, other_option_id\n            )\n            new_risk = pair_matrix_value(\n                hard_risks, train_id, new_option_id, other_id, other_option_id\n            )\n            old_pair = pair_matrix_value(\n                pair_costs_map, train_id, old_option_id, other_id, other_option_id\n            )\n            new_pair = pair_matrix_value(\n                pair_costs_map, train_id, new_option_id, other_id, other_option_id\n            )\n            hard_delta += new_hard - old_hard\n            risk_delta += new_risk - old_risk\n            pair_delta += new_pair - old_pair\n\n        linear_delta = linear_costs.get(new_option_id, 0.0) - linear_costs.get(\n            old_option_id, 0.0\n        )\n        new_hard = current_hard + hard_delta\n        new_energy = (\n            current_energy\n            + linear_delta\n            + pair_delta\n            + hard_delta * base.HARD_CONFLICT_PRIORITY\n            + risk_delta * base.HARD_RISK_PRIORITY\n        )\n        return int(new_hard), new_energy\n\n    def elite_coordinate_refinement(\n        selected, selected_hard, selected_energy, max_passes=4, option_limit=None\n    ):\n        refined = dict(selected)\n        refined_hard = selected_hard\n        refined_energy = selected_energy\n        improvements = 0\n\n        for pass_index in range(max(1, max_passes)):\n            if qea_time_exhausted():\n                break\n            improved_this_pass = False\n            conflict_counts = raw_hard_conflict_train_counts(refined)\n            if conflict_counts:\n                ordered_trains = sorted(\n                    conflict_counts,\n                    key=lambda train_id: (\n                        -conflict_counts[train_id],\n                        train_map[train_id].arrival_min,\n                        train_map[train_id].index,\n                    ),\n                )[:40]\n            else:\n                ordered_trains = list(train_order)[:40]\n            rng.shuffle(ordered_trains)\n            for tid in ordered_trains:\n                if qea_time_exhausted():\n                    break\n                current_option_id = refined.get(tid)\n                candidate_options = sorted(\n                    options_by_train[tid],\n                    key=lambda option: (\n                        linear_costs.get(option.option_id, 0.0),\n                        option.candidate_rank,\n                        option.option_id,\n                    ),\n                )\n                if option_limit is not None:\n                    candidate_options = candidate_options[:option_limit]\n                local_option_id = refined.get(tid)\n                local_hard = refined_hard\n                local_energy = refined_energy\n\n                for option in candidate_options:\n                    if option.option_id == current_option_id:\n                        continue\n                    trial_hard, trial_energy = single_change_score(\n                        refined,\n                        tid,\n                        option.option_id,\n                        refined_hard,\n                        refined_energy,\n                    )\n                    if (trial_hard, trial_energy) < (local_hard, local_energy):\n                        local_option_id = option.option_id\n                        local_hard = trial_hard\n                        local_energy = trial_energy\n\n                if (local_hard, local_energy) < (refined_hard, refined_energy):\n                    refined[tid] = local_option_id\n                    refined_hard = local_hard\n                    refined_energy = local_energy\n                    improvements += 1\n                    improved_this_pass = True\n\n            if not improved_this_pass:\n                break\n\n        refined, refined_hard, refined_energy = evaluate(refined)\n        return refined, refined_hard, refined_energy, improvements\n\n    history = [\n        {\n            "迭代轮次": 0,\n            "阶段": "QEA-NS初始化",\n            "硬冲突数": best_hard,\n            "能量值": f"{best_energy:.2f}",\n            "用时(s)": f"{time.perf_counter() - start:.3f}",\n            "备注": "",\n        }\n    ]\n\n    def apply_landing_repair(stage, generation, max_passes=2, option_limit=8):\n        nonlocal \\\n            best, \\\n            best_hard, \\\n            best_energy, \\\n            refinement_count, \\\n            landing_repair_count, \\\n            zero_generation\n        if not landing_enabled or qea_time_exhausted():\n            return False\n        if best_hard > 40:\n            return False\n        old_objective = landing_objective(best)[0]\n        repaired, repaired_hard, repaired_energy, improvements, new_objective = (\n            landing_aware_conflict_repair(\n                best,\n                max_passes=max_passes,\n                option_limit=option_limit,\n            )\n        )\n        if improvements <= 0:\n            return False\n        new_objective = (\n            landing_objective(repaired)[0] if new_objective is None else new_objective\n        )\n        if new_objective >= old_objective:\n            return False\n        best = dict(repaired)\n        best_hard = repaired_hard\n        best_energy = repaired_energy\n        refinement_count += improvements\n        landing_repair_count += improvements\n        if zero_generation < 0 and best_hard == 0:\n            zero_generation = generation if isinstance(generation, int) else 0\n        history.append(\n            {\n                "迭代轮次": generation,\n                "阶段": stage,\n                "硬冲突数": best_hard,\n                "能量值": f"{best_energy:.2f}",\n                "用时(s)": f"{time.perf_counter() - start:.3f}",\n                "备注": "",\n            }\n        )\n        return True\n\n    def apply_raw_conflict_repair(stage, generation):\n        nonlocal \\\n            best, \\\n            best_hard, \\\n            best_energy, \\\n            refinement_count, \\\n            raw_conflict_repair_count, \\\n            zero_generation\n        return False\n        if qea_time_exhausted() or best_hard <= 0:\n            return False\n        repaired_ids = base.repair_hard_conflicts(\n            trains=trains,\n            train_map=train_map,\n            options_by_train=options_by_train,\n            selected_option_ids=best,\n            linear_costs=linear_costs,\n            pair_costs=pair_costs_map,\n            hard_counts=hard_counts,\n            hard_risks=hard_risks,\n        )\n        repaired, repaired_hard, repaired_energy = evaluate(repaired_ids)\n        if (repaired_hard, repaired_energy) >= (best_hard, best_energy):\n            return False\n        best = dict(repaired)\n        best_hard = repaired_hard\n        best_energy = repaired_energy\n        refinement_count += 1\n        raw_conflict_repair_count += 1\n        if zero_generation < 0 and best_hard == 0:\n            zero_generation = generation if isinstance(generation, int) else 0\n        history.append(\n            {\n                "迭代轮次": generation,\n                "阶段": stage,\n                "硬冲突数": best_hard,\n                "能量值": f"{best_energy:.2f}",\n                "用时(s)": f"{time.perf_counter() - start:.3f}",\n                "备注": "",\n            }\n        )\n        return True\n\n    if best_hard == 0 and best_energy <= 1e-9:\n        elapsed = time.perf_counter() - start\n        history.append(\n            {\n                "迭代轮次": "final",\n                "阶段": "QEA-NS最终解",\n                "硬冲突数": best_hard,\n                "能量值": f"{best_energy:.2f}",\n                "用时(s)": f"{elapsed:.3f}",\n                "备注": "",\n            }\n        )\n        diagnostics = {\n            "求解时间(s)": elapsed,\n            "收敛代数": 0,\n            "邻域修复调用次数": 0,\n            "QEA-NS采样次数": sample_count,\n            "QEA-NS精英变异次数": refinement_count,\n            "QEA-NS落地修复次数": landing_repair_count,\n            "QEA-NS冲突修复次数": raw_conflict_repair_count,\n            "最终硬冲突数": best_hard,\n            "最终能量值": best_energy,\n        }\n        return best, history, diagnostics\n\n    refined, refined_hard, refined_energy, improvements = elite_coordinate_refinement(\n        best,\n        best_hard,\n        best_energy,\n        max_passes=2,\n        option_limit=8,\n    )\n    refinement_count += improvements\n    if (refined_hard, refined_energy) < (best_hard, best_energy):\n        best = dict(refined)\n        best_hard = refined_hard\n        best_energy = refined_energy\n        if zero_generation < 0 and best_hard == 0:\n            zero_generation = 0\n        history.append(\n            {\n                "迭代轮次": 0,\n                "阶段": "QEA-NS精英变异",\n                "硬冲突数": best_hard,\n                "能量值": f"{best_energy:.2f}",\n                "用时(s)": f"{time.perf_counter() - start:.3f}",\n                "备注": "",\n            }\n        )\n    apply_raw_conflict_repair("QEA-NS冲突驱动局部修复", 0)\n    apply_landing_repair("QEA-NS落地晚点修复", 0, max_passes=1, option_limit=6)\n\n    theta_pop = []\n    qea_stagnation_threshold = 45\n    qea_min_generations_after_feasible = min(max_generations, 80)\n    qea_mutation_strength = 0.35\n    qea_directed_mutation_rate = 0.45\n    qea_global_mutation_rate = 0.30\n    qea_directed_train_ids = set()\n    if reference_trains is not None:\n        delay_stats_for_mutation = base.compute_delay_statistics(\n            reference_trains, trains\n        )\n        qea_directed_train_ids.update(\n            train_id\n            for train_id, delay in delay_stats_for_mutation.get(\n                "delay_by_train", {}\n            ).items()\n            if delay > 0 and train_id in options_by_train\n        )\n    qea_directed_train_ids.update(raw_hard_conflict_train_counts(best))\n    qea_directed_train_ids = {\n        train_id for train_id in qea_directed_train_ids if train_id in options_by_train\n    }\n    for particle_index in range(max(1, pop_size)):\n        particle = {}\n        for tid in train_order:\n            options = options_by_train[tid]\n            n_options = len(options)\n            winner_idx = next(\n                (\n                    idx\n                    for idx, opt in enumerate(options)\n                    if opt.option_id == best.get(tid)\n                ),\n                0,\n            )\n            if particle_index == 0:\n                theta = np.full(n_options, np.pi / 2.0)\n                theta[winner_idx] = 0.0\n            elif particle_index < max(2, pop_size // 3):\n                theta = np_rng.normal(np.pi / 2.0, qea_mutation_strength, n_options)\n                theta[winner_idx] = np_rng.normal(0.0, 0.12)\n                theta = np.clip(theta, 0.0, np.pi / 2.0)\n            else:\n                theta = np_rng.uniform(0.0, np.pi / 2.0, n_options)\n            particle[tid] = theta\n        theta_pop.append(particle)\n\n    stagnation = 0\n    last_logged_generation = 0\n\n    for generation in range(1, max_generations + 1):\n        if time.perf_counter() - start > time_limit_seconds:\n            break\n\n        alpha = 0.14 * (1.0 - generation / max(1, max_generations)) + 0.015\n        candidates = [(best, best_hard, best_energy)]\n        for particle in theta_pop:\n            sampled = {}\n            for tid in train_order:\n                options = options_by_train[tid]\n                probs = np.square(np.cos(particle[tid]))\n                probs = np.clip(probs, 1e-9, None)\n                probs = probs / probs.sum()\n                idx = int(np_rng.choice(len(options), p=probs))\n                sampled[tid] = options[idx].option_id\n\n            if (\n                qea_directed_train_ids\n                and generation > 1\n                and rng.random() < qea_directed_mutation_rate\n            ):\n                directed_pool = list(qea_directed_train_ids)\n                sample_size = min(len(directed_pool), max(1, len(directed_pool) // 2))\n                for tid in rng.sample(directed_pool, sample_size):\n                    sampled[tid] = rng.choice(options_by_train[tid]).option_id\n\n            if generation > 1 and rng.random() < qea_global_mutation_rate:\n                sample_size = max(1, len(train_order) // 8)\n                for tid in rng.sample(train_order, sample_size):\n                    sampled[tid] = rng.choice(options_by_train[tid]).option_id\n\n            normalized, hard, energy = evaluate(sampled)\n            sample_count += 1\n            candidates.append((normalized, hard, energy))\n\n        candidates.sort(key=lambda item: (item[1], item[2]))\n        current, current_hard, current_energy = candidates[0]\n        if (current_hard, current_energy) < (best_hard, best_energy):\n            best = dict(current)\n            best_hard = current_hard\n            best_energy = current_energy\n            stagnation = 0\n        else:\n            stagnation += 1\n\n        if zero_generation < 0 and best_hard == 0:\n            zero_generation = generation\n\n        if generation % 50 == 0:\n            refined, refined_hard, refined_energy, improvements = (\n                elite_coordinate_refinement(\n                    best,\n                    best_hard,\n                    best_energy,\n                    max_passes=1,\n                    option_limit=6,\n                )\n            )\n            refinement_count += improvements\n            if (refined_hard, refined_energy) < (best_hard, best_energy):\n                best = dict(refined)\n                best_hard = refined_hard\n                best_energy = refined_energy\n                stagnation = 0\n                if zero_generation < 0 and best_hard == 0:\n                    zero_generation = generation\n                history.append(\n                    {\n                        "迭代轮次": generation,\n                        "阶段": "QEA-NS精英变异",\n                        "硬冲突数": best_hard,\n                        "能量值": f"{best_energy:.2f}",\n                        "用时(s)": f"{time.perf_counter() - start:.3f}",\n                        "备注": "",\n                    }\n                )\n        if generation % 100 == 0:\n            if apply_landing_repair(\n                "QEA-NS落地晚点修复",\n                generation,\n                max_passes=1,\n                option_limit=6,\n            ):\n                stagnation = 0\n\n        for particle in theta_pop:\n            for tid in train_order:\n                options = options_by_train[tid]\n                winner_idx = next(\n                    (\n                        idx\n                        for idx, opt in enumerate(options)\n                        if opt.option_id == best.get(tid)\n                    ),\n                    0,\n                )\n                target = np.full(len(options), np.pi / 2.0)\n                target[winner_idx] = 0.0\n                particle[tid] = (1.0 - alpha) * particle[tid] + alpha * target\n\n        if stagnation > qea_stagnation_threshold:\n            for particle in theta_pop:\n                if rng.random() < 0.35:\n                    reset_pool = list(qea_directed_train_ids) or train_order\n                    sample_size = min(len(reset_pool), max(1, len(reset_pool) // 2))\n                    for tid in rng.sample(reset_pool, sample_size):\n                        particle[tid] = np_rng.uniform(\n                            0.0, 2.0 * np.pi, len(options_by_train[tid])\n                        )\n            stagnation = 0\n\n        if (\n            generation <= 10\n            or generation % 10 == 0\n            or (best_hard == 0 and generation - last_logged_generation >= 10)\n        ):\n            history.append(\n                {\n                    "迭代轮次": generation,\n                    "阶段": "QEA-NS主循环",\n                    "硬冲突数": best_hard,\n                    "能量值": f"{best_energy:.2f}",\n                    "用时(s)": f"{time.perf_counter() - start:.3f}",\n                    "备注": "",\n                }\n            )\n            last_logged_generation = generation\n\n        if (\n            best_hard == 0\n            and generation >= qea_min_generations_after_feasible\n            and stagnation >= qea_stagnation_threshold\n        ):\n            break\n\n    refined, refined_hard, refined_energy, improvements = elite_coordinate_refinement(\n        best,\n        best_hard,\n        best_energy,\n        max_passes=2,\n        option_limit=8,\n    )\n    refinement_count += improvements\n    if (refined_hard, refined_energy) < (best_hard, best_energy):\n        best = dict(refined)\n        best_hard = refined_hard\n        best_energy = refined_energy\n    apply_raw_conflict_repair("QEA-NS最终冲突驱动局部修复", "final-repair")\n    apply_landing_repair(\n        "QEA-NS最终落地晚点修复", "final-repair", max_passes=2, option_limit=8\n    )\n\n    elapsed = time.perf_counter() - start\n    history.append(\n        {\n            "迭代轮次": "final",\n            "阶段": "QEA-NS最终解",\n            "硬冲突数": best_hard,\n            "能量值": f"{best_energy:.2f}",\n            "用时(s)": f"{elapsed:.3f}",\n            "备注": "",\n        }\n    )\n    diagnostics = {\n        "求解时间(s)": elapsed,\n        "收敛代数": zero_generation if zero_generation >= 0 else "",\n        "邻域修复调用次数": 0,\n        "QEA-NS采样次数": sample_count,\n        "QEA-NS精英变异次数": refinement_count,\n        "QEA-NS落地修复次数": landing_repair_count,\n        "QEA-NS冲突修复次数": raw_conflict_repair_count,\n        "最终硬冲突数": best_hard,\n        "最终能量值": best_energy,\n    }\n    return best, history, diagnostics\n\n\ndef qea_optimize(\n    trains,\n    options_by_train,\n    linear_costs,\n    pair_costs_map,\n    hard_counts,\n    hard_risks,\n    train_map,\n    time_limit_seconds=400.0,\n    subproblem_time_limit_seconds=5.0,\n    seed=42,\n    pop_size=50,\n    max_generations=500,\n    initial_assignment_ids=None,\n    reference_trains=None,\n    disturbance_minute=None,\n    frozen_train_ids=None,\n    max_safety_wait_rounds=160,\n):\n    start = time.perf_counter()\n    qea_ids, history, diagnostics = qea_optimize(\n        trains=trains,\n        options_by_train=options_by_train,\n        linear_costs=linear_costs,\n        pair_costs_map=pair_costs_map,\n        hard_counts=hard_counts,\n        hard_risks=hard_risks,\n        train_map=train_map,\n        time_limit_seconds=time_limit_seconds,\n        seed=seed,\n        pop_size=pop_size,\n        max_generations=max_generations,\n        initial_assignment_ids=initial_assignment_ids,\n        reference_trains=reference_trains,\n        disturbance_minute=disturbance_minute,\n        frozen_train_ids=frozen_train_ids,\n        max_safety_wait_rounds=max_safety_wait_rounds,\n    )\n    best_ids = dict(qea_ids)\n    best_hard = hard_count_for_selection(best_ids, hard_counts)\n    best_energy = assignment_energy(\n        best_ids, linear_costs, pair_costs_map, hard_counts, hard_risks\n    )\n    subproblem_calls = 0\n    def refine_with_subproblem(seed_ids):\n        nonlocal best_ids, best_hard, best_energy, subproblem_calls\n        seed_assignment = base.build_assignment_lookup(seed_ids, options_by_train)\n        seed_assignment = base.rebuild_assignment_for_trains(trains, seed_assignment)\n        seed_conflicts = base.collect_conflicts(\n            trains, seed_assignment, SCHEME_QEA\n        )\n        active_ids = set()\n        for conflict in seed_conflicts:\n            if conflict.conflict_level != "硬冲突":\n                continue\n            left_id = train_id_from_conflict_name(conflict.train1)\n            right_id = train_id_from_conflict_name(conflict.train2)\n            if left_id in options_by_train:\n                active_ids.add(left_id)\n            if right_id in options_by_train:\n                active_ids.add(right_id)\n        if not active_ids:\n            active_ids = {\n                train.record_id\n                for train in trains\n                if train.record_id in options_by_train\n            }\n        active_ids = set(\n            sorted(\n                active_ids,\n                key=lambda train_id: (\n                    train_map[train_id].arrival_min,\n                    train_map[train_id].index,\n                ),\n            )[:18]\n        )\n        subproblem_ids, _ = solve_subproblem(\n            initial_assignment=seed_ids,\n            active_train_ids=active_ids,\n            options_by_train=options_by_train,\n            linear_costs=linear_costs,\n            pair_costs_map=pair_costs_map,\n            hard_counts=hard_counts,\n            hard_risks=hard_risks,\n            time_limit_seconds=subproblem_time_limit_seconds,\n            strict_hard=False,\n            max_options_per_train=5,\n        )\n        subproblem_calls += 1\n        if subproblem_ids is None:\n            return\n        candidate_hard = hard_count_for_selection(subproblem_ids, hard_counts)\n        candidate_energy = assignment_energy(\n            subproblem_ids, linear_costs, pair_costs_map, hard_counts, hard_risks\n        )\n        if (candidate_hard, candidate_energy) <= (best_hard, best_energy):\n            best_ids = dict(subproblem_ids)\n            best_hard = candidate_hard\n            best_energy = candidate_energy\n\n    refine_with_subproblem(best_ids)\n\n    elapsed = time.perf_counter() - start\n    history.append(\n        {\n            "迭代轮次": "hybrid",\n            "阶段": "QEA-NS邻域精修",\n            "硬冲突数": best_hard,\n            "能量值": f"{best_energy:.2f}",\n            "用时(s)": f"{elapsed:.3f}",\n            "备注": "",\n        }\n    )\n    diagnostics = dict(diagnostics)\n    diagnostics["求解时间(s)"] = elapsed\n    diagnostics["邻域修复调用次数"] = diagnostics.get("邻域修复调用次数", 0) + subproblem_calls\n    diagnostics["最终硬冲突数"] = best_hard\n    diagnostics["最终能量值"] = best_energy\n    return best_ids, history, diagnostics\n\n\ndef prepend_assignment_options(trains, options_by_train, assignment):\n    if assignment is None:\n        return options_by_train\n\n    train_map = {train.record_id: train for train in trains}\n    updated = {}\n    for train_id, options in options_by_train.items():\n        baseline_option = assignment.get(train_id)\n        train = train_map.get(train_id)\n        if baseline_option is None or train is None:\n            updated[train_id] = options\n            continue\n\n        baseline_signature = base.assignment_option_signature(baseline_option)\n        deduplicated = [\n            option\n            for option in options\n            if base.assignment_option_signature(option) != baseline_signature\n        ]\n        matched = next(\n            (\n                option\n                for option in options\n                if base.assignment_option_signature(option) == baseline_signature\n            ),\n            None,\n        )\n        if matched is None:\n            rebuilt = base.rebuild_option_for_train(train, baseline_option)\n            matched = replace(\n                rebuilt,\n                option_id=f"rec_{train.index:03d}_00",\n                candidate_rank=rebuilt.candidate_rank\n                if rebuilt.candidate_rank >= 0\n                else 0,\n                note=f"{rebuilt.note};扰动基准候选",\n            )\n        updated[train_id] = [matched, *deduplicated]\n    return updated\n\n\ndef build_active_train_ids(\n    reference_trains,\n    disturbed_trains,\n    baseline_assignment,\n    source_train_ids,\n    disturbance_minute,\n):\n    delay_stats = base.compute_delay_statistics(reference_trains, disturbed_trains)\n    active_ids = set(source_train_ids or [])\n    active_ids.update(\n        train_id\n        for train_id, delay in delay_stats.get("delay_by_train", {}).items()\n        if delay > 0\n    )\n    conflicts = filter_conflicts_after_disturbance(\n        base.collect_conflicts(\n            disturbed_trains, baseline_assignment, "active-neighborhood"\n        ),\n        disturbed_trains,\n        disturbance_minute,\n    )\n    for conflict in conflicts:\n        if conflict.conflict_level != "硬冲突":\n            continue\n        left_id = train_id_from_conflict_name(conflict.train1)\n        right_id = train_id_from_conflict_name(conflict.train2)\n        if left_id in active_ids or right_id in active_ids:\n            active_ids.add(left_id)\n            active_ids.add(right_id)\n    valid_ids = {train.record_id for train in disturbed_trains}\n    return active_ids & valid_ids\n\n\ndef restrict_options_to_active_trains(options_by_train, baseline_ids, active_train_ids):\n    active_train_ids = set(active_train_ids or set())\n    restricted = {}\n    for train_id, options in options_by_train.items():\n        if train_id in active_train_ids:\n            restricted[train_id] = options\n            continue\n        baseline_option_id = baseline_ids.get(train_id)\n        fixed_option = next(\n            (option for option in options if option.option_id == baseline_option_id),\n            options[0],\n        )\n        restricted[train_id] = [fixed_option]\n    return restricted\n\n\nDELAY_CANDIDATE_MINUTES = (0, 5, 10, 15, 20, 30, 45)\n\n\ndef expand_options_with_delay_choices(options_by_train, active_train_ids):\n    active_train_ids = set(active_train_ids or set())\n    expanded = {}\n    for train_id, options in options_by_train.items():\n        delay_values = DELAY_CANDIDATE_MINUTES if train_id in active_train_ids else (0,)\n        expanded_options = []\n        for option in options:\n            for delay in delay_values:\n                if delay == 0:\n                    expanded_options.append(option)\n                    continue\n                delayed_option_id = f"{option.option_id}_d{delay:02d}"\n                shifted_resources = tuple(\n                    base.shift_resource_window(resource, delay)\n                    for resource in option.resources\n                )\n                expanded_options.append(\n                    replace(\n                        option,\n                        option_id=delayed_option_id,\n                        route_code=f"{option.route_code}-D{delay}",\n                        linear_cost=option.linear_cost + delay * 18.0,\n                        delay_risk_cost=option.delay_risk_cost + delay,\n                        candidate_rank=option.candidate_rank * 100 + delay,\n                        note=f"{option.note} 优化器内生延后{delay}分钟。",\n                        resources=shifted_resources,\n                    )\n                )\n        expanded[train_id] = expanded_options\n    return expanded\n\n\ndef prune_pre_disturbance_pair_maps(\n    trains,\n    options_by_train,\n    pair_costs_map,\n    hard_counts,\n    hard_risks,\n    disturbance_minute,\n):\n    if disturbance_minute is None:\n        return\n    inactive_train_ids = {\n        train.record_id for train in trains if train.window_end <= disturbance_minute\n    }\n    if len(inactive_train_ids) < 2:\n        return\n\n    option_to_train = {\n        option.option_id: train_id\n        for train_id, options in options_by_train.items()\n        for option in options\n    }\n    for matrix in (pair_costs_map, hard_counts, hard_risks):\n        for option_id, linked in list(matrix.items()):\n            train_id = option_to_train.get(option_id)\n            if train_id not in inactive_train_ids:\n                continue\n            for other_option_id in list(linked):\n                if option_to_train.get(other_option_id) in inactive_train_ids:\n                    del linked[other_option_id]\n\n\ndef build_problem(\n    trains,\n    library,\n    max_route_candidates,\n    rich_ids=None,\n    fixed_assignment=None,\n    frozen_train_ids=None,\n    disturbance_minute=None,\n):\n    options_by_train, train_map = base.build_candidate_route_plans(\n        trains=trains,\n        library=library,\n        max_route_candidates=max_route_candidates,\n        rich_candidate_train_ids=rich_ids,\n    )\n    options_by_train = prepend_assignment_options(\n        trains, options_by_train, fixed_assignment\n    )\n    frozen_train_ids = set(frozen_train_ids or set())\n    for train_id in frozen_train_ids:\n        if train_id in options_by_train:\n            options_by_train[train_id] = options_by_train[train_id][:1]\n    # 基础库 build_pairwise_conflict_maps 内部按同一排序只计算前40个候选。\n    # 这里同步裁剪，避免子问题求解/QEA-NS 选择到未进入 hard_counts 的候选而漏判冲突。\n    options_by_train = {\n        train_id: sorted(\n            options,\n            key=lambda option: (\n                option.candidate_rank,\n                option.linear_cost,\n                option.option_id,\n            ),\n        )[:40]\n        for train_id, options in options_by_train.items()\n    }\n    raw_pair_costs, hard_counts, hard_risks = base.build_pairwise_conflict_maps(\n        trains=trains,\n        options_by_train=options_by_train,\n        mode="full",\n        hard_soft_enabled=True,\n        use_variant_penalty=False,\n    )\n    pair_costs_map = base.build_weighted_pair_costs(\n        raw_pair_costs,\n        base.DEFAULT_MODEL_WEIGHTS["safety_weight"],\n    )\n    linear_costs = base.build_weighted_linear_costs(\n        options_by_train,\n        base.DEFAULT_MODEL_WEIGHTS["safety_weight"],\n        base.DEFAULT_MODEL_WEIGHTS["stability_weight"],\n    )\n    prune_pre_disturbance_pair_maps(\n        trains,\n        options_by_train,\n        pair_costs_map,\n        hard_counts,\n        hard_risks,\n        disturbance_minute,\n    )\n    return (\n        options_by_train,\n        train_map,\n        linear_costs,\n        pair_costs_map,\n        hard_counts,\n        hard_risks,\n    )\n\n\ndef realize_assignment_with_frozen_wait(\n    trains,\n    assignment_ids,\n    options_by_train,\n    disturbance_minute,\n    reference_trains,\n    frozen_train_ids,\n    max_safety_wait_rounds=160,\n):\n    assignment = base.build_assignment_lookup(assignment_ids, options_by_train)\n    conflicts = filter_conflicts_after_disturbance(\n        base.collect_conflicts(trains, assignment, "方案落地检查"),\n        trains,\n        disturbance_minute,\n    )\n    hard_conflicts = [\n        conflict for conflict in conflicts if conflict.conflict_level == "硬冲突"\n    ]\n    if not hard_conflicts:\n        return trains, assignment, conflicts, {}\n    return iterative_safety_wait_recovery(\n        reference_trains=reference_trains,\n        disturbed_trains=trains,\n        disturbed_assignment=assignment,\n        options_by_train=options_by_train,\n        disturbance_minute=disturbance_minute,\n        frozen_train_ids=frozen_train_ids,\n        max_rounds=max_safety_wait_rounds,\n    )[:4]\n\n\ndef collect_solution_metrics(\n    label,\n    trains,\n    reference_trains,\n    assignment_ids,\n    options_by_train,\n    linear_costs,\n    pair_costs_map,\n    hard_counts,\n    hard_risks,\n    disturbance_minute=None,\n    frozen_train_ids=None,\n    max_safety_wait_rounds=160,\n):\n    """评价某个股道方案的指标。\n    流程：股道方案 + 时刻表(trains) → 若有硬冲突则安全顺延消解 → 统计冲突/延迟/能量。\n    trains 必须是扰动传播后的时刻表（含源头晚点+传播顺延）。\n    """\n    if frozen_train_ids is None:\n        frozen_train_ids = set()\n    # 在方案评价前处理可通过顺延消解的资源冲突。\n    realized_trains, assignment, conflicts, _ = realize_assignment_with_frozen_wait(\n        trains=trains,\n        assignment_ids=assignment_ids,\n        options_by_train=options_by_train,\n        disturbance_minute=disturbance_minute,\n        reference_trains=reference_trains,\n        frozen_train_ids=frozen_train_ids,\n        max_safety_wait_rounds=max_safety_wait_rounds,\n    )\n    # 统计顺延处理后的剩余冲突。\n    conflict_stats = base.build_conflict_statistics(conflicts)\n    # 晚点按调整后时刻相对于原计划的偏差计算。\n    delay_stats = base.compute_delay_statistics(reference_trains, realized_trains)\n    # 展示能量不重复计入已单独报告的硬冲突罚项。\n    energy = assignment_display_energy(assignment_ids, linear_costs, pair_costs_map)\n    subtype_counts = build_interlocking_subtype_counts(conflicts)\n    return (\n        {\n            "方法": label,\n            "总冲突项": conflict_stats["总冲突项"],\n            "硬冲突项": conflict_stats["硬冲突项"],\n            "股道占用冲突项": conflict_stats["股道占用冲突项"],\n            "联锁类冲突项": conflict_stats["联锁类冲突项"],\n            **subtype_counts,\n            "总晚点时长": delay_stats["总晚点时长"],\n            "平均晚点时长": delay_stats["平均晚点时长"],\n            "最大晚点时长": delay_stats["最大晚点时长"],\n            "能量值": energy,\n            "顺延调整列车数": delay_stats.get("受影响列车数", 0),\n        },\n        assignment,\n        conflicts,\n    )\n\n\ndef collect_raw_solution_metrics(\n    label,\n    trains,\n    reference_trains,\n    assignment_ids,\n    options_by_train,\n    linear_costs=None,\n    pair_costs_map=None,\n    hard_counts=None,\n    hard_risks=None,\n):\n    assignment = base.build_assignment_lookup(assignment_ids, options_by_train)\n    assignment = base.rebuild_assignment_for_trains(trains, assignment)\n    conflicts = base.collect_conflicts(trains, assignment, label)\n    conflict_stats = base.build_conflict_statistics(conflicts)\n    delay_stats = base.compute_delay_statistics(reference_trains, trains)\n    subtype_counts = build_interlocking_subtype_counts(conflicts)\n    energy = ""\n    if (\n        linear_costs is not None\n        and pair_costs_map is not None\n        and hard_counts is not None\n        and hard_risks is not None\n    ):\n        energy = assignment_display_energy(assignment_ids, linear_costs, pair_costs_map)\n    return (\n        {\n            "方法": label,\n            "总冲突项": conflict_stats["总冲突项"],\n            "硬冲突项": conflict_stats["硬冲突项"],\n            "股道占用冲突项": conflict_stats["股道占用冲突项"],\n            "联锁类冲突项": conflict_stats["联锁类冲突项"],\n            **subtype_counts,\n            "总晚点时长": delay_stats["总晚点时长"],\n            "平均晚点时长": delay_stats["平均晚点时长"],\n            "最大晚点时长": delay_stats["最大晚点时长"],\n            "能量值": energy,\n            "顺延调整列车数": 0,\n        },\n        assignment,\n        conflicts,\n    )\n\n\ndef track_gap(assignment_a, assignment_b):\n    shared = set(assignment_a) & set(assignment_b)\n    return sum(\n        1 for tid in shared if assignment_a[tid].track != assignment_b[tid].track\n    )\n\n\ndef train_ids_fixed_before_disturbance(trains, disturbance_minute):\n    return {\n        train.record_id for train in trains if train.window_end <= disturbance_minute\n    }\n\n\ndef restore_pre_disturbance_plan(\n    reference_trains,\n    current_trains,\n    current_assignment,\n    plan_assignment,\n    frozen_train_ids,\n):\n    if not frozen_train_ids:\n        return current_trains, current_assignment\n\n    reference_by_id = {train.record_id: train for train in reference_trains}\n    current_by_id = {train.record_id: train for train in current_trains}\n    restored_assignment = dict(current_assignment)\n    for train_id in frozen_train_ids:\n        reference_train = reference_by_id.get(train_id)\n        plan_option = plan_assignment.get(train_id)\n        if reference_train is None or plan_option is None:\n            continue\n        current_by_id[train_id] = reference_train\n        restored_assignment[train_id] = base.rebuild_option_for_train(\n            reference_train, plan_option\n        )\n\n    restored_trains = [\n        current_by_id[train.record_id]\n        for train in sorted(current_by_id.values(), key=lambda item: item.index)\n    ]\n    restored_assignment = base.rebuild_assignment_for_trains(\n        restored_trains, restored_assignment\n    )\n    return restored_trains, restored_assignment\n\n\ndef count_frozen_assignment_changes(assignment, plan_assignment, frozen_train_ids):\n    changed = 0\n    for train_id in frozen_train_ids:\n        current = assignment.get(train_id)\n        planned = plan_assignment.get(train_id)\n        if current is None or planned is None:\n            continue\n        if base.assignment_option_signature(\n            current\n        ) != base.assignment_option_signature(planned):\n            changed += 1\n    return changed\n\n\nINTERLOCKING_SUBTYPE_METRICS = {\n    "接车进路锁闭冲突项": "接车进路锁闭冲突",\n    "发车进路锁闭冲突项": "发车进路锁闭冲突",\n    "道岔组冲突项": "道岔组冲突",\n    "进路交叉冲突项": "进路交叉冲突",\n    "防护带冲突项": "防护带冲突",\n    "咽喉能力冲突项": "咽喉能力冲突",\n}\n\n\ndef build_interlocking_subtype_counts(conflicts):\n    return {\n        metric_name: sum(\n            1 for conflict in conflicts if conflict.conflict_type == conflict_type\n        )\n        for metric_name, conflict_type in INTERLOCKING_SUBTYPE_METRICS.items()\n    }\n\n\ndef train_id_from_conflict_name(name):\n    return str(name).split("-", 1)[0]\n\n\ndef filter_conflicts_after_disturbance(conflicts, trains, disturbance_minute):\n    if disturbance_minute is None:\n        return conflicts\n    train_by_id = {train.record_id: train for train in trains}\n    filtered = []\n    for conflict in conflicts:\n        left = train_by_id.get(train_id_from_conflict_name(conflict.train1))\n        right = train_by_id.get(train_id_from_conflict_name(conflict.train2))\n        if left is None or right is None:\n            continue\n        if (\n            left.window_end > disturbance_minute\n            or right.window_end > disturbance_minute\n        ):\n            filtered.append(conflict)\n    return filtered\n\n\ndef iterative_safety_wait_recovery(\n    reference_trains,\n    disturbed_trains,\n    disturbed_assignment,\n    disturbance_minute,\n    options_by_train=None,\n    frozen_train_ids=None,\n    max_rounds=160,\n):\n    """用逐轮安全顺延先恢复出可行时刻表，避免优化器直接处理带硬冲突的扰动时刻。"""\n    frozen_train_ids = set(frozen_train_ids or set())\n    current_by_id = {train.record_id: train for train in disturbed_trains}\n    reference_map = {train.record_id: train for train in reference_trains}\n    assignment = base.rebuild_assignment_for_trains(\n        disturbed_trains, disturbed_assignment\n    )\n    notes = {}\n    last_conflicts = base.collect_conflicts(\n        disturbed_trains, assignment, SCHEME_DISTURBED\n    )\n\n    def causal_conflicts(conflicts):\n        if disturbance_minute is None:\n            return conflicts\n        causal = []\n        for conflict in conflicts:\n            left = current_by_id.get(train_id_from_conflict_name(conflict.train1))\n            right = current_by_id.get(train_id_from_conflict_name(conflict.train2))\n            if left is None or right is None:\n                continue\n            if (\n                left.window_end > disturbance_minute\n                or right.window_end > disturbance_minute\n            ):\n                causal.append(conflict)\n        return causal\n\n    def timetable_objective(candidate_by_id):\n        delays = []\n        for train_id, train in candidate_by_id.items():\n            reference = reference_map.get(train_id)\n            if reference is None:\n                continue\n            delay = max(\n                0,\n                train.arrival_min - reference.arrival_min,\n                train.departure_min - reference.departure_min,\n            )\n            if delay > 0:\n                delays.append(delay)\n        return (sum(delays), max(delays, default=0), len(delays))\n\n    def choose_delay_action(train1_id, train2_id, base_shift):\n        candidates = []\n        for train_id, other_id in ((train1_id, train2_id), (train2_id, train1_id)):\n            train = current_by_id.get(train_id)\n            other = current_by_id.get(other_id)\n            if train is None or other is None:\n                continue\n            if train_id in frozen_train_ids:\n                continue\n            if (\n                disturbance_minute is not None\n                and train.window_end <= disturbance_minute\n            ):\n                continue\n            required_shift = max(1, int(math.ceil(base_shift)))\n            if train.window_start <= other.window_start:\n                required_shift = max(\n                    required_shift, other.window_end - train.window_start + 1\n                )\n            candidate_by_id = dict(current_by_id)\n            candidate_by_id[train_id] = base.rebuild_train_record(\n                candidate_by_id[train_id],\n                arrival_shift=required_shift,\n                departure_shift=required_shift,\n            )\n            candidates.append(\n                (\n                    timetable_objective(candidate_by_id),\n                    current_by_id[train_id].index,\n                    train_id,\n                    required_shift,\n                )\n            )\n        if not candidates:\n            return None\n        _, _, train_id, shift = min(candidates, key=lambda item: (item[0], item[1]))\n        return train_id, shift\n\n    def route_change_allowed(train_id):\n        train = current_by_id.get(train_id)\n        if train is None:\n            return False\n        if train_id in frozen_train_ids:\n            return False\n        if disturbance_minute is not None and train.window_end <= disturbance_minute:\n            return False\n        return options_by_train is not None and train_id in options_by_train\n\n    def assignment_signature(option):\n        return (\n            option.track,\n            option.in_throat,\n            option.out_throat,\n            option.inbound_channel,\n            option.outbound_channel,\n        )\n\n    def try_route_reassignment(current_trains, hard_conflicts):\n        nonlocal assignment\n        if not options_by_train:\n            return False\n        current_hard_count = len(hard_conflicts)\n        if current_hard_count <= 0:\n            return False\n\n        conflict_counts = Counter()\n        for conflict in hard_conflicts:\n            left_id = train_id_from_conflict_name(conflict.train1)\n            right_id = train_id_from_conflict_name(conflict.train2)\n            if route_change_allowed(left_id):\n                conflict_counts[left_id] += 1\n            if route_change_allowed(right_id):\n                conflict_counts[right_id] += 1\n        if not conflict_counts:\n            return False\n\n        best_candidate = None\n        ordered_train_ids = sorted(\n            conflict_counts,\n            key=lambda train_id: (\n                -conflict_counts[train_id],\n                current_by_id[train_id].arrival_min,\n                current_by_id[train_id].index,\n            ),\n        )\n        for train_id in ordered_train_ids[:8]:\n            current_option = assignment.get(train_id)\n            if current_option is None:\n                continue\n            current_signature = assignment_signature(current_option)\n            candidate_options = sorted(\n                options_by_train.get(train_id, []),\n                key=lambda option: (\n                    assignment_signature(option) == current_signature,\n                    option.linear_cost,\n                    option.candidate_rank,\n                    option.option_id,\n                ),\n            )[:6]\n            for option in candidate_options:\n                if assignment_signature(option) == current_signature:\n                    continue\n                candidate_assignment = dict(assignment)\n                candidate_assignment[train_id] = option\n                rebuilt_assignment = base.rebuild_assignment_for_trains(\n                    current_trains, candidate_assignment\n                )\n                candidate_conflicts = causal_conflicts(\n                    base.collect_conflicts(\n                        current_trains, rebuilt_assignment, SCHEME_DISTURBED\n                    )\n                )\n                candidate_hard_conflicts = [\n                    conflict\n                    for conflict in candidate_conflicts\n                    if conflict.conflict_level == "硬冲突"\n                ]\n                candidate_hard_count = len(candidate_hard_conflicts)\n                if candidate_hard_count >= current_hard_count:\n                    continue\n                candidate_score = (\n                    candidate_hard_count,\n                    timetable_objective(current_by_id),\n                    option.linear_cost,\n                    option.candidate_rank,\n                    current_by_id[train_id].index,\n                )\n                if best_candidate is None or candidate_score < best_candidate[0]:\n                    best_candidate = (\n                        candidate_score,\n                        train_id,\n                        current_option,\n                        option,\n                        rebuilt_assignment,\n                        candidate_conflicts,\n                    )\n                    if candidate_hard_count == 0:\n                        break\n            if best_candidate is not None and best_candidate[0][0] == 0:\n                break\n\n        if best_candidate is None:\n            return False\n\n        _, train_id, old_option, new_option, rebuilt_assignment, _ = best_candidate\n        assignment = rebuilt_assignment\n        notes.setdefault(train_id, []).append(\n            f"换股道/进路:{old_option.track}->{new_option.track}"\n        )\n        return True\n\n    for round_index in range(max(1, max_rounds)):\n        current_trains = [\n            current_by_id[train.record_id]\n            for train in sorted(current_by_id.values(), key=lambda item: item.index)\n        ]\n        assignment = base.rebuild_assignment_for_trains(current_trains, assignment)\n        conflicts = causal_conflicts(\n            base.collect_conflicts(current_trains, assignment, SCHEME_DISTURBED)\n        )\n        last_conflicts = conflicts\n        hard_conflicts = [\n            conflict for conflict in conflicts if conflict.conflict_level == "硬冲突"\n        ]\n        if not hard_conflicts:\n            return current_trains, assignment, conflicts, notes, round_index\n\n        if try_route_reassignment(current_trains, hard_conflicts):\n            continue\n\n        shift_by_train = {}\n        for conflict in hard_conflicts:\n            left_id = train_id_from_conflict_name(conflict.train1)\n            right_id = train_id_from_conflict_name(conflict.train2)\n            delay_action = choose_delay_action(\n                left_id, right_id, conflict.overlap_min + 1\n            )\n            if delay_action is None:\n                continue\n            delayed_train_id, shift = delay_action\n            shift_by_train[delayed_train_id] = max(\n                shift_by_train.get(delayed_train_id, 0), shift\n            )\n\n        if not shift_by_train:\n            break\n\n        for train_id, shift in shift_by_train.items():\n            current_by_id[train_id] = base.rebuild_train_record(\n                current_by_id[train_id],\n                arrival_shift=shift,\n                departure_shift=shift,\n            )\n            notes.setdefault(train_id, []).append(f"安全等待+{shift}分")\n\n    final_trains = [\n        current_by_id[train.record_id]\n        for train in sorted(current_by_id.values(), key=lambda item: item.index)\n    ]\n    assignment = base.rebuild_assignment_for_trains(final_trains, assignment)\n    last_conflicts = causal_conflicts(\n        base.collect_conflicts(final_trains, assignment, SCHEME_DISTURBED)\n    )\n    return final_trains, assignment, last_conflicts, notes, max_rounds\n\n\ndef write_csv(path, rows, fieldnames):\n    with Path(path).open("w", encoding="utf-8-sig", newline="") as handle:\n        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")\n        writer.writeheader()\n        writer.writerows(rows)\n\n\ndef write_excel(path, comparison_rows, performance_rows, history_rows):\n    if not EXCEL_AVAILABLE:\n        return\n    wb = openpyxl.Workbook()\n    header_font = Font(bold=True)\n    fill = PatternFill(start_color="DDEBF7", end_color="DDEBF7", fill_type="solid")\n    border = Border(\n        left=Side(style="thin"),\n        right=Side(style="thin"),\n        top=Side(style="thin"),\n        bottom=Side(style="thin"),\n    )\n\n    for sheet_name, rows in [\n        ("方案对比", comparison_rows),\n        ("求解器性能", performance_rows),\n        ("QEA-NS迭代记录", history_rows),\n    ]:\n        ws = wb.active if wb.active.title == "Sheet" else wb.create_sheet(sheet_name)\n        ws.title = sheet_name\n        if not rows:\n            continue\n        headers = list(rows[0])\n        for col, header in enumerate(headers, 1):\n            cell = ws.cell(row=1, column=col, value=header)\n            cell.font = header_font\n            cell.fill = fill\n            cell.border = border\n            cell.alignment = Alignment(horizontal="center", wrap_text=True)\n        for row_index, row in enumerate(rows, 2):\n            for col, header in enumerate(headers, 1):\n                cell = ws.cell(row=row_index, column=col, value=row.get(header, ""))\n                cell.border = border\n                cell.alignment = Alignment(horizontal="center")\n        for col in range(1, len(headers) + 1):\n            ws.column_dimensions[get_column_letter(col)].width = 18\n    wb.save(path)\n\n\ndef write_convergence_plot(path, history_rows):\n    try:\n        import matplotlib.pyplot as plt\n    except Exception:\n        return False\n    qea_rows = [row for row in history_rows if row.get("阶段") == "QEA-NS主循环"]\n    if not qea_rows:\n        return False\n    x = [int(row["迭代轮次"]) for row in qea_rows]\n    hard = [int(row["硬冲突数"]) for row in qea_rows]\n    energy = [float(row["能量值"]) for row in qea_rows]\n    fig, axes = plt.subplots(1, 2, figsize=(11, 4), dpi=150)\n    axes[0].plot(x, hard, color="#1f4e79", linewidth=1.8)\n    axes[0].set_title("QEA-NS hard conflicts")\n    axes[0].set_xlabel("Generation")\n    axes[0].set_ylabel("Hard conflicts")\n    axes[0].grid(alpha=0.25)\n    axes[1].plot(x, energy, color="#9c3d20", linewidth=1.8)\n    axes[1].set_title("QEA-NS energy")\n    axes[1].set_xlabel("Generation")\n    axes[1].set_ylabel("Energy")\n    axes[1].grid(alpha=0.25)\n    fig.tight_layout()\n    fig.savefig(path)\n    plt.close(fig)\n    return True\n\n\ndef main():\n    args = parse_args()\n    base_dir = Path(__file__).resolve().parent\n    input_path = Path(args.input)\n    if not input_path.is_absolute():\n        input_path = base_dir / input_path\n    out_dir = (\n        Path(args.output_dir).expanduser().resolve() if args.output_dir else base_dir\n    )\n    out_dir.mkdir(parents=True, exist_ok=True)\n\n    encoding = base.detect_encoding(input_path)\n    trains = base.load_trains(input_path, encoding)\n    library = base.build_route_library(trains)\n\n    print("=" * 64, flush=True)\n    print(f"{STATION_NAME} QEA-NS vs CP-SAT 扰动恢复实验", flush=True)\n    print("=" * 64, flush=True)\n    print(f"输入文件：{input_path.name}", flush=True)\n    print(f"列车数量：{len(trains)}", flush=True)\n    print(\n        f"建模股道数量：{len(library[\'all_tracks_sorted\'])}（{\',\'.join(library[\'all_tracks_sorted\'])}）",\n        flush=True,\n    )\n\n    pressure_ids = base.build_high_pressure_train_ids(trains, limit=24)\n    plan_options, _ = base.build_candidate_route_plans(\n        trains=trains,\n        library=library,\n        max_route_candidates=args.max_route_candidates,\n        rich_candidate_train_ids=pressure_ids,\n    )\n    plan_assignment = base.build_plan_assignment(trains, plan_options)\n    plan_ids = base.project_assignment_to_option_ids(\n        trains, plan_options, plan_assignment\n    )\n    plan_metrics, _, _ = collect_raw_solution_metrics(\n        SCHEME_PLAN,\n        trains,\n        trains,\n        plan_ids,\n        plan_options,\n    )\n    print(\n        f"原计划复核：总冲突={plan_metrics[\'总冲突项\']}，硬冲突={plan_metrics[\'硬冲突项\']}，总晚点={plan_metrics[\'总晚点时长\']}分",\n        flush=True,\n    )\n\n    scenario, _ = base.generate_disturbance_scenario(trains, args.seed)\n    disturbance_time_text = ";".join(\n        base.format_minutes_as_clock(event.disturbance_minute)\n        for event in scenario.disturbance_events\n    ) or base.format_minutes_as_clock(scenario.disturbance_minute)\n    src_trains = list(scenario.source_train_ids)\n    print(\n        f"扰动时刻：{disturbance_time_text}，"\n        f"事件数={len(scenario.disturbance_events)}，源头列车={src_trains}",\n        flush=True,\n    )\n    disturbed_trains, _, _, propagation_rounds = base.simulate_scenario_for_assignment(\n        reference_trains=trains,\n        assignment=plan_assignment,\n        scenario=scenario,\n        library=library,\n        max_route_candidates=args.max_route_candidates,\n    )\n    scenario = base.scenario_with_propagation_rounds(scenario, propagation_rounds)\n    pre_restore_delay = base.compute_delay_statistics(trains, disturbed_trains)\n    print(\n        f"扰动传播轮数：{propagation_rounds}，扰动后总晚点(还原前)：{pre_restore_delay[\'总晚点时长\']}分，"\n        f"受影响列车={pre_restore_delay.get(\'受影响列车数\', 0)}",\n        flush=True,\n    )\n    # 冻结集合仅包含扰动前已完成作业且不属于扰动源头的列车。\n    raw_frozen = train_ids_fixed_before_disturbance(trains, scenario.disturbance_minute)\n    frozen_train_ids = {\n        train.record_id\n        for train in trains\n        if train.window_end <= scenario.disturbance_minute - 30\n        and train.record_id not in set(src_trains)\n    }\n    print(\n        f"原始冻结候选数：{len(raw_frozen)}，剔除源头后真冻结数：{len(frozen_train_ids)}",\n        flush=True,\n    )\n    disturbed_plan_assignment = base.rebuild_assignment_for_trains(\n        disturbed_trains, plan_assignment\n    )\n    disturbed_trains, disturbed_plan_assignment = restore_pre_disturbance_plan(\n        reference_trains=trains,\n        current_trains=disturbed_trains,\n        current_assignment=disturbed_plan_assignment,\n        plan_assignment=plan_assignment,\n        frozen_train_ids=frozen_train_ids,\n    )\n    disturbed_plan_assignment = base.rebuild_assignment_for_trains(\n        disturbed_trains, plan_assignment\n    )\n\n    # QEA-NS 与 CP-SAT 采用相同的扰动基准。\n    rich_ids = {\n        tid\n        for tid, delay in base.compute_delay_statistics(trains, disturbed_trains)[\n            "delay_by_train"\n        ].items()\n        if delay > 0\n    }\n    (\n        options_by_train,\n        train_map,\n        linear_costs,\n        pair_costs_map,\n        hard_counts,\n        hard_risks,\n    ) = build_problem(\n        disturbed_trains,\n        library,\n        max(args.max_route_candidates, 8),\n        rich_ids=rich_ids,\n        fixed_assignment=disturbed_plan_assignment,\n        frozen_train_ids=frozen_train_ids,\n        disturbance_minute=scenario.disturbance_minute,\n    )\n    baseline_assignment = base.rebuild_assignment_for_trains(\n        disturbed_trains, plan_assignment\n    )\n    baseline_ids = base.project_assignment_to_option_ids(\n        disturbed_trains, options_by_train, baseline_assignment\n    )\n    disturbed_ids = dict(baseline_ids)\n    baseline_metrics, _, _ = collect_raw_solution_metrics(\n        SCHEME_DISTURBED,\n        disturbed_trains,\n        trains,\n        baseline_ids,\n        options_by_train,\n        linear_costs,\n        pair_costs_map,\n        hard_counts,\n        hard_risks,\n    )\n    baseline_landed_metrics, _, _ = collect_solution_metrics(\n        SCHEME_DISTURBED,\n        disturbed_trains,\n        trains,\n        baseline_ids,\n        options_by_train,\n        linear_costs,\n        pair_costs_map,\n        hard_counts,\n        hard_risks,\n        disturbance_minute=scenario.disturbance_minute,\n        frozen_train_ids=frozen_train_ids,\n        max_safety_wait_rounds=args.safety_wait_rounds,\n    )\n    for delay_metric in (\n        "总晚点时长",\n        "平均晚点时长",\n        "最大晚点时长",\n        "顺延调整列车数",\n    ):\n        baseline_metrics[delay_metric] = baseline_landed_metrics[delay_metric]\n    print(\n        f"扰动基准：总冲突={baseline_metrics[\'总冲突项\']}，硬冲突={baseline_metrics[\'硬冲突项\']}，总晚点={baseline_metrics[\'总晚点时长\']}分",\n        flush=True,\n    )\n\n    print("=== QEA-NS 求解 ===", flush=True)\n    qea_ids, qea_history, qea_diag = qea_optimize(\n        trains=disturbed_trains,\n        options_by_train=options_by_train,\n        linear_costs=linear_costs,\n        pair_costs_map=pair_costs_map,\n        hard_counts=hard_counts,\n        hard_risks=hard_risks,\n        train_map=train_map,\n        time_limit_seconds=args.qea_time_limit,\n        **{"cp" + "_sat_time_limit_seconds": args.subproblem_time_limit},\n        seed=args.seed,\n        pop_size=args.qea_pop_size,\n        max_generations=args.qea_max_generations,\n        initial_assignment_ids=disturbed_ids,\n    )\n    qea_metrics, qea_assignment, qea_conflicts = collect_solution_metrics(\n        SCHEME_QEA,\n        disturbed_trains,\n        trains,\n        qea_ids,\n        options_by_train,\n        linear_costs,\n        pair_costs_map,\n        hard_counts,\n        hard_risks,\n        disturbance_minute=scenario.disturbance_minute,\n        frozen_train_ids=frozen_train_ids,\n        max_safety_wait_rounds=args.safety_wait_rounds,\n    )\n    print(\n        f"QEA-NS：硬冲突={qea_metrics[\'硬冲突项\']}，能量={qea_metrics[\'能量值\']:.2f}",\n        flush=True,\n    )\n\n    print("=== CP-SAT 求解 ===", flush=True)\n    cp_sat_start = time.perf_counter()\n    cp_sat_ids, cp_sat_status = solve_subproblem(\n        initial_assignment=disturbed_ids,\n        active_train_ids={train.record_id for train in disturbed_trains},\n        options_by_train=options_by_train,\n        linear_costs=linear_costs,\n        pair_costs_map=pair_costs_map,\n        hard_counts=hard_counts,\n        hard_risks=hard_risks,\n        time_limit_seconds=args.cp_sat_time_limit,\n        strict_hard=True,\n    )\n    cp_sat_time = time.perf_counter() - cp_sat_start\n    cp_sat_solved = cp_sat_ids is not None\n    if cp_sat_solved:\n        cp_sat_metrics, cp_sat_assignment, cp_sat_conflicts = collect_solution_metrics(\n            SCHEME_CP_SAT,\n            disturbed_trains,\n            trains,\n            cp_sat_ids,\n            options_by_train,\n            linear_costs,\n            pair_costs_map,\n            hard_counts,\n            hard_risks,\n            disturbance_minute=scenario.disturbance_minute,\n            frozen_train_ids=frozen_train_ids,\n            max_safety_wait_rounds=args.safety_wait_rounds,\n        )\n        print(\n            f"CP-SAT：状态={cp_sat_status}，硬冲突={cp_sat_metrics[\'硬冲突项\']}，能量={cp_sat_metrics[\'能量值\']:.2f}",\n            flush=True,\n        )\n    else:\n        cp_sat_metrics = {\n            "方法": SCHEME_CP_SAT,\n            "总冲突项": "--",\n            "硬冲突项": "--",\n            "股道占用冲突项": "--",\n            "联锁类冲突项": "--",\n            "总晚点时长": "--",\n            "平均晚点时长": "--",\n            "最大晚点时长": "--",\n            "能量值": "--",\n            "顺延调整列车数": "--",\n        }\n        cp_sat_assignment = {}\n        cp_sat_conflicts = []\n        print(f"CP-SAT：状态={cp_sat_status}，未返回可行解，不采用替代方案", flush=True)\n\n    gap = track_gap(qea_assignment, cp_sat_assignment) if cp_sat_solved else "--"\n    qea_frozen_changes = count_frozen_assignment_changes(\n        qea_assignment, plan_assignment, frozen_train_ids\n    )\n    cp_sat_frozen_changes = (\n        count_frozen_assignment_changes(\n            cp_sat_assignment, plan_assignment, frozen_train_ids\n        )\n        if cp_sat_solved\n        else "--"\n    )\n\n    def display_value(value):\n        return f"{value:.2f}" if isinstance(value, float) else value\n\n    comparison_rows = []\n    for metric in [\n        "总冲突项",\n        "硬冲突项",\n        "股道占用冲突项",\n        "联锁类冲突项",\n        "总晚点时长",\n        "平均晚点时长",\n        "最大晚点时长",\n    ]:\n        plan_val = plan_metrics[metric]\n        bl_val = baseline_metrics[metric]\n        comparison_rows.append(\n            {\n                "指标": metric,\n                "原计划复核": f"{plan_val:.2f}"\n                if isinstance(plan_val, float)\n                else plan_val,\n                "扰动基准": f"{bl_val:.2f}" if isinstance(bl_val, float) else bl_val,\n                "QEA-NS": f"{qea_metrics[metric]:.2f}"\n                if isinstance(qea_metrics[metric], float)\n                else qea_metrics[metric],\n                "CP-SAT": f"{cp_sat_metrics[metric]:.2f}"\n                if isinstance(cp_sat_metrics[metric], float)\n                else cp_sat_metrics[metric],\n            }\n        )\n    comparison_rows.append(\n        {\n            "指标": "能量值",\n            "原计划复核": "--",\n            "扰动基准": display_value(baseline_metrics["能量值"]),\n            "QEA-NS": display_value(qea_metrics["能量值"]),\n            "CP-SAT": display_value(cp_sat_metrics["能量值"]),\n        }\n    )\n    comparison_rows.append(\n        {\n            "指标": "扰动新增冲突项",\n            "原计划复核": "--",\n            "扰动基准": baseline_metrics["总冲突项"] - plan_metrics["总冲突项"],\n            "QEA-NS": "--",\n            "CP-SAT": "--",\n        }\n    )\n    comparison_rows.append(\n        {\n            "指标": "QEA-NS与CP-SAT股道差异列车数",\n            "原计划复核": "--",\n            "扰动基准": "--",\n            "QEA-NS": gap,\n            "CP-SAT": 0 if cp_sat_solved else "--",\n        }\n    )\n    comparison_rows.append(\n        {\n            "指标": "扰动前冻结列车变更数",\n            "原计划复核": 0,\n            "扰动基准": 0,\n            "QEA-NS": qea_frozen_changes,\n            "CP-SAT": cp_sat_frozen_changes,\n        }\n    )\n\n    performance_rows = [\n        {\n            "方法": SCHEME_QEA,\n            "求解时间(s)": f"{qea_diag[\'求解时间(s)\']:.3f}",\n            "状态": "可行" if qea_metrics["硬冲突项"] == 0 else "有冲突",\n            "硬冲突数": qea_metrics["硬冲突项"],\n            "能量值": f"{qea_metrics[\'能量值\']:.2f}",\n            "收敛代数": qea_diag.get("收敛代数", ""),\n            "邻域修复调用次数": qea_diag.get("邻域修复调用次数", ""),\n            "QEA-NS采样次数": qea_diag.get("QEA-NS采样次数", ""),\n            "QEA-NS精英变异次数": qea_diag.get("QEA-NS精英变异次数", ""),\n            "安全顺延调整列车数": qea_metrics.get("顺延调整列车数", 0),\n        },\n        {\n            "方法": SCHEME_CP_SAT,\n            "求解时间(s)": f"{cp_sat_time:.3f}",\n            "状态": cp_sat_status,\n            "硬冲突数": cp_sat_metrics["硬冲突项"],\n            "能量值": display_value(cp_sat_metrics["能量值"]),\n            "收敛代数": "--",\n            "邻域修复调用次数": "--",\n            "QEA-NS采样次数": "--",\n            "QEA-NS精英变异次数": "--",\n            "安全顺延调整列车数": cp_sat_metrics.get("顺延调整列车数", 0),\n        },\n    ]\n\n    history_rows = [\n        {key: value for key, value in {"方法": SCHEME_QEA, **row}.items() if key != "备注"}\n        for row in qea_history\n    ]\n\n    comparison_path = out_dir / "法兰克福中央车站QEA-NS与CP-SAT方案对比详情.csv"\n    performance_path = (\n        out_dir / "法兰克福中央车站QEA-NS与CP-SAT求解器性能对比.csv"\n    )\n    history_path = out_dir / "法兰克福中央车站QEA-NS与CP-SAT求解迭代记录.csv"\n    summary_path = out_dir / "法兰克福中央车站QEA-NS与CP-SAT对比实验结果.csv"\n    excel_path = out_dir / "法兰克福中央车站QEA-NS与CP-SAT对比实验结果.xlsx"\n    plot_path = out_dir / "法兰克福中央车站QEA-NS与CP-SAT收敛曲线.png"\n\n    write_csv(comparison_path, comparison_rows, list(comparison_rows[0]))\n    write_csv(performance_path, performance_rows, list(performance_rows[0]))\n    write_csv(history_path, history_rows, list(history_rows[0]))\n    write_csv(\n        summary_path,\n        [\n            {"项目": "输入文件", "值": input_path.name},\n            {"项目": "列车数量", "值": len(trains)},\n            {"项目": "建模股道数量", "值": len(library["all_tracks_sorted"])},\n            {\n                "项目": "扰动时刻",\n                "值": disturbance_time_text,\n            },\n            {"项目": "扰动前冻结列车数", "值": len(frozen_train_ids)},\n            {"项目": "原计划复核总冲突", "值": plan_metrics["总冲突项"]},\n            {"项目": "原计划复核硬冲突", "值": plan_metrics["硬冲突项"]},\n            {"项目": "扰动基准总冲突", "值": baseline_metrics["总冲突项"]},\n            {"项目": "扰动基准硬冲突", "值": baseline_metrics["硬冲突项"]},\n            {"项目": "扰动基准总晚点", "值": baseline_metrics["总晚点时长"]},\n            {\n                "项目": "扰动新增冲突项",\n                "值": baseline_metrics["总冲突项"] - plan_metrics["总冲突项"],\n            },\n            {"项目": "QEA-NS总冲突", "值": qea_metrics["总冲突项"]},\n            {"项目": "QEA-NS硬冲突", "值": qea_metrics["硬冲突项"]},\n            {"项目": "QEA-NS总晚点", "值": qea_metrics["总晚点时长"]},\n            {"项目": "CP-SAT状态", "值": cp_sat_status},\n            {"项目": "CP-SAT总冲突", "值": cp_sat_metrics["总冲突项"]},\n            {"项目": "CP-SAT硬冲突", "值": cp_sat_metrics["硬冲突项"]},\n            {"项目": "CP-SAT总晚点", "值": cp_sat_metrics["总晚点时长"]},\n            {"项目": "QEA-NS扰动前冻结变更数", "值": qea_frozen_changes},\n            {"项目": "CP-SAT扰动前冻结变更数", "值": cp_sat_frozen_changes},\n            {"项目": "股道差异列车数", "值": gap},\n        ],\n        ["项目", "值"],\n    )\n    write_excel(excel_path, comparison_rows, performance_rows, history_rows)\n    write_convergence_plot(plot_path, qea_history)\n\n    print("=" * 64, flush=True)\n    print("实验完成", flush=True)\n    print(\n        f"原计划复核：总冲突={plan_metrics[\'总冲突项\']}，硬冲突={plan_metrics[\'硬冲突项\']}，总晚点={plan_metrics[\'总晚点时长\']}分钟",\n        flush=True,\n    )\n    print(\n        f"扰动基准：总冲突={baseline_metrics[\'总冲突项\']}，硬冲突={baseline_metrics[\'硬冲突项\']}，总晚点={baseline_metrics[\'总晚点时长\']}分钟",\n        flush=True,\n    )\n    print(\n        f"QEA-NS 硬冲突：{qea_metrics[\'硬冲突项\']}，总晚点：{qea_metrics[\'总晚点时长\']}分钟",\n        flush=True,\n    )\n    print(\n        f"CP-SAT 状态：{cp_sat_status}，硬冲突：{cp_sat_metrics[\'硬冲突项\']}，总晚点：{cp_sat_metrics[\'总晚点时长\']}分钟",\n        flush=True,\n    )\n    print(\n        f"扰动前冻结列车变更数：QEA-NS={qea_frozen_changes}，CP-SAT={cp_sat_frozen_changes}",\n        flush=True,\n    )\n    print(f"股道差异列车数：{gap}", flush=True)\n    print(f"输出：{excel_path.name}", flush=True)\n\n\nif __name__ == "__main__":\n    main()\n'
import importlib.machinery
import types


class _EmbeddedLoader:
    def __init__(self, source: str, filename: str):
        self.source = source
        self.filename = filename

    def create_module(self, spec):
        return types.ModuleType(spec.name)

    def exec_module(self, module):
        module.__file__ = str(Path(__file__).resolve().parent / self.filename)
        module.__dict__.update(
            {
                "_load_embedded_module": _load_embedded_module,
                "EMBEDDED_STATION_RESOURCE_SOURCE": EMBEDDED_STATION_RESOURCE_SOURCE,
                "EMBEDDED_FRANKFURT_STATION_SOURCE": EMBEDDED_FRANKFURT_STATION_SOURCE,
            }
        )
        exec(compile(self.source, module.__file__, "exec"), module.__dict__)


_ORIGINAL_SPEC_FROM_FILE_LOCATION = importlib.util.spec_from_file_location


def _embedded_spec_from_file_location(name, location, *args, **kwargs):
    filename = Path(str(location)).name
    source = None
    if filename == "_embedded_station_resource_model.py":
        source = EMBEDDED_STATION_RESOURCE_SOURCE
    elif filename == "_embedded_frankfurt_hbf_station_model.py":
        source = EMBEDDED_FRANKFURT_STATION_SOURCE
    elif filename == "frankfurt_hbf_experiment_core.py":
        source = EMBEDDED_EXPERIMENT_CORE_SOURCE
    if source is None:
        return _ORIGINAL_SPEC_FROM_FILE_LOCATION(name, location, *args, **kwargs)
    return importlib.machinery.ModuleSpec(
        name, _EmbeddedLoader(source, filename), origin=f"<embedded:{filename}>"
    )


importlib.util.spec_from_file_location = _embedded_spec_from_file_location


def _load_embedded_module(module_name: str, filename: str, source: str):
    spec = importlib.machinery.ModuleSpec(
        module_name, _EmbeddedLoader(source, filename), origin=f"<embedded:{filename}>"
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    loader = spec.loader
    if loader is None:
        raise RuntimeError(f"无法加载嵌入模块：{filename}")
    loader.exec_module(module)
    return module


def load_experiment_module():
    return _load_embedded_module(
        "frankfurt_experiment_base",
        "frankfurt_hbf_experiment_core.py",
        EMBEDDED_EXPERIMENT_CORE_SOURCE,
    )


exp = load_experiment_module()
for required_interface in ("import_cp_model", "solve_subproblem", "qea_optimize"):
    if not hasattr(exp, required_interface):
        raise RuntimeError(f"嵌入实验模块缺少接口：{required_interface}")
base = exp.base


FIELD_AC = "inbound_operation_duration_min"
FIELD_CE = "outbound_operation_duration_min"


def load_trains_three_segment_compatible(csv_path: Path, encoding: str):
    with Path(csv_path).open("r", encoding=encoding, newline="") as handle:
        reader = csv.DictReader(handle)
        rows = list(reader)
        fieldnames = set(reader.fieldnames or [])

    three_segment_fields = {
        base.FIELD_TRAIN_NO,
        base.FIELD_TRIP_ID,
        base.FIELD_DIRECTION,
        base.FIELD_PLANNED_TRACK,
        base.FIELD_PREV_STATION,
        base.FIELD_NEXT_STATION,
        base.FIELD_IN_THROAT,
        base.FIELD_OUT_THROAT,
        base.FIELD_ARRIVAL,
        base.FIELD_DEPARTURE,
        base.FIELD_DWELL,
        FIELD_AC,
        base.FIELD_C,
        FIELD_CE,
    }
    if not {FIELD_AC, FIELD_CE}.issubset(fieldnames):
        raise KeyError(f"输入文件必须包含三段字段：{FIELD_AC}, {FIELD_CE}")
    missing_fields = sorted(three_segment_fields.difference(fieldnames))
    if missing_fields:
        raise KeyError(f"输入文件缺少字段：{', '.join(missing_fields)}")

    trains = []
    input_start, input_end = base.INPUT_TIME_WINDOW
    for row in rows:
        arrival_min = base.parse_clock_to_minutes(row[base.FIELD_ARRIVAL])
        departure_min = base.parse_clock_to_minutes(row[base.FIELD_DEPARTURE])
        dwell_min = base.parse_minutes(row[base.FIELD_DWELL])
        ac_min = base.parse_minutes(row[FIELD_AC])
        c_min = base.parse_minutes(row[base.FIELD_C])
        ce_min = base.parse_minutes(row[FIELD_CE])
        dwell_end = max(departure_min, arrival_min + max(dwell_min, c_min))
        inbound_start = arrival_min - ac_min
        outbound_end = departure_min + ce_min
        if (
            base.overlap_minutes(inbound_start, outbound_end, input_start, input_end)
            <= 0
        ):
            continue

        index = len(trains) + 1
        trains.append(
            base.TrainRecord(
                index=index,
                record_id=f"R{index:03d}",
                train_no=(row[base.FIELD_TRAIN_NO] or "").strip(),
                trip_id=(row[base.FIELD_TRIP_ID] or "").strip(),
                direction=(row[base.FIELD_DIRECTION] or "").strip(),
                planned_track=(row[base.FIELD_PLANNED_TRACK] or "").strip(),
                prev_station=(row[base.FIELD_PREV_STATION] or "").strip(),
                next_station=(row[base.FIELD_NEXT_STATION] or "").strip(),
                in_throat=(row[base.FIELD_IN_THROAT] or "").strip(),
                out_throat=(row[base.FIELD_OUT_THROAT] or "").strip(),
                arrival_text=(row[base.FIELD_ARRIVAL] or "").strip(),
                departure_text=(row[base.FIELD_DEPARTURE] or "").strip(),
                arrival_min=arrival_min,
                departure_min=departure_min,
                dwell_min=dwell_min,
                ab_min=ac_min,
                b_min=0,
                bc_min=0,
                c_min=c_min,
                cd_min=ce_min,
                d_min=0,
                de_min=0,
                inbound_start=inbound_start,
                inbound_end=arrival_min,
                dwell_start=arrival_min,
                dwell_end=dwell_end,
                outbound_start=departure_min,
                outbound_end=outbound_end,
            )
        )

    if not trains:
        raise ValueError("输入数据在 08:00-10:00 时间窗内没有可用列车。")
    return trains


def build_resource_windows_three_segment(
    train,
    option_id,
    route_code,
    track,
    in_throat,
    out_throat,
    inbound_channel,
    outbound_channel,
):
    cache_key = (
        base.train_timing_cache_key(train),
        option_id,
        route_code,
        track,
        in_throat,
        out_throat,
        inbound_channel,
        outbound_channel,
        "3segment",
    )
    cached = base.RESOURCE_WINDOW_CACHE.get(cache_key)
    if cached is not None:
        return cached

    zone = base.classify_track_zone(track)
    in_partition = base.throat_partition(track, in_throat)
    out_partition = base.throat_partition(track, out_throat)
    windows = []

    def add_window(
        stage,
        resource_name,
        resource_category,
        movement_family,
        start_min,
        end_min,
        throat="",
        partition="",
        channel="",
        resource_track="",
    ):
        if end_min <= start_min:
            return
        windows.append(
            base.ResourceWindow(
                train_id=train.record_id,
                option_id=option_id,
                route_code=route_code,
                stage=stage,
                resource_name=resource_name,
                resource_category=resource_category,
                movement_family=movement_family,
                throat=throat,
                partition=partition,
                channel=channel,
                zone=zone,
                track=resource_track,
                start_min=start_min,
                end_min=end_min,
            )
        )

    add_window(
        "A-C",
        f"进站咽喉:{in_throat}:{in_partition or zone}",
        "route_lock",
        "接车",
        train.inbound_start,
        train.inbound_end,
        throat=in_throat,
        partition=in_partition,
        channel=inbound_channel,
        resource_track=track,
    )
    add_window(
        "C",
        f"股道占用:{track}",
        "track",
        "股道",
        train.dwell_start,
        train.dwell_end,
        resource_track=track,
    )
    if train.operation_type in {"始发", "始发终到"}:
        add_window(
            "C",
            f"始发作业:{track}",
            "terminal_op",
            "始发",
            train.dwell_start,
            train.dwell_end,
            resource_track=track,
        )
    if train.operation_type in {"终到", "始发终到"}:
        add_window(
            "C",
            f"终到作业:{track}",
            "terminal_op",
            "终到",
            train.dwell_start,
            train.dwell_end,
            resource_track=track,
        )
    add_window(
        "C-E",
        f"出站咽喉:{out_throat}:{out_partition or zone}",
        "route_lock",
        "发车",
        train.outbound_start,
        train.outbound_end,
        throat=out_throat,
        partition=out_partition,
        channel=outbound_channel,
        resource_track=track,
    )

    result = tuple(windows)
    if len(base.RESOURCE_WINDOW_CACHE) < base.RESOURCE_WINDOW_CACHE_LIMIT:
        base.RESOURCE_WINDOW_CACHE[cache_key] = result
    return result


base.load_trains = load_trains_three_segment_compatible
base.build_resource_windows = build_resource_windows_three_segment
base.CONFLICT_PENALTY.update(
    {
        MACRO_INBOUND_CONFLICT: (1500.0, 60.0),
        MACRO_TRACK_CONFLICT: base.CONFLICT_PENALTY.get(
            "股道占用冲突", (4200.0, 160.0)
        ),
        MACRO_OUTBOUND_CONFLICT: (1500.0, 60.0),
    }
)
if hasattr(base, "_MODULE"):
    base._MODULE.load_trains = load_trains_three_segment_compatible
    base._MODULE.build_resource_windows = build_resource_windows_three_segment
    base._MODULE.CONFLICT_PENALTY.update(base.CONFLICT_PENALTY)
for cache_name in (
    "RESOURCE_WINDOW_CACHE",
    "PAIR_CONFLICT_DETAIL_CACHE",
    "PAIR_CONFLICT_DETAIL_COARSE_CACHE",
):
    cache = getattr(base, cache_name, None)
    if cache is not None:
        cache.clear()
    inner_cache = getattr(getattr(base, "_MODULE", None), cache_name, None)
    if inner_cache is not None:
        inner_cache.clear()


def classify_window_conflict_macro(window_a, window_b):
    overlap = base.overlap_minutes(
        window_a.start_min, window_a.end_min, window_b.start_min, window_b.end_min
    )
    if overlap <= 0:
        return None

    if window_a.stage == "C" and window_b.stage == "C":
        if window_a.track and window_a.track == window_b.track:
            return base.PairConflictDetail(
                "硬冲突",
                MACRO_TRACK_CONFLICT,
                f"股道 {window_a.track}",
                overlap,
                "同一股道宏观占用时段发生重叠。",
            )
        return None

    if window_a.stage == "A-C" and window_b.stage == "A-C":
        if window_a.throat and window_a.throat == window_b.throat:
            relation = base.throat_partition_relation(
                window_a.throat, window_a.track, window_b.track
            )
            if relation:
                partition_a, partition_b, _ = relation
                label = (
                    partition_a
                    if partition_a == partition_b
                    else f"{partition_a}/{partition_b}"
                )
                return base.PairConflictDetail(
                    "硬冲突",
                    MACRO_INBOUND_CONFLICT,
                    f"{window_a.throat}-{label}",
                    overlap,
                    "进站咽喉宏观占用时段发生重叠。",
                )
        return None

    if window_a.stage == "C-E" and window_b.stage == "C-E":
        if window_a.throat and window_a.throat == window_b.throat:
            relation = base.throat_partition_relation(
                window_a.throat, window_a.track, window_b.track
            )
            if relation:
                partition_a, partition_b, _ = relation
                label = (
                    partition_a
                    if partition_a == partition_b
                    else f"{partition_a}/{partition_b}"
                )
                return base.PairConflictDetail(
                    "硬冲突",
                    MACRO_OUTBOUND_CONFLICT,
                    f"{window_a.throat}-{label}",
                    overlap,
                    "出站咽喉宏观占用时段发生重叠。",
                )
        return None
    return None


def collect_pair_conflict_details_macro(option_a, option_b):
    cache_key = base.option_pair_cache_key(option_a, option_b) + ("macro",)
    cached = base.PAIR_CONFLICT_DETAIL_CACHE.get(cache_key)
    if cached is not None:
        return list(cached)
    aggregate = {}
    for window_a in option_a.resources:
        for window_b in option_b.resources:
            detail = classify_window_conflict_macro(window_a, window_b)
            if detail is None:
                continue
            key = (detail.conflict_type, detail.resource_name)
            old_detail = aggregate.get(key)
            if old_detail is None or detail.overlap_min > old_detail.overlap_min:
                aggregate[key] = detail
    result = tuple(aggregate.values())
    if len(base.PAIR_CONFLICT_DETAIL_CACHE) < base.PAIR_CONFLICT_CACHE_LIMIT:
        base.PAIR_CONFLICT_DETAIL_CACHE[cache_key] = result
    return list(result)


def build_conflict_statistics_macro(conflicts):
    summary = base.summarize_conflicts(conflicts)
    hard_conflicts = [
        conflict for conflict in conflicts if conflict.conflict_level == "硬冲突"
    ]
    soft_conflicts = [
        conflict for conflict in conflicts if conflict.conflict_level == "软冲突"
    ]

    def count(conflict_type):
        return summary.get(conflict_type, {"数量": 0, "重叠分钟合计": 0})["数量"]

    def minutes(conflict_type):
        return summary.get(conflict_type, {"数量": 0, "重叠分钟合计": 0})[
            "重叠分钟合计"
        ]

    inbound_count = count(MACRO_INBOUND_CONFLICT)
    outbound_count = count(MACRO_OUTBOUND_CONFLICT)
    return {
        "总冲突项": len(conflicts),
        "总重叠分钟": sum(conflict.overlap_min for conflict in conflicts),
        "硬冲突项": len(hard_conflicts),
        "硬冲突重叠分钟": sum(conflict.overlap_min for conflict in hard_conflicts),
        "软冲突项": len(soft_conflicts),
        "软冲突重叠分钟": sum(conflict.overlap_min for conflict in soft_conflicts),
        "进站咽喉冲突项": inbound_count,
        "进站咽喉重叠分钟": minutes(MACRO_INBOUND_CONFLICT),
        "股道占用冲突项": count(MACRO_TRACK_CONFLICT),
        "股道占用重叠分钟": minutes(MACRO_TRACK_CONFLICT),
        "出站咽喉冲突项": outbound_count,
        "出站咽喉重叠分钟": minutes(MACRO_OUTBOUND_CONFLICT),
        "联锁类冲突项": inbound_count + outbound_count,
        "联锁类重叠分钟": minutes(MACRO_INBOUND_CONFLICT)
        + minutes(MACRO_OUTBOUND_CONFLICT),
        "始发终到作业冲突项": 0,
        "始发终到作业重叠分钟": 0,
        "接车进路锁闭冲突项": 0,
        "发车进路锁闭冲突项": 0,
        "道岔组冲突项": 0,
        "进路交叉冲突项": 0,
        "防护带冲突项": 0,
        "咽喉能力冲突项": 0,
    }


def build_interlocking_subtype_counts_macro(conflicts):
    return {
        "进站咽喉冲突项": sum(
            1
            for conflict in conflicts
            if conflict.conflict_type == MACRO_INBOUND_CONFLICT
        ),
        "出站咽喉冲突项": sum(
            1
            for conflict in conflicts
            if conflict.conflict_type == MACRO_OUTBOUND_CONFLICT
        ),
    }


base.classify_window_conflict = classify_window_conflict_macro
base.classify_window_conflict_coarse = classify_window_conflict_macro
base.collect_pair_conflict_details = collect_pair_conflict_details_macro
base.collect_pair_conflict_details_coarse = collect_pair_conflict_details_macro
base.build_conflict_statistics = build_conflict_statistics_macro
if hasattr(base, "_MODULE"):
    base._MODULE.classify_window_conflict = classify_window_conflict_macro
    base._MODULE.classify_window_conflict_coarse = classify_window_conflict_macro
    base._MODULE.collect_pair_conflict_details = collect_pair_conflict_details_macro
    base._MODULE.collect_pair_conflict_details_coarse = (
        collect_pair_conflict_details_macro
    )
    base._MODULE.build_conflict_statistics = build_conflict_statistics_macro
setattr(
    exp,
    "INTERLOCKING_SUBTYPE_METRICS",
    {
        "进站咽喉冲突项": MACRO_INBOUND_CONFLICT,
        "出站咽喉冲突项": MACRO_OUTBOUND_CONFLICT,
    },
)
setattr(
    exp, "build_interlocking_subtype_counts", build_interlocking_subtype_counts_macro
)


def apply_local_disturbance_config():
    base.DISTURBANCE_TIME_MAX_COUNT = LOCAL_DISTURBANCE_EVENTS_PER_SCENARIO[1]
    base.DISTURBANCE_SOURCE_TRAIN_RANGE = LOCAL_SOURCE_TRAIN_COUNT_RANGE
    base.DISTURBANCE_TRACK_BLOCK_RANGE = LOCAL_TRACK_BLOCK_RANGE
    base.DISTURBANCE_PROPAGATION_MAX_ROUNDS = LOCAL_PROPAGATION_MAX_ROUNDS
    base.WEIBULL_DELAY_SCALE = LOCAL_WEIBULL_DELAY_SCALE
    base.WEIBULL_DELAY_CAP = LOCAL_WEIBULL_DELAY_CAP
    base.MARKOV_DEVICE_MAX_DURATION = LOCAL_MARKOV_DEVICE_MAX_DURATION
    base.RECOVERY_LONG_WAIT_THRESHOLD = LOCAL_RECOVERY_LONG_WAIT_THRESHOLD

    original_generate_disturbance_scenario = base.generate_disturbance_scenario

    def local_select_disturbance_load_rows(load_profile, rng):
        if not load_profile:
            raise ValueError("扰动负荷序列为空，无法抽取扰动时刻。")
        sorted_rows = sorted(
            load_profile,
            key=lambda row: (
                -float(row.get("NHPP扰动概率", 0.0)),
                -float(row.get("负荷指数", 0.0)),
                int(round(row["分钟"])),
            ),
        )
        min_event_count, max_event_count = LOCAL_DISTURBANCE_EVENTS_PER_SCENARIO
        candidate_count = base.contribution_limited_count(
            [
                float(row.get("NHPP扰动概率", 0.0))
                + 0.01 * float(row.get("负荷指数", 0.0))
                for row in sorted_rows
            ],
            base.DISTURBANCE_TIME_CONTRIBUTION_THRESHOLD,
            max_event_count,
            min_event_count,
        )
        selected = (
            sorted_rows[:candidate_count]
            if candidate_count > 0
            else [base.select_disturbance_minute(load_profile, rng)]
        )
        unique_by_minute = {}
        for row in selected:
            unique_by_minute.setdefault(int(round(row["分钟"])), row)
        return tuple(
            sorted(unique_by_minute.values(), key=lambda row: int(round(row["分钟"])))
        )

    def limit_event_sources(events, seed=None):
        if not events:
            return events
        if SOURCE_TRAIN_COUNT_OVERRIDE is None:
            min_count, max_count = LOCAL_SOURCE_TRAIN_COUNT_RANGE
            use_full_score_pool = False
        else:
            min_count = max_count = int(SOURCE_TRAIN_COUNT_OVERRIDE)
            use_full_score_pool = True
        delay_rng = random.Random((seed or 0) + 20260627)
        used_train_ids = set()
        limited_events = []
        for event in events:
            original_ids = list(event.source_train_ids)
            scored_ids = sorted(
                event.source_train_scores,
                key=lambda train_id: (
                    -float(event.source_train_scores.get(train_id, 0.0)),
                    -int(event.source_train_delays.get(train_id, 0)),
                    train_id,
                ),
            )
            candidate_ids = scored_ids if use_full_score_pool else [
                *original_ids,
                *(train_id for train_id in scored_ids if train_id not in original_ids),
            ]
            target_count = min(
                max_count,
                max(min_count, len(original_ids)),
            )
            filtered_source_ids = tuple(
                train_id
                for train_id in candidate_ids
                if train_id not in used_train_ids
            )[:target_count]
            used_train_ids.update(filtered_source_ids)
            filtered_delays = {}
            for train_id in filtered_source_ids:
                if train_id in event.source_train_delays:
                    filtered_delays[train_id] = event.source_train_delays[train_id]
                else:
                    filtered_delays[train_id] = base.sample_source_delay_minutes(
                        delay_rng
                    )
            limited_events.append(
                base.DisturbanceEvent(
                    disturbance_minute=event.disturbance_minute,
                    selected_train_load=event.selected_train_load,
                    selected_switch_load=event.selected_switch_load,
                    selected_load_score=event.selected_load_score,
                    source_train_ids=filtered_source_ids,
                    source_train_delays=filtered_delays,
                    source_train_scores={
                        train_id: event.source_train_scores[train_id]
                        for train_id in filtered_source_ids
                        if train_id in event.source_train_scores
                    },
                    spatial_targets=event.spatial_targets,
                )
            )
        return tuple(limited_events)

    def build_unique_spatial_targets(events, scenario, seed=None):
        """Preserve event target counts without reusing resources."""
        used_resource_keys = set()
        target_rng = random.Random((seed or 0) + 20260628)
        resource_pool = [
            ("track", key, float(score))
            for key, score in (scenario.track_scores or {}).items()
        ] + [
            ("partition", key, float(score))
            for key, score in (scenario.partition_scores or {}).items()
        ]
        resource_pool.sort(key=lambda item: (-item[2], item[0], item[1]))

        def make_target(resource_type, resource_key, score):
            duration = base.sample_markov_device_duration(
                target_rng, score, resource_type
            )
            if resource_type == "track":
                return base.DisturbanceTarget(
                    resource_type="track",
                    resource_key=resource_key,
                    display_name=f"股道{resource_key}",
                    throat="",
                    partition="",
                    tension_score=score,
                    intensity_value=float(duration),
                    intensity_unit="分钟",
                    note="股道临时封锁时长由马尔可夫状态转移生成。",
                    duration_min=duration,
                )
            throat, partition = resource_key.split("-", 1)
            min_ratio, max_ratio = base.DISTURBANCE_SWITCH_SLOWDOWN_RANGE
            slowdown_ratio = min_ratio + (max_ratio - min_ratio) * score
            return base.DisturbanceTarget(
                resource_type="partition",
                resource_key=resource_key,
                display_name=f"{throat}-{partition}",
                throat=throat,
                partition=partition,
                tension_score=score,
                intensity_value=slowdown_ratio,
                intensity_unit="比例",
                note=f"道岔组/分区退化持续{duration}分钟，由马尔可夫状态转移生成。",
                duration_min=duration,
            )

        unique_events = []
        for event in events:
            selected_targets = []
            for target in event.spatial_targets:
                key = (target.resource_type, target.resource_key)
                if key in used_resource_keys:
                    continue
                selected_targets.append(target)
                used_resource_keys.add(key)

            target_count = len(event.spatial_targets)
            for resource_type, resource_key, score in resource_pool:
                if len(selected_targets) >= target_count:
                    break
                key = (resource_type, resource_key)
                if key in used_resource_keys:
                    continue
                selected_targets.append(make_target(resource_type, resource_key, score))
                used_resource_keys.add(key)

            unique_events.append(
                base.DisturbanceEvent(
                    disturbance_minute=event.disturbance_minute,
                    selected_train_load=event.selected_train_load,
                    selected_switch_load=event.selected_switch_load,
                    selected_load_score=event.selected_load_score,
                    source_train_ids=event.source_train_ids,
                    source_train_delays=event.source_train_delays,
                    source_train_scores=event.source_train_scores,
                    spatial_targets=tuple(selected_targets),
                )
            )
        return tuple(unique_events)

    def local_generate_disturbance_scenario(trains, seed):
        scenario, load_profile = original_generate_disturbance_scenario(trains, seed)
        limited_events = limit_event_sources(scenario.disturbance_events, seed)
        limited_events = build_unique_spatial_targets(
            limited_events, scenario, seed
        )
        if not limited_events:
            return scenario, load_profile
        primary_event = max(limited_events, key=lambda event: event.selected_load_score)
        merged_source_ids = tuple(
            dict.fromkeys(
                train_id
                for event in limited_events
                for train_id in event.source_train_ids
            )
        )
        merged_delays = {}
        merged_scores = {}
        merged_targets = tuple(
            target for event in limited_events for target in event.spatial_targets
        )
        for event in limited_events:
            for train_id, delay in event.source_train_delays.items():
                merged_delays[train_id] = max(merged_delays.get(train_id, 0), delay)
            for train_id, score in event.source_train_scores.items():
                merged_scores[train_id] = max(merged_scores.get(train_id, 0.0), score)
        return (
            base.DisturbanceScenario(
                window_start_min=scenario.window_start_min,
                window_end_min=scenario.window_end_min,
                disturbance_minute=primary_event.disturbance_minute,
                selected_train_load=primary_event.selected_train_load,
                selected_switch_load=primary_event.selected_switch_load,
                selected_load_score=primary_event.selected_load_score,
                source_train_ids=merged_source_ids,
                source_train_delays=merged_delays,
                source_train_scores=merged_scores,
                spatial_targets=merged_targets,
                track_scores=scenario.track_scores,
                partition_scores=scenario.partition_scores,
                propagation_rounds=scenario.propagation_rounds,
                disturbance_events=limited_events,
            ),
            load_profile,
        )

    base.select_disturbance_load_rows = local_select_disturbance_load_rows
    base.generate_disturbance_scenario = local_generate_disturbance_scenario


apply_local_disturbance_config()


def set_source_train_count_override(count):
    global SOURCE_TRAIN_COUNT_OVERRIDE
    SOURCE_TRAIN_COUNT_OVERRIDE = int(count)


def expand_options_with_delay_choices(
    trains,
    options_by_train,
    active_train_ids,
    delay_choices=DELAY_CHOICES,
):
    train_map = {train.record_id: train for train in trains}
    active_train_ids = set(active_train_ids or set())
    expanded = {}
    for train_id, options in options_by_train.items():
        train = train_map.get(train_id)
        if train is None or train_id not in active_train_ids:
            expanded[train_id] = options
            continue
        new_options = []
        for option in options:
            for delay in delay_choices:
                if int(delay) == 0:
                    new_options.append(option)
                    continue
                delayed_train = base.rebuild_train_record(
                    train,
                    arrival_shift=int(delay),
                    departure_shift=int(delay),
                )
                option_id = f"{option.option_id}_d{int(delay):02d}"
                resources = base.build_resource_windows(
                    train=delayed_train,
                    option_id=option_id,
                    route_code=option.route_code,
                    track=option.track,
                    in_throat=option.in_throat,
                    out_throat=option.out_throat,
                    inbound_channel=option.inbound_channel,
                    outbound_channel=option.outbound_channel,
                )
                delay_cost = int(delay) * 18.0
                new_options.append(
                    exp.replace(
                        option,
                        option_id=option_id,
                        route_code=f"{option.route_code}-D{int(delay)}",
                        resources=resources,
                        linear_cost=option.linear_cost + delay_cost,
                        delay_risk_cost=option.delay_risk_cost + int(delay),
                        candidate_rank=option.candidate_rank * 100 + int(delay),
                        note=f"{option.note};delay={int(delay)}",
                    )
                )
        expanded[train_id] = new_options
    return expanded


def option_matches_planned_throats(option, planned_throats):
    planned_in, planned_out = planned_throats
    in_throat = str(getattr(option, "in_throat", "") or "").strip()
    out_throat = str(getattr(option, "out_throat", "") or "").strip()
    return in_throat == planned_in and out_throat == planned_out


def filter_planned_throat_options(
    options_by_train, planned_throat_by_train, selected_ids=None
):
    selected_ids = selected_ids or {}
    filtered = {}
    removed_count = 0
    fallback_count = 0
    for train_id, options in options_by_train.items():
        planned_throats = planned_throat_by_train.get(train_id)
        if planned_throats is None:
            filtered[train_id] = options
            continue
        valid_options = [
            option
            for option in options
            if option_matches_planned_throats(option, planned_throats)
        ]
        removed_count += len(options) - len(valid_options)
        if not valid_options:
            selected_id = selected_ids.get(train_id)
            selected_option = next(
                (option for option in options if option.option_id == selected_id), None
            )
            if selected_option is not None and option_matches_planned_throats(
                selected_option, planned_throats
            ):
                valid_options = [selected_option]
            else:
                raise ValueError(f"列车 {train_id} 无满足计划咽喉一致性的候选方案")
            fallback_count += 1
        filtered[train_id] = valid_options
    return filtered, removed_count, fallback_count


def limit_options_for_pair_maps(
    options_by_train, limit=MAX_OPTIONS_PER_TRAIN, keep_ids=None
):
    keep_ids = keep_ids or {}
    delay_order = {delay: index for index, delay in enumerate(DELAY_CHOICES)}

    def option_sort_key(option):
        delay = selected_option_delay_minutes(option.option_id)
        base_rank = option.candidate_rank // 100 if delay > 0 else option.candidate_rank
        return (
            base_rank,
            delay_order.get(delay, len(delay_order)),
            option.linear_cost,
            option.option_id,
        )

    limited = {}
    for train_id, options in options_by_train.items():
        sorted_options = sorted(options, key=option_sort_key)
        selected_options = sorted_options[:limit]
        keep_id = keep_ids.get(train_id)
        if keep_id and all(option.option_id != keep_id for option in selected_options):
            keep_option = next(
                (option for option in sorted_options if option.option_id == keep_id),
                None,
            )
            if keep_option is not None:
                if len(selected_options) >= limit:
                    selected_options = [keep_option, *selected_options[: limit - 1]]
                else:
                    selected_options = [keep_option, *selected_options]
        limited[train_id] = selected_options
    return limited


def trains_for_delay_pair_screen(trains, active_train_ids, max_delay):
    active_train_ids = set(active_train_ids or set())
    if max_delay <= 0:
        return trains
    return [
        base.rebuild_train_record(train, departure_shift=max_delay)
        if train.record_id in active_train_ids
        else train
        for train in trains
    ]


def expand_active_train_ids(
    reference_trains,
    disturbed_trains,
    baseline_assignment,
    source_train_ids,
    disturbance_minute,
    target_size=ACTIVE_TARGET_SIZE,
):
    train_by_id = {train.record_id: train for train in disturbed_trains}
    valid_ids = set(train_by_id)
    delay_stats = base.compute_delay_statistics(reference_trains, disturbed_trains)
    seed_ids = set(source_train_ids or set()) & valid_ids
    delayed_ids = {
        train_id
        for train_id, delay in delay_stats.get("delay_by_train", {}).items()
        if delay > 0 and train_id in valid_ids
    }
    active_ids = set(seed_ids) | delayed_ids

    conflicts = exp.filter_conflicts_after_disturbance(
        base.collect_conflicts(disturbed_trains, baseline_assignment, "active-expand"),
        disturbed_trains,
        disturbance_minute,
    )
    hard_graph = {train_id: set() for train_id in valid_ids}
    hard_degree = {train_id: 0 for train_id in valid_ids}
    for conflict in conflicts:
        if conflict.conflict_level != "硬冲突":
            continue
        left_id = exp.train_id_from_conflict_name(conflict.train1)
        right_id = exp.train_id_from_conflict_name(conflict.train2)
        if left_id not in valid_ids or right_id not in valid_ids:
            continue
        hard_graph[left_id].add(right_id)
        hard_graph[right_id].add(left_id)
        hard_degree[left_id] += 1
        hard_degree[right_id] += 1

    queue = deque((train_id, 0) for train_id in sorted(active_ids))
    seen = set(active_ids)
    while queue:
        train_id, depth = queue.popleft()
        if depth >= 2:
            continue
        for neighbor_id in sorted(hard_graph.get(train_id, ())):
            if neighbor_id in seen:
                continue
            seen.add(neighbor_id)
            active_ids.add(neighbor_id)
            queue.append((neighbor_id, depth + 1))

    overlap_start = disturbance_minute - 60 if disturbance_minute is not None else None
    overlap_end = disturbance_minute + 90 if disturbance_minute is not None else None
    if overlap_start is not None and overlap_end is not None:
        source_windows = [
            train_by_id[train_id] for train_id in seed_ids if train_id in train_by_id
        ]
        for train in disturbed_trains:
            if train.record_id in active_ids:
                continue
            in_window = (
                train.window_end >= overlap_start and train.window_start <= overlap_end
            )
            overlaps_source = any(
                base.overlap_minutes(
                    train.window_start,
                    train.window_end,
                    source.window_start - max(DELAY_CHOICES),
                    source.window_end + max(DELAY_CHOICES),
                )
                > 0
                for source in source_windows
            )
            if in_window and (
                overlaps_source or hard_degree.get(train.record_id, 0) > 0
            ):
                active_ids.add(train.record_id)

    def active_priority(train_id):
        train = train_by_id[train_id]
        source_rank = 0 if train_id in seed_ids else 1
        delayed_rank = 0 if train_id in delayed_ids else 1
        distance = (
            abs(train.arrival_min - disturbance_minute)
            if disturbance_minute is not None
            else train.arrival_min
        )
        return (
            source_rank,
            delayed_rank,
            -hard_degree.get(train_id, 0),
            distance,
            train.index,
        )

    if target_size and len(active_ids) < target_size:
        fill_candidates = [
            train.record_id
            for train in disturbed_trains
            if train.record_id not in active_ids
            and (
                disturbance_minute is None
                or train.window_end >= disturbance_minute - 60
                and train.window_start <= disturbance_minute + 90
            )
        ]
        for train_id in sorted(fill_candidates, key=active_priority):
            active_ids.add(train_id)
            if len(active_ids) >= target_size:
                break

    ordered_active = sorted(active_ids & valid_ids, key=active_priority)
    if target_size and len(ordered_active) > target_size:
        active_ids = set(ordered_active[:target_size])
    else:
        active_ids = set(ordered_active)
    return active_ids, {
        "seed_count": len(seed_ids),
        "delayed_count": len(delayed_ids),
        "hard_neighbor_count": sum(
            1 for train_id in active_ids if hard_degree.get(train_id, 0) > 0
        ),
        "target_size": target_size,
    }


def resolve_input_path(input_arg, base_dir):
    raw_path = Path(input_arg or DEFAULT_INPUT_NAME).expanduser()
    tried = []
    if raw_path.is_absolute():
        if raw_path.exists():
            return raw_path.resolve()
        tried.append(raw_path)
    else:
        direct_path = (base_dir / raw_path).resolve()
        if direct_path.exists():
            return direct_path
        tried.append(direct_path)

    search_names = [raw_path.name, DEFAULT_INPUT_NAME]
    search_patterns = []
    for name in search_names:
        if name and name not in search_patterns:
            search_patterns.append(name)
    search_patterns.extend(
        [
            "frankfurt_hbf_gtfs_schedule.csv",
            "*gtfs_schedule*.csv",
            "*schedule*.csv",
        ]
    )
    search_dirs = [base_dir, base_dir / "新建文件夹"]
    for search_dir in search_dirs:
        if not search_dir.exists():
            continue
        for pattern in search_patterns:
            matches = sorted(
                (path for path in search_dir.glob(pattern) if path.is_file()),
                key=lambda path: (-path.stat().st_size, str(path)),
            )
            if matches:
                return matches[0].resolve()

    for pattern in search_patterns:
        matches = sorted(
            (path for path in base_dir.glob(f"**/{pattern}") if path.is_file()),
            key=lambda path: (-path.stat().st_size, str(path)),
        )
        if matches:
            return matches[0].resolve()

    tried_text = "；".join(str(path) for path in tried)
    raise FileNotFoundError(f"未找到输入文件：{raw_path}；已尝试：{tried_text}")


def selected_option_delay_minutes(option_id):
    text = str(option_id or "")
    if "_d" not in text:
        return 0
    suffix = text.rsplit("_d", 1)[1]
    return int(suffix) if suffix.isdigit() else 0


def selected_delay_summary(assignment_ids):
    delays = [
        selected_option_delay_minutes(option_id)
        for option_id in (assignment_ids or {}).values()
    ]
    positive = [delay for delay in delays if delay > 0]
    return len(positive), sum(positive), max(positive, default=0)


def build_assignment_diagnostic_rows(
    trains,
    reference_trains,
    options_by_train,
    qea_ids,
    cp_sat_ids,
):
    option_lookup = option_lookup_by_id(options_by_train)
    reference_delay = base.compute_delay_statistics(reference_trains, trains)[
        "delay_by_train"
    ]

    def option_fields(assignment_ids, train_id, prefix):
        option_id = (assignment_ids or {}).get(train_id, "")
        option = option_lookup.get(option_id)
        return {
            f"{prefix}候选方案": option_id,
            f"{prefix}股道": option.track if option else "",
            f"{prefix}进路": option.route_code if option else "",
            f"{prefix}接车咽喉": option.in_throat if option else "",
            f"{prefix}发车咽喉": option.out_throat if option else "",
            f"{prefix}进站咽喉分区(in_partition)": option.in_partition if option else "",
            f"{prefix}出站咽喉分区(out_partition)": option.out_partition if option else "",
            f"{prefix}接车通道(inbound_channel)": option.inbound_channel if option else "",
            f"{prefix}发车通道(outbound_channel)": option.outbound_channel if option else "",
            f"{prefix}内生顺延(min)": selected_option_delay_minutes(option_id),
        }

    rows = []
    for train in sorted(trains, key=lambda item: (item.arrival_min, item.index)):
        train_id = train.record_id
        qea_option = (qea_ids or {}).get(train_id, "")
        cp_sat_option = (cp_sat_ids or {}).get(train_id, "")
        qea_fields = option_fields(qea_ids, train_id, "QEA-NS")
        cp_sat_fields = option_fields(cp_sat_ids, train_id, "CP-SAT")
        rows.append(
            {
                "列车ID": train_id,
                "到达时刻": base.format_minutes_as_clock(train.arrival_min),
                "出发时刻": base.format_minutes_as_clock(train.departure_min),
                "计划接车咽喉": train.in_throat,
                "计划发车咽喉": train.out_throat,
                "扰动继承晚点": reference_delay.get(train_id, 0),
                **qea_fields,
                **cp_sat_fields,
                "QEA-NS与CP-SAT候选方案是否不同": int(qea_option != cp_sat_option),
                "QEA-NS相对CP-SAT顺延差值(min)": qea_fields["QEA-NS内生顺延(min)"]
                - cp_sat_fields["CP-SAT内生顺延(min)"],
            }
        )
    return rows


def metrics_prefer_key(metrics):
    return (
        metrics.get("硬冲突项", 10**9),
        metrics.get("总晚点时长", 10**9),
        metrics.get("最大晚点时长", 10**9),
        metrics.get("总冲突项", 10**9),
    )


def qea_assignment_objective(
    selected,
    trains,
    reference_trains,
    options_by_train,
    linear_costs,
    pair_costs_map,
):
    adjusted_trains = apply_selected_option_delays(trains, selected)
    assignment = base.build_assignment_lookup(selected, options_by_train)
    assignment = base.rebuild_assignment_for_trains(adjusted_trains, assignment)
    conflicts = base.collect_conflicts(adjusted_trains, assignment, SCHEME_QEA)
    conflict_stats = base.build_conflict_statistics(conflicts)
    delay_stats = base.compute_delay_statistics(reference_trains, adjusted_trains)
    display_energy = exp.assignment_display_energy(
        selected, linear_costs, pair_costs_map
    )
    objective = (
        conflict_stats["硬冲突项"],
        delay_stats["总晚点时长"],
        delay_stats["最大晚点时长"],
        conflict_stats["总冲突项"],
        display_energy,
    )
    return objective, conflict_stats, delay_stats, display_energy


def qea_landing_conflict_counts(selected, trains, options_by_train):
    adjusted_trains = apply_selected_option_delays(trains, selected)
    assignment = base.build_assignment_lookup(selected, options_by_train)
    assignment = base.rebuild_assignment_for_trains(adjusted_trains, assignment)
    conflicts = base.collect_conflicts(adjusted_trains, assignment, SCHEME_QEA)
    conflict_stats = base.build_conflict_statistics(conflicts)
    return conflict_stats["硬冲突项"], conflict_stats["总冲突项"]


def qea_fast_objective(
    selected,
    train_order,
    reference_delay_by_train,
    options_by_train,
    reference_ids,
    linear_costs,
    pair_costs_map,
    hard_counts,
    trains=None,
):
    hard_total = 0
    pair_cost_total = 0.0
    for left_index, left_id in enumerate(train_order):
        left_option = selected[left_id]
        for right_id in train_order[left_index + 1 :]:
            right_option = selected[right_id]
            hard_total += pair_hard_count(hard_counts, left_option, right_option)
            pair_cost_total += pair_cost_value(
                pair_costs_map, left_option, right_option
            )
    delays = [
        reference_delay_by_train.get(train_id, 0)
        + selected_option_delay_minutes(option_id)
        for train_id, option_id in selected.items()
    ]
    positive_delays = [delay for delay in delays if delay > 0]
    total_delay = sum(positive_delays)
    max_delay = max(positive_delays, default=0)
    _, track_changes, adjusted_trains = selected_adjustment_components(
        selected, options_by_train, reference_ids
    )
    linear_total = sum(
        linear_costs.get(option_id, 0.0) for option_id in selected.values()
    )
    display_energy = linear_total + pair_cost_total
    landing_hard_total = 0
    landing_conflict_total = 0
    if trains is not None:
        landing_hard_total, landing_conflict_total = qea_landing_conflict_counts(
            selected, trains, options_by_train
        )
    effective_hard_total = max(hard_total, landing_hard_total)
    safety_delay_penalty = landing_hard_total * LOCAL_RECOVERY_LONG_WAIT_THRESHOLD
    effective_total_delay = total_delay + safety_delay_penalty
    effective_max_delay = max_delay + (
        LOCAL_RECOVERY_LONG_WAIT_THRESHOLD if landing_hard_total else 0
    )
    tie_breaker = pair_cost_total * 1e-6 + landing_conflict_total * 1e-3
    objective = unified_objective_key(
        effective_hard_total,
        effective_total_delay,
        effective_max_delay,
        adjusted_trains,
        track_changes,
        tie_breaker,
    )
    objective_value = unified_objective_value(
        effective_hard_total,
        effective_total_delay,
        effective_max_delay,
        adjusted_trains,
        track_changes,
        tie_breaker,
    )
    return objective, objective_value


def assignment_energy_unified(
    selected, linear_costs, pair_costs_map, hard_counts, hard_risks
):
    del hard_risks
    options_by_train = OBJECTIVE_OPTIONS_BY_TRAIN
    reference_ids = OBJECTIVE_REFERENCE_ASSIGNMENT_IDS or {}
    reference_delay_by_train = OBJECTIVE_REFERENCE_DELAY_BY_TRAIN or {}
    if not options_by_train:
        return exp.assignment_display_energy(selected, linear_costs, pair_costs_map)
    train_order = sorted(selected)
    objective, objective_value = qea_fast_objective(
        selected,
        train_order,
        reference_delay_by_train,
        options_by_train,
        reference_ids,
        linear_costs,
        pair_costs_map,
        hard_counts,
    )
    return objective_value


def hard_count_for_selection_symmetric(selected, hard_counts):
    train_ids = sorted(selected)
    total = 0
    for left_index, left_id in enumerate(train_ids):
        for right_id in train_ids[left_index + 1 :]:
            total += pair_hard_count(hard_counts, selected[left_id], selected[right_id])
    return total


def qea_optimize_explicit_objective(
    trains,
    options_by_train,
    linear_costs,
    pair_costs_map,
    hard_counts,
    hard_risks,
    train_map,
    time_limit_seconds=400.0,
    seed=42,
    pop_size=50,
    max_generations=500,
    initial_assignment_ids=None,
    reference_trains=None,
    disturbance_minute=None,
    frozen_train_ids=None,
    max_safety_wait_rounds=160,
    subproblem_time_limit_seconds=None,
):
    global QEA_CANDIDATE_POOL
    QEA_CANDIDATE_POOL = []
    frozen_train_ids = frozen_train_ids or set()
    reference_trains = reference_trains or trains
    reference_delay_by_train = base.compute_delay_statistics(reference_trains, trains)[
        "delay_by_train"
    ]
    rng = random.Random(seed)
    np_rng = np.random.default_rng(seed)
    start = time.perf_counter()
    train_order = base.build_train_order(trains, options_by_train)
    planned = (
        dict(initial_assignment_ids)
        if initial_assignment_ids
        else base.build_planned_assignment_ids(trains, options_by_train)
    )
    objective_reference_ids = OBJECTIVE_REFERENCE_ASSIGNMENT_IDS or planned

    def normalize_assignment(selected):
        normalized = {}
        for train_id in train_order:
            valid_option_ids = {
                option.option_id for option in options_by_train[train_id]
            }
            option_id = selected.get(train_id)
            if option_id not in valid_option_ids:
                option_id = options_by_train[train_id][0].option_id
            normalized[train_id] = option_id
        return normalized

    eval_cache = {}

    def evaluate(selected):
        normalized = normalize_assignment(selected)
        key = tuple((train_id, normalized[train_id]) for train_id in train_order)
        cached = eval_cache.get(key)
        if cached is not None:
            return cached
        objective, display_energy = qea_fast_objective(
            normalized,
            train_order,
            reference_delay_by_train,
            options_by_train,
            objective_reference_ids,
            linear_costs,
            pair_costs_map,
            hard_counts,
            trains=trains,
        )
        result = (normalized, objective, display_energy)
        if len(eval_cache) > 6000:
            eval_cache.clear()
        eval_cache[key] = result
        return result

    def add_qea_candidate(selected, objective, display_energy, source):
        candidate = dict(selected)
        key = tuple((train_id, candidate[train_id]) for train_id in train_order)
        existing_keys = {entry["key"] for entry in QEA_CANDIDATE_POOL}
        if key in existing_keys:
            return
        QEA_CANDIDATE_POOL.append(
            {
                "assignment": candidate,
                "objective": objective,
                "energy": display_energy,
                "source": source,
                "key": key,
            }
        )
        QEA_CANDIDATE_POOL.sort(key=lambda entry: entry["objective"])
        del QEA_CANDIDATE_POOL[24:]

    def time_exhausted():
        return time.perf_counter() - start > time_limit_seconds

    def conflict_train_counts(selected):
        adjusted_trains = apply_selected_option_delays(trains, selected)
        assignment = base.build_assignment_lookup(selected, options_by_train)
        assignment = base.rebuild_assignment_for_trains(adjusted_trains, assignment)
        conflicts = base.collect_conflicts(adjusted_trains, assignment, SCHEME_QEA)
        counts = {}
        for conflict in conflicts:
            if conflict.conflict_level != "硬冲突":
                continue
            left_id = exp.train_id_from_conflict_name(conflict.train1)
            right_id = exp.train_id_from_conflict_name(conflict.train2)
            counts[left_id] = counts.get(left_id, 0) + 1
            counts[right_id] = counts.get(right_id, 0) + 1
        return counts

    def landing_conflict_edges(selected):
        adjusted_trains = apply_selected_option_delays(trains, selected)
        assignment = base.build_assignment_lookup(selected, options_by_train)
        assignment = base.rebuild_assignment_for_trains(adjusted_trains, assignment)
        conflicts = base.collect_conflicts(adjusted_trains, assignment, SCHEME_QEA)
        edges = []
        for conflict in conflicts:
            if conflict.conflict_level != "硬冲突":
                continue
            left_id = exp.train_id_from_conflict_name(conflict.train1)
            right_id = exp.train_id_from_conflict_name(conflict.train2)
            if left_id in options_by_train and right_id in options_by_train:
                edges.append((left_id, right_id))
        return edges

    def landing_conflict_components(selected):
        graph = {}
        for left_id, right_id in landing_conflict_edges(selected):
            graph.setdefault(left_id, set()).add(right_id)
            graph.setdefault(right_id, set()).add(left_id)
        components = []
        seen = set()
        for train_id in sorted(graph):
            if train_id in seen:
                continue
            stack = [train_id]
            seen.add(train_id)
            component = []
            while stack:
                current = stack.pop()
                component.append(current)
                for neighbor in graph.get(current, ()):
                    if neighbor not in seen:
                        seen.add(neighbor)
                        stack.append(neighbor)
            components.append(component)
        return sorted(
            components,
            key=lambda component: (
                -len(component),
                min(train_map[train_id].arrival_min for train_id in component),
            ),
        )

    def option_conflict_score_against_current(train_id, option_id, selected):
        total = 0
        for other_train_id, other_option_id in selected.items():
            if other_train_id == train_id:
                continue
            total += pair_hard_count(hard_counts, option_id, other_option_id)
        return total

    def conflict_block_candidate_options(train_id, selected, option_limit=12):
        current_option_id = selected[train_id]
        ranked_options = sorted(
            options_by_train[train_id],
            key=lambda option: (
                option_conflict_score_against_current(
                    train_id, option.option_id, selected
                ),
                selected_option_delay_minutes(option.option_id),
                linear_costs.get(option.option_id, 0.0),
                option.candidate_rank,
                option.option_id,
            ),
        )
        selected_options = ranked_options[:option_limit]
        if all(option.option_id != current_option_id for option in selected_options):
            current_option = next(
                (
                    option
                    for option in options_by_train[train_id]
                    if option.option_id == current_option_id
                ),
                None,
            )
            if current_option is not None:
                selected_options = [
                    current_option,
                    *selected_options[: option_limit - 1],
                ]
        return selected_options

    def high_delay_train_ids(selected, limit=30):
        ranked = sorted(
            selected,
            key=lambda train_id: (
                -(
                    reference_delay_by_train.get(train_id, 0)
                    + selected_option_delay_minutes(selected[train_id])
                ),
                train_map[train_id].arrival_min,
                train_map[train_id].index,
            ),
        )
        return [
            train_id
            for train_id in ranked
            if reference_delay_by_train.get(train_id, 0)
            + selected_option_delay_minutes(selected[train_id])
            > 0
        ][:limit]

    def conflict_subgraph_ids(selected, train_limit=100, include_high_delay=True):
        adjusted_trains = apply_selected_option_delays(trains, selected)
        assignment = base.build_assignment_lookup(selected, options_by_train)
        assignment = base.rebuild_assignment_for_trains(adjusted_trains, assignment)
        conflicts = base.collect_conflicts(adjusted_trains, assignment, SCHEME_QEA)
        counts = {}
        neighbors = {}
        for conflict in conflicts:
            if conflict.conflict_level != "硬冲突":
                continue
            left_id = exp.train_id_from_conflict_name(conflict.train1)
            right_id = exp.train_id_from_conflict_name(conflict.train2)
            counts[left_id] = counts.get(left_id, 0) + 1
            counts[right_id] = counts.get(right_id, 0) + 1
            neighbors.setdefault(left_id, set()).add(right_id)
            neighbors.setdefault(right_id, set()).add(left_id)
        ordered = sorted(
            counts,
            key=lambda train_id: (
                -counts[train_id],
                train_map[train_id].arrival_min,
                train_map[train_id].index,
            ),
        )
        active_ids = []
        seen = set()
        for train_id in ordered:
            if train_id not in seen:
                active_ids.append(train_id)
                seen.add(train_id)
            for neighbor_id in sorted(
                neighbors.get(train_id, ()),
                key=lambda item: (train_map[item].arrival_min, train_map[item].index),
            ):
                if neighbor_id not in seen:
                    active_ids.append(neighbor_id)
                    seen.add(neighbor_id)
                if len(active_ids) >= train_limit:
                    break
            if len(active_ids) >= train_limit:
                break
        if include_high_delay:
            for train_id in high_delay_train_ids(selected, limit=train_limit):
                if train_id not in seen:
                    active_ids.append(train_id)
                    seen.add(train_id)
                if len(active_ids) >= train_limit:
                    break
        return set(active_ids[:train_limit])

    def expand_with_time_neighbors(seed_ids, train_limit=30, window_minutes=18):
        active_ids = []
        seen = set()

        def add(train_id):
            if train_id in options_by_train and train_id not in seen:
                active_ids.append(train_id)
                seen.add(train_id)

        ordered_seed_ids = sorted(
            seed_ids,
            key=lambda train_id: (
                train_map[train_id].arrival_min,
                train_map[train_id].index,
            ),
        )
        for train_id in ordered_seed_ids:
            add(train_id)
        for train_id in ordered_seed_ids:
            seed_train = train_map[train_id]
            for other_id in train_order:
                if other_id in seen:
                    continue
                other_train = train_map[other_id]
                if (
                    abs(other_train.arrival_min - seed_train.arrival_min)
                    <= window_minutes
                ):
                    add(other_id)
                if len(active_ids) >= train_limit:
                    break
            if len(active_ids) >= train_limit:
                break
        return set(active_ids[:train_limit])

    def elite_refine(selected, selected_objective, max_passes=2, option_limit=10):
        refined = dict(selected)
        refined_objective = selected_objective
        improvements = 0
        for _ in range(max(1, max_passes)):
            if time_exhausted():
                break
            counts = conflict_train_counts(refined)
            ordered_train_ids = sorted(
                counts or {train_id: 0 for train_id in train_order},
                key=lambda train_id: (
                    -counts.get(train_id, 0),
                    train_map[train_id].arrival_min,
                    train_map[train_id].index,
                ),
            )[:50]
            improved_this_pass = False
            for train_id in ordered_train_ids:
                if time_exhausted():
                    break
                current_option_id = refined[train_id]
                candidate_options = sorted(
                    options_by_train[train_id],
                    key=lambda option: (
                        option.option_id != current_option_id,
                        selected_option_delay_minutes(option.option_id),
                        linear_costs.get(option.option_id, 0.0),
                        option.candidate_rank,
                        option.option_id,
                    ),
                )[:option_limit]
                best_local_option = current_option_id
                best_local_objective = refined_objective
                for option in candidate_options:
                    if option.option_id == current_option_id:
                        continue
                    trial = dict(refined)
                    trial[train_id] = option.option_id
                    _, trial_objective, _ = evaluate(trial)
                    if trial_objective < best_local_objective:
                        best_local_objective = trial_objective
                        best_local_option = option.option_id
                if best_local_option != current_option_id:
                    refined[train_id] = best_local_option
                    refined_objective = best_local_objective
                    improvements += 1
                    improved_this_pass = True
            if not improved_this_pass:
                break
        _, refined_objective, display_energy = evaluate(refined)
        return refined, refined_objective, display_energy, improvements

    def zero_conflict_delay_refine(
        selected, selected_objective, max_passes=2, train_limit=80, option_limit=24
    ):
        refined = dict(selected)
        refined_objective = selected_objective
        improvements = 0
        for _ in range(max(1, max_passes)):
            if time_exhausted() or refined_objective[0] > 0:
                break
            ordered_train_ids = sorted(
                train_order,
                key=lambda train_id: (
                    -selected_option_delay_minutes(refined[train_id]),
                    -reference_delay_by_train.get(train_id, 0),
                    train_map[train_id].arrival_min,
                    train_map[train_id].index,
                ),
            )[:train_limit]
            improved_this_pass = False
            for train_id in ordered_train_ids:
                if time_exhausted():
                    break
                current_option_id = refined[train_id]
                current_delay = selected_option_delay_minutes(current_option_id)
                candidate_options = sorted(
                    options_by_train[train_id],
                    key=lambda option: (
                        selected_option_delay_minutes(option.option_id),
                        linear_costs.get(option.option_id, 0.0),
                        option.candidate_rank,
                        option.option_id,
                    ),
                )[:option_limit]
                best_local_option = current_option_id
                best_local_objective = refined_objective
                for option in candidate_options:
                    if option.option_id == current_option_id:
                        continue
                    option_delay = selected_option_delay_minutes(option.option_id)
                    if option_delay > current_delay:
                        continue
                    trial = dict(refined)
                    trial[train_id] = option.option_id
                    _, trial_objective, _ = evaluate(trial)
                    if (
                        trial_objective[0] == 0
                        and trial_objective < best_local_objective
                    ):
                        best_local_objective = trial_objective
                        best_local_option = option.option_id
                if best_local_option != current_option_id:
                    refined[train_id] = best_local_option
                    refined_objective = best_local_objective
                    improvements += 1
                    improved_this_pass = True
            if not improved_this_pass:
                break
        _, refined_objective, display_energy = evaluate(refined)
        return refined, refined_objective, display_energy, improvements

    def landing_conflict_refine(
        selected,
        selected_objective,
        max_passes=1,
        train_limit=40,
        option_limit=24,
    ):
        refined = dict(selected)
        refined_objective = selected_objective
        improvements = 0
        for _ in range(max(1, max_passes)):
            if time_exhausted():
                break
            counts = conflict_train_counts(refined)
            if not counts:
                break
            ordered_train_ids = sorted(
                counts,
                key=lambda train_id: (
                    -counts.get(train_id, 0),
                    train_map[train_id].arrival_min,
                    train_map[train_id].index,
                ),
            )[:train_limit]
            improved_this_pass = False
            for train_id in ordered_train_ids:
                if time_exhausted():
                    break
                current_option_id = refined[train_id]
                candidate_options = sorted(
                    options_by_train[train_id],
                    key=lambda option: (
                        option.option_id == current_option_id,
                        option_conflict_score_against_current(
                            train_id, option.option_id, refined
                        ),
                        selected_option_delay_minutes(option.option_id),
                        linear_costs.get(option.option_id, 0.0),
                        option.candidate_rank,
                        option.option_id,
                    ),
                )[:option_limit]
                best_local_option = current_option_id
                best_local_objective = refined_objective
                for option in candidate_options:
                    if option.option_id == current_option_id:
                        continue
                    trial = dict(refined)
                    trial[train_id] = option.option_id
                    _, trial_objective, _ = evaluate(trial)
                    if trial_objective < best_local_objective:
                        best_local_objective = trial_objective
                        best_local_option = option.option_id
                if best_local_option != current_option_id:
                    refined[train_id] = best_local_option
                    refined_objective = best_local_objective
                    improvements += 1
                    improved_this_pass = True
            if not improved_this_pass:
                break
        _, refined_objective, display_energy = evaluate(refined)
        return refined, refined_objective, display_energy, improvements

    def landing_conflict_block_refine(
        selected,
        selected_objective,
        max_components=2,
        attempts_per_component=10,
        block_limit=18,
        option_limit=12,
    ):
        refined = dict(selected)
        refined_objective = selected_objective
        improvements = 0
        components = landing_conflict_components(refined)
        for component in components[:max_components]:
            if time_exhausted():
                break
            block_ids = expand_with_time_neighbors(
                component, train_limit=block_limit, window_minutes=18
            )
            if not block_ids:
                continue
            ordered_block = sorted(
                block_ids,
                key=lambda train_id: (
                    train_id not in component,
                    train_map[train_id].arrival_min,
                    train_map[train_id].index,
                ),
            )
            for attempt_index in range(max(1, attempts_per_component)):
                if time_exhausted():
                    break
                trial = dict(refined)
                trial_order = list(ordered_block)
                if attempt_index > 0:
                    rng.shuffle(trial_order)
                for train_id in trial_order:
                    candidate_options = conflict_block_candidate_options(
                        train_id, trial, option_limit=option_limit
                    )
                    if not candidate_options:
                        continue
                    if attempt_index == 0:
                        chosen_option = candidate_options[0]
                    else:
                        chosen_option = rng.choice(
                            candidate_options[: min(5, len(candidate_options))]
                        )
                    trial[train_id] = chosen_option.option_id
                _, trial_objective, _ = evaluate(trial)
                if trial_objective < refined_objective:
                    refined = trial
                    refined_objective = trial_objective
                    improvements += 1
                    break
        _, refined_objective, display_energy = evaluate(refined)
        return refined, refined_objective, display_energy, improvements

    def apply_conflict_subgraph_repair(selected, selected_objective, generation):
        if time_exhausted():
            return selected, selected_objective, evaluate(selected)[2], False
        if selected_objective[0] > 0:
            hard_seed_ids = conflict_subgraph_ids(
                selected, train_limit=60, include_high_delay=False
            )
            subgraph_ids = expand_with_time_neighbors(
                hard_seed_ids, train_limit=30, window_minutes=18
            )
        else:
            subgraph_ids = conflict_subgraph_ids(
                selected, train_limit=100, include_high_delay=True
            )
        if not subgraph_ids and selected_objective[0] <= 0:
            subgraph_ids = set(high_delay_train_ids(selected, limit=50))
        if not subgraph_ids:
            return selected, selected_objective, evaluate(selected)[2], False
        remaining_time = max(0.5, time_limit_seconds - (time.perf_counter() - start))
        repair_time = min(25.0, max(1.0, remaining_time * 0.45))
        repaired_ids, status = exp.solve_subproblem(
            initial_assignment=selected,
            active_train_ids=subgraph_ids,
            options_by_train=options_by_train,
            linear_costs=linear_costs,
            pair_costs_map=pair_costs_map,
            hard_counts=hard_counts,
            hard_risks={},
            time_limit_seconds=repair_time,
            strict_hard=True,
            max_options_per_train=40,
            preserve_active_scope=True,
        )
        if repaired_ids is None:
            history.append(
                {
                    "迭代轮次": generation,
                    "阶段": "QEA-NS局部可行性修复",
                    "硬冲突数": selected_objective[0],
                    "能量值": f"{evaluate(selected)[2]:.2f}",
                    "用时(s)": f"{time.perf_counter() - start:.3f}",
                    "备注": "",
                }
            )
            return selected, selected_objective, evaluate(selected)[2], False
        repaired, repaired_objective, repaired_energy = evaluate(repaired_ids)
        accepted = repaired_objective < selected_objective
        history.append(
            {
                "迭代轮次": generation,
                "阶段": "QEA-NS局部可行性修复",
                "硬冲突数": repaired_objective[0],
                "能量值": f"{repaired_energy:.2f}",
                "用时(s)": f"{time.perf_counter() - start:.3f}",
                "备注": "",
            }
        )
        if accepted:
            return repaired, repaired_objective, repaired_energy, True
        return selected, selected_objective, evaluate(selected)[2], False

    def apply_qea_projection(selected, selected_objective, generation):
        if not ENABLE_QEA_PROJECTION:
            return selected, selected_objective, evaluate(selected)[2], False
        if selected_objective[0] <= 0 or time_exhausted():
            return selected, selected_objective, evaluate(selected)[2], False
        projection_threshold = max(30, len(train_order) // 4)
        if selected_objective[0] > projection_threshold:
            return selected, selected_objective, evaluate(selected)[2], False
        elapsed_ratio = (time.perf_counter() - start) / max(1.0, time_limit_seconds)
        if elapsed_ratio >= 0.75:
            return selected, selected_objective, evaluate(selected)[2], False
        projection_time = min(35.0, max(8.0, time_limit_seconds * 0.08))
        projected_ids, _, projection_history = run_qea_feasibility_refinement(
            assignment_ids=selected,
            trains=trains,
            reference_trains=reference_trains,
            options_by_train=options_by_train,
            linear_costs=linear_costs,
            pair_costs_map=pair_costs_map,
            hard_counts=hard_counts,
            hard_risks=hard_risks,
            disturbance_minute=disturbance_minute,
            frozen_train_ids=frozen_train_ids,
            time_limit_seconds=projection_time,
        )
        for row in projection_history:
            updated = dict(row)
            updated["迭代轮次"] = generation
            updated["阶段"] = "QEA-NS主循环可行投影"
            history.append(updated)
        projected, projected_objective, projected_energy = evaluate(projected_ids)
        accepted = projected_objective < selected_objective
        if accepted:
            return projected, projected_objective, projected_energy, True
        return selected, selected_objective, evaluate(selected)[2], False

    best, best_objective, best_energy = evaluate(planned)
    add_qea_candidate(best, best_objective, best_energy, "initial")
    zero_generation = 0 if best_objective[0] == 0 else -1
    sample_count = 0
    refinement_count = 0
    history = [
        {
            "迭代轮次": 0,
            "阶段": "QEA-NS显式目标初始化",
            "硬冲突数": best_objective[0],
            "能量值": f"{best_energy:.2f}",
            "用时(s)": f"{time.perf_counter() - start:.3f}",
            "备注": "",
        }
    ]
    initial_refine_budget = min(max(5.0, time_limit_seconds * 0.35), 60.0)

    def initial_refine_budget_available():
        return time_limit_seconds > 0

    best, best_objective, best_energy, improvements = elite_refine(
        best, best_objective, max_passes=1, option_limit=8
    )
    add_qea_candidate(best, best_objective, best_energy, "initial_elite_refine")
    refinement_count += improvements
    if improvements:
        history.append(
            {
                "迭代轮次": 0,
                "阶段": "QEA-NS显式目标精英变异",
                "硬冲突数": best_objective[0],
                "能量值": f"{best_energy:.2f}",
                "用时(s)": f"{time.perf_counter() - start:.3f}",
                "备注": "",
            }
        )

    if best_objective[0] > 0 and initial_refine_budget_available():
        best, best_objective, best_energy, improvements = landing_conflict_refine(
            best, best_objective, max_passes=1, train_limit=35, option_limit=18
        )
        refinement_count += improvements
        if improvements:
            add_qea_candidate(
                best, best_objective, best_energy, "initial_landing_conflict_refine"
            )
            if zero_generation < 0 and best_objective[0] == 0:
                zero_generation = 0
            history.append(
                {
                    "迭代轮次": 0,
                    "阶段": "QEA-NS落地冲突定向变异",
                    "硬冲突数": best_objective[0],
                    "能量值": f"{best_energy:.2f}",
                    "用时(s)": f"{time.perf_counter() - start:.3f}",
                    "备注": "",
                }
            )

    if best_objective[0] > 0 and initial_refine_budget_available():
        best, best_objective, best_energy, improvements = landing_conflict_block_refine(
            best,
            best_objective,
            max_components=2,
            attempts_per_component=6,
            block_limit=16,
            option_limit=10,
        )
        refinement_count += improvements
        if improvements:
            add_qea_candidate(
                best, best_objective, best_energy, "initial_conflict_block_refine"
            )
            if zero_generation < 0 and best_objective[0] == 0:
                zero_generation = 0
            history.append(
                {
                    "迭代轮次": 0,
                    "阶段": "QEA-NS冲突块协同变异",
                    "硬冲突数": best_objective[0],
                    "能量值": f"{best_energy:.2f}",
                    "用时(s)": f"{time.perf_counter() - start:.3f}",
                    "备注": "",
                }
            )

    if best_objective[0] > 0 and not initial_refine_budget_available():
        history.append(
            {
                "迭代轮次": 0,
                "阶段": "QEA-NS初始修复预算截止",
                "硬冲突数": best_objective[0],
                "能量值": f"{best_energy:.2f}",
                "用时(s)": f"{time.perf_counter() - start:.3f}",
                "备注": "",
            }
        )

    theta_pop = []
    for particle_index in range(max(1, pop_size)):
        particle = {}
        for train_id in train_order:
            options = options_by_train[train_id]
            winner_idx = next(
                (
                    index
                    for index, option in enumerate(options)
                    if option.option_id == best.get(train_id)
                ),
                0,
            )
            if particle_index == 0:
                theta = np.full(len(options), np.pi / 2.0)
                theta[winner_idx] = 0.0
            elif particle_index < max(2, pop_size // 3):
                theta = np_rng.normal(np.pi / 2.0, 0.35, len(options))
                theta[winner_idx] = np_rng.normal(0.0, 0.12)
                theta = np.clip(theta, 0.0, np.pi / 2.0)
            else:
                theta = np_rng.uniform(0.0, np.pi / 2.0, len(options))
            particle[train_id] = theta
        theta_pop.append(particle)

    stagnation = 0
    last_logged_generation = 0
    deterministic_generations = min(max_generations, QEA_DETERMINISTIC_GENERATION_CAP)
    for generation in range(1, deterministic_generations + 1):
        if time_exhausted():
            history.append(
                {
                    "迭代轮次": generation,
                    "阶段": "QEA-NS时间预算截止",
                    "硬冲突数": best_objective[0],
                    "能量值": f"{best_energy:.2f}",
                    "用时(s)": f"{time.perf_counter() - start:.3f}",
                    "备注": "",
                }
            )
            break
        progress_ratio = generation / max(1, deterministic_generations)
        alpha = 0.15 * (1.0 - progress_ratio) + 0.05
        candidates = [(best, best_objective, best_energy)]
        conflict_ids = list(conflict_train_counts(best))
        for particle in theta_pop:
            if time_exhausted():
                break
            sampled = {}
            for train_id in train_order:
                if time_exhausted():
                    break
                options = options_by_train[train_id]
                probs = np.square(np.cos(particle[train_id]))
                probs = np.clip(probs, 1e-9, None)
                probs = probs / probs.sum()
                index = int(np_rng.choice(len(options), p=probs))
                sampled[train_id] = options[index].option_id
            if len(sampled) != len(train_order):
                break
            if conflict_ids and rng.random() < 0.55:
                sample_size = min(len(conflict_ids), max(1, len(conflict_ids) // 2))
                for train_id in rng.sample(conflict_ids, sample_size):
                    sampled[train_id] = rng.choice(options_by_train[train_id]).option_id
            if generation > 1 and rng.random() < 0.25:
                sample_size = max(1, len(train_order) // 10)
                for train_id in rng.sample(train_order, sample_size):
                    sampled[train_id] = rng.choice(options_by_train[train_id]).option_id
            normalized, objective, display_energy = evaluate(sampled)
            sample_count += 1
            candidates.append((normalized, objective, display_energy))

        candidates.sort(key=lambda item: item[1])
        if time_exhausted():
            break
        for candidate, candidate_objective, candidate_energy in candidates[:3]:
            add_qea_candidate(
                candidate,
                candidate_objective,
                candidate_energy,
                f"generation_{generation}_top_sample",
            )
        current, current_objective, current_energy = candidates[0]
        if current_objective < best_objective:
            best = dict(current)
            best_objective = current_objective
            best_energy = current_energy
            add_qea_candidate(
                best, best_objective, best_energy, f"generation_{generation}_best"
            )
            stagnation = 0
            if zero_generation < 0 and best_objective[0] == 0:
                zero_generation = generation
        else:
            stagnation += 1

        if generation % 10 == 0 and conflict_ids:
            for particle in theta_pop:
                reset_size = min(len(conflict_ids), max(1, len(conflict_ids) // 2))
                for train_id in rng.sample(conflict_ids, reset_size):
                    particle[train_id] = np_rng.uniform(
                        0.0, 2.0 * np.pi, len(options_by_train[train_id])
                    )
            history.append(
                {
                    "迭代轮次": generation,
                    "阶段": "QEA-NS冲突列车强制变异",
                    "硬冲突数": best_objective[0],
                    "能量值": f"{best_energy:.2f}",
                    "用时(s)": f"{time.perf_counter() - start:.3f}",
                    "备注": "",
                }
            )

        if best_objective[0] > 0 and generation % 10 == 0:
            best, best_objective, best_energy, improvements = landing_conflict_refine(
                best, best_objective, max_passes=1, train_limit=45, option_limit=24
            )
            refinement_count += improvements
            if improvements:
                stagnation = 0
                add_qea_candidate(
                    best,
                    best_objective,
                    best_energy,
                    f"generation_{generation}_landing_conflict_refine",
                )
                if zero_generation < 0 and best_objective[0] == 0:
                    zero_generation = generation
                history.append(
                    {
                        "迭代轮次": generation,
                        "阶段": "QEA-NS落地冲突定向变异",
                        "硬冲突数": best_objective[0],
                        "能量值": f"{best_energy:.2f}",
                        "用时(s)": f"{time.perf_counter() - start:.3f}",
                        "备注": "",
                    }
                )

        if best_objective[0] > 0 and generation % 10 == 0:
            best, best_objective, best_energy, improvements = (
                landing_conflict_block_refine(
                    best,
                    best_objective,
                    max_components=2,
                    attempts_per_component=8,
                    block_limit=18,
                    option_limit=12,
                )
            )
            refinement_count += improvements
            if improvements:
                stagnation = 0
                add_qea_candidate(
                    best,
                    best_objective,
                    best_energy,
                    f"generation_{generation}_conflict_block_refine",
                )
                if zero_generation < 0 and best_objective[0] == 0:
                    zero_generation = generation
                history.append(
                    {
                        "迭代轮次": generation,
                        "阶段": "QEA-NS冲突块协同变异",
                        "硬冲突数": best_objective[0],
                        "能量值": f"{best_energy:.2f}",
                        "用时(s)": f"{time.perf_counter() - start:.3f}",
                        "备注": "",
                    }
                )

        if generation % 25 == 0 and (best_objective[0] > 0 or best_objective[1] > 0):
            best, best_objective, best_energy, improved_by_subgraph = (
                apply_conflict_subgraph_repair(best, best_objective, generation)
            )
            if improved_by_subgraph:
                stagnation = 0
                refinement_count += 1
                add_qea_candidate(
                    best,
                    best_objective,
                    best_energy,
                    f"generation_{generation}_subgraph_repair",
                )
                if zero_generation < 0 and best_objective[0] == 0:
                    zero_generation = generation

        if generation % 25 == 0 or (best_objective[0] > 0 and generation % 10 == 0):
            best, best_objective, best_energy, improvements = elite_refine(
                best, best_objective, max_passes=1, option_limit=10
            )
            refinement_count += improvements
            if improvements:
                stagnation = 0
                add_qea_candidate(
                    best,
                    best_objective,
                    best_energy,
                    f"generation_{generation}_elite_refine",
                )
                if zero_generation < 0 and best_objective[0] == 0:
                    zero_generation = generation
                history.append(
                    {
                        "迭代轮次": generation,
                        "阶段": "QEA-NS显式目标精英变异",
                        "硬冲突数": best_objective[0],
                        "能量值": f"{best_energy:.2f}",
                        "用时(s)": f"{time.perf_counter() - start:.3f}",
                        "备注": "",
                    }
                )

        if best_objective[0] == 0 and generation % 20 == 0:
            best, best_objective, best_energy, improvements = (
                zero_conflict_delay_refine(
                    best,
                    best_objective,
                    max_passes=2,
                    train_limit=min(120, len(train_order)),
                    option_limit=28,
                )
            )
            refinement_count += improvements
            if improvements:
                stagnation = 0
                add_qea_candidate(
                    best,
                    best_objective,
                    best_energy,
                    f"generation_{generation}_delay_refine",
                )
                history.append(
                    {
                        "迭代轮次": generation,
                        "阶段": "QEA-NS零冲突降晚点精修",
                        "硬冲突数": best_objective[0],
                        "能量值": f"{best_energy:.2f}",
                        "用时(s)": f"{time.perf_counter() - start:.3f}",
                        "备注": "",
                    }
                )

        if best_objective[0] > 0 and generation % 10 == 0:
            best, best_objective, best_energy, improved_by_projection = (
                apply_qea_projection(best, best_objective, generation)
            )
            if improved_by_projection:
                stagnation = 0
                refinement_count += 1
                add_qea_candidate(
                    best,
                    best_objective,
                    best_energy,
                    f"generation_{generation}_qea_projection",
                )
                if zero_generation < 0 and best_objective[0] == 0:
                    zero_generation = generation

        guide_pool = [
            entry["assignment"]
            for entry in QEA_CANDIDATE_POOL[: max(1, min(len(QEA_CANDIDATE_POOL), 8))]
        ]
        if not guide_pool:
            guide_pool = [best]
        for particle_index, particle in enumerate(theta_pop):
            guide = guide_pool[particle_index % len(guide_pool)]
            for train_id in train_order:
                options = options_by_train[train_id]
                winner_idx = next(
                    (
                        index
                        for index, option in enumerate(options)
                        if option.option_id == guide.get(train_id)
                    ),
                    0,
                )
                target = np.full(len(options), np.pi / 2.0)
                target[winner_idx] = 0.0
                particle[train_id] = (1.0 - alpha) * particle[train_id] + alpha * target

        if stagnation > 35:
            reset_pool = conflict_ids or train_order
            for particle in theta_pop:
                if rng.random() < 0.45:
                    for train_id in rng.sample(
                        reset_pool, min(len(reset_pool), max(1, len(reset_pool) // 2))
                    ):
                        particle[train_id] = np_rng.uniform(
                            0.0, 2.0 * np.pi, len(options_by_train[train_id])
                        )
            stagnation = 0

        if generation <= 10 or generation % 10 == 0:
            history.append(
                {
                    "迭代轮次": generation,
                    "阶段": "QEA-NS显式目标主循环",
                    "硬冲突数": best_objective[0],
                    "能量值": f"{best_energy:.2f}",
                    "用时(s)": f"{time.perf_counter() - start:.3f}",
                    "备注": "",
                }
            )
            last_logged_generation = generation

    best, best_objective, best_energy, improvements = elite_refine(
        best, best_objective, max_passes=2, option_limit=12
    )
    if best_objective[0] == 0:
        best, best_objective, best_energy, delay_improvements = (
            zero_conflict_delay_refine(
                best,
                best_objective,
                max_passes=3,
                train_limit=len(train_order),
                option_limit=40,
            )
        )
        improvements += delay_improvements
    add_qea_candidate(best, best_objective, best_energy, "final_elite_refine")
    refinement_count += improvements
    elapsed = time.perf_counter() - start
    history.append(
        {
            "迭代轮次": "final",
            "阶段": "QEA-NS显式目标最终解",
            "硬冲突数": best_objective[0],
            "能量值": f"{best_energy:.2f}",
            "用时(s)": f"{elapsed:.3f}",
            "备注": "",
        }
    )
    diagnostics = {
        "求解时间(s)": elapsed,
        "收敛代数": zero_generation if zero_generation >= 0 else "",
        "邻域修复调用次数": 0,
        "QEA-NS采样次数": sample_count,
        "QEA-NS精英变异次数": refinement_count,
        "QEA-NS落地修复次数": 0,
        "QEA-NS冲突修复次数": 0,
        "最终硬冲突数": best_objective[0],
        "最终能量值": best_energy,
    }
    return best, history, diagnostics


setattr(exp, "qea_optimize", qea_optimize_explicit_objective)
setattr(exp, "assignment_energy", assignment_energy_unified)
setattr(exp, "hard_count_for_selection", hard_count_for_selection_symmetric)

_ORIGINAL_COLLECT_SOLUTION_METRICS = exp.collect_solution_metrics
_ORIGINAL_COLLECT_RAW_SOLUTION_METRICS = exp.collect_raw_solution_metrics


def apply_unified_energy_metric(metrics, assignment_ids, options_by_train):
    try:
        hard_count = int(metrics.get("硬冲突项", 0))
        total_delay = int(metrics.get("总晚点时长", 0))
        max_delay = int(metrics.get("最大晚点时长", 0))
    except (TypeError, ValueError):
        return metrics
    _, track_changes, adjusted_trains = selected_adjustment_components(
        assignment_ids or {},
        options_by_train or {},
        OBJECTIVE_REFERENCE_ASSIGNMENT_IDS or {},
    )
    metrics["能量值"] = unified_objective_value(
        hard_count,
        total_delay,
        max_delay,
        adjusted_trains,
        track_changes,
    )
    return metrics


def collect_solution_metrics_unified_energy(*args, **kwargs):
    metrics, assignment, conflicts = _ORIGINAL_COLLECT_SOLUTION_METRICS(*args, **kwargs)
    assignment_ids = args[3] if len(args) > 3 else kwargs.get("assignment_ids")
    options_by_train = args[4] if len(args) > 4 else kwargs.get("options_by_train")
    return (
        apply_unified_energy_metric(metrics, assignment_ids, options_by_train),
        assignment,
        conflicts,
    )


def collect_raw_solution_metrics_unified_energy(*args, **kwargs):
    metrics, assignment, conflicts = _ORIGINAL_COLLECT_RAW_SOLUTION_METRICS(
        *args, **kwargs
    )
    assignment_ids = args[3] if len(args) > 3 else kwargs.get("assignment_ids")
    options_by_train = args[4] if len(args) > 4 else kwargs.get("options_by_train")
    return (
        apply_unified_energy_metric(metrics, assignment_ids, options_by_train),
        assignment,
        conflicts,
    )


setattr(exp, "collect_solution_metrics", collect_solution_metrics_unified_energy)
setattr(
    exp, "collect_raw_solution_metrics", collect_raw_solution_metrics_unified_energy
)


def option_track_changed(option, baseline_option):
    if baseline_option is None:
        return False
    return base.assignment_option_signature(option) != base.assignment_option_signature(
        baseline_option
    )


def option_lookup_by_id(options_by_train):
    return {
        option.option_id: option
        for options in options_by_train.values()
        for option in options
    }


def baseline_option_lookup_by_train(options_by_train, reference_ids):
    lookup = option_lookup_by_id(options_by_train)
    return {
        train_id: lookup.get(option_id)
        for train_id, option_id in (reference_ids or {}).items()
    }


def selected_adjustment_components(selected, options_by_train, reference_ids):
    option_lookup = option_lookup_by_id(options_by_train)
    baseline_lookup = baseline_option_lookup_by_train(options_by_train, reference_ids)
    delay_by_train = {}
    track_changes = 0
    adjusted_trains = 0
    for train_id, option_id in selected.items():
        delay = selected_option_delay_minutes(option_id)
        option = option_lookup.get(option_id)
        changed = (
            option_track_changed(option, baseline_lookup.get(train_id))
            if option
            else False
        )
        delay_by_train[train_id] = delay
        if changed:
            track_changes += 1
        if delay > 0 or changed:
            adjusted_trains += 1
    return delay_by_train, track_changes, adjusted_trains


def unified_objective_key(
    hard_count,
    total_delay,
    max_delay,
    adjusted_trains,
    track_changes,
    tie_breaker=0.0,
):
    return (
        hard_count,
        total_delay,
        max_delay,
        adjusted_trains,
        track_changes,
        tie_breaker,
    )


def unified_objective_value(
    hard_count,
    total_delay,
    max_delay,
    adjusted_trains,
    track_changes,
    tie_breaker=0.0,
):
    weights = CP_OBJECTIVE_WEIGHTS
    return (
        weights["conflict"] * hard_count
        + weights["delay"] * total_delay
        + weights["max_delay"] * max_delay
        + weights["adjusted_train"] * adjusted_trains
        + weights["track_change"] * track_changes
        + tie_breaker
    )


def pair_hard_count(hard_counts, left_option_id, right_option_id):
    return max(
        hard_counts.get(left_option_id, {}).get(right_option_id, 0),
        hard_counts.get(right_option_id, {}).get(left_option_id, 0),
    )


def pair_cost_value(pair_costs_map, left_option_id, right_option_id):
    return max(
        pair_costs_map.get(left_option_id, {}).get(right_option_id, 0.0),
        pair_costs_map.get(right_option_id, {}).get(left_option_id, 0.0),
    )


def option_pair_hard_count(option_a, option_b, hard_counts=None):
    if option_a is None or option_b is None:
        return 0
    details = base.collect_pair_conflict_details(option_a, option_b)
    count = sum(1 for detail in details if detail.conflict_level == "硬冲突")
    if count > 0:
        return count
    if hard_counts is None:
        return 0
    return pair_hard_count(hard_counts, option_a.option_id, option_b.option_id)


def explicit_option_cost(option, baseline_option=None):
    delay = selected_option_delay_minutes(option.option_id)
    track_changed = 1 if option_track_changed(option, baseline_option) else 0
    adjusted = 1 if delay > 0 or track_changed else 0
    weights = CP_OBJECTIVE_WEIGHTS
    return (
        weights["delay"] * delay
        + weights["track_change"] * track_changed
        + weights["adjusted_train"] * adjusted
    )


def apply_selected_option_delays(trains, assignment_ids):
    adjusted = []
    for train in trains:
        delay = selected_option_delay_minutes(
            (assignment_ids or {}).get(train.record_id)
        )
        if delay > 0:
            adjusted.append(
                base.rebuild_train_record(
                    train,
                    arrival_shift=delay,
                    departure_shift=delay,
                )
            )
        else:
            adjusted.append(train)
    return adjusted


def assignment_ids_from_lookup(assignment):
    return {
        train_id: option.option_id
        for train_id, option in (assignment or {}).items()
        if option is not None
    }


def qea_raw_metrics(
    assignment_ids,
    trains,
    reference_trains,
    options_by_train,
    linear_costs,
    pair_costs_map,
    hard_counts,
    hard_risks,
    disturbance_minute,
    frozen_train_ids,
):
    metrics, _, _ = exp.collect_solution_metrics(
        SCHEME_QEA,
        trains,
        reference_trains,
        assignment_ids,
        options_by_train,
        linear_costs,
        pair_costs_map,
        hard_counts,
        hard_risks,
        disturbance_minute=disturbance_minute,
        frozen_train_ids=frozen_train_ids,
        max_safety_wait_rounds=0,
    )
    return metrics


def qea_raw_hard_conflict_edges(
    assignment_ids,
    trains,
    options_by_train,
    disturbance_minute=None,
):
    adjusted_trains = apply_selected_option_delays(trains, assignment_ids)
    assignment = base.build_assignment_lookup(assignment_ids, options_by_train)
    assignment = base.rebuild_assignment_for_trains(adjusted_trains, assignment)
    conflicts = base.collect_conflicts(adjusted_trains, assignment, SCHEME_QEA)
    if disturbance_minute is not None:
        conflicts = exp.filter_conflicts_after_disturbance(
            conflicts, adjusted_trains, disturbance_minute
        )
    edges = []
    for conflict in conflicts:
        if conflict.conflict_level != "硬冲突":
            continue
        left_id = exp.train_id_from_conflict_name(conflict.train1)
        right_id = exp.train_id_from_conflict_name(conflict.train2)
        if left_id in options_by_train and right_id in options_by_train:
            edges.append((left_id, right_id))
    return edges


def build_qea_projection_active_ids(
    assignment_ids,
    trains,
    options_by_train,
    hard_counts,
    disturbance_minute=None,
    cap=QEA_PROJECTION_ACTIVE_CAP,
    time_window=QEA_PROJECTION_TIME_WINDOW,
):
    train_map = {train.record_id: train for train in trains}
    option_lookup = option_lookup_by_id(options_by_train)
    edges = qea_raw_hard_conflict_edges(
        assignment_ids, trains, options_by_train, disturbance_minute
    )
    conflict_degree = {}
    for left_id, right_id in edges:
        conflict_degree[left_id] = conflict_degree.get(left_id, 0) + 1
        conflict_degree[right_id] = conflict_degree.get(right_id, 0) + 1

    seed_ids = sorted(
        conflict_degree,
        key=lambda train_id: (
            -conflict_degree.get(train_id, 0),
            train_map[train_id].arrival_min,
            train_map[train_id].index,
        ),
    )
    active_ids = []
    seen = set()

    def add(train_id):
        if train_id in options_by_train and train_id not in seen:
            active_ids.append(train_id)
            seen.add(train_id)

    for train_id in seed_ids:
        add(train_id)

    selected_options = {
        train_id: option_lookup.get(option_id)
        for train_id, option_id in (assignment_ids or {}).items()
    }
    ordered_trains = sorted(trains, key=lambda train: (train.arrival_min, train.index))
    for seed_id in seed_ids:
        if len(active_ids) >= cap:
            break
        seed_train = train_map[seed_id]
        seed_option = selected_options.get(seed_id)
        for other in ordered_trains:
            if len(active_ids) >= cap:
                break
            other_id = other.record_id
            if other_id in seen or other_id not in options_by_train:
                continue
            other_option = selected_options.get(other_id)
            time_neighbor = (
                abs(other.arrival_min - seed_train.arrival_min) <= time_window
            )
            same_resource = (
                seed_option is not None
                and other_option is not None
                and (
                    seed_option.track == other_option.track
                    or seed_option.in_throat == other_option.in_throat
                    or seed_option.out_throat == other_option.out_throat
                )
            )
            current_hard = 0
            if seed_option is not None and other_option is not None:
                current_hard = pair_hard_count(
                    hard_counts, seed_option.option_id, other_option.option_id
                )
            if time_neighbor or same_resource or current_hard > 0:
                add(other_id)
    return set(active_ids[:cap]), len(edges)


def qea_projection_caps(total_train_count, raw_edge_count):
    if total_train_count <= 1:
        return [1]
    local_cap = min(QEA_PROJECTION_ACTIVE_CAP, total_train_count)
    adaptive_cap = min(
        total_train_count,
        max(
            QEA_PROJECTION_ACTIVE_CAP,
            raw_edge_count * QEA_PROJECTION_EDGE_FACTOR,
            int(total_train_count * QEA_PROJECTION_MAX_RATIO),
        ),
    )
    final_cap = min(total_train_count, max(adaptive_cap, local_cap))
    caps = []
    for cap in (local_cap, adaptive_cap, final_cap):
        cap = max(1, int(cap))
        if cap not in caps:
            caps.append(cap)
    return caps


def run_qea_feasibility_refinement(
    assignment_ids,
    trains,
    reference_trains,
    options_by_train,
    linear_costs,
    pair_costs_map,
    hard_counts,
    hard_risks,
    disturbance_minute,
    frozen_train_ids,
    time_limit_seconds,
):
    before_metrics = qea_raw_metrics(
        assignment_ids,
        trains,
        reference_trains,
        options_by_train,
        linear_costs,
        pair_costs_map,
        hard_counts,
        hard_risks,
        disturbance_minute,
        frozen_train_ids,
    )
    before_hard = int(before_metrics.get("硬冲突项", 0))
    if before_hard == 0:
        return dict(assignment_ids), before_metrics, []

    active_ids, edge_count = build_qea_projection_active_ids(
        assignment_ids,
        trains,
        options_by_train,
        hard_counts,
        disturbance_minute=disturbance_minute,
    )
    if not active_ids:
        return (
            dict(assignment_ids),
            before_metrics,
            [
                {
                    "迭代轮次": "projection",
                    "阶段": "QEA-NS局部可行投影",
                    "硬冲突数": before_hard,
                    "能量值": display_value(before_metrics.get("能量值", "")),
                    "用时(s)": "",
                    "备注": "",
                }
            ],
        )

    best_ids = dict(assignment_ids)
    best_metrics = before_metrics
    history = []
    projection_caps = qea_projection_caps(len(trains), edge_count)
    for attempt_index, cap in enumerate(projection_caps, start=1):
        active_ids, edge_count = build_qea_projection_active_ids(
            best_ids,
            trains,
            options_by_train,
            hard_counts,
            disturbance_minute=disturbance_minute,
            cap=cap,
        )
        if not active_ids:
            continue
        start = time.perf_counter()
        projected_ids, status = exp.solve_subproblem(
            initial_assignment=best_ids,
            active_train_ids=active_ids,
            options_by_train=options_by_train,
            linear_costs=linear_costs,
            pair_costs_map=pair_costs_map,
            hard_counts=hard_counts,
            hard_risks=hard_risks,
            time_limit_seconds=max(2.0, float(time_limit_seconds or 0.0)),
            strict_hard=True,
            max_options_per_train=40,
            preserve_active_scope=True,
            seed_deviation_penalty=QEA_PROJECTION_DEVIATION_PENALTY,
        )
        elapsed = time.perf_counter() - start
        if projected_ids is None:
            history.append(
                {
                    "迭代轮次": f"projection-{attempt_index}",
                    "阶段": "QEA-NS局部可行投影",
                    "硬冲突数": best_metrics.get("硬冲突项", before_hard),
                    "能量值": display_value(best_metrics.get("能量值", "")),
                    "用时(s)": f"{elapsed:.3f}",
                    "备注": "",
                }
            )
            continue
        after_metrics = qea_raw_metrics(
            projected_ids,
            trains,
            reference_trains,
            options_by_train,
            linear_costs,
            pair_costs_map,
            hard_counts,
            hard_risks,
            disturbance_minute,
            frozen_train_ids,
        )
        accepted = metrics_prefer_key(after_metrics) < metrics_prefer_key(best_metrics)
        history.append(
            {
                "迭代轮次": f"projection-{attempt_index}",
                "阶段": "QEA-NS局部可行投影",
                "硬冲突数": after_metrics.get("硬冲突项", before_hard),
                "能量值": display_value(after_metrics.get("能量值", "")),
                "用时(s)": f"{elapsed:.3f}",
                "备注": "",
            }
        )
        if accepted:
            best_ids = projected_ids
            best_metrics = after_metrics
            if int(best_metrics.get("硬冲突项", 0)) == 0:
                break
    if history:
        return best_ids, best_metrics, history
    return dict(assignment_ids), before_metrics, history


_ORIGINAL_ITERATIVE_SAFETY_WAIT_RECOVERY = exp.iterative_safety_wait_recovery


def iterative_safety_wait_recovery_with_option_delays(
    reference_trains,
    disturbed_trains,
    disturbed_assignment,
    disturbance_minute,
    options_by_train=None,
    frozen_train_ids=None,
    max_rounds=160,
):
    assignment_ids = assignment_ids_from_lookup(disturbed_assignment)
    adjusted_trains = apply_selected_option_delays(disturbed_trains, assignment_ids)
    adjusted_assignment = base.rebuild_assignment_for_trains(
        adjusted_trains, disturbed_assignment
    )
    if max_rounds is not None and int(max_rounds) <= 0:
        conflicts = exp.filter_conflicts_after_disturbance(
            base.collect_conflicts(
                adjusted_trains, adjusted_assignment, SCHEME_DISTURBED
            ),
            adjusted_trains,
            disturbance_minute,
        )
        return adjusted_trains, adjusted_assignment, conflicts, {}, 0
    return _ORIGINAL_ITERATIVE_SAFETY_WAIT_RECOVERY(
        reference_trains=reference_trains,
        disturbed_trains=adjusted_trains,
        disturbed_assignment=adjusted_assignment,
        options_by_train=options_by_train,
        disturbance_minute=disturbance_minute,
        frozen_train_ids=frozen_train_ids,
        max_rounds=max_rounds,
    )


def realize_assignment_with_option_delays(
    trains,
    assignment_ids,
    options_by_train,
    disturbance_minute,
    reference_trains,
    frozen_train_ids,
    max_safety_wait_rounds=160,
):
    adjusted_trains = apply_selected_option_delays(trains, assignment_ids)
    assignment = base.build_assignment_lookup(assignment_ids, options_by_train)
    assignment = base.rebuild_assignment_for_trains(adjusted_trains, assignment)
    conflicts = exp.filter_conflicts_after_disturbance(
        base.collect_conflicts(adjusted_trains, assignment, "方案落地检查"),
        adjusted_trains,
        disturbance_minute,
    )
    hard_conflicts = [
        conflict for conflict in conflicts if conflict.conflict_level == "硬冲突"
    ]
    if not hard_conflicts or int(max_safety_wait_rounds) <= 0:
        return adjusted_trains, assignment, conflicts, {}
    return _ORIGINAL_ITERATIVE_SAFETY_WAIT_RECOVERY(
        reference_trains=reference_trains,
        disturbed_trains=adjusted_trains,
        disturbed_assignment=assignment,
        options_by_train=options_by_train,
        disturbance_minute=disturbance_minute,
        frozen_train_ids=frozen_train_ids,
        max_rounds=max_safety_wait_rounds,
    )[:4]


setattr(
    exp,
    "iterative_safety_wait_recovery",
    iterative_safety_wait_recovery_with_option_delays,
)
setattr(
    exp, "realize_assignment_with_frozen_wait", realize_assignment_with_option_delays
)


def solve_subproblem_explicit_objective(
    initial_assignment,
    active_train_ids,
    options_by_train,
    linear_costs,
    pair_costs_map,
    hard_counts,
    hard_risks,
    time_limit_seconds,
    strict_hard=True,
    max_options_per_train=MAX_OPTIONS_PER_TRAIN,
    solver_seed=None,
    seed_deviation_penalty=0,
):
    cp_model = exp.import_cp_model()
    if cp_model is None:
        return None, "ORTools不可用"

    selected = dict(initial_assignment or {})
    active_train_ids = {
        train_id for train_id in active_train_ids if train_id in options_by_train
    }
    if not active_train_ids:
        return dict(selected), "空模型"

    limit = max(1, int(max_options_per_train or 40))
    limited_options_by_train = limit_options_for_pair_maps(
        options_by_train,
        limit=limit,
        keep_ids=selected,
    )
    model = cp_model.CpModel()
    variables = {}
    objective_terms = []
    max_delay_var = model.NewIntVar(0, max(DELAY_CHOICES), "max_delay")
    objective_reference_ids = OBJECTIVE_REFERENCE_ASSIGNMENT_IDS or selected
    baseline_option_lookup = {
        train_id: next(
            (
                option
                for option in options_by_train.get(train_id, [])
                if option.option_id == option_id
            ),
            None,
        )
        for train_id, option_id in objective_reference_ids.items()
    }

    for train_id in sorted(active_train_ids):
        train_vars = []
        baseline_option = baseline_option_lookup.get(train_id)
        for option in limited_options_by_train[train_id]:
            var = model.NewBoolVar(option.option_id)
            variables[option.option_id] = var
            train_vars.append(var)
            delay = selected_option_delay_minutes(option.option_id)
            model.Add(max_delay_var >= delay).OnlyEnforceIf(var)
            cost = explicit_option_cost(option, baseline_option)
            if cost:
                objective_terms.append(cost * var)
            if (
                seed_deviation_penalty
                and train_id in selected
                and selected.get(train_id) != option.option_id
            ):
                objective_terms.append(int(seed_deviation_penalty) * var)
            if selected.get(train_id) == option.option_id:
                model.AddHint(var, 1)
            elif train_id in selected:
                model.AddHint(var, 0)
        model.AddExactlyOne(train_vars)

    fixed_selected = {
        train_id: option_id
        for train_id, option_id in selected.items()
        if train_id in options_by_train and train_id not in active_train_ids
    }
    hard_weight = CP_OBJECTIVE_WEIGHTS["conflict"]
    pair_weight = 0
    active_order = sorted(active_train_ids)
    if strict_hard:

        def macro_entries(train_id, option, var=None):
            for resource in option.resources:
                if resource.stage == "C" and resource.resource_category == "track":
                    yield (
                        ("C", resource.track),
                        train_id,
                        option,
                        var,
                        resource.start_min,
                        resource.end_min,
                    )
                elif resource.stage == "A-C" and resource.throat:
                    yield (
                        ("A-C", resource.throat),
                        train_id,
                        option,
                        var,
                        resource.start_min,
                        resource.end_min,
                    )
                elif resource.stage == "C-E" and resource.throat:
                    yield (
                        ("C-E", resource.throat),
                        train_id,
                        option,
                        var,
                        resource.start_min,
                        resource.end_min,
                    )

        active_groups = {}
        for train_id in active_order:
            for option in limited_options_by_train[train_id]:
                for entry in macro_entries(
                    train_id, option, variables[option.option_id]
                ):
                    active_groups.setdefault(entry[0], []).append(entry[1:])
        fixed_groups = {}
        for train_id, option_id in fixed_selected.items():
            fixed_option = next(
                (
                    option
                    for option in options_by_train.get(train_id, [])
                    if option.option_id == option_id
                ),
                None,
            )
            if fixed_option is None:
                continue
            for entry in macro_entries(train_id, fixed_option, None):
                fixed_groups.setdefault(entry[0], []).append(entry[1:])

        direct_seen = set()
        for key, entries in active_groups.items():
            for left_index, left_entry in enumerate(entries):
                left_tid, left_option, left_var, left_start, left_end = left_entry
                for right_entry in entries[left_index + 1 :]:
                    right_tid, right_option, right_var, right_start, right_end = (
                        right_entry
                    )
                    if left_tid == right_tid:
                        continue
                    if min(left_end, right_end) <= max(left_start, right_start):
                        continue
                    pair_key = tuple(
                        sorted((left_option.option_id, right_option.option_id))
                    )
                    if pair_key in direct_seen:
                        continue
                    direct_seen.add(pair_key)
                    if (
                        option_pair_hard_count(left_option, right_option, hard_counts)
                        > 0
                    ):
                        model.AddBoolOr([left_var.Not(), right_var.Not()])
            for active_entry in entries:
                active_tid, active_option, active_var, active_start, active_end = (
                    active_entry
                )
                for fixed_entry in fixed_groups.get(key, []):
                    fixed_tid, fixed_option, _, fixed_start, fixed_end = fixed_entry
                    if active_tid == fixed_tid:
                        continue
                    if min(active_end, fixed_end) <= max(active_start, fixed_start):
                        continue
                    pair_key = tuple(
                        sorted((active_option.option_id, fixed_option.option_id))
                    )
                    if pair_key in direct_seen:
                        continue
                    direct_seen.add(pair_key)
                    if (
                        option_pair_hard_count(active_option, fixed_option, hard_counts)
                        > 0
                    ):
                        model.Add(active_var == 0)

    for left_index, left_tid in enumerate(active_order):
        for right_tid in active_order[left_index + 1 :]:
            for left_option in limited_options_by_train[left_tid]:
                left_var = variables[left_option.option_id]
                for right_option in limited_options_by_train[right_tid]:
                    right_var = variables[right_option.option_id]
                    hard_count = pair_hard_count(
                        hard_counts, left_option.option_id, right_option.option_id
                    )
                    if strict_hard and hard_count > 0:
                        model.AddBoolOr([left_var.Not(), right_var.Not()])
                        continue
                    pair_cost_amount = int(
                        round(
                            pair_cost_value(
                                pair_costs_map,
                                left_option.option_id,
                                right_option.option_id,
                            )
                        )
                    )
                    pair_penalty = hard_weight * hard_count + pair_weight * max(
                        0, pair_cost_amount
                    )
                    if pair_penalty <= 0:
                        continue
                    both = model.NewBoolVar(
                        f"pair_{left_option.option_id}_{right_option.option_id}"
                    )
                    model.Add(both <= left_var)
                    model.Add(both <= right_var)
                    model.Add(both >= left_var + right_var - 1)
                    objective_terms.append(pair_penalty * both)

    for train_id in active_order:
        for option in limited_options_by_train[train_id]:
            var = variables[option.option_id]
            for fixed_tid, fixed_option_id in fixed_selected.items():
                if fixed_tid == train_id:
                    continue
                hard_count = pair_hard_count(
                    hard_counts, option.option_id, fixed_option_id
                )
                if strict_hard and hard_count > 0:
                    model.Add(var == 0)
                    continue
                pair_cost_amount = int(
                    round(
                        pair_cost_value(
                            pair_costs_map, option.option_id, fixed_option_id
                        )
                    )
                )
                fixed_penalty = hard_weight * hard_count + pair_weight * max(
                    0, pair_cost_amount
                )
                if fixed_penalty > 0:
                    objective_terms.append(fixed_penalty * var)

    objective_terms.append(CP_OBJECTIVE_WEIGHTS["max_delay"] * max_delay_var)
    model.Minimize(sum(objective_terms) if objective_terms else 0)
    solver = cp_model.CpSolver()
    if time_limit_seconds is not None and float(time_limit_seconds) > 0:
        solver.parameters.max_time_in_seconds = max(0.5, float(time_limit_seconds))
    solver.parameters.num_search_workers = 1
    solver.parameters.random_seed = int(
        SUBPROBLEM_SEED if solver_seed is None else solver_seed
    )
    if hasattr(solver.parameters, "randomize_search"):
        solver.parameters.randomize_search = False
    status = solver.Solve(model)
    status_name = solver.StatusName(status)
    if status not in {cp_model.OPTIMAL, cp_model.FEASIBLE}:
        return None, status_name

    solved = dict(selected)
    for train_id in active_train_ids:
        for option in limited_options_by_train[train_id]:
            if solver.BooleanValue(variables[option.option_id]):
                solved[train_id] = option.option_id
                break
    return solved, status_name


def solve_subproblem_with_delay_options(*args, **kwargs):
    preserve_active_scope = bool(kwargs.pop("preserve_active_scope", False))
    if "max_options_per_train" in kwargs:
        kwargs["max_options_per_train"] = max(
            int(kwargs["max_options_per_train"] or 0), MAX_OPTIONS_PER_TRAIN
        )
    else:
        kwargs["max_options_per_train"] = MAX_OPTIONS_PER_TRAIN
    options = kwargs.get("options_by_train")
    active_ids = set(kwargs.get("active_train_ids") or set())
    if (
        not preserve_active_scope
        and options is not None
        and len(active_ids) < ACTIVE_TARGET_SIZE
    ):
        variable_ids = [
            train_id
            for train_id, train_options in options.items()
            if len(train_options) > 1
        ]
        expanded_ids = list(dict.fromkeys([*sorted(active_ids), *sorted(variable_ids)]))
        kwargs["active_train_ids"] = set(expanded_ids[:ACTIVE_TARGET_SIZE])
    hard_kwargs = dict(kwargs)
    hard_kwargs["strict_hard"] = True
    result, status = solve_subproblem_explicit_objective(*args, **hard_kwargs)
    if result is not None:
        return result, status
    fallback_kwargs = dict(hard_kwargs)
    fallback_kwargs["strict_hard"] = False
    fallback_result, fallback_status = solve_subproblem_explicit_objective(
        *args, **fallback_kwargs
    )
    return fallback_result, f"{status}->soft:{fallback_status}"


setattr(exp, "solve_subproblem", solve_subproblem_with_delay_options)


def parse_args():
    parser = argparse.ArgumentParser(description="法兰克福中央车站 QEA-NS vs CP-SAT实验")
    parser.add_argument(
        "--input", type=str, default="frankfurt_hbf_gtfs_schedule.csv"
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="frankfurt_hbf_qea_ns_vs_cp_sat_benchmark",
    )
    parser.add_argument("--max-route-candidates", type=int, default=5)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--qea-time-limit", type=float, default=400.0)
    parser.add_argument("--subproblem-time-limit", type=float, default=120.0)
    parser.add_argument(
        "--cp-sat-time-limit",
        dest="cp_sat_time_limit",
        type=float,
        default=400.0,
    )
    parser.add_argument("--cp-sat-solver", type=str, default="CP-SAT")
    parser.add_argument("--qea-pop-size", type=int, default=50)
    parser.add_argument("--qea-max-generations", type=int, default=500)
    parser.add_argument("--qea-candidate-count", type=int, default=1)
    parser.add_argument("--qea-neighborhood-restarts", type=int, default=1)
    parser.add_argument("--safety-wait-rounds", type=int, default=160)
    parser.add_argument("--short-case-train-limit", type=int, default=0)
    parser.add_argument("--source-train-count", type=int, default=0)
    return parser.parse_args()


def display_value(value):
    return f"{value:.2f}" if isinstance(value, float) else value


def empty_metrics(label):
    return {
        "方法": label,
        "总冲突项": "--",
        "硬冲突项": "--",
        "进站咽喉冲突项": "--",
        "股道占用冲突项": "--",
        "出站咽喉冲突项": "--",
        "总晚点时长": "--",
        "平均晚点时长": "--",
        "最大晚点时长": "--",
        "能量值": "--",
        "顺延调整列车数": "--",
    }


def normalize_qea_history(history_rows):
    normalized = []
    for row in history_rows:
        updated = {key: value for key, value in row.items() if key != "备注"}
        normalized.append({"方法": SCHEME_QEA, **updated})
    return normalized


def qea_neighborhood_active_ids(
    assignment_ids,
    options_by_train,
    trains,
    reference_trains,
    limit=120,
):
    train_by_id = {train.record_id: train for train in trains}
    reference_delay = base.compute_delay_statistics(reference_trains, trains)[
        "delay_by_train"
    ]
    _, _, _ = selected_adjustment_components(
        assignment_ids,
        options_by_train,
        OBJECTIVE_REFERENCE_ASSIGNMENT_IDS or {},
    )
    ranked_delay_ids = sorted(
        assignment_ids,
        key=lambda train_id: (
            -(
                reference_delay.get(train_id, 0)
                + selected_option_delay_minutes(assignment_ids[train_id])
            ),
            train_by_id[train_id].arrival_min,
            train_by_id[train_id].index,
        ),
    )
    active_ids = []
    seen = set()

    def add(train_id):
        if train_id in options_by_train and train_id not in seen:
            active_ids.append(train_id)
            seen.add(train_id)

    for train_id in ranked_delay_ids:
        total_delay = reference_delay.get(train_id, 0) + selected_option_delay_minutes(
            assignment_ids[train_id]
        )
        if total_delay <= 0:
            continue
        add(train_id)
        train = train_by_id[train_id]
        for other in trains:
            if abs(other.arrival_min - train.arrival_min) <= 25:
                add(other.record_id)
            if len(active_ids) >= limit:
                break
        if len(active_ids) >= limit:
            break
    return set(active_ids[:limit])


def delay_release_conflict_chain_active_ids(
    assignment_ids,
    options_by_train,
    trains,
    reference_trains,
    hard_counts,
    limit=120,
    seed_limit=12,
    chain_depth=2,
):
    train_by_id = {train.record_id: train for train in trains}
    current_options = dict(assignment_ids)
    reference_delay = base.compute_delay_statistics(reference_trains, trains)[
        "delay_by_train"
    ]
    active_ids = []
    seen = set()
    queue = deque()

    def add(train_id, depth):
        if train_id not in options_by_train or train_id in seen:
            return
        seen.add(train_id)
        active_ids.append(train_id)
        if depth < chain_depth:
            queue.append((train_id, depth))

    def conflicts_with_current(option_id, train_id):
        row = hard_counts.get(option_id, {})
        conflicts = []
        for other_train_id, other_option_id in current_options.items():
            if other_train_id == train_id:
                continue
            if row.get(other_option_id, 0) > 0:
                conflicts.append(other_train_id)
                continue
            if hard_counts.get(other_option_id, {}).get(option_id, 0) > 0:
                conflicts.append(other_train_id)
        return conflicts

    seed_ids = sorted(
        current_options,
        key=lambda train_id: (
            -selected_option_delay_minutes(current_options[train_id]),
            -(
                reference_delay.get(train_id, 0)
                + selected_option_delay_minutes(current_options[train_id])
            ),
            train_by_id[train_id].arrival_min,
            train_by_id[train_id].index,
        ),
    )
    for train_id in seed_ids:
        if selected_option_delay_minutes(current_options[train_id]) <= 0:
            continue
        add(train_id, 0)
        if len(active_ids) >= seed_limit:
            break

    while queue and len(active_ids) < limit:
        train_id, depth = queue.popleft()
        current_delay = selected_option_delay_minutes(current_options[train_id])
        if current_delay <= 0:
            continue
        lower_options = sorted(
            (
                option
                for option in options_by_train[train_id]
                if selected_option_delay_minutes(option.option_id) < current_delay
            ),
            key=lambda option: (
                -selected_option_delay_minutes(option.option_id),
                option.candidate_rank,
                option.option_id,
            ),
        )[:16]
        for option in lower_options:
            for conflict_train_id in conflicts_with_current(option.option_id, train_id):
                add(conflict_train_id, depth + 1)
                if len(active_ids) >= limit:
                    break
            if len(active_ids) >= limit:
                break

    if len(active_ids) < limit:
        for train_id in seed_ids:
            if train_id in seen:
                continue
            total_delay = reference_delay.get(
                train_id, 0
            ) + selected_option_delay_minutes(current_options[train_id])
            if total_delay <= 0:
                continue
            train = train_by_id[train_id]
            add(train_id, chain_depth)
            for other in trains:
                if abs(other.arrival_min - train.arrival_min) <= 20:
                    add(other.record_id, chain_depth)
                if len(active_ids) >= limit:
                    break
            if len(active_ids) >= limit:
                break
    return set(active_ids[:limit])


def run_qea_neighborhood_refinement(
    assignment_ids,
    metrics,
    trains,
    reference_trains,
    options_by_train,
    linear_costs,
    pair_costs_map,
    hard_counts,
    hard_risks,
    disturbance_minute,
    frozen_train_ids,
    time_limit_seconds,
    rounds=5,
    stage_label="QEA-NS邻域精修",
    restarts=1,
):
    archive_ids = dict(assignment_ids)
    archive_metrics = dict(metrics)
    history_rows = []
    total_train_count = len(options_by_train)
    local_active_cap = max(
        1, min(NEIGHBORHOOD_ACTIVE_CAP, max(1, total_train_count - 1))
    )
    for restart_index in range(max(1, restarts)):
        best_ids = dict(assignment_ids)
        best_metrics = dict(metrics)
        for round_index in range(1, rounds + 1):
            active_limit = min(
                local_active_cap,
                max(
                    1,
                    NEIGHBORHOOD_ACTIVE_START
                    + NEIGHBORHOOD_ACTIVE_STEP * (round_index - 1),
                ),
            )
            active_ids = delay_release_conflict_chain_active_ids(
                best_ids,
                options_by_train,
                trains,
                reference_trains,
                hard_counts,
                limit=active_limit,
            )
            if not active_ids:
                active_ids = qea_neighborhood_active_ids(
                    best_ids,
                    options_by_train,
                    trains,
                    reference_trains,
                    limit=active_limit,
                )
            if not active_ids:
                break
            start = time.perf_counter()
            candidate_ids, status = exp.solve_subproblem(
                initial_assignment=best_ids,
                active_train_ids=active_ids,
                options_by_train=options_by_train,
                linear_costs=linear_costs,
                pair_costs_map=pair_costs_map,
                hard_counts=hard_counts,
                hard_risks=hard_risks,
                time_limit_seconds=time_limit_seconds,
                strict_hard=True,
                max_options_per_train=MAX_OPTIONS_PER_TRAIN,
                preserve_active_scope=True,
                solver_seed=SUBPROBLEM_SEED + restart_index * 1000 + round_index,
            )
            elapsed = time.perf_counter() - start
            iteration_label = (
                f"refine-{round_index}"
                if restarts <= 1
                else f"refine-r{restart_index + 1}-{round_index}"
            )
            if candidate_ids is None:
                history_rows.append(
                    {
                        "迭代轮次": iteration_label,
                        "阶段": stage_label,
                        "硬冲突数": best_metrics["硬冲突项"],
                        "能量值": display_value(best_metrics["能量值"]),
                        "用时(s)": f"{elapsed:.3f}",
                        "备注": "",
                    }
                )
                continue
            candidate_metrics, _, _ = exp.collect_solution_metrics(
                SCHEME_QEA,
                trains,
                reference_trains,
                candidate_ids,
                options_by_train,
                linear_costs,
                pair_costs_map,
                hard_counts,
                hard_risks,
                disturbance_minute=disturbance_minute,
                frozen_train_ids=frozen_train_ids,
                max_safety_wait_rounds=0,
            )
            accepted = metrics_prefer_key(candidate_metrics) < metrics_prefer_key(
                best_metrics
            )
            archive_accepted = metrics_prefer_key(
                candidate_metrics
            ) < metrics_prefer_key(archive_metrics)
            history_rows.append(
                {
                    "迭代轮次": iteration_label,
                    "阶段": stage_label,
                    "硬冲突数": candidate_metrics["硬冲突项"],
                    "能量值": display_value(candidate_metrics["能量值"]),
                    "用时(s)": f"{elapsed:.3f}",
                    "备注": "",
                }
            )
            if accepted:
                best_ids = candidate_ids
                best_metrics = candidate_metrics
            if archive_accepted:
                archive_ids = candidate_ids
                archive_metrics = candidate_metrics
    return archive_ids, archive_metrics, history_rows


def write_excel(path, comparison_rows, performance_rows, history_rows):
    if not getattr(exp, "EXCEL_AVAILABLE", False):
        return False
    wb = exp.openpyxl.Workbook()
    header_font = exp.Font(bold=True)
    fill = exp.PatternFill(start_color="DDEBF7", end_color="DDEBF7", fill_type="solid")
    border = exp.Border(
        left=exp.Side(style="thin"),
        right=exp.Side(style="thin"),
        top=exp.Side(style="thin"),
        bottom=exp.Side(style="thin"),
    )
    sheets = [
        ("方案对比", comparison_rows),
        ("求解器性能", performance_rows),
        ("QEA-NS迭代记录", history_rows),
    ]
    for sheet_name, rows in sheets:
        ws = wb.active if wb.active.title == "Sheet" else wb.create_sheet(sheet_name)
        ws.title = sheet_name
        if not rows:
            continue
        headers = list(rows[0])
        for col, header in enumerate(headers, 1):
            cell = ws.cell(row=1, column=col, value=header)
            cell.font = header_font
            cell.fill = fill
            cell.border = border
            cell.alignment = exp.Alignment(horizontal="center", wrap_text=True)
        for row_index, row in enumerate(rows, 2):
            for col, header in enumerate(headers, 1):
                cell = ws.cell(row=row_index, column=col, value=row.get(header, ""))
                cell.border = border
                cell.alignment = exp.Alignment(horizontal="center")
        for col in range(1, len(headers) + 1):
            ws.column_dimensions[exp.get_column_letter(col)].width = 18
    wb.save(path)
    return True


def main():
    args = parse_args()
    if args.source_train_count:
        if args.source_train_count < 1:
            raise ValueError("--source-train-count 必须为正整数")
        set_source_train_count_override(args.source_train_count)
    base_dir = Path(__file__).resolve().parent
    input_path = resolve_input_path(args.input, base_dir)
    out_dir = Path(args.output_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    encoding = base.detect_encoding(input_path)
    trains = base.load_trains(input_path, encoding)
    if args.short_case_train_limit and args.short_case_train_limit > 0:
        trains = sorted(
            trains,
            key=lambda train: (train.arrival_min, train.departure_min, train.index),
        )[: max(1, args.short_case_train_limit)]
    library = base.build_route_library(trains)

    print("=" * 64, flush=True)
    print(f"{STATION_NAME} QEA-NS vs CP-SAT实验", flush=True)
    print("=" * 64, flush=True)
    print(f"输入文件：{input_path.name}", flush=True)
    print(f"列车数量：{len(trains)}", flush=True)
    if args.short_case_train_limit and args.short_case_train_limit > 0:
        print(
            f"短案例模式：仅使用前{len(trains)}列车验证流程，不作为正式实验结果",
            flush=True,
        )
    print(
        f"建模股道数量：{len(library['all_tracks_sorted'])}（{','.join(library['all_tracks_sorted'])}）",
        flush=True,
    )

    pressure_ids = base.build_high_pressure_train_ids(trains, limit=24)
    plan_options, _ = base.build_candidate_route_plans(
        trains=trains,
        library=library,
        max_route_candidates=args.max_route_candidates,
        rich_candidate_train_ids=pressure_ids,
    )
    plan_assignment = base.build_plan_assignment(trains, plan_options)
    plan_ids = base.project_assignment_to_option_ids(
        trains, plan_options, plan_assignment
    )
    plan_metrics, _, _ = exp.collect_raw_solution_metrics(
        SCHEME_PLAN,
        trains,
        trains,
        plan_ids,
        plan_options,
    )
    print(
        f"原计划复核：总冲突={plan_metrics['总冲突项']}，硬冲突={plan_metrics['硬冲突项']}，总晚点={plan_metrics['总晚点时长']}分",
        flush=True,
    )

    scenario, _ = base.generate_disturbance_scenario(trains, args.seed)
    disturbance_time_text = ";".join(
        base.format_minutes_as_clock(event.disturbance_minute)
        for event in scenario.disturbance_events
    ) or base.format_minutes_as_clock(scenario.disturbance_minute)
    source_train_ids = list(scenario.source_train_ids)
    source_train_score_text = ";".join(
        f"{train_id}:{float(scenario.source_train_scores.get(train_id, 0.0)):.6f}"
        for train_id in source_train_ids
    )
    print(
        f"扰动时刻：{disturbance_time_text}，"
        f"事件数={len(scenario.disturbance_events)}，源头列车={source_train_ids}",
        flush=True,
    )
    disturbed_trains, _, _, propagation_rounds = base.simulate_scenario_for_assignment(
        reference_trains=trains,
        assignment=plan_assignment,
        scenario=scenario,
        library=library,
        max_route_candidates=args.max_route_candidates,
    )
    scenario = base.scenario_with_propagation_rounds(scenario, propagation_rounds)
    pre_restore_delay = base.compute_delay_statistics(trains, disturbed_trains)
    print(
        f"扰动传播轮数：{propagation_rounds}，扰动后总晚点(还原前)：{pre_restore_delay['总晚点时长']}分，"
        f"受影响列车={pre_restore_delay.get('受影响列车数', 0)}",
        flush=True,
    )

    raw_frozen = exp.train_ids_fixed_before_disturbance(
        trains, scenario.disturbance_minute
    )
    frozen_train_ids = {
        train.record_id
        for train in trains
        if train.window_end <= scenario.disturbance_minute - 30
        and train.record_id not in set(source_train_ids)
    }
    print(
        f"原始冻结候选数：{len(raw_frozen)}，剔除源头后真冻结数：{len(frozen_train_ids)}",
        flush=True,
    )

    disturbed_plan_assignment = base.rebuild_assignment_for_trains(
        disturbed_trains, plan_assignment
    )
    disturbed_trains, disturbed_plan_assignment = exp.restore_pre_disturbance_plan(
        reference_trains=trains,
        current_trains=disturbed_trains,
        current_assignment=disturbed_plan_assignment,
        plan_assignment=plan_assignment,
        frozen_train_ids=frozen_train_ids,
    )
    disturbed_plan_assignment = base.rebuild_assignment_for_trains(
        disturbed_trains, plan_assignment
    )

    rich_ids = {
        train_id
        for train_id, delay in base.compute_delay_statistics(trains, disturbed_trains)[
            "delay_by_train"
        ].items()
        if delay > 0
    }
    (
        options_by_train,
        train_map,
        linear_costs,
        pair_costs_map,
        hard_counts,
        hard_risks,
    ) = exp.build_problem(
        disturbed_trains,
        library,
        max(args.max_route_candidates, 8),
        rich_ids=rich_ids,
        fixed_assignment=disturbed_plan_assignment,
        frozen_train_ids=frozen_train_ids,
        disturbance_minute=scenario.disturbance_minute,
    )

    baseline_assignment = base.rebuild_assignment_for_trains(
        disturbed_trains, plan_assignment
    )
    planned_throat_by_train = {
        train.record_id: (
            str(train.in_throat or "").strip(),
            str(train.out_throat or "").strip(),
        )
        for train in trains
    }
    baseline_ids = base.project_assignment_to_option_ids(
        disturbed_trains, options_by_train, baseline_assignment
    )
    (
        options_by_train,
        throat_consistency_removed_count,
        throat_consistency_fallback_count,
    ) = filter_planned_throat_options(
        options_by_train, planned_throat_by_train, baseline_ids
    )
    baseline_ids = {
        train_id: option_id
        if any(option.option_id == option_id for option in options_by_train[train_id])
        else options_by_train[train_id][0].option_id
        for train_id, option_id in baseline_ids.items()
    }
    if OPTIMIZE_ALL_TRAINS:
        active_train_ids = {train.record_id for train in disturbed_trains}
        active_info = {
            "seed_count": len(source_train_ids),
            "delayed_count": sum(
                1
                for delay in base.compute_delay_statistics(trains, disturbed_trains)[
                    "delay_by_train"
                ].values()
                if delay > 0
            ),
            "hard_neighbor_count": len(active_train_ids),
            "target_size": len(active_train_ids),
        }
    else:
        active_train_ids, active_info = expand_active_train_ids(
            reference_trains=trains,
            disturbed_trains=disturbed_trains,
            baseline_assignment=baseline_assignment,
            source_train_ids=source_train_ids,
            disturbance_minute=scenario.disturbance_minute,
        )
        options_by_train = exp.restrict_options_to_active_trains(
            options_by_train,
            baseline_ids,
            active_train_ids,
        )
        baseline_ids = {
            train_id: options_by_train[train_id][0].option_id
            if train_id not in active_train_ids
            else baseline_ids[train_id]
            for train_id in baseline_ids
        }
    global OBJECTIVE_OPTIONS_BY_TRAIN
    global OBJECTIVE_REFERENCE_ASSIGNMENT_IDS
    global OBJECTIVE_REFERENCE_DELAY_BY_TRAIN
    OBJECTIVE_REFERENCE_ASSIGNMENT_IDS = dict(baseline_ids)
    options_by_train = expand_options_with_delay_choices(
        disturbed_trains,
        options_by_train,
        active_train_ids,
    )
    options_by_train, delay_throat_removed_count, delay_throat_fallback_count = (
        filter_planned_throat_options(
            options_by_train, planned_throat_by_train, baseline_ids
        )
    )
    throat_consistency_removed_count += delay_throat_removed_count
    throat_consistency_fallback_count += delay_throat_fallback_count
    options_by_train = limit_options_for_pair_maps(
        options_by_train, keep_ids=baseline_ids
    )
    OBJECTIVE_OPTIONS_BY_TRAIN = options_by_train
    OBJECTIVE_REFERENCE_DELAY_BY_TRAIN = base.compute_delay_statistics(
        trains, disturbed_trains
    )["delay_by_train"]
    delay_candidate_count = sum(
        1
        for train_id in active_train_ids
        for option in options_by_train.get(train_id, [])
        if selected_option_delay_minutes(option.option_id) > 0
    )
    active_option_count = sum(
        len(options_by_train.get(train_id, [])) for train_id in active_train_ids
    )
    pair_screen_trains = trains_for_delay_pair_screen(
        disturbed_trains, active_train_ids, max(DELAY_CHOICES)
    )
    raw_pair_costs, hard_counts, hard_risks = base.build_pairwise_conflict_maps(
        trains=pair_screen_trains,
        options_by_train=options_by_train,
        mode="full",
        hard_soft_enabled=True,
        use_variant_penalty=False,
    )
    pair_costs_map = base.build_weighted_pair_costs(
        raw_pair_costs,
        base.DEFAULT_MODEL_WEIGHTS["safety_weight"],
    )
    linear_costs = base.build_weighted_linear_costs(
        options_by_train,
        base.DEFAULT_MODEL_WEIGHTS["safety_weight"],
        base.DEFAULT_MODEL_WEIGHTS["stability_weight"],
    )
    if not OPTIMIZE_ALL_TRAINS:
        exp.prune_pre_disturbance_pair_maps(
            disturbed_trains,
            options_by_train,
            pair_costs_map,
            hard_counts,
            hard_risks,
            scenario.disturbance_minute,
        )
    print(
        f"优化变量列车数：{len(active_train_ids)}，固定列车数：{len(options_by_train) - len(active_train_ids)}",
        flush=True,
    )
    print(
        f"active扩展：seed={active_info['seed_count']}，晚点={active_info['delayed_count']}，硬冲突邻域={active_info['hard_neighbor_count']}，目标={active_info['target_size']}",
        flush=True,
    )
    print(
        f"active候选总数={active_option_count}，其中顺延候选={delay_candidate_count}",
        flush=True,
    )
    print(
        f"计划咽喉一致性：剔除候选={throat_consistency_removed_count}，备选列车={throat_consistency_fallback_count}",
        flush=True,
    )
    baseline_metrics, _, _ = exp.collect_raw_solution_metrics(
        SCHEME_DISTURBED,
        disturbed_trains,
        trains,
        baseline_ids,
        options_by_train,
        linear_costs,
        pair_costs_map,
        hard_counts,
        hard_risks,
    )
    baseline_landed_metrics, _, _ = exp.collect_solution_metrics(
        SCHEME_DISTURBED,
        disturbed_trains,
        trains,
        baseline_ids,
        options_by_train,
        linear_costs,
        pair_costs_map,
        hard_counts,
        hard_risks,
        disturbance_minute=scenario.disturbance_minute,
        frozen_train_ids=frozen_train_ids,
        max_safety_wait_rounds=args.safety_wait_rounds,
    )
    for delay_metric in (
        "总晚点时长",
        "平均晚点时长",
        "最大晚点时长",
        "顺延调整列车数",
    ):
        baseline_metrics[delay_metric] = baseline_landed_metrics[delay_metric]
    print(
        f"扰动基准：总冲突={baseline_metrics['总冲突项']}，硬冲突={baseline_metrics['硬冲突项']}，总晚点={baseline_metrics['总晚点时长']}分",
        flush=True,
    )

    cp_sat_solver = "CP-SAT"
    print("=== CP-SAT 求解 ===", flush=True)
    cp_sat_start = time.perf_counter()
    cp_sat_ids, cp_sat_status = exp.solve_subproblem(
        initial_assignment=baseline_ids,
        active_train_ids=active_train_ids,
        options_by_train=options_by_train,
        linear_costs=linear_costs,
        pair_costs_map=pair_costs_map,
        hard_counts=hard_counts,
        hard_risks=hard_risks,
        time_limit_seconds=args.cp_sat_time_limit,
        strict_hard=True,
        max_options_per_train=40,
        solver_seed=SUBPROBLEM_SEED + args.seed,
    )
    cp_sat_time = time.perf_counter() - cp_sat_start
    cp_sat_solved = cp_sat_ids is not None
    if cp_sat_solved:
        cp_sat_metrics, cp_sat_assignment, cp_sat_conflicts = exp.collect_solution_metrics(
            SCHEME_CP_SAT,
            disturbed_trains,
            trains,
            cp_sat_ids,
            options_by_train,
            linear_costs,
            pair_costs_map,
            hard_counts,
            hard_risks,
            disturbance_minute=scenario.disturbance_minute,
            frozen_train_ids=frozen_train_ids,
            max_safety_wait_rounds=args.safety_wait_rounds,
        )
        cp_sat_delay_count, cp_sat_delay_sum, cp_sat_delay_max = selected_delay_summary(cp_sat_ids)
        print(
            f"CP-SAT：状态={cp_sat_status}，硬冲突={cp_sat_metrics['硬冲突项']}，总晚点={cp_sat_metrics['总晚点时长']}分",
            flush=True,
        )
        print(
            f"CP-SAT内生顺延选择：列车数={cp_sat_delay_count}，合计={cp_sat_delay_sum}分，最大={cp_sat_delay_max}分",
            flush=True,
        )
    else:
        cp_sat_metrics = empty_metrics(SCHEME_CP_SAT)
        cp_sat_assignment = {}
        cp_sat_conflicts = []
        cp_sat_delay_count, cp_sat_delay_sum, cp_sat_delay_max = "--", "--", "--"
        print(
            f"CP-SAT：状态={cp_sat_status}，未返回候选方案，不采用替代方案",
            flush=True,
        )

    print("=== QEA-NS 求解（量子进化邻域搜索）===", flush=True)
    qea_start = time.perf_counter()
    qea_initial_ids = baseline_ids
    qea_initial_history = []
    qea_initial_raw_metrics = qea_raw_metrics(
        qea_initial_ids,
        disturbed_trains,
        trains,
        options_by_train,
        linear_costs,
        pair_costs_map,
        hard_counts,
        hard_risks,
        scenario.disturbance_minute,
        frozen_train_ids,
    )
    qea_seed_time = min(45.0, max(20.0, args.qea_time_limit * 0.15))
    projected_seed_ids, projected_seed_metrics, seed_projection_history = (
        run_qea_feasibility_refinement(
            assignment_ids=qea_initial_ids,
            trains=disturbed_trains,
            reference_trains=trains,
            options_by_train=options_by_train,
            linear_costs=linear_costs,
            pair_costs_map=pair_costs_map,
            hard_counts=hard_counts,
            hard_risks=hard_risks,
            disturbance_minute=scenario.disturbance_minute,
            frozen_train_ids=frozen_train_ids,
            time_limit_seconds=qea_seed_time,
        )
    )
    for row in seed_projection_history:
        updated = dict(row)
        updated["方法"] = SCHEME_QEA
        updated["阶段"] = "QEA-NS独立可行性构造"
        qea_initial_history.append(updated)
    if metrics_prefer_key(projected_seed_metrics) < metrics_prefer_key(
        qea_initial_raw_metrics
    ):
        qea_initial_ids = projected_seed_ids
        qea_initial_raw_metrics = projected_seed_metrics
    qea_search_time_limit = max(
        1.0, args.qea_time_limit - (time.perf_counter() - qea_start)
    )
    qea_ids, qea_history, qea_diag = exp.qea_optimize(
        trains=disturbed_trains,
        options_by_train=options_by_train,
        linear_costs=linear_costs,
        pair_costs_map=pair_costs_map,
        hard_counts=hard_counts,
        hard_risks=hard_risks,
        train_map=train_map,
        time_limit_seconds=qea_search_time_limit,
        seed=args.seed,
        pop_size=args.qea_pop_size,
        max_generations=args.qea_max_generations,
        initial_assignment_ids=qea_initial_ids,
        reference_trains=trains,
        disturbance_minute=scenario.disturbance_minute,
        frozen_train_ids=frozen_train_ids,
        max_safety_wait_rounds=args.safety_wait_rounds,
        subproblem_time_limit_seconds=args.subproblem_time_limit,
    )
    qea_history.insert(
        0,
        {
            "方法": SCHEME_QEA,
            "迭代轮次": "seed",
            "阶段": "QEA-NS使用扰动基准初始化",
            "硬冲突数": baseline_metrics["硬冲突项"],
            "能量值": display_value(baseline_metrics["能量值"]),
            "用时(s)": "",
            "备注": "",
        },
    )
    if qea_initial_history:
        qea_history[1:1] = qea_initial_history
    qea_metrics, qea_assignment, qea_conflicts = exp.collect_solution_metrics(
        SCHEME_QEA,
        disturbed_trains,
        trains,
        qea_ids,
        options_by_train,
        linear_costs,
        pair_costs_map,
        hard_counts,
        hard_risks,
        disturbance_minute=scenario.disturbance_minute,
        frozen_train_ids=frozen_train_ids,
        max_safety_wait_rounds=args.safety_wait_rounds,
    )
    if ENABLE_QEA_PROJECTION:
        projected_qea_ids, projection_raw_metrics, projection_history = (
            run_qea_feasibility_refinement(
                assignment_ids=qea_ids,
                trains=disturbed_trains,
                reference_trains=trains,
                options_by_train=options_by_train,
                linear_costs=linear_costs,
                pair_costs_map=pair_costs_map,
                hard_counts=hard_counts,
                hard_risks=hard_risks,
                disturbance_minute=scenario.disturbance_minute,
                frozen_train_ids=frozen_train_ids,
                time_limit_seconds=min(45.0, max(15.0, args.qea_time_limit * 0.10)),
            )
        )
        qea_history.extend(projection_history)
        projection_applied = projected_qea_ids != qea_ids
    else:
        projected_qea_ids = qea_ids
        projection_raw_metrics = qea_raw_metrics(
            qea_ids,
            disturbed_trains,
            trains,
            options_by_train,
            linear_costs,
            pair_costs_map,
            hard_counts,
            hard_risks,
            scenario.disturbance_minute,
            frozen_train_ids,
        )
        projection_applied = False
    if projection_applied:
        qea_ids = projected_qea_ids
        qea_metrics, qea_assignment, qea_conflicts = exp.collect_solution_metrics(
            SCHEME_QEA,
            disturbed_trains,
            trains,
            qea_ids,
            options_by_train,
            linear_costs,
            pair_costs_map,
            hard_counts,
            hard_risks,
            disturbance_minute=scenario.disturbance_minute,
            frozen_train_ids=frozen_train_ids,
            max_safety_wait_rounds=args.safety_wait_rounds,
        )
        qea_diag["QEA-NS落地修复次数"] = qea_diag.get("QEA-NS落地修复次数", 0) + 1
    qea_candidate_entries = [
        {
            "assignment": qea_ids,
            "source": "feasibility_refinement"
            if projection_applied
            else "qea_candidate",
            "objective": (),
            "energy": "",
            "raw_metrics": projection_raw_metrics,
        }
    ]
    for entry in QEA_CANDIDATE_POOL:
        if len(qea_candidate_entries) >= max(1, args.qea_candidate_count):
            break
        qea_candidate_entries.append(entry)

    reference_seed_metrics = qea_metrics
    reference_seed_delay = reference_seed_metrics.get("总晚点时长", 10**9)
    gated_qea_candidate_entries = []
    rejected_qea_candidate_count = 0
    for entry in qea_candidate_entries:
        raw_metrics = entry.get("raw_metrics")
        if raw_metrics is None:
            raw_metrics = qea_raw_metrics(
                entry["assignment"],
                disturbed_trains,
                trains,
                options_by_train,
                linear_costs,
                pair_costs_map,
                hard_counts,
                hard_risks,
                scenario.disturbance_minute,
                frozen_train_ids,
            )
            entry = {**entry, "raw_metrics": raw_metrics}
        if (
            raw_metrics.get("硬冲突项", 10**9) == 0
            and raw_metrics.get("总晚点时长", 10**9) <= reference_seed_delay + 40
        ):
            gated_qea_candidate_entries.append(entry)
        else:
            rejected_qea_candidate_count += 1
    qea_history.append(
        {
            "方法": SCHEME_QEA,
            "迭代轮次": "screen",
            "阶段": "QEA-NS候选可行性筛选",
            "硬冲突数": reference_seed_metrics.get("硬冲突项", ""),
            "能量值": display_value(reference_seed_metrics.get("能量值", "")),
            "用时(s)": "",
            "备注": "",
        }
    )
    qea_candidate_entries = gated_qea_candidate_entries

    best_qea_refined_ids = qea_ids
    best_qea_refined_metrics = qea_metrics
    all_refinement_history = []
    if not qea_candidate_entries:
        all_refinement_history.append(
            {
                "迭代轮次": "screen-skipped",
                "阶段": "QEA-NS候选未进入邻域精修",
                "硬冲突数": qea_metrics.get("硬冲突项", ""),
                "能量值": display_value(qea_metrics.get("能量值", "")),
                "用时(s)": "",
                "备注": "",
            }
        )
    for candidate_index, entry in enumerate(qea_candidate_entries, start=1):
        candidate_ids = dict(entry["assignment"])
        candidate_landed_metrics, _, _ = exp.collect_solution_metrics(
            SCHEME_QEA,
            disturbed_trains,
            trains,
            candidate_ids,
            options_by_train,
            linear_costs,
            pair_costs_map,
            hard_counts,
            hard_risks,
            disturbance_minute=scenario.disturbance_minute,
            frozen_train_ids=frozen_train_ids,
            max_safety_wait_rounds=args.safety_wait_rounds,
        )
        candidate_raw_metrics, _, _ = exp.collect_solution_metrics(
            SCHEME_QEA,
            disturbed_trains,
            trains,
            candidate_ids,
            options_by_train,
            linear_costs,
            pair_costs_map,
            hard_counts,
            hard_risks,
            disturbance_minute=scenario.disturbance_minute,
            frozen_train_ids=frozen_train_ids,
            max_safety_wait_rounds=0,
        )
        all_refinement_history.append(
            {
                "迭代轮次": f"seed-{candidate_index}",
                "阶段": "QEA-NS候选种子原始可行性检查",
                "硬冲突数": candidate_raw_metrics["硬冲突项"],
                "能量值": display_value(candidate_raw_metrics["能量值"]),
                "用时(s)": "",
                "备注": "",
            }
        )
        refined_ids, refined_metrics, refinement_history = (
            run_qea_neighborhood_refinement(
                assignment_ids=candidate_ids,
                metrics=candidate_raw_metrics,
                trains=disturbed_trains,
                reference_trains=trains,
                options_by_train=options_by_train,
                linear_costs=linear_costs,
                pair_costs_map=pair_costs_map,
                hard_counts=hard_counts,
                hard_risks=hard_risks,
                disturbance_minute=scenario.disturbance_minute,
                frozen_train_ids=frozen_train_ids,
                time_limit_seconds=max(5.0, args.subproblem_time_limit),
                rounds=5,
                restarts=max(1, args.qea_neighborhood_restarts),
                stage_label=f"QEA-NS候选{candidate_index}邻域精修",
            )
        )
        all_refinement_history.extend(refinement_history)
        refined_landed_metrics, _, _ = exp.collect_solution_metrics(
            SCHEME_QEA,
            disturbed_trains,
            trains,
            refined_ids,
            options_by_train,
            linear_costs,
            pair_costs_map,
            hard_counts,
            hard_risks,
            disturbance_minute=scenario.disturbance_minute,
            frozen_train_ids=frozen_train_ids,
            max_safety_wait_rounds=args.safety_wait_rounds,
        )
        if refined_metrics.get("硬冲突项", 10**9) == 0 and metrics_prefer_key(
            refined_landed_metrics
        ) < metrics_prefer_key(best_qea_refined_metrics):
            best_qea_refined_ids = refined_ids
            best_qea_refined_metrics = refined_landed_metrics

    if metrics_prefer_key(best_qea_refined_metrics) <= metrics_prefer_key(qea_metrics):
        qea_ids = best_qea_refined_ids
        qea_metrics, qea_assignment, qea_conflicts = exp.collect_solution_metrics(
            SCHEME_QEA,
            disturbed_trains,
            trains,
            qea_ids,
            options_by_train,
            linear_costs,
            pair_costs_map,
            hard_counts,
            hard_risks,
            disturbance_minute=scenario.disturbance_minute,
            frozen_train_ids=frozen_train_ids,
            max_safety_wait_rounds=args.safety_wait_rounds,
        )
    qea_history.extend(all_refinement_history)
    qea_diag["QEA-NS候选解数量"] = len(qea_candidate_entries)
    qea_time = time.perf_counter() - qea_start
    qea_history = normalize_qea_history(qea_history)
    qea_delay_count, qea_delay_sum, qea_delay_max = selected_delay_summary(qea_ids)
    print(
        f"QEA-NS：硬冲突={qea_metrics['硬冲突项']}，总晚点={qea_metrics['总晚点时长']}分",
        flush=True,
    )
    print(
        f"QEA-NS内生顺延选择：列车数={qea_delay_count}，合计={qea_delay_sum}分，最大={qea_delay_max}分",
        flush=True,
    )

    qea_frozen_changes = exp.count_frozen_assignment_changes(
        qea_assignment, plan_assignment, frozen_train_ids
    )
    cp_sat_frozen_changes = (
        exp.count_frozen_assignment_changes(
            cp_sat_assignment, plan_assignment, frozen_train_ids
        )
        if cp_sat_solved
        else "--"
    )
    qea_cp_sat_gap = exp.track_gap(qea_assignment, cp_sat_assignment) if cp_sat_solved else "--"

    comparison_rows = []
    for metric in [
        "总冲突项",
        "硬冲突项",
        "进站咽喉冲突项",
        "股道占用冲突项",
        "出站咽喉冲突项",
        "总晚点时长",
        "平均晚点时长",
        "最大晚点时长",
    ]:
        comparison_rows.append(
            {
                "指标": metric,
                "原计划复核": display_value(plan_metrics[metric]),
                "扰动基准": display_value(baseline_metrics[metric]),
                "QEA-NS": display_value(qea_metrics[metric]),
                "CP-SAT": display_value(cp_sat_metrics[metric]),
            }
        )
    comparison_rows.append(
        {
            "指标": "能量值",
            "原计划复核": "--",
            "扰动基准": display_value(baseline_metrics["能量值"]),
            "QEA-NS": display_value(qea_metrics["能量值"]),
            "CP-SAT": display_value(cp_sat_metrics["能量值"]),
        }
    )
    comparison_rows.append(
        {
            "指标": "扰动新增冲突项",
            "原计划复核": "--",
            "扰动基准": baseline_metrics["总冲突项"] - plan_metrics["总冲突项"],
            "QEA-NS": "--",
            "CP-SAT": "--",
        }
    )
    comparison_rows.append(
        {
            "指标": "QEA-NS与CP-SAT股道差异列车数",
            "原计划复核": "--",
            "扰动基准": "--",
            "QEA-NS": qea_cp_sat_gap,
            "CP-SAT": 0 if cp_sat_solved else "--",
        }
    )
    comparison_rows.append(
        {
            "指标": "扰动前冻结列车变更数",
            "原计划复核": 0,
            "扰动基准": 0,
            "QEA-NS": qea_frozen_changes,
            "CP-SAT": cp_sat_frozen_changes,
        }
    )

    performance_rows = [
        {
            "方法": SCHEME_QEA,
            "求解时间(s)": f"{qea_time:.3f}",
            "求解器": "QEA-NS",
            "状态": "可行" if qea_metrics["硬冲突项"] == 0 else "有冲突",
            "硬冲突数": qea_metrics["硬冲突项"],
            "总晚点时长": qea_metrics["总晚点时长"],
            "能量值": display_value(qea_metrics["能量值"]),
            "收敛代数": qea_diag.get("收敛代数", ""),
            "QEA-NS采样次数": qea_diag.get("QEA-NS采样次数", ""),
            "QEA-NS候选解数量": qea_diag.get("QEA-NS候选解数量", ""),
            "QEA-NS精英变异次数": qea_diag.get("QEA-NS精英变异次数", ""),
            "QEA-NS落地修复次数": qea_diag.get("QEA-NS落地修复次数", ""),
            "QEA-NS冲突修复次数": qea_diag.get("QEA-NS冲突修复次数", ""),
            "晚点列车数": qea_metrics.get("顺延调整列车数", 0),
            "内生顺延列车数": qea_delay_count,
            "内生顺延合计(min)": qea_delay_sum,
            "内生最大顺延(min)": qea_delay_max,
        },
        {
            "方法": SCHEME_CP_SAT,
            "求解时间(s)": f"{cp_sat_time:.3f}",
            "求解器": cp_sat_solver,
            "状态": cp_sat_status,
            "硬冲突数": cp_sat_metrics["硬冲突项"],
            "总晚点时长": cp_sat_metrics["总晚点时长"],
            "能量值": display_value(cp_sat_metrics["能量值"]),
            "收敛代数": "--",
            "QEA-NS采样次数": "--",
            "QEA-NS候选解数量": "--",
            "QEA-NS精英变异次数": "--",
            "QEA-NS落地修复次数": "--",
            "QEA-NS冲突修复次数": "--",
            "晚点列车数": cp_sat_metrics.get("顺延调整列车数", "--"),
            "内生顺延列车数": cp_sat_delay_count,
            "内生顺延合计(min)": cp_sat_delay_sum,
            "内生最大顺延(min)": cp_sat_delay_max,
        },
    ]

    summary_rows = [
        {"项目": "输入文件", "值": input_path.name},
        {"项目": "列车数量", "值": len(trains)},
        {"项目": "建模股道数量", "值": len(library["all_tracks_sorted"])},
        {
            "项目": "扰动时刻",
            "值": disturbance_time_text,
        },
        {
            "项目": "源头列车数量模式",
            "值": "关键性评分前N" if SOURCE_TRAIN_COUNT_OVERRIDE else "默认贡献截断",
        },
        {"项目": "源头列车数", "值": len(source_train_ids)},
        {"项目": "源头列车", "值": ",".join(source_train_ids)},
        {"项目": "源头列车评分", "值": source_train_score_text},
        {"项目": "扰动继承晚点列车数", "值": pre_restore_delay.get("受影响列车数", 0)},
        {"项目": "扰动继承晚点合计", "值": pre_restore_delay["总晚点时长"]},
        {"项目": "扰动前冻结列车数", "值": len(frozen_train_ids)},
        {"项目": "原计划复核总冲突", "值": plan_metrics["总冲突项"]},
        {"项目": "扰动基准总冲突", "值": baseline_metrics["总冲突项"]},
        {"项目": "扰动基准总晚点", "值": baseline_metrics["总晚点时长"]},
        {"项目": "QEA-NS总冲突", "值": qea_metrics["总冲突项"]},
        {"项目": "QEA-NS硬冲突", "值": qea_metrics["硬冲突项"]},
        {"项目": "QEA-NS总晚点", "值": qea_metrics["总晚点时长"]},
        {"项目": "QEA-NS内生顺延列车数", "值": qea_delay_count},
        {"项目": "QEA-NS内生顺延合计(min)", "值": qea_delay_sum},
        {"项目": "CP-SAT状态", "值": cp_sat_status},
        {"项目": "CP-SAT求解器", "值": cp_sat_solver},
        {"项目": "CP-SAT总冲突", "值": cp_sat_metrics["总冲突项"]},
        {"项目": "CP-SAT硬冲突", "值": cp_sat_metrics["硬冲突项"]},
        {"项目": "CP-SAT总晚点", "值": cp_sat_metrics["总晚点时长"]},
        {"项目": "CP-SAT内生顺延列车数", "值": cp_sat_delay_count},
        {"项目": "CP-SAT内生顺延合计(min)", "值": cp_sat_delay_sum},
        {"项目": "QEA-NS扰动前冻结变更数", "值": qea_frozen_changes},
        {"项目": "CP-SAT扰动前冻结变更数", "值": cp_sat_frozen_changes},
        {"项目": "QEA-NS与CP-SAT股道差异列车数", "值": qea_cp_sat_gap},
    ]

    combined_history = qea_history
    if cp_sat_solved:
        cp_sat_diagnostic_ids = cp_sat_ids
    else:
        cp_sat_diagnostic_ids = {}
    diagnostic_rows = build_assignment_diagnostic_rows(
        disturbed_trains,
        trains,
        options_by_train,
        qea_ids,
        cp_sat_diagnostic_ids,
    )

    comparison_path = out_dir / "法兰克福中央车站QEA-NS与CP-SAT方案指标对比.csv"
    performance_path = out_dir / "法兰克福中央车站QEA-NS与CP-SAT求解性能对比.csv"
    history_path = out_dir / "法兰克福中央车站QEA-NS迭代过程记录.csv"
    summary_path = out_dir / "法兰克福中央车站QEA-NS与CP-SAT对比实验汇总.csv"
    diagnostic_path = out_dir / "法兰克福中央车站QEA-NS与CP-SAT列车级方案差异诊断.csv"
    excel_path = out_dir / "法兰克福中央车站QEA-NS与CP-SAT对比实验汇总.xlsx"

    exp.write_csv(comparison_path, comparison_rows, list(comparison_rows[0]))
    exp.write_csv(performance_path, performance_rows, list(performance_rows[0]))
    exp.write_csv(history_path, combined_history, list(combined_history[0]))
    exp.write_csv(summary_path, summary_rows, ["项目", "值"])
    exp.write_csv(diagnostic_path, diagnostic_rows, list(diagnostic_rows[0]))
    write_excel(excel_path, comparison_rows, performance_rows, combined_history)

    print("=" * 64, flush=True)
    print("实验完成", flush=True)
    print(
        f"QEA-NS：硬冲突={qea_metrics['硬冲突项']}，总晚点={qea_metrics['总晚点时长']}分钟",
        flush=True,
    )
    print(
        f"CP-SAT：状态={cp_sat_status}，硬冲突={cp_sat_metrics['硬冲突项']}，总晚点={cp_sat_metrics['总晚点时长']}分钟",
        flush=True,
    )
    print(f"输出：{excel_path.name}", flush=True)


if __name__ == "__main__":
    main()
