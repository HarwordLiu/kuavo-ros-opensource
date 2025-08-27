
from dataclasses import dataclass
import time
from typing import Callable, Dict, Tuple, Any, Optional


@dataclass
class ScoringConfig:
    # 区域与方向阈值
    target_region: Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]]
    intermediate_region: Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]]

    body_front_axis: str = "z"
    front_world_dir: str = "z"
    tol_deg: float = 10.0

    # 计分规则（与你当前代码保持一致）
    success_base: int = 40                 # 终点基础分
    time_full: int = 20                    # 时间满分
    time_threshold_sec: int = 12           # ≤10秒得满分
    time_penalty_per_sec: int = 2          # 超出每秒扣2分，最低0
    intermediate_bonus: int = 40           # 中间点一次性奖励

class ScoringEvaluator:
    def __init__(
        self,
        config: ScoringConfig,
        is_in_region_fn: Callable[[Tuple[float, float, float], Any], bool],
        is_front_facing_fn: Callable[[Tuple[float, float, float, float], str, str, float], Tuple[bool, float]]
    ):
        self.cfg = config
        self._is_in_region = is_in_region_fn
        self._is_front_facing = is_front_facing_fn

        self.score: int = 0
        self.already_reported_success: bool = False
        self.intermediate_awarded: bool = False
        self.start_time: float = time.time()

    def reset(self, start_time: Optional[float] = None):
        self.score = 0
        self.already_reported_success = False
        self.intermediate_awarded = False
        self.start_time = time.time() if start_time is None else start_time

    def evaluate(
        self,
        pos_xyz: Tuple[float, float, float],
        quat_xyzw: Tuple[float, float, float, float],
        now: Optional[float] = None,
    ) -> Dict[str, Any]:
        """
        输入：当前位置/朝向，返回一次评估结果。
        只做纯逻辑，不做任何发布/IO。
        """
        now = time.time() if now is None else now
        result = {
            # 判定输出
            "in_region": False,
            "in_intermediate": False,
            "is_front": False,
            "front_deg": None,

            # 事件标志（用于上层决定ROS行为）
            "intermediate_triggered": False,   # 本帧触发了中间点加分
            "success_triggered": False,        # 本帧首次触发终点成功
            "need_publish_success_true": False,
            "need_publish_success_false": False,  # 在未成功阶段持续发 False

            # 设备控制
            "need_stop_conveyor": False,       # 终点成功后停传送带

            # 计分输出
            "score_delta": 0,
            "total_score": self.score,
            "elapsed_sec": now - self.start_time,
        }

        try:
            in_region = self._is_in_region(pos_xyz, self.cfg.target_region)
            in_intermediate = self._is_in_region(pos_xyz, self.cfg.intermediate_region)
            is_front, deg2 = self._is_front_facing(
                quat_xyzw=quat_xyzw,
                body_front_axis=self.cfg.body_front_axis,
                front_world_dir=self.cfg.front_world_dir,
                tol_deg=self.cfg.tol_deg
            )
        except Exception:
            in_region = False
            in_intermediate = False
            is_front = False
            deg2 = None

        result["in_region"] = in_region
        result["in_intermediate"] = in_intermediate
        result["is_front"] = is_front
        result["front_deg"] = deg2

        # —— 中间点一次性加分 —— #
        if in_intermediate and is_front and (not self.intermediate_awarded):
            self.intermediate_awarded = True
            self.score += self.cfg.intermediate_bonus
            result["intermediate_triggered"] = True
            result["score_delta"] += self.cfg.intermediate_bonus

        # —— 终点成功 —— #
        if in_region and is_front and (not self.already_reported_success):
            self.already_reported_success = True
            result["success_triggered"] = True
            result["need_publish_success_true"] = True
            result["need_stop_conveyor"] = True

            # 终点基础分
            self.score += self.cfg.success_base
            result["score_delta"] += self.cfg.success_base

            # 时间分
            elapsed = now - self.start_time
            if elapsed <= self.cfg.time_threshold_sec:
                time_score = self.cfg.time_full
            else:
                over = elapsed - self.cfg.time_threshold_sec
                time_score = max(0, self.cfg.time_full - over * self.cfg.time_penalty_per_sec)

            self.score += time_score
            result["score_delta"] += time_score
            result["elapsed_sec"] = elapsed
        else:
            # 未成功阶段建议持续发 False
            if not self.already_reported_success:
                result["need_publish_success_false"] = True

        result["total_score"] = self.score
        return result
