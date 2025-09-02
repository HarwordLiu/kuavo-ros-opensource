
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

    # # 计分规则
    # time_full: int = 20                    # 时间满分
    # time_threshold_sec: int = 12           # ≤10秒得满分
    # time_penalty_per_sec: int = 2          # 超出每秒扣2分，最低0

@dataclass
class ScoringConfig3:
    # 区域与方向阈值
    target_region: Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]]

    body_front_axis: str = "-y"
    front_world_dir: str = "z"
    tol_deg: float = 10.0

    # # 计分规则
    # time_full: int = 20                    # 时间满分
    # time_threshold_sec: int = 30           # ≤10秒得满分
    # time_penalty_per_sec: int = 2          # 超出每秒扣2分，最低0
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

        self.intermediate_pos_awarded: bool = False
        self.intermediate_ori_awarded: bool = False
        # self.intermediate_awarded: bool = False

        self.final_pos_awarded: bool = False
        self.final_ori_awarded: bool = False
        # self.final_awarded: bool = False        
        # self.start_time: float = time.time()

    def reset(self, start_time: Optional[float] = None):
        self.score = 0
        self.already_reported_success = False
        self.intermediate_pos_awarded = False
        self.intermediate_ori_awarded = False
        # self.intermediate_awarded = False

        self.final_pos_awarded = False
        self.final_ori_awarded = False
        # self.final_awarded = False 
        # self.start_time = time.time() if start_time is None else start_time

    def evaluate(
        self,
        pos_xyz: Tuple[float, float, float],
        quat_xyzw: Tuple[float, float, float, float],
        # now: Optional[float] = None,
    ) -> Dict[str, Any]:
        """
        输入：当前位置/朝向，返回一次评估结果。
        只做纯逻辑，不做任何发布/IO。
        """
        # now = time.time() if now is None else now
        result = {
            # 判定输出
            "in_region": False,
            "in_intermediate": False,
            "is_front": False,
            "front_deg": None,

            # 事件标志（用于上层决定ROS行为）
            # "intermediate_triggered": False,   # 本帧触发了中间点加分
            # "final_triggered" : False,          # 本帧首次触发终点成功       
            "need_publish_success_true": False,
            "need_publish_success_false": False,  # 在未成功阶段持续发 False

            # 设备控制
            "need_stop_conveyor": False,       # 终点成功后停传送带

            # 计分输出
            "score_delta": 0,
            "total_score": self.score,
            # "elapsed_sec": now - self.start_time,
            "intermediate_pos_added": False,
            "intermediate_ori_added": False,
            "final_pos_added": False,   # 30 或 0
            "final_ori_added": False,   # 10 或 0
            # "time_score_added": 0,    # 时间分
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
        # —— 中间点分项一次性加分：位置30 + 方向10 —— #
        # 规则：必须“在中间区域 in_intermediate”时才考虑加分；
        #       位置分：第一次检测到 in_intermediate 即加 30 分（只加一次）；
        #       方向分：在 in_intermediate 且 is_front 为真时，加 10 分（只加一次）。
        if in_intermediate:
            # 位置40分（只加一次）
            if not self.intermediate_pos_awarded:
                self.intermediate_pos_awarded = True
                self.score += 40
                result["score_delta"] += 40
                # result["intermediate_triggered"] = True  # 本帧有中间点得分事件
                result["intermediate_pos_added"] = True

            # 方向10分（只加一次，且仅当朝向正确）
            if is_front and not self.intermediate_ori_awarded:
                self.intermediate_ori_awarded = True
                self.score += 10
                result["score_delta"] += 10
                result["intermediate_ori_added"] = True

            # # 当两个子项都拿到时，标记“中间点完成”（用于兼容原字段/上层逻辑）
            # if self.intermediate_pos_awarded and self.intermediate_ori_awarded:
            #     self.intermediate_awarded = True

        if in_region:
            # 位置40分（只加一次）
            if not self.final_pos_awarded:
                self.final_pos_awarded = True
                self.score += 40
                result["score_delta"] += 40
                # result["final_triggered"] = True  # 本帧有中间点得分事件
                result["final_pos_added"] = True

            # 方向10分（只加一次，且仅当朝向正确）
            if is_front and not self.final_ori_awarded:
                self.final_ori_awarded = True
                self.score += 10
                result["score_delta"] += 10
                result["final_ori_added"] = True

            # # 当两个子项都拿到时，标记“中间点完成”（用于兼容原字段/上层逻辑）
            # if self.final_pos_awarded and self.final_ori_awarded:
            #     self.final_awarded = True

        # —— 终点成功 —— #
        if in_region and (not self.already_reported_success):
            self.already_reported_success = True
            result["need_publish_success_true"] = True
            result["need_stop_conveyor"] = True

            # # 时间分
            # elapsed = now - self.start_time
            # if elapsed <= self.cfg.time_threshold_sec:
            #     time_score = self.cfg.time_full
            # else:
            #     over = elapsed - self.cfg.time_threshold_sec
            #     time_score = max(0, self.cfg.time_full - over * self.cfg.time_penalty_per_sec)

            # self.score += time_score
            # result["score_delta"] += time_score
            # result["elapsed_sec"] = elapsed
            # result["time_score_added"] = int(time_score)
        else:
            # 未成功阶段建议持续发 False
            if not self.already_reported_success:
                result["need_publish_success_false"] = True

        result["total_score"] = self.score
        return result

class ScoringEvaluator3:
    def __init__(
        self,
        config: ScoringConfig3,
        is_in_region_fn: Callable[[Tuple[float, float, float], Any], bool],
        is_front_facing_fn: Callable[[Tuple[float, float, float, float], str, str, float], Tuple[bool, float]]
    ):
        self.cfg = config
        self._is_in_region = is_in_region_fn
        self._is_front_facing = is_front_facing_fn

        self.score: int = 0
        self.already_reported_success: bool = False

        self.front_obj1_pos_awarded: bool = False
        self.front_obj1_ori_awarded: bool = False
        self.front_obj1_awarded: bool = False
        
        self.back_obj1_pos_awarded: bool = False
        self.back_obj1_ori_awarded: bool = False
        self.back_obj1_awarded: bool = False

        self.back_obj2_pos_awarded: bool = False
        self.back_obj2_ori_awarded: bool = False
        self.back_obj2_awarded: bool = False

        # self.start_time: float = time.time()

    def reset(self, start_time: Optional[float] = None):
        self.score = 0
        self.already_reported_success = False

        self.front_obj1_pos_awarded = False
        self.front_obj1_ori_awarded = False
        self.front_obj1_awarded = False
        
        self.back_obj1_pos_awarded = False
        self.back_obj1_ori_awarded = False
        self.back_obj1_awarded = False

        self.back_obj2_pos_awarded = False
        self.back_obj2_ori_awarded = False
        self.back_obj2_awarded = False

        # self.start_time = time.time() if start_time is None else start_time

    def evaluate(
        self,
        pos_xyz_front_obj1: Tuple[float, float, float],
        quat_xyzw_front_obj1: Tuple[float, float, float, float],
        pos_xyz_back_obj1: Tuple[float, float, float],
        quat_xyzw_back_obj1: Tuple[float, float, float, float],
        pos_xyz_back_obj2: Tuple[float, float, float],
        quat_xyzw_back_obj2: Tuple[float, float, float, float],

        # now: Optional[float] = None,
    ) -> Dict[str, Any]:
        """
        输入：当前位置/朝向，返回一次评估结果。
        只做纯逻辑，不做任何发布/IO。
        """
        # now = time.time() if now is None else now
        result = {
            # 判定输出
            "in_region_front_obj1": False,
            "is_front_front_obj1": False,
            "deg_front_obj1": None,

            "in_region_back_obj1": False,
            "is_front_back_obj1": False,
            "deg_back_obj1": None,

            "in_region_back_obj2": False,
            "is_front_back_obj2": False,
            "deg_back_obj2": None,

            # 事件标志（用于上层决定ROS行为）
            "front_obj1_triggered": False,   # 本帧触发了中间点加分
            "back_obj1_triggered": False,
            "back_obj2_triggered": False,

            "success_triggered": False,        # 本帧首次触发终点成功
            "need_publish_success_true": False,
            "need_publish_success_false": False,  # 在未成功阶段持续发 False

            # 计分输出
            "score_delta": 0,
            "total_score": self.score,
            # "elapsed_sec": now - self.start_time,

            "front_obj1_pos_added": False,
            "front_obj1_ori_added": False,
            "back_obj1_pos_added": False,  
            "back_obj1_ori_added": False,  
            "back_obj2_pos_added": False,  
            "back_obj2_ori_added": False,
            # "time_score_added": 0,    # 时间分
        }

        try:
            in_region_front_obj1 = self._is_in_region(pos_xyz_front_obj1, self.cfg.target_region)
            is_front_front_obj1, deg_front_obj1 = self._is_front_facing(
                quat_xyzw=quat_xyzw_front_obj1,
                body_front_axis=self.cfg.body_front_axis,
                front_world_dir=self.cfg.front_world_dir,
                tol_deg=self.cfg.tol_deg)

            in_region_back_obj1 = self._is_in_region(pos_xyz_back_obj1, self.cfg.target_region)
            is_front_back_obj1, deg_back_obj1 = self._is_front_facing(
                quat_xyzw=quat_xyzw_back_obj1,
                body_front_axis=self.cfg.body_front_axis,
                front_world_dir=self.cfg.front_world_dir,
                tol_deg=self.cfg.tol_deg)
        
            in_region_back_obj2 = self._is_in_region(pos_xyz_back_obj2, self.cfg.target_region)
            is_front_back_obj2, deg_back_obj2 = self._is_front_facing(
                quat_xyzw=quat_xyzw_back_obj2,
                body_front_axis=self.cfg.body_front_axis,
                front_world_dir=self.cfg.front_world_dir,
                tol_deg=self.cfg.tol_deg)
            
        except Exception:
            in_region_front_obj1 = False
            is_front_front_obj1 = False
            deg_front_obj1 = None
    
            in_region_back_obj1 = False
            is_front_back_obj1 = False
            deg_back_obj1 = None

            in_region_back_obj2 = False
            is_front_back_obj2 = False
            deg_back_obj2 = None

        result["in_region_front_obj1"] = in_region_front_obj1
        result["is_front_front_obj1"] = is_front_front_obj1
        result["deg_front_obj1"] = deg_front_obj1

        result["in_region_back_obj1"] = in_region_back_obj1
        result["is_front_back_obj1"] = is_front_back_obj1
        result["deg_back_obj1"] = deg_back_obj1

        result["in_region_back_obj2"] = in_region_back_obj2
        result["is_front_back_obj2"] = is_front_back_obj2
        result["deg_back_obj2"] = deg_back_obj2

        # —— 反面物体加分 —— #
        if in_region_back_obj1:
            # 位置10分（只加一次）
            if not self.back_obj1_pos_awarded:
                self.back_obj1_pos_awarded = True
                self.score += 10
                result["score_delta"] += 10
                result["back_obj1_triggered"] = True  # 本帧有中间点得分事件
                result["back_obj1_pos_added"] = True

            # 方向20分（只加一次，且仅当朝向正确）
            if is_front_back_obj1 and not self.back_obj1_ori_awarded:
                self.back_obj1_ori_awarded = True
                self.score += 25
                result["score_delta"] += 25
                result["back_obj1_ori_added"] = True

            # 当两个子项都拿到时，标记“中间点完成”
            if self.back_obj1_pos_awarded and self.back_obj1_ori_awarded:
                self.back_obj1_awarded = True

        # —— 反面物体加分 —— #
        if in_region_back_obj2:
            # 位置10分（只加一次）
            if not self.back_obj2_pos_awarded:
                self.back_obj2_pos_awarded = True
                self.score += 10
                result["score_delta"] += 10
                result["back_obj2_triggered"] = True  # 本帧有中间点得分事件
                result["back_obj2_pos_added"] = True

            # 方向20分（只加一次，且仅当朝向正确）
            if is_front_back_obj2 and not self.back_obj2_ori_awarded:
                self.back_obj2_ori_awarded = True
                self.score += 25
                result["score_delta"] += 25
                result["back_obj2_ori_added"] = True

            # 当两个子项都拿到时，标记“中间点完成”
            if self.back_obj2_pos_awarded and self.back_obj2_ori_awarded:
                self.back_obj2_awarded = True

        # —— 正面物体加分 —— #
        if in_region_front_obj1:
            # 位置30分（只加一次）
            if not self.front_obj1_pos_awarded:
                self.front_obj1_pos_awarded = True
                self.score += 10
                result["score_delta"] += 10
                result["front_obj1_triggered"] = True  # 本帧有中间点得分事件
                result["front_obj1_pos_added"] = True

            # 方向10分（只加一次，且仅当朝向正确）
            if is_front_front_obj1 and not self.front_obj1_ori_awarded:
                self.front_obj1_ori_awarded = True
                self.score += 15
                result["score_delta"] += 15
                result["front_obj1_ori_added"] = True

            # 当两个子项都拿到时，标记“中间点完成”
            if self.front_obj1_pos_awarded and self.front_obj1_ori_awarded:
                self.front_obj1_awarded = True


        # —— 终点成功 —— #
        if self.front_obj1_awarded and self.back_obj1_awarded and self.back_obj2_awarded and (not self.already_reported_success):
            self.already_reported_success = True
            result["success_triggered"] = True
            result["need_publish_success_true"] = True
            result["need_stop_conveyor"] = True

            # # 时间分
            # elapsed = now - self.start_time
            # if elapsed <= self.cfg.time_threshold_sec:
            #     time_score = self.cfg.time_full
            # else:
            #     over = elapsed - self.cfg.time_threshold_sec
            #     time_score = max(0, self.cfg.time_full - over * self.cfg.time_penalty_per_sec)

            # self.score += time_score
            # result["score_delta"] += time_score
            # result["elapsed_sec"] = elapsed
            # result["time_score_added"] = int(time_score)
        else:
            # 未成功阶段建议持续发 False
            if not self.already_reported_success:
                result["need_publish_success_false"] = True

        result["total_score"] = self.score
        return result