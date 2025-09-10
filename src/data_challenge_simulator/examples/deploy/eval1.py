import os, sys, time, math, threading, traceback

current_dir = os.path.dirname(os.path.abspath(__file__))
data_challenge_simulator_dir = os.path.dirname(current_dir)
if data_challenge_simulator_dir not in sys.path:
    sys.path.insert(0, data_challenge_simulator_dir)

from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot, KuavoRobotState
from utils.gripper_controller import GripperController
from utils.conveyor_controller import ConveyorController
from utils.object_pos import ObjectPose
from utils.object_randomizer import ObjectRandomizer
from utils.trajectory_controller import TrajectoryController
from utils.utils import Utils
from utils.evaluator import ScoringEvaluator, ScoringConfig
import rospy
from std_msgs.msg import Bool, Int32
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
import numpy as np

class SimulatorTask1():
    def __init__(self,seed):
        rospy.init_node('simulator_task1', anonymous=False)

        self.init_service = rospy.ServiceProxy('/simulator/init', Trigger)
        self.pub_success = rospy.Publisher('/simulator/success',Bool, queue_size=10)
        # self.pub_score   = rospy.Publisher('/simulator/score', Int32, queue_size=10, latch=True)
        # 等待外部信号
        self.start_service = rospy.Service('/simulator/start', Trigger, self._on_start_service)
        self.reset_service = rospy.Service('/simulator/reset', Trigger, self._on_reset_service)

        # 事件控制
        self.start_evt = threading.Event()
        self.reset_evt = threading.Event()

        # 设备与控制器
        self.robot = None
        self.robot_state = None
        self.conveyor_ctrl = None
        self.gripper_ctrl = None
        self.traj_ctrl = None
        self.obj_pos = None
        self.score = 0
        default_score_file = "/tmp/simulator_score_last.txt"
        self.score_file = os.environ.get("SCORE_FILE", default_score_file)
        # 成功状态
        self.started = False
        self.already_reported_success = False
        self.intermediate_awarded = False
        # 区域阈值
        self.seed = seed
        self.obj_pos = ObjectPose()
        # self.target_region = [
        #     (0.36, 0.62),   # x
        #     (-0.6, -0.32),  # y
        #     (0.85, 1.0)     # z
        # ]
        # self.intermediate_region = [
        #     (0.38, 0.52),   # x 范围
        #     (-0.1, 0.04),   # y 范围
        #     (0.85, 1.0)  # z 范围
        # ]
        self.marker1_pos = self.obj_pos.wait_for_position("marker1", timeout=5.0)
        self.marker2_pos = self.obj_pos.wait_for_position("marker2", timeout=5.0)

        self.intermediate_region = [
        (self.marker1_pos[0]-0.03, self.marker1_pos[0]+0.03),   # x 范围
        (self.marker1_pos[1]-0.03, self.marker1_pos[1]+0.03),   # y 范围
        (0.85, 0.98)  # z 范围
        ]


        self.target_region = [
        (self.marker2_pos[0]-0.035, self.marker2_pos[0]+0.035),   # x 范围
        (self.marker2_pos[1]-0.035, self.marker2_pos[1]+0.035),   # y 范围
        (0.85, 0.98)  # z 范围
        ]

        self.evaluator = ScoringEvaluator(
            ScoringConfig(
                target_region=self.target_region,
                intermediate_region=self.intermediate_region,
                body_front_axis='z',
                front_world_dir='z',
                tol_deg=10.0,
                time_full=20,
                time_threshold_sec=13,
                time_penalty_per_sec=2,
                # intermediate_bonus=40,
            ),
            is_in_region_fn=lambda pos, region: Utils.is_in_target_region(pos, region),
            is_front_facing_fn=lambda quat_xyzw, body_front_axis, front_world_dir, tol_deg:
                Utils.is_front_facing(quat_xyzw=quat_xyzw,
                                    body_front_axis=body_front_axis,
                                    front_world_dir=front_world_dir,
                                    tol_deg=tol_deg)
        )
    # ========== 服务回调 - 等待外部信号 ==========
    def _on_start_service(self, req):
        """等待外部代码发送 start 信号"""
        rospy.loginfo("[sim] 收到外部 start 信号，开始执行任务")
        self.started = True
        self.start_evt.set()
        return TriggerResponse(success=True, message="Task started successfully")

    def _on_reset_service(self, req):
        """等待外部代码发送 reset 信号"""
        rospy.loginfo("[sim] 收到外部 reset 信号，准备重置任务")
        self.reset_evt.set()
        return TriggerResponse(success=True, message="Task reset triggered")

    def _sample_position_with_seed(self, seed: int, position_ranges: dict):
        """
        使用固定种子采样位置，返回 (x, y, z)
        """
        rng = np.random.default_rng(int(seed))
        x = float(rng.uniform(*position_ranges['x']))
        y = float(rng.uniform(*position_ranges['y']))
        z = float(rng.uniform(*position_ranges['z']))
        return x, y, z

    # ========== 主流程 ==========
    def run(self):
        try:
            self.reset_evt.clear()
            self.start_evt.clear()

            # 1) 初始化 SDK 与控制器
            if not KuavoSDK().Init(options=KuavoSDK.Options.WithIK):
                print("Init KuavoSDK failed, exit!")
                return  # 直接退出，deploy.py 会重启一轮

            self.robot = KuavoRobot()
            self.robot_state = KuavoRobotState()

            self.conveyor_ctrl = ConveyorController()
            self.obj_pos       = ObjectPose()

            # 随机化物体位置
            obj_pos = ObjectRandomizer()
            x, y, z = self._sample_position_with_seed(seed=self.seed,position_ranges={
                    'x': [0.7, 0.9],
                    'y': [0.45, 0.65],
                    'z': [0.95, 0.95]
                })
            
            result = obj_pos.set_object_position(
                object_name='box_grab',
                position = {"x":x, "y":y, "z":z}
            )
            print("\033[92mXXXXXXXX\033[0m",[x,y,z])
            # 2) 预抓位
            num = 20
            q_target1 = [0, 0, 0, -105, -70, 0, 0,   50, 0, 0, -140, 90, 0, 0]
            q_list1 = Utils.interpolate_joint_trajectory(q_target1, num=num)

            q_target2 = [-10, 15, 25, -100, -120, 0, 0,   50, 0, 0, -120, 90, 0, 0]
            q_list2 = Utils.interpolate_joint_trajectory(q_target2, q_target1, num=num)

            q_target3 = [-10, 15, 25, -95, -180, 25, -20,   50, 0, 0, -140, 90, 0, 0]
            q_list3 = Utils.interpolate_joint_trajectory(q_target3, q_target2, num=num)

            for q in q_list1 :
                self.robot.control_arm_joint_positions(q)
                time.sleep(0.02)
            for q in q_list2:
                self.robot.control_arm_joint_positions(q)
                time.sleep(0.02)
            for q in q_list3:
                self.robot.control_arm_joint_positions(q)
                time.sleep(0.02)
            
            # 3) 发布 msg0：init=True
            rospy.wait_for_service('/simulator/init')
            try:
                rospy.loginfo("[sim] 发布 /simulator/init 服务完成")
                resp = self.init_service(TriggerRequest())
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")


            # 4) 等待外部代码调用 start 服务
            rospy.loginfo("[sim] 等待外部代码调用 /simulator/start 服务...")
            while not rospy.is_shutdown() and not self.reset_evt.is_set():
                if self.start_evt.wait(timeout=0.1):
                    break

            if self.reset_evt.is_set() or rospy.is_shutdown():
                rospy.logwarn("[sim] 等待 start 期间收到 reset / shutdown，退出")
                self._cleanup()
                sys.exit(0)

            # 5) 收到外部 start 信号 → 开传送带，循环上报 success
            rospy.loginfo("[sim] 收到外部 start 信号，开始传送带，持续上报 success")
            self.conveyor_ctrl.control_speed(-0.1)

            rate = rospy.Rate(10)  # 10Hz 上报
            self.already_reported_success = False
            # start_time = time.time()  # 循环开始前计时
            self.evaluator.reset()
            
            while not rospy.is_shutdown() and not self.reset_evt.is_set():
                try:
                    pos = self.obj_pos.get_position("box_grab")
                    ori = self.obj_pos.get_orientation("box_grab")
                except Exception as e:
                    rospy.logwarn(f"[sim] 获取位置/姿态出错：{e}")
                    pos, ori = None, None

                if pos is None or ori is None:
                    # 取不到传感就当未成功
                    # 未成功阶段持续发 False
                    self.pub_success.publish(Bool(data=False))
                    # 持续发布分数
                    # self.pub_score.publish(Int32(data=self.evaluator.score))
                    rate.sleep()
                    continue

                # 调用通用评估器
                out = self.evaluator.evaluate(pos, ori, now=time.time())

                # 按评估器的“建议标志”做 ROS 行为
                if out["need_publish_success_true"]:
                    rospy.loginfo("[sim] ✅ 任务成功，发布 /simulator/success=True")
                    self.pub_success.publish(Bool(data=True))

                elif out["need_publish_success_false"]:
                    self.pub_success.publish(Bool(data=False))

                if out["intermediate_pos_added"] or out["intermediate_ori_added"]:
                    parts = []
                    if out["intermediate_pos_added"]: parts.append("+30(位置)")
                    if out["intermediate_ori_added"]: parts.append("+10(方向)")
                    rospy.loginfo(f"[sim] 🟡 中间点达成：{' '.join(parts)}，总分 {out['total_score']}")

                if out["final_pos_added"] or out["final_ori_added"]:
                    parts = []
                    if out["final_pos_added"]: parts.append("+30(位置)")
                    if out["final_ori_added"]: parts.append("+10(方向)")
                    parts.append(f"+{out['time_score_added']}(时间)")
                    rospy.loginfo(f"[sim] 🟡 终点成功:{' '.join(parts)}， 用时 {out['elapsed_sec']:.2f}s，总分 {out['total_score']}")
                    # 停止传送带
                    if out["need_stop_conveyor"]:
                        self.conveyor_ctrl.control_speed(0.0)

                # 持续发布当前分数
                self.score = out["total_score"]

                rate.sleep()


            # 6) 收到 reset → 清理并退出 (让 deploy.py 重启新一轮)
            rospy.loginfo("[sim] 收到 reset/shutdown，开始清理并退出")
            self._cleanup()
            sys.exit(0)

        except KeyboardInterrupt:
            rospy.loginfo("[sim] 用户中断")
            self._cleanup()
            sys.exit(0)
        except Exception as e:
            rospy.logerr(f"[sim] 程序执行出错: {e}")
            traceback.print_exc()
            self._cleanup()
            sys.exit(1)

    def _cleanup(self):
        try:
            if self.conveyor_ctrl:
                self.conveyor_ctrl.control_speed(0.0)
        except Exception:
            pass
        try:
            if self.traj_ctrl:
                self.traj_ctrl.stop()
        except Exception:
            pass
        if self.started:   # self.started 在 _on_start_service 里置 True
            try:
                os.makedirs(os.path.dirname(self.score_file), exist_ok=True)
                with open(self.score_file, "w") as f:
                    f.write(f"{int(self.score)}\n")
                rospy.loginfo(f"[sim] 本轮最终得分已写入: {self.score_file}（得分={int(self.score)}）")
            except Exception as e:
                rospy.logwarn(f"[sim] 写入最终得分失败: {e}")
        else:
            rospy.loginfo("[sim] 本轮未触发 start，不写入得分文件。")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed",type = int)
    args = parser.parse_args()
    seed = args.seed
    task = SimulatorTask1(seed)
    task.run()
