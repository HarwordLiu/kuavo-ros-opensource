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

import rospy
from std_msgs.msg import Bool

class SimulatorTask1:
    def __init__(self):
        rospy.init_node('simulator_task1', anonymous=False)

        self.pub_init = rospy.Publisher('/simulator/init', Bool, queue_size=10)
        self.pub_success = rospy.Publisher('/simulator/success',Bool, queue_size=10)

        rospy.Subscriber('/simulator/start', Bool, self._on_start)
        rospy.Subscriber('/simulator/reset', Bool, self._on_reset)

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

        # 成功状态
        self.already_reported_success = False

        # 区域阈值（你原注释内的范围）
        self.target_region = [
            (0.36, 0.62),   # x
            (-0.6, -0.32),  # y
            (0.85, 1.5)     # z
        ]

    # ========== 回调 ==========
    def _on_start(self, msg: Bool):
        if msg.data:
            rospy.loginfo("[sim] 收到 /simulator/start=True")
            self.start_evt.set()

    def _on_reset(self, msg: Bool):
        if msg.data:
            rospy.loginfo("[sim] 收到 /simulator/reset=True")
            self.reset_evt.set()

    # ========== 主流程 ==========
    def run(self):
        try:
            # 1) 初始化 SDK 与控制器
            if not KuavoSDK().Init(options=KuavoSDK.Options.WithIK):
                print("Init KuavoSDK failed, exit!")
                return  # 直接退出，deploy.py 会重启一轮

            self.robot = KuavoRobot()
            self.robot_state = KuavoRobotState()

            self.conveyor_ctrl = ConveyorController()
            self.gripper_ctrl  = GripperController()
            self.traj_ctrl     = TrajectoryController(self.robot)
            self.obj_pos       = ObjectPose()

            # 随机化物体位置（可按需保留/删除）
            random_pos = ObjectRandomizer()
            result = random_pos.randomize_object_position(
                object_name='box_grab',
                position_ranges={
                    'x': [0.8, 0.8],
                    'y': [0.45, 0.65],
                    'z': [0.95, 0.95]
                }
            )
            if result['success']:
                rospy.loginfo(f"物体随机化成功: {result['final_position']}")
            else:
                rospy.logwarn(f"物体随机化失败: {result['message']}")

            # 2) 预抓位
            num = 20
            q_target1 = [0, 0, 0, -105, -70, 0, 0,   30, 0, 0, -140, 90, 0, 0]
            q_list1 = Utils.interpolate_joint_trajectory(q_target1, num=num)

            q_target2 = [-10, 15, 25, -100, -120, 0, 0,   30, 0, 0, -120, 90, 0, 0]
            q_list2 = Utils.interpolate_joint_trajectory(q_target2, q_target1, num=num)

            q_target3 = [-10, 15, 25, -95, -180, 25, -20,   30, 0, 0, -140, 90, 0, 0]
            q_list3 = Utils.interpolate_joint_trajectory(q_target3, q_target2, num=num)

            self.traj_ctrl.execute_trajectory(q_list1, sleep_time=0.02)
            self.traj_ctrl.execute_trajectory(q_list2, sleep_time=0.02)
            self.traj_ctrl.execute_trajectory(q_list3, sleep_time=0.02)

            # 3) 发布 msg0：init=True
            rospy.loginfo("[sim] 预抓位完成，发布 /simulator/init=True")
            self.pub_init.publish(Bool(data=True))

            # 4) 等待 msg1：start=True
            rospy.loginfo("[sim] 等待 /simulator/start=True...")
            while not rospy.is_shutdown() and not self.reset_evt.is_set():
                if self.start_evt.wait(timeout=0.1):
                    break

            if self.reset_evt.is_set() or rospy.is_shutdown():
                rospy.logwarn("[sim] 等待 start 期间收到 reset / shutdown，退出")
                self._cleanup()
                sys.exit(0)

            # 5) 收到 start → 开传送带，循环上报 success=False
            rospy.loginfo("[sim] 开始传送带，持续上报 success=False")
            self.conveyor_ctrl.control_speed(-0.1)

            rate = rospy.Rate(10)  # 10Hz 上报
            self.already_reported_success = False

            while not rospy.is_shutdown() and not self.reset_evt.is_set():
                # 检查是否成功
                try:
                    pos = self.obj_pos.get_position("box_grab")  # [x, y, z]
                    in_region = Utils.is_in_target_region(pos, self.target_region)
                except Exception as e:
                    rospy.logwarn(f"[sim] 获取位置出错：{e}")
                    in_region = False

                if in_region and not self.already_reported_success:
                    rospy.loginfo("[sim] ✅ 任务成功，发布 /simulator/success=True")
                    self.pub_success.publish(Bool(data=True))
                    self.already_reported_success = True
                    # 成功就停止传送带
                    self.conveyor_ctrl.control_speed(0.0)
                else:
                    # 未成功则持续发 False
                    if not self.already_reported_success:
                        self.pub_success.publish(Bool(data=False))

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
            if self.gripper_ctrl:
                self.gripper_ctrl.stop()
        except Exception:
            pass
        try:
            if self.traj_ctrl:
                self.traj_ctrl.stop()
        except Exception:
            pass

if __name__ == "__main__":
    task = SimulatorTask1()
    task.run()
