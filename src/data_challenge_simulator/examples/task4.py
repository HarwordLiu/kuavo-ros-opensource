import os, sys
current_dir = os.path.dirname(os.path.abspath(__file__))
data_challenge_simulator_dir = os.path.dirname(current_dir)
if data_challenge_simulator_dir not in sys.path:
    sys.path.insert(0, data_challenge_simulator_dir)
from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot, KuavoPose, KuavoManipulationMpcFrame, KuavoRobotState, KuavoRobotInfo, KuavoRobotArm
import rospy
import time
from utils.gripper_controller import GripperController  # 使用高频版本
from utils.conveyor_controller import ConveyorController
from utils.trajectory_controller import TrajectoryController  # 新增的轨迹控制器
from utils.object_pos import ObjectPose
from utils.object_randomizer import ObjectRandomizer
from utils.utils import Utils
import math
from typing import Tuple
from utils.object_pos import ObjectPose
def main():


    target_poses1 = [
        [1.5, [0, 0, 0, 0, 0, 0, 0,   30, 0, 15, -130, 90, 0, 0]],
        [3.0, [0, 0, 0, 0, 0, 0, 0,   -15, 0, 25, -80, 90, 0, 0]],
    ]
    # target_pose1_1_left = Pose.from_euler(pos =(0, 0, -0.270229), euler= (0.0374, -14.5817, 0.0205), frame=Frame.BASE, degrees=True)
    # taregt_pose1_1_right = Pose.from_euler(pos =(0.5,-0.05,0.6), euler= (0,0,0), frame=Frame.BASE, degrees=True)
    # pose1_left = KuavoPose(position=target_pose1_1_left.pos, orientation=target_pose1_1_left.quat)
    # pose1_right = KuavoPose(position=taregt_pose1_1_right.pos, orientation=taregt_pose1_1_right.quat)
    times1 = [pose[0] for pose in target_poses1]
    q_frames1 = [[math.radians(angle) for angle in pose[1]] for pose in target_poses1]
    

    target_poses2 = [
        [1.0, [0, 0, 0, 0, 0, 0, 0,   -15, 15, 25, -120, 90, 0, 0]],

    ]
    
    times2 = [pose[0] for pose in target_poses2]
    q_frames2 = [[math.radians(angle) for angle in pose[1]] for pose in target_poses2]

    target_poses3 = [
        [1.0, [0, 0, 0, 0, 0, 0, 0,   -10, 0, -10, -100, 90, 0, 0]],
    ]
    
    times3 = [pose[0] for pose in target_poses3]
    q_frames3 = [[math.radians(angle) for angle in pose[1]] for pose in target_poses3]


    target_poses4 = [
        [1.0, [0, 0, 0, 0, 0, 0, 0,   20, 0, -10, -130, 90, 0, 0]],
        [2.0, [0, 0, 0, 0, 90, 0, 0,   0, 0, 0, 0, 0, 0, 0]],
    ]
    
    times4 = [pose[0] for pose in target_poses4]
    q_frames4 = [[math.radians(angle) for angle in pose[1]] for pose in target_poses4]

    target_poses5 = [
        [1.0, [30, 0, -15, -130, 90, 0, 0,   0, 0, 0, 0, 0, 0, 0]],
        [2.0, [-22, 0, -27, -72, 90, 10, 0,   0, 0, 0, 0, 0, 0, 0]],
    ]
    
    times5 = [pose[0] for pose in target_poses5]
    q_frames5 = [[math.radians(angle) for angle in pose[1]] for pose in target_poses5]


    target_poses6 = [
        [1.0, [-15, -15, -25, -120, 90, 0, 0,   0, 0, 0, 0, 0, 0, 0]],

    ]
    
    times6 = [pose[0] for pose in target_poses6]
    q_frames6 = [[math.radians(angle) for angle in pose[1]] for pose in target_poses6]


    target_poses7 = [
        [1.0, [-10, 0, 7, -100, 90, 0, 0,   0, 0, 0, -0, 0, 0, 0]],
    ]
    
    times7 = [pose[0] for pose in target_poses7]
    q_frames7 = [[math.radians(angle) for angle in pose[1]] for pose in target_poses7]


    # Initialize SDK
    if not KuavoSDK().Init(options=KuavoSDK.Options.WithIK):
        print("Init KuavoSDK failed, exit!")
        exit(1)
    # Initialize robot
    robot = KuavoRobot()
    robot_state = KuavoRobotState()

    # Initialize controllers with high-frequency publishing
    conveyor_ctrl = ConveyorController()
    gripper_ctrl = GripperController() 

    obj_pos = ObjectPose()

    randomizer = ObjectRandomizer()
    object_configs = [
        {"name": "item1",
        "position_ranges": {
                    'x': [1.33, 1.33],
                    'y': [-0.3, -0.3],
                    'z': [0.9, 0.9]
                }},

        {"name": "item2",
        "position_ranges": {
                    'x': [1.33, 1.33],
                    'y': [0.3, 0.3],
                    'z': [0.9, 0.9]
                }},

        {"name": "left_bin_A",
        "position_ranges": {
                    'x': [-1.35, -1.35],
                    'y': [0.3, 0.3],
                    'z': [0.76, 0.76]
                },
        "orientation": {'w': 0.7071, 'x': 0, 'y': 0, 'z': 0.7071}},

        {"name": "left_bin_B",
        "position_ranges": {
                    'x': [-1.35, -1.35],
                    'y': [-0.3, -0.3],
                    'z': [0.76, 0.76]
                },
        "orientation": {'w': 0.7071, 'x': 0, 'y': 0, 'z': 0.7071}},
    ]

    randomizer.randomize_multiple_objects(object_configs)

    conveyor_ctrl.control_speed(-0.1)
    num = 30

    item1_pos = obj_pos.wait_for_position("item1")
    item2_pos = obj_pos.wait_for_position("item2")
    left_bin_A_pos = obj_pos.wait_for_position("left_bin_A")
    left_bin_B_pos = obj_pos.wait_for_position("left_bin_B")

    print(item1_pos,item2_pos,left_bin_A_pos,left_bin_B_pos)
    # #=========================第一段走路与抓取=============================
    try:
        print("行走到目标区域")
        robot.stance()
        offset1 = 0.3
        robot.control_command_pose_world(0.9, item1_pos[1]+offset1, 0, 0)
        # robot.control_command_pose(0.9, 0, 0.0, 0)
        time.sleep(4)
        robot.stance()

        traj_ctrl = TrajectoryController(robot)

        print("开始抓取物体")
        q1_target1 = [0, 0, 0, 0, 0, 0, 0,   30, 0, 0, -130, 90, 0, 0]
        q1_list1 = Utils.interpolate_joint_trajectory(q1_target1, num = num)
        traj_ctrl.execute_trajectory(q1_list1, sleep_time=0.02)

        time.sleep(2.0)
        
        curr_q1 = robot_state.arm_joint_state().position
        l_pose1, r_pose1 = robot.arm_fk(curr_q1)
        r_pose_new1 = [item1_pos[0], item1_pos[1]-0.01, 0.95]
        _, pose1_right1 = Utils.compute_pose(
            robot,
            robot_state,
            mode="right",
            pos_right=r_pose_new1,
            quat_right=r_pose1.orientation,
        )
        
        pose1_left1 = [math.radians(x) for x in q1_target1[0:7]]
        q_target1 = pose1_left1 + pose1_right1
        q_target_deg1 = [math.degrees(x) for x in q_target1]
        q_list_target1 = Utils.interpolate_joint_trajectory(q_target_deg1, q1_target1, num=30)
        traj_ctrl.execute_trajectory(q_list_target1, sleep_time=0.02)

        time.sleep(0.5)
        print("关闭夹爪并提起")
        gripper_ctrl.control_right_gripper(150)
        time.sleep(0.5)


        q1_target2 = [0, 0, 0, 0, 0, 0, 0,   -15, 15, 25, -120, 90, 0, 0]
        q1_list2 = Utils.interpolate_joint_trajectory(q1_target2,q_target_deg1, num = num)
        traj_ctrl.execute_trajectory(q1_list2, sleep_time=0.02)
        time.sleep(1)
        # #=========================第一段走路与抓取=============================

        # #=========================第二段走路与放置=============================
        print("行走至放置地区")
        # robot_state.wait_for_stance()
        robot.stance()
        robot.control_command_pose_world(0.6, item1_pos[1]+offset1, 0, 0)
        time.sleep(3)
        offset2 = -0.3
        robot.control_command_pose_world(-0.85, left_bin_A_pos[1]+offset2, 0, 3.14)
        time.sleep(12)
        robot.stance()

        print("放置于右侧盒子")

        q1_target3 = [0, 0, 0, 0, 0, 0, 0,   -10, 0, -10, -100, 90, 0, 0]
        q1_list3 = Utils.interpolate_joint_trajectory(q1_target3,q1_target2, num = num)
        traj_ctrl.execute_trajectory(q1_list3, sleep_time=0.02)

        time.sleep(1)
        gripper_ctrl.control_right_gripper(0)
        time.sleep(1)
        # #=========================第二段走路与放置=============================


        # #=========================第三段走路与抓取=============================
        print("返回到目标区域")
        robot.stance()
        robot.control_command_pose_world(-0.55, left_bin_A_pos[1]+offset2, 0, 3.14)
        time.sleep(3)
        robot.stance()

        q1_target4 = [0, 0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0, 0]
        q1_list4 = Utils.interpolate_joint_trajectory(q1_target4,q1_target3, num = num)
        traj_ctrl.execute_trajectory(q1_list4, sleep_time=0.02)

        offset3 = -0.3

        robot.control_command_pose_world(0.9, item2_pos[1]+offset3, 0, 0)
        time.sleep(12)
        robot.stance()
        print("左手开始抓取")
        q1_target5 = [30, 0, 0, -130, -90, 0, 0,   0, 0, 0, 0, 0, 0, 0]
        q1_list5 = Utils.interpolate_joint_trajectory(q1_target5,q1_target4, num = num)
        traj_ctrl.execute_trajectory(q1_list5, sleep_time=0.02)

        time.sleep(2)
        
        curr_q2 = robot_state.arm_joint_state().position
        l_pose2, r_pose2 = robot.arm_fk(curr_q2)
        l_pose_new2 = [item2_pos[0], item2_pos[1]-0.025, 0.95]
        pose1_left2 , _ = Utils.compute_pose(
            robot,
            robot_state,
            mode="left",
            pos_left=l_pose_new2,
            quat_left=l_pose2.orientation,
        )
        
        pose1_right2 = [math.radians(x) for x in q1_target5[7:14]]
        q_target2 = pose1_left2 + pose1_right2
        q_target_deg2 = [math.degrees(x) for x in q_target2]
        q_list_target2 = Utils.interpolate_joint_trajectory(q_target_deg2, q1_target5, num=30)
        traj_ctrl.execute_trajectory(q_list_target2, sleep_time=0.02)

        time.sleep(0.5)
        print("关闭夹爪并提起")
        gripper_ctrl.control_left_gripper(150)

        q1_target6 = [-15, -15, -25, -120, -90, 0, 0,   0, 0, 0, 0, 0, 0, 0]
        q1_list6 = Utils.interpolate_joint_trajectory(q1_target6,q_target_deg2, num = num)
        traj_ctrl.execute_trajectory(q1_list6, sleep_time=0.02)
        time.sleep(1)
        # #=========================第三段走路与抓取=============================


        # #=========================第四段走路与放置=============================
        print("行走至放置地区")
        robot.stance()
        robot.control_command_pose_world(0.6, item2_pos[1]+offset3, 0, 0)
        time.sleep(3)
        offset4 = 0.3
        robot.control_command_pose_world(-0.85, left_bin_B_pos[1]+offset4, 0, 3.14)
        time.sleep(12)
        robot.stance()

        print("放置于左侧盒子")
        q1_target7 = [-10, 0, 7, -100, -90, 0, 0,   0, 0, 0, -0, 0, 0, 0]
        q1_list7 = Utils.interpolate_joint_trajectory(q1_target7,q1_target6, num = num)
        traj_ctrl.execute_trajectory(q1_list7, sleep_time=0.02)
        time.sleep(0.5)
        gripper_ctrl.control_left_gripper(0)
        time.sleep(1.0)
        #=========================第四段走路与放置=============================

        pos1 = obj_pos.get_position("item1")

        target_region1 = [
            (-1.535, -1.165),   # x 范围
            (-0.445, -0.155),   # y 范围
            (0.76, 1)  # z 范围
        ]

        pos2 = obj_pos.get_position("item2")
        target_region2 = [
            (-1.535, -1.165),   # x 范围
            (0.155, 0.445),   # y 范围
            (0.76, 1)  # z 范围
        ]

        if Utils.is_in_target_region(pos1, target_region1) and Utils.is_in_target_region(pos2, target_region2):
            print("✅ 任务成功")
            with open("task_result.txt", "w") as f:
                f.write("success")
        else:
            print("❌ 任务失败")
            with open("task_result.txt", "w") as f:
                f.write("fail")
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        print(f"程序执行出错: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 确保所有控制器正确停止
        print("正在停止控制器...")
        gripper_ctrl.stop()
        traj_ctrl.stop()
        print("控制器已停止")

if __name__ == "__main__":
        main()