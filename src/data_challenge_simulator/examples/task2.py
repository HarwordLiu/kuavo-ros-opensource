import os, sys
current_dir = os.path.dirname(os.path.abspath(__file__))
data_challenge_simulator_dir = os.path.dirname(current_dir)
if data_challenge_simulator_dir not in sys.path:
    sys.path.insert(0, data_challenge_simulator_dir)

from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot, KuavoRobotState
import time
from utils.gripper_controller import GripperController
from utils.conveyor_controller import ConveyorController
from utils.trajectory_controller import TrajectoryController
from utils.object_pos import ObjectPose
from utils.object_randomizer import ObjectRandomizer
from utils.utils import Utils
import math, random
from typing import Tuple
# from scipy.spatial.transform import Rotation as R
from utils.xml_random import randomize_mjcf

def main():
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
    traj_ctrl = TrajectoryController(robot)
    obj_pos = ObjectPose()
    marker1_pos = obj_pos.wait_for_position("marker1", timeout=5.0)
    marker2_pos = obj_pos.wait_for_position("marker2", timeout=5.0)
    print("🟡marker1",marker1_pos)
    print("🟡marker2",marker2_pos)

    num = 30

    grasp_region = [
        (marker1_pos[0]-0.07, marker1_pos[0]+0.07),   # x 范围
        (marker1_pos[1]-0.07, marker1_pos[1]+0.07),   # y 范围
        (0.95, 1.02)  # z 范围
    ]
    target_region = [
        (marker2_pos[0]-0.035, marker2_pos[0]+0.035),   # x 范围
        (marker2_pos[1]-0.035, marker2_pos[1]+0.035),   # y 范围
        (0.85, 0.98)  # z 范围
    ]



    offset_grasp= [0.03, 0.075, 0]
    x_grasp = marker1_pos[0] + offset_grasp[0]
    y_grasp = marker1_pos[1] + offset_grasp[1]
    z_grasp = 1.2 + offset_grasp[2]
    ori_grasp = (-0.2984213394927986, -0.7120386976902843, 0.5975628002405355, 0.2164816317740088)

    offset_final= [0.035, 0.09, 0]
    x_final = marker2_pos[0] + offset_final[0]
    y_final = marker2_pos[1] + offset_final[1]
    z_final = 1.15 + offset_final[2]
    ori_final = (0.6865845535190419, -0.3805043451070304, -0.21774522952327083, 0.5800044045036089)


    try:
        # 随机物体位置
        y_target = random.uniform(-0.5, -0.2)  #-0.2-0.5
        random_pos = ObjectRandomizer()
        result = random_pos.randomize_object_position(
            object_name='box_grab',
            position_ranges={
                'x': [0.4, 0.55],    # x轴范围
                'y': [-0.8, -0.8],   # y轴范围  
                'z': [0.95, 0.95]     # z轴范围
            }
        )
        if result['success']:
            print(f"物体随机化成功！新位置: {result['final_position']}")
        else:
            print(f"物体随机化失败: {result['message']}")
            
        robot.control_head(yaw=0, pitch=math.radians(12))

        # 预抓位1
        q_target1 = [0, 0, 0, 0, -90, 0, 0,   80, -30, 0, -130, 45, 0, 0]
        q_list1 = Utils.interpolate_joint_trajectory(q_target1, num=num) 
            
        q_target2 = [0, 0, 0, 0, -90, 0, 0,   20, 0, 0, -120, 90, 0, 0]
        q_list2 = Utils.interpolate_joint_trajectory(q_target2, q_target1, num=num)


        print("双手移动到初始位置,传送带启动")
        
        traj_ctrl.execute_trajectory(q_list1, sleep_time=0.02)
        traj_ctrl.execute_trajectory(q_list2, sleep_time=0.02)

        time.sleep(2.5)
        conveyor_ctrl.control_speed(0.1)
        time_pause_base = 2.4

        def time_pause(y_target):
            time_pause_sec = (y_target + 0.5)*10 + time_pause_base
            return time_pause_sec
        
        time_pause_sec = time_pause(y_target=y_target)
        time.sleep(time_pause_sec)
        # 计算俯仰角并调整姿态
        curr_q_rot = robot_state.arm_joint_state().position
        l_pose_rot, r_pose_rot = robot.arm_fk(curr_q_rot)
        pos_box1_rot = obj_pos.get_position("box_grab")

        curr_pos = r_pose_rot.position

        dx = pos_box1_rot[0] - curr_pos[0] + 0.12
        dy = y_target - curr_pos[1]
        pitch_des = math.atan2(dy, dx)

        q_target_pitch = [0, 0, 0, 0, -90, 0, 0,   20, 0, math.degrees(pitch_des)*0.5, -120, 90, 0, -math.degrees(pitch_des)*0.5]
        q_list_pitch = Utils.interpolate_joint_trajectory(q_target_pitch, q_target2, num=5)
        
        traj_ctrl.execute_trajectory(q_list_pitch, sleep_time=0.01)
        time.sleep(0.5)


        curr_q = robot_state.arm_joint_state().position
        l_pose, r_pose = robot.arm_fk(curr_q)

        # 抓取 - 等待抓取姿态
        pos_box1 = obj_pos.get_position("box_grab")
        _, right_pose = Utils.wait_for_grasp_pose(
            robot=robot,
            robot_state=robot_state,
            y_grasp_right=y_target,
            quat_right=r_pose.orientation,
            v=0.08,
            mode="right",
            obj_name_right="box_grab",
            x_grasp_right=pos_box1[0]+0.11,
            z_grasp_right=pos_box1[2]+0.1,
            time_offset=0,
            move_lead_time=0.65,
            check_interval=0.0001)
        
        left_pose = [math.radians(x) for x in q_target2[0:7]]
        q_target_deg = [math.degrees(x) for x in left_pose+right_pose]
        q_list_target = Utils.interpolate_joint_trajectory(q_target_deg, q_target_pitch, num=15)
        
        traj_ctrl.execute_trajectory(q_list_target, sleep_time=0.02)
        time.sleep(0.5)

        gripper_ctrl.control_right_gripper(160)
        time.sleep(0.5)
        

        # 移动到称
        q_target3 = [80, -30, 0, -130, -87, 0, 0,   -10, -5, 25, -130, 87, 0, 0]
        q_list3 = Utils.interpolate_joint_trajectory(q_target3, q_target_deg, num=num) 

        q_target4 = [50, 0, -20, -150, -90, 0, 0,   -30, 30, 37, -70, 87, 0, 0]
        q_list4 = Utils.interpolate_joint_trajectory(q_target4, q_target3, num=num)

        traj_ctrl.execute_trajectory(q_list3, sleep_time=0.02)
        # traj_ctrl.execute_trajectory(q_list4, sleep_time=0.02)

        pose_grasp = [x_grasp, y_grasp, z_grasp]
        _, pose_grasp_right = Utils.compute_pose(
            robot,
            robot_state,
            mode="right",
            pos_right=pose_grasp,
            quat_right=ori_grasp,
        )

        pose_grasp_left = [math.radians(x) for x in q_target3[0:7]]
        pose_grasp_radian = pose_grasp_left + pose_grasp_right
        pose_grasp_deg = [math.degrees(x) for x in pose_grasp_radian]
        q_list_grasp = Utils.interpolate_joint_trajectory(pose_grasp_deg, q_target3, num=num)

        traj_ctrl.execute_trajectory(q_list_grasp, sleep_time=0.02)
        print("右手移动至称，打开夹爪")

        time.sleep(0.5)
        
        # 右夹爪释放
        gripper_ctrl.control_right_gripper(0)
        time.sleep(0.5)


        all_success = True
        pos1 = obj_pos.get_position("box_grab")
        ori1 = obj_pos.get_orientation("box_grab")
        in_region1 = Utils.is_in_target_region(pos1, grasp_region)
        is_front1, deg1  = Utils.is_front_facing(quat_xyzw=ori1,body_front_axis='z',front_world_dir='z',tol_deg=10)

        if not (in_region1 and is_front1):
            all_success = False
            print("第一次失败❌")
        elif in_region1 and is_front1:
            print("第一次成功✅")

        # 左手开始抓取
        q_target5 = [20, 0, -25, -140, -87, 0, 0,   -10, 0, 0, -140, 87, 0, 0]
        q_list5 = Utils.interpolate_joint_trajectory(q_target5, pose_grasp_deg, num=num) 
        
        print("左手抓取")
        traj_ctrl.execute_trajectory(q_list5, sleep_time=0.02)

        time.sleep(0.5)
        
        # 计算左手精确抓取位置
        curr_q1 = robot_state.arm_joint_state().position
        l_pose1, r_pose1 = robot.arm_fk(curr_q1)
        pos_box1 = obj_pos.get_position("box_grab")
        l_pose_new1 = [pos_box1[0]+0.04, pos_box1[1]-0.025, l_pose1.position[2] + robot_state.robot_position()[2]]
        pose1_left, _ = Utils.compute_pose(
            robot,
            robot_state,
            mode="left",
            pos_left=l_pose_new1,
            quat_left=l_pose1.orientation,
        )
        pose1_right1 = [math.radians(x) for x in q_target2[7:14]]
        q_target1 = pose1_left + pose1_right1
        q_target_deg1 = [math.degrees(x) for x in q_target1]
        q_list_target1 = Utils.interpolate_joint_trajectory(q_target_deg1, q_target5, num=num)
        
        # 抓取
        traj_ctrl.execute_trajectory(q_list_target1, sleep_time=0.02)

        time.sleep(0.5)
        # 左夹爪抓取 
        gripper_ctrl.control_left_gripper(160)
        print("左手放置到目标位置")

        pose_final = [x_final, y_final, z_final]
        pose_final_left, _  = Utils.compute_pose(
            robot,
            robot_state,
            mode="left",
            pos_left=pose_final,
            quat_left=ori_final,
        )

        pose_final_right = [math.radians(x) for x in q_target2[7:14]]
        pose_final_radian = pose_final_left + pose_final_right
        pose_final_deg = [math.degrees(x) for x in pose_final_radian]
        q_list_final = Utils.interpolate_joint_trajectory(pose_final_deg, q_target_deg1, num=num)

        traj_ctrl.execute_trajectory(q_list_final, sleep_time=0.02)

        time.sleep(0.5)
        
        # time.sleep(1)
        # curr_q_grasp = robot_state.arm_joint_state().position
        # l_pose_grasp, r = robot.arm_fk(curr_q_grasp)
        # print("XXXXXX",l_pose_grasp.orientation)

        # # 左夹爪释放
        gripper_ctrl.control_left_gripper(0)

        time.sleep(0.5)
        
        # 检查任务完成情况
        pos2 = obj_pos.get_position("box_grab")
        ori2 = obj_pos.get_orientation("box_grab")
        in_region2 = Utils.is_in_target_region(pos2, target_region)
        is_front2,deg2  = Utils.is_front_facing(quat_xyzw=ori2,body_front_axis='z',front_world_dir='z',tol_deg=10)

        if not (in_region2 and is_front2):
            all_success = False
            print("第二次失败❌")
        elif in_region2 and is_front2:
            print("第二次成功✅")

        # 检查任务完成情况
        if all_success:
            print("\033[92m✅ 任务成功\033[0m")
            with open("task_result.txt", "w") as f:
                f.write("success")
        else:
            print("\033[91m❌ 任务失败\033[0m")
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
    cfg = {
    "rules": [
        # ---- marker1：位置 + 颜色 ----
        {
            "select": ".//body[@name='marker1']",
            "attributes": {
                "pos": {  # 基于 (0.52, -0.50, 0.8721)
                    "per_dim": [
                        {"uniform": [0.35, 0.45]},    # x
                        {"uniform": [0.05, 0.15]},  # y
                        {"set": 0.89}               # z 固定
                    ]
                }
            }
        },
        {
            "select": ".//body[@name='marker1']/geom",
            "attributes": {
                "rgba": {"color": {"alpha": [1.0, 1.0]}}  # 随机 RGB，alpha=1
            }
        },

        # ---- marker2：位置 + 颜色 ----
        {
            "select": ".//body[@name='marker2']",
            "attributes": {
                "pos": {  # 基于 (0.42, -0.03, 0.8721)
                    "per_dim": [
                        {"uniform": [0.3, 0.50]},    # x
                        {"uniform": [0.5, 0.65]},   # y
                        {"set": 0.871}               # z 固定
                    ]
                }
            }
        },
        {
            "select": ".//body[@name='marker2']/geom",
            "attributes": {
                "rgba": {"color": {"alpha": [1.0, 1.0]}}
            }
        },

        # ---- box_grab 两个几何的颜色 ----
        {
            "select": ".//geom[@name='box_on_belt']",
            "attributes": {
                "rgba": {"color": {"alpha": [1.0, 1.0]}}
            }
        },
        {
            "select": ".//geom[@name='box_base_disk']",
            "attributes": {
                "rgba": {"color": {"alpha": [1.0, 1.0]}}
            }
        },

        # ---- 光照与视角 ----
        {
            "select": ".//visual/headlight",
            "attributes": {
                "diffuse": {"uniform": [0.2, 0.6], "ndim": 3},
                "ambient": {"per_dim": [
                {"uniform": [0.0, 0.3]},
                {"uniform": [0.0, 0.3]},
                {"uniform": [0.0, 0.3]}
                ]},
                "specular": {"uniform": [0.0, 0.2], "ndim": 3}
            }
        }
    ]
}


    n_changed = randomize_mjcf(
        in_path="/root/kuavo_ws/src/data_challenge_simulator/models/biped_s45/xml/scene2.xml",
        out_path="/root/kuavo_ws/src/data_challenge_simulator/models/biped_s45/xml/scene2.xml",
        config=cfg,
    )

    print("changed:", n_changed)
    main()