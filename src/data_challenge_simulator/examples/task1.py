import os, sys
current_dir = os.path.dirname(os.path.abspath(__file__))
data_challenge_simulator_dir = os.path.dirname(current_dir)
if data_challenge_simulator_dir not in sys.path:
    sys.path.insert(0, data_challenge_simulator_dir)

from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot, KuavoRobotState
import time
from utils.gripper_controller import GripperController
from utils.conveyor_controller import ConveyorController
from utils.object_pos import ObjectPose
from utils.object_randomizer import ObjectRandomizer
from utils.trajectory_controller import TrajectoryController  # 新增的轨迹控制器
from utils.utils import Utils
import math
from typing import Tuple
from utils.xml_random import randomize_mjcf

def main():
    # Initialize SDK
    if not KuavoSDK().Init(options=KuavoSDK.Options.WithIK):
        print("Init KuavoSDK failed, exit!")
        exit(1)
    
    # Initialize robot
    robot = KuavoRobot()
    robot_state = KuavoRobotState()
    # Initialize controllers
    conveyor_ctrl = ConveyorController()
    gripper_ctrl = GripperController() 
    traj_ctrl = TrajectoryController(robot)
    obj_pos = ObjectPose()
    marker1_pos = obj_pos.wait_for_position("marker1", timeout=5.0)
    marker2_pos = obj_pos.wait_for_position("marker2", timeout=5.0)
    print("🟡marker1",marker1_pos)
    print("🟡marker2",marker2_pos)
    num = 20

    grasp_region = [
        (marker1_pos[0]-0.03, marker1_pos[0]+0.03),   # x 范围
        (marker1_pos[1]-0.03, marker1_pos[1]+0.03),   # y 范围
        (0.85, 1.1)  # z 范围
    ]
    target_region = [
        (marker2_pos[0]-0.035, marker2_pos[0]+0.035),   # x 范围
        (marker2_pos[1]-0.035, marker2_pos[1]+0.035),   # y 范围
        (0.85, 1.1)  # z 范围
    ]

    offset_grasp= [0.05, -0.06, 0]
    x_grasp = marker1_pos[0] + offset_grasp[0]
    y_grasp = marker1_pos[1] + offset_grasp[1]
    z_grasp = 1.25 + offset_grasp[2]
    ori_grasp = (0.3775488479595588, -0.6276236743210512, -0.626605010384081, 0.2662922300738775)

    offset_final= [0.03, -0.06, 0]
    x_final = marker2_pos[0] + offset_final[0]
    y_final = marker2_pos[1] + offset_final[1]
    z_final = 1.2 + offset_final[2]
    ori_final = (-0.6139610282920115, -0.512807918741992, 0.35133137051614377, 0.48646290948578014)

    try:
        # 随机物体位置
        random_pos = ObjectRandomizer()
        result = random_pos.randomize_object_position(
            object_name='box_grab',
            position_ranges={
                'x': [0.8, 1.0],    # x轴范围
                'y': [0.48, 0.63],   # y轴范围  
                'z': [0.95, 0.95]     # z轴范围
            }
        )
        if result['success']:
            print(f"物体随机化成功！新位置: {result['final_position']}")
        else:
            print(f"物体随机化失败: {result['message']}")



        
        robot.control_head(yaw=0, pitch=math.radians(12))

        # 预抓位
        q_target1 = [0, 0, 0, -105, -70, 0, 0,   30, 0, 0, -140, 90, 0, 0]
        q_list1 = Utils.interpolate_joint_trajectory(q_target1, num=num) 
        
        q_target2 = [-10, 15, 25, -100, -120, 0, 0,   30, 0, 0, -120, 90, 0, 0]
        q_list2 = Utils.interpolate_joint_trajectory(q_target2, q_target1, num=num)

        q_target3 = [-10, 15, 25, -95, -180, 15, -20,   30, 0, 0, -140, 90, 0, 0]
        q_list3 = Utils.interpolate_joint_trajectory(q_target3, q_target2, num=num)

        conveyor_ctrl.control_speed(-0.1)
        print("双手移动到初始位置,传送带启动")
        
        traj_ctrl.execute_trajectory(q_list1, sleep_time=0.02)
        traj_ctrl.execute_trajectory(q_list2, sleep_time=0.02)
        traj_ctrl.execute_trajectory(q_list3, sleep_time=0.02)
        time.sleep(0.5)

        # 计算新的目标位置
        curr_q = robot_state.arm_joint_state().position
        l_pose, r_pose = robot.arm_fk(curr_q)
        pos_box = obj_pos.get_position("box_grab")
        offset = 0.04
        if pos_box[1] <= 0.5:
            offset = 0.015
        elif pos_box[1] >= 0.55:
            offset = 0.055

        l_pose_new = [l_pose.position[0], pos_box[1]+offset, l_pose.position[2] + robot_state.robot_position()[2]+0.05]
        pose1_left, _ = Utils.compute_pose(
            robot,
            robot_state,
            mode="left",
            pos_left=l_pose_new,
            quat_left=l_pose.orientation,
        )
        
        pose1_right = [math.radians(x) for x in q_target3[7:14]]
        q_target = pose1_left + pose1_right
        q_target_deg = [math.degrees(x) for x in q_target]
        q_list_target = Utils.interpolate_joint_trajectory(q_target_deg, q_target3, num=num)
        
        traj_ctrl.execute_trajectory(q_list_target, sleep_time=0.02)

        time.sleep(4.8)
        gripper_ctrl.control_left_gripper(140)
        time.sleep(0.5)

        # 继续后续动作
        q_target4 = [-10, 15, 0, -130, -180, 0, 0,    30, 0, 0, -140, 90, 0, 0]
        q_list4 = Utils.interpolate_joint_trajectory(q_target4, q_target_deg, num=num)

        q_target5 = [-20, 15, -20, -120, -180, 0, 0,    30, 0, 0, -140, 90, 0, 0]
        q_list5 = Utils.interpolate_joint_trajectory(q_target5, q_target4, num=num)

        q_target6 = [0, -5, -30, -110, -160, 0, 0,  30, 0, 0, -140, 90, 0, 0]
        q_list6 = Utils.interpolate_joint_trajectory(q_target6, q_target5, num=num)

        # 使用高频轨迹控制器
        # traj_ctrl.execute_trajectory(q_list4, sleep_time=0.02)
        # traj_ctrl.execute_trajectory(q_list5, sleep_time=0.02)
        # traj_ctrl.execute_trajectory(q_list6, sleep_time=0.02)
      
        # curr_q_grasp = robot_state.arm_joint_state().position
        # l_pose_grasp, _ = robot.arm_fk(curr_q_grasp)
        # print("XXXXXX",l_pose_grasp.orientation)

        pose_grasp = [x_grasp, y_grasp, z_grasp]
        pose_grasp_left,_ = Utils.compute_pose(
            robot,
            robot_state,
            mode="left",
            pos_left=pose_grasp,
            quat_left=ori_grasp,
        )

        pose_grasp_right = [math.radians(x) for x in q_target_deg[7:14]]
        pose_grasp_radian = pose_grasp_left + pose_grasp_right
        pose_grasp_deg = [math.degrees(x) for x in pose_grasp_radian]
        q_list_grasp = Utils.interpolate_joint_trajectory(pose_grasp_deg, q_target_deg, num=30)

        traj_ctrl.execute_trajectory(q_list_grasp, sleep_time=0.02)

        time.sleep(0.5)
        gripper_ctrl.control_left_gripper(0)  # 高频发布
        time.sleep(0.5)

        # 判定是否正确翻面以及是否在区域内
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

        # 右臂动作
        q_target7 = [-10, 5, 0, -105, -180, 25, -20,     15, 0, 20, -140, 90, 0, 0]
        q_list7 = Utils.interpolate_joint_trajectory(q_target7, q_target6, num=num)

        q_target8 = [-10, 5, 0, -105, -180, 25, -20,         10, 0, 20, -110, 90, 0, -15]
        q_list8 = Utils.interpolate_joint_trajectory(q_target8, q_target7, num=num)

        traj_ctrl.execute_trajectory(q_list7, sleep_time=0.02)
        # traj_ctrl.execute_trajectory(q_list8, sleep_time=0.02)

        time.sleep(0.5)
        # curr_q_grasp = robot_state.arm_joint_state().position
        # _,r_pose_grasp = robot.arm_fk(curr_q_grasp)
        # print("XXXXXX",r_pose_grasp.orientation)

        # 计算右臂新位置
        curr_q1 = robot_state.arm_joint_state().position
        l_pose1, r_pose1 = robot.arm_fk(curr_q1)
        pos_box1 = obj_pos.get_position("box_grab")
        r_pose_new1 = [pos_box1[0]+0.1, pos_box1[1]+0.09, 1.05]
        _, pose1_right = Utils.compute_pose(
            robot,
            robot_state,
            mode="right",
            pos_right=r_pose_new1,
            quat_right=(-0.32003473385844655, -0.6252743784873512, 0.6065302238876438, 0.3724658484240057),
        )
        
        pose1_left1 = [math.radians(x) for x in q_target7[0:7]]
        q_target1 = pose1_left1 + pose1_right
        q_target_deg1 = [math.degrees(x) for x in q_target1]
        q_list_target1 = Utils.interpolate_joint_trajectory(q_target_deg1, q_target7, num=30)
        
        # 使用高频轨迹控制器
        traj_ctrl.execute_trajectory(q_list_target1, sleep_time=0.02)

        time.sleep(0.5)
        gripper_ctrl.control_right_gripper(140)  # 高频发布
        time.sleep(0.5)

        # 最后的动作
        q_target10 = [-10, 5, 0, -95, -180, 25, -20,                -45, 0, 30, -100, 90, 0, 0]
        q_list10 = Utils.interpolate_joint_trajectory(q_target10, q_target_deg1, num=num)

        q_target11 = [-10, 5, 0, -95, -180, 25, -20,                -35, -10, -15, -80, 90, 0, 0]
        q_list11 = Utils.interpolate_joint_trajectory(q_target11, q_target10, num=num)

        q_target12 = [-10, 5, 0, -95, -180, 25, -20,                30, 0, 0, -140, 90, 0, 0]
        q_list12 = Utils.interpolate_joint_trajectory(q_target12, q_target11, num=num)

        # time.sleep(1)
        # curr = robot_state.arm_joint_state().position
        # l, r = robot.arm_fk(curr)
        # print("XXXXXX",r.orientation)


        curr_q_final = robot_state.arm_joint_state().position
        _, r_pose_final = robot.arm_fk(curr_q_final)
        pose_final = [x_final, y_final, z_final]
        _, pose_final_right = Utils.compute_pose(
            robot,
            robot_state,
            mode="right",
            pos_right=pose_final,
            quat_right=ori_final,
        )

        pose_final_left = [math.radians(x) for x in q_target_deg1[0:7]]
        pose_final_radian = pose_final_left + pose_final_right
        pose_final_deg = [math.degrees(x) for x in pose_final_radian]
        q_list_final = Utils.interpolate_joint_trajectory(pose_final_deg, q_target_deg1, num=30)

        traj_ctrl.execute_trajectory(q_list_final, sleep_time=0.02)


        time.sleep(0.5)
        gripper_ctrl.control_right_gripper(0)  # 高频发布
        time.sleep(1.0)
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
                            {"uniform": [0.48, 0.52]},    # x
                            {"uniform": [-0.05, 0]},  # y
                            {"set": 0.8721}               # z 固定
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
                            {"uniform": [0.48, 0.56]},    # x
                            {"uniform": [-0.58, -0.48]},   # y
                            {"set": 0.8721}               # z 固定
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
        in_path="/root/kuavo_ws/src/data_challenge_simulator/models/biped_s45/xml/scene1.xml",
        out_path="/root/kuavo_ws/src/data_challenge_simulator/models/biped_s45/xml/scene1.xml",
        config=cfg,
    )

    print("changed:", n_changed)
    main()