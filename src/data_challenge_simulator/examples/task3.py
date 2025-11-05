import os, sys
current_dir = os.path.dirname(os.path.abspath(__file__))
data_challenge_simulator_dir = os.path.dirname(current_dir)
if data_challenge_simulator_dir not in sys.path:
    sys.path.insert(0, data_challenge_simulator_dir)

from kuavo_humanoid_sdk import KuavoSDK, KuavoRobot, KuavoRobotState
import time
from utils.gripper_controller import GripperController  # 使用高频版本
from utils.conveyor_controller import ConveyorController
from utils.trajectory_controller import TrajectoryController  # 新增的轨迹控制器
from utils.object_pos import ObjectPose
from utils.object_randomizer import ObjectRandomizer
from utils.utils import Utils
import math,random
from typing import Tuple
import mujoco
from utils.xml_random import randomize_mjcf
# from scipy.spatial.transform import Rotation as R

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
    gripper_ctrl = GripperController()  # 100Hz gripper commands
    traj_ctrl = TrajectoryController(robot)  # 100Hz trajectory commands
    obj_pos = ObjectPose()
    
    num = 20
    # 定义随机范围
    REGIONS = {
        "shampoo1": {"x": (0.13, 0.35), "y": (-0.18, -0.1), "z": (0.69, 0.69)},  # 区域 A 23-28  17-15
        "shampoo2": {"x": (0.13, 0.35), "y": (-0.33, -0.25), "z": (0.69, 0.69)},  # 区域 B 3-27
        "shampoo3": {"x": (0.13, 0.35), "y": (-0.46, -0.38), "z": (0.69, 0.69)},  # 区域 C 42-4
    }
    FRONT = [0.5, -0.5, 0.5, -0.5] 
    BACK  = [0.5,  0.5, 0.5,  0.5]

    randomizer = ObjectRandomizer()

    # 随机选择两个物体为正面
    all_objects = list(REGIONS.keys())
    front_objects = random.sample(all_objects, 1)
    back_objects = [obj for obj in all_objects if obj not in front_objects]
    object_configs = []
    for name, region in REGIONS.items():
        r = region
        x, y, z = random.uniform(*r["x"]), random.uniform(*r["y"]), random.uniform(*r["z"]),
        
        if name in front_objects:
            qw, qx, qy, qz = FRONT
        else:
            qw, qx, qy, qz = BACK

        object_configs.append({
            "name": name,
            "position_ranges": {
                "x": list(region["x"]),
                "y": list(region["y"]),
                "z": list(region["z"]),
            },
            "orientation": {  # xyzw 顺序
                "x": qx, "y": qy, "z": qz, "w": qw,
            },
        })

    randomizer.randomize_multiple_objects(object_configs)
    obj1 = back_objects[0]
    obj2 = back_objects[1]
    obj3 = front_objects[0]

    marker1_pos = obj_pos.wait_for_position("marker1", timeout=5.0)

    target_region = [
    (marker1_pos[0]-0.18, marker1_pos[0]+0.18),   # x 范围
    (marker1_pos[1]-0.18, marker1_pos[1]+0.18),   # y 范围
    (0.67, 0.69)  # z 范围
    ]

    # target_region = [
    #     (0.16, 0.50),
    #     (0.21, 0.58),
    #     (0.6, 1.00),
    # ]

    try:
        robot.control_head(yaw=0, pitch=math.radians(12))
        ### 抓取第一个反面物体
        ##  到达预抓位 - 使用高频轨迹控制器
        q1_target1 = [60, 0, 0, -100, 0, 0, 0,   70, 0, 0, -135, 70, 40, 0]
        q1_list1 = Utils.interpolate_joint_trajectory(q1_target1, num = num) 
            
        q1_target2 = [30, 0, 0, -140, 70, 0, 0,   10, 5, 0, -130, 90, 75, 0]
        q1_list2 = Utils.interpolate_joint_trajectory(q1_target2, q1_target1, num = num)

        print("移动到第一个物体预抓位")
        # 使用高频轨迹控制器
        traj_ctrl.execute_trajectory(q1_list1, sleep_time=0.02)
        traj_ctrl.execute_trajectory(q1_list2, sleep_time=0.02)
        time.sleep(0.5)

        conveyor_ctrl.control_speed(-0.1)
        time.sleep(1.5)
        # ## 抓取第一个物体
        # # 先到达正上方
        curr_q1 = robot_state.arm_joint_state().position
        l_pose1, r_pose1 = robot.arm_fk(curr_q1)
        pos_obj1 = obj_pos.get_position(obj1)
        offset1_x = 0.075
        offset1_y = -0.016
        offset1_z = 0.05
        r_pose1_new1 = [pos_obj1[0], pos_obj1[1], pos_obj1[2]+offset1_z+0.1]
        r_pose1_new2 = [pos_obj1[0]+offset1_x, pos_obj1[1]+offset1_y, pos_obj1[2]+offset1_z] 

        [], pose1_right1= Utils.compute_pose(
            robot,
            robot_state,
            mode="right",
            pos_right=r_pose1_new1,
            quat_right=r_pose1.orientation,
        )
        
        pose1_left = [math.radians(x) for x in q1_target2[0:7]]
        q_obj1_1 = pose1_left+pose1_right1
        q_obj1_deg_1 = [math.degrees(x) for x in q_obj1_1]
        q_list_obj1_1 = Utils.interpolate_joint_trajectory(q_obj1_deg_1, q1_target2, num = num)
        
        print("移动到第一个物体正上方")
        traj_ctrl.execute_trajectory(q_list_obj1_1, sleep_time=0.02)
        time.sleep(0.5)
        
        # 再下去抓
        curr_q1_2 = robot_state.arm_joint_state().position
        l_pose2_2, r_pose1_2 = robot.arm_fk(curr_q1_2)
        [], pose1_right2= Utils.compute_pose(
            robot,
            robot_state,
            mode="right",
            pos_right=r_pose1_new2,
            quat_right=r_pose1_2.orientation,
        )
        
        pose1_left = [math.radians(x) for x in q1_target2[0:7]]
        q_obj1_2 = pose1_left+pose1_right2
        q_obj1_deg_2 = [math.degrees(x) for x in q_obj1_2]
        q_list_obj1_2 = Utils.interpolate_joint_trajectory(q_obj1_deg_2, q_obj1_deg_1, num = num)

        print("下降到第一个物体抓取位置")
        traj_ctrl.execute_trajectory(q_list_obj1_2, sleep_time=0.02)

        time.sleep(0.5)
        # 右夹爪抓取 - 高频发布
        gripper_ctrl.control_right_gripper(130)
        time.sleep(0.5)

        # 后续动作 - 使用高频轨迹控制器
        q1_target3 = [-10, 20, -40, -110, -40, 0, -40,   -15, 0, 30, -120, 90, 20, -40]
        q1_list3 = Utils.interpolate_joint_trajectory(q1_target3,q_obj1_deg_2, num = num) 
            
        q1_target4 = [-25, 13, -35, -100, -40, 0, -40,   -12, 10, 30, -100, 90, -10, -40]
        q1_list4 = Utils.interpolate_joint_trajectory(q1_target4, q1_target3, num = 30)

        q1_target5 = [-30, 20, -35, -100, -40, -10, -40,   -12, 10, 30, -80, 90, -10, -40]
        q1_list5 = Utils.interpolate_joint_trajectory(q1_target5,q1_target4, num = num) 
            
            
        q1_target7 = [-10, 0, 0, -70, -120, -50, -10,   70, 0, 0, -135, 70, 40, 0]
        q1_list7 = Utils.interpolate_joint_trajectory(q1_target7, q1_target5, num = num)

        q1_target8 = [30, 0, 0, -140, -90, 0, 0,   70, 0, 0, -135, 70, 40, 0]
        q1_list8 = Utils.interpolate_joint_trajectory(q1_target8, q1_target7, num = num)

        print("第一个物体翻面操作")
        traj_ctrl.execute_trajectory(q1_list3, sleep_time=0.02)
        time.sleep(0.5)
        traj_ctrl.execute_trajectory(q1_list4, sleep_time=0.02)
        time.sleep(0.5)
        
        print("左手关闭夹爪")
        gripper_ctrl.control_left_gripper(130)
        gripper_ctrl.control_right_gripper(130)
        time.sleep(0.5)
        
        print("右手张开夹爪")
        gripper_ctrl.control_left_gripper(130)
        gripper_ctrl.control_right_gripper(0)

        traj_ctrl.execute_trajectory(q1_list5, sleep_time=0.02)
        traj_ctrl.execute_trajectory(q1_list7, sleep_time=0.02)
        
        print("放置于目标区域")
        time.sleep(1)
        gripper_ctrl.control_left_gripper(0)
        time.sleep(0.5)

        traj_ctrl.execute_trajectory(q1_list8, sleep_time=0.02)
        time.sleep(0.5)

        # 判定是否正确翻面以及是否在区域内
        all_success = True

        pos1 = obj_pos.get_position(obj1)
        ori1 = obj_pos.get_orientation(obj1)

        in_region1 = Utils.is_in_target_region(pos1, target_region)
        is_front1, deg1  = Utils.is_front_facing(quat_xyzw=ori1,body_front_axis='-y',front_world_dir='z',tol_deg=10)

        if not (in_region1 and is_front1):
            all_success = False
            print("第一次失败❌")
        elif in_region1 and is_front1:
            print("第一次成功✅")

        # ### 抓取第二个反面物体
        # ##  到达预抓位 - 使用高频轨迹控制器
        q2_target1 = [10, 0, 0, -130, 70, 0, 0,   70, 0, 0, -135, 70, 40, 0]
        q2_list1 = Utils.interpolate_joint_trajectory(q2_target1,q1_target8, num = num) 
            
        q2_target2 = [10, 0, 0, -130, 70, 0, 0,   10, 5, 0, -135, 90, 75, 0]
        q2_list2 = Utils.interpolate_joint_trajectory(q2_target2, q2_target1, num = num)

        print("移动到第二个物体预抓位")
        traj_ctrl.execute_trajectory(q2_list1, sleep_time=0.02)
        traj_ctrl.execute_trajectory(q2_list2, sleep_time=0.02)
        time.sleep(0.5)

        ## 抓取第二个物体
        curr_q2 = robot_state.arm_joint_state().position
        l_pose2, r_pose2 = robot.arm_fk(curr_q2)
        pos_obj2 = obj_pos.get_position(obj2)

        offset2_x = 0.075
        offset2_y = -0.02
        offset2_z = 0.046
        r_pose2_new1 = [pos_obj2[0], pos_obj2[1], pos_obj2[2]+offset2_z+0.1]
        r_pose2_new2 = [pos_obj2[0]+offset2_x, pos_obj2[1]+offset2_y, pos_obj2[2]+offset2_z] 

        [], pose2_right1= Utils.compute_pose(
            robot,
            robot_state,
            mode="right",
            pos_right=r_pose2_new1,
            quat_right=r_pose2.orientation,
        )

        pose2_left = [math.radians(x) for x in q2_target2[0:7]]
        q_obj2_1 = pose2_left+pose2_right1
        q_obj2_deg_1 = [math.degrees(x) for x in q_obj2_1]
        q_list_obj2_1 = Utils.interpolate_joint_trajectory(q_obj2_deg_1, q2_target2, num = num)


        print("下降到第二个物体抓取位置")
        traj_ctrl.execute_trajectory(q_list_obj2_1, sleep_time=0.02)
        time.sleep(0.5)

        #再下去抓
        curr_q2_2 = robot_state.arm_joint_state().position
        l_pose2_2, r_pose2_2 = robot.arm_fk(curr_q2_2)
        [], pose2_right2= Utils.compute_pose(
            robot,
            robot_state,
            mode="right",
            pos_right=r_pose2_new2,
            quat_right=r_pose2_2.orientation,
        )
        
        q_obj2_2 = pose2_left+pose2_right2
        q_obj2_deg_2 = [math.degrees(x) for x in q_obj2_2]
        q_list_obj2_2 = Utils.interpolate_joint_trajectory(q_obj2_deg_2, q_obj2_deg_1, num = num)

        traj_ctrl.execute_trajectory(q_list_obj2_2, sleep_time=0.02)

        time.sleep(0.5)
        # 右夹爪抓取 - 高频发布
        gripper_ctrl.control_right_gripper(130)
        time.sleep(0.5)

        #后续动作 - 使用高频轨迹控制器
        q2_target3 = [-10, 20, -40, -110, -40, 0, -40,   -15, 0, 30, -120, 90, 20, -40]
        q2_list3 = Utils.interpolate_joint_trajectory(q2_target3,q_obj2_deg_2, num = num) 
            
        q2_target4 = [-25, 13, -35, -100, -40, 0, -40,   -12, 10, 30, -100, 90, -10, -40]
        q2_list4 = Utils.interpolate_joint_trajectory(q2_target4, q2_target3, num = 30)

        q2_target5 = [-30, 20, -35, -100, -40, 0, -40,   -12, 10, 30, -80, 90, -10, -40]
        q2_list5 = Utils.interpolate_joint_trajectory(q2_target5,q2_target4, num = num) 
            
        q2_target6 = [-50, 20, -10, -80, -90, 0, -40,   60, 0, 10, -100, 90, 10, 0]
        q2_list6 = Utils.interpolate_joint_trajectory(q2_target6, q2_target5, num = num)
            
        q2_target7 = [-10, 10, 15, -70, -90, -50, -40,   70, 0, 0, -135, 70, 40, 0]
        q2_list7 = Utils.interpolate_joint_trajectory(q2_target7, q2_target6, num = num)

        print("第二个物体翻面操作")
        traj_ctrl.execute_trajectory(q2_list3, sleep_time=0.02)
        time.sleep(0.5)
        traj_ctrl.execute_trajectory(q2_list4, sleep_time=0.02)
        time.sleep(0.5)
        
        print("左手关闭夹爪")
        gripper_ctrl.control_left_gripper(130)
        gripper_ctrl.control_right_gripper(130)
        time.sleep(0.5)
        
        print("右手张开夹爪")
        gripper_ctrl.control_left_gripper(130)
        gripper_ctrl.control_right_gripper(0)

        traj_ctrl.execute_trajectory(q2_list5, sleep_time=0.02)
        traj_ctrl.execute_trajectory(q2_list6, sleep_time=0.02)
        traj_ctrl.execute_trajectory(q2_list7, sleep_time=0.02)
        
        print("放置于目标区域")
        time.sleep(0.5)
        gripper_ctrl.control_left_gripper(0)
        gripper_ctrl.control_right_gripper(0)
        time.sleep(0.5)

        # 判定是否正确翻面以及是否在区域内
        pos2 = obj_pos.get_position(obj2)
        ori2 = obj_pos.get_orientation(obj2)

        in_region2 = Utils.is_in_target_region(pos2, target_region)
        is_front2,deg2  = Utils.is_front_facing(quat_xyzw=ori2,body_front_axis='-y',front_world_dir='z',tol_deg=10)

        if not (in_region2 and is_front2):
            all_success = False
            print("第二次失败❌")
        elif in_region2 and is_front2:
            print("第二次成功✅")
        


        ### 抓取第一个正面物体
        ##  到达预抓位 - 使用高频轨迹控制器
        q3_target1 = [30, 0, 0, -140, -90, 0, 0,   70, 0, 0, -135, 70, 40, 0]
        q3_list1 = Utils.interpolate_joint_trajectory(q3_target1,q2_target7, num = num) 
            
        q3_target2 = [30, 0, 0, -140, -90, 0, 0,   10, 5, 0, -130, 90, 75, 0]
        q3_list2 = Utils.interpolate_joint_trajectory(q3_target2, q3_target1, num = num)

        print("移动到第三个物体预抓位")
        # 使用高频轨迹控制器
        traj_ctrl.execute_trajectory(q3_list1, sleep_time=0.02)
        traj_ctrl.execute_trajectory(q3_list2, sleep_time=0.02)
        time.sleep(0.5)

        # 先到达正上方
        curr_q3 = robot_state.arm_joint_state().position
        l_pose3, r_pose3 = robot.arm_fk(curr_q3)
        pos_obj3 = obj_pos.get_position(obj3)
        offset3_x = 0.07
        offset3_y = -0.016
        offset3_z = 0.03
        r_pose3_new1 = [pos_obj3[0], pos_obj3[1], pos_obj3[2]+offset3_z+0.1]
        r_pose3_new2 = [pos_obj3[0]+offset3_x, pos_obj3[1]+offset3_y, pos_obj3[2]+offset3_z] 

        [], pose3_right1= Utils.compute_pose(
            robot,
            robot_state,
            mode="right",
            pos_right=r_pose3_new1,
            quat_right=r_pose3.orientation,
        )
        
        pose3_left = [math.radians(x) for x in q3_target2[0:7]]
        q_obj3_1 = pose3_left+pose3_right1
        q_obj3_deg_1 = [math.degrees(x) for x in q_obj3_1]
        q_list_obj3_1 = Utils.interpolate_joint_trajectory(q_obj3_deg_1, q3_target2, num = num)
        
        print("移动到第一个物体正上方")
        traj_ctrl.execute_trajectory(q_list_obj3_1, sleep_time=0.02)
        time.sleep(0.5)
        
        # 再下去抓
        curr_q3_2 = robot_state.arm_joint_state().position
        l_pose3_2, r_pose3_2 = robot.arm_fk(curr_q3_2)
        [], pose3_right2= Utils.compute_pose(
            robot,
            robot_state,
            mode="right",
            pos_right=r_pose3_new2,
            quat_right=r_pose3_2.orientation,
        )
        
        pose3_left = [math.radians(x) for x in q3_target2[0:7]]
        q_obj3_2 = pose3_left+pose3_right2
        q_obj3_deg_2 = [math.degrees(x) for x in q_obj3_2]
        q_list_obj3_2 = Utils.interpolate_joint_trajectory(q_obj3_deg_2, q_obj3_deg_1, num = num)

        print("下降到第一个物体抓取位置")
        traj_ctrl.execute_trajectory(q_list_obj3_2, sleep_time=0.02)

        time.sleep(0.5)
        # 右夹爪抓取 - 高频发布
        gripper_ctrl.control_right_gripper(130)
        time.sleep(0.5)

        q3_target3 = [-50, 20, -75, -100, -90, -60, 0,   -50, -20, 90, -110, 90, 75, 25]
        q3_list3 = Utils.interpolate_joint_trajectory(q3_target3,q_obj3_deg_1, num = num) 
            
        q3_target4 = [-50, 5, -80, -90, -90, -50, 0,   -50, -15, 90, -110, 90, 75, 25]
        q3_list4 = Utils.interpolate_joint_trajectory(q3_target4, q3_target3, num = num)

        traj_ctrl.execute_trajectory(q3_list3, sleep_time=0.02)
        time.sleep(0.5)
        traj_ctrl.execute_trajectory(q3_list4, sleep_time=0.02)

        time.sleep(0.5)
        # 右夹爪抓取 - 高频发布
        gripper_ctrl.control_left_gripper(130)
        gripper_ctrl.control_right_gripper(130)
        time.sleep(0.5)

        gripper_ctrl.control_left_gripper(130)
        gripper_ctrl.control_right_gripper(0)
        time.sleep(0.5)

        q3_target5 = [-50, 20, -80, -90, -90, 0, 0,   10, 5, 0, -130, 90, 75, 0]
        q3_list5 = Utils.interpolate_joint_trajectory(q3_target5,q3_target4, num = num) 
            
        q3_target6 = [-50, 20, -10, -80, -90, 0, -40,   0, 0, 0, -130, 90, 0, 0]
        q3_list6 = Utils.interpolate_joint_trajectory(q3_target6, q3_target5, num = num)

        q3_target7 = [-10, 25, 15, -70, -90, -50, -40,   0, 0, 0, -130, 90, 0, 0]
        q3_list7 = Utils.interpolate_joint_trajectory(q3_target7, q3_target6, num = num)

        traj_ctrl.execute_trajectory(q3_list5, sleep_time=0.02)
        traj_ctrl.execute_trajectory(q3_list6, sleep_time=0.02)
        traj_ctrl.execute_trajectory(q3_list7, sleep_time=0.02)
        time.sleep(0.5)
        gripper_ctrl.control_left_gripper(0)
        time.sleep(0.5)

        pos3 = obj_pos.get_position(obj3)
        ori3 = obj_pos.get_orientation(obj3)

        in_region3 = Utils.is_in_target_region(pos3, target_region)
        is_front3,deg3  = Utils.is_front_facing(quat_xyzw=ori3,body_front_axis='-y',front_world_dir='z',tol_deg=10)

        if not (in_region3 and is_front3):
            all_success = False
            print("第三次失败❌")
        elif in_region3 and is_front3:
            print("第三次成功✅")

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
        {
            "select": ".//body[@name='table']/geom[@name='table_top']",
            "attributes": {
                "rgba": {"color": {"alpha": [1.0, 1.0]}}
            }
        },

        {
            "select": ".//body[@name='marker1']/geom",
            "attributes": {
                "rgba": {"color": {"alpha": [1.0, 1.0]}}
            }
        },

        # ---- 光照与视角 ----
        {
            "select": ".//visual/headlight",
            "attributes": {
                "diffuse": {"uniform": [0.1, 0.8], "ndim": 3},
                "ambient": {"per_dim": [
                {"uniform": [0.0, 0.5]},
                {"uniform": [0.0, 0.5]},
                {"uniform": [0.0, 0.5]}
                ]},
                "specular": {"uniform": [0.0, 0.4], "ndim": 3}
            }
        }
    ]
}


    n_changed = randomize_mjcf(
        in_path="/root/kuavo_ws/src/data_challenge_simulator/models/biped_s45/xml/scene3.xml",
        out_path="/root/kuavo_ws/src/data_challenge_simulator/models/biped_s45/xml/scene3.xml",
        config=cfg,
    )

    print("changed:", n_changed)
    main()