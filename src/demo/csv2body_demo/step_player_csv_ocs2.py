#!/usr/bin/env python3
import rospy
import json
import numpy as np
import os
import time
from kuavo_msgs.msg import footPose
from kuavo_msgs.msg import footPoseTargetTrajectories, armTargetPoses
from kuavo_msgs.msg import gaitTimeName
from kuavo_msgs.srv import changeArmCtrlMode
from ocs2_msgs.msg import mpc_observation
import csv
import math
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import argparse
from std_msgs.msg import Float64MultiArray

class ActionPlayer:
    """动作播放器，用于控制机器人执行预定义的动作序列"""
    
    def __init__(self):
        # 初始化ROS节点
        if not rospy.core.is_initialized():
            rospy.init_node('action_player', anonymous=True)
        
        # 创建发布者
        self.foot_pose_pub = rospy.Publisher(
            '/humanoid_mpc_foot_pose_target_trajectories', 
            footPoseTargetTrajectories, 
            queue_size=10
        )
        self.arm_target_pub = rospy.Publisher(
            '/kuavo_arm_target_poses',
            armTargetPoses,
            queue_size=10
        )
        
        # 等待手臂控制模式服务
        rospy.loginfo("等待手臂控制模式服务...")
        rospy.wait_for_service('/humanoid_change_arm_ctrl_mode')
        self.change_arm_mode = rospy.ServiceProxy('/arm_traj_change_mode', changeArmCtrlMode)
        rospy.loginfo("手臂控制模式服务已就绪")
        
        # 动作数据
        self.step_control = []
        
        # MPC时间
        self.mpc_time = None
        self.mpc_time_received = False
        
        # 步态执行时间
        self.gait_start_time = None
        self.gait_start_time_received = False
        
        # 订阅MPC观测话题
        self.mpc_obs_sub = rospy.Subscriber(
            '/humanoid_mpc_observation',
            mpc_observation,
            self.mpc_observation_callback
        )
        
        # 订阅步态时间话题
        self.gait_time_sub = rospy.Subscriber(
            '/humanoid_mpc_gait_time_name',
            gaitTimeName,
            self.gait_time_callback
        )

        # mode_3 累积时间
        self.mode_3_t = 0
        
    def mpc_observation_callback(self, msg):
        """MPC观测回调函数"""
        self.mpc_time = msg.time
        self.mpc_time_received = True
        
    def gait_time_callback(self, msg):
        """步态时间回调函数"""
        if self.mpc_time_received and msg.gait_name == "custom_gait":
            self.gait_start_time = msg.start_time
            self.gait_start_time_received = True
            rospy.loginfo(f"收到步态开始时间: {self.gait_start_time}, MPC当前时间: {self.mpc_time}")
    
    def load_action_with_csv(self, csv_file):
        """从CSV文件加载手臂数据"""
        start_time = time.time()
        try:
            with open(csv_file, 'r') as f:
                reader = csv.reader(f)
                for row in reader:
                    # 解析每一行数据
                    ft = float(row[0])
                    mode = int(float(row[1]))
                    torso_pose = list(map(float, row[2:6]))  # 躯干位姿 (x, y, z, yaw)
                    left_foot_pose = list(map(float, row[6:9]))  # 左脚位姿 (x, y, yaw)
                    right_foot_pose = list(map(float, row[9:12]))  # 右脚位姿 (x, y, yaw)
                    arm_actions = list(map(float, row[12:26]))  # 手臂动作 (14个数据)

                    # 存储到动作数据中
                    self.step_control.append({
                        'time': ft,
                        'mode': mode,
                        'torso_pose': torso_pose,
                        'left_foot_pose': left_foot_pose,
                        'right_foot_pose': right_foot_pose,
                        'arm_actions': arm_actions
                    })
            print(f"加载CSV文件完成, 耗时: {time.time() - start_time:.2f}秒")
            return True
        except Exception as e:
            rospy.logerr(f"加载CSV文件失败: {str(e)}")
            return False

    def generate_foot_trajectory_with_csv(self):
        """生成足部轨迹，使用累积时间"""
        if not self.step_control:
            rospy.logerr("未加载姿态控制数据")
            return None
            
        msg = footPoseTargetTrajectories()
        msg.timeTrajectory = []
        msg.footIndexTrajectory = []
        msg.footPoseTrajectory = []
        msg.swingHeightTrajectory = []  # 初始化摆动高度轨迹
        
        initial_pos = None

        # 上一帧的左脚和右脚位置
        first_frame = self.step_control[0]  # 获取第一帧数据
        left_foot_pose = first_frame['left_foot_pose']  # 取出 left_foot_pose
        right_foot_pose = first_frame['right_foot_pose']  # 取出 right_foot_pose
        prev_left_foot_pos = left_foot_pose
        prev_right_foot_pos = right_foot_pose

        # 记录第一个躯干高度
        first_torso_height = None

        for i, step in enumerate(self.step_control):

            time = step['time'] + self.mode_3_t
            mode = step['mode']
            torso_pose = step['torso_pose']
            left_foot_pose = step['left_foot_pose']
            right_foot_pose = step['right_foot_pose']

            # 如果是第一帧，记录第一个躯干高度
            if first_torso_height is None:
                first_torso_height = torso_pose[2]
            
            # 计算当前躯干高度相对于第一个高度的偏移量
            height_offset = torso_pose[2] - first_torso_height
            height_offset = 0

            # 检查是否需要跳过当前数据
            skip_current = False

            # 模式 1 和模式 2 需要检查后续数据
            if mode == 1 or mode == 2:
                # 检查后续一个数据是否存在相同模式
                if i + 1 < len(self.step_control) and self.step_control[i + 1]['mode'] == mode:
                    skip_current = True

            if skip_current:
                continue  # 跳过当前数据

            # 根据模式生成控制指令
            if mode == 0:  # 双脚支撑 (SS)
                if prev_left_foot_pos is not None and prev_right_foot_pos is not None:
                    # 将躯干的 x-y 坐标映射到当前帧左脚位置与上一帧右脚位置的连线上
                    # print(f"prev_torso_pose: {torso_pose}")
                    projected_torso_pos = self.project_torso_to_line(
                        torso_pose[:2],  # 躯干 x-y
                        torso_pose[3],  # 躯干 yaw
                        prev_left_foot_pos[:2],  # 当前帧左脚 x-y
                        prev_right_foot_pos[:2],  # 上一帧右脚 x-y
                        offset=0.03
                    )
                    torso_pose[0] = projected_torso_pos[0]
                    torso_pose[1] = projected_torso_pos[1]
                    # print(f"projected_torso_pos: {projected_torso_pos}")
                msg.timeTrajectory.append(time)
                msg.footIndexTrajectory.append(2)  # 双脚支撑
                msg.swingHeightTrajectory.append(0.0)  # 无摆动高度
                
                # 创建脚部位姿消息
                foot_pose = footPose()
                foot_pose.footPose = [0, 0, 0, 0]  # 双脚支撑，脚部位置不变
                foot_pose.torsoPose = [
                    torso_pose[0],  # x
                    torso_pose[1],  # y
                    height_offset,  # z (使用第一个高度作为参考)
                    torso_pose[3]  # yaw
                ]
                msg.footPoseTrajectory.append(foot_pose)

            elif mode == 2:  # 左脚摆动 (FS)
                if prev_right_foot_pos is not None:
                    # 将躯干的 x-y 坐标映射到当前帧左脚位置与上一帧右脚位置的连线上
                    projected_torso_pos = self.project_torso_to_line(
                        torso_pose[:2],  # 躯干 x-y
                        torso_pose[3],  # 躯干 yaw
                        left_foot_pose[:2],  # 当前帧左脚 x-y
                        prev_right_foot_pos[:2],  # 上一帧右脚 x-y
                        offset=0.03
                    )
                    torso_pose[0] = projected_torso_pos[0]
                    torso_pose[1] = projected_torso_pos[1]
                msg.timeTrajectory.append(time)
                msg.footIndexTrajectory.append(0)  # 左脚支撑
                msg.swingHeightTrajectory.append(0.10)  # 摆动高度
                
                # 创建脚部位姿消息
                foot_pose = footPose()
                left_foot_pose = [left_foot_pose[0], left_foot_pose[1], 0.0, left_foot_pose[2]]  # 添加 z 值
                foot_pose.footPose = left_foot_pose  # 左脚位姿
                foot_pose.torsoPose = [
                    torso_pose[0],  # x
                    torso_pose[1],  # y
                    height_offset,  # z (使用第一个高度作为参考)
                    torso_pose[3]  # yaw
                ]
                msg.footPoseTrajectory.append(foot_pose)

                # 更新上一帧的脚部位置
                prev_left_foot_pos = left_foot_pose
            
            elif mode == 1:  # 右脚摆动 (SF)
                if prev_left_foot_pos is not None:
                    # 将躯干的 x-y 坐标映射到当前帧右脚位置与上一帧左脚位置的连线上
                    projected_torso_pos = self.project_torso_to_line(
                        torso_pose[:2],  # 躯干 x-y
                        torso_pose[3],  # 躯干 yaw
                        right_foot_pose[:2],  # 当前帧右脚 x-y
                        prev_left_foot_pos[:2],  # 上一帧左脚 x-y
                        offset=0.03
                    )
                    torso_pose[0] = projected_torso_pos[0]
                    torso_pose[1] = projected_torso_pos[1]
                
                msg.timeTrajectory.append(time)
                msg.footIndexTrajectory.append(1)  # 右脚支撑
                msg.swingHeightTrajectory.append(0.10)  # 摆动高度
                
                # 创建脚部位姿消息
                foot_pose = footPose()
                right_foot_pose = [right_foot_pose[0], right_foot_pose[1], 0.0, right_foot_pose[2]]  # 添加 z 值
                foot_pose.footPose = right_foot_pose  # 右脚位姿
                foot_pose.torsoPose = [
                    torso_pose[0],  # x
                    torso_pose[1],  # y
                    height_offset,  # z (使用第一个高度作为参考)
                    torso_pose[3]  # yaw
                ]
                msg.footPoseTrajectory.append(foot_pose)

                # 更新上一帧的脚部位置
                prev_right_foot_pos = right_foot_pose
            
            elif mode == 3:  # 三步调整
                swing_time = 0.6
                swing_height = 0.1
                # 第一步：双脚支撑 (SS)
                if i != len(self.step_control) - 1:
                    self.mode_3_t += 2.0
                else:
                    swing_time = 0.4
                    swing_height = 0.05
                msg.timeTrajectory.append(time)
                msg.footIndexTrajectory.append(2)
                msg.swingHeightTrajectory.append(0.0)
                
                # 将躯干的 x-y 坐标映射到当前帧右脚位置与上一帧左脚位置的连线上
                projected_torso_pos = self.project_torso_to_line(
                    torso_pose[:2],  # 躯干 x-y
                    torso_pose[3],  # 躯干 yaw
                    right_foot_pose[:2],  # 当前帧右脚 x-y
                    left_foot_pose[:2],  # 上一帧左脚 x-y
                    offset=0.03
                )
                torso_pose[0] = projected_torso_pos[0]
                torso_pose[1] = projected_torso_pos[1]

                foot_pose = footPose()
                foot_pose.footPose = [0, 0, 0, 0]
                foot_pose.torsoPose = torso_pose
                foot_pose.torsoPose = [
                    torso_pose[0],  # x
                    torso_pose[1],  # y
                    height_offset,  # z (使用第一个高度作为参考)
                    torso_pose[3]  # yaw
                ]
                msg.footPoseTrajectory.append(foot_pose)

                # 第二步：左脚摆动 (FS)
                msg.timeTrajectory.append(time + swing_time)
                msg.footIndexTrajectory.append(0)
                msg.swingHeightTrajectory.append(swing_height)
                
                foot_pose = footPose()
                left_foot_pose = [left_foot_pose[0], left_foot_pose[1], 0.0, left_foot_pose[2]]  # 添加 z 值
                foot_pose.footPose = left_foot_pose
                foot_pose.torsoPose = [
                    torso_pose[0],  # x
                    torso_pose[1],  # y
                    height_offset,  # z (使用第一个高度作为参考)
                    torso_pose[3]  # yaw
                ]
                msg.footPoseTrajectory.append(foot_pose)
                
                # 第三步：右脚摆动 (SF)
                msg.timeTrajectory.append(time + swing_time*2)
                msg.footIndexTrajectory.append(1)
                msg.swingHeightTrajectory.append(swing_height)
                
                foot_pose = footPose()
                right_foot_pose = [right_foot_pose[0], right_foot_pose[1], 0.0, right_foot_pose[2]]  # 添加 z 值
                foot_pose.footPose = right_foot_pose
                foot_pose.torsoPose = [
                    torso_pose[0],  # x
                    torso_pose[1],  # y
                    height_offset,  # z (使用第一个高度作为参考)
                    torso_pose[3]  # yaw
                ]
                msg.footPoseTrajectory.append(foot_pose)
                # 更新上一帧的脚部位置
                prev_left_foot_pos = left_foot_pose
                prev_right_foot_pos = right_foot_pose
        
        if msg:
            # 确保时间轨迹是严格递增的
            if len(msg.timeTrajectory) > 1:
                # 检查并修复时间轨迹顺序
                for i in range(1, len(msg.timeTrajectory)):
                    if msg.timeTrajectory[i] <= msg.timeTrajectory[i-1]:
                        msg.timeTrajectory[i] = msg.timeTrajectory[i-1] + 0.01  # 添加最小时间间隔
                        rospy.logwarn(f"修复时间轨迹顺序，索引 {i}: {msg.timeTrajectory[i-1]} -> {msg.timeTrajectory[i]}")
            
            self.foot_pose_pub.publish(msg)
            rospy.loginfo("已发送足部轨迹")
    
    def project_torso_to_line(self, torso_pos, torso_yaw, foot_pos1, foot_pos2, offset=0.01):
        """将躯干位置映射到脚部位置连线的中垂线上的偏移位置
        
        Args:
            torso_pos: 躯干位置 [x, y]
            torso_yaw: 躯干朝向（弧度）
            foot_pos1: 第一个脚的位置 [x, y]
            foot_pos2: 第二个脚的位置 [x, y]
            offset: 向后偏移距离（米）
            
        Returns:
            projected_pos: 投影后的位置 [x, y]
        """
        # 计算连线向量
        line_vec = np.array(foot_pos2) - np.array(foot_pos1)
        torso_vec = np.array(torso_pos) - np.array(foot_pos1)
        
        # 计算投影
        t = np.dot(torso_vec, line_vec) / np.dot(line_vec, line_vec)
        t = np.clip(t, 0.3, 0.7)  # 保持在两脚之间的合理范围内
        
        # 计算投影点
        projected_point = np.array(foot_pos1) + t * line_vec
        
        # 计算中垂线方向（将line_vec旋转90度）
        perpendicular_vec = np.array([-line_vec[1], line_vec[0]])
        perpendicular_vec = perpendicular_vec / np.linalg.norm(perpendicular_vec)  # 单位化
        
        # 计算躯干朝向的单位向量
        torso_direction = np.array([np.cos(torso_yaw), np.sin(torso_yaw)])
        
        # 确定偏移方向（如果与躯干朝向相反，则翻转）
        if np.dot(perpendicular_vec, torso_direction) > 0:
            perpendicular_vec = -perpendicular_vec
            
        # 应用偏移
        final_pos = projected_point + offset * perpendicular_vec
        
        return final_pos.tolist()

    def generate_arm_trajectory_with_csv(self, start_time):
        """生成手臂轨迹
        
        参数:
            start_time: float, 步态开始执行的时间
        """
        if not self.step_control:
            rospy.logerr("未加载手臂数据")
            return None
            
        msg = armTargetPoses()
        msg.times = []
        msg.values = []
        
        current_time = start_time
        for step in self.step_control:
            # 累加时间间隔
            current_time = step['time'] + self.mode_3_t  # 每行数据的时间间隔为0.01秒
            msg.times.append(current_time)
            # 添加手臂数据
            # 将角度从度转换为弧度
            arm_row_rad = [math.degrees(angle) for angle in step['arm_actions']]
            
            # 添加手臂数据（弧度单位）
            msg.values.extend(arm_row_rad)
        
        return msg
    
    def set_arm_external_control(self):
        """设置手臂为外部控制模式"""
        try:
            # 2 表示外部控制模式
            response = self.change_arm_mode(2)
            if hasattr(response, 'result'):
                if response.result:
                    rospy.loginfo("成功切换到手臂外部控制模式")
                    return True
            elif hasattr(response, 'success'):
                if response.success:
                    rospy.loginfo("成功切换到手臂外部控制模式")
                    return True
            rospy.logerr("切换手臂控制模式失败")
            return False
        except rospy.ServiceException as e:
            rospy.logerr(f"调用手臂控制模式服务失败: {str(e)}")
            return False
    
    def execute_action_with_csv(self, time_offset=None, target_time=None):
        """执行动作序列
        
        Args:
            time_offset: 时间偏移量，如果为None则不设置偏移
            target_time: 目标执行时间（纳秒时间戳）
        """
        if not self.step_control:
            rospy.logerr("未加载手臂数据")
            return
            
        rospy.loginfo("开始执行动作序列...")
        
        # 设置手臂为外部控制模式
        mode_start_ns = time.time_ns()
        rospy.loginfo("🔧 更改手臂控制模式为2...")
        if not self.set_arm_external_control():
            rospy.logerr("无法切换到手臂外部控制模式，终止执行")
            return
        mode_end_ns = time.time_ns()
        mode_duration_ns = mode_end_ns - mode_start_ns
        rospy.loginfo(f"   模式更改耗时: {mode_duration_ns} ns ({mode_duration_ns/1e6:.3f} ms)")
            
        # 等待接收到MPC时间
        timeout = rospy.Duration(5.0)  # 5秒超时
        start_wait = rospy.Time.now()
        while not self.mpc_time_received:
            if (rospy.Time.now() - start_wait) > timeout:
                rospy.logerr("等待MPC时间超时")
                return
            rospy.sleep(0.1)
        
        # 如果指定了目标时间，等待到该时间点
        if target_time is not None:
            rospy.loginfo("🎯 检测到目标时间参数，等待到指定时间点...")
            wait_for_target_time(target_time)
        
        # 如果指定了时间偏移
        if time_offset is not None and time_offset > 0:
            # 等待接收到MPC时间
            timeout = rospy.Duration(5.0)  # 5秒超时
            start_wait = rospy.Time.now()
            while not self.mpc_time_received:
                if (rospy.Time.now() - start_wait) > timeout:
                    rospy.logerr("等待MPC时间超时")
                    return False
                rospy.sleep(0.1)
            
            # 设置调度参数
            target_time = self.mpc_time + time_offset
            rospy.set_param('/mpc/schedule/enable', True)
            rospy.set_param('/mpc/schedule/start_time', target_time)
            rospy.loginfo(f"设置调度参数: 启用=True, 起始时间={target_time:.2f}")
        else:
            rospy.set_param('/mpc/schedule/enable', False)
            
        # 重置步态时间标志
        self.gait_start_time_received = False
        
        # 生成并发布足部轨迹
        self.generate_foot_trajectory_with_csv()
            
        # 等待接收到步态开始时间
        timeout = rospy.Duration(5.0)  # 5秒超时
        start_wait = rospy.Time.now()
        while not self.gait_start_time_received:
            if (rospy.Time.now() - start_wait) > timeout:
                rospy.logerr("等待步态开始时间超时")
                return
            rospy.sleep(0.1)
        rospy.loginfo(f"步态开始时间: {self.gait_start_time}, 等待手臂轨迹发布")
        
        # 等待到达步态开始时间
        while self.mpc_time < self.gait_start_time:
            rospy.sleep(0.1)
        rospy.loginfo("开始发布手臂轨迹")
            
        # 生成并发布手臂轨迹（基于步态开始时间）
        arm_traj = self.generate_arm_trajectory_with_csv(0)
        if arm_traj:
            self.arm_target_pub.publish(arm_traj)
            rospy.loginfo(f"已发送手臂轨迹，起始时间: {self.gait_start_time}")
            
        # 计算总持续时间
        total_arm_time = len(self.step_control) * 0.1  # 每行数据0.01秒
        
        rospy.loginfo(f"等待动作完成，预计耗时: {total_arm_time}秒")
        start_time = rospy.get_time()
        while (rospy.get_time() - start_time) < total_arm_time and not rospy.is_shutdown():
            rospy.sleep(0.1)  # 短间隔睡眠，保证响应中断
        rospy.loginfo("动作序列执行完成")

def wait_for_target_time(target_time_ns):
    """等待到指定的绝对时间点（纳秒）"""
    
    current_time_ns = time.time_ns()
    wait_time_ns = target_time_ns - current_time_ns
    
    # 打印高精度时间信息
    current_time_s = current_time_ns / 1e9
    target_time_s = target_time_ns / 1e9
    
    rospy.loginfo("="*60)
    rospy.loginfo("⏰ 高精度时间同步信息:")
    rospy.loginfo(f"   当前时间: {current_time_ns} ns ({current_time_s:.9f} s)")
    rospy.loginfo(f"   目标时间: {target_time_ns} ns ({target_time_s:.9f} s)")
    rospy.loginfo(f"   时间差: {wait_time_ns} ns")
    
    if wait_time_ns > 0:
        wait_time_s = wait_time_ns / 1e9
        wait_time_ms = wait_time_ns / 1e6
        wait_time_us = wait_time_ns / 1e3
        
        rospy.loginfo(f"   等待时间: {wait_time_s:.3f} s = {wait_time_ms:.3f} ms = {wait_time_us:.3f} μs")
        
        # 精确等待到目标时间
        start_wait_ns = time.time_ns()
        while time.time_ns() < target_time_ns:
            remaining_ns = target_time_ns - time.time_ns()
            if remaining_ns > 1000000:  # 如果剩余时间大于1ms
                time.sleep(0.001)  # 休眠1ms
            else:
                # 小于1ms时，忙等待以获得更高精度
                pass
        
        # 计算实际等待精度
        end_wait_ns = time.time_ns()
        actual_wait_ns = end_wait_ns - start_wait_ns
        timing_error_ns = actual_wait_ns - wait_time_ns
        
        actual_exec_ns = time.time_ns()
        timing_error_exec_ns = actual_exec_ns - target_time_ns
        
        rospy.loginfo("⏱️  等待完成统计:")
        rospy.loginfo(f"   计划等待: {wait_time_ns} ns")
        rospy.loginfo(f"   实际等待: {actual_wait_ns} ns")
        rospy.loginfo(f"   等待误差: {timing_error_ns:+d} ns ({timing_error_ns/1e3:+.3f} μs)")
        rospy.loginfo(f"   执行时间: {actual_exec_ns} ns")
        rospy.loginfo(f"   执行误差: {timing_error_exec_ns:+d} ns ({timing_error_exec_ns/1e3:+.3f} μs)")
        rospy.loginfo("✅ 到达目标时间点，开始执行动作")
        rospy.loginfo("="*60)
    else:
        # 目标时间已过的情况
        overdue_ns = -wait_time_ns
        overdue_ms = overdue_ns / 1e6
        rospy.logwarn(f"⚠️  目标时间已过: {overdue_ns} ns ({overdue_ms:.3f} ms)")
        rospy.logwarn("⚠️  立即执行动作")
        rospy.loginfo("="*60)

def main():
    """主函数"""
    # 创建命令行参数解析器
    parser = argparse.ArgumentParser(description='CSV轨迹播放器')
    parser.add_argument('csv_file', type=str, help='CSV文件路径', nargs='?', 
                       default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "actions", "taiji_step_roban_stable.csv"))
    parser.add_argument('--time-offset', type=float, help='轨迹开始时间的偏移量(秒)。正值表示在当前MPC时间基础上延迟执行，负值或当前时间之前的值将被忽略（使用默认值-1）')
    parser.add_argument('--target-time', type=int, help='目标执行时间（纳秒时间戳）')
    
    # 解析命令行参数
    args = parser.parse_args()
    
    # 处理目标时间
    target_time_ns = args.target_time
    if target_time_ns:
        import time
        start_time_ns = time.time_ns()
        
        # 打印从参数接收的原始目标时间
        original_target_s = target_time_ns / 1e9
        rospy.loginfo("🎯 接收到目标时间参数:")
        rospy.loginfo(f"   原始目标时间: {target_time_ns} ns ({original_target_s:.9f} s)")
        rospy.loginfo("   目标时间已记录，将用于时间同步")
        
        # 检查是否已经超过目标时间
        if start_time_ns > target_time_ns:
            overdue_ns = start_time_ns - target_time_ns
            overdue_ms = overdue_ns / 1e6
            rospy.logwarn(f"⚠️  当前时间已超过目标时间: {overdue_ns} ns ({overdue_ms:.3f} ms)")
            
            # 自动延后5秒
            target_time_ns = start_time_ns + int(5e9)  # 5秒 = 5,000,000,000 ns
            new_target_s = target_time_ns / 1e9
            rospy.logwarn(f"🕐 自动调整目标时间延后5秒: {target_time_ns} ns ({new_target_s:.9f} s)")
        else:
            rospy.loginfo(f"   目标时间: {target_time_ns} ns")
            rospy.loginfo("   将在指定时间执行手臂动作")
    else:
        rospy.loginfo("   未指定目标时间，立即执行")
    
    player = ActionPlayer()
    
    # 加载动作数据
    if not player.load_action_with_csv(args.csv_file):
        return
    
    # 等待ROS系统就绪
    rospy.sleep(1)
    
    try:
        # 执行动作序列
        player.execute_action_with_csv(args.time_offset, args.target_time)
    except rospy.ROSInterruptException:
        rospy.loginfo("动作执行被中断")
    except Exception as e:
        rospy.logerr(f"执行动作时出错: {str(e)}")

if __name__ == '__main__':
    main()
