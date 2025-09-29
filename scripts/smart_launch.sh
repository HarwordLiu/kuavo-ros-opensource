#!/bin/bash

# 智能启动脚本 - 自动等待节点完全加载
# 使用方法: ./scripts/smart_launch.sh load_kuavo_mujoco_sim.launch

LAUNCH_FILE=$1
if [ -z "$LAUNCH_FILE" ]; then
    echo "用法: $0 <launch_file>"
    exit 1
fi

echo "启动仿真系统..."
roslaunch humanoid_controllers $LAUNCH_FILE &
LAUNCH_PID=$!

echo "等待核心节点启动..."

# 等待关键节点启动的函数
wait_for_node() {
    local node_name=$1
    local timeout=$2
    local count=0

    echo "等待节点: $node_name"
    while ! rosnode list | grep -q $node_name; do
        sleep 1
        count=$((count + 1))
        if [ $count -gt $timeout ]; then
            echo "警告: 节点 $node_name 启动超时"
            return 1
        fi
        echo -n "."
    done
    echo " ✓ $node_name 已启动"
    return 0
}

# 等待关键topic发布的函数
wait_for_topic() {
    local topic_name=$1
    local timeout=$2
    local count=0

    echo "等待topic: $topic_name"
    while ! rostopic list | grep -q $topic_name; do
        sleep 1
        count=$((count + 1))
        if [ $count -gt $timeout ]; then
            echo "警告: topic $topic_name 发布超时"
            return 1
        fi
        echo -n "."
    done
    echo " ✓ $topic_name 已发布"
    return 0
}

# 检查topic数据流的函数
check_topic_data() {
    local topic_name=$1
    echo "检查 $topic_name 数据流..."

    timeout 5s rostopic hz $topic_name > /dev/null 2>&1
    if [ $? -eq 0 ]; then
        echo " ✓ $topic_name 数据流正常"
        return 0
    else
        echo " ✗ $topic_name 数据流异常"
        return 1
    fi
}

echo "开始检测系统启动状态..."

# 按顺序等待关键节点
wait_for_node "robot_state_publisher" 30
wait_for_node "humanoid_mpc" 45
wait_for_node "humanoid_controller" 30

# 等待关键topic
wait_for_topic "/joint_cmd" 20
wait_for_topic "/humanoid_mpc_policy" 30
wait_for_topic "/state_estimate/joint/pos" 25

echo "进行数据流检查..."
sleep 5  # 给数据流一点时间稳定

# 检查数据流
check_topic_data "/joint_cmd"
check_topic_data "/humanoid_mpc_policy"

echo ""
echo "🎉 系统启动完成！"
echo "关键检查项:"
echo "- 所有核心节点已启动"
echo "- 关键topic正在发布"
echo "- 数据流检查完成"
echo ""
echo "你现在可以:"
echo "1. 在MuJoCo窗口按 'o' 启动机器人"
echo "2. 使用手柄控制或发送 /cmd_vel 指令"
echo "3. 监控状态: rostopic echo /monitor"
echo ""

# 保持脚本运行，监控系统状态
echo "监控系统运行状态... (Ctrl+C 退出)"
while true; do
    sleep 10
    if ! kill -0 $LAUNCH_PID 2>/dev/null; then
        echo "警告: launch进程已退出"
        break
    fi

    # 简单健康检查
    if ! rostopic hz /joint_cmd > /dev/null 2>&1; then
        echo "警告: /joint_cmd 数据流中断"
    fi
done