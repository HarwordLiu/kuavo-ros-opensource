#!/bin/bash

# 系统健康检查脚本
# 快速诊断仿真系统状态

echo "🔍 Kuavo仿真系统健康检查"
echo "=========================="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

check_status() {
    if [ $1 -eq 0 ]; then
        echo -e "${GREEN}✓${NC} $2"
        return 0
    else
        echo -e "${RED}✗${NC} $2"
        return 1
    fi
}

warn_status() {
    echo -e "${YELLOW}⚠${NC} $1"
}

# 1. 检查环境变量
echo "1. 环境配置检查"
if [ -z "$ROBOT_VERSION" ]; then
    warn_status "ROBOT_VERSION 未设置"
    echo "   建议: export ROBOT_VERSION=45"
else
    check_status 0 "ROBOT_VERSION = $ROBOT_VERSION"
fi

# 2. 检查ROS Master
echo -e "\n2. ROS Master检查"
rosnode list > /dev/null 2>&1
check_status $? "ROS Master运行状态"

# 3. 检查关键节点
echo -e "\n3. 关键节点检查"
CRITICAL_NODES=(
    "robot_state_publisher"
    "humanoid_mpc"
    "humanoid_controller"
    "mujoco_sim"
)

for node in "${CRITICAL_NODES[@]}"; do
    rosnode list 2>/dev/null | grep -q $node
    check_status $? "$node 节点"
done

# 4. 检查关键Topic
echo -e "\n4. 关键Topic检查"
CRITICAL_TOPICS=(
    "/joint_cmd"
    "/humanoid_mpc_policy"
    "/state_estimate/joint/pos"
    "/sensor_data_raw"
)

for topic in "${CRITICAL_TOPICS[@]}"; do
    rostopic list 2>/dev/null | grep -q $topic
    if check_status $? "$topic Topic存在"; then
        # 检查数据流
        timeout 3s rostopic hz $topic > /dev/null 2>&1
        if [ $? -eq 0 ]; then
            echo -e "   ${GREEN}→${NC} 数据流正常"
        else
            echo -e "   ${RED}→${NC} 数据流异常或很慢"
        fi
    fi
done

# 5. 系统性能检查
echo -e "\n5. 系统性能检查"

# CPU使用率
cpu_usage=$(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | sed 's/%us,//')
echo -e "CPU使用率: ${cpu_usage}%"

# 内存使用率
mem_usage=$(free | grep Mem | awk '{printf "%.1f", $3/$2 * 100.0}')
echo -e "内存使用率: ${mem_usage}%"

# ROS节点数量
node_count=$(rosnode list 2>/dev/null | wc -l)
echo -e "运行的ROS节点数: $node_count"

# 6. 快速修复建议
echo -e "\n6. 问题修复建议"
echo "=========================="

# 检查是否有僵尸进程
zombie_count=$(ps aux | awk '{print $8}' | grep -c Z)
if [ $zombie_count -gt 0 ]; then
    warn_status "检测到 $zombie_count 个僵尸进程"
    echo "   修复: killall -9 roslaunch; killall -9 roscore"
fi

# 检查MuJoCo是否响应
if pgrep -f mujoco > /dev/null; then
    echo -e "${GREEN}✓${NC} MuJoCo进程运行中"
    echo "   提示: 在MuJoCo窗口按 'o' 启动机器人"
else
    warn_status "MuJoCo进程未找到"
fi

# 检查是否需要重启
rostopic hz /joint_cmd > /dev/null 2>&1
if [ $? -ne 0 ]; then
    echo -e "\n${YELLOW}🔄 建议操作:${NC}"
    echo "1. 重启系统: ./scripts/smart_launch.sh load_kuavo_mujoco_sim.launch"
    echo "2. 或手动重启:"
    echo "   rosnode kill -a"
    echo "   killall -9 roscore"
    echo "   roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch"
fi

echo -e "\n检查完成! 🏁"