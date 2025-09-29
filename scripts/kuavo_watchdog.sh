#!/bin/bash

# Kuavo仿真看门狗脚本
# 自动监控和重启异常的仿真系统

LOCK_FILE="/tmp/kuavo_sim.lock"
LOG_FILE="/tmp/kuavo_watchdog.log"

log_message() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" | tee -a $LOG_FILE
}

# 检查是否已经有实例在运行
if [ -f $LOCK_FILE ]; then
    pid=$(cat $LOCK_FILE)
    if kill -0 $pid 2>/dev/null; then
        log_message "看门狗已在运行 (PID: $pid)"
        exit 1
    else
        rm -f $LOCK_FILE
    fi
fi

# 创建锁文件
echo $$ > $LOCK_FILE

# 清理函数
cleanup() {
    log_message "看门狗退出，清理资源..."
    rm -f $LOCK_FILE
    exit 0
}

trap cleanup INT TERM

log_message "Kuavo仿真看门狗启动"

while true; do
    # 检查关键节点是否存在
    if ! rosnode list | grep -q "humanoid_mpc\|humanoid_controller"; then
        log_message "检测到关键节点缺失，重启系统..."

        # 停止现有进程
        rosnode kill -a
        killall -9 roslaunch roscore rosmaster
        sleep 5

        # 重新启动
        cd $(dirname $0)/..
        source devel/setup.bash
        export ROBOT_VERSION=${ROBOT_VERSION:-45}

        roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch &

        log_message "系统重启中，等待节点稳定..."
        sleep 30
    fi

    # 检查数据流
    if ! timeout 5s rostopic hz /joint_cmd >/dev/null 2>&1; then
        log_message "检测到数据流异常，尝试恢复..."
        rosnode kill /humanoid_controller_node
        sleep 5
    fi

    sleep 10  # 每10秒检查一次
done