# Kuavo Data Challenge Simulator - 简易安装教程

> ⚠️ **通用安装教程请参考 main branch**

## 1. 克隆代码仓库
```bash
git clone https://github.com/LejuRobotics/kuavo-ros-opensource.git
cd kuavo-ros-opensource
git checkout opensource/kuavo-data-challenge
````

## 2. 启动 Docker 环境
- docker镜像可以使用`./docker/Dockerfile`构建，或者下载已经编译好的镜像：

```bash  
wget https://kuavo.lejurobot.com/docker_images/kuavo_opensource_mpc_wbc_img_v0.6.1.tar.gz
```

- 执行以下命令导入容器镜像：
```bash
docker load -i kuavo_opensource_mpc_wbc_img_v0.6.1.tar.gz
```
- 执行`./docker/run.sh`进入容器后（此处推荐使用gpu版本的run_with_gpu.sh,需要相应的nvidia container toolkit），默认在仓库的映射目录`/root/kuavo_ws`

## 3. 设置机器人版本环境变量

```bash
export ROBOT_VERSION=45
```

## 4. 添加密钥

```bash
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys AD19BAB3CBF125EA
```

## 5. 安装缺失依赖

```bash
sudo apt-get update
sudo apt-get install ros-noetic-geographic-msgs
```

## 6. 编译工程

```bash
catkin config -DCMAKE_ASM_COMPILER=/usr/bin/as -DCMAKE_BUILD_TYPE=Release
source installed/setup.zsh
catkin build humanoid_controllers data_challenge_simulator
source devel/setup.zsh
```

## 7. 安装 SDK

```bash
cd src/kuavo_humanoid_sdk
./install.sh
```

## 8. 运行 MuJoCo 仿真

```bash
roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch
```

等待机器人加载完成后退出。若能正常运行则说明环境配置成功 

后续可在kuavo_ws目录下进入./zshrc，添加
```bash
export ROBOT_VERSION=45
source devel/setup.zsh
```

# 内容说明
## 如何使用
模型推理测试

⚠️ 此readme只包含推理时仿真侧的操作，推理侧具体请查看kuavo-data-challenge仓库的readme

进入data_challenge_simulator/examples/deploy文件夹，运行
```bash
python3 deploy.py
```
选择此次推理的任务

仿真侧会自动打开对应场景文件，并将机器人移动到初始预抓位，此时等待推理侧响应，开始推理（注意，第一次会自动被reset并结束所有进程）

推理过程中，若完成任务或者推理结束，都会自动结束此次推理并结束所有进程，开始下一次推理过程

每一轮的得分和目前的平均分会在terminal中显示，只取有效的数据计入总成绩


## 数据与任务说明
### 数据说明
采集的rosbag包含如下topic：
```text
'/gripper/command',                                # 夹爪action      (type: JointState, 仅使用position:0(打开) - 255(闭合)) [left_gripper, right_gripper]
'/gripper/state',                                  # 夹爪state       (type: JointState, 仅使用position:0(打开) - 0.8(闭合)) [left_gripper, right_gripper]
'/sensors_data_raw',                               # 手臂state       (包含joint_data,imu_data,end_effort_data，其中手臂state为joint_data.joint_q,单位为弧度)
'/kuavo_arm_traj',                                 # 手臂action (type: JointState, 仅使用position:单位:角度, [0:7]:左臂七个关节角度指令，[7:14]:右臂七个关节角度)

'/joint_cmd',                                      # 一般不使用

'/cam_h/color/image_raw/compressed',               # 头部彩色相机     （格式：jpg， 640x480）
'/cam_l/color/image_raw/compressed',               # 左腕彩色相机     （格式：jpg， 640x480）
'/cam_r/color/image_raw/compressed',               # 右腕彩色相机     （格式：jpg， 640x480）

'/cam_h/depth/image_raw/compressedDepth',          # 头部深度相机     （格式：png， 640x480）
'/cam_r/depth/image_rect_raw/compressedDepth',     # 左腕深度相机     （格式：png， 640x480）
'/cam_l/depth/image_rect_raw/compressedDepth',     # 右腕深度相机     （格式：png， 640x480）
```

### 任务说明
1. 任务一
机器人左臂将传送带上的物体抓取并放置到桌面指定区域1，右臂再抓取并放置到区域2

其中，环境光照强度，颜色，物体的颜色，在传送带上的位置，区域1与区域2的位置，均会在一定范围内随机

评分标准：
a. 成功抓住传送带上物体并放到桌面上指定区域，加30分，保持方向正确，加10分

b. 成功抓住放置到桌面物体并放到最终目标区域，加30分，保持方向正确，加10分

c. 在指定时间N秒内完成，加20分，超出N秒，每一秒扣2分

关于位置和方向的判定：

物体必须在桌面上，且中心位置必须位于区域色块内才视为位置正确，仅位置成功时才会触发方向正确加分，物体z轴与世界z轴偏差不超过10度视为方向正确