# Kuavo Data Challenge Simulator - 简易安装教程

> ⚠️ **通用安装教程请参考 main branch**

## 📦 1. 克隆代码仓库
```bash
git clone https://github.com/LejuRobotics/kuavo-ros-opensource.git
cd kuavo-ros-opensource
git checkout opensource/kuavo-data-challenge
````

## 🐳 2. 启动 Docker 环境

```bash
cd docker
./run_with_gpu.sh
```

## ⚙️ 3. 设置机器人版本环境变量

```bash
export ROBOT_VERSION=45
```

## 🔑 4. 添加密钥

```bash
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys AD19BAB3CBF125EA
```

## 📥 5. 安装缺失依赖

```bash
sudo apt-get update
sudo apt-get install ros-noetic-geographic-msgs
```

## 🛠 6. 编译工程

```bash
catkin config -DCMAKE_ASM_COMPILER=/usr/bin/as -DCMAKE_BUILD_TYPE=Release
source installed/setup.zsh
catkin build humanoid_controllers data_challenge_simulator
source devel/setup.zsh
```

## 🔧 7. 安装 SDK

```bash
cd src/kuavo_humanoid_sdk
./install.sh
```

## 🚀 8. 运行 MuJoCo 仿真

```bash
roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch
```

等待机器人加载完成后退出。

## ▶️ 9. 运行示例

```bash
cd src/data_challenge_simulator/example
python3 automation.py
```

根据提示选择任务和执行方式，若能正常运行则说明环境配置成功 ✅
后续可在kuavo_ws目录下进入./zshrc，添加
```bash
export ROBOT_VERSION=45
source devel/setup.zsh
```

# 内容说明

## 文件结构

data_challenge_simulator
├── examples                        #脚本运行文件
│   ├── automation.py               #自动采集数据文件
│   ├── deploy
│   │   ├── deploy.py               #推理过程仿真侧文件
│   │   └── eval1.py
│   ├── launch.py                   #单独运行任务文件
│   ├── task1.py                    #任务1执行文件
│   ├── task2.py                    #任务2执行文件
│   ├── task3.py                    #任务3执行文件
│   └── task4.py                    #任务4执行文件
├── launch                          #roslaunch文件
│
├── models                          #机器人模型与场景文件
│   ├── assets
│   │   └── textures
│   └── biped_s45
│       └── xml
│           ├── biped_s45.xml       #机器人模型
│           ├── scene1.xml          #场景1
│           ├── scene2.xml          #场景2
│           ├── scene3.xml          #场景3
│           └── scene4.xml          #场景4
│ 
└── utils                           #功能函数
│    ├── conveyor_controller.py     #传送带控制
│    ├── evaluator.py               #打分器
│    ├── gripper_controller.py      #夹爪控制
│    ├── object_pos.py              #获取物体位置和方向
│    ├── object_randomizer.py       #设置物体位置
│    ├── trajectory_controller.py   #机器人手臂运动
│    └── utils.py                   #helper function
├── CMakeLists.txt
├── package.xml
├── readme.md

## 如何使用
1. 数据采集
进入data_challenge_simulator/examples 文件夹
运行
```bash
python3 automation.py
```
先选择要进行数据采集的任务，再选择是否录制rosbag（若选择否，则单纯执行一次任务），选择是后，输入要采集的次数，即开始自动采集，采集过程中只会保留任务成功的rosbag，失败的任务会自动删除。保存路径为data_challenge_simulator/examples/bags.若要对bags进行修改，可能需要先对bags内文件夹加权限，
```bash
chmod -R 777 bags 
```
2. 模型推理测试
⚠️ 此readme只包含推理时仿真侧的操作，推理侧具体请查看kuavo-data-challenge仓库的readme
进入data_challenge_simulator/examples/deploy文件夹，运行
```bash
python3 deploy.py
```
选择此次推理的任务
仿真侧会自动打开对应场景文件，并将机器人移动到初始预抓位，此时等待推理侧响应，开始推理（注意，第一次会自动被reset并结束所有进程）
推理过程中，若完成任务或者推理结束，都会自动结束此次推理并结束所有进程，开始下一次推理过程
每一轮的得分和目前的平均分会在terminal中显示，只取有效的数据计入总成绩

评分标准（任务1）：
a. 成功抓住传送带上物体并放到桌面上指定区域，加30分，保持方向正确，加10分
b. 成功抓住放置到桌面物体并放到最终目标区域，，加30分，保持方向正确，加10分
c. 在规定时间内完成，加20分，超出规定时间，每1秒扣2分