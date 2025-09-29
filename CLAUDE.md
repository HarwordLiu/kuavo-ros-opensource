# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is the Kuavo humanoid robot ROS opensource repository, containing ROS packages for controlling Kuavo humanoid robots with MPC (Model Predictive Control) and WBC (Whole Body Control) architecture. The system supports both simulation (MuJoCo, Gazebo, Isaac Sim) and real robot control.

## Environment Setup

### Robot Version Configuration
The robot version must be set via environment variable:
```bash
export ROBOT_VERSION=45  # Replace with actual robot version (30-49)
echo $ROBOT_VERSION  # Verify setting
```

### Docker Environment (Recommended)
```bash
# Download pre-built image
wget https://kuavo.lejurobot.com/docker_images/kuavo_opensource_mpc_wbc_img_v0.6.1.tar.gz
docker load -i kuavo_opensource_mpc_wbc_img_v0.6.1.tar.gz

# Run container
./docker/run.sh  # or ./docker/run_with_gpu.sh for GPU support
```

## Build Commands

### Initial Setup
```bash
# Configure catkin with required settings
catkin config -DCMAKE_ASM_COMPILER=/usr/bin/as -DCMAKE_BUILD_TYPE=Release

# Source pre-installed dependencies
source installed/setup.bash  # or setup.zsh for zsh

# Build main controllers (builds all dependencies)
catkin build humanoid_controllers

# Source workspace
source devel/setup.bash  # or setup.zsh for zsh
```

### Additional Packages
```bash
# For Gazebo simulation
catkin build humanoid_controllers gazebo_sim

# For Isaac Sim
catkin build humanoid_controllers isaac_sim

# For data challenge simulator
catkin build humanoid_controllers data_challenge_simulator
```

## Run Commands

### Simulation
```bash
# MuJoCo simulation (default)
roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch

# Gazebo simulation
roslaunch humanoid_controllers load_kuavo_gazebo_sim.launch

# Isaac Sim
roslaunch humanoid_controllers load_kuavo_isaac_sim.launch

# With VR control
roslaunch humanoid_controllers load_kuavo_mujoco_sim_with_vr.launch

# With joystick control
roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch use_joystick:=true

# Half upper body only
roslaunch humanoid_controllers load_kuavo_mujoco_sim_half_up_body.launch

# Keyboard control simulation
roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch joystick_type:=sim
```

### Real Robot
```bash
# First time calibration mode
roslaunch humanoid_controllers load_kuavo_real.launch cali:=true

# Normal operation
roslaunch humanoid_controllers load_kuavo_real.launch

# Half upper body only (for i9 CPU)
roslaunch humanoid_controllers load_kuavo_real.launch

# Half upper body only (for i7 CPU)
roslaunch humanoid_controllers load_kuavo_real_half_up_body.launch

# With C++ SDK (default: true)
roslaunch humanoid_controllers load_kuavo_real.launch ruiwo_cxx_sdk:=true

# With VR control
roslaunch humanoid_controllers load_kuavo_real_with_vr.launch
```

### Control Interfaces
```bash
# VR control (Quest3)
roslaunch noitom_hi5_hand_udp_python launch_quest3_ik.launch ip_address:=<QUEST3_IP>

# Keyboard control node
rosrun humanoid_interface_ros joystickSimulator.py

# Joystick types: bt2, bt2pro, h12, sim
roslaunch humanoid_controllers <launch_file> joystick_type:=bt2
```

## Architecture Overview

### Core Components
- **MPC (Model Predictive Control)**: OCS2-based trajectory optimization for whole-body motion planning
- **WBC (Whole Body Control)**: Low-level joint control using constrained optimization
- **State Estimation**: Sensor fusion for robot state estimation
- **Humanoid Controllers**: Main control interface coordinating MPC/WBC

### Key Directories
- `src/humanoid-control/`: Main control system packages
  - `humanoid_controllers/`: Primary controller launch files and logic
  - `humanoid_common/`: Shared utilities and common functionality
  - `humanoid_arm_control/`: Arm-specific control modules
  - `mobile_manipulator_controllers/`: Combined locomotion and manipulation
- `src/ocs2/`: OCS2 optimal control library (MPC implementation)
- `src/kuavo_assets/`: Robot models, configurations, and URDF files
  - `config/kuavo_v*/`: Version-specific robot configurations
  - `models/`: URDF models for different robot versions
- `src/demo/`: Example applications and demonstrations
- `src/gazebo/`, `src/kuavo-isaac-sim/`: Simulation environments
- `installed/`: Pre-built ROS packages and hardware dependencies

### Configuration System
Robot configurations are version-dependent and stored in:
- `src/kuavo_assets/config/kuavo_v${ROBOT_VERSION}/kuavo.json`: Main robot config
- `~/.config/lejuconfig/TotalMassV${ROBOT_VERSION}`: Robot mass calibration
- `~/.config/lejuconfig/offset.csv`: Joint zero positions (legs/shoulders)
- `~/.config/lejuconfig/arms_zero.yaml`: Arm joint zero positions

### Control Flow
1. **Target Generation**: Goals sent via `/humanoid_mpc_target` topic
2. **MPC Planning**: OCS2 generates optimal trajectories
3. **WBC Execution**: Converts MPC plans to joint commands via `/joint_cmd`
4. **State Feedback**: Sensor data flows back through `/sensor_data_raw` and `/state_estimate/*`

## Important Development Notes

### Robot Version Dependencies
- Always verify `$ROBOT_VERSION` matches your robot before building/running
- Mass calibration in `TotalMassV${ROBOT_VERSION}` affects model dynamics
- Changing mass deletes OCS2 caches in `/var/ocs2/biped_v${ROBOT_VERSION}` (4min rebuild)

### Control Modes
End effector types in `kuavo.json`:
- `none`: No end effector
- `qiangnao`: Dexterous hand (default)
- `lejuclaw`: Two-finger gripper
- `qiangnao_touch`: Tactile dexterous hand

Upper body only mode: Set `"only_half_up_body": true` in `kuavo.json`

### Real Robot Calibration
- Zero point calibration required after motor/encoder replacement
- Reboot calibration needed for robots without multi-turn encoder memory
- Use `cali:=true` mode for guided calibration process

### Common Topics
- `/joint_cmd`: Joint position/velocity/torque commands
- `/sensor_data_raw`: Raw sensor data from robot/simulation
- `/humanoid_mpc_target`: MPC target trajectory
- `/humanoid_mpc_observation`: Current state for MPC
- `/state_estimate/*`: Estimated robot state (position, velocity, contacts)
- `/cmd_vel`: High-level velocity commands (auto-gait mode)

### Testing
- No standardized test framework
- Check demo applications in `src/demo/` for usage examples
- Verify with simulation before real robot deployment
- Use monitoring topics `/monitor` for performance analysis

### Compilation Cache
- Uses ccache for faster builds (configured in docker setup)
- Use `./generate_compile_commands.sh` to generate compile_commands.json for IDE support