#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Roban2机器人手臂和腿部统一磨线启动脚本
Author: AI Assistant
Date: 2025-01-27
Description: 统一启动Roban2机器人的手臂磨线（RUIWO电机）和腿部磨线（EC_Master电机）
"""

import os
import sys
import subprocess
import threading
import time
import signal
from pathlib import Path

class Colors:
    """终端颜色定义"""
    RED = '\033[0;31m'
    GREEN = '\033[0;32m'
    YELLOW = '\033[1;33m'
    BLUE = '\033[0;34m'
    CYAN = '\033[0;36m'
    NC = '\033[0m'  # No Color

class Roban2UnifiedBreakin:
    def __init__(self):
        self.current_dir = Path(__file__).parent.absolute()
        self.prefix = self.current_dir.parent.parent.parent
        
        # 定义脚本路径
        self.arm_breakin_script = self.current_dir.parent / "arm_breakin.sh"
        self.arm_breakin_py_script = self.current_dir.parent / "arm_breakin.sh"  # 改为调用arm_breakin.sh
        self.arm_setzero_script = self.current_dir.parent / "arm_setzero.sh"
        self.ruiwo_zero_script = self.current_dir.parent / "ruiwo_zero_set.sh"
        self.leg_breakin_script = self.current_dir / "roban2_leg_breakin" / "roban2_leg_breakin.py"
        
        # 进程管理
        self.arm_process = None
        self.leg_process = None
        self.log_dir = Path("/tmp/roban2_breakin_logs")
        
    def print_colored(self, message, color=Colors.NC):
        """打印带颜色的消息"""
        print(f"{color}{message}{Colors.NC}")
        
    def check_root_permission(self):
        """检查root权限"""
        if os.geteuid() != 0:
            self.print_colored("错误：请使用root权限运行此脚本", Colors.RED)
            self.print_colored("请使用: sudo python3 roban2_unified_breakin.py", Colors.YELLOW)
            sys.exit(1)
            
    def check_scripts_exist(self):
        """检查脚本文件是否存在"""
        self.print_colored("检查脚本文件...", Colors.YELLOW)
        
        if not self.arm_breakin_py_script.exists():
            self.print_colored(f"错误：手臂磨线脚本不存在: {self.arm_breakin_py_script}", Colors.RED)
            sys.exit(1)
            
        if not self.arm_setzero_script.exists():
            self.print_colored(f"错误：手臂零点设置脚本不存在: {self.arm_setzero_script}", Colors.RED)
            sys.exit(1)
            
        if not self.ruiwo_zero_script.exists():
            self.print_colored(f"错误：RUIWO零点设置脚本不存在: {self.ruiwo_zero_script}", Colors.RED)
            sys.exit(1)
            
        if not self.leg_breakin_script.exists():
            self.print_colored(f"错误：腿部磨线脚本不存在: {self.leg_breakin_script}", Colors.RED)
            sys.exit(1)
            
        self.print_colored(f"✓ 手臂磨线脚本: {self.arm_breakin_py_script}", Colors.GREEN)
        self.print_colored(f"✓ 手臂零点设置脚本: {self.arm_setzero_script}", Colors.GREEN)
        self.print_colored(f"✓ RUIWO零点设置脚本: {self.ruiwo_zero_script}", Colors.GREEN)
        self.print_colored(f"✓ 腿部磨线脚本: {self.leg_breakin_script}", Colors.GREEN)
        print()
        
    def run_arm_zero_setup(self):
        """运行手臂零点设置"""
        self.print_colored("开始执行手臂零点设置...", Colors.GREEN)
        self.print_colored("注意：请确保手臂已摆正到零点位置", Colors.YELLOW)
        
        try:
            # 执行手臂零点设置
            self.print_colored("步骤1：执行手臂零点设置...", Colors.BLUE)
            result1 = subprocess.run(
                ["bash", str(self.arm_setzero_script)],
                cwd=str(self.current_dir),
                text=True
            )
            
            if result1.returncode != 0:
                self.print_colored("手臂零点设置失败", Colors.RED)
                return False
                
            # 执行RUIWO零点设置
            self.print_colored("步骤2：执行RUIWO零点设置...", Colors.BLUE)
            result2 = subprocess.run(
                ["bash", str(self.ruiwo_zero_script)],
                cwd=str(self.current_dir),
                text=True
            )
            
            if result2.returncode != 0:
                self.print_colored("RUIWO零点设置失败", Colors.RED)
                return False
                
            self.print_colored("✓ 手臂零点设置完成", Colors.GREEN)
            return True
            
        except Exception as e:
            self.print_colored(f"手臂零点设置出错: {e}", Colors.RED)
            return False
        
    def run_arm_breakin(self):
        """运行手臂磨线"""
        self.print_colored("启动手臂磨线...", Colors.GREEN)
        self.print_colored("注意：手臂磨线使用RUIWO电机控制", Colors.YELLOW)
        
        # 先执行零点设置
        self.print_colored("在执行手臂磨线之前，需要先完成零点设置", Colors.YELLOW)
        if not self.run_arm_zero_setup():
            self.print_colored("零点设置失败，无法继续执行手臂磨线", Colors.RED)
            return 1
        
        # 获取用户输入参数
        print()
        self.print_colored("手臂磨线测试时长：", Colors.YELLOW)
        # Roban2手臂磨线最少需要14秒（7个动作 * 2秒/动作）
        arm_min_duration = 14.0
        while True:
            try:
                arm_duration_input = input(f"请输入手臂磨线测试时长（秒），最少{arm_min_duration}秒: ").strip()
                if arm_duration_input.lower() == 'q':
                    self.print_colored("已取消操作", Colors.YELLOW)
                    return 0
                arm_duration = int(arm_duration_input)
                if arm_duration <= 0:
                    self.print_colored("测试时长必须大于0，请重新输入", Colors.RED)
                    continue
                if arm_duration < arm_min_duration:
                    self.print_colored(f"错误：测试时长必须至少{arm_min_duration}秒才能完成一个完整的动作周期，请重新输入", Colors.RED)
                    continue
                
                # 计算完整轮数
                arm_rounds = arm_duration // arm_min_duration
                self.print_colored(f"手臂磨线将完整运行 {arm_rounds} 轮（每轮{arm_min_duration}秒）", Colors.CYAN)
                break
            except ValueError:
                self.print_colored("输入无效，请输入一个有效的整数", Colors.RED)
            except KeyboardInterrupt:
                self.print_colored("\n已取消操作", Colors.YELLOW)
                return 0
        
        # 添加12秒延迟
        self.print_colored("手臂磨线将在12秒后开始运行...", Colors.YELLOW)
        for i in range(12, 0, -1):
            self.print_colored(f"倒计时: {i}秒", Colors.CYAN)
            time.sleep(1)
        self.print_colored("开始执行手臂磨线！", Colors.GREEN)
        
        try:
            # 运行手臂磨线shell脚本
            result = subprocess.run(
                ["bash", str(self.arm_breakin_py_script)],
                cwd=str(self.current_dir.parent),
                input=str(arm_duration) + "\n",
                text=True
            )
            return result.returncode
        except Exception as e:
            self.print_colored(f"手臂磨线运行出错: {e}", Colors.RED)
            return 1
            
    def run_leg_breakin(self):
        """运行腿部磨线"""
        self.print_colored("启动腿部磨线...", Colors.GREEN)
        self.print_colored("注意：腿部磨线使用EC_Master电机控制", Colors.YELLOW)
        
        # 获取用户输入参数
        print()
        self.print_colored("腿部磨线参数：", Colors.YELLOW)
        # 腿部磨线最少需要时长
        min_duration = 14.0
        while True:
            try:
                duration_input = input(f"请输入腿部磨线运行时长（秒），最少{min_duration}秒: ").strip()
                if duration_input.lower() == 'q':
                    self.print_colored("已取消操作", Colors.YELLOW)
                    return 0
                duration = float(duration_input)
                if duration <= 0:
                    self.print_colored("运行时长必须大于0，请重新输入", Colors.RED)
                    continue
                if duration < min_duration:
                    self.print_colored(f"错误：运行时长必须至少{min_duration}秒才能完成一个完整的动作周期，请重新输入", Colors.RED)
                    continue
                
                # 计算完整轮数
                leg_rounds = int(duration // min_duration)
                self.print_colored(f"腿部磨线将完整运行 {leg_rounds} 轮（每轮{min_duration}秒）", Colors.CYAN)
                break
            except ValueError:
                self.print_colored("输入无效，请输入一个有效的数字", Colors.RED)
            except KeyboardInterrupt:
                self.print_colored("\n已取消操作", Colors.YELLOW)
                return 0
        
        try:
            # 腿部磨线程序需要在它自己的目录中运行
            leg_script_dir = self.leg_breakin_script.parent
            # 直接运行腿部磨线脚本
            result = subprocess.run(
                ["python3", str(self.leg_breakin_script)],
                cwd=str(leg_script_dir),
                input=str(duration) + "\n",
                text=True
            )
            return result.returncode
        except Exception as e:
            self.print_colored(f"腿部磨线运行出错: {e}", Colors.RED)
            return 1
            
    def get_user_inputs(self):
        """获取用户输入参数"""
        print()
        self.print_colored("=" * 50, Colors.CYAN)
        self.print_colored("    参数配置", Colors.CYAN)
        self.print_colored("=" * 50, Colors.CYAN)
        
        # 获取手臂磨线测试时长
        print()
        self.print_colored("手臂磨线测试时长：", Colors.YELLOW)
        # Roban2手臂磨线最少需要14秒（7个动作 * 2秒/动作）
        arm_min_duration = 14.0
        while True:
            try:
                arm_duration_input = input(f"请输入手臂磨线测试时长（秒），最少{arm_min_duration}秒: ").strip()
                if arm_duration_input.lower() == 'q':
                    return None, None
                arm_duration = int(arm_duration_input)
                if arm_duration <= 0:
                    self.print_colored("测试时长必须大于0，请重新输入", Colors.RED)
                    continue
                if arm_duration < arm_min_duration:
                    self.print_colored(f"错误：测试时长必须至少{arm_min_duration}秒才能完成一个完整的动作周期，请重新输入", Colors.RED)
                    continue
                
                # 计算完整轮数
                arm_rounds = arm_duration // arm_min_duration
                self.print_colored(f"手臂磨线将完整运行 {arm_rounds} 轮（每轮{arm_min_duration}秒）", Colors.CYAN)
                break
            except ValueError:
                self.print_colored("输入无效，请输入一个有效的整数", Colors.RED)
            except KeyboardInterrupt:
                self.print_colored("\n已取消操作", Colors.YELLOW)
                return None, None
        
        # 获取腿部磨线运行时长
        print()
        self.print_colored("腿部磨线参数：", Colors.YELLOW)
        # 腿部磨线最少需要时长
        leg_min_duration = 14.0
        while True:
            try:
                leg_duration_input = input(f"请输入腿部磨线运行时长（秒），最少{leg_min_duration}秒: ").strip()
                if leg_duration_input.lower() == 'q':
                    return None, None
                leg_duration = float(leg_duration_input)
                if leg_duration <= 0:
                    self.print_colored("运行时长必须大于0，请重新输入", Colors.RED)
                    continue
                if leg_duration < leg_min_duration:
                    self.print_colored(f"错误：运行时长必须至少{leg_min_duration}秒才能完成一个完整的动作周期，请重新输入", Colors.RED)
                    continue
                
                # 计算完整轮数
                leg_rounds = int(leg_duration // leg_min_duration)
                self.print_colored(f"腿部磨线将完整运行 {leg_rounds} 轮（每轮{leg_min_duration}秒）", Colors.CYAN)
                break
            except ValueError:
                self.print_colored("输入无效，请输入一个有效的数字", Colors.RED)
            except KeyboardInterrupt:
                self.print_colored("\n已取消操作", Colors.YELLOW)
                return None, None
        
        # 显示轮数总结
        print()
        self.print_colored("=" * 50, Colors.CYAN)
        self.print_colored("    轮数计算总结", Colors.CYAN)
        self.print_colored("=" * 50, Colors.CYAN)
        arm_rounds = arm_duration // 14.0
        leg_rounds = int(leg_duration // 14.0)
        self.print_colored(f"手臂磨线：{arm_duration}秒 ÷ 14秒/轮 = {arm_rounds} 轮", Colors.GREEN)
        self.print_colored(f"腿部磨线：{leg_duration}秒 ÷ 14秒/轮 = {leg_rounds} 轮", Colors.GREEN)
        self.print_colored("=" * 50, Colors.CYAN)
        
        return arm_duration, leg_duration
        
    def run_arm_breakin_background(self, log_file, arm_duration):
        """在后台运行手臂磨线"""
        def delayed_arm_breakin():
            """延迟执行手臂磨线的内部函数"""
            try:
                # 延迟12秒
                time.sleep(12)
                
                with open(log_file, 'a') as f:
                    f.write(f"手臂磨线延迟12秒后开始执行...\n")
                    f.flush()
                    
                    # 创建输入管道
                    input_pipe = subprocess.PIPE
                    
                    # 运行手臂磨线shell脚本
                    self.arm_process = subprocess.Popen(
                        ["bash", str(self.arm_breakin_py_script)],
                        cwd=str(self.current_dir.parent),
                        stdout=f,
                        stderr=subprocess.STDOUT,
                        stdin=input_pipe,
                        text=True
                    )
                    
                    # 发送用户输入
                    if arm_duration is not None:
                        # 发送测试时长
                        self.arm_process.stdin.write(str(arm_duration) + "\n")
                        self.arm_process.stdin.flush()
                        self.arm_process.stdin.close()
                        
            except Exception as e:
                with open(log_file, 'a') as f:
                    f.write(f"手臂磨线执行出错: {e}\n")
        
        try:
            # 启动延迟线程
            arm_thread = threading.Thread(target=delayed_arm_breakin, daemon=True)
            arm_thread.start()
            
            # 返回一个虚拟的PID（实际PID会在延迟后设置）
            return 99999  # 使用特殊值表示延迟启动
        except Exception as e:
            self.print_colored(f"启动手臂磨线失败: {e}", Colors.RED)
            return None
            
    def run_leg_breakin_background(self, log_file, duration):
        """在后台运行腿部磨线"""
        try:
            with open(log_file, 'w') as f:
                # 创建输入管道
                input_pipe = subprocess.PIPE
                # 腿部磨线程序需要在它自己的目录中运行
                leg_script_dir = self.leg_breakin_script.parent
                self.leg_process = subprocess.Popen(
                    ["python3", str(self.leg_breakin_script)],
                    cwd=str(leg_script_dir),
                    stdout=f,
                    stderr=subprocess.STDOUT,
                    stdin=input_pipe,
                    text=True
                )
                
                # 发送用户输入
                if duration is not None:
                    self.leg_process.stdin.write(str(duration) + "\n")
                    self.leg_process.stdin.flush()
                    self.leg_process.stdin.close()
                    
            return self.leg_process.pid
        except Exception as e:
            self.print_colored(f"启动腿部磨线失败: {e}", Colors.RED)
            return None
            
    def run_both_breakin(self):
        """同时运行手臂和腿部磨线"""
        self.print_colored("同时启动手臂和腿部磨线...", Colors.GREEN)
        self.print_colored("注意：", Colors.YELLOW)
        self.print_colored("  - 手臂磨线使用RUIWO电机控制", Colors.YELLOW)
        self.print_colored("  - 腿部磨线使用EC_Master电机控制", Colors.YELLOW)
        self.print_colored("  - 两个程序将并行运行", Colors.YELLOW)
        
        # 先执行手臂零点设置
        self.print_colored("在执行手臂磨线之前，需要先完成零点设置", Colors.YELLOW)
        if not self.run_arm_zero_setup():
            self.print_colored("零点设置失败，无法继续执行手臂磨线", Colors.RED)
            return 1
        
        # 获取用户输入参数
        arm_duration, leg_duration = self.get_user_inputs()
        if arm_duration is None or leg_duration is None:
            self.print_colored("用户取消操作", Colors.YELLOW)
            return 0
        
        print()
        self.print_colored("参数配置完成，开始启动程序...", Colors.GREEN)
        
        # 创建日志目录
        self.log_dir.mkdir(exist_ok=True)
        
        # 启动手臂磨线（后台运行）
        self.print_colored("启动手臂磨线进程...", Colors.BLUE)
        arm_log = self.log_dir / "arm_breakin.log"
        arm_pid = self.run_arm_breakin_background(arm_log, arm_duration)
        
        if arm_pid is None:
            self.print_colored("手臂磨线启动失败", Colors.RED)
            return 1
            
        if arm_pid == 99999:
            self.print_colored("手臂磨线进程已启动（延迟12秒执行）", Colors.GREEN)
        else:
            self.print_colored(f"手臂磨线进程ID: {arm_pid}", Colors.GREEN)
        self.print_colored("手臂磨线将在12秒后开始运行...", Colors.YELLOW)
        
        # 等待一秒确保手臂磨线启动
        time.sleep(1)
        
        # 启动腿部磨线（后台运行）
        self.print_colored("启动腿部磨线进程...", Colors.BLUE)
        leg_log = self.log_dir / "leg_breakin.log"
        leg_pid = self.run_leg_breakin_background(leg_log, leg_duration)
        
        if leg_pid is None:
            self.print_colored("腿部磨线启动失败", Colors.RED)
            # 停止已启动的手臂磨线
            if self.arm_process:
                self.arm_process.terminate()
            return 1
            
        self.print_colored(f"腿部磨线进程ID: {leg_pid}", Colors.GREEN)
        
        print()
        self.print_colored("两个磨线程序已启动！", Colors.GREEN)
        self.print_colored("进程信息：", Colors.YELLOW)
        if arm_pid == 99999:
            print(f"  手臂磨线进程: 延迟启动（12秒后开始）")
        else:
            print(f"  手臂磨线进程ID: {arm_pid}")
        print(f"  腿部磨线进程ID: {leg_pid}")
        print(f"  日志目录: {self.log_dir}")
        print()
        self.print_colored("监控命令：", Colors.YELLOW)
        print(f"  查看手臂磨线日志: tail -f {arm_log}")
        print(f"  查看腿部磨线日志: tail -f {leg_log}")
        print("  查看所有进程: ps aux | grep -E '(arm_breakin|roban2_leg_breakin)'")
        print()
        self.print_colored("停止命令：", Colors.YELLOW)
        if arm_pid == 99999:
            print(f"  停止手臂磨线: 查看日志文件获取实际PID后使用kill命令")
        else:
            print(f"  停止手臂磨线: kill {arm_pid}")
        print(f"  停止腿部磨线: kill {leg_pid}")
        if arm_pid != 99999:
            print(f"  停止所有磨线: kill {arm_pid} {leg_pid}")
        else:
            print(f"  停止所有磨线: kill {leg_pid} 和查看日志获取手臂磨线PID")
        print()
        self.print_colored("按 Ctrl+C 可以退出此脚本，但磨线程序会继续运行", Colors.BLUE)
        self.print_colored("如需停止磨线程序，请使用上述停止命令", Colors.BLUE)
        
        # 设置信号处理
        def signal_handler(signum, frame):
            self.print_colored("\n脚本退出，但磨线程序仍在运行", Colors.YELLOW)
            sys.exit(0)
            
        signal.signal(signal.SIGINT, signal_handler)
        
        # 监控进程状态
        try:
            while True:
                if self.arm_process and self.arm_process.poll() is not None:
                    self.print_colored("手臂磨线进程已结束", Colors.RED)
                    break
                if self.leg_process and self.leg_process.poll() is not None:
                    self.print_colored("腿部磨线进程已结束", Colors.RED)
                    break
                time.sleep(2)
        except KeyboardInterrupt:
            self.print_colored("\n用户中断，磨线程序继续运行", Colors.YELLOW)
            
        self.print_colored("磨线程序执行完成", Colors.GREEN)
        return 0
        
    def show_menu(self):
        """显示菜单"""
        self.print_colored("请选择启动模式：", Colors.YELLOW)
        print("1. 仅启动手臂磨线（包含零点设置）")
        print("2. 仅启动腿部磨线（零点位置开始）")
        print("3. 同时启动手臂和腿部磨线（包含零点设置）")
        print("4. 仅执行手臂零点设置")
        print("5. 退出")
        print()
        
    def run(self):
        """主运行函数"""
        # 显示标题
        self.print_colored("=" * 40, Colors.BLUE)
        self.print_colored("    Roban2机器人统一磨线启动脚本", Colors.BLUE)
        self.print_colored("=" * 40, Colors.BLUE)
        print()
        
        # 检查权限和脚本
        self.check_root_permission()
        self.check_scripts_exist()
        
        # 显示菜单并处理用户选择
        while True:
            self.show_menu()
            choice = input("请输入选项 (1-5): ").strip()
            
            if choice == "1":
                return self.run_arm_breakin()
            elif choice == "2":
                return self.run_leg_breakin()
            elif choice == "3":
                return self.run_both_breakin()
            elif choice == "4":
                if self.run_arm_zero_setup():
                    self.print_colored("手臂零点设置完成", Colors.GREEN)
                else:
                    self.print_colored("手臂零点设置失败", Colors.RED)
                return 0
            elif choice == "5":
                self.print_colored("退出程序", Colors.YELLOW)
                return 0
            else:
                self.print_colored("无效选项，请输入 1-5", Colors.RED)

def main():
    """主函数"""
    try:
        app = Roban2UnifiedBreakin()
        exit_code = app.run()
        sys.exit(exit_code)
    except KeyboardInterrupt:
        print(f"\n{Colors.YELLOW}程序被用户中断{Colors.NC}")
        sys.exit(0)
    except Exception as e:
        print(f"{Colors.RED}程序运行出错: {e}{Colors.NC}")
        sys.exit(1)

if __name__ == "__main__":
    main()
