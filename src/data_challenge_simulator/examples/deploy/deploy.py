import os
import sys
import time
import signal
import subprocess

from pathlib import Path

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

def run_task(task_id: int):
    task_script = os.path.join(SCRIPT_DIR, f"task{task_id}.py")
    launch_file = f"load_kuavo_mujoco_sim{task_id}.launch"

    if not os.path.exists(task_script):
        print(f"[ERROR] 任务脚本不存在：{task_script}")
        sys.exit(1)

    # === 无限循环，直到 Ctrl+C 结束 ===
    cycle_idx = 1
    while True:
        print(f"\n[INFO] === 第 {cycle_idx} 轮：启动仿真环境：{launch_file} ===")
        launch_process = subprocess.Popen(
            ['roslaunch', 'data_challenge_simulator', launch_file],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid
        )

        try:
            # 等仿真起来
            time.sleep(15)

            print(f"[INFO] 运行任务脚本：{task_script}")
            env = os.environ.copy()
            EXAMPLES_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '..'))
            PKG_ROOT = os.path.dirname(EXAMPLES_DIR)
            env['PYTHONPATH'] = PKG_ROOT + os.pathsep + env.get('PYTHONPATH', '')

            task_process = subprocess.Popen(['python3', task_script], env=env)
            # 等到任务脚本退出（收到 /simulator/reset 后会自行退出）
            task_process.wait()
            print("[INFO] 任务脚本退出，准备关闭仿真环境...")

        except KeyboardInterrupt:
            print("\n[INFO] 收到中断信号，正在退出循环...")
            # 跳出循环，统一到 finally 中收尾
            break

        finally:
            # 关闭仿真环境
            try:
                os.killpg(os.getpgid(launch_process.pid), signal.SIGTERM)
            except ProcessLookupError:
                pass
            time.sleep(1)
            print("[INFO] 仿真环境已关闭。")

        cycle_idx += 1

def main():
    task_id = None
    if len(sys.argv) >= 2:
        try:
            task_id = int(sys.argv[1])
        except ValueError:
            pass

    while task_id not in [1, 2, 3, 4]:
        print("========== 模型推理 ==========")
        print("请选择任务编号（1-4）：")
        print("1: 任务1 —— 传送带物品分拣")
        print("2: 任务2 —— 传送带物品称重")
        print("3: 任务3 —— 物品翻面")
        print("4: 任务4 —— 货架物品运送")
        try:
            task_id = int(input("请输入任务编号 (1-4): ").strip())
        except Exception:
            task_id = None

    run_task(task_id)

if __name__ == "__main__":
    main()
