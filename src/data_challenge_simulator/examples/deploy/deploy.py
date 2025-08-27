import os
import sys
import time
import signal
import subprocess
from datetime import datetime
from pathlib import Path

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


GREEN = "\033[92m"
CYAN  = "\033[96m"
YELLOW= "\033[93m"
RESET = "\033[0m"

def read_score_from_file(score_file: str) -> int:
    try:
        with open(score_file, "r") as f:
            line = f.readline().strip()
            return int(line)
    except Exception:
        return None

def append_history(history_file: str, cycle_idx: int, score: int):
    try:
        os.makedirs(os.path.dirname(history_file), exist_ok=True)
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with open(history_file, "a") as f:
            f.write(f"{ts}\tcycle={cycle_idx}\tscore={score}\n")
    except Exception:
        pass

    # === 无限循环，直到 Ctrl+C 结束 ===
def run_task(task_id: int):

    print("[INFO] 启动 roscore...")
    roscore_process = subprocess.Popen(
        ["roscore"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid
    )
    time.sleep(1)
    task_script = os.path.join(SCRIPT_DIR, f"eval{task_id}.py")
    launch_file = f"load_kuavo_mujoco_sim{task_id}.launch"

    if not os.path.exists(task_script):
        print(f"[ERROR] 任务脚本不存在：{task_script}")
        sys.exit(1)

    # 分数文件与历史文件
    scores_dir = os.path.join(SCRIPT_DIR, "scores")
    score_file = os.path.join(scores_dir, f"score_task{task_id}_last.txt")
    history_file = os.path.join(scores_dir, f"run_scores_task{task_id}.txt")

    # 统计所有轮次的分数
    scores = []

    # === 无限循环，直到 Ctrl+C 结束 ===
    cycle_idx = 1
    while True:
        print(f"\n[INFO] === 第 {cycle_idx} 轮：启动仿真环境：{launch_file} ===")
        launch_process = subprocess.Popen(
            ['roslaunch', 'data_challenge_simulator', launch_file],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid  # 独立进程组，便于整体 SIGTERM
        )

        try:
            # 等仿真起来
            time.sleep(2)

            print(f"[INFO] 运行任务脚本：{task_script}")
            env = os.environ.copy()
            # 传递分数文件路径给任务脚本
            env['SCORE_FILE'] = score_file

            EXAMPLES_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '..'))
            PKG_ROOT = os.path.dirname(EXAMPLES_DIR)
            env['PYTHONPATH'] = PKG_ROOT + os.pathsep + env.get('PYTHONPATH', '')

            # 先清空旧分数文件，避免误读上轮
            try:
                if os.path.exists(score_file):
                    os.remove(score_file)
            except Exception:
                pass

            task_process = subprocess.Popen(['python3', task_script], env=env)

            # 等到任务脚本退出（收到 /simulator/reset 后会自行退出）
            task_process.wait()
            print("[INFO] 任务脚本退出，读取本轮最终得分...")

            # ✅ 从 txt 读取分数
            score = read_score_from_file(score_file)
            if score is None:
                # 没有成绩（可能是 reset 轮），不计入平均
                valid_cnt = len(scores)
                avg = (sum(scores) / valid_cnt) if valid_cnt > 0 else 0.0
                print(
                    f"{CYAN}[RESULT]{RESET} "
                    f"第 {YELLOW}{cycle_idx}{RESET} 轮：{YELLOW}无成绩（reset 轮）{RESET} | "
                    f"当前平均分: {GREEN}{avg:.2f}{RESET} （有效 {valid_cnt} 轮）"
                )
                append_history(history_file, cycle_idx, -1)  # -1 表示无效轮
            else:
                scores.append(score)
                valid_cnt = len(scores)
                avg = sum(scores) / valid_cnt if valid_cnt > 0 else 0.0
                append_history(history_file, cycle_idx, score)
                print(
                    f"{CYAN}[RESULT]{RESET} "
                    f"第 {YELLOW}{cycle_idx}{RESET} 轮得分: {GREEN}{score}{RESET} | "
                    f"当前平均分: {GREEN}{avg:.2f}{RESET} （有效 {valid_cnt} 轮）"
                )

        except KeyboardInterrupt:
            print("\n[INFO] 收到中断信号，正在退出循环...")
            break

        finally:
            # 关闭仿真环境（roslaunch 进程组）
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
