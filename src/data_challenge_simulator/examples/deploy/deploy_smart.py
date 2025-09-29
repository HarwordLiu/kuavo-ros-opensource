import os
import sys
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

import time
import signal
import subprocess
from datetime import datetime
from pathlib import Path
import argparse
from utils.xml_random import randomize_mjcf
import json

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

GREEN = "\033[92m"
CYAN  = "\033[96m"
YELLOW= "\033[93m"
RED   = "\033[91m"
BLUE  = "\033[94m"
RESET = "\033[0m"

# 智能等待系统启动的函数
def wait_for_system_ready(timeout_seconds=60):
    """智能等待ROS系统完全启动"""
    print(f"{BLUE}[SMART LAUNCH]{RESET} 等待系统完全启动...")

    start_time = time.time()

    # 1. 等待关键节点
    critical_nodes = [
        "robot_state_publisher",
        "humanoid_mpc_node",
        "humanoid_controller_node"
    ]

    for node in critical_nodes:
        print(f"{BLUE}[WAIT]{RESET} 等待节点: {node}")
        node_timeout = 30
        elapsed = 0

        while elapsed < node_timeout:
            try:
                result = subprocess.run(
                    ["rosnode", "list"],
                    capture_output=True,
                    text=True,
                    timeout=5
                )
                if result.returncode == 0 and node in result.stdout:
                    print(f"{GREEN}✓{RESET} {node} 已启动")
                    break
            except (subprocess.TimeoutExpired, subprocess.CalledProcessError):
                pass

            time.sleep(1)
            elapsed += 1

            if elapsed >= node_timeout:
                print(f"{YELLOW}⚠{RESET} {node} 启动超时，继续等待其他节点...")
                break

    # 2. 等待关键topic
    critical_topics = [
        "/joint_cmd",
        "/humanoid_mpc_policy",
        "/state_estimate/joint/pos"
    ]

    for topic in critical_topics:
        print(f"{BLUE}[WAIT]{RESET} 等待topic: {topic}")
        topic_timeout = 20
        elapsed = 0

        while elapsed < topic_timeout:
            try:
                result = subprocess.run(
                    ["rostopic", "list"],
                    capture_output=True,
                    text=True,
                    timeout=5
                )
                if result.returncode == 0 and topic in result.stdout:
                    print(f"{GREEN}✓{RESET} {topic} 已发布")
                    break
            except (subprocess.TimeoutExpired, subprocess.CalledProcessError):
                pass

            time.sleep(1)
            elapsed += 1

            if elapsed >= topic_timeout:
                print(f"{YELLOW}⚠{RESET} {topic} 发布超时，继续检查...")
                break

    # 3. 验证数据流
    print(f"{BLUE}[VERIFY]{RESET} 验证关键数据流...")
    time.sleep(3)  # 给数据流一点稳定时间

    data_topics = ["/joint_cmd", "/humanoid_mpc_policy"]
    for topic in data_topics:
        try:
            # 检查topic是否有数据流
            result = subprocess.run(
                ["timeout", "5s", "rostopic", "hz", topic],
                capture_output=True,
                text=True
            )
            if "average rate" in result.stdout or "rate" in result.stdout:
                print(f"{GREEN}✓{RESET} {topic} 数据流正常")
            else:
                print(f"{YELLOW}⚠{RESET} {topic} 数据流检查失败，但继续执行...")
        except Exception:
            print(f"{YELLOW}⚠{RESET} {topic} 数据流检查异常，但继续执行...")

    total_wait_time = time.time() - start_time
    print(f"{GREEN}[READY]{RESET} 系统启动完成! 总等待时间: {total_wait_time:.1f}秒")

    # 额外安全等待
    print(f"{BLUE}[SAFETY]{RESET} 额外安全等待5秒确保系统稳定...")
    time.sleep(5)

    return True

def check_system_health():
    """快速健康检查"""
    print(f"{BLUE}[HEALTH]{RESET} 系统健康检查...")

    # 检查关键进程
    try:
        result = subprocess.run(["rosnode", "list"], capture_output=True, text=True, timeout=5)
        node_count = len(result.stdout.strip().split('\n')) if result.stdout.strip() else 0
        print(f"{GREEN}✓{RESET} ROS节点数: {node_count}")

        # 检查是否有关键节点
        critical_found = sum(1 for node in ["humanoid", "mpc", "controller"]
                           if node in result.stdout)
        if critical_found >= 2:
            print(f"{GREEN}✓{RESET} 关键节点检查通过")
            return True
        else:
            print(f"{RED}✗{RESET} 关键节点检查失败")
            return False

    except Exception as e:
        print(f"{RED}✗{RESET} 健康检查失败: {e}")
        return False

# 原有配置保持不变
cfg1 = {
    "rules": [
        # ---- marker1：位置 + 颜色 ----
        {
            "select": ".//body[@name='marker1']",
            "attributes": {
                "pos": {  # 基于 (0.52, -0.50, 0.8721)
                    "per_dim": [
                        {"uniform": [0.42, 0.52]},    # x
                        {"uniform": [-0.05, -0.05]},  # y
                        {"set": 0.8721}               # z 固定（也可用小范围 uniform）
                    ]
                }
            }
        },
        {
            "select": ".//body[@name='marker1']/geom",
            "attributes": {
                "rgba": {"color": {"alpha": [1.0, 1.0]}}  # 随机 RGB，alpha=1
            }
        },

        # ---- marker2：位置 + 颜色 ----
        {
            "select": ".//body[@name='marker2']",
            "attributes": {
                "pos": {  # 基于 (0.42, -0.03, 0.8721)
                    "per_dim": [
                        {"uniform": [0.39, 0.54]},    # x
                        {"uniform": [-0.56, -0.42]},   # y
                        {"set": 0.8721}               # z 固定
                    ]
                }
            }
        },
        {
            "select": ".//body[@name='marker2']/geom",
            "attributes": {
                "rgba": {"color": {"alpha": [1.0, 1.0]}}
            }
        },

        # ---- box_grab 两个几何的颜色 ----
        {
            "select": ".//geom[@name='box_on_belt']",
            "attributes": {
                "rgba": {"color": {"alpha": [1.0, 1.0]}}
            }
        },
        {
            "select": ".//geom[@name='box_base_disk']",
            "attributes": {
                "rgba": {"color": {"alpha": [1.0, 1.0]}}
            }
        },

        # ---- 光照与视角 ----
        {
            "select": ".//visual/headlight",
            "attributes": {
                "diffuse": {"uniform": [0.2, 0.6], "ndim": 3},
                "ambient": {"per_dim": [
                {"uniform": [0.0, 0.3]},
                {"uniform": [0.0, 0.3]},
                {"uniform": [0.0, 0.3]}
                ]},
                "specular": {"uniform": [0.0, 0.2], "ndim": 3}
            }
        }
    ]
}

cfg2 = {
    "rules": [
        # ---- marker1：位置 + 颜色 ----
        {
            "select": ".//body[@name='marker1']",
            "attributes": {
                "pos": {  # 基于 (0.52, -0.50, 0.8721)
                    "per_dim": [
                        {"uniform": [0.35, 0.45]},    # x
                        {"uniform": [0.05, 0.15]},  # y
                        {"set": 0.89}               # z 固定
                    ]
                }
            }
        },
        {
            "select": ".//body[@name='marker1']/geom",
            "attributes": {
                "rgba": {"color": {"alpha": [1.0, 1.0]}}  # 随机 RGB，alpha=1
            }
        },

        # ---- marker2：位置 + 颜色 ----
        {
            "select": ".//body[@name='marker2']",
            "attributes": {
                "pos": {  # 基于 (0.42, -0.03, 0.8721)
                    "per_dim": [
                        {"uniform": [0.3, 0.50]},    # x
                        {"uniform": [0.5, 0.65]},   # y
                        {"set": 0.871}               # z 固定
                    ]
                }
            }
        },
        {
            "select": ".//body[@name='marker2']/geom",
            "attributes": {
                "rgba": {"color": {"alpha": [1.0, 1.0]}}
            }
        },

        # ---- box_grab 两个几何的颜色 ----
        {
            "select": ".//geom[@name='box_on_belt']",
            "attributes": {
                "rgba": {"color": {"alpha": [1.0, 1.0]}}
            }
        },
        {
            "select": ".//geom[@name='box_base_disk']",
            "attributes": {
                "rgba": {"color": {"alpha": [1.0, 1.0]}}
            }
        },

        # ---- 光照与视角 ----
        {
            "select": ".//visual/headlight",
            "attributes": {
                "diffuse": {"uniform": [0.2, 0.6], "ndim": 3},
                "ambient": {"per_dim": [
                {"uniform": [0.0, 0.3]},
                {"uniform": [0.0, 0.3]},
                {"uniform": [0.0, 0.3]}
                ]},
                "specular": {"uniform": [0.0, 0.2], "ndim": 3}
            }
        }
    ]
}

# 其他辅助函数保持不变
def read_score_from_file(score_file: str) -> int:
    try:
        with open(score_file, "r") as f:
            line = f.readline().strip()
            return int(line)
    except Exception:
        return None

def pre_run_cleanup(scores_dir: str, score_file: str):
    os.makedirs(scores_dir, exist_ok=True)
    targets = [
        os.path.join(scores_dir, "score.json"),
        score_file,
        os.path.splitext(score_file)[0] + ".json",
    ]
    for p in targets:
        try:
            os.remove(p)
        except FileNotFoundError:
            pass
        except Exception:
            pass

def write_score_json(path, task_id, scores, comp_sum):
    import os, json

    # 1) 总分平均（按有效轮次）
    valid_scores = [s for s in scores if isinstance(s, (int, float))]
    valid_cycles = len(valid_scores)
    avg_total = (sum(valid_scores) / valid_cycles) if valid_cycles else 0.0

    # 2) 获取 keys + process_keys
    keys = set(comp_sum.keys())
    process_keys, time_key = [], "time"
    try:
        catalog_path = os.path.join(os.path.dirname(path), "components_catalog.json")
        if os.path.exists(catalog_path):
            with open(catalog_path, "r") as cf:
                catalog = json.load(cf)
            expected = catalog.get(str(task_id)) or catalog.get(task_id) or []
            keys |= set(expected)
            # 从 catalog 里分离出 time 和 process 部分
            process_keys = [k for k in expected if k.lower() != "time"]
            if "time" in expected:
                time_key = "time"
    except Exception:
        pass

    # 3) 原子项平均
    denom = valid_cycles if valid_cycles else 1
    components_avg = {}
    for k in sorted(keys):
        total_for_k = float(comp_sum.get(k, 0.0))
        components_avg[k] = total_for_k / denom

    # 4) 聚合 process
    process_avg = sum(components_avg.get(k, 0.0) for k in process_keys)
    for k in process_keys:  # 删除原子项
        components_avg.pop(k, None)
    components_avg["process"] = process_avg

    payload = {
        "task_id": task_id,
        "valid_cycles": valid_cycles,
        "avg_total_score": avg_total,
        "components_avg": components_avg
    }
    with open(path, "w") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)

def try_read_detail_json(score_file):
    try:
        base, _ = os.path.splitext(score_file)
        detail_path = base + ".json"
        if os.path.exists(detail_path):
            with open(detail_path, "r") as f:
                return json.load(f)
    except Exception:
        pass
    return None

def append_history(history_file: str, cycle_idx: int, score: int):
    try:
        os.makedirs(os.path.dirname(history_file), exist_ok=True)
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with open(history_file, "a") as f:
            f.write(f"{ts}\tcycle={cycle_idx}\tscore={score}\n")
    except Exception:
        pass

def run_task(task_id: int, headless: bool):
    print(f"{GREEN}[SMART DEPLOY]{RESET} 启动数据挑战智能部署系统")

    print("[INFO] 启动 roscore...")
    roscore_process = subprocess.Popen(
        ["roscore"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid
    )
    time.sleep(1)

    # 配置虚拟屏幕参数
    if headless:
        task_env = os.environ.copy()
        task_env["MUJOCO_HEADLESS"] = "1"
    else:
        task_env = os.environ.copy()

    task_script = os.path.join(SCRIPT_DIR, f"eval{task_id}.py")
    launch_file = f"load_kuavo_mujoco_sim{task_id}.launch"

    if not os.path.exists(task_script):
        print(f"[ERROR] 任务脚本不存在：{task_script}")
        sys.exit(1)

    # 分数文件与历史文件
    scores_dir = os.path.join(SCRIPT_DIR, "scores")
    score_file = os.path.join(scores_dir, f"score_task{task_id}_last.txt")

    pre_run_cleanup(scores_dir, score_file)
    scores = []
    components_sum = {}
    score_json_path = os.path.join(scores_dir, "score.json")
    os.makedirs(scores_dir, exist_ok=True)

    cfg_map = {
        1: cfg1,
        2: cfg2,
    }

    scene_map = {
        1: "scene1.xml",
        2: "scene2.xml",
        3: "scene3.xml",
        4: "scene4.xml",
    }

    cfg = cfg_map[task_id]
    scene_file = scene_map[task_id]
    scene_path = f"/root/kuavo_ws/src/data_challenge_simulator/models/biped_s45/xml/{scene_file}"

    cycle_idx = 1
    while True:
        seed = 242 + cycle_idx
        n_changed = randomize_mjcf(
            in_path=scene_path,
            out_path=scene_path,
            config=cfg,
            seed=seed
        )

        print(f"\n{CYAN}[CYCLE {cycle_idx}]{RESET} === 启动仿真环境：{launch_file} ===")
        launch_process = subprocess.Popen(
            ['roslaunch', 'data_challenge_simulator', launch_file],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
            env=task_env
        )

        try:
            # ⭐ 关键改进：智能等待系统完全启动
            success = wait_for_system_ready(timeout_seconds=60)

            if not success:
                print(f"{RED}[ERROR]{RESET} 系统启动超时或失败，跳过本轮")
                continue

            # 最终健康检查
            if not check_system_health():
                print(f"{YELLOW}[WARNING]{RESET} 系统健康检查未通过，但继续执行...")

            print(f"{GREEN}[START TASK]{RESET} 运行任务脚本：{task_script}")
            env = os.environ.copy()
            env['SCORE_FILE'] = score_file

            EXAMPLES_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '..'))
            PKG_ROOT = os.path.dirname(EXAMPLES_DIR)
            env['PYTHONPATH'] = PKG_ROOT + os.pathsep + env.get('PYTHONPATH', '')

            # 先清空旧分数文件
            try:
                if os.path.exists(score_file):
                    os.remove(score_file)
            except Exception:
                pass

            cmd = ['python3', task_script, '--seed', str(seed)]
            task_process = subprocess.Popen(cmd, env=env)

            # 等到任务脚本退出
            task_process.wait()
            print("[INFO] 任务脚本退出，读取本轮最终得分...")

            # 读取分数并处理结果
            score = read_score_from_file(score_file)
            if score is None:
                valid_cnt = len(scores)
                avg = (sum(scores) / valid_cnt) if valid_cnt > 0 else 0.0
                print(
                    f"{CYAN}[RESULT]{RESET} "
                    f"第 {YELLOW}{cycle_idx}{RESET} 轮：{YELLOW}无成绩（reset 轮）{RESET} | "
                    f"当前平均分: {GREEN}{avg:.2f}{RESET} （有效 {valid_cnt} 轮）"
                )

                detail = try_read_detail_json(score_file)
                if detail and isinstance(detail.get("components"), dict):
                    comps = detail["components"]
                    for k, v in comps.items():
                        if isinstance(v, (int, float)):
                            components_sum[k] = components_sum.get(k, 0.0) + float(v)

                write_score_json(score_json_path, task_id, scores, components_sum)

            else:
                scores.append(score)
                valid_cnt = len(scores)
                avg = sum(scores) / valid_cnt if valid_cnt > 0 else 0.0

                detail = try_read_detail_json(score_file)
                if detail and isinstance(detail.get("components"), dict):
                    comps = detail["components"]
                    for k, v in comps.items():
                        if isinstance(v, (int, float)):
                            components_sum[k] = components_sum.get(k, 0.0) + float(v)

                write_score_json(score_json_path, task_id, scores, components_sum)

                print(
                    f"{CYAN}[RESULT]{RESET} "
                    f"第 {YELLOW}{cycle_idx}{RESET} 轮得分: {GREEN}{score}{RESET} | "
                    f"当前平均分: {GREEN}{avg:.2f}{RESET} （有效 {valid_cnt} 轮）"
                )

        except KeyboardInterrupt:
            print(f"\n{YELLOW}[INFO]{RESET} 收到中断信号，正在退出循环...")
            break

        finally:
            # 关闭仿真环境
            try:
                write_score_json(score_json_path, task_id, scores, components_sum)
                os.killpg(os.getpgid(launch_process.pid), signal.SIGTERM)
            except ProcessLookupError:
                pass
            time.sleep(1)
            print(f"{BLUE}[CLEANUP]{RESET} 仿真环境已关闭。")

        cycle_idx += 1

    # 清理roscore
    try:
        os.killpg(os.getpgid(roscore_process.pid), signal.SIGTERM)
    except:
        pass

def main(headless, task_id):
    if task_id is None:
        while task_id not in [1, 2, 3, 4]:
            print("========== 智能模型推理系统 ==========")
            print("请选择任务编号（1-4）：")
            print("1: 任务1 —— 传送带物品分拣")
            print("2: 任务2 —— 传送带物品称重")
            print("3: 任务3 —— 物品翻面")
            print("4: 任务4 —— 货架物品运送")
            try:
                task_id = int(input("请输入任务编号 (1-4): ").strip())
            except Exception:
                task_id = None

    run_task(task_id, headless)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", default=False, help="是否使用无头模式")
    parser.add_argument("--task_id", type=int, choices=[1, 2, 3, 4], help="任务编号 (1-4)")
    args = parser.parse_args()
    headless = args.headless
    task_id = args.task_id
    main(headless, task_id)