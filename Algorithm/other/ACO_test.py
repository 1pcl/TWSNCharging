import numpy as np
import os
import sys
import json
import logging
import pandas as pd
from typing import Dict, List, Any
import time
from datetime import datetime

# --- 路径设置 ---
current_script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_script_dir)
if project_root not in sys.path:
    sys.path.insert(0, project_root)

try:
    from Scenario.UAVSchedulingEnv import UAVSchedulingEnv
    from Scenario.WSNsimulation import WSNSimulation 
    # 确保 ACO.py 中有 ACOSolver 类
    from other.ACO import ACOSolver 
except ImportError as e:
    logging.error(f"导入自定义模块失败: {e}。请检查相关文件路径。")
    sys.exit(1)

def batch_run_aco(start_seed: int, end_seed: int, data_input: str, output_dir: str = "results"):
    """
    参考 NPSO_test，通过改变环境种子批量运行 ACO 算法并保存结果。
    """
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filename = os.path.join(output_dir, f"aco_results_{timestamp}.csv")
    summary_filename = os.path.join(output_dir, f"aco_summary_{timestamp}.txt")

    all_results = []

    print(f"开始 ACO 批量测试 (种子 {start_seed} 到 {end_seed})...")

    for seed in range(start_seed, end_seed + 1):
        # print(f"\n>>> 正在运行种子: {seed}")
        try:
            # 1. 环境初始化 (关键：使用当前的 seed)
            wsnSimulation = WSNSimulation(data_input=data_input, seed=seed)
            data = wsnSimulation.get_data()
            env = UAVSchedulingEnv(graph_G=data["graph_G"], point_info=data["point_info"], max_min=data['max_min'])

            # 2. 求解器初始化
            # 使用和你 NPSO 类似的迭代规模
            solver = ACOSolver(env, n_ants=100, n_iterations=1000, early_stop_patience=100)

            # 3. 运行优化
            start_time = time.time()
            best_path, best_durations, best_reward, exec_time = solver.solve()
            opt_time = time.time() - start_time

            # 4. 最终验证仿真获取指标 (使用统一接口)
            # 注意：这里调用的是我在上一个回复中建议你加在 ACO.py 里的统一仿真函数
            final_reward, steps_info = solver._run_path_simulation(best_path, best_durations, render=True)

            # 5. 统计指标 (与 NPSO_test 保持一致)
            total_dist = 0
            total_charge = 0
            total_cost = 0
            total_stay = 0
            
            for step in steps_info:
                info = step[3]
                total_dist += info.get("distance_needed", 0)
                total_charge += info.get("step_charge_energy", 0)
                total_cost += (info.get("step_move_energy", 0) + info.get("step_tx_energy", 0) + info.get("active_energy", 0))
                # 记录充电时长 (level 转时间逻辑需参考环境)
                total_stay += info.get("charge_time", 0) if isinstance(info.get("charge_time"), (int, float)) else 0

            eue = total_charge / total_cost if total_cost > 0 else 0

            # 存储单次结果
            result = {
                "seed": seed,
                "total_reward": round(final_reward, 4),
                "charged_node_count": len(best_path),
                "total_eue": round(eue, 6),
                "total_path_length": round(total_dist, 2),
                "total_stay_time": round(total_stay, 2),
                "optimization_time": round(opt_time, 2)
            }
            all_results.append(result)
            print(f"种子 {seed} 完成: Reward={result['total_reward']}, EUE={result['total_eue']}")

        except Exception as e:
            print(f"种子 {seed} 运行失败: {e}")
            continue

    # --- 保存结果与摘要 (完全参考 NPSO_test) ---
    if not all_results:
        print("没有成功运行的结果。")
        return

    df = pd.DataFrame(all_results)
    df.to_csv(csv_filename, index=False)
    
    avg_metrics = df.mean(numeric_only=True).to_dict()

    summary_content = [
        "="*60,
        f"ACO 算法性能分析摘要 ({timestamp})",
        "="*60,
        # f"测试数据: {data_input}",
        f"测试种子范围: {start_seed} - {end_seed}",
        f"测试成功次数: {len(all_results)}",
        "-"*60,
        f"平均累积奖励: {avg_metrics.get('total_reward', 0):.2f}",
        f"平均充电节点数: {avg_metrics.get('charged_node_count', 0):.2f}",
        f"平均能量利用率 (EUE): {avg_metrics.get('total_eue', 0):.4f}",
        f"平均总路径长度: {avg_metrics.get('total_path_length', 0):.2f} m",
        f"平均单次优化耗时: {avg_metrics.get('optimization_time', 0):.2f} s",
        "="*60
    ]

    summary_text = "\n".join(summary_content)
    with open(summary_filename, "w", encoding="utf-8") as f:
        f.write(summary_text)

    print("\n" + summary_text)
    print(f"\n结果已保存至: {output_dir}")

if __name__ == "__main__":
    START_SEED = 0
    END_SEED = 100 
    # DATA_FILE = "./data/chargingScheduling_data.json"
    with open("./data/chargingScheduling_data.json", "r") as f:
        data = json.load(f)
    batch_run_aco(START_SEED, END_SEED, data)