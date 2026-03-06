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
    from NPSO import NPSOSolver, run_detailed_simulation_npso
except ImportError as e:
    logging.error(f"导入自定义模块失败: {e}。请检查 NPSO.py 是否在同一目录下。")
    sys.exit(1)

def batch_run_npso(start_seed: int, end_seed: int, data_input: str, output_dir: str = "results"):
    """
    批量运行 NPSO 优化算法并将结果保存到文件中。
    """
    # 1. 创建输出目录
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filename = os.path.join(output_dir, f"npso_results_{timestamp}.csv")
    summary_filename = os.path.join(output_dir, f"npso_summary_{timestamp}.txt")

    all_results: List[Dict[str, Any]] = []
    total_optimization_time = 0.0
    
    # ----------------------------------------------------------------------
    # 2. 批量运行
    # ----------------------------------------------------------------------
    header = f"\n| Seed | Reward | Charged | Avg EUE | Path (m) | Opt Time (s) |"
    divider = "|--------|----------|---------|---------|----------|--------------|"
    
    print(f"\n批量运行 NPSO 方案 C 仿真 (种子: {start_seed}-{end_seed})")
    print(header)
    print(divider)
    
    for seed in range(start_seed, end_seed + 1):
        # 初始化环境（保持与贪心测试一致的参数）
        try:
            wsnSimulation = WSNSimulation(
                data_input=data_input, 
                sensorBattery=10800.0, 
                gatewayBattery=21600.0, 
                seed=seed 
            )
            data = wsnSimulation.get_data()
            env = UAVSchedulingEnv(
                graph_G=data["graph_G"], 
                point_info=data["point_info"], 
                max_min=data['max_min'],
                seed=seed
            )
        except Exception as e:
            print(f"种子 {seed} 环境初始化失败: {e}")
            continue

        # --- 执行 NPSO 优化 ---
        start_opt = time.time()
        solver = NPSOSolver(env, n_particles=100, max_iterations=1000) 
        solver.solve()
        opt_time = time.time() - start_opt
        total_optimization_time += opt_time
        
        # --- 执行最终决策仿真获取详细指标 ---
        total_reward, _, stats = run_detailed_simulation_npso(env, solver.gbest_path, solver.gbest_times)
        
        # 记录结果
        res_entry = {
            'seed': seed,
            'total_reward': total_reward,
            'charged_node_count': stats['charged_nodes'],
            'total_eue': stats['avg_eue'],
            'total_path_length': stats['total_path_length'],
            'total_stay_time': stats['total_stay_time'],
            'optimization_time': opt_time,
            'total_steps': stats['total_steps']
        }
        all_results.append(res_entry)
        
        # 实时打印到控制台
        print(f"| {seed:<6} | {total_reward:<8.2f} | {stats['charged_nodes']:<7} | {stats['avg_eue']:<7.4f} | {stats['total_path_length']:<8.2f} | {opt_time:<12.2f} |")

    # ----------------------------------------------------------------------
    # 3. 数据处理与保存
    # ----------------------------------------------------------------------
    if not all_results:
        print("没有运行结果。")
        return

    df = pd.DataFrame(all_results)
    df.to_csv(csv_filename, index=False)
    
    avg_metrics = df.mean(numeric_only=True).to_dict()
    num_runs = len(all_results)

    summary_content = [
        "="*60,
        f"NPSO 算法性能分析摘要 ({timestamp})",
        "="*60,
        f"测试种子范围: {start_seed} - {end_seed}",
        f"测试成功次数: {num_runs}",
        "-"*60,
        f"平均累积奖励: {avg_metrics.get('total_reward', 0):.2f}",
        f"平均充电节点数: {avg_metrics.get('charged_node_count', 0):.2f}",
        f"平均能量利用率 (EUE): {avg_metrics.get('total_eue', 0):.4f}",
        f"平均总路径长度: {avg_metrics.get('total_path_length', 0):.2f} m",
        f"平均停留充电时间: {avg_metrics.get('total_stay_time', 0):.2f} s",
        f"平均单次优化耗时: {avg_metrics.get('optimization_time', 0):.2f} s",
        "="*60
    ]

    with open(summary_filename, 'w', encoding='utf-8') as f:
        f.write("\n".join(summary_content))

    for line in summary_content:
        print(line)

    print(f"\n[完成] 详细结果保存至: {csv_filename}")
    
    return all_results, avg_metrics

if __name__ == "__main__":
    # 配置参数
    START_SEED = 0
    END_SEED = 100 
    # DATA_FILE = "./data/chargingScheduling_data.json"
    with open("./data/chargingScheduling_data.json", "r") as f:
        data = json.load(f)
    
    # 执行批量测试
    batch_run_npso(START_SEED, END_SEED, data)