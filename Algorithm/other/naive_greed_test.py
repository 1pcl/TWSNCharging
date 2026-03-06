import numpy as np
import argparse
import os
import sys
import json
import logging
import pandas as pd
from typing import Dict, List, Any, Set
import time
from collections import defaultdict
from datetime import datetime

# --- 路径设置 ---
current_script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_script_dir)
if project_root not in sys.path:
    sys.path.insert(0, project_root)

try:
    from Scenario.UAVSchedulingEnv import UAVSchedulingEnv
    from Scenario.WSNsimulation import WSNSimulation 
    from naive_greed import NaiveHeuristicGreedyAgent, run_greedy_simulation
except ImportError as e:
    logging.error(f"导入自定义模块失败: {e}。")
    sys.exit(1)

def batch_run_simulation(start_seed: int, end_seed: int, data_input: str, output_dir: str = "results"):
    """
    批量运行仿真并将结果保存到文件中。
    """
    # 1. 创建输出目录
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filename = os.path.join(output_dir, f"naive_greedy_results_{timestamp}.csv")
    summary_filename = os.path.join(output_dir, f"naive_greedy_summary_{timestamp}.txt")

    all_results: List[Dict[str, Any]] = []
    total_run_time = 0.0
    
    # ----------------------------------------------------------------------
    # 2. 批量运行
    # ----------------------------------------------------------------------
    header = f"\n| Seed | Total Steps | Total Reward  | Charged Nodes | Avg EUE | Path Length (m) | Run Time (s) |"
    divider = "|--------|--------------|------------|---------------|-----------------|--------------|--------------|"
    
    print(f"\n批量运行 贪心策略 仿真 (种子: {start_seed}-{end_seed})")
    print(header)
    print(divider)
    
    for seed in range(start_seed, end_seed + 1):
        wsnSimulation = WSNSimulation(
            data_input=data_input, 
            sensorBattery=10800.0, 
            gatewayBattery=10800 * 2.0, 
            seed=seed 
        )
        data = wsnSimulation.get_data()
        graph_G = data["graph_G"]
        point_info = data["point_info"]
        total_nodes = len(point_info) - 1 
        
        env = UAVSchedulingEnv(graph_G=graph_G, point_info=point_info, max_min= data['max_min'],seed=seed)
        agent = NaiveHeuristicGreedyAgent(env)
        
        start_time = time.time()
        results = run_greedy_simulation(env, agent, print_log=False) 
        run_time = time.time() - start_time
        total_run_time += run_time
        
        # 记录结果
        results['seed'] = seed
        results['run_time'] = run_time
        all_results.append(results)
        
        # 实时打印到控制台
        print(f"| {seed:<4} | {results['total_steps']:<11} | {results['total_reward']:.2f}{'':<9}| {results['charged_node_count']:<13} | {results['total_eue']:.4f} | {results['total_path_length']:.2f}{'':<13} | {run_time:.4f}{'':<7} |")

    # ----------------------------------------------------------------------
    # 3. 数据处理与保存
    # ----------------------------------------------------------------------
    # 保存详细 CSV
    df = pd.DataFrame(all_results)
    df.to_csv(csv_filename, index=False)
    
    # 计算平均值
    num_runs = len(all_results)
    avg_metrics = df.mean(numeric_only=True).to_dict()
    avg_run_time = total_run_time / num_runs if num_runs > 0 else 0.0

    # 准备摘要内容
    summary_content = [
        "="*50,
        f"贪心算法性能分析摘要 ({timestamp})",
        "="*50,
        f"测试种子范围: {start_seed} - {end_seed}",
        f"测试次数: {num_runs}",
        f"可充电总节点数: {total_nodes}",
        "-"*50,
        f"平均总步数: {avg_metrics.get('total_steps', 0):.2f}",
        f"平均累积奖励: {avg_metrics.get('total_reward', 0):.2f}",
        f"平均充电节点数: {avg_metrics.get('charged_node_count', 0):.2f}",
        f"平均能量利用率: {avg_metrics.get('total_eue', 0):.4f}",
        f"平均总路径长度: {avg_metrics.get('total_path_length', 0):.2f} m",
        f"平均单次仿真耗时: {avg_run_time:.4f} s",
        "="*50
    ]

    # 写入 TXT 文件
    with open(summary_filename, 'w', encoding='utf-8') as f:
        f.write("\n".join(summary_content))

    # 同时在控制台打印摘要
    for line in summary_content:
        print(line)

    print(f"\n[完成] 详细结果已保存至: {csv_filename}")
    print(f"[完成] 统计摘要已保存至: {summary_filename}")
    
    return all_results, avg_metrics

if __name__ == "__main__":
    START_SEED = 0
    END_SEED = 100
    # DATA_FILE = "./data/chargingScheduling_data.json"

    with open("./data/chargingScheduling_data.json", "r") as f:
        data = json.load(f)
    
    # 执行
    batch_run_simulation(START_SEED, END_SEED, data)