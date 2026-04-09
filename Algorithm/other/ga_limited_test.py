# batch_test_ga_limited.py
"""
遗传算法 (Genetic Algorithm) 批量测试 - 有限电池容量版本
简单版本，类似贪心算法的批量测试
"""

import numpy as np
import os
import sys
import time
import json
import pandas as pd
from typing import Dict, List, Any

# --- 路径设置 ---
current_script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_script_dir)
if project_root not in sys.path:
    sys.path.insert(0, project_root)

try:
    from Scenario.uav_scheduling_limited import UAVSchedulingLimitedEnv
    from Scenario.WSNsimulation import WSNSimulation
    from ga_limited import RoadNetworkWithChargingNodes, GeneticAlgorithmLimited, GASimulatorLimited
except ImportError as e:
    print(f"导入模块失败: {e}")
    sys.exit(1)

# 创建结果保存目录
RESULTS_DIR = os.path.join(current_script_dir, "ga_limited_batch_results")
os.makedirs(RESULTS_DIR, exist_ok=True)


def run_single_test(seed: int, network: RoadNetworkWithChargingNodes) -> Dict[str, Any]:
    """
    运行单个种子的测试（有限电池版本）
    """
    start_time = time.time()
    
    try:
        # 创建 WSNSimulation 实例
        wsn_sim = WSNSimulation(
            data_input=network.data,
            sensor_battery=240000.0,
            relay_battery=800000.0,
            station_battery=1000000.0,
            seed=seed
        )
        
        # 获取 WSN 仿真数据
        wsn_data = wsn_sim.get_data()
        point_info = wsn_data['point_info']
        
        # 统计需要充电的节点
        charging_nodes_info = [n for n in point_info if n['battery_info'].get('needs_charge', False)]
        sensors_count = len([n for n in charging_nodes_info if n['type'] == 'sensor'])
        relays_count = len([n for n in charging_nodes_info if n['type'] == 'relay'])
        
        # GA 路径规划（有限电池版本）
        planner_start = time.time()
        ga = GeneticAlgorithmLimited(
            network=network,
            charging_node_info=point_info,
            agent_max_battery=100000000.0,
            agent_move_consumption=1000.0,
            agent_tx_consumption=100.0,
            sensor_max_battery=240000.0,
            gateway_max_battery=800000.0,
            min_charging_rate=0.7,
            population_size=150,
            max_iterations=400,
            crossover_rate=0.9,
            mutation_rate=0.15,
            elite_ratio=0.1,
            stagnation_limit=100
        )
        plan = ga.optimize(verbose=False)  # 批量测试时不打印详细信息
        planning_time = time.time() - planner_start
        
        # UAV 仿真（有限电池版本）
        simulator = GASimulatorLimited(wsn_data)
        results = simulator.run_sequence(plan['complete_sequence'], plan['charging_sequence'])
        
        elapsed_time = time.time() - start_time
        
        # 构建结果字典
        result = {
            'seed': seed,
            'planning_time_s': planning_time,
            'simulation_time_s': elapsed_time,
            'total_steps': results['total_steps'],
            'total_path_length_m': results['total_distance'],
            'charged_node_count': results['charged_count'],
            'total_node_count': results['total_nodes'],
            'num_trips': results['num_trips'],  # 实际行程次数
            'num_trips_plan': plan['num_trips'],  # 规划行程次数
            'eue': results['eue'],
            'total_reward': results['total_reward'],
            'charge_energy_j': results['charge_energy'],
            'move_energy_j': results['move_energy'],
            'tx_energy_j': results['tx_energy'],
            'active_energy_j': results['active_energy'],
            'sensors_charged': sensors_count,
            'relays_charged': relays_count,
            'total_charging_nodes': len(charging_nodes_info),
            'optimal_distance_m': plan['total_distance'],
            'ga_iterations': plan['iterations']
        }
        
        return result
        
    except Exception as e:
        print(f"  种子 {seed} 测试失败: {e}")
        return {
            'seed': seed,
            'error': str(e),
            'planning_time_s': 0,
            'simulation_time_s': 0,
            'total_steps': 0,
            'total_path_length_m': 0,
            'charged_node_count': 0,
            'total_node_count': 0,
            'num_trips': 0,
            'num_trips_plan': 0,
            'eue': 0,
            'total_reward': 0,
            'charge_energy_j': 0,
            'move_energy_j': 0,
            'tx_energy_j': 0,
            'active_energy_j': 0,
            'sensors_charged': 0,
            'relays_charged': 0,
            'total_charging_nodes': 0,
            'optimal_distance_m': 0,
            'ga_iterations': 0
        }


def batch_test(seed_start: int = 0, seed_end: int = 99):
    """
    批量测试多个种子（有限电池版本）
    """
    print("="*70)
    print(f"遗传算法批量测试 - 有限电池容量版本 (种子 {seed_start} 到 {seed_end})")
    print("="*70)
    
    # 加载道路网络（只需加载一次）
    NETWORK_FILE = "./data/road_graph_with_nodes.json"
    print("\n[1] 加载道路网络...")
    try:
        network = RoadNetworkWithChargingNodes(NETWORK_FILE)
    except FileNotFoundError:
        print(f"错误: 找不到文件 {NETWORK_FILE}")
        print("请先运行 deploy_nodes.py 生成道路网络数据")
        return
    
    # 存储所有结果
    all_results = []
    failed_seeds = []
    
    total_seeds = seed_end - seed_start + 1
    print(f"\n[2] 开始批量测试 (共 {total_seeds} 个种子)...")
    
    batch_start_time = time.time()
    
    for i, seed in enumerate(range(seed_start, seed_end + 1)):
        print(f"\n进度: {i+1}/{total_seeds} (种子: {seed})", end="")
        
        result = run_single_test(seed, network)
        
        if 'error' in result:
            failed_seeds.append(seed)
            print(f" ❌ 失败")
        else:
            all_results.append(result)
            print(f" ✅ 最优距离: {result['optimal_distance_m']/1000:.2f}km, "
                  f"行程数: {result['num_trips_plan']}, "
                  f"EUE: {result['eue']:.4f}, "
                  f"充电: {result['charged_node_count']}/{result['total_charging_nodes']}")
    
    batch_time = time.time() - batch_start_time
    
    # ==================== 结果统计 ====================
    print("\n" + "="*70)
    print("批量测试完成")
    print("="*70)
    print(f"总耗时: {batch_time:.2f}s ({batch_time/60:.2f}分钟)")
    print(f"成功测试数: {len(all_results)}/{total_seeds}")
    print(f"失败种子: {failed_seeds}" if failed_seeds else "无失败种子")
    
    if all_results:
        # 转换为 DataFrame
        df = pd.DataFrame(all_results)
        
        # 保存所有结果到 CSV
        csv_filename = f"ga_limited_batch_results_{seed_start}_{seed_end}.csv"
        csv_path = os.path.join(RESULTS_DIR, csv_filename)
        df.to_csv(csv_path, index=False)
        print(f"\n所有结果已保存到: {csv_path}")
        
        # 计算平均值并打印
        print("\n" + "="*70)
        print("平均值统计")
        print("="*70)
        
        avg_stats = {
            '总奖励': df['total_reward'].mean(),
            'EUE': df['eue'].mean(),
            '总路径长度(km)': df['total_path_length_m'].mean() / 1000,
            '最优距离(km)': df['optimal_distance_m'].mean() / 1000,
            '充电节点数': df['charged_node_count'].mean(),
            '总节点数': df['total_node_count'].mean(),
            '行程次数(实际)': df['num_trips'].mean(),
            '行程次数(规划)': df['num_trips_plan'].mean(),
            '规划时间(s)': df['planning_time_s'].mean(),
            '仿真时间(s)': df['simulation_time_s'].mean(),
            'GA迭代次数': df['ga_iterations'].mean(),
            '充电能量(J)': df['charge_energy_j'].mean(),
            '移动能量(J)': df['move_energy_j'].mean(),
            '传输能量(J)': df['tx_energy_j'].mean(),
            '激活能量(J)': df['active_energy_j'].mean(),
            '传感器充电数': df['sensors_charged'].mean(),
            '中继充电数': df['relays_charged'].mean(),
            '总充电节点数': df['total_charging_nodes'].mean()
        }
        
        for key, value in avg_stats.items():
            if '能量' in key or '时间' in key or '距离' in key:
                print(f"{key}: {value:.2f}")
            elif key == 'EUE':
                print(f"{key}: {value:.4f}")
            else:
                print(f"{key}: {value:.1f}")
        
        # 保存平均值到单独的CSV
        avg_df = pd.DataFrame([avg_stats])
        avg_path = os.path.join(RESULTS_DIR, f"ga_limited_average_stats_{seed_start}_{seed_end}.csv")
        avg_df.to_csv(avg_path, index=False)
        print(f"\n平均值已保存到: {avg_path}")
        
        # 打印前5个种子的详细信息
        print("\n" + "="*70)
        print("前5个种子的详细结果")
        print("="*70)
        print(df[['seed', 'eue', 'optimal_distance_m', 'total_path_length_m', 
                  'charged_node_count', 'num_trips_plan', 'ga_iterations']].head().to_string(index=False))
    
    print("\n" + "="*70)
    print(f"所有结果已保存到: {RESULTS_DIR}")
    print("="*70)


def main():
    """主函数"""
    # 测试种子 0-99
    batch_test(seed_start=0, seed_end=99)


if __name__ == "__main__":
    main()