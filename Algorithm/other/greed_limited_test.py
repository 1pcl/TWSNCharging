# batch_test_greedy_limited.py
import numpy as np
import os
import sys
import time
import json
import pandas as pd
from typing import Dict, List, Any
from collections import defaultdict

# --- 路径设置 ---
current_script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_script_dir)
if project_root not in sys.path:
    sys.path.insert(0, project_root)

try:
    from Scenario.uav_scheduling_limited import UAVSchedulingLimitedEnv
    from Scenario.WSNsimulation import WSNSimulation
    from greed_limited import RoadNetworkWithChargingNodes, GreedyPathPlannerWithBattery, UAVSimulator
except ImportError as e:
    print(f"导入模块失败: {e}")
    sys.exit(1)

# 创建结果保存目录
RESULTS_DIR = os.path.join(current_script_dir, "batch_results_limited")
os.makedirs(RESULTS_DIR, exist_ok=True)


def run_single_test(seed: int, network: RoadNetworkWithChargingNodes) -> Dict[str, Any]:
    """
    运行单个种子的测试（有限电池版本）
    
    Args:
        seed: 随机种子
        network: 道路网络对象
    
    Returns:
        包含所有指标的字典
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
        
        # 贪心路径规划（考虑电池容量）
        planner_start = time.time()
        planner = GreedyPathPlannerWithBattery(
            network, 
            point_info,
            agent_max_battery=100000000.0,  
            agent_move_consumption=1000.0,
            agent_tx_consumption=100.0,
            sensor_max_battery=240000.0,
            gateway_max_battery=800000.0
        )
        plan = planner.get_plan()
        planning_time = time.time() - planner_start
        
        # UAV 仿真
        simulator = UAVSimulator(wsn_data)
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
            'num_trips_planned': plan['num_trips'],  # 规划的次数
            'num_trips_actual': results['num_trips'],  # 实际执行的次数
            'eue': results['eue'],
            'total_reward': results['total_reward'],
            'charge_energy_j': results['charge_energy'],
            'move_energy_j': results['move_energy'],
            'tx_energy_j': results['tx_energy'],
            'active_energy_j': results['active_energy'],
            'sensors_charged': sensors_count,
            'relays_charged': relays_count,
            'total_charging_nodes': len(charging_nodes_info)
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
            'num_trips_planned': 0,
            'num_trips_actual': 0,
            'eue': 0,
            'total_reward': 0,
            'charge_energy_j': 0,
            'move_energy_j': 0,
            'tx_energy_j': 0,
            'active_energy_j': 0,
            'sensors_charged': 0,
            'relays_charged': 0,
            'total_charging_nodes': 0
        }


def batch_test(seed_start: int = 0, seed_end: int = 99):
    """
    批量测试多个种子（有限电池版本）
    
    Args:
        seed_start: 起始种子
        seed_end: 结束种子（包含）
    """
    print("="*70)
    print(f"贪心算法批量测试（有限电池版本）(种子 {seed_start} 到 {seed_end})")
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
            print(f" ✅ EUE: {result['eue']:.4f}, 距离: {result['total_path_length_m']/1000:.2f}km, "
                  f"行程: {result['num_trips_actual']}, 充电: {result['charged_node_count']}/{result['total_charging_nodes']}")
    
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
        csv_filename = f"batch_results_limited_{seed_start}_{seed_end}.csv"
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
            '充电节点数': df['charged_node_count'].mean(),
            '总节点数': df['total_node_count'].mean(),
            '规划行程数': df['num_trips_planned'].mean(),
            '实际行程数': df['num_trips_actual'].mean(),
            '规划时间(s)': df['planning_time_s'].mean(),
            '仿真时间(s)': df['simulation_time_s'].mean(),
            '充电能量(J)': df['charge_energy_j'].mean(),
            '移动能量(J)': df['move_energy_j'].mean(),
            '传输能量(J)': df['tx_energy_j'].mean(),
            '激活能量(J)': df['active_energy_j'].mean(),
            '传感器充电数': df['sensors_charged'].mean(),
            '中继充电数': df['relays_charged'].mean(),
            '总充电节点数': df['total_charging_nodes'].mean()
        }
        
        for key, value in avg_stats.items():
            if '能量' in key or '时间' in key:
                print(f"{key}: {value:.2f}")
            elif key == 'EUE':
                print(f"{key}: {value:.4f}")
            else:
                print(f"{key}: {value:.1f}")
        
        # 保存平均值到单独的CSV
        avg_df = pd.DataFrame([avg_stats])
        avg_path = os.path.join(RESULTS_DIR, f"average_stats_limited_{seed_start}_{seed_end}.csv")
        avg_df.to_csv(avg_path, index=False)
        print(f"\n平均值已保存到: {avg_path}")
        
        # 打印前5个种子的详细信息
        print("\n" + "="*70)
        print("前5个种子的详细结果")
        print("="*70)
        print(df[['seed', 'eue', 'total_path_length_m', 'charged_node_count', 
                  'num_trips_planned', 'num_trips_actual']].head().to_string(index=False))
        
        # 统计行程数分布
        print("\n" + "="*70)
        print("行程数统计")
        print("="*70)
        trip_counts = df['num_trips_actual'].value_counts().sort_index()
        for trips, count in trip_counts.items():
            print(f"  行程数 {trips}: {count} 个种子 ({count/len(df)*100:.1f}%)")
        
        # 计算充电完成率
        charge_completion_rate = (df['charged_node_count'] / df['total_charging_nodes']).mean()
        print(f"\n平均充电完成率: {charge_completion_rate*100:.2f}%")
    
    print("\n" + "="*70)
    print(f"所有结果已保存到: {RESULTS_DIR}")
    print("="*70)


def main():
    """主函数"""
    # 测试种子 0-99
    batch_test(seed_start=0, seed_end=99)


if __name__ == "__main__":
    main()