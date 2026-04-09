"""
改进蚁群算法批量测试 - 有限电池容量版本
参考论文: Wireless sensor network routing optimization based on improved ant colony algorithm
Heliyon 10 (2024) e23577

批量测试多个随机种子，统计平均性能指标（考虑无人车电池容量限制）
"""

import numpy as np
import os
import sys
import time
import json
import random
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
    from aco_limited import RoadNetworkWithChargingNodes, ImprovedAntColonyOptimizerLimited, ACOSimulatorLimited
except ImportError as e:
    print(f"导入模块失败: {e}")
    sys.exit(1)

# 创建结果保存目录
RESULTS_DIR = os.path.join(current_script_dir, "batch_results_improved_aco_limited")
os.makedirs(RESULTS_DIR, exist_ok=True)


def run_single_test(seed: int, network: RoadNetworkWithChargingNodes,
                    agent_max_battery: float = 100000000.0,
                    agent_move_consumption: float = 1000.0,
                    agent_tx_consumption: float = 100.0,
                    sensor_max_battery: float = 240000.0,
                    gateway_max_battery: float = 800000.0,
                    min_charging_rate: float = 0.7,
                    num_ants: int = 60, max_iterations: int = 400,
                    alpha0: float = 0.7, alpha1: float = 1.8, K: float = 0.008,
                    beta: float = 1.5, gamma: float = 0.5,
                    rho: float = 0.02, Q: float = 200.0,
                    tau_min: float = 0.0005, tau_max: float = 15.0,
                    distance_threshold: float = 6000.0,
                    elite_ant_ratio: float = 0.25,
                    exploration_rate: float = 0.2,
                    stagnation_limit: int = 100,
                    verbose: bool = False) -> Dict[str, Any]:
    """
    运行单个种子的测试（有限电池容量版本）
    """
    start_time = time.time()
    
    try:
        # 设置随机种子
        np.random.seed(seed)
        random.seed(seed)
        
        # 创建 WSNSimulation 实例
        wsn_sim = WSNSimulation(
            data_input=network.data,
            sensor_battery=sensor_max_battery,
            relay_battery=gateway_max_battery,
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
        
        # 改进蚁群算法路径规划（有限电池容量版本）
        planner_start = time.time()
        aco = ImprovedAntColonyOptimizerLimited(
            network=network,
            charging_node_info=point_info,
            agent_max_battery=agent_max_battery,
            agent_move_consumption=agent_move_consumption,
            agent_tx_consumption=agent_tx_consumption,
            sensor_max_battery=sensor_max_battery,
            gateway_max_battery=gateway_max_battery,
            min_charging_rate=min_charging_rate,
            num_ants=num_ants,
            max_iterations=max_iterations,
            alpha0=alpha0,
            alpha1=alpha1,
            K=K,
            beta=beta,
            gamma=gamma,
            rho=rho,
            Q=Q,
            tau_min=tau_min,
            tau_max=tau_max,
            distance_threshold=distance_threshold,
            elite_ant_ratio=elite_ant_ratio,
            exploration_rate=exploration_rate,
            stagnation_limit=stagnation_limit
        )
        plan = aco.optimize(verbose=verbose)
        planning_time = time.time() - planner_start
        
        # UAV 仿真（有限电池容量版本）
        simulator = ACOSimulatorLimited(wsn_data)
        results = simulator.run_sequence(plan['complete_sequence'], plan['charging_sequence'])
        
        elapsed_time = time.time() - start_time
        
        # 构建结果字典（所有值都是基本类型，便于保存CSV）
        result = {
            'seed': seed,
            'planning_time_s': planning_time,
            'simulation_time_s': elapsed_time,
            'total_steps': results['total_steps'],
            'total_path_length_m': results['total_distance'],
            'charged_node_count': results['charged_count'],
            'total_node_count': results['total_nodes'],
            'num_returns': results['num_trips'],
            'num_trips_planned': plan['num_trips'],
            'eue': results['eue'],
            'total_reward': results['total_reward'],
            'charge_energy_j': results['charge_energy'],
            'move_energy_j': results['move_energy'],
            'tx_energy_j': results['tx_energy'],
            'active_energy_j': results['active_energy'],
            'sensors_charged': sensors_count,
            'relays_charged': relays_count,
            'total_charging_nodes': len(charging_nodes_info),
            'aco_iterations': plan['iterations'],
            'aco_optimal_distance_m': plan['total_distance'],
            'aco_optimization_time_s': plan['optimization_time'],
            'aco_best_energy_metric': plan['best_energy_metric']
        }
        
        # 将行程详情转换为字符串（用于保存）
        trip_details_str = ','.join([str(len(trip)) for trip in plan['trips']])
        result['trip_details_str'] = trip_details_str
        result['num_trips'] = len(plan['trips'])  # 实际行程数
        
        return result
        
    except Exception as e:
        print(f"  种子 {seed} 测试失败: {e}")
        import traceback
        traceback.print_exc()
        return {
            'seed': seed,
            'error': str(e),
            'planning_time_s': 0,
            'simulation_time_s': 0,
            'total_steps': 0,
            'total_path_length_m': 0,
            'charged_node_count': 0,
            'total_node_count': 0,
            'num_returns': 0,
            'num_trips_planned': 0,
            'eue': 0,
            'total_reward': 0,
            'charge_energy_j': 0,
            'move_energy_j': 0,
            'tx_energy_j': 0,
            'active_energy_j': 0,
            'sensors_charged': 0,
            'relays_charged': 0,
            'total_charging_nodes': 0,
            'aco_iterations': 0,
            'aco_optimal_distance_m': 0,
            'aco_optimization_time_s': 0,
            'aco_best_energy_metric': 0,
            'trip_details_str': '',
            'num_trips': 0
        }


def batch_test(seed_start: int = 0, seed_end: int = 99,
               agent_max_battery: float = 100000000.0,
               agent_move_consumption: float = 1000.0,
               agent_tx_consumption: float = 100.0,
               sensor_max_battery: float = 240000.0,
               gateway_max_battery: float = 800000.0,
               min_charging_rate: float = 0.7,
               num_ants: int = 60, max_iterations: int = 400,
               alpha0: float = 0.7, alpha1: float = 1.8, K: float = 0.008,
               beta: float = 1.5, gamma: float = 0.5,
               rho: float = 0.02, Q: float = 200.0,
               tau_min: float = 0.0005, tau_max: float = 15.0,
               distance_threshold: float = 6000.0,
               elite_ant_ratio: float = 0.25,
               exploration_rate: float = 0.2,
               stagnation_limit: int = 100,
               verbose: bool = False):
    """
    批量测试多个种子（有限电池容量版本）
    """
    print("="*70)
    print(f"改进蚁群算法批量测试 - 有限电池容量版本 (种子 {seed_start} 到 {seed_end})")
    print("="*70)
    print(f"\n改进蚁群算法参数:")
    print(f"  - 蚂蚁数量: {num_ants}")
    print(f"  - 最大迭代次数: {max_iterations}")
    print(f"  - 动态α参数: α0={alpha0}, α1={alpha1}, K={K}")
    print(f"  - β (距离权重): {beta}")
    print(f"  - γ (能量紧急度权重): {gamma}")
    print(f"  - ρ (挥发率): {rho}")
    print(f"  - Q (信息素强度): {Q}")
    print(f"  - 信息素范围: [{tau_min}, {tau_max}]")
    print(f"  - 距离阈值 d0: {distance_threshold} m")
    print(f"  - 精英蚂蚁比例: {elite_ant_ratio}")
    print(f"  - 随机探索概率: {exploration_rate}")
    print(f"  - 停滞阈值: {stagnation_limit}")
    print(f"\n无人车参数:")
    print(f"  - 电池容量: {agent_max_battery/1000:.0f} kJ")
    print(f"  - 移动能耗: {agent_move_consumption} J/m")
    print(f"  - 传输能耗: {agent_tx_consumption} J/s")
    print(f"  - 最小充电效率: {min_charging_rate}")
    print(f"\n节点电池参数:")
    print(f"  - 传感器最大电池: {sensor_max_battery/1000:.0f} kJ")
    print(f"  - 中继最大电池: {gateway_max_battery/1000:.0f} kJ")
    
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
        
        result = run_single_test(
            seed=seed,
            network=network,
            agent_max_battery=agent_max_battery,
            agent_move_consumption=agent_move_consumption,
            agent_tx_consumption=agent_tx_consumption,
            sensor_max_battery=sensor_max_battery,
            gateway_max_battery=gateway_max_battery,
            min_charging_rate=min_charging_rate,
            num_ants=num_ants,
            max_iterations=max_iterations,
            alpha0=alpha0,
            alpha1=alpha1,
            K=K,
            beta=beta,
            gamma=gamma,
            rho=rho,
            Q=Q,
            tau_min=tau_min,
            tau_max=tau_max,
            distance_threshold=distance_threshold,
            elite_ant_ratio=elite_ant_ratio,
            exploration_rate=exploration_rate,
            stagnation_limit=stagnation_limit,
            verbose=verbose
        )
        
        if 'error' in result:
            failed_seeds.append(seed)
            print(f" ❌ 失败")
        else:
            all_results.append(result)
            print(f" ✅ EUE: {result['eue']:.4f}, 距离: {result['total_path_length_m']/1000:.2f}km, "
                  f"充电: {result['charged_node_count']}/{result['total_charging_nodes']}, "
                  f"行程: {result['num_trips_planned']}, 迭代: {result['aco_iterations']}, "
                  f"能量指标: {result['aco_best_energy_metric']:.4f}")
    
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
        
        # 删除不需要保存的列
        cols_to_drop = ['error'] if 'error' in df.columns else []
        if 'trip_details_str' in df.columns:
            # 保留但不需要打印
            pass
        
        df_to_save = df.drop(columns=cols_to_drop, errors='ignore')
        
        # 保存所有结果到 CSV
        csv_filename = f"improved_aco_limited_batch_results_{seed_start}_{seed_end}.csv"
        csv_path = os.path.join(RESULTS_DIR, csv_filename)
        df_to_save.to_csv(csv_path, index=False)
        print(f"\n所有结果已保存到: {csv_path}")
        
        # 验证文件是否保存成功
        if os.path.exists(csv_path):
            file_size = os.path.getsize(csv_path)
            print(f"  文件大小: {file_size/1024:.2f} KB")
        else:
            print(f"  ⚠️ 警告: 文件保存失败！")
        
        # 计算平均值并打印
        print("\n" + "="*70)
        print("改进蚁群算法平均值统计（有限电池容量版本）")
        print("="*70)
        
        avg_stats = {
            '总奖励': df['total_reward'].mean(),
            'EUE': df['eue'].mean(),
            '总路径长度(km)': df['total_path_length_m'].mean() / 1000,
            '最优路径距离(km)': df['aco_optimal_distance_m'].mean() / 1000,
            '能量均衡指标': df['aco_best_energy_metric'].mean(),
            '充电节点数': df['charged_node_count'].mean(),
            '总节点数': df['total_node_count'].mean(),
            '实际返回次数': df['num_returns'].mean(),
            '规划行程次数': df['num_trips_planned'].mean(),
            '规划时间(s)': df['planning_time_s'].mean(),
            '仿真时间(s)': df['simulation_time_s'].mean(),
            '优化时间(s)': df['aco_optimization_time_s'].mean(),
            '实际迭代次数': df['aco_iterations'].mean(),
            '充电能量(kJ)': df['charge_energy_j'].mean() / 1000,
            '移动能量(kJ)': df['move_energy_j'].mean() / 1000,
            '传输能量(kJ)': df['tx_energy_j'].mean() / 1000,
            '激活能量(kJ)': df['active_energy_j'].mean() / 1000,
            '总消耗能量(kJ)': (df['move_energy_j'] + df['tx_energy_j'] + df['active_energy_j']).mean() / 1000,
            '传感器充电数': df['sensors_charged'].mean(),
            '中继充电数': df['relays_charged'].mean(),
            '总充电节点数': df['total_charging_nodes'].mean()
        }
        
        for key, value in avg_stats.items():
            if '能量' in key or '时间' in key or '路径' in key:
                print(f"{key}: {value:.2f}")
            elif key == 'EUE' or key == '能量均衡指标':
                print(f"{key}: {value:.4f}")
            else:
                print(f"{key}: {value:.1f}")
        
        # 保存平均值到单独的CSV
        avg_df = pd.DataFrame([avg_stats])
        avg_path = os.path.join(RESULTS_DIR, f"improved_aco_limited_average_stats_{seed_start}_{seed_end}.csv")
        avg_df.to_csv(avg_path, index=False)
        print(f"\n平均值已保存到: {avg_path}")
        
        # 打印前5个种子的详细信息
        print("\n" + "="*70)
        print("前5个种子的详细结果")
        print("="*70)
        display_cols = ['seed', 'eue', 'total_path_length_m', 'charged_node_count', 
                        'total_node_count', 'num_trips_planned', 'num_returns',
                        'aco_iterations', 'aco_best_energy_metric']
        print(df[display_cols].head().to_string(index=False))
        
        # 计算统计信息（标准差、最小值、最大值）
        print("\n" + "="*70)
        print("统计信息（标准差、最小值、最大值）")
        print("="*70)
        
        stats_cols = ['eue', 'total_reward', 'total_path_length_m', 'charged_node_count', 
                      'aco_optimal_distance_m', 'aco_best_energy_metric', 
                      'num_trips_planned', 'aco_iterations', 'planning_time_s']
        
        for col in stats_cols:
            col_data = df[col]
            if col == 'eue' or col == 'aco_best_energy_metric':
                print(f"{col}:")
                print(f"  均值: {col_data.mean():.4f} ± {col_data.std():.4f}")
                print(f"  最小: {col_data.min():.4f}, 最大: {col_data.max():.4f}")
            else:
                print(f"{col}:")
                print(f"  均值: {col_data.mean():.2f} ± {col_data.std():.2f}")
                print(f"  最小: {col_data.min():.2f}, 最大: {col_data.max():.2f}")
        
        # 行程次数分布统计
        print("\n" + "="*70)
        print("行程次数分布统计")
        print("="*70)
        trip_counts = df['num_trips_planned'].value_counts().sort_index()
        for trips, count in trip_counts.items():
            print(f"  {trips} 趟行程: {count} 个种子 ({count/len(df)*100:.1f}%)")
        
        # 检查是否有种子未能完成所有充电
        incomplete_charge = df[df['charged_node_count'] < df['total_charging_nodes']]
        if len(incomplete_charge) > 0:
            print(f"\n⚠️ 警告: {len(incomplete_charge)} 个种子未能完成所有节点充电")
            print(incomplete_charge[['seed', 'charged_node_count', 'total_charging_nodes', 'num_trips_planned']].to_string(index=False))
    
    print("\n" + "="*70)
    print(f"所有结果已保存到: {RESULTS_DIR}")
    print("="*70)


def main():
    """主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description='改进蚁群算法批量测试 - 有限电池容量版本')
    parser.add_argument('--start', type=int, default=0, help='起始种子')
    parser.add_argument('--end', type=int, default=99, help='结束种子')  
    parser.add_argument('--num_seeds', type=int, default=None, help='测试种子数量（替代--end）')
    
    # 无人车参数
    parser.add_argument('--agent_battery', type=float, default=100000000.0, help='无人车电池容量 (J)')
    parser.add_argument('--move_consumption', type=float, default=1000.0, help='移动能耗 (J/m)')
    parser.add_argument('--tx_consumption', type=float, default=100.0, help='传输能耗 (J/s)')
    parser.add_argument('--min_charge_rate', type=float, default=0.7, help='最小充电效率')
    
    # 搜索能力参数
    parser.add_argument('--num_ants', type=int, default=60, help='蚂蚁数量')
    parser.add_argument('--max_iter', type=int, default=400, help='最大迭代次数')
    parser.add_argument('--exploration_rate', type=float, default=0.2, help='随机探索概率')
    parser.add_argument('--stagnation', type=int, default=100, help='停滞阈值')
    
    # 路径选择参数
    parser.add_argument('--alpha0', type=float, default=0.7, help='最终α值')
    parser.add_argument('--alpha1', type=float, default=1.8, help='初始α增量')
    parser.add_argument('--K', type=float, default=0.008, help='衰减速率常数')
    parser.add_argument('--beta', type=float, default=1.5, help='距离权重')
    parser.add_argument('--gamma', type=float, default=0.5, help='能量紧急度权重')
    
    # 信息素更新参数
    parser.add_argument('--rho', type=float, default=0.02, help='信息素挥发率')
    parser.add_argument('--Q', type=float, default=200.0, help='信息素强度')
    parser.add_argument('--tau_min', type=float, default=0.0005, help='最小信息素')
    parser.add_argument('--tau_max', type=float, default=15.0, help='最大信息素')
    parser.add_argument('--elite_ratio', type=float, default=0.25, help='精英蚂蚁比例')
    
    # 问题特定参数
    parser.add_argument('--distance_threshold', type=float, default=6000.0, help='距离阈值 d0 (m)')
    
    parser.add_argument('--verbose', action='store_true', help='打印详细优化信息')
    
    args = parser.parse_args()
    
    # 处理种子范围
    if args.num_seeds is not None:
        seed_end = args.start + args.num_seeds - 1
    else:
        seed_end = args.end
    
    batch_test(
        seed_start=args.start,
        seed_end=seed_end,
        agent_max_battery=args.agent_battery,
        agent_move_consumption=args.move_consumption,
        agent_tx_consumption=args.tx_consumption,
        sensor_max_battery=240000.0,
        gateway_max_battery=800000.0,
        min_charging_rate=args.min_charge_rate,
        num_ants=args.num_ants,
        max_iterations=args.max_iter,
        alpha0=args.alpha0,
        alpha1=args.alpha1,
        K=args.K,
        beta=args.beta,
        gamma=args.gamma,
        rho=args.rho,
        Q=args.Q,
        tau_min=args.tau_min,
        tau_max=args.tau_max,
        distance_threshold=args.distance_threshold,
        elite_ant_ratio=args.elite_ratio,
        exploration_rate=args.exploration_rate,
        stagnation_limit=args.stagnation,
        verbose=args.verbose
    )


if __name__ == "__main__":
    main()