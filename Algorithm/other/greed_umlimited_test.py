# batch_test_greedy.py
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
    from Scenario.uav_scheduling_unlimited import UAVSchedulingUnlimitedEnv
    from Scenario.WSNsimulation import WSNSimulation
    from greed_unlimited import RoadNetworkWithChargingNodes, GreedyPathPlanner, UAVSimulator
except ImportError as e:
    print(f"导入模块失败: {e}")
    sys.exit(1)

# 创建结果保存目录
RESULTS_DIR = os.path.join(current_script_dir, "batch_results")
os.makedirs(RESULTS_DIR, exist_ok=True)


def run_single_test(seed: int, network: RoadNetworkWithChargingNodes) -> Dict[str, Any]:
    """
    运行单个种子的测试
    
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
            seed=seed
        )
        
        # 获取 WSN 仿真数据
        wsn_data = wsn_sim.get_data()
        point_info = wsn_data['point_info']
        
        # 统计需要充电的节点
        charging_nodes_info = [n for n in point_info if n['battery_info'].get('needs_charge', False)]
        sensors_count = len([n for n in charging_nodes_info if n['type'] == 'sensor'])
        relays_count = len([n for n in charging_nodes_info if n['type'] == 'relay'])
        
        # 贪心路径规划
        planner_start = time.time()
        planner = GreedyPathPlanner(network, point_info)
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
            'num_returns': results['num_trips'],
            'eue': results['eue'],
            'total_reward': results['total_reward'],
            'charge_energy_j': results['charge_energy'],
            'move_energy_j': results['move_energy'],
            'tx_energy_j': results['tx_energy'],
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
            'num_returns': 0,
            'eue': 0,
            'total_reward': 0,
            'charge_energy_j': 0,
            'move_energy_j': 0,
            'tx_energy_j': 0,
            'sensors_charged': 0,
            'relays_charged': 0,
            'total_charging_nodes': 0
        }


def batch_test(seed_start: int = 0, seed_end: int = 99):
    """
    批量测试多个种子
    
    Args:
        seed_start: 起始种子
        seed_end: 结束种子（包含）
    """
    print("="*70)
    print(f"贪心算法批量测试 (种子 {seed_start} 到 {seed_end})")
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
            print(f" ✅ EUE: {result['eue']:.4f}, 距离: {result['total_path_length_m']/1000:.2f}km,reward: {result['total_reward']:.2f},"
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
        csv_filename = f"batch_results_{seed_start}_{seed_end}.csv"
        csv_path = os.path.join(RESULTS_DIR, csv_filename)
        df.to_csv(csv_path, index=False)
        print(f"\n所有结果已保存到: {csv_path}")
        
        # ==================== 平均值 ± 标准差 统计 ====================
        # 统一指标定义: (显示名称, DataFrame列名, 缩放系数, 格式化)
        # 【共有指标】（与模型、ACO、GA 脚本完全一致）
        # 贪心算法无特有指标
        metrics_def = [
            ('总奖励',            'total_reward',           1.0,   '.2f'),
            ('EUE',               'eue',                    1.0,   '.4f'),
            ('总路径长度(km)',     'total_path_length_m',    1e-3,  '.2f'),
            ('充电节点数',         'charged_node_count',     1.0,   '.1f'),
            ('总节点数',           'total_node_count',       1.0,   '.1f'),
            ('返回次数',           'num_returns',            1.0,   '.1f'),
            ('总步数',             'total_steps',            1.0,   '.1f'),
            ('规划时间(s)',        'planning_time_s',        1.0,   '.2f'),
            ('仿真时间(s)',        'simulation_time_s',      1.0,   '.2f'),
            ('充电能量(J)',        'charge_energy_j',        1.0,   '.2f'),
            ('移动能量(J)',        'move_energy_j',          1.0,   '.2f'),
            ('传输能量(J)',        'tx_energy_j',            1.0,   '.2f'),
            ('传感器充电数',       'sensors_charged',        1.0,   '.1f'),
            ('中继充电数',         'relays_charged',         1.0,   '.1f'),
            ('总充电节点数',       'total_charging_nodes',   1.0,   '.1f'),
        ]
        
        # 计算均值和标准差
        mean_dict = {}
        std_dict = {}
        for display_name, col, scale, fmt in metrics_def:
            mean_dict[display_name] = df[col].mean() * scale
            std_dict[display_name] = df[col].std() * scale
        
        # ---- 打印 平均值 ± 标准差 ----
        print("\n" + "="*70)
        print("贪心算法 平均值 ± 标准差 统计")
        print("="*70)
        for display_name, col, scale, fmt in metrics_def:
            m = mean_dict[display_name]
            s = std_dict[display_name]
            print(f"{display_name}: {m:{fmt}} ± {s:{fmt}}")
        
        # ---- 保存 平均值±标准差 到CSV ----
        formatted_rows = []
        for display_name, col, scale, fmt in metrics_def:
            m = mean_dict[display_name]
            s = std_dict[display_name]
            formatted_rows.append({
                '指标': display_name,
                '平均值': m,
                '标准差': s,
                '平均值±标准差': f"{m:{fmt}} ± {s:{fmt}}"
            })
        avg_df = pd.DataFrame(formatted_rows)
        avg_path = os.path.join(
            RESULTS_DIR,
            f"average_stats_{seed_start}_{seed_end}.csv"
        )
        avg_df.to_csv(avg_path, index=False, encoding='utf-8-sig')
        print(f"\n平均值±标准差 已保存到: {avg_path}")
        
        # ---- 前5个种子详细信息 ----
        print("\n" + "="*70)
        print("前5个种子的详细结果")
        print("="*70)
        print(df[['seed', 'eue', 'total_path_length_m', 'charged_node_count',
                  'total_node_count', 'num_returns']].head().to_string(index=False))
        
        # ---- 综合批量统计CSV（含 ± 格式） ----
        stats_df = pd.DataFrame([{
            'total_seeds':       total_seeds,
            'successful_seeds':  len(all_results),
            'failed_seeds':      len(failed_seeds),
            'failed_seeds_list': str(failed_seeds) if failed_seeds else '',
            'batch_time_s':      batch_time,
            'reward(mean±std)':  f"{mean_dict['总奖励']:.2f} ± {std_dict['总奖励']:.2f}",
            'eue(mean±std)':     f"{mean_dict['EUE']:.4f} ± {std_dict['EUE']:.4f}",
            'path_km(mean±std)': f"{mean_dict['总路径长度(km)']:.2f} ± {std_dict['总路径长度(km)']:.2f}",
            'charged(mean±std)': f"{mean_dict['充电节点数']:.1f} ± {std_dict['充电节点数']:.1f}",
            'returns(mean±std)': f"{mean_dict['返回次数']:.1f} ± {std_dict['返回次数']:.1f}",
            'planning_time(mean±std)': f"{mean_dict['规划时间(s)']:.2f} ± {std_dict['规划时间(s)']:.2f}",
            # 保留原始数值列，方便后续分析
            'avg_reward':        mean_dict['总奖励'],
            'std_reward':        std_dict['总奖励'],
            'avg_eue':           mean_dict['EUE'],
            'std_eue':           std_dict['EUE'],
            'avg_path_km':       mean_dict['总路径长度(km)'],
            'std_path_km':       std_dict['总路径长度(km)'],
            'avg_charged':       mean_dict['充电节点数'],
            'std_charged':       std_dict['充电节点数'],
            'avg_returns':       mean_dict['返回次数'],
            'std_returns':       std_dict['返回次数'],
        }])
        stats_path = os.path.join(
            RESULTS_DIR,
            f"batch_stats_{seed_start}_{seed_end}.csv"
        )
        stats_df.to_csv(stats_path, index=False, encoding='utf-8-sig')
        print(f"批量统计已保存到: {stats_path}")
    
    print("\n" + "="*70)
    print(f"所有结果已保存到: {RESULTS_DIR}")
    print("="*70)


def main():
    """主函数"""
    # 测试种子 0-99
    batch_test(seed_start=0, seed_end=99)


if __name__ == "__main__":
    main()