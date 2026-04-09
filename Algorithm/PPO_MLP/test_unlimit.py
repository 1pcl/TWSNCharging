import numpy as np
import os
import sys
import time
import json
import torch
import pandas as pd
from typing import Dict, List, Any
from collections import defaultdict
import heapq

# --- 路径设置 ---
current_script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_script_dir)
if project_root not in sys.path:
    sys.path.insert(0, project_root)

try:
    from Scenario.uav_scheduling_unlimited import UAVSchedulingUnlimitedEnv
    from Scenario.WSNsimulation import WSNSimulation
    from attention_model_wrapper import CustomMaskableActorCriticPolicy
except ImportError as e:
    print(f"导入模块失败: {e}")
    sys.exit(1)

from sb3_contrib import MaskablePPO

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# 创建结果保存目录
RESULTS_DIR = os.path.join(current_script_dir, "batch_results_model")
os.makedirs(RESULTS_DIR, exist_ok=True)

# 模型路径
MODEL_PATH = "./models/unlimited_best_model.zip"
DATA_PATH = "./data/road_graph_with_nodes.json"


def run_single_test(seed: int, model, json_data, node_count: int, time_levels: int) -> Dict[str, Any]:
    """
    运行单个种子的测试
    
    Args:
        seed: 随机种子
        model: 训练好的模型
        json_data: 道路网络数据
        node_count: 节点数
        time_levels: 充电时间等级数
    
    Returns:
        包含所有指标的字典
    """
    start_time = time.time()
    
    try:
        # 创建 WSNSimulation 实例
        wsn_sim = WSNSimulation(
            data_input=json_data,
            sensor_battery=240000.0,
            relay_battery=800000.0,
            station_battery=1000000.0,
            seed=seed
        )
        
        # 获取 WSN 仿真数据
        wsn_data = wsn_sim.get_data()
        point_info = wsn_data['point_info']
        
        # 构建节点ID到类型的映射字典
        node_id_to_type = {}
        for node in point_info:
            node_id_to_type[node['node_id']] = node['type']
        
        # 统计需要充电的节点
        charging_nodes_info = [n for n in point_info if n['battery_info'].get('needs_charge', False)]
        sensors_count = len([n for n in charging_nodes_info if n['type'] == 'sensor'])
        relays_count = len([n for n in charging_nodes_info if n['type'] == 'relay'])
        
        # 创建环境
        env = UAVSchedulingUnlimitedEnv(
            graph_G=wsn_data["graph_G"],
            point_info=point_info,
            max_min=wsn_data['max_min'],
            seed=seed,
            is_train=False,
            json_data=json_data
        )
        
        # 设置模型参数
        model.policy.node_count = node_count
        model.policy.time_levels = time_levels
        
        # 构建节点映射（全局ID -> 原始ID）
        global_to_original = {}
        for gid, data in env.global_to_original_map.items():
            global_to_original[gid] = data['original_id']
        
        # 获取充电站ID
        station_original_id = None
        for gid, data in env.global_to_original_map.items():
            if data["original_type"] == "station":
                station_original_id = data["original_id"]
                break
        
        # 运行测试
        obs, _ = env.reset()
        done = False
        
        ep_reward = 0
        ep_steps = 0
        charging_nodes = []
        
        # 统计推理时间
        inference_time = 0.0
        env_step_time = 0.0
        
        while not done:
            # 模型推理
            infer_start = time.time()
            action_masks = env.action_masks()
            action, _ = model.predict(obs, action_masks=action_masks, deterministic=True)
            inference_time += time.time() - infer_start
            
            # 解码动作
            action_dict = env._decode_action(action)
            target_global_id = action_dict["target_node_id"]
            # action_charge_time = action_dict["charge_time"][0]
            # target_global_id = int(action[0])
            target_original_id = global_to_original[target_global_id]
            
            # 获取节点类型（使用映射字典）
            target_type = node_id_to_type.get(target_original_id, 'unknown')
            
            # 记录充电节点
            if target_type != 'station':
                charging_nodes.append(target_original_id)
            
            # 环境执行
            step_start = time.time()
            obs, reward, terminated, truncated, info = env.step(action)
            env_step_time += time.time() - step_start
            
            ep_reward += reward
            ep_steps += 1
            
            done = terminated or truncated
        
        # 获取最终信息
        final_info = env._get_info()
        
        # 计算 EUE
        total_consumed = env.tx_energy + env.move_energy + env.active_energy
        eue = env.charg_energy / total_consumed if total_consumed > 0 else 0
        
        elapsed_time = time.time() - start_time
        
        # 构建结果字典
        result = {
            'seed': seed,
            'planning_time_s': inference_time,  # 模型推理时间相当于规划时间
            'simulation_time_s': elapsed_time,
            'inference_time_s': inference_time,
            'env_step_time_s': env_step_time,
            'total_steps': ep_steps,
            'total_path_length_m': env.total_distance,
            'charged_node_count': final_info['node_charge_count'],
            'total_node_count': len([n for n in point_info if n['type'] != 'station']),
            'num_returns': 1,  # 模型最终会返回充电站
            'eue': eue,
            'total_reward': ep_reward,
            'charge_energy_j': env.charg_energy,
            'move_energy_j': env.move_energy,
            'tx_energy_j': env.tx_energy,
            'active_energy_j': env.active_energy,
            'sensors_charged': sensors_count,
            'relays_charged': relays_count,
            'total_charging_nodes': len(charging_nodes_info)
        }
        
        env.close()
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
            'inference_time_s': 0,
            'env_step_time_s': 0,
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
            'active_energy_j': 0,
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
    print("=" * 70)
    print(f"模型批量测试 (种子 {seed_start} 到 {seed_end})")
    print("=" * 70)
    
    # 加载道路网络数据
    print("\n[1] 加载道路网络数据...")
    try:
        with open(DATA_PATH, 'r') as f:
            json_data = json.load(f)
    except FileNotFoundError:
        print(f"错误: 找不到文件 {DATA_PATH}")
        return
    
    # 加载模型
    print(f"\n[2] 加载模型: {MODEL_PATH}...")
    try:
        # 创建临时环境获取动作空间维度
        temp_wsn = WSNSimulation(
            data_input=json_data,
            sensor_battery=240000.0,
            relay_battery=800000.0,
            station_battery=1000000.0,
            num_sensors=50,
            num_relays=5,
            seed=42
        )
        temp_data = temp_wsn.get_data()
        temp_env = UAVSchedulingUnlimitedEnv(
            graph_G=temp_data["graph_G"],
            point_info=temp_data["point_info"],
            max_min=temp_data['max_min'],
            seed=42,
            is_train=False,
            json_data=json_data
        )
        
        action_space_dims = temp_env.action_space.nvec
        node_count = int(action_space_dims[0])
        time_levels = int(action_space_dims[1])
        temp_env.close()
        
        # 加载模型
        model = MaskablePPO.load(MODEL_PATH, device=device)
        model.policy.node_count = node_count
        model.policy.time_levels = time_levels
        
        print(f"  节点数: {node_count}")
        print(f"  充电时间等级: {time_levels}")
        
    except Exception as e:
        print(f"错误: 模型加载失败 - {e}")
        return
    
    # 存储所有结果
    all_results = []
    failed_seeds = []
    
    total_seeds = seed_end - seed_start + 1
    print(f"\n[3] 开始批量测试 (共 {total_seeds} 个种子)...")
    
    batch_start_time = time.time()
    
    for i, seed in enumerate(range(seed_start, seed_end + 1)):
        print(f"\n进度: {i+1}/{total_seeds} (种子: {seed})", end="", flush=True)
        
        result = run_single_test(seed, model, json_data, node_count, time_levels)
        
        if 'error' in result:
            failed_seeds.append(seed)
            print(f" ❌ 失败")
        else:
            all_results.append(result)
            print(f" ✅ EUE: {result['eue']:.4f}, 距离: {result['total_path_length_m']/1000:.2f}km, "
                  f"充电: {result['charged_node_count']}/{result['total_charging_nodes']}, "
                  f"步数: {result['total_steps']}")
    
    batch_time = time.time() - batch_start_time
    
    # ==================== 结果统计 ====================
    print("\n" + "=" * 70)
    print("批量测试完成")
    print("=" * 70)
    print(f"总耗时: {batch_time:.2f}s ({batch_time/60:.2f}分钟)")
    print(f"成功测试数: {len(all_results)}/{total_seeds}")
    print(f"失败种子: {failed_seeds}" if failed_seeds else "无失败种子")
    
    if all_results:
        # 转换为 DataFrame
        df = pd.DataFrame(all_results)
        
        # 保存所有结果到 CSV
        csv_filename = f"model_batch_results_{seed_start}_{seed_end}.csv"
        csv_path = os.path.join(RESULTS_DIR, csv_filename)
        df.to_csv(csv_path, index=False)
        print(f"\n所有结果已保存到: {csv_path}")
        
        # 计算平均值并打印
        print("\n" + "=" * 70)
        print("平均值统计")
        print("=" * 70)
        
        avg_stats = {
            '总奖励': df['total_reward'].mean(),
            'EUE': df['eue'].mean(),
            '总路径长度(km)': df['total_path_length_m'].mean() / 1000,
            '充电节点数': df['charged_node_count'].mean(),
            '总节点数': df['total_node_count'].mean(),
            '返回次数': df['num_returns'].mean(),
            '总步数': df['total_steps'].mean(),
            '规划时间(s)': df['planning_time_s'].mean(),  # 推理时间
            '仿真时间(s)': df['simulation_time_s'].mean(),
            '推理时间(s)': df['inference_time_s'].mean(),
            '环境执行时间(s)': df['env_step_time_s'].mean(),
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
        
        # 计算标准差
        print("\n" + "=" * 70)
        print("标准差统计")
        print("=" * 70)
        
        std_stats = {
            '总奖励': df['total_reward'].std(),
            'EUE': df['eue'].std(),
            '总路径长度(km)': df['total_path_length_m'].std() / 1000,
            '充电节点数': df['charged_node_count'].std(),
            '总步数': df['total_steps'].std(),
            '规划时间(s)': df['planning_time_s'].std(),
            '充电能量(J)': df['charge_energy_j'].std(),
            '移动能量(J)': df['move_energy_j'].std(),
        }
        
        for key, value in std_stats.items():
            if '能量' in key or '时间' in key:
                print(f"{key}: {value:.2f}")
            elif key == 'EUE':
                print(f"{key}: {value:.4f}")
            else:
                print(f"{key}: {value:.1f}")
        
        # 保存平均值到单独的CSV
        avg_df = pd.DataFrame([avg_stats])
        avg_path = os.path.join(RESULTS_DIR, f"model_average_stats_{seed_start}_{seed_end}.csv")
        avg_df.to_csv(avg_path, index=False)
        print(f"\n平均值已保存到: {avg_path}")
        
        # 保存统计信息到CSV
        stats_df = pd.DataFrame([{
            'total_seeds': total_seeds,
            'successful_seeds': len(all_results),
            'failed_seeds': len(failed_seeds),
            'failed_seeds_list': str(failed_seeds) if failed_seeds else '',
            'batch_time_s': batch_time,
            'avg_reward': avg_stats['总奖励'],  
            'std_reward': std_stats['总奖励'],  
            'avg_eue': avg_stats['EUE'],
            'std_eue': std_stats['EUE'],
            'avg_path_km': avg_stats['总路径长度(km)'],
            'std_path_km': std_stats['总路径长度(km)'],
            'avg_charged': avg_stats['充电节点数'],
            'std_charged': std_stats['充电节点数'],
            'avg_steps': avg_stats['总步数'],
            'std_steps': std_stats['总步数']
        }])
        stats_path = os.path.join(RESULTS_DIR, f"model_batch_stats_{seed_start}_{seed_end}.csv")
        stats_df.to_csv(stats_path, index=False)
        print(f"批量统计已保存到: {stats_path}")
        
        # 打印前5个种子的详细信息
        print("\n" + "=" * 70)
        print("前5个种子的详细结果")
        print("=" * 70)
        print(df[['seed', 'total_reward', 'eue', 'total_path_length_m', 'charged_node_count', 'total_steps', 'planning_time_s']].head().to_string(index=False))
    
    print("\n" + "=" * 70)
    print(f"所有结果已保存到: {RESULTS_DIR}")
    print("=" * 70)


def main():
    """主函数"""
    # 测试种子 0-99
    batch_test(seed_start=0, seed_end=99)


if __name__ == "__main__":
    main()