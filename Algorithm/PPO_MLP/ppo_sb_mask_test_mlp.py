import json
import numpy as np
import argparse
import os
import sys
import torch
import csv
import time
import pandas as pd
from datetime import datetime
from typing import Dict, List, Any

# --- 路径设置 ---
current_script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_script_dir)
if project_root not in sys.path:
    sys.path.insert(0, project_root)

try:
    from attention_model_wrapper import CustomMaskableActorCriticPolicy
    from Scenario.UAVSchedulingEnv import UAVSchedulingEnv
    from Scenario.WSNsimulation import WSNSimulation 
except ImportError as e:
    print(f"导入自定义模块失败: {e}")
    sys.exit(1)

from sb3_contrib import MaskablePPO  # 必须使用这个类来加载模型

def make_test_env(seed, json_data, is_train=False):
    """创建测试环境（不使用包装器）"""
    def _init():
        wsn_sim = WSNSimulation(json_data, seed=seed)
        data = wsn_sim.get_data()
        env = UAVSchedulingEnv(
            graph_G=data["graph_G"], 
            point_info=data["point_info"],
            max_min=data['max_min'],
            seed=seed,
            is_train=is_train,
            json_data=json_data
        )
        return env
    return _init

def run_batch_test(model_path: str, start_seed: int, end_seed: int, json_file_path: str, output_dir: str = "results"):
    """
    批量测试函数，输出多个场景的统计指标
    """
    # 1. 创建输出目录
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_filename = os.path.join(output_dir, f"ppo_mlp_results_{timestamp}.csv")
    summary_filename = os.path.join(output_dir, f"ppo_mlp_summary_{timestamp}.txt")
    # detailed_log_filename = os.path.join(output_dir, f"ppo_detailed_log_{timestamp}.txt")

    # 加载JSON数据
    print(f"正在加载场景数据...")
    with open(json_file_path, "r") as f:
        json_data = json.load(f)

    # 2. 加载模型 - 使用自定义策略类
    print(f"正在加载模型: {model_path}")
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = MaskablePPO.load(
        model_path, 
        device=device,
        custom_objects={
            "policy_class": CustomMaskableActorCriticPolicy
        }
    )

    all_results: List[Dict[str, Any]] = []
    total_run_time = 0.0
    
    # # 打开详细日志文件
    # with open(detailed_log_filename, 'w', encoding='utf-8') as log_file:
    #     log_file.write("PPO模型批量测试详细日志\n")
    #     log_file.write("="*80 + "\n")
    #     log_file.write(f"测试时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
    #     log_file.write(f"模型文件: {model_path}\n")
    #     log_file.write(f"测试种子范围: {start_seed}-{end_seed}\n")
    #     log_file.write("="*80 + "\n\n")

    header = f"\n| Seed | Total Steps | Total Reward  | Charged Nodes | Avg EUE | Path Length (m) | Run Time (s) |"
    divider = "|------|-------------|---------------|---------------|---------|-----------------|--------------|"
    print(f"\n批量运行 PPO 模型测试 (种子: {start_seed}-{end_seed})")
    print(header)
    print(divider)

    for seed in range(start_seed, end_seed + 1):
        test_seed = seed  # 可以直接使用seed，不需要加时间戳保持一致性
        
        # 创建环境（不使用包装器）
        env_func = make_test_env(test_seed, json_data, is_train=False)
        env = env_func()
        
        obs, _ = env.reset()
        done = False
        
        # 记录单次实验指标
        ep_reward = 0.0
        ep_steps = 0
        ep_path = 0.0

        charge_e = 0.0
        move_e = 0.0
        tx_e = 0.0
        active_e = 0.0
        
        # 详细动作序列
        action_sequence = []
        
        start_time = time.time()
        
        while not done:
            from_id = env.uav_state["node_id"]
            from_key = env.global_to_original_map[from_id]["point_data"]["node_id"]
            
            # 获取动作掩码并预测动作
            action_masks = env.action_masks()
            action, _ = model.predict(obs, action_masks=action_masks, deterministic=True)
            
            # 解码动作
            action = np.array(action).flatten()
            target_id = int(action[0])
            target_key = env.global_to_original_map[target_id]["point_data"]["node_id"]
            decoded = env._decode_action(action)
            actual_charge_time = decoded["charge_time"][0]
            
            # 执行动作
            obs, reward, terminated, truncated, info = env.step(action)
            
            # 累计指标
            ep_reward += reward
            ep_steps += 1
            ep_path += info.get("distance_needed", 0)
            charge_e += info["step_charge_energy"]
            move_e += info["step_move_energy"]
            tx_e += info["step_tx_energy"]
            active_e += info["active_energy"]
            
            # 记录动作序列
            action_sequence.append({
                "step": ep_steps,
                "from_node": from_key,
                "to_node": target_key,
                "charge_time": actual_charge_time,
                "distance": info.get("distance_needed", 0),
                "reward": reward
            })
            
            done = terminated or truncated
        
        run_time = time.time() - start_time
        total_run_time += run_time
        denom = move_e + tx_e + active_e
        total_eue = charge_e / denom if denom > 0 else 0.0
        
        # 整理单次结果
        res = {
            'seed': seed,
            'total_steps': ep_steps,
            'total_reward': ep_reward,
            'charged_node_count': info.get('node_charge_count', 0),
            'total_eue': total_eue,
            'total_path_length': ep_path,
            'run_time': run_time,
            'charge_energy': charge_e,
            'move_energy': move_e,
            'tx_energy': tx_e,
            'active_energy': active_e
        }
        all_results.append(res)
        
        # 打印结果
        print(f"| {seed:<4} | {res['total_steps']:<11} | {res['total_reward']:<13.2f} | {res['charged_node_count']:<13} | {res['total_eue']:<7.4f} | {res['total_path_length']:<15.2f} | {run_time:<12.4f} |")

        # # 写入详细日志
        # with open(detailed_log_filename, 'a', encoding='utf-8') as log_file:
        #     log_file.write(f"\nSeed {seed} 详细测试结果:\n")
        #     log_file.write("-"*60 + "\n")
        #     log_file.write("动作序列:\n")
        #     log_file.write("| Step | From Node  | To Node    | Time (s) | Dist (m) | Reward |\n")
        #     log_file.write("|------|------------|------------|----------|----------|--------|\n")
        #     for action in action_sequence:
        #         charge_tag = "<--" if action["charge_time"] > 0 else "---"
        #         log_file.write(f"| {action['step']:<4} | {action['from_node']:<10} | {action['to_node']:<10} | "
        #                      f"{action['charge_time']:.2f}{charge_tag:<5} | {action['distance']:.2f}{'':<5} | {action['reward']:.2f} |\n")
        #     log_file.write(f"\n汇总: 步数={ep_steps}, 奖励={ep_reward:.2f}, 充电节点={info.get('node_charge_count', 0)}, EUE={total_eue:.4f}, 路径长度={ep_path:.2f}m\n\n")
        
        env.close()

    # 3. 保存与总结
    df = pd.DataFrame(all_results)
    df.to_csv(csv_filename, index=False)
    
    avg_metrics = df.mean(numeric_only=True).to_dict()
    summary_content = [
        "="*60,
        f"PPO 模型性能分析摘要 ({timestamp})",
        "="*60,
        f"模型文件: {model_path}",
        f"测试种子范围: {start_seed} - {end_seed}",
        f"测试次数: {len(all_results)}",
        "-"*60,
        f"平均总步数: {avg_metrics.get('total_steps', 0):.2f}",
        f"平均累积奖励: {avg_metrics.get('total_reward', 0):.2f}",
        f"平均充电节点数: {avg_metrics.get('charged_node_count', 0):.2f}",
        f"平均能量利用率 (EUE): {avg_metrics.get('total_eue', 0):.4f}",
        f"平均总路径长度: {avg_metrics.get('total_path_length', 0):.2f} m",
        f"平均充电能量: {avg_metrics.get('charge_energy', 0):.2f} J",
        f"平均移动能量: {avg_metrics.get('move_energy', 0):.2f} J",
        f"平均传输能量: {avg_metrics.get('tx_energy', 0):.2f} J",
        f"平均活跃能量: {avg_metrics.get('active_energy', 0):.2f} J",
        f"平均单次仿真耗时: {total_run_time/len(all_results):.4f} s",
        "="*60,
        # f"详细日志已保存至: {detailed_log_filename}",
        f"CSV结果已保存至: {csv_filename}",
        f"摘要文件已保存至: {summary_filename}"
    ]

    with open(summary_filename, 'w', encoding='utf-8') as f:
        f.write("\n".join(summary_content))
    
    print("\n" + "="*60)
    print("批量测试完成！")
    for line in summary_content[2:-3]:  # 跳过开头和结尾的文件路径信息
        print(line)
    
    return df

if __name__ == "__main__":
    # 解析命令行参数
    parser = argparse.ArgumentParser(description="PPO模型批量测试")
    parser.add_argument("--model_path", type=str, default="./models/ppo_sb_mask_mlp_best_model.zip",
                       help="模型文件路径")
    parser.add_argument("--start_seed", type=int, default=0,
                       help="起始种子")
    parser.add_argument("--end_seed", type=int, default=100,
                       help="结束种子")
    parser.add_argument("--data_file", type=str, default="./data/chargingScheduling_data.json",
                       help="数据文件路径")
    parser.add_argument("--output_dir", type=str, default="results",
                       help="输出目录")
    
    args = parser.parse_args()
    
    # 检查模型文件是否存在
    if not os.path.exists(args.model_path):
        print(f"错误: 找不到模型文件 {args.model_path}")
        print("请检查模型路径或使用 --model_path 参数指定正确的模型文件")
        sys.exit(1)
    
    # 运行批量测试
    run_batch_test(
        model_path=args.model_path,
        start_seed=args.start_seed,
        end_seed=args.end_seed,
        json_file_path=args.data_file,
        output_dir=args.output_dir
    )