import argparse
import json
import math
import time
import numpy as np
import os
import sys
import torch
import csv
from datetime import datetime
from collections import deque
import gymnasium as gym 

from attention_model_wrapper import CustomMaskableActorCriticPolicy

# --- 路径配置 ---
current_script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_script_dir)
if project_root not in sys.path:
    sys.path.insert(0, project_root)

try:
    from Scenario.uav_scheduling_unlimited import UAVSchedulingUnlimitedEnv
    # from Scenario.uav_scheduling_limited import UAVSchedulingLimitedEnv
    from Scenario.WSNsimulation import WSNSimulation 
except ImportError as e:
    print(f"导入自定义模块失败: {e}")
    sys.exit(1)

from stable_baselines3.common.vec_env import DummyVecEnv
from sb3_contrib import MaskablePPO

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")


def make_env(seed, json_data,is_train):
    def _init():
        wsn_sim = WSNSimulation(json_data, seed=seed)
        data = wsn_sim.get_data()
        env = UAVSchedulingUnlimitedEnv(
            graph_G=data["graph_G"], 
            point_info=data["point_info"], 
            max_min= data['max_min'],
            seed=seed,
            is_train=is_train,
            json_data = json_data
        )
        return env
    return _init

def run_detailed_evaluation(model, json_data, n_episodes=20):
    """详细评估函数"""
    all_metrics = {"reward": [], "steps": [], "charged": [], "eue": [], "path": []}

    for i in range(n_episodes):
        eval_env = make_env(int(time.time()), json_data, is_train=False)()
        obs, _ = eval_env.reset()
        done = False
        ep_reward, ep_steps = 0, 0
        path_total = 0.0
        charge_e = move_e = tx_e = 0.0

        while not done:
            action_masks = eval_env.action_masks()
            action, _ = model.predict(obs, action_masks=action_masks, deterministic=True)
            obs, reward, terminated, truncated, info = eval_env.step(action)
            
            ep_reward += reward
            ep_steps += 1
            path_total += info.get("distance_needed", 0)
            charge_e += info.get('step_charge_energy', 0)
            move_e += info.get('step_move_energy', 0)
            tx_e += info.get('step_tx_energy', 0)
            done = terminated or truncated
        
        denom = move_e + tx_e
        eue = charge_e / denom if denom > 0 else 0.0

        all_metrics["reward"].append(ep_reward)
        all_metrics["steps"].append(ep_steps)
        all_metrics["charged"].append(info.get('node_charge_count', 0))
        all_metrics["eue"].append(eue)
        all_metrics["path"].append(path_total)
        eval_env.close()

    return {k: np.mean(v) for k, v in all_metrics.items()}

def run_training(args):
    model_dir, log_dir = "./models/", "./logs/"
    os.makedirs(model_dir, exist_ok=True); os.makedirs(log_dir, exist_ok=True)
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_log_path = os.path.join(log_dir, f"{args.model_name}_training_log_{timestamp}.csv")
    base_json_path = os.path.join(project_root, args.data_base_path)

    with open(base_json_path, "r") as f:
        json_data = json.load(f) 

    headers = ['Eval#', 'TotalStep', 'avg_reward', 'avg_steps', 'avg_charged_nodes', 'avg_eue', 'avg_path_length']
    with open(csv_log_path, 'w', newline='') as f:
        cvs_writer = csv.writer(f)
        cvs_writer.writerow(headers)

    # 创建单个环境实例来获取动作空间
    sample_env = make_env(0, json_data, is_train=True)()  # 创建单个环境

    # 从单个环境获取动作空间维度
    action_space_dims = sample_env.action_space.nvec
    node_count = int(action_space_dims[0])
    time_levels = int(action_space_dims[1])

    # 创建环境
    vec_env = DummyVecEnv([make_env(0, json_data,is_train=True) for i in range(args.num_envs)])
    
    # 创建模型
    model = MaskablePPO(
        CustomMaskableActorCriticPolicy,
        vec_env, 
        learning_rate=args.learning_rate,
        n_steps=args.n_steps, 
        batch_size=args.batch_size, 
        n_epochs=args.n_epochs,    
        ent_coef=args.ent_coef,    
        gamma=args.gamma,
        gae_lambda=args.gae_lambda,
        clip_range=args.clip_range, 
        verbose=0, 
        device=device,
        tensorboard_log="ppo_uav_tensorboard",            
        vf_coef=args.vf_coef,
        max_grad_norm=args.max_grad_norm,
        normalize_advantage=args.normalize_advantage,
        policy_kwargs=dict(
            node_count=node_count,
            time_levels=time_levels,
            node_dim=10,
            uav_dim=6, 
            embedding_dim=128,
        ),
    )

    # 打印配置信息
    steps_per_cycle = args.n_steps * args.num_envs
    print(f"\n训练配置:")
    print(f"  并行环境数: {args.num_envs}")
    print(f"  每周期步数: {steps_per_cycle:,} (n_steps={args.n_steps} × num_envs={args.num_envs})")
    print(f"  评估间隔: {args.eval_interval}个周期 ({steps_per_cycle * args.eval_interval:,}步)")
    print("="*125)
    
    header_str = f"{'Eval#':<8} | {'TotalStep':<10} | {'Reward':<10} | {'Steps':<8} | {'Charged':<8} | {'EUE':<8} | {'Path':<10}"
    print(header_str)
    print("-" * 125)

    # 训练状态
    current_steps = 0
    eval_count = 0  # 评估次数
    cycle_count = 0  # 训练周期数
    best_reward = -float('inf')
    
    # 主训练循环
    while current_steps < args.max_timesteps:
        model.learn(total_timesteps=steps_per_cycle, reset_num_timesteps=False)
        current_steps += steps_per_cycle
        cycle_count += 1
        
        # 控制评估频率
        if cycle_count % args.eval_interval == 0:
            eval_count += 1
            
            # 运行评估
            m = run_detailed_evaluation(model, json_data, n_episodes=20)

            model.logger.record("eval/avg_reward", m['reward'])
            model.logger.record("eval/avg_steps", m['steps'])
            model.logger.record("eval/avg_eue", m['eue'])
            model.logger.record("eval/avg_path_length", m['path'])
            model.logger.record("eval/avg_charged_nodes", m['charged'])

            model.logger.dump(step=current_steps)

            # 打印评估结果
            print(f"{eval_count:<8} | {current_steps:<10} | {m['reward']:<10.2f} | "
                  f"{m['steps']:<8.1f} | {m['charged']:<8.1f} | {m['eue']:<8.4f} | "
                  f"{m['path']:<10.2f} ")

            # 记录到CSV
            with open(csv_log_path, 'a', newline='') as f:
                cvs_writer = csv.writer(f)
                cvs_writer.writerow([
                    eval_count, current_steps, round(m['reward'], 2), 
                    round(m['steps'], 1), round(m['charged'], 1), 
                    round(m['eue'], 4), round(m['path'], 2)
                ])
            
            # 保存最佳模型
            if m['reward'] > best_reward:
                best_reward = m['reward']
                model.save(os.path.join(model_dir, f"{args.model_name}_best_model"))
                print(f"  [新最佳模型] 奖励: {best_reward:.2f}")

    vec_env.close()
    print(f"\n训练完成! 最终步数: {current_steps:,}, 最佳奖励: {best_reward:.2f}")

def main():
    parser = argparse.ArgumentParser()
    
    # 基础参数
    parser.add_argument("--model_name", type=str, default="unlimited")
    # parser.add_argument("--model_name", type=str, default="limited")
    parser.add_argument("--num_envs", type=int, default=8)
    parser.add_argument("--max_timesteps", type=int, default=5_000_000)
    parser.add_argument("--data_base_path", default="data/road_graph_with_nodes.json")
    
    # 评估参数
    parser.add_argument("--eval_interval", type=int, default=3,
        help="每多少个训练周期评估一次")
    
    parser.add_argument("--learning_rate", type=float, default=1e-4)
    parser.add_argument("--n_steps", type=int, default=1024)
    parser.add_argument("--batch_size", type=int, default=1024) 
    parser.add_argument("--n_epochs", type=int, default=8)
    parser.add_argument("--ent_coef", type=float, default=0.03)
    parser.add_argument("--clip_range", type=float, default=0.10)
    parser.add_argument("--gamma", type=float, default=0.99)
    parser.add_argument("--gae_lambda", type=float, default=0.96)

    
    args = parser.parse_args()

    args.vf_coef = 0.3  # 价值函数系数
    args.max_grad_norm = 0.6  # 梯度裁剪
    args.normalize_advantage = True  # 优势归一化

    run_training(args)

if __name__ == '__main__':
    main()
