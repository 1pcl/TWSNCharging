import json
import os
import sys
import numpy as np
import torch
import time

# --- 1. 路径与环境配置 ---
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

from sb3_contrib import MaskablePPO

# 检查计算设备
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

def make_test_env(seed, json_data, is_train=False):
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

def run_single_test(model_path, data_json, seed=42):
    """
    加载模型并在单个场景下运行，输出类似贪心算法的详细日志
    """
    # 加载JSON数据
    print(f"正在加载场景数据 (Seed: {seed})...")
    with open(data_json, "r") as f:
        json_data = json.load(f)
    
    # 创建环境（不使用包装器）
    env_func = make_test_env(seed, json_data, is_train=False)
    env = env_func()

    # 加载模型 - 使用自定义策略类
    if not os.path.exists(model_path):
        print(f"错误: 找不到模型文件 {model_path}")
        return
    
    print(f"正在加载 PPO 模型: {model_path}")
    
    # 需要从环境中获取动作空间维度，以便正确加载自定义策略
    action_space_dims = env.action_space.nvec
    node_count = int(action_space_dims[0])
    time_levels = int(action_space_dims[1])
    
    # 加载模型时指定自定义策略和参数
    model = MaskablePPO.load(
        model_path, 
        device=device,
        custom_objects={
            "policy_class": CustomMaskableActorCriticPolicy
        }
    )
    
    # 重置环境
    obs, info = env.reset()
    done = False
    
    # 指标初始化
    total_reward = 0.0
    total_path_length = 0.0

    charge_e = 0.0
    move_e = 0.0
    tx_e = 0.0
    active_e = 0.0
    
    print("\n| --- PPO 策略动作序列日志 (类似贪心算法格式) --- |")
    print("| Step | From Node  | To Node    | Time (s) | Dist (m) | Reward |")
    print("|------|------------|------------|----------|----------|--------|")

    start_time = time.time()
    
    while not done:
        from_id = env.uav_state["node_id"]
        from_key = env.global_to_original_map[from_id]["point_data"]["node_id"]
        
        # 使用模型预测动作 (使用动作掩码保证合法性)
        action_masks = env.action_masks()
        action, _ = model.predict(obs, action_masks=action_masks, deterministic=True)
        
        # 解码动作 - PPO模型输出的是二维数组[[target_id, charge_level]]
        # 首先展平数组
        action = np.array(action).flatten()
        
        # 解码目标节点名称与充电时间
        target_id = int(action[0])
        target_key = env.global_to_original_map[target_id]["point_data"]["node_id"]
        decoded = env._decode_action(action)
        actual_charge_time = decoded["charge_time"][0]
        
        # 执行动作
        obs, reward, terminated, truncated, info = env.step(action)
        
        # 累计数据
        total_reward += reward
        dist = info.get("distance_needed", 0)
        total_path_length += dist
        charge_e += info["step_charge_energy"]
        move_e += info["step_move_energy"]
        tx_e += info["step_tx_energy"]
        active_e += info["active_energy"]
            
        # 打印当前步日志
        charge_tag = "<--" if actual_charge_time > 0 else "---"
        print(f"| {env.current_step:<4} | {from_key:<10} | {target_key:<10} | {actual_charge_time:.2f}{charge_tag:<5} | {dist:.2f}{'':<5} | {reward:.2f} |")
        
        done = terminated or truncated

    end_time = time.time()
    denom = move_e + tx_e + active_e
    total_eue = charge_e / denom if denom > 0 else 0.0

    # --- 最终汇总打印 ---
    print("\n" + "="*60)
    print("           PPO 策略单次测试结果")
    print(f"| 总步数 (total_steps): {env.current_step:<30} |")
    print(f"| 累积奖励 (total_reward): {total_reward:.2f}{'':<25} |")
    print(f"| 平均能量利用效率 (avg_eue): {total_eue:.4f}{'':<25} |")
    print(f"| 总路径长度 (total_path): {total_path_length:.2f} m{'':<22} |")
    print(f"| 决策耗时: {end_time - start_time:.4f} s{'':<32} |")
    print("="*60 + "\n")
    
    # 关闭环境
    env.close()

if __name__ == "__main__":
    # 配置参数
    MODEL_FILE = "./models/ppo_sb_mask_mlp_best_model.zip"  # 你的 PPO 模型路径
    DATA_FILE = "./data/chargingScheduling_data.json"
    TEST_SEED = 42  # 设为与 greed.py 相同的种子 42 进行对比
    
    run_single_test(MODEL_FILE, DATA_FILE, TEST_SEED)