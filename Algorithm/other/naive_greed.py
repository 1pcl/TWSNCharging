import numpy as np
import os
import sys
import time
import logging
from typing import Dict, Any

# --- 路径设置 ---
current_script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_script_dir)
if project_root not in sys.path:
    sys.path.insert(0, project_root)

try:
    from Scenario.UAVSchedulingEnv import UAVSchedulingEnv
    from Scenario.WSNsimulation import WSNSimulation 
except ImportError as e:
    logging.error(f"导入自定义模块失败: {e}")
    sys.exit(1)

class NaiveHeuristicGreedyAgent:
    """
    朴素启发式贪心策略
    不进行环境模拟，直接根据当前观测状态决策。
    """
    def __init__(self, env: UAVSchedulingEnv):
        self.env = env
        self.station_idx = env.start_node_id
        # 在初始化时就决定访问顺序
        self.visit_sequence = self._generate_static_sequence()
        self.current_target_idx = 0  # 记录当前该去序列里的第几个点

    def _generate_static_sequence(self) -> list:
        """
        贪心路径生成
        """
        unvisited = list(range(self.env.num_nodes))
        unvisited.remove(self.station_idx)
        
        sequence = []
        current_ref_id = self.station_idx
        
        
        #只考虑路径长度，然后只给它充满电
        while unvisited:
            best_node = -1
            # max_score = -float('inf')
            min_dist = float('inf')
            
            for node_id in unvisited:

                
                # 1. 距离分数 (越近分越高) - 对应 f4_path_length
                dist = self.env.node_distance[current_ref_id, node_id]

                
                if dist < min_dist:
                    min_dist = dist
                    best_node = node_id
            
            # 记录并更新
            sequence.append(best_node)
            unvisited.remove(best_node)
            current_ref_id = best_node
            
        return sequence

    def get_action(self) -> np.ndarray:
        curr_id = self.env.uav_state["node_id"]
        curr_batt = self.env.uav_state["remaining_battery"]
        
        # 如果所有点都跑完了，或者没点可去了，回港
        if self.current_target_idx >= len(self.visit_sequence):
            return np.array([self.station_idx, 0])

        target_node_id = self.visit_sequence[self.current_target_idx]
        
        # 检查电量是否足够：去目标点 + 回充电站
        dist_to_node = self.env.node_distance[curr_id, target_node_id]
        dist_back_to_station = self.env.node_distance[target_node_id, self.station_idx]
        required_batt = (dist_to_node + dist_back_to_station) * self.env.agent_move_consumption
        
        if curr_batt < required_batt:
            # 电量不够，强行回港充电
            return np.array([self.station_idx, 0])
        
        # 电量够，去目标点充电
        # 朴素逻辑：直接执行最大充电等级（充满）
        # max_charge_level = self.env.charge_time_levels - 1
        max_barrey = self.env.sensor_max_battery
        if self.env.global_to_original_map[target_node_id]["original_type"] == "sensor":
            max_barrey = self.env.sensor_max_battery
        elif self.env.global_to_original_map[target_node_id]["original_type"] == "gateWay":
            max_barrey = self.env.gateway_max_battery
        node_state = self.env.dynamic_node_states[target_node_id]  
        charge_time = (max_barrey - node_state["current_energy"]) / (self.env.agent_tx_consumption * node_state["charging_rate"] )
        charge_level = charge_time/self.env.charge_time_levels
        
        # 准备前往该点，将索引移向序列中的下一个
        self.current_target_idx += 1
        
        return np.array([target_node_id, charge_level])

def run_greedy_simulation(env: UAVSchedulingEnv, agent: NaiveHeuristicGreedyAgent, print_log: bool = True):
    """
    运行仿真，指标收集逻辑与原 greed.py 保持完全一致。
    """
    total_reward = 0.0
    total_path_length = 0.0
    total_charged_steps = 0

    charge_e = 0.0
    move_e = 0.0
    tx_e = 0.0
    active_e = 0.0
    

    # 动作序列日志
    action_log = []
    
    obs, info = env.reset()
    terminated = False
    truncated = False
    
    if print_log:
        print("\n| Step | To Node    | Time (s) | Dist (m) | Reward | Remaining Batt |")
        print("|------|------------|----------|----------|--------|----------------|")

    while not terminated and not truncated:
        from_id = env.uav_state["node_id"]
        from_key = env.global_to_original_map[from_id]["point_data"]["node_id"]

        action = agent.get_action()
        target_id = action[0]
        
        # 获取动作前的信息用于日志
        decoded = env._decode_action(action)
        actual_charge_time = decoded["charge_time"][0]
        target_key = env.global_to_original_map[target_id]["point_data"]["node_id"]
        env.action_masks()

        # 执行动作
        obs, reward, terminated, truncated, info = env.step(action)
        
        # 累积指标
        total_reward += reward
        total_path_length += info["distance_needed"]
        charge_e += info["step_charge_energy"]
        move_e += info["step_move_energy"]
        tx_e += info["step_tx_energy"]
        active_e += info["active_energy"]

        log_entry = {
            "step": env.current_step,
            "uav_from": from_key,
            "uav_to": target_key,
            "charge_time": actual_charge_time,
            "distance": info['distance_needed'],
            "move_time":info['distance_needed']/env.agent_speed,
            "reward": reward,
        }
        
        if target_id != env.start_node_id:
            total_charged_steps += 1

        action_log.append(log_entry)

        # 打印动作序列
        if print_log:
            print(f"| {log_entry['step']:<4} | {log_entry['uav_from']:<10} | {log_entry['uav_to']:<10} | {log_entry['charge_time']:.2f}{'<--' if log_entry['charge_time'] > 0 else '---':<5} | {log_entry['distance']:.2f}{'':<5} | {log_entry['reward']:.2f} |")
        

    # 汇总
    # avg_eue = total_eue / total_charged_steps if total_charged_steps > 0 else 0.0
    denom = move_e + tx_e + active_e
    avg_eue = charge_e / denom if denom > 0 else 0.0
    
    return {
        "total_steps": env.current_step,
        "total_reward": total_reward,
        "charged_node_count": info["node_charge_count"],
        "total_node_count": env.num_nodes - 1,
        "total_eue": avg_eue,
        "total_path_length": total_path_length,
        "action_log": action_log
    }

if __name__ == "__main__":
    # --- 环境初始化 (逻辑同 greed.py) ---
    try:
        wsnSimulation = WSNSimulation(
            data_input="./data/chargingScheduling_data.json", 
            seed=42
        )
        data = wsnSimulation.get_data()
        env = UAVSchedulingEnv(graph_G=data["graph_G"], point_info=data["point_info"], max_min= data['max_min'])
    except Exception as e:
        print(f"初始化失败: {e}")
        exit()

    agent = NaiveHeuristicGreedyAgent(env)
    
    start_time = time.time()
    results = run_greedy_simulation(env, agent, print_log=True)
    end_time = time.time()

    # --- 5. 打印最终指标 ---
    print("\n" + "="*60)
    print("           朴素启发式贪心策略仿真结果")
    print(f"| 总节点数 (可充电): {results['total_node_count']:<42} |")
    print(f"| 总步数 (total_steps): {results['total_steps']:<43} |")
    print(f"| 累积奖励 (total_reward): {results['total_reward']:.2f}{'<--' if results['total_reward'] > 0 else '---':<18} |")
    print(f"| 充电节点数 (charged_node_count): {results['charged_node_count']:<40} |")
    print(f"| 平均能量利用效率 (total_eue): {results['total_eue']:.4f} |")
    print(f"| 总路径长度 (total_path_length): {results['total_path_length']:.2f}{' m':<32} |")
    print(f"| 仿真耗时: {end_time - start_time:.4f} {'s':<37} |")