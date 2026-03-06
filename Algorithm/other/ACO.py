import numpy as np
import random
import time
import os
import sys
from copy import deepcopy

# --- 环境导入逻辑 ---
current_script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_script_dir)
if project_root not in sys.path:
    sys.path.insert(0, project_root)

try:
    from Scenario.UAVSchedulingEnv import UAVSchedulingEnv
    from Scenario.WSNsimulation import WSNSimulation 
except ImportError as e:
    print(f"导入自定义模块失败: {e}")
    sys.exit(1)

class ACOSolver:
    def __init__(self, env, n_ants=50, n_iterations=500, alpha=1.0, beta=2.0, rho=0.1, Q=100, 
                 alpha_charge=1.0, rho_charge=0.1, Q_charge=50, early_stop_patience=100):
        self.env = env
        self.all_nodes = list(env.node_global_ids)
        self.station_idx = env.start_node_id
        self.sensor_nodes = [n for n in self.all_nodes if n != self.station_idx]
        
        # ACO 参数
        self.n_ants = n_ants
        self.n_iterations = n_iterations
        self.alpha = alpha  # 信息素重要程度
        self.beta = beta    # 启发式因子重要程度（使用2.0，参考文件）
        self.rho = rho      # 挥发率
        self.Q = Q          # 信息素增强系数
        
        # 新增：充电时间学习参数
        self.alpha_charge = alpha_charge  # 充电信息素重要程度
        self.rho_charge = rho_charge      # 充电信息素挥发率
        self.Q_charge = Q_charge          # 充电信息素增强系数
        
        self.early_stop_patience = early_stop_patience
        self.no_improve_count = 0
        
        # 初始化信息素矩阵
        self.pheromone = np.ones((env.num_nodes, env.num_nodes))
        
        # 新增：充电时间信息素矩阵
        self.charge_pheromone = np.ones((env.num_nodes, env.charge_time_levels))
        
        # 充电时长初始化
        self.best_durations = self._initialize_durations()
        
        self.best_path = None
        self.best_reward = -np.inf

    def _initialize_durations(self):
        """初始化充电时长"""
        durations = {}
        for node_id in self.sensor_nodes:
            info = self.env.global_to_original_map[node_id]
            max_b = self.env.sensor_max_battery if info["original_type"] == "sensor" else self.env.gateway_max_battery
            state = self.env.dynamic_node_states[node_id]
            needed = max_b - state["current_energy"]
            pwr = self.env.agent_tx_consumption * state["charging_rate"]
            
            # 计算需要的充电时间比例
            if pwr > 0:
                charge_time_needed = needed / pwr
                # 映射到充电等级
                ratio = min(1.0, charge_time_needed / (self.env.charge_time_levels * 10))
                level = int(ratio * (self.env.charge_time_levels - 1))
            else:
                level = self.env.charge_time_levels // 2
            
            durations[node_id] = max(0, min(self.env.charge_time_levels - 1, level))
        
        return durations
    
    def _select_charge_level(self, node_id):
        """基于信息素选择充电等级"""
        if node_id == self.station_idx:
            return 0  # 基站不充电
        
        probs = self.charge_pheromone[node_id] ** self.alpha_charge
        total = np.sum(probs)
        
        if total == 0:
            return random.randint(0, self.env.charge_time_levels - 1)
        
        probs = probs / total
        return np.random.choice(self.env.charge_time_levels, p=probs)
    
    def _generate_charge_durations(self):
        """为蚂蚁生成充电时间配置"""
        durations = {}
        for node_id in self.sensor_nodes:
            level = self._select_charge_level(node_id)
            durations[node_id] = level
        
        # 添加扰动
        for node_id in durations:
            if random.random() < 0.3:
                change = np.random.randint(-1, 2)
                durations[node_id] = np.clip(durations[node_id] + change, 0, self.env.charge_time_levels - 1)
        
        return durations

    def _get_heuristic(self, curr_node, next_node, uav_current_energy):
        """启发式因子（参考文件：简化的启发式）"""
        dist = self.env.node_distance[curr_node, next_node]
        if dist == 0: 
            return 0.0001  # 避免除零
        
        # 目标节点紧迫性
        node_state = self.env.dynamic_node_states[next_node]
        info = self.env.global_to_original_map[next_node]
        max_batt = self.env.sensor_max_battery if info["original_type"] == "sensor" else self.env.gateway_max_battery
        energy_ratio = (node_state["current_energy"] / max_batt) + 0.05  # 防止除零
        
        # 软约束：如果去这个点再回站电量危险，大幅降低启发值
        dist_to_station = self.env.node_distance[next_node, self.station_idx]
        estimated_cost = (dist + dist_to_station) * self.env.agent_move_consumption
        
        if uav_current_energy < estimated_cost * 1.2:
            return 0.0001  # 危险路径，大幅降低启发值
            
        # 参考文件：简化的启发式 (100.0 / dist)**2 * (1.0 / energy_ratio)
        # 使用 (1/dist)^2 * (1/energy_ratio) 的思想
        heuristic_value = (1.0 / dist) ** 2 * (1.0 / energy_ratio)
        
        return max(0.0001, heuristic_value)

    def _construct_ant_path(self, ant_id):
        """构建蚂蚁路径"""
        path = []
        visited_nodes = set()
        curr_node = self.station_idx
        visited_nodes.add(curr_node)
        
        remaining_nodes = set(self.sensor_nodes)
        uav_energy = self.env.agent_max_battery
        
        while remaining_nodes:
            candidates = list(remaining_nodes)
            probs = []
            feasible_candidates = []
            
            for next_node in candidates:
                # 检查能量约束（能否去节点并返回基站）
                dist_to_node = self.env.node_distance[curr_node, next_node]
                dist_back = self.env.node_distance[next_node, self.station_idx]
                required_energy = (dist_to_node + dist_back) * self.env.agent_move_consumption
                
                if uav_energy >= required_energy:
                    # 计算转移概率
                    tau = self.pheromone[curr_node, next_node] ** self.alpha
                    eta = self._get_heuristic(curr_node, next_node, uav_energy) ** self.beta
                    prob = tau * eta
                    
                    if prob > 0:
                        probs.append(prob)
                        feasible_candidates.append(next_node)
            
            # 如果没有可行节点，尝试放宽约束
            if not feasible_candidates:
                for next_node in candidates:
                    dist_to_node = self.env.node_distance[curr_node, next_node]
                    if uav_energy >= dist_to_node * self.env.agent_move_consumption:
                        tau = self.pheromone[curr_node, next_node] ** self.alpha
                        eta = self._get_heuristic(curr_node, next_node, uav_energy) ** self.beta
                        prob = tau * eta
                        
                        if prob > 0:
                            probs.append(prob)
                            feasible_candidates.append(next_node)
            
            if not feasible_candidates:
                break
            
            total_prob = sum(probs)
            if total_prob == 0:
                next_node = random.choice(feasible_candidates)
            else:
                normalized_probs = [p / total_prob for p in probs]
                next_node = np.random.choice(feasible_candidates, p=normalized_probs)
            
            path.append(next_node)
            visited_nodes.add(next_node)
            remaining_nodes.remove(next_node)
            
            dist_traveled = self.env.node_distance[curr_node, next_node]
            uav_energy -= dist_traveled * self.env.agent_move_consumption
            curr_node = next_node
        
        # 检查是否需要返回基站
        if path and path[-1] != self.station_idx:
            dist_to_station = self.env.node_distance[curr_node, self.station_idx]
            required_energy = dist_to_station * self.env.agent_move_consumption
            
            if uav_energy >= required_energy:
                path.append(self.station_idx)
            else:
                return path, False
        
        return path, True

    def _run_path_simulation(self, path, durations, render=False):
        """评估路径"""
        temp_env = deepcopy(self.env)
        temp_env.reset(seed=42)
        
        total_reward = 0
        actual_steps = []
        
        for node_id in path:
            if not temp_env.action_masks()[node_id]:
                return -100, []  # 给予惩罚
            
            if node_id == self.station_idx:
                level = 0
            else:
                level = int(np.clip(round(durations.get(node_id, self.env.charge_time_levels // 2)), 
                                    0, self.env.charge_time_levels - 1))
            
            obs, reward, terminated, truncated, info = temp_env.step((node_id, level))
            
            total_reward += reward
            if render:
                actual_steps.append((node_id, level, reward, info))
            
            if terminated or truncated:
                break
        
        return total_reward, actual_steps

    def _update_pheromone(self, paths, rewards, charge_configs):
        """更新信息素（同时更新路径和充电信息素）"""
        # 信息素挥发
        self.pheromone *= (1 - self.rho)
        self.charge_pheromone *= (1 - self.rho_charge)
        
        if rewards:
            best_idx = np.argmax(rewards)
            best_reward = rewards[best_idx]
            best_path = paths[best_idx]
            best_charges = charge_configs[best_idx]
            
            # 只对较好的路径进行信息素增强
            if best_reward > -50:
                # 路径信息素增量与路径质量成正比
                delta_path = self.Q * max(0.1, best_reward / 100.0)
                
                # 增强路径上的边
                for i in range(len(best_path) - 1):
                    from_node = best_path[i]
                    to_node = best_path[i + 1]
                    self.pheromone[from_node, to_node] += delta_path
                
                # 更新充电信息素
                delta_charge = self.Q_charge * max(0.1, best_reward / 100.0)
                for node_id, charge_level in best_charges.items():
                    if node_id != self.station_idx:
                        self.charge_pheromone[node_id, charge_level] += delta_charge
                
                # 确保信息素值在合理范围内
                self.pheromone = np.clip(self.pheromone, 0.1, 10.0)
                self.charge_pheromone = np.clip(self.charge_pheromone, 0.1, 10.0)

    def solve(self):
        print(f"ACO优化开始: {self.n_ants}只蚂蚁, {self.n_iterations}次迭代, {len(self.sensor_nodes)}个传感器节点")
        start_t = time.time()
        
        for iteration in range(self.n_iterations):
            paths = []
            rewards = []
            charge_configs = []
            complete_paths = 0
            
            for ant_id in range(self.n_ants):
                # 构建路径
                path, is_complete = self._construct_ant_path(ant_id)
                
                if is_complete and len(set(path) - {self.station_idx}) == len(self.sensor_nodes):
                    complete_paths += 1
                
                # 生成充电配置
                ant_durations = self._generate_charge_durations()
                
                # 评估路径
                if len(path) > 0:
                    reward, _ = self._run_path_simulation(path, ant_durations)
                else:
                    reward = -1000
                
                paths.append(path)
                rewards.append(reward)
                charge_configs.append(ant_durations)
                
                # 更新全局最优
                if reward > self.best_reward and is_complete:
                    visited_sensors = set(path) - {self.station_idx}
                    if len(visited_sensors) == len(self.sensor_nodes):
                        self.best_reward = reward
                        self.best_path = deepcopy(path)
                        self.best_durations = deepcopy(ant_durations)
                        self.no_improve_count = 0
            
            self.no_improve_count += 1
            
            # 更新信息素
            self._update_pheromone(paths, rewards, charge_configs)
            
            # 打印进度
            if (iteration + 1) % 10 == 0:
                visited_count = len(set(self.best_path) - {self.station_idx}) if self.best_path else 0
                
                # 计算平均充电等级
                avg_charge_level = 0
                if self.best_durations:
                    avg_charge_level = sum(self.best_durations.values()) / len(self.best_durations)
                
                print(f"Iter {iteration+1}/{self.n_iterations} | "
                      f"Best: {self.best_reward:.2f} | "
                      f"Nodes: {visited_count}/{len(self.sensor_nodes)} | "
                      f"Avg Charge: {avg_charge_level:.1f} | "
                      f"Patience: {self.no_improve_count}")
            
            # 早停检查
            if self.no_improve_count > self.early_stop_patience:
                print(f"早停触发: 连续{self.no_improve_count}代无改进")
                break
        
        exec_time = time.time() - start_t
        print(f"ACO优化完成, 耗时: {exec_time:.2f}s")
        
        # 打印最终的充电时间分布
        if self.best_durations:
            print("最终充电时间分布:")
            for node_id, level in self.best_durations.items():
                if node_id != self.station_idx:
                    info = self.env.global_to_original_map[node_id]
                    node_name = info["point_data"]["node_id"]
                    print(f"  节点 {node_name}: 充电等级 {level}/{self.env.charge_time_levels-1}")
        
        return self.best_path, self.best_durations, self.best_reward, exec_time

def run_detailed_simulation_aco(env, solver):
    path, durations, reward, exec_time = solver.solve()
    
    if path is None:
        print("错误: 未找到有效路径!")
        return
    
    # 最终评估
    final_reward, steps = solver._run_path_simulation(path, durations, render=True)
    
    print("\n" + "="*75)
    print(f"{'ACO 最终优化结果轨迹':^75}")
    print("="*75)
    print(f"| {'Step':<4} | {'From':<8} | {'To':<8} | {'Lvl':<3} | {'Dist(m)':<8} | {'Reward':<7} |")
    print("-" * 75)
    
    curr_id = env.start_node_id
    total_dist = 0
    charge_e, cost_e = 0, 0
    
    for i, (node_id, level, r, info) in enumerate(steps):
        from_node = env.global_to_original_map[curr_id]["point_data"]["node_id"]
        to_node = env.global_to_original_map[node_id]["point_data"]["node_id"]
        dist = info.get("distance_needed", 0)
        
        print(f"| {i+1:<4} | {from_node:<8} | {to_node:<8} | {level:<3} | {dist:<8.1f} | {r:<7.2f} |")
        
        total_dist += dist
        charge_e += info.get("step_charge_energy", 0)
        cost_e += info.get("step_move_energy", 0) + info.get("step_tx_energy", 0)
        curr_id = node_id
    
    print("-" * 75)
    print(f"执行时长: {exec_time:.2f}s | 最终奖励: {final_reward:.2f}")
    
    visited_sensors = set(path) - {env.start_node_id}
    print(f"访问节点数: {len(visited_sensors)}/{len(solver.sensor_nodes)}")
    
    if len(visited_sensors) < len(solver.sensor_nodes):
        missing = len(solver.sensor_nodes) - len(visited_sensors)
        print(f"警告: 有{missing}个节点未访问")
    
    print(f"总路径长: {total_dist:.2f}m | 能量利用率: {charge_e/(cost_e+1e-6):.4f}")
    print("="*75)

if __name__ == "__main__":
    wsnSimulation = WSNSimulation(data_input="./data/chargingScheduling_data.json", seed=42)
    data = wsnSimulation.get_data()
    env = UAVSchedulingEnv(graph_G=data["graph_G"], point_info=data["point_info"], max_min=data['max_min'])
    
    # 使用简化的启发式函数，beta设为2.0（对应平方）
    solver = ACOSolver(env, n_ants=100, n_iterations=1000, beta=2.0,
                       alpha_charge=1.0, rho_charge=0.1, Q_charge=50)
    run_detailed_simulation_aco(env, solver)