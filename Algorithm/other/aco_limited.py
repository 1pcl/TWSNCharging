"""
基于改进蚁群算法的WRSN充电调度优化 - 有限电池容量版本
参考论文: Wireless sensor network routing optimization based on improved ant colony algorithm
Heliyon 10 (2024) e23577

核心改进（基于aco_unlimited）：
1. 转移概率引入能量紧急度因子 [公式23] - 优先选择能量低的节点充电
2. 分段启发式函数 [公式24] - 避免远距离节点被过早选择
3. 动态α参数 [公式25] - 初期快速收敛，后期探索新路径
4. 信息素浓度限制 [公式26,28] - 防止过早收敛
5. 路径能量均衡评价 [公式27] - 选择能量分布均衡的路径
6. 支持多次往返充电站（有限电池容量）
"""

import numpy as np
import os
import sys
import time
import json
import random
import matplotlib.pyplot as plt
from typing import Dict, List, Tuple, Any
from collections import defaultdict
import heapq

# --- 路径设置 ---
current_script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_script_dir)
if project_root not in sys.path:
    sys.path.insert(0, project_root)

try:
    from Scenario.uav_scheduling_limited import UAVSchedulingLimitedEnv
    from Scenario.WSNsimulation import WSNSimulation
except ImportError as e:
    print(f"导入模块失败: {e}")
    sys.exit(1)

# 创建图片保存目录
IMAGE_DIR = os.path.join(current_script_dir, "images_aco_limited")
os.makedirs(IMAGE_DIR, exist_ok=True)


class RoadNetworkWithChargingNodes:
    """道路网络加载器"""
    
    def __init__(self, json_file: str = "./data/road_graph_with_nodes.json"):
        start_time = time.time()
        with open(json_file, 'r') as f:
            self.data = json.load(f)
        
        self.nodes = {}
        for node_data in self.data['nodes']:
            node_id = node_data['id']
            self.nodes[node_id] = {
                'node_id': node_id,
                'type': node_data['node_type'],
                'x': node_data['x_m'],
                'y': node_data['y_m'],
                'degree': node_data['degree'],
                'neighbors': node_data['neighbors']
            }
        
        self.full_graph = defaultdict(dict)
        for edge in self.data['edges']:
            u = edge['from']
            v = edge['to']
            length = edge['length_m']
            self.full_graph[u][v] = length
            self.full_graph[v][u] = length
        
        self.statistics = self.data['statistics']
        load_time = time.time() - start_time
        print(f"加载道路网络成功: {load_time:.4f}s")
        print(f"  总节点数: {self.statistics['total_nodes']}")
        print(f"  路口数: {self.statistics['intersections']}")
        print(f"  传感器数: {self.statistics['sensors']}")
        print(f"  中继节点数: {self.statistics['relays']}")
        print(f"  充电站数: {self.statistics['stations']}")
        print(f"  道路段数: {self.statistics['total_edges']}")
    
    def compute_shortest_paths_full(self, node_ids: List[str]):
        """计算节点间最短路径距离"""
        start_time = time.time()
        n = len(node_ids)
        node_to_idx = {node_id: i for i, node_id in enumerate(node_ids)}
        dist_matrix = np.full((n, n), float('inf'), dtype=np.float32)
        
        for i, start_id in enumerate(node_ids):
            dist_matrix[i, i] = 0.0
            distances = {node: float('inf') for node in self.nodes}
            distances[start_id] = 0
            pq = [(0, start_id)]
            visited = set()
            
            while pq:
                current_dist, current = heapq.heappop(pq)
                if current in visited:
                    continue
                visited.add(current)
                
                for neighbor, length in self.full_graph.get(current, {}).items():
                    new_dist = current_dist + length
                    if new_dist < distances[neighbor]:
                        distances[neighbor] = new_dist
                        heapq.heappush(pq, (new_dist, neighbor))
            
            for j, end_id in enumerate(node_ids):
                if end_id in distances:
                    dist_matrix[i, j] = distances[end_id]
        
        # elapsed = time.time() - start_time
        # print(f"  距离矩阵计算完成: {elapsed:.4f}s")
        return dist_matrix, node_to_idx
    
    def get_path_between_full(self, start_id: str, end_id: str) -> List[str]:
        """获取两点间最短路径节点序列"""
        if start_id == end_id:
            return [start_id]
        
        distances = {node: float('inf') for node in self.nodes}
        previous = {node: None for node in self.nodes}
        distances[start_id] = 0
        pq = [(0, start_id)]
        visited = set()
        
        while pq:
            current_dist, current = heapq.heappop(pq)
            if current in visited:
                continue
            visited.add(current)
            
            if current == end_id:
                break
            
            for neighbor, length in self.full_graph.get(current, {}).items():
                new_dist = current_dist + length
                if new_dist < distances[neighbor]:
                    distances[neighbor] = new_dist
                    previous[neighbor] = current
                    heapq.heappush(pq, (new_dist, neighbor))
        
        path = []
        current = end_id
        while current is not None:
            path.append(current)
            current = previous[current]
        path.reverse()
        
        return path if path and path[0] == start_id else [start_id, end_id]
    
    def get_node_by_id(self, node_id: str) -> Dict:
        return self.nodes.get(node_id)


class ImprovedAntColonyOptimizerLimited:
    """
    改进蚁群算法（有限电池容量版本）
    
    基于aco_unlimited的改进思想：
    1. 转移概率引入能量紧急度因子 [公式23] - 优先选择能量低的节点充电
    2. 分段启发式函数 [公式24] - 根据距离阈值选择不同的启发式
    3. 动态α参数 [公式25] - α(n) = α0 + α1 * e^(-K*n)
    4. 信息素浓度限制 [公式26,28] - τ ∈ [τ_min, τ_max]
    5. 路径能量均衡评价 [公式27] - v_a = (E_avg × E_min) / E_total
    6. 多次往返充电站（有限电池容量）
    """
    
    def __init__(self, 
                 network: RoadNetworkWithChargingNodes,
                 charging_node_info: List[Dict],
                 agent_max_battery: float = 100000000.0,
                 agent_move_consumption: float = 1000.0,
                 agent_tx_consumption: float = 100.0,
                 sensor_max_battery: float = 240000.0,
                 gateway_max_battery: float = 800000.0,
                 min_charging_rate: float = 0.7,
                 num_ants: int = 60,
                 max_iterations: int = 400,
                 alpha0: float = 0.7,           # 最终α值 (公式25)
                 alpha1: float = 1.8,           # 初始α增量 (公式25)
                 K: float = 0.008,              # 衰减速率常数 (公式25)
                 beta: float = 1.5,             # 距离重要性因子
                 gamma: float = 0.5,            # 能量紧急度重要性因子
                 rho: float = 0.02,             # 信息素挥发率
                 Q: float = 200.0,              # 信息素强度
                 tau_min: float = 0.0005,       # 最小信息素浓度 (公式26)
                 tau_max: float = 15.0,         # 最大信息素浓度 (公式26)
                 distance_threshold: float = 6000.0,  # 分段距离阈值 d0 (公式24)
                 elite_ant_ratio: float = 0.25,  # 精英蚂蚁比例
                 exploration_rate: float = 0.2,  # 随机探索概率
                 stagnation_limit: int = 100):   # 停滞阈值
        
        self.network = network
        self.agent_max_battery = agent_max_battery
        self.agent_move_consumption = agent_move_consumption
        self.agent_tx_consumption = agent_tx_consumption
        self.sensor_max_battery = sensor_max_battery
        self.gateway_max_battery = gateway_max_battery
        self.min_charging_rate = min_charging_rate
        self.num_ants = num_ants
        self.max_iterations = max_iterations
        self.alpha0 = alpha0
        self.alpha1 = alpha1
        self.K = K
        self.beta = beta
        self.gamma = gamma
        self.rho = rho
        self.Q = Q
        self.tau_min = tau_min
        self.tau_max = tau_max
        self.distance_threshold = distance_threshold
        self.elite_ant_ratio = elite_ant_ratio
        self.exploration_rate = exploration_rate
        self.stagnation_limit = stagnation_limit
        
        # 从WSN数据中提取需要充电的节点
        self.charging_node_ids = []
        self.charging_node_energy = {}
        self.node_max_energy = {}
        self.station_id = None
        
        for node in charging_node_info:
            node_id = node['node_id']
            node_type = node['type']
            
            if node_type == 'station':
                self.station_id = node_id
                self.node_max_energy[node_id] = float('inf')
            elif node['battery_info'].get('needs_charge', False):
                self.charging_node_ids.append(node_id)
                current_energy = node['battery_info'].get('current_energy', 0)
                self.charging_node_energy[node_id] = current_energy
                
                # 记录节点最大能量容量
                if node_type == 'sensor':
                    self.node_max_energy[node_id] = sensor_max_battery
                elif node_type == 'relay':
                    self.node_max_energy[node_id] = gateway_max_battery
        
        if self.station_id is None:
            raise ValueError("未找到充电站节点")
        
        # 统计信息
        sensor_count = len([n for n in self.charging_node_ids 
                           if network.get_node_by_id(n) and network.get_node_by_id(n)['type'] == 'sensor'])
        relay_count = len(self.charging_node_ids) - sensor_count
        
        # print(f"  需充电节点: {len(self.charging_node_ids)}个 (传感器: {sensor_count}, 中继: {relay_count})")
        
        # 构建完整节点列表
        self.all_node_ids = self.charging_node_ids + [self.station_id]
        self.n_nodes = len(self.charging_node_ids)
        
        # 计算最短路径距离矩阵
        # print(f"  计算 {len(self.all_node_ids)} 个节点之间的最短路径...")
        self.dist_matrix, self.node_to_idx = network.compute_shortest_paths_full(self.all_node_ids)
        
        self.idx_to_node = {idx: node_id for node_id, idx in self.node_to_idx.items()}
        self.station_idx = self.node_to_idx[self.station_id]
        self.charging_indices = [self.node_to_idx[node_id] for node_id in self.charging_node_ids]
        
        # 构建节点能量紧急度数组（能量越低，紧急度越高）
        self.energy_urgency = np.zeros(self.n_nodes + 1)
        for node_id, idx in self.node_to_idx.items():
            if node_id == self.station_id:
                self.energy_urgency[idx] = 0  # 充电站紧急度为0
            else:
                current = self.charging_node_energy.get(node_id, 0)
                max_energy = self.node_max_energy.get(node_id, 1)
                # 论文公式(23)：能量缺口比例，能量越低，紧急度越高
                if max_energy > 0:
                    self.energy_urgency[idx] = (max_energy - current) / max_energy
                else:
                    self.energy_urgency[idx] = 0
        
        # 计算充电参数
        self._calculate_charging_parameters()
        
        # 信息素矩阵初始化
        self.pheromone = np.ones((self.n_nodes + 1, self.n_nodes + 1)) * 0.5
        self._limit_pheromone()
        
        # 启发式信息矩阵 (论文公式24: 分段函数)
        self.heuristic = np.zeros((self.n_nodes + 1, self.n_nodes + 1))
        for i in range(self.n_nodes + 1):
            for j in range(self.n_nodes + 1):
                if i != j:
                    dist = self.dist_matrix[i, j]
                    if dist > 0:
                        # 论文公式(24): 分段启发式函数
                        if dist < self.distance_threshold:
                            self.heuristic[i, j] = 1.0 / (dist ** 2)
                        else:
                            self.heuristic[i, j] = 1.0 / (dist ** 4)
        
        # 记录最优解
        self.best_trips = None
        self.best_complete_sequence = None
        self.best_charging_sequence = None
        self.best_total_distance = float('inf')
        self.best_energy_metric = -float('inf')
        self.convergence_history = []
        self.distance_history = []
        self.alpha_history = []
        
        # 初始化概率矩阵
        self._update_probabilities(iteration=0)
        
        # print(f"  改进蚁群算法初始化完成（有限电池容量版本）")
        # print(f"    蚂蚁数量: {num_ants}")
        # print(f"    最大迭代次数: {max_iterations}")
        # print(f"    动态α参数: α0={alpha0}, α1={alpha1}, K={K}")
        # print(f"    β (距离权重): {beta}")
        # print(f"    γ (能量紧急度权重): {gamma}")
        # print(f"    ρ (挥发率): {rho}")
        # print(f"    信息素范围: [{tau_min}, {tau_max}]")
        # print(f"    距离阈值 d0: {distance_threshold:.2f} m")
        # print(f"    随机探索概率: {exploration_rate}")
        # print(f"    无人车电池容量: {agent_max_battery/1000:.0f} kJ")
        # print(f"    移动能耗: {agent_move_consumption} J/m")
    
    def _calculate_charging_parameters(self):
        """计算充电参数"""
        self.tx_rate = self.min_charging_rate * self.agent_tx_consumption
        self.sensor_max_charge = self.sensor_max_battery / self.tx_rate
        self.gateway_max_charge = self.gateway_max_battery / self.tx_rate
    
    def _get_charging_energy(self, node_id: str) -> float:
        """计算给节点充满电需要的传输能量"""
        node_info = self.network.get_node_by_id(node_id)
        if not node_info:
            return 0
        
        node_type = node_info['type']
        
        if node_id in self.charging_node_energy:
            current = self.charging_node_energy[node_id]
            if node_type == 'sensor':
                max_battery = self.sensor_max_battery
            else:
                max_battery = self.gateway_max_battery
            energy_needed = max_battery - current
            # 考虑充电效率
            charge_energy = energy_needed / self.min_charging_rate
            return max(0, charge_energy)
        
        return 0
    
    def _get_distance(self, node1_idx: int, node2_idx: int) -> float:
        """获取两个节点之间的最短路径距离"""
        return self.dist_matrix[node1_idx, node2_idx]
    
    def _can_go_and_return(self, current_idx: int, target_idx: int, remaining_battery: float) -> bool:
        """检查从起点出发到目标节点并返回充电站是否可行"""
        dist_to_target = self._get_distance(current_idx, target_idx)
        target_node_id = self.idx_to_node[target_idx]
        dist_back_to_station = self._get_distance(target_idx, self.station_idx)
        
        # 移动能耗
        move_energy = (dist_to_target + dist_back_to_station) * self.agent_move_consumption
        
        # 充电能耗
        charge_energy = self._get_charging_energy(target_node_id)
        
        total_needed_energy = move_energy + charge_energy
        
        # 预留安全裕度
        safe_energy = remaining_battery * 0.9
        
        return total_needed_energy <= safe_energy
    
    def _limit_pheromone(self):
        """限制信息素浓度 (论文公式26, 28)"""
        self.pheromone = np.clip(self.pheromone, self.tau_min, self.tau_max)
    
    def _get_dynamic_alpha(self, iteration: int) -> float:
        """动态调整α参数 (论文公式25): α(n) = α0 + α1 * e^(-K * n)"""
        return self.alpha0 + self.alpha1 * np.exp(-self.K * iteration)
    
    def _update_probabilities(self, iteration: int):
        """更新转移概率矩阵 (论文公式23)"""
        current_alpha = self._get_dynamic_alpha(iteration)
        
        # 计算转移概率: τ^α × η^β × U^γ
        tau_term = self.pheromone ** current_alpha
        eta_term = self.heuristic ** self.beta
        
        # 能量紧急度项
        urgency_term = np.ones_like(self.pheromone)
        for j in range(self.n_nodes + 1):
            urgency_term[:, j] = self.energy_urgency[j] ** self.gamma
        
        prob_matrix = tau_term * eta_term * urgency_term
        
        # 归一化
        row_sums = prob_matrix.sum(axis=1, keepdims=True)
        row_sums[row_sums == 0] = 1e-10
        self.prob_matrix = prob_matrix / row_sums
        
        self.alpha_history.append(current_alpha)
    
    def _calculate_energy_metric(self, charging_indices_in_path: List[int], distance: float) -> float:
        """
        计算路径能量均衡指标 (论文公式27)
        v_a = (E_avg × E_min) / E_total
        
        其中:
        - E_avg: 路径上节点的平均剩余能量
        - E_min: 路径上节点的最小剩余能量
        - E_total: 路径总能耗（用路径距离近似）
        
        该值越大，表示路径能量分布越均衡，越优
        """
        if not charging_indices_in_path:
            return 0
        
        # 获取各节点的剩余能量（原始值，不是紧急度）
        energies = []
        for idx in charging_indices_in_path:
            node_id = self.idx_to_node[idx]
            if node_id in self.charging_node_energy:
                energies.append(self.charging_node_energy[node_id])
        
        if not energies:
            return 0
        
        E_avg = np.mean(energies)
        E_min = np.min(energies)
        
        # 用路径距离作为总能耗的近似
        E_total = distance / 1000.0  # 归一化
        
        if E_total <= 0:
            return 0
        
        v_a = (E_avg * E_min) / (E_total + 1e-10)
        
        return v_a
    
    def _construct_trip(self, remaining_indices: set, start_idx: int, 
                        remaining_battery: float, iteration: int) -> Tuple[List[int], float, List[int]]:
        """
        使用改进蚁群算法的转移概率构建一趟行程
        从充电站出发，选择下一个节点，直到无法再访问更多节点
        
        返回:
            trip_indices: 行程中的节点索引列表
            trip_distance: 行程总移动距离
            visited_indices: 实际访问的节点索引
        """
        if not remaining_indices:
            return [], 0, []
        
        current = start_idx
        trip = []
        trip_distance = 0
        visited = []
        current_battery = remaining_battery
        
        # 随机探索
        if random.random() < self.exploration_rate:
            # 随机顺序访问
            random_order = list(remaining_indices)
            random.shuffle(random_order)
            visited = []
            trip_distance = 0
            current = start_idx
            
            for next_node in random_order:
                if not self._can_go_and_return(current, next_node, current_battery):
                    break
                
                dist = self._get_distance(current, next_node)
                charge_energy = self._get_charging_energy(self.idx_to_node[next_node])
                
                current_battery -= dist * self.agent_move_consumption
                current_battery -= charge_energy
                trip_distance += dist
                visited.append(next_node)
                current = next_node
            
            # 返回充电站
            if visited:
                return_dist = self._get_distance(current, self.station_idx)
                trip_distance += return_dist
            
            return visited, trip_distance, visited
        
        # 正常选择（使用转移概率）
        feasible_nodes = {idx for idx in remaining_indices 
                         if self._can_go_and_return(current, idx, current_battery)}
        
        while feasible_nodes:
            candidates = list(feasible_nodes)
            
            # 候选节点剪枝：只考虑最近的15个候选节点
            if len(candidates) > 15:
                dist_to_candidates = [(c, self.dist_matrix[current, c]) for c in candidates]
                dist_to_candidates.sort(key=lambda x: x[1])
                candidates = [c for c, _ in dist_to_candidates[:15]]
            
            # 计算转移概率
            probs = []
            current_alpha = self._get_dynamic_alpha(iteration)
            
            for candidate in candidates:
                # 信息素项
                tau_term = self.pheromone[current, candidate] ** current_alpha
                
                # 启发式项（分段函数）
                dist = self.dist_matrix[current, candidate]
                if dist < self.distance_threshold:
                    eta_term = (1.0 / (dist ** 2)) ** self.beta
                else:
                    eta_term = (1.0 / (dist ** 4)) ** self.beta
                
                # 能量紧急度项（能量越低，概率越大）
                urgency_term = self.energy_urgency[candidate] ** self.gamma
                
                prob = tau_term * eta_term * urgency_term
                probs.append(prob)
            
            # 轮盘赌选择
            total_prob = sum(probs)
            if total_prob < 1e-10:
                next_node = random.choice(candidates)
            else:
                probs = [p / total_prob for p in probs]
                r = random.random()
                cumsum = 0
                next_node = candidates[-1]
                for candidate, prob in zip(candidates, probs):
                    cumsum += prob
                    if r <= cumsum:
                        next_node = candidate
                        break
            
            # 更新电量
            dist = self._get_distance(current, next_node)
            charge_energy = self._get_charging_energy(self.idx_to_node[next_node])
            
            current_battery -= dist * self.agent_move_consumption
            current_battery -= charge_energy
            trip_distance += dist
            
            trip.append(next_node)
            visited.append(next_node)
            current = next_node
            
            # 更新可行节点集合
            feasible_nodes = {idx for idx in remaining_indices if idx not in visited
                             and self._can_go_and_return(current, idx, current_battery)}
        
        # 返回充电站
        if visited and current != self.station_idx:
            return_dist = self._get_distance(current, self.station_idx)
            trip_distance += return_dist
        
        return trip, trip_distance, visited
    
    def _construct_complete_path(self, iteration: int) -> Tuple[List[List[int]], List[int], float, List[int]]:
        """
        构建一条完整路径（多次往返充电站）
        使用改进的转移概率
        
        返回:
            trips: 每趟行程的节点索引列表
            complete_sequence: 完整序列（包含充电站）
            total_distance: 总路径距离
            all_charging_indices: 所有访问的充电节点索引（用于能量评价）
        """
        unvisited = set(self.charging_indices)
        trips = []
        complete_sequence = [self.station_idx]
        total_distance = 0
        all_charging_indices = []
        
        while unvisited:
            current_battery = self.agent_max_battery
            
            # 构建一趟行程
            trip, trip_distance, visited = self._construct_trip(
                unvisited, self.station_idx, current_battery, iteration
            )
            
            if not trip:
                break
            
            trips.append(trip)
            total_distance += trip_distance
            all_charging_indices.extend(visited)
            
            # 添加到完整序列
            for node in trip:
                complete_sequence.append(node)
            complete_sequence.append(self.station_idx)
            
            # 移除已访问节点
            for node in visited:
                unvisited.remove(node)
        
        return trips, complete_sequence, total_distance, all_charging_indices
    
    def _update_pheromone(self, all_paths: List[Tuple[List[List[int]], List[int], float, List[int], float]], 
                          iteration: int):
        """
        更新信息素 (论文公式26, 28)
        """
        # 信息素挥发
        self.pheromone *= (1 - self.rho)
        
        # 对所有路径增加信息素
        for trips, complete_sequence, total_distance, all_charging_indices, energy_metric in all_paths:
            # 信息素增量与路径距离成反比，与能量均衡指标成正比
            delta = self.Q / total_distance * (1 + energy_metric / 1000.0)
            for i in range(len(complete_sequence) - 1):
                from_idx = complete_sequence[i]
                to_idx = complete_sequence[i + 1]
                self.pheromone[from_idx, to_idx] += delta
                self.pheromone[to_idx, from_idx] += delta
        
        # 精英蚂蚁策略：对最优路径额外增加信息素
        if self.best_complete_sequence is not None and self.best_total_distance < float('inf'):
            elite_delta = self.elite_ant_ratio * self.Q / self.best_total_distance
            for i in range(len(self.best_complete_sequence) - 1):
                from_idx = self.best_complete_sequence[i]
                to_idx = self.best_complete_sequence[i + 1]
                self.pheromone[from_idx, to_idx] += elite_delta
                self.pheromone[to_idx, from_idx] += elite_delta
        
        # 限制信息素浓度
        self._limit_pheromone()
        
        # 更新概率矩阵
        self._update_probabilities(iteration)
    
    def optimize(self, verbose: bool = True) -> Dict:
        """执行改进蚁群优化"""
        # print("\n  开始改进蚁群优化（有限电池容量版本）...")
        start_time = time.time()
        
        stagnation_counter = 0
        
        for iteration in range(self.max_iterations):
            # 构建所有蚂蚁的路径
            all_paths = []
            iteration_best_distance = float('inf')
            iteration_best_trips = None
            iteration_best_sequence = None
            iteration_best_charging_indices = None
            iteration_best_energy_metric = -float('inf')
            
            for ant in range(self.num_ants):
                trips, complete_sequence, total_distance, all_charging_indices = self._construct_complete_path(iteration)
                energy_metric = self._calculate_energy_metric(all_charging_indices, total_distance)
                num_trips = len(trips)
                
                all_paths.append((trips, complete_sequence, total_distance, all_charging_indices, energy_metric))
                
                # 使用能量均衡指标作为主要评价标准
                if energy_metric > iteration_best_energy_metric:
                    iteration_best_energy_metric = energy_metric
                    iteration_best_distance = total_distance
                    iteration_best_trips = trips
                    iteration_best_sequence = complete_sequence
                    iteration_best_charging_indices = all_charging_indices
            
            # 更新全局最优解
            if iteration_best_energy_metric > self.best_energy_metric:
                improvement = iteration_best_energy_metric - self.best_energy_metric
                self.best_energy_metric = iteration_best_energy_metric
                self.best_total_distance = iteration_best_distance
                self.best_trips = iteration_best_trips
                self.best_complete_sequence = iteration_best_sequence
                
                # 构建充电序列
                charging_sequence = []
                for trip in self.best_trips:
                    charging_sequence.extend([self.idx_to_node[idx] for idx in trip])
                self.best_charging_sequence = charging_sequence
                
                stagnation_counter = 0
                
                if verbose and improvement > 0:
                    print(f"    迭代 {iteration+1}: 找到更优路径, "
                          f"能量指标={self.best_energy_metric:.4f}, "
                          f"距离={self.best_total_distance:.2f}m, "
                          f"行程数={len(self.best_trips)}")
            else:
                stagnation_counter += 1
            
            # 更新信息素
            self._update_pheromone(all_paths, iteration)
            
            # 记录收敛历史
            self.convergence_history.append(iteration_best_distance)
            self.distance_history.append(self.best_total_distance)
            
            # 进度输出
            if verbose and (iteration + 1) % 30 == 0:
                current_alpha = self._get_dynamic_alpha(iteration)
                print(f"    迭代 {iteration+1}/{self.max_iterations}: "
                      f"α={current_alpha:.4f}, "
                      f"最优距离={self.best_total_distance/1000:.2f}km, "
                      f"行程数={len(self.best_trips)}, "
                      f"能量指标={self.best_energy_metric:.4f}")
            
            # 停滞检测
            if stagnation_counter >= self.stagnation_limit:
                if verbose:
                    print(f"    在迭代 {iteration+1} 检测到停滞，提前终止")
                break
        
        elapsed = time.time() - start_time
        print(f"\n  优化完成: {elapsed:.4f}s")
        print(f"    最优路径距离: {self.best_total_distance:.2f} m ({self.best_total_distance/1000:.2f} km)")
        print(f"    最优能量均衡指标: {self.best_energy_metric:.4f}")
        print(f"    行程次数: {len(self.best_trips)}")
        for i, trip in enumerate(self.best_trips):
            print(f"      第 {i+1} 趟: {len(trip)} 个节点")
        print(f"    总迭代次数: {iteration+1}")
        
        # 构建完整序列（字符串格式）
        complete_sequence_str = [self.idx_to_node[idx] for idx in self.best_complete_sequence]
        
        return {
            'complete_sequence': complete_sequence_str,
            'charging_sequence': self.best_charging_sequence,
            'trips': [[self.idx_to_node[idx] for idx in trip] for trip in self.best_trips],
            'station_id': self.station_id,
            'total_distance': self.best_total_distance,
            'total_nodes': len(self.best_charging_sequence),
            'num_trips': len(self.best_trips),
            'best_energy_metric': self.best_energy_metric,
            'convergence_history': self.convergence_history,
            'distance_history': self.distance_history,
            'alpha_history': self.alpha_history,
            'optimization_time': elapsed,
            'iterations': iteration + 1
        }
    
    def get_full_path(self, sequence: List[str]) -> List[str]:
        """获取完整的大地图路径节点序列"""
        full_path = []
        current = sequence[0]
        
        for target_id in sequence[1:]:
            path = self.network.get_path_between_full(current, target_id)
            if len(path) > 1:
                full_path.extend(path[1:])
            else:
                full_path.extend(path)
            current = target_id
        
        full_path = [sequence[0]] + full_path
        return full_path


class ACOSimulatorLimited:
    """蚁群算法仿真器 - 使用 UAVSchedulingLimitedEnv 计算性能指标（有限电池容量）"""
    
    def __init__(self, wsn_data: Dict[str, Any]):
        self.graph_G = wsn_data['graph_G']
        self.point_info = wsn_data['point_info']
        self.max_min = wsn_data['max_min']
        
        self.env = UAVSchedulingLimitedEnv(
            graph_G=self.graph_G,
            point_info=self.point_info,
            max_min=self.max_min,
            agent_max_battery=100000000.0,
            agent_move_consumption=1000.0,
            agent_tx_consumption=100.0,
            agent_speed=5.0,
            min_charging_rate=0.70,
            sensor_max_battery=240000.0,
            gateway_max_battery=800000.0,
            charge_time_levels=100,
            is_train=False
        )
        
        self.env.reset()
        
        self.original_to_global = {}
        self.global_to_original = {}
        self.original_to_type = {}
        
        for gid, data in self.env.global_to_original_map.items():
            orig_id = data['original_id']
            self.original_to_global[orig_id] = gid
            self.global_to_original[gid] = orig_id
            self.original_to_type[orig_id] = data['original_type']
        
        self.station_original_id = None
        for gid, data in self.env.global_to_original_map.items():
            if data["original_type"] == "station":
                self.station_original_id = data["original_id"]
                break
        
        self.charge_power = 100.0
        self.sensor_max_battery = 240000.0
        self.gateway_max_battery = 800000.0
        
        min_charging_rate = 0.70
        tx_rate_min = min_charging_rate * self.charge_power
        self.sensor_charge_time_step = (self.sensor_max_battery / tx_rate_min) / 100
        self.gateway_charge_time_step = (self.gateway_max_battery / tx_rate_min) / 100
    
    def _calculate_charge_level(self, target_id: str) -> int:
        node_info = None
        for info in self.point_info:
            if info['node_id'] == target_id:
                node_info = info
                break
        
        if node_info is None:
            return 0
        
        node_type = node_info['type']
        current_energy = node_info['battery_info'].get('current_energy', 0)
        charging_rate = node_info['battery_info'].get('charging_rate', 0.85)
        
        effective_power = self.charge_power * charging_rate
        
        if node_type == 'sensor':
            max_battery = self.sensor_max_battery
            charge_time_step = self.sensor_charge_time_step
        elif node_type == 'relay':
            max_battery = self.gateway_max_battery
            charge_time_step = self.gateway_charge_time_step
        else:
            return 0
        
        energy_needed = max_battery - current_energy
        if energy_needed <= 0:
            return 0
        
        charge_time = energy_needed / effective_power if effective_power > 0 else 0
        charge_level = int(charge_time / charge_time_step)
        charge_level = min(99, max(0, charge_level))
        
        return charge_level
    
    def run_sequence(self, complete_sequence: List[str], charging_sequence: List[str]) -> Dict:
        total_reward = 0
        total_distance = 0
        step = 0
        return_count = 0
        step_rewards = []
        
        for i, target_id in enumerate(complete_sequence):
            is_station = (target_id == self.station_original_id)
            
            if i == 0 and is_station:
                continue
            
            if target_id not in self.original_to_global:
                continue
            
            target_global_id = self.original_to_global[target_id]
            node_type = self.original_to_type[target_id]
            
            if node_type == "station":
                charge_level = 0
                return_count += 1
            else:
                charge_level = self._calculate_charge_level(target_id)
            
            action = np.array([target_global_id, charge_level])
            obs, reward, terminated, truncated, info = self.env.step(action)
            
            total_reward += reward
            step_rewards.append(reward)
            distance = info.get("distance_needed", 0)
            total_distance += distance
            step += 1
            
            if terminated or truncated:
                break
        
        final_info = self.env._get_info()
        
        total_consumed = self.env.tx_energy + self.env.move_energy + self.env.active_energy
        eue = self.env.charg_energy / total_consumed if total_consumed > 0 else 0
        
        return {
            'total_steps': step,
            'total_distance': total_distance,
            'total_reward': total_reward,
            'step_rewards': step_rewards,
            'charged_count': final_info["node_charge_count"],
            'total_nodes': len(charging_sequence),
            'num_trips': return_count,
            'eue': eue,
            'charge_energy': self.env.charg_energy,
            'move_energy': self.env.move_energy,
            'tx_energy': self.env.tx_energy,
            'active_energy': self.env.active_energy
        }


class ACOVisualizerLimited:
    """蚁群算法结果可视化器（支持多趟行程）"""
    
    def __init__(self, network: RoadNetworkWithChargingNodes):
        self.network = network
    
    def plot_original(self, title="Original Road Network", save_path=None):
        fig, ax = plt.subplots(figsize=(14, 12))
        
        for edge in self.network.data['edges']:
            from_node = self.network.nodes[edge['from']]
            to_node = self.network.nodes[edge['to']]
            ax.plot([from_node['x'], to_node['x']], [from_node['y'], to_node['y']], 
                   'black', linewidth=1, alpha=0.6, zorder=1)
        
        for node_id, node in self.network.nodes.items():
            x, y = node['x'], node['y']
            node_type = node['type']
            
            if node_type == 'intersection':
                ax.scatter(x, y, c='black', s=20, marker='o', alpha=0.7, zorder=2)
            elif node_type == 'sensor':
                ax.scatter(x, y, c='lightblue', s=25, marker='o', alpha=0.7, zorder=3)
            elif node_type == 'relay':
                ax.scatter(x, y, c='darkblue', s=120, marker='*', alpha=0.9, zorder=3)
            elif node_type == 'station':
                ax.scatter(x, y, c='green', s=80, marker='s', alpha=0.9, zorder=3)
        
        ax.set_xlabel('X (km)', fontsize=12)
        ax.set_ylabel('Y (km)', fontsize=12)
        
        from matplotlib.ticker import FuncFormatter
        ax.xaxis.set_major_formatter(FuncFormatter(lambda x, p: f'{x/1000:.1f}'))
        ax.yaxis.set_major_formatter(FuncFormatter(lambda y, p: f'{y/1000:.1f}'))
        
        ax.set_title(title, fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.2, linestyle='--')
        ax.set_aspect('equal')
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"  图片已保存: {save_path}")
        plt.close()
        return fig
    
    def plot_charging_nodes(self, charging_node_info: List[Dict], title="Charging Nodes", save_path=None):
        fig, ax = plt.subplots(figsize=(14, 12))
        
        charging_sensor_ids = []
        charging_relay_ids = []
        for node in charging_node_info:
            if node['battery_info'].get('needs_charge', False):
                if node['type'] == 'sensor':
                    charging_sensor_ids.append(node['node_id'])
                elif node['type'] == 'relay':
                    charging_relay_ids.append(node['node_id'])
        
        sensor_set = set(charging_sensor_ids)
        relay_set = set(charging_relay_ids)
        
        for edge in self.network.data['edges']:
            from_node = self.network.nodes[edge['from']]
            to_node = self.network.nodes[edge['to']]
            ax.plot([from_node['x'], to_node['x']], [from_node['y'], to_node['y']], 
                   'black', linewidth=1, alpha=0.5, zorder=1)
        
        for node_id, node in self.network.nodes.items():
            x, y = node['x'], node['y']
            node_type = node['type']
            
            if node_type == 'intersection':
                ax.scatter(x, y, c='gray', s=15, marker='o', alpha=0.5, zorder=2)
            elif node_type == 'sensor':
                if node_id in sensor_set:
                    ax.scatter(x, y, c='red', s=60, marker='o', alpha=0.9, zorder=4)
                else:
                    ax.scatter(x, y, c='lightblue', s=25, marker='o', alpha=0.6, zorder=2)
            elif node_type == 'relay':
                if node_id in relay_set:
                    ax.scatter(x, y, c='orange', s=120, marker='*', alpha=0.9, zorder=4)
                else:
                    ax.scatter(x, y, c='darkblue', s=80, marker='*', alpha=0.6, zorder=2)
            elif node_type == 'station':
                ax.scatter(x, y, c='green', s=80, marker='s', alpha=0.9, zorder=3)
        
        ax.set_xlabel('X (km)', fontsize=12)
        ax.set_ylabel('Y (km)', fontsize=12)
        
        from matplotlib.ticker import FuncFormatter
        ax.xaxis.set_major_formatter(FuncFormatter(lambda x, p: f'{x/1000:.1f}'))
        ax.yaxis.set_major_formatter(FuncFormatter(lambda y, p: f'{y/1000:.1f}'))
        
        ax.set_title(f"{title} (Total: {len(sensor_set) + len(relay_set)} nodes)", fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.2, linestyle='--')
        ax.set_aspect('equal')
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"  图片已保存: {save_path}")
        plt.close()
        return fig
    
    def plot_path(self, path_node_ids: List[str], charging_sequence: List[str], 
                  trips: List[List[str]], title="Improved ACO Limited Path Planning Result", save_path=None):
        """绘制路径规划结果（支持多趟行程不同颜色）"""
        fig, ax = plt.subplots(figsize=(14, 12))
        
        # 绘制所有道路
        for edge in self.network.data['edges']:
            from_node = self.network.nodes[edge['from']]
            to_node = self.network.nodes[edge['to']]
            ax.plot([from_node['x'], to_node['x']], [from_node['y'], to_node['y']], 
                'black', linewidth=1, alpha=0.5, zorder=1)
        
        # 绘制路径（绿色粗线）
        if len(path_node_ids) > 1:
            path_x = []
            path_y = []
            for node_id in path_node_ids:
                if node_id in self.network.nodes:
                    x, y = self.network.nodes[node_id]['x'], self.network.nodes[node_id]['y']
                    path_x.append(x)
                    path_y.append(y)
            
            ax.plot(path_x, path_y, 'green', linewidth=3, alpha=0.9, zorder=3, label='Travel Path')
        
        # 获取需要充电的节点类型
        charging_node_types = {}
        for node_id in charging_sequence:
            node_info = self.network.get_node_by_id(node_id)
            if node_info:
                charging_node_types[node_id] = node_info['type']
        
        # 绘制所有节点
        for node_id, node in self.network.nodes.items():
            x, y = node['x'], node['y']
            node_type = node['type']
            
            if node_type == 'intersection':
                ax.scatter(x, y, c='black', s=15, marker='o', alpha=0.5, zorder=2)
            elif node_type == 'sensor':
                if node_id in charging_node_types:
                    ax.scatter(x, y, c='red', s=35, marker='o', alpha=0.9, zorder=4)
                else:
                    ax.scatter(x, y, c='lightblue', s=25, marker='o', alpha=0.7, zorder=4)
            elif node_type == 'relay':
                if node_id in charging_node_types:
                    ax.scatter(x, y, c='orange', s=120, marker='*', alpha=0.9, zorder=4)
                else:
                    ax.scatter(x, y, c='darkblue', s=100, marker='*', alpha=0.8, zorder=4)
            elif node_type == 'station':
                ax.scatter(x, y, c='green', s=80, marker='s', alpha=0.9, zorder=4)
        
        # 生成行程颜色
        import colorsys
        num_trips = len(trips)
        colors = []
        
        if num_trips <= 7:
            predefined_colors = ['red', 'orange', 'purple', 'brown', 'pink', 'cyan', 'magenta']
            colors = predefined_colors[:num_trips]
        else:
            for i in range(num_trips):
                hue = (i * 137.5) % 360
                rgb = colorsys.hsv_to_rgb(hue / 360, 0.8, 0.9)
                colors.append(rgb)
        
        # 为每个节点分配行程颜色
        node_color = {}
        for trip_idx, trip in enumerate(trips):
            color = colors[trip_idx]
            for node_id in trip:
                node_color[node_id] = color
        
        # 标注充电顺序
        for i, node_id in enumerate(charging_sequence):
            if node_id in self.network.nodes:
                x, y = self.network.nodes[node_id]['x'], self.network.nodes[node_id]['y']
                edge_color = node_color.get(node_id, 'red')
                
                ax.annotate(str(i+1), (x, y), fontsize=11, ha='center', va='center', fontweight='bold',
                        bbox=dict(boxstyle="circle,pad=0.25", facecolor="white", 
                                    edgecolor=edge_color, linewidth=2, alpha=0.95), zorder=5)
        
        # 标注起点/终点
        for node_id, node in self.network.nodes.items():
            if node['type'] == 'station':
                x, y = node['x'], node['y']
                ax.annotate('START/END', (x, y), fontsize=9, ha='center', va='bottom',
                        bbox=dict(boxstyle="round,pad=0.2", facecolor="yellow", alpha=0.8))
        
        # 构建图例
        from matplotlib.lines import Line2D
        from matplotlib.patches import Patch
        
        legend_elements = [
            Line2D([0], [0], color='green', linewidth=2, label='Travel Path'),
            Line2D([0], [0], color='black', linewidth=1, label='Road'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='black', markersize=8, label='Intersection'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='red', markeredgecolor='darkred', 
                markersize=8, label='Charging Sensor'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='lightblue', markeredgecolor='deepskyblue', 
                markersize=8, label='Normal Sensor'),
            Line2D([0], [0], marker='*', color='w', markerfacecolor='orange', markeredgecolor='darkorange', 
                markersize=10, label='Charging Relay'),
            Line2D([0], [0], marker='*', color='w', markerfacecolor='darkblue', markeredgecolor='navy', 
                markersize=10, label='Normal Relay'),
            Line2D([0], [0], marker='s', color='w', markerfacecolor='green', markeredgecolor='darkgreen', 
                markersize=8, label='Station'),
        ]
        
        for trip_idx, trip in enumerate(trips):
            color = colors[trip_idx]
            if isinstance(color, tuple):
                legend_elements.append(
                    Patch(facecolor=color, edgecolor=color, alpha=0.7, 
                        label=f'Trip {trip_idx+1} ({len(trip)} nodes)')
                )
            else:
                legend_elements.append(
                    Line2D([0], [0], marker='o', color='w', markerfacecolor='white', 
                        markeredgecolor=color, markersize=8, label=f'Trip {trip_idx+1} ({len(trip)} nodes)')
                )
        
        ax.set_xlabel('X (km)', fontsize=12)
        ax.set_ylabel('Y (km)', fontsize=12)
        
        from matplotlib.ticker import FuncFormatter
        ax.xaxis.set_major_formatter(FuncFormatter(lambda x, p: f'{x/1000:.1f}'))
        ax.yaxis.set_major_formatter(FuncFormatter(lambda y, p: f'{y/1000:.1f}'))
        
        ax.set_title(title, fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.2, linestyle='--')
        ax.set_aspect('equal')
        
        plt.subplots_adjust(right=0.85)
        
        if num_trips > 8:
            ax.legend(handles=legend_elements, loc='center left', bbox_to_anchor=(1.02, 0.98), 
                    fontsize=9, framealpha=0.9, ncol=2)
        else:
            ax.legend(handles=legend_elements, loc='center left', bbox_to_anchor=(1.02, 0.98), 
                    fontsize=10, framealpha=0.9)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"  图片已保存: {save_path}")
        plt.close()
        return fig
    
    def plot_convergence(self, convergence_history: List[float], distance_history: List[float],
                         alpha_history: List[float] = None, title="Improved ACO Limited Convergence Curve", 
                         save_path=None):
        """绘制收敛曲线（可选包含α变化）"""
        if alpha_history is not None:
            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10), sharex=True)
        else:
            fig, ax1 = plt.subplots(figsize=(10, 6))
            ax2 = None
        
        iterations = range(1, len(convergence_history) + 1)
        
        ax1.plot(iterations, convergence_history, 'b-', linewidth=2, label='Iteration Best Distance')
        ax1.plot(iterations, distance_history, 'r--', linewidth=2, label='Global Best Distance')
        ax1.set_ylabel('Path Distance (m)', fontsize=12)
        ax1.set_title(title, fontsize=14, fontweight='bold')
        ax1.legend(fontsize=10)
        ax1.grid(True, alpha=0.3, linestyle='--')
        
        if ax2 is not None and alpha_history is not None:
            alpha_plot = alpha_history[:len(iterations)]
            ax2.plot(iterations, alpha_plot, 'g-', linewidth=2, label='α(n)')
            ax2.set_xlabel('Iteration', fontsize=12)
            ax2.set_ylabel('α (Pheromone Importance)', fontsize=12)
            ax2.legend(fontsize=10)
            ax2.grid(True, alpha=0.3, linestyle='--')
        
        ax1.set_xlabel('Iteration' if ax2 is None else '', fontsize=12)
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"  图片已保存: {save_path}")
        plt.close()
        return fig
    
    def plot_rewards(self, step_rewards: List[float], title="Step Rewards", save_path=None):
        fig, ax = plt.subplots(figsize=(10, 6))
        
        steps = range(1, len(step_rewards) + 1)
        
        ax.bar(steps, step_rewards, color='steelblue', alpha=0.7, edgecolor='navy')
        ax.axhline(y=0, color='red', linestyle='--', linewidth=1, alpha=0.5)
        
        ax.set_xlabel('Step', fontsize=12)
        ax.set_ylabel('Reward', fontsize=12)
        ax.set_title(title, fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3, linestyle='--', axis='y')
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"  图片已保存: {save_path}")
        plt.close()
        return fig


def main():
    print("="*70)
    print("改进蚁群算法 (Improved Ant Colony Optimization) 路径规划仿真 - 有限电池容量版本")
    print("参考论文: Wireless sensor network routing optimization based on improved ant colony algorithm")
    print("Heliyon 10 (2024) e23577")
    print("="*70)
    
    NETWORK_FILE = "./data/road_graph_with_nodes.json"
    total_start_time = time.time()
    
    # 1. 加载道路网络
    print("\n[1] 加载完整道路网络数据...")
    try:
        network = RoadNetworkWithChargingNodes(NETWORK_FILE)
    except FileNotFoundError:
        print(f"\n错误: 找不到文件 {NETWORK_FILE}")
        print("请先运行 deploy_nodes.py 生成道路网络数据")
        return
    
    # 2. 获取需要充电的节点
    print("\n[2] 使用 WSNSimulation 选择需要充电的节点...")
    start_time = time.time()
    
    wsn_sim = WSNSimulation(
        data_input=network.data,
        sensor_battery=240000.0,
        relay_battery=800000.0,
        station_battery=1000000.0,
        seed=42
    )
    
    wsn_data = wsn_sim.get_data()
    point_info = wsn_data['point_info']
    
    charging_nodes_info = [n for n in point_info if n['battery_info'].get('needs_charge', False)]
    sensors_count = len([n for n in charging_nodes_info if n['type'] == 'sensor'])
    relays_count = len([n for n in charging_nodes_info if n['type'] == 'relay'])
    
    wsn_time = time.time() - start_time
    print(f"  WSN 初始化完成: {wsn_time:.4f}s")
    print(f"  需要充电的传感器: {sensors_count} 个")
    print(f"  需要充电的中继: {relays_count} 个")
    
    # 3. 改进蚁群算法路径规划（有限电池容量）
    print("\n[3] 改进蚁群算法路径规划（有限电池容量版本）...")
    
    aco = ImprovedAntColonyOptimizerLimited(
        network=network,
        charging_node_info=point_info,
        agent_max_battery=100000000.0,
        agent_move_consumption=1000.0,
        agent_tx_consumption=100.0,
        sensor_max_battery=240000.0,
        gateway_max_battery=800000.0,
        min_charging_rate=0.7,
        # === 搜索能力参数 ===
        num_ants=60,
        max_iterations=400,
        exploration_rate=0.2,
        stagnation_limit=100,
        # === 路径选择参数 ===
        alpha0=0.7,
        alpha1=1.8,
        K=0.008,
        beta=1.5,
        gamma=0.5,
        # === 信息素更新参数 ===
        rho=0.02,
        Q=200.0,
        tau_min=0.0005,
        tau_max=15.0,
        elite_ant_ratio=0.25,
        # === 问题特定参数 ===
        distance_threshold=6000.0,
    )
    
    plan = aco.optimize(verbose=True)
    
    # 4. UAV仿真
    print("\n[4] 使用 UAV 仿真计算性能指标（有限电池容量）...")
    sim_start_time = time.time()
    
    simulator = ACOSimulatorLimited(wsn_data)
    results = simulator.run_sequence(plan['complete_sequence'], plan['charging_sequence'])
    
    sim_time = time.time() - sim_start_time
    total_time = time.time() - total_start_time
    
    # 5. 输出结果
    print("\n" + "="*70)
    print("改进蚁群算法仿真结果（有限电池容量版本）")
    print("="*70)
    print(f"| 参数设置 (基于论文):")
    print(f"|   - 蚂蚁数量: {aco.num_ants}")
    print(f"|   - 最大迭代次数: {aco.max_iterations}")
    print(f"|   - 动态α参数: α0={aco.alpha0}, α1={aco.alpha1}, K={aco.K}")
    print(f"|   - β (距离权重): {aco.beta}")
    print(f"|   - γ (能量紧急度权重): {aco.gamma}")
    print(f"|   - ρ (挥发率): {aco.rho}")
    print(f"|   - 信息素范围: [{aco.tau_min}, {aco.tau_max}]")
    print(f"|   - 距离阈值 d0: {aco.distance_threshold:.2f} m")
    print(f"|   - 随机探索概率: {aco.exploration_rate}")
    print(f"|   - 无人车电池容量: {aco.agent_max_battery/1000:.0f} kJ")
    print(f"|")
    print(f"| 优化结果:")
    print(f"|   - 优化耗时: {plan['optimization_time']:.4f} s")
    print(f"|   - 实际迭代次数: {plan['iterations']}")
    print(f"|   - 最优路径距离: {plan['total_distance']:.2f} m ({plan['total_distance']/1000:.2f} km)")
    print(f"|   - 能量均衡指标: {plan['best_energy_metric']:.4f}")
    print(f"|   - 行程次数: {plan['num_trips']}")
    for i, trip in enumerate(plan['trips']):
        print(f"|     第 {i+1} 趟: {len(trip)} 个节点")
    print(f"|")
    print(f"| 仿真结果:")
    print(f"|   - 仿真耗时: {sim_time:.4f} s")
    print(f"|   - 总步数: {results['total_steps']}")
    print(f"|   - 总路径长度: {results['total_distance']:.2f} m")
    print(f"|   - 累积奖励: {results['total_reward']:.2f}")
    print(f"|   - 平均每步奖励: {results['total_reward']/results['total_steps']:.2f}" if results['total_steps'] > 0 else "|   - 平均每步奖励: N/A")
    print(f"|   - 实际充电节点数: {results['charged_count']}")
    print(f"|   - 总节点数: {results['total_nodes']}")
    print(f"|   - 返回充电站次数: {results['num_trips']}")
    print(f"|   - 平均能量利用效率 (EUE): {results['eue']:.4f}")
    print(f"|")
    print(f"| 能量统计:")
    print(f"|   - 充电能量: {results['charge_energy']:.2f} J")
    print(f"|   - 移动能量: {results['move_energy']:.2f} J")
    print(f"|   - 传输能量: {results['tx_energy']:.2f} J")
    print(f"|   - 激活能量: {results['active_energy']:.2f} J")
    print(f"|   - 总消耗能量: {results['move_energy'] + results['tx_energy'] + results['active_energy']:.2f} J")
    print(f"|")
    print(f"| 总耗时: {total_time:.4f} s")
    print("="*70)
    
    # 6. 生成可视化
    print("\n[5] 生成可视化图片...")
    
    visualizer = ACOVisualizerLimited(network)
    
    visualizer.plot_original(
        title="Original Road Network (Improved ACO Limited)",
        save_path=os.path.join(IMAGE_DIR, "01_original_road_network_improved_aco_limited.png")
    )
    
    visualizer.plot_charging_nodes(
        point_info,
        title=f"Charging Nodes (Improved ACO Limited)",
        save_path=os.path.join(IMAGE_DIR, "02_charging_nodes_improved_aco_limited.png")
    )
    
    full_path = aco.get_full_path(plan['complete_sequence'])
    visualizer.plot_path(
        full_path,
        plan['charging_sequence'],
        plan['trips'],
        title=f"Improved ACO Limited Path Planning Result ({plan['num_trips']} trips, Dist: {plan['total_distance']/1000:.2f} km)",
        save_path=os.path.join(IMAGE_DIR, "03_path_planning_result_improved_aco_limited.png")
    )
    
    visualizer.plot_convergence(
        plan['convergence_history'],
        plan['distance_history'],
        alpha_history=plan.get('alpha_history'),
        title="Improved ACO Limited Convergence Curve",
        save_path=os.path.join(IMAGE_DIR, "04_convergence_curve_improved_aco_limited.png")
    )
    
    visualizer.plot_rewards(
        results['step_rewards'],
        title=f"Step Rewards (Total: {results['total_reward']:.2f})",
        save_path=os.path.join(IMAGE_DIR, "05_step_rewards_improved_aco_limited.png")
    )
    
    print(f"\n所有图片已保存到: {IMAGE_DIR}")
    print("="*70)
    print("改进蚁群算法（有限电池容量版本）仿真完成!")
    print("="*70)
    
    return plan, results


if __name__ == "__main__":
    # 设置随机种子
    random.seed(42)
    np.random.seed(42)
    plan, results = main()