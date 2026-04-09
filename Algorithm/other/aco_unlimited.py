"""
基于改进蚁群算法的WRSN充电调度优化
参考论文: Wireless sensor network routing optimization based on improved ant colony algorithm
Heliyon 10 (2024) e23577

核心改进（正确实现）：
1. 转移概率引入能量紧急度因子 [公式23] - 优先选择能量低的节点充电
2. 分段启发式函数 [公式24] - 避免远距离节点被过早选择
3. 动态α参数 [公式25] - 初期快速收敛，后期探索新路径
4. 信息素浓度限制 [公式26,28] - 防止过早收敛
5. 路径能量均衡评价 [公式27] - 选择能量分布均衡的路径
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

current_script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_script_dir)
if project_root not in sys.path:
    sys.path.insert(0, project_root)

try:
    from Scenario.uav_scheduling_unlimited import UAVSchedulingUnlimitedEnv
    from Scenario.WSNsimulation import WSNSimulation
except ImportError as e:
    print(f"导入模块失败: {e}")
    sys.exit(1)

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


class ImprovedAntColonyOptimizer:
    """
    改进蚁群算法（论文正确实现）
    
    核心改进：
    1. 转移概率引入能量紧急度因子 [公式23] - 优先选择能量低的节点充电
    2. 分段启发式函数 [公式24] - 根据距离阈值选择不同的启发式
    3. 动态α参数 [公式25] - α(n) = α0 + α1 * e^(-K*n)
    4. 信息素浓度限制 [公式26,28] - τ ∈ [τ_min, τ_max]
    5. 路径能量均衡评价 [公式27] - v_a = (E_avg × E_min) / E_total
            num_ants=60,                        # 适中数量
        max_iterations=400,                 # 足够迭代
        exploration_rate=0.2,              # 高探索率
        stagnation_limit=100,               # 高容忍度
        
        # === 路径选择参数（正确α公式）===
        alpha0=0.7,                         # 最终α
        alpha1=1.8,                         # 初始增量（初始α=2.5）
        K=0.008,                            # 慢衰减
        
        beta=1.5,                           # 距离权重
        gamma=0.5,                          # 能量权重（优先低电量）
        
        # === 信息素更新参数 ===
        rho=0.02,                           # 低挥发率
        Q=200.0,                            # 高信息素强度
        tau_min=0.0005,
        tau_max=15.0,
        elite_ant_ratio=0.25,               # 高精英比例
        
        # === 问题特定参数 ===
        distance_threshold=6000.0,           # 6km阈值
    """
    
    def __init__(self, 
                 network: RoadNetworkWithChargingNodes,
                 charging_node_info: List[Dict],
                 num_ants: int = 60,           # 蚂蚁数量
                 max_iterations: int = 400,     # 最大迭代次数
                 alpha0: float = 0.7,           # 初始α值 (公式25)
                 alpha1: float = 1.8,           # α衰减幅度 (公式25)
                 K: float = 0.008,              # 衰减速率常数 (公式25)
                 beta: float = 1.5,             # 距离重要性因子
                 gamma: float = 0.5,            # 能量紧急度重要性因子
                 rho: float = 0.02,             # 信息素挥发率
                 Q: float = 200.0,              # 信息素强度
                 tau_min: float = 0.0005,        # 最小信息素浓度 (公式26)
                 tau_max: float = 15.0,          # 最大信息素浓度 (公式26)
                 distance_threshold: float = 6000.0,  # 分段距离阈值 d0 (公式24)
                 elite_ant_ratio: float = 0.25,  # 精英蚂蚁比例
                 exploration_rate: float = 0.2,  # 随机探索概率
                 stagnation_limit: int = 100):   # 停滞阈值
        
        self.network = network
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
                    self.node_max_energy[node_id] = 240000.0
                elif node_type == 'relay':
                    self.node_max_energy[node_id] = 800000.0
        
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
                # 论文公式(23)中的E_j(t)应该使用能量紧急度：能量缺口比例
                # 能量越低，紧急度越高，越应该优先充电
                if max_energy > 0:
                    self.energy_urgency[idx] = (max_energy - current) / max_energy
                else:
                    self.energy_urgency[idx] = 0
        
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
        self.best_sequence = None
        self.best_distance = float('inf')
        self.best_energy_metric = -float('inf')
        self.convergence_history = []
        self.distance_history = []
        self.alpha_history = []
        
        # 初始化概率矩阵
        self._update_probabilities(iteration=0)
        
        # print(f"  改进蚁群算法初始化完成")
        # print(f"    蚂蚁数量: {num_ants}")
        # print(f"    最大迭代次数: {max_iterations}")
        # print(f"    动态α参数: α0={alpha0}, α1={alpha1}, K={K}")
        # print(f"    β (距离权重): {beta}")
        # print(f"    γ (能量紧急度权重): {gamma}")
        # print(f"    ρ (挥发率): {rho}")
        # print(f"    信息素范围: [{tau_min}, {tau_max}]")
        # print(f"    距离阈值 d0: {distance_threshold:.2f} m")
        # print(f"    随机探索概率: {exploration_rate}")
    
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
        # 其中U是能量紧急度（能量越低，U越大）
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
    
    def _calculate_energy_metric(self, path_indices: List[int], distance: float) -> float:
        """
        计算路径能量均衡指标 (论文公式27)
        v_a = (E_avg × E_min) / E_total
        
        其中:
        - E_avg: 路径上节点的平均剩余能量
        - E_min: 路径上节点的最小剩余能量
        - E_total: 路径总能耗（用路径距离近似）
        
        该值越大，表示路径能量分布越均衡，越优
        """
        # 提取路径上需要充电的节点（排除充电站）
        charging_indices_in_path = [idx for idx in path_indices 
                                    if idx != self.station_idx]
        
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
    
    def _construct_path(self, iteration: int) -> Tuple[List[int], float]:
        """
        构建一条完整路径（从充电站出发，访问所有节点，返回充电站）
        使用论文公式(23)的转移概率
        """
        unvisited = set(self.charging_indices)
        path = [self.station_idx]
        current = self.station_idx
        total_dist = 0
        
        # 随机探索：一定概率随机选择，避免过早收敛
        if random.random() < self.exploration_rate:
            # 随机顺序访问所有节点
            random_order = list(unvisited)
            random.shuffle(random_order)
            path = [self.station_idx] + random_order + [self.station_idx]
            
            for i in range(len(path) - 1):
                total_dist += self.dist_matrix[path[i], path[i+1]]
            
            return path, total_dist
        
        # 正常选择（使用转移概率）
        while unvisited:
            candidates = list(unvisited)
            
            # 候选节点剪枝：只考虑最近的15个候选节点，提高计算效率
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
            
            dist = self.dist_matrix[current, next_node]
            total_dist += dist
            path.append(next_node)
            unvisited.remove(next_node)
            current = next_node
        
        # 返回充电站
        return_dist = self.dist_matrix[current, self.station_idx]
        total_dist += return_dist
        path.append(self.station_idx)
        
        return path, total_dist
    
    def _update_pheromone(self, all_paths: List[Tuple[List[int], float, float]], iteration: int):
        """
        更新信息素 (论文公式26, 28)
        """
        # 信息素挥发
        self.pheromone *= (1 - self.rho)
        
        # 对所有路径增加信息素
        for path, distance, energy_metric in all_paths:
            # 信息素增量与路径距离成反比，与能量均衡指标成正比
            delta = self.Q / distance * (1 + energy_metric / 1000.0)
            for i in range(len(path) - 1):
                self.pheromone[path[i], path[i+1]] += delta
                self.pheromone[path[i+1], path[i]] += delta
        
        # 精英蚂蚁策略：对最优路径额外增加信息素
        if self.best_sequence is not None and self.best_distance < float('inf'):
            elite_delta = self.elite_ant_ratio * self.Q / self.best_distance
            for i in range(len(self.best_sequence) - 1):
                self.pheromone[self.best_sequence[i], self.best_sequence[i+1]] += elite_delta
                self.pheromone[self.best_sequence[i+1], self.best_sequence[i]] += elite_delta
        
        # 限制信息素浓度
        self._limit_pheromone()
        
        # 更新概率矩阵
        self._update_probabilities(iteration)
    
    def optimize(self, verbose: bool = True) -> Dict:
        """执行改进蚁群优化"""
        print("\n  开始改进蚁群优化...")
        start_time = time.time()
        
        stagnation_counter = 0
        
        for iteration in range(self.max_iterations):
            # 构建所有蚂蚁的路径
            all_paths = []
            iteration_best_distance = float('inf')
            iteration_best_path = None
            iteration_best_energy_metric = -float('inf')
            
            for ant in range(self.num_ants):
                path, distance = self._construct_path(iteration)
                energy_metric = self._calculate_energy_metric(path, distance)
                
                all_paths.append((path, distance, energy_metric))
                
                # 使用能量均衡指标作为主要评价标准
                if energy_metric > iteration_best_energy_metric:
                    iteration_best_energy_metric = energy_metric
                    iteration_best_distance = distance
                    iteration_best_path = path
            
            # 更新全局最优解
            if iteration_best_energy_metric > self.best_energy_metric:
                improvement = iteration_best_energy_metric - self.best_energy_metric
                self.best_energy_metric = iteration_best_energy_metric
                self.best_distance = iteration_best_distance
                self.best_sequence = iteration_best_path
                stagnation_counter = 0
                
                if verbose and improvement > 0:
                    print(f"    迭代 {iteration+1}: 找到更优路径, "
                          f"能量指标={self.best_energy_metric:.4f}, "
                          f"距离={self.best_distance:.2f}m")
            else:
                stagnation_counter += 1
            
            # 更新信息素
            self._update_pheromone(all_paths, iteration)
            
            # 记录收敛历史
            self.convergence_history.append(iteration_best_distance)
            self.distance_history.append(self.best_distance)
            
            # 进度输出
            if verbose and (iteration + 1) % 30 == 0:
                current_alpha = self._get_dynamic_alpha(iteration)
                print(f"    迭代 {iteration+1}/{self.max_iterations}: "
                      f"α={current_alpha:.4f}, "
                      f"最优距离={self.best_distance:.2f}m, "
                      f"能量指标={self.best_energy_metric:.4f}")
            
            # 停滞检测
            if stagnation_counter >= self.stagnation_limit:
                if verbose:
                    print(f"    在迭代 {iteration+1} 检测到停滞，提前终止")
                break
        
        elapsed = time.time() - start_time
        print(f"\n  优化完成: {elapsed:.4f}s")
        print(f"    最优路径距离: {self.best_distance:.2f} m ({self.best_distance/1000:.2f} km)")
        print(f"    最优能量均衡指标: {self.best_energy_metric:.4f}")
        print(f"    总迭代次数: {iteration+1}")
        
        # 构建完整序列
        complete_sequence = [self.idx_to_node[idx] for idx in self.best_sequence]
        charging_sequence = [node for node in complete_sequence if node != self.station_id]
        
        return {
            'complete_sequence': complete_sequence,
            'charging_sequence': charging_sequence,
            'station_id': self.station_id,
            'total_distance': self.best_distance,
            'total_nodes': len(charging_sequence),
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

    def get_full_path_coords(self) -> List[List[Tuple[float, float]]]:
        """
        获取整个规划路径的坐标段列表
        返回: [ [(x1,y1),...], [(x2,y2),...], ... ]
        使用道路网络的实际路径
        """
        if self.best_sequence is None:
            return []
        
        path_coords_list = []
        current = self.station_id
        
        # best_sequence 是索引列表，需要转换为节点ID
        sequence_ids = [self.idx_to_node[idx] for idx in self.best_sequence]
        
        for target_id in sequence_ids[1:]:
            path_coords = self.get_path_coords_between(current, target_id)
            if path_coords:
                path_coords_list.append(path_coords)
            current = target_id
        
        return path_coords_list

    def get_path_coords_between(self, from_node_id: str, to_node_id: str) -> List[Tuple[float, float]]:
        """
        获取两个充电节点之间的完整路径坐标
        使用道路网络的实际路径
        返回: [(x1, y1), (x2, y2), ...]
        """
        if from_node_id == to_node_id:
            node = self.network.get_node_by_id(from_node_id)
            if node:
                return [(node['x'], node['y'])]
            return []
        
        # 获取完整路径的节点ID列表（实际道路路径）
        path_node_ids = self.network.get_path_between_full(from_node_id, to_node_id)
        
        # 转换为坐标
        coords = []
        for node_id in path_node_ids:
            node = self.network.get_node_by_id(node_id)
            if node:
                coords.append((node['x'], node['y']))
        
        return coords

class ACOSimulator:
    """蚁群算法仿真器"""
    
    def __init__(self, wsn_data: Dict[str, Any]):
        self.graph_G = wsn_data['graph_G']
        self.point_info = wsn_data['point_info']
        self.max_min = wsn_data['max_min']
        
        self.env = UAVSchedulingUnlimitedEnv(
            graph_G=self.graph_G,
            point_info=self.point_info,
            max_min=self.max_min,
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

def main():
    print("="*70)
    print("改进蚁群算法 (Improved Ant Colony Optimization) 路径规划仿真")
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
    
    # 3. 改进蚁群算法路径规划
    print("\n[3] 改进蚁群算法路径规划...")
    
    aco = ImprovedAntColonyOptimizer(
        network=network,
        charging_node_info=point_info,
        # === 搜索能力参数 ===
        num_ants=60,                        # 适中数量
        max_iterations=400,                 # 足够迭代
        exploration_rate=0.2,              # 高探索率
        stagnation_limit=100,               # 高容忍度        
        # === 路径选择参数（正确α公式）===
        alpha0=0.7,                         # 最终α
        alpha1=1.8,                         # 初始增量（初始α=2.5）
        K=0.008,                            # 慢衰减
        
        beta=1.5,                           # 距离权重
        gamma=0.5,                          # 能量权重（优先低电量）
        
        # === 信息素更新参数 ===
        rho=0.02,                           # 低挥发率
        Q=200.0,                            # 高信息素强度
        tau_min=0.0005,
        tau_max=15.0,
        elite_ant_ratio=0.25,               # 高精英比例
        
        # === 问题特定参数 ===
        distance_threshold=6000.0,           # 6km阈值
    )
    
    plan = aco.optimize(verbose=True)
    
    # 4. UAV仿真
    print("\n[4] 使用 UAV 仿真计算性能指标...")
    sim_start_time = time.time()
    
    simulator = ACOSimulator(wsn_data)
    results = simulator.run_sequence(plan['complete_sequence'], plan['charging_sequence'])
    
    sim_time = time.time() - sim_start_time
    total_time = time.time() - total_start_time
    
    # 5. 输出结果
    print("\n" + "="*70)
    print("改进蚁群算法仿真结果（性能指标）")
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
    print(f"|")
    print(f"| 优化结果:")
    print(f"|   - 优化耗时: {plan['optimization_time']:.4f} s")
    print(f"|   - 实际迭代次数: {plan['iterations']}")
    print(f"|   - 最优路径距离: {plan['total_distance']:.2f} m ({plan['total_distance']/1000:.2f} km)")
    print(f"|   - 能量均衡指标: {plan['best_energy_metric']:.4f}")
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
    
    print("="*70)
    print("改进蚁群算法仿真完成!")
    print("="*70)
    
    return plan, results


if __name__ == "__main__":
    plan, results = main()