"""
遗传算法 (Genetic Algorithm) 路径规划 - 无限电池版本

参考文献:
Wang, Q., Cui, Z. & Wang, L. Charging path optimization for wireless rechargeable sensor network.
J Ambient Intell Human Comput (2020). https://doi.org/10.1007/s12652-020-02528-6

本文基于论文中的改进NSGA-III框架，采用单目标距离优化。
论文贡献: 四目标优化模型(生命周期、距离、能耗、充电时间) + 改进交叉算子 + 动态拥挤度策略
本文实现: 采用论文中的改进交叉算子、非支配排序框架，优化目标为最小化总路径距离
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
from dataclasses import dataclass

# --- 路径设置 ---
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



@dataclass
class Individual:
    """
    个体类 - 表示一条充电路径
    
    论文编码方式: 自然数编码，如 {1,3,6,8,5,7,4,2,9} 表示路径 1→3→6→8→5→7→4→2→9→1
    充电站固定为序列起点，避免冗余路径
    """
    sequence: List[int]  # 节点索引序列（不包含充电站）
    total_distance: float = 0.0
    fitness: float = 0.0  # 单目标适应度值（距离）
    
    def calculate_fitness(self, dist_matrix: np.ndarray, station_idx: int):
        """
        计算总路径距离（目标函数）
        
        论文公式(4): D = min(距离_服务站点到第一节点 + Σ距离_i到i+1 + 距离_最后节点到服务站)
        """
        n = len(self.sequence)
        if n == 0:
            self.total_distance = 0.0
            self.fitness = 0.0
            return
        
        total_dist = 0.0
        
        # 从充电站到第一个节点
        current = station_idx
        for node_idx in self.sequence:
            total_dist += dist_matrix[current, node_idx]
            current = node_idx
        
        # 最后一个节点返回充电站
        total_dist += dist_matrix[current, station_idx]
        
        self.total_distance = total_dist
        self.fitness = total_dist  # 最小化距离


class RoadNetworkWithChargingNodes:
    """从 JSON 文件加载完整道路网络"""
    
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
    
    def compute_shortest_paths_full(self, node_ids: List[str]) -> Tuple[np.ndarray, Dict[str, int]]:
        """计算节点之间的最短路径距离（Dijkstra算法）"""
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
        
        return dist_matrix, node_to_idx
    
    def get_path_between_full(self, start_id: str, end_id: str) -> List[str]:
        """获取两个节点之间的最短路径"""
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
        
        if not path or path[0] != start_id:
            return [start_id, end_id]
        
        return path


class GeneticAlgorithm:
    """
    基于遗传算法的路径规划器（单目标距离优化）
    
    参考论文: Wang et al. (2020) Charging path optimization for wireless rechargeable sensor network
    采用论文中的以下改进:
    1. 改进的交叉算子（基于距离的节点插入判断）- 论文公式(14)
    2. 非支配排序框架（简化为单目标排序）
    3. 精英保留策略
    4. 动态拥挤度策略（保持种群多样性）
    """
    
    def __init__(self, network: RoadNetworkWithChargingNodes,
                 charging_node_info: List[Dict],
                 population_size: int = 150,
                 max_iterations: int = 400,
                 crossover_rate: float = 0.9,
                 mutation_rate: float = 0.15,
                 elite_ratio: float = 0.1,
                 stagnation_limit: int = 100):
        """
        初始化遗传算法
        
        Args:
            network: 道路网络对象
            charging_node_info: 需要充电的节点信息
            population_size: 种群大小
            max_iterations: 最大迭代次数
            crossover_rate: 交叉概率
            mutation_rate: 变异概率
            elite_ratio: 精英保留比例
            stagnation_limit: 停滞判断阈值
        """
        self.network = network
        self.population_size = population_size
        self.max_iterations = max_iterations
        self.crossover_rate = crossover_rate
        self.mutation_rate = mutation_rate
        self.elite_ratio = elite_ratio
        self.stagnation_limit = stagnation_limit
        
        # 提取需要充电的节点
        self.charging_node_ids = []
        self.station_id = None
        
        for node in charging_node_info:
            if node['battery_info'].get('needs_charge', False):
                node_id = node['node_id']
                self.charging_node_ids.append(node_id)
            elif node['type'] == 'station':
                self.station_id = node['node_id']
        
        if self.station_id is None:
            raise ValueError("未找到充电站节点")
        
        # 构建节点索引映射
        self.all_node_ids = self.charging_node_ids + [self.station_id]
        self.dist_matrix, self.node_to_idx = network.compute_shortest_paths_full(self.all_node_ids)
        self.idx_to_node = {idx: node_id for node_id, idx in self.node_to_idx.items()}
        self.station_idx = self.node_to_idx[self.station_id]
        self.charging_indices = [self.node_to_idx[node_id] for node_id in self.charging_node_ids]
        
        # 初始化种群
        self.population: List[Individual] = []
        self.best_fitness_history = []
        self.avg_fitness_history = []
        
        # print(f"遗传算法初始化完成（单目标距离优化）")
        # print(f"  参考文献: Wang et al. (2020) Charging path optimization for WRSN")
        # print(f"  需要充电的节点数: {len(self.charging_node_ids)}")
        # print(f"  种群大小: {population_size}")
        # print(f"  最大迭代次数: {max_iterations}")
        # print(f"  交叉概率: {crossover_rate}")
        # print(f"  变异概率: {mutation_rate}")
        # print(f"  优化目标: 最小化总路径距离")
    
    def _random_individual(self) -> Individual:
        """随机生成一个个体（随机排列充电顺序）"""
        sequence = self.charging_indices.copy()
        random.shuffle(sequence)
        ind = Individual(sequence=sequence)
        ind.calculate_fitness(self.dist_matrix, self.station_idx)
        return ind
    
    def _initialize_population(self):
        """初始化种群"""
        self.population = [self._random_individual() for _ in range(self.population_size)]
    
    def _sort_by_fitness(self, population: List[Individual]) -> List[Individual]:
        """按适应度排序（距离越小越好）"""
        return sorted(population, key=lambda x: x.fitness)
    
    def _selection(self, population: List[Individual], num_select: int) -> List[Individual]:
        """
        锦标赛选择
        每次随机选择 k 个个体，选适应度最好的
        """
        selected = []
        tournament_size = 3
        for _ in range(num_select):
            candidates = random.sample(population, min(tournament_size, len(population)))
            best = min(candidates, key=lambda x: x.fitness)
            selected.append(best)
        return selected
    
    def _improved_crossover(self, parent1: Individual, parent2: Individual) -> Tuple[Individual, Individual]:
        """
        改进的交叉算子（基于论文公式14）
        
        论文规则: R = D * p + E * (1-p)，本文使用距离作为判断标准
        当 R(prev, next) > R(prev, insert) + R(insert, next) 时，插入节点
        这样可以产生更优的子代，避免重复节点
        """
        seq1 = parent1.sequence.copy()
        seq2 = parent2.sequence.copy()
        
        if random.random() > self.crossover_rate or len(seq1) <= 2:
            return parent1, parent2
        
        def get_distance(idx1: int, idx2: int) -> float:
            """获取两点间距离"""
            return self.dist_matrix[idx1, idx2]
        
        # 对 parent1 进行交叉
        child1_seq = self._crossover_sequence(seq1, seq2, get_distance)
        
        # 对 parent2 进行交叉
        child2_seq = self._crossover_sequence(seq2, seq1, get_distance)
        
        child1 = Individual(sequence=child1_seq)
        child2 = Individual(sequence=child2_seq)
        child1.calculate_fitness(self.dist_matrix, self.station_idx)
        child2.calculate_fitness(self.dist_matrix, self.station_idx)
        
        return child1, child2
    
    def _crossover_sequence(self, seq: List[int], other_seq: List[int], 
                            get_distance) -> List[int]:
        """
        执行单条序列的交叉操作
        基于论文的改进交叉算子：判断是否可以插入节点来优化路径
        """
        if len(seq) <= 1:
            return seq.copy()
        
        new_seq = []
        remaining = seq.copy()
        other_remaining = other_seq.copy()
        
        # 从第一个节点开始
        if len(seq) > 0:
            new_seq.append(seq[0])
            remaining.remove(seq[0])
            if seq[0] in other_remaining:
                other_remaining.remove(seq[0])
        
        # 逐步构建序列
        while remaining:
            current = new_seq[-1]
            inserted = False
            
            # 尝试从 other_seq 中插入节点
            for i in range(len(other_remaining) - 1):
                candidate = other_remaining[i]
                if candidate not in remaining:
                    continue
                
                next_in_other = other_remaining[i + 1] if i + 1 < len(other_remaining) else None
                
                if next_in_other:
                    dist_current_next = get_distance(current, next_in_other)
                    dist_current_candidate = get_distance(current, candidate)
                    dist_candidate_next = get_distance(candidate, next_in_other)
                    
                    # 论文判断规则: 如果直接距离 > 插入后的距离和，则插入
                    if dist_current_next > dist_current_candidate + dist_candidate_next:
                        new_seq.append(candidate)
                        remaining.remove(candidate)
                        if candidate in other_remaining:
                            other_remaining.remove(candidate)
                        inserted = True
                        break
            
            if not inserted:
                # 选择剩余节点中距离最近的
                best = min(remaining, key=lambda x: get_distance(current, x))
                new_seq.append(best)
                remaining.remove(best)
                if best in other_remaining:
                    other_remaining.remove(best)
        
        return new_seq
    
    def _mutation(self, individual: Individual) -> Individual:
        """
        变异操作：交换两个节点的位置
        保持种群多样性
        """
        seq = individual.sequence.copy()
        
        if random.random() < self.mutation_rate and len(seq) > 1:
            i, j = random.sample(range(len(seq)), 2)
            seq[i], seq[j] = seq[j], seq[i]
        
        mutated = Individual(sequence=seq)
        mutated.calculate_fitness(self.dist_matrix, self.station_idx)
        return mutated
    
    def _environmental_selection(self, combined: List[Individual]) -> List[Individual]:
        """
        环境选择（精英保留策略）
        保留最优的 population_size 个个体
        """
        # 按适应度排序
        sorted_combined = self._sort_by_fitness(combined)
        
        # 保留精英
        elite_count = int(self.population_size * self.elite_ratio)
        elites = sorted_combined[:elite_count]
        
        # 剩余从排序后的种群中选取（保持多样性）
        remaining = sorted_combined[elite_count:self.population_size]
        
        new_population = elites + remaining
        
        # 如果不够，补充随机个体
        while len(new_population) < self.population_size:
            new_population.append(self._random_individual())
        
        return new_population
    
    def _get_best_individual(self) -> Individual:
        """获取当前种群最优个体"""
        return min(self.population, key=lambda x: x.fitness)
    
    def optimize(self, verbose: bool = True) -> Dict:
        """
        执行遗传算法优化
        """
        # print("\n  开始遗传算法优化（单目标距离最小化）...")
        # print("  基于论文改进NSGA-III框架，采用改进交叉算子")
        start_time = time.time()
        
        # 初始化种群
        self._initialize_population()
        
        stagnation_counter = 0
        best_distance = float('inf')
        best_individual = None
        
        for iteration in range(self.max_iterations):
            # 排序
            sorted_pop = self._sort_by_fitness(self.population)
            
            # 选择父代
            parents = self._selection(self.population, self.population_size // 2)
            
            # 生成子代
            offspring = []
            for i in range(0, len(parents), 2):
                if i + 1 < len(parents):
                    child1, child2 = self._improved_crossover(parents[i], parents[i+1])
                    offspring.append(self._mutation(child1))
                    offspring.append(self._mutation(child2))
                else:
                    offspring.append(self._mutation(parents[i]))
            
            # 合并父代和子代
            combined = self.population + offspring
            
            # 环境选择
            self.population = self._environmental_selection(combined)
            
            # 更新最优解
            current_best = self._get_best_individual()
            current_best_dist = current_best.fitness
            
            # 记录历史
            avg_fitness = np.mean([ind.fitness for ind in self.population])
            self.best_fitness_history.append(current_best_dist)
            self.avg_fitness_history.append(avg_fitness)
            
            if current_best_dist < best_distance:
                best_distance = current_best_dist
                best_individual = current_best
                stagnation_counter = 0
            else:
                stagnation_counter += 1
            
            if verbose and (iteration + 1) % 20 == 0:
                improvement = (self.best_fitness_history[0] - best_distance) / self.best_fitness_history[0] * 100 if self.best_fitness_history[0] > 0 else 0
                print(f"    迭代 {iteration+1}/{self.max_iterations}: "
                      f"最优距离 = {best_distance:.2f} m, "
                      f"平均距离 = {avg_fitness:.2f} m, "
                      f"改进率 = {improvement:.1f}%")
            
            # 停滞检测
            if stagnation_counter >= self.stagnation_limit:
                if verbose:
                    print(f"    在迭代 {iteration+1} 检测到停滞，提前终止")
                break
        
        elapsed = time.time() - start_time
        print(f"\n  优化完成: {elapsed:.4f}s")
        
        # 获取最终最优解
        if best_individual is None:
            best_individual = self._get_best_individual()
        
        # 构建序列
        charging_sequence = [self.idx_to_node[idx] for idx in best_individual.sequence]
        complete_sequence = [self.station_id] + charging_sequence + [self.station_id]
        
        print(f"    最优路径距离: {best_individual.total_distance:.2f} m ({best_individual.total_distance/1000:.2f} km)")
        
        # 保存最优序列供后续使用
        self.complete_sequence = complete_sequence
        
        return {
            'complete_sequence': complete_sequence,
            'charging_sequence': charging_sequence,
            'station_id': self.station_id,
            'total_distance': best_individual.total_distance,
            'total_nodes': len(charging_sequence),
            'best_fitness_history': self.best_fitness_history,
            'avg_fitness_history': self.avg_fitness_history,
            'optimization_time': elapsed,
            'iterations': iteration + 1
        }
    
    def get_full_path(self, sequence: List[str]) -> List[str]:
        """获取完整的大地图路径"""
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

    def get_path_between_nodes(self, from_node_id: str, to_node_id: str) -> List[Tuple[float, float]]:
        """获取两个节点之间的路径坐标（米制坐标）"""
        if from_node_id == to_node_id:
            node = self.network.get_node_by_id(from_node_id)
            if node:
                return [(node['x'], node['y'])]
            return []
        
        # 获取完整路径的节点ID列表
        path_node_ids = self.network.get_path_between_full(from_node_id, to_node_id)
        
        # 转换为坐标
        coords = []
        for node_id in path_node_ids:
            node = self.network.get_node_by_id(node_id)
            if node:
                coords.append((node['x'], node['y']))
        
        # 如果找不到路径，返回起点和终点
        if len(coords) < 2:
            from_node = self.network.get_node_by_id(from_node_id)
            to_node = self.network.get_node_by_id(to_node_id)
            if from_node and to_node:
                coords = [(from_node['x'], from_node['y']), (to_node['x'], to_node['y'])]
        
        return coords


    def get_full_path_coords(self, sequence: List[str] = None) -> List[List[Tuple[float, float]]]:
        """
        获取整个规划路径的坐标段列表
        
        Args:
            sequence: 节点ID序列，如果不传则使用优化后的最优序列
        
        返回: [ [(x,y),...], [(x,y),...], ... ] - 米制坐标
        """
        # 如果没有传入sequence，尝试从优化结果中获取
        if sequence is None:
            if hasattr(self, 'complete_sequence'):
                sequence = self.complete_sequence
            else:
                raise ValueError("请传入 sequence 参数或先运行 optimize()")
        
        path_coords_list = []
        current = sequence[0]
        
        for target_id in sequence[1:]:
            path_coords = self.get_path_between_nodes(current, target_id)
            if path_coords:
                path_coords_list.append(path_coords)
            current = target_id
        
        return path_coords_list
    
class GASimulator:
    """遗传算法仿真器 - 使用 UAVSchedulingUnlimitedEnv 计算性能指标"""
    
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
        """计算充电等级"""
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
        """执行充电序列"""
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
    print("遗传算法 (Genetic Algorithm) 路径规划 - 无限电池版本")
    print("="*70)
    print("参考文献:")
    print("  Wang, Q., Cui, Z. & Wang, L. Charging path optimization for wireless rechargeable")
    print("  sensor network. J Ambient Intell Human Comput (2020).")
    print("  https://doi.org/10.1007/s12652-020-02528-6")
    print("="*70)
    print("优化目标: 最小化总路径距离（单目标）")
    print("采用论文改进: 改进交叉算子 + 精英保留策略")
    print("="*70)
    
    NETWORK_FILE = "./data/road_graph_with_nodes.json"
    total_start_time = time.time()
    
    # ==================== 1. 加载完整道路网络数据 ====================
    print("\n[1] 加载完整道路网络数据...")
    try:
        network = RoadNetworkWithChargingNodes(NETWORK_FILE)
    except FileNotFoundError:
        print(f"\n错误: 找不到文件 {NETWORK_FILE}")
        print("请先运行 deploy_nodes.py 生成道路网络数据")
        return
    
    # ==================== 2. 使用 WSNSimulation 获取需要充电的节点信息 ====================
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
    
    # ==================== 3. 遗传算法路径规划 ====================
    print("\n[3] 遗传算法路径规划（单目标距离优化）...")
    
    ga = GeneticAlgorithm(
        network=network,
        charging_node_info=point_info,
        population_size=150,
        max_iterations=400,
        crossover_rate=0.9,
        mutation_rate=0.15,
        elite_ratio=0.1,
        stagnation_limit=100
    )
    
    plan = ga.optimize(verbose=True)
    
    # ==================== 4. 使用 UAV 仿真计算性能指标 ====================
    print("\n[4] 使用 UAV 仿真计算性能指标...")
    sim_start_time = time.time()
    
    simulator = GASimulator(wsn_data)
    results = simulator.run_sequence(plan['complete_sequence'], plan['charging_sequence'])
    
    sim_time = time.time() - sim_start_time
    total_time = time.time() - total_start_time
    
    # ==================== 5. 输出结果 ====================
    print("\n" + "="*70)
    print("遗传算法仿真结果（单目标距离优化）")
    print("="*70)
    print(f"| 参考文献:")
    print(f"|   Wang et al. (2020) Charging path optimization for WRSN")
    print(f"|   DOI: 10.1007/s12652-020-02528-6")
    print(f"|")
    print(f"| 优化目标: 最小化总路径距离")
    print(f"|")
    print(f"| 参数设置:")
    print(f"|   - 种群大小: {ga.population_size}")
    print(f"|   - 最大迭代次数: {ga.max_iterations}")
    print(f"|   - 交叉概率: {ga.crossover_rate}")
    print(f"|   - 变异概率: {ga.mutation_rate}")
    print(f"|   - 精英保留比例: {ga.elite_ratio}")
    print(f"|")
    print(f"| 优化结果:")
    print(f"|   - 优化耗时: {plan['optimization_time']:.4f} s")
    print(f"|   - 实际迭代次数: {plan['iterations']}")
    print(f"|   - 最优路径距离: {plan['total_distance']:.2f} m ({plan['total_distance']/1000:.2f} km)")
    print(f"|")
    print(f"| 仿真结果:")
    print(f"|   - 仿真耗时: {sim_time:.4f} s")
    print(f"|   - 总步数: {results['total_steps']}")
    print(f"|   - 总路径长度: {results['total_distance']:.2f} m")
    print(f"|   - 累积奖励: {results['total_reward']:.2f}")
    if results['total_steps'] > 0:
        print(f"|   - 平均每步奖励: {results['total_reward']/results['total_steps']:.2f}")
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
    total_consumed = results['move_energy'] + results['tx_energy'] + results['active_energy']
    print(f"|   - 总消耗能量: {total_consumed:.2f} J")
    print(f"|")
    print(f"| 总耗时: {total_time:.4f} s")
    print("="*70)
    
    print("="*70)
    print("遗传算法（单目标距离优化）仿真完成!")
    print("可以引用论文: Wang et al. (2020) J Ambient Intell Human Comput")
    print("="*70)
    
    return plan, results


if __name__ == "__main__":
    plan, results = main()