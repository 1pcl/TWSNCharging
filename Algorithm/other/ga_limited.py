"""
遗传算法 (Genetic Algorithm) 路径规划 - 有限电池容量版本

参考文献:
Wang, Q., Cui, Z. & Wang, L. Charging path optimization for wireless rechargeable sensor network.
J Ambient Intell Human Comput (2020). https://doi.org/10.1007/s12652-020-02528-6

本文基于论文中的改进NSGA-III框架，采用单目标距离优化，并考虑无人机电池容量限制。
论文贡献: 四目标优化模型(生命周期、距离、能耗、充电时间) + 改进交叉算子 + 动态拥挤度策略
本文实现: 采用论文中的改进交叉算子、非支配排序框架，优化目标为最小化总路径距离，
         同时考虑无人机电池容量限制，支持多次往返充电站
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
    from Scenario.uav_scheduling_limited import UAVSchedulingLimitedEnv
    from Scenario.WSNsimulation import WSNSimulation
except ImportError as e:
    print(f"导入模块失败: {e}")
    sys.exit(1)

# 创建图片保存目录
IMAGE_DIR = os.path.join(current_script_dir, "images_ga_limited")
os.makedirs(IMAGE_DIR, exist_ok=True)


@dataclass
class Individual:
    """
    个体类 - 表示多条充电路径（多次往返）
    
    编码方式: 将充电节点分组，每组表示一趟行程
    例如: [[1,3,5], [2,4,6]] 表示两趟行程
    """
    trips: List[List[int]]  # 多次行程的节点索引列表
    total_distance: float = 0.0
    fitness: float = 0.0  # 单目标适应度值（距离）


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


class GeneticAlgorithmLimited:
    """
    基于遗传算法的路径规划器（有限电池容量版本）
    
    参考论文: Wang et al. (2020) Charging path optimization for wireless rechargeable sensor network
    采用论文中的以下改进:
    1. 改进的交叉算子（基于距离的节点插入判断）
    2. 精英保留策略
    3. 考虑电池容量限制的可行解检查
    """
    
    def __init__(self, network: RoadNetworkWithChargingNodes,
                 charging_node_info: List[Dict],
                 agent_max_battery: float = 100000000.0,
                 agent_move_consumption: float = 1000.0,
                 agent_tx_consumption: float = 100.0,
                 sensor_max_battery: float = 240000.0,
                 gateway_max_battery: float = 800000.0,
                 min_charging_rate: float = 0.7,
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
            agent_max_battery: 无人车最大电池容量 (J)
            agent_move_consumption: 移动能耗 (J/m)
            agent_tx_consumption: 传输能耗 (J/s)
            sensor_max_battery: 传感器最大电池容量 (J)
            gateway_max_battery: 中继最大电池容量 (J)
            min_charging_rate: 最小充电率
            population_size: 种群大小
            max_iterations: 最大迭代次数
            crossover_rate: 交叉概率
            mutation_rate: 变异概率
            elite_ratio: 精英保留比例
            stagnation_limit: 停滞判断阈值
        """
        self.network = network
        self.agent_max_battery = agent_max_battery
        self.agent_move_consumption = agent_move_consumption
        self.agent_tx_consumption = agent_tx_consumption
        self.sensor_max_battery = sensor_max_battery
        self.gateway_max_battery = gateway_max_battery
        self.min_charging_rate = min_charging_rate
        self.population_size = population_size
        self.max_iterations = max_iterations
        self.crossover_rate = crossover_rate
        self.mutation_rate = mutation_rate
        self.elite_ratio = elite_ratio
        self.stagnation_limit = stagnation_limit
        
        # 提取需要充电的节点
        self.charging_node_ids = []
        self.charging_node_info = {}  # 存储节点类型
        self.station_id = None
        
        for node in charging_node_info:
            if node['battery_info'].get('needs_charge', False):
                node_id = node['node_id']
                self.charging_node_ids.append(node_id)
                self.charging_node_info[node_id] = node  # 存储完整信息
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
        
        # 预计算充电能量需求（直接从 charging_node_info 获取）
        self.charge_energy = {}
        for node_id in self.charging_node_ids:
            self.charge_energy[node_id] = self._get_charging_energy(node_id)
        
        # 初始化种群
        self.population: List[Individual] = []
        self.best_fitness_history = []
        self.avg_fitness_history = []
        
        # print(f"遗传算法初始化完成（有限电池容量版本）")
        # print(f"  参考文献: Wang et al. (2020) Charging path optimization for WRSN")
        # print(f"  需要充电的节点数: {len(self.charging_node_ids)}")
        # print(f"  种群大小: {population_size}")
        # print(f"  最大迭代次数: {max_iterations}")
        # print(f"  无人车电池容量: {agent_max_battery/1000:.0f} kJ")
        # print(f"  移动能耗: {agent_move_consumption} J/m")
        # print(f"  传输能耗: {agent_tx_consumption} J/s")
    
    def _get_charging_energy(self, node_id: str) -> float:
        """计算给节点充满电需要的传输能量"""
        node_info = self.charging_node_info.get(node_id)
        if not node_info:
            return 0
        
        node_type = node_info['type']
        
        if node_type == 'sensor':
            max_battery = self.sensor_max_battery
        else:
            max_battery = self.gateway_max_battery
        
        # 充电能量 = 能量需求 / 充电率
        charge_energy = max_battery / self.min_charging_rate
        return max(0, charge_energy)
    
    def _calculate_trip_energy(self, trip: List[int], station_idx: int) -> Tuple[float, float]:
        """
        计算一趟行程所需的总移动能量和充电能量
        
        Returns:
            (total_move_energy, total_charge_energy)
        """
        if not trip:
            return 0, 0
        
        total_move_dist = 0
        total_charge = 0
        
        # 从充电站到第一个节点
        current = station_idx
        for node_idx in trip:
            total_move_dist += self.dist_matrix[current, node_idx]
            node_id = self.idx_to_node[node_idx]
            total_charge += self.charge_energy.get(node_id, 0)
            current = node_idx
        
        # 最后一个节点返回充电站
        total_move_dist += self.dist_matrix[current, station_idx]
        
        total_move_energy = total_move_dist * self.agent_move_consumption
        
        return total_move_energy, total_charge
    
    def _is_feasible_trip(self, trip: List[int], station_idx: int) -> bool:
        """检查一趟行程是否可行（电池容量足够）"""
        if not trip:
            return True
        
        move_energy, charge_energy = self._calculate_trip_energy(trip, station_idx)
        total_energy = move_energy + charge_energy
        
        # 预留10%安全裕度
        return total_energy <= self.agent_max_battery * 0.9
    
    def _is_feasible_individual(self, individual: Individual) -> bool:
        """检查个体是否可行（所有行程都满足电池容量限制）"""
        for trip in individual.trips:
            if not self._is_feasible_trip(trip, self.station_idx):
                return False
        return True
    
    def _calculate_individual_fitness(self, individual: Individual) -> float:
        """计算个体的总距离（目标函数）"""
        total_dist = 0
        
        for trip in individual.trips:
            if not trip:
                continue
            
            current = self.station_idx
            for node_idx in trip:
                total_dist += self.dist_matrix[current, node_idx]
                current = node_idx
            total_dist += self.dist_matrix[current, self.station_idx]
        
        individual.total_distance = total_dist
        individual.fitness = total_dist
        
        return total_dist
    
    def _random_individual(self) -> Individual:
        """随机生成一个个体（将充电节点随机分组）"""
        # 随机排列节点顺序
        shuffled = self.charging_indices.copy()
        random.shuffle(shuffled)
        
        # 贪心分组：依次将节点加入当前行程，直到电池不够
        trips = []
        current_trip = []
        
        for node_idx in shuffled:
            # 尝试将节点加入当前行程
            test_trip = current_trip + [node_idx]
            if self._is_feasible_trip(test_trip, self.station_idx):
                current_trip = test_trip
            else:
                # 电池不够，结束当前行程，开始新行程
                if current_trip:
                    trips.append(current_trip)
                current_trip = [node_idx]
        
        # 添加最后一趟行程
        if current_trip:
            trips.append(current_trip)
        
        # 尝试合并行程
        trips = self._merge_trips(trips)
        
        ind = Individual(trips=trips)
        self._calculate_individual_fitness(ind)
        
        return ind
    
    def _merge_trips(self, trips: List[List[int]]) -> List[List[int]]:
        """尝试合并行程以减少往返次数"""
        if len(trips) <= 1:
            return trips
        
        merged = []
        used = [False] * len(trips)
        
        for i in range(len(trips)):
            if used[i]:
                continue
            
            current_trip = trips[i].copy()
            used[i] = True
            
            # 尝试将其他行程合并到当前行程
            changed = True
            while changed:
                changed = False
                for j in range(len(trips)):
                    if used[j]:
                        continue
                    
                    # 尝试将行程j的所有节点按顺序插入
                    test_trip = current_trip + trips[j]
                    if self._is_feasible_trip(test_trip, self.station_idx):
                        current_trip = test_trip
                        used[j] = True
                        changed = True
            
            merged.append(current_trip)
        
        return merged
    
    def _initialize_population(self):
        """初始化种群"""
        self.population = [self._random_individual() for _ in range(self.population_size)]
        
        # 确保所有个体可行
        for i, ind in enumerate(self.population):
            if not self._is_feasible_individual(ind):
                self.population[i] = self._random_individual()
    
    def _sort_by_fitness(self, population: List[Individual]) -> List[Individual]:
        """按适应度排序（距离越小越好）"""
        return sorted(population, key=lambda x: x.fitness)
    
    def _selection(self, population: List[Individual], num_select: int) -> List[Individual]:
        """锦标赛选择"""
        selected = []
        tournament_size = 3
        for _ in range(num_select):
            candidates = random.sample(population, min(tournament_size, len(population)))
            best = min(candidates, key=lambda x: x.fitness)
            selected.append(best)
        return selected
    
    def _crossover(self, parent1: Individual, parent2: Individual) -> Tuple[Individual, Individual]:
        """
        交叉操作：合并两个父代的节点，然后重新排序和分组
        """
        if random.random() > self.crossover_rate:
            return parent1, parent2
        
        # 合并两个父代的所有节点
        all_nodes = []
        for trip in parent1.trips:
            all_nodes.extend(trip)
        for trip in parent2.trips:
            all_nodes.extend(trip)
        
        # 去重（每个节点只出现一次）
        all_nodes = list(dict.fromkeys(all_nodes))
        
        # 重新排序：使用贪心策略，从充电站出发，每次选最近节点
        new_order = []
        remaining = all_nodes.copy()
        current = self.station_idx
        
        while remaining:
            # 选择距离最近的节点
            best = min(remaining, key=lambda x: self.dist_matrix[current, x])
            new_order.append(best)
            remaining.remove(best)
            current = best
        
        # 贪心分组
        child_trips = []
        current_trip = []
        
        for node_idx in new_order:
            test_trip = current_trip + [node_idx]
            if self._is_feasible_trip(test_trip, self.station_idx):
                current_trip = test_trip
            else:
                if current_trip:
                    child_trips.append(current_trip)
                current_trip = [node_idx]
        
        if current_trip:
            child_trips.append(current_trip)
        
        child1 = Individual(trips=child_trips)
        child2 = Individual(trips=child_trips.copy())  # 第二个子代相同，后续变异会区分
        
        self._calculate_individual_fitness(child1)
        self._calculate_individual_fitness(child2)
        
        return child1, child2
    
    def _mutation(self, individual: Individual) -> Individual:
        """
        变异操作：随机交换两个节点的位置或移动节点到不同行程
        """
        if random.random() > self.mutation_rate:
            return individual
        
        # 收集所有节点
        all_nodes = []
        for trip in individual.trips:
            all_nodes.extend(trip)
        
        if len(all_nodes) < 2:
            return individual
        
        # 选择变异方式
        mutation_type = random.choice(['swap', 'move'])
        
        if mutation_type == 'swap' and len(all_nodes) >= 2:
            # 交换两个节点的位置
            i, j = random.sample(range(len(all_nodes)), 2)
            all_nodes[i], all_nodes[j] = all_nodes[j], all_nodes[i]
        elif mutation_type == 'move' and len(all_nodes) >= 2:
            # 移动一个节点到新位置
            i = random.randint(0, len(all_nodes) - 1)
            node = all_nodes.pop(i)
            j = random.randint(0, len(all_nodes))
            all_nodes.insert(j, node)
        
        # 重新分组
        new_trips = []
        current_trip = []
        
        for node_idx in all_nodes:
            test_trip = current_trip + [node_idx]
            if self._is_feasible_trip(test_trip, self.station_idx):
                current_trip = test_trip
            else:
                if current_trip:
                    new_trips.append(current_trip)
                current_trip = [node_idx]
        
        if current_trip:
            new_trips.append(current_trip)
        
        mutated = Individual(trips=new_trips)
        self._calculate_individual_fitness(mutated)
        
        return mutated
    
    def _environmental_selection(self, combined: List[Individual]) -> List[Individual]:
        """环境选择（精英保留策略）"""
        # 按适应度排序
        sorted_combined = self._sort_by_fitness(combined)
        
        # 保留精英
        elite_count = int(self.population_size * self.elite_ratio)
        elites = sorted_combined[:elite_count]
        
        # 剩余从排序后的种群中选取
        new_population = elites + sorted_combined[elite_count:self.population_size]
        
        # 如果不够，补充随机个体
        while len(new_population) < self.population_size:
            new_population.append(self._random_individual())
        
        return new_population
    
    def _get_best_individual(self) -> Individual:
        """获取当前种群最优个体"""
        return min(self.population, key=lambda x: x.fitness)
    
    def optimize(self, verbose: bool = True) -> Dict:
        """执行遗传算法优化"""
        # print("\n  开始遗传算法优化（有限电池容量版本）...")
        start_time = time.time()
        
        # 初始化种群
        self._initialize_population()
        
        stagnation_counter = 0
        best_distance = float('inf')
        best_individual = None
        
        for iteration in range(self.max_iterations):
            # 选择父代
            parents = self._selection(self.population, self.population_size // 2)
            
            # 生成子代
            offspring = []
            for i in range(0, len(parents), 2):
                if i + 1 < len(parents):
                    child1, child2 = self._crossover(parents[i], parents[i+1])
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
                print(f"    迭代 {iteration+1}/{self.max_iterations}: "
                      f"最优距离 = {best_distance:.2f} m, "
                      f"平均距离 = {avg_fitness:.2f} m, "
                      f"行程数 = {len(best_individual.trips)}")
            
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
        charging_sequence = []
        complete_sequence = [self.station_id]
        
        for trip in best_individual.trips:
            for node_idx in trip:
                node_id = self.idx_to_node[node_idx]
                charging_sequence.append(node_id)
                complete_sequence.append(node_id)
            complete_sequence.append(self.station_id)
        
        print(f"    最优路径距离: {best_individual.total_distance:.2f} m ({best_individual.total_distance/1000:.2f} km)")
        print(f"    行程次数: {len(best_individual.trips)}")
        
        return {
            'complete_sequence': complete_sequence,
            'charging_sequence': charging_sequence,
            'trips': [[self.idx_to_node[idx] for idx in trip] for trip in best_individual.trips],
            'station_id': self.station_id,
            'total_distance': best_individual.total_distance,
            'total_nodes': len(charging_sequence),
            'num_trips': len(best_individual.trips),
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


class GASimulatorLimited:
    """遗传算法仿真器 - 使用 UAVSchedulingLimitedEnv 计算性能指标"""
    
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


class GAVisualizerLimited:
    """遗传算法结果可视化器（有限电池版本）"""
    
    def __init__(self, network: RoadNetworkWithChargingNodes):
        self.network = network
    
    def plot_original(self, title="Original Road Network", save_path=None):
        """绘制原始道路网络"""
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
        """绘制需要充电的节点"""
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
                  trips: List[List[str]], title="GA Path Planning Result", save_path=None):
        """绘制路径规划结果（支持多次往返）"""
        fig, ax = plt.subplots(figsize=(14, 12))
        
        # 绘制道路
        for edge in self.network.data['edges']:
            from_node = self.network.nodes[edge['from']]
            to_node = self.network.nodes[edge['to']]
            ax.plot([from_node['x'], to_node['x']], [from_node['y'], to_node['y']], 
                'black', linewidth=1, alpha=0.5, zorder=1)
        
        # 绘制路径
        if len(path_node_ids) > 1:
            path_x = []
            path_y = []
            for node_id in path_node_ids:
                if node_id in self.network.nodes:
                    x, y = self.network.nodes[node_id]['x'], self.network.nodes[node_id]['y']
                    path_x.append(x)
                    path_y.append(y)
            
            ax.plot(path_x, path_y, 'green', linewidth=3, alpha=0.9, zorder=3, label='Travel Path')
        
        # 获取需要充电的节点
        charging_node_set = set(charging_sequence)
        
        # 绘制节点
        for node_id, node in self.network.nodes.items():
            x, y = node['x'], node['y']
            node_type = node['type']
            
            if node_type == 'intersection':
                ax.scatter(x, y, c='black', s=15, marker='o', alpha=0.5, zorder=2)
            elif node_type == 'sensor':
                if node_id in charging_node_set:
                    ax.scatter(x, y, c='red', s=35, marker='o', alpha=0.9, zorder=4)
                else:
                    ax.scatter(x, y, c='lightblue', s=25, marker='o', alpha=0.7, zorder=4)
            elif node_type == 'relay':
                if node_id in charging_node_set:
                    ax.scatter(x, y, c='orange', s=120, marker='*', alpha=0.9, zorder=4)
                else:
                    ax.scatter(x, y, c='darkblue', s=100, marker='*', alpha=0.8, zorder=4)
            elif node_type == 'station':
                ax.scatter(x, y, c='green', s=80, marker='s', alpha=0.9, zorder=4)
        
        # 生成颜色
        import colorsys
        num_trips = len(trips)
        colors = []
        if num_trips <= 1:
            colors = ['red']
        elif num_trips <= 7:
            predefined_colors = ['red', 'orange', 'purple', 'brown', 'pink', 'cyan', 'magenta']
            colors = predefined_colors[:num_trips]
        else:
            for i in range(num_trips):
                hue = (i * 137.5) % 360
                rgb = colorsys.hsv_to_rgb(hue / 360, 0.8, 0.9)
                colors.append(rgb)
        
        # 为每趟行程的节点标注不同颜色
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
        
        # 标注充电站
        for node_id, node in self.network.nodes.items():
            if node['type'] == 'station':
                x, y = node['x'], node['y']
                ax.annotate('START/END', (x, y), fontsize=9, ha='center', va='bottom',
                        bbox=dict(boxstyle="round,pad=0.2", facecolor="yellow", alpha=0.8))
        
        # 图例
        from matplotlib.lines import Line2D
        from matplotlib.patches import Patch
        
        legend_elements = [
            Line2D([0], [0], color='green', linewidth=2, label='Travel Path'),
            Line2D([0], [0], color='black', linewidth=1, label='Road'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='black', markersize=8, label='Intersection'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='red', markersize=8, label='Charging Sensor'),
            Line2D([0], [0], marker='*', color='w', markerfacecolor='orange', markersize=10, label='Charging Relay'),
            Line2D([0], [0], marker='s', color='w', markerfacecolor='green', markersize=8, label='Station'),
        ]
        
        for trip_idx, trip in enumerate(trips):
            color = colors[trip_idx]
            legend_elements.append(
                Patch(facecolor=color, edgecolor=color, alpha=0.7, 
                    label=f'Trip {trip_idx+1} ({len(trip)} nodes)')
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
        ax.legend(handles=legend_elements, loc='center left', bbox_to_anchor=(1.02, 0.98), 
                fontsize=9, framealpha=0.9, ncol=2 if num_trips > 8 else 1)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"  图片已保存: {save_path}")
        plt.close()
        return fig
    
    def plot_convergence(self, best_fitness_history: List[float], avg_fitness_history: List[float],
                         title="GA Convergence Curve", save_path=None):
        """绘制收敛曲线"""
        fig, ax = plt.subplots(figsize=(10, 6))
        
        iterations = range(1, len(best_fitness_history) + 1)
        
        ax.plot(iterations, best_fitness_history, 'b-', linewidth=2, label='Best Distance')
        ax.plot(iterations, avg_fitness_history, 'r--', linewidth=2, label='Average Distance')
        
        ax.set_xlabel('Iteration', fontsize=12)
        ax.set_ylabel('Path Distance (m)', fontsize=12)
        ax.set_title(title, fontsize=14, fontweight='bold')
        ax.legend(fontsize=10)
        ax.grid(True, alpha=0.3, linestyle='--')
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"  图片已保存: {save_path}")
        plt.close()
        return fig
    
    def plot_rewards(self, step_rewards: List[float], title="Step Rewards", save_path=None):
        """绘制每一步的奖励"""
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
    print("遗传算法 (Genetic Algorithm) 路径规划 - 有限电池容量版本")
    print("="*70)
    print("参考文献:")
    print("  Wang, Q., Cui, Z. & Wang, L. Charging path optimization for wireless rechargeable")
    print("  sensor network. J Ambient Intell Human Comput (2020).")
    print("  https://doi.org/10.1007/s12652-020-02528-6")
    print("="*70)
    print("优化目标: 最小化总路径距离（单目标）")
    print("考虑约束: 无人机电池容量限制")
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
    print("\n[3] 遗传算法路径规划（有限电池容量版本）...")
    
    ga = GeneticAlgorithmLimited(
        network=network,
        charging_node_info=point_info,
        agent_max_battery=100000000.0,
        agent_move_consumption=1000.0,
        agent_tx_consumption=100.0,
        sensor_max_battery=240000.0,
        gateway_max_battery=800000.0,
        min_charging_rate=0.7,
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
    
    simulator = GASimulatorLimited(wsn_data)
    results = simulator.run_sequence(plan['complete_sequence'], plan['charging_sequence'])
    
    sim_time = time.time() - sim_start_time
    total_time = time.time() - total_start_time
    
    # ==================== 5. 输出结果 ====================
    print("\n" + "="*70)
    print("遗传算法仿真结果（有限电池容量版本）")
    print("="*70)
    print(f"| 参考文献:")
    print(f"|   Wang et al. (2020) Charging path optimization for WRSN")
    print(f"|   DOI: 10.1007/s12652-020-02528-6")
    print(f"|")
    print(f"| 优化目标: 最小化总路径距离")
    print(f"| 约束条件: 无人机电池容量限制")
    print(f"|")
    print(f"| 参数设置:")
    print(f"|   - 种群大小: {ga.population_size}")
    print(f"|   - 最大迭代次数: {ga.max_iterations}")
    print(f"|   - 交叉概率: {ga.crossover_rate}")
    print(f"|   - 变异概率: {ga.mutation_rate}")
    print(f"|   - 精英保留比例: {ga.elite_ratio}")
    print(f"|   - 无人车电池容量: {ga.agent_max_battery/1000:.0f} kJ")
    print(f"|")
    print(f"| 优化结果:")
    print(f"|   - 优化耗时: {plan['optimization_time']:.4f} s")
    print(f"|   - 实际迭代次数: {plan['iterations']}")
    print(f"|   - 最优路径距离: {plan['total_distance']:.2f} m ({plan['total_distance']/1000:.2f} km)")
    print(f"|   - 行程次数: {plan['num_trips']}")
    print(f"|")
    print(f"| 仿真结果:")
    print(f"|   - 仿真耗时: {sim_time:.4f} s")
    print(f"|   - 总步数: {results['total_steps']}")
    print(f"|   - 总路径长度: {results['total_distance']:.2f} m")
    print(f"|   - 累积奖励: {results['total_reward']:.2f}")
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
    print(f"|")
    print(f"| 总耗时: {total_time:.4f} s")
    print("="*70)
    
    # ==================== 6. 生成可视化图片 ====================
    print("\n[5] 生成可视化图片...")
    
    visualizer = GAVisualizerLimited(network)
    
    print("  生成图1: 原始道路网络...")
    visualizer.plot_original(
        title="Original Road Network (GA Limited - Distance Optimization)",
        save_path=os.path.join(IMAGE_DIR, "01_original_road_network_ga_limited.png")
    )
    
    print("  生成图2: 需要充电的节点...")
    visualizer.plot_charging_nodes(
        point_info,
        title=f"Charging Nodes (Total: {plan['total_nodes']} nodes)",
        save_path=os.path.join(IMAGE_DIR, "02_charging_nodes_ga_limited.png")
    )
    
    print("  生成图3: 路径规划图...")
    full_path = ga.get_full_path(plan['complete_sequence'])
    visualizer.plot_path(
        full_path,
        plan['charging_sequence'],
        plan['trips'],
        title=f"GA Path Planning Result - Distance: {plan['total_distance']/1000:.2f} km, Trips: {plan['num_trips']}",
        save_path=os.path.join(IMAGE_DIR, "03_path_planning_result_ga_limited.png")
    )
    
    print("  生成图4: 收敛曲线...")
    visualizer.plot_convergence(
        plan['best_fitness_history'],
        plan['avg_fitness_history'],
        title="GA Convergence Curve (Limited Battery)",
        save_path=os.path.join(IMAGE_DIR, "04_convergence_curve_ga_limited.png")
    )
    
    print("  生成图5: 步骤奖励分布...")
    visualizer.plot_rewards(
        results['step_rewards'],
        title=f"Step Rewards (Total: {results['total_reward']:.2f})",
        save_path=os.path.join(IMAGE_DIR, "05_step_rewards_ga_limited.png")
    )
    
    print(f"\n所有图片已保存到: {IMAGE_DIR}")
    print("="*70)
    print("遗传算法（有限电池容量版本）仿真完成!")
    print("="*70)
    
    return plan, results


if __name__ == "__main__":
    plan, results = main()