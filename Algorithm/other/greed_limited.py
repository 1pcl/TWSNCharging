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
IMAGE_DIR = os.path.join(current_script_dir, "images")
os.makedirs(IMAGE_DIR, exist_ok=True)


class RoadNetworkWithChargingNodes:
    """
    从 deploy_nodes.py 生成的 JSON 文件加载完整道路网络
    用于贪心算法在完整大地图上进行路径规划
    """
    
    def __init__(self, json_file: str = "./data/road_graph_with_nodes.json"):
        start_time = time.time()
        with open(json_file, 'r') as f:
            self.data = json.load(f)
        
        # 节点信息
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
        
        # 构建完整的图邻接表（包含所有节点，用于大地图路径计算）
        self.full_graph = defaultdict(dict)
        for edge in self.data['edges']:
            u = edge['from']
            v = edge['to']
            length = edge['length_m']  # 已经是米
            self.full_graph[u][v] = length
            self.full_graph[v][u] = length
        
        # 统计
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
        """在完整大地图上计算指定节点之间的最短路径距离"""
        start_time = time.time()
        n = len(node_ids)
        node_to_idx = {node_id: i for i, node_id in enumerate(node_ids)}
        dist_matrix = np.full((n, n), float('inf'), dtype=np.float32)
        
        # 对每个节点运行 Dijkstra
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
        
        elapsed = time.time() - start_time
        # print(f"  最短路径计算完成: {elapsed:.4f}s")
        
        return dist_matrix, node_to_idx
    
    def get_path_between_full(self, start_id: str, end_id: str) -> List[str]:
        """在完整大地图上获取两个节点之间的最短路径节点序列"""
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
        
        # 重建路径
        path = []
        current = end_id
        while current is not None:
            path.append(current)
            current = previous[current]
        
        path.reverse()
        
        # 如果找不到路径，返回空列表
        if not path or path[0] != start_id:
            return [start_id, end_id]
        
        return path
    
    def get_node_by_id(self, node_id: str) -> Dict:
        """根据节点ID获取节点信息"""
        if node_id in self.nodes:
            return self.nodes[node_id]
        return None


class GreedyPathPlannerWithBattery:
    """
    基于完整大地图的贪心路径规划器（考虑电池容量限制，包括充电能耗）
    需要多次往返充电站
    """
    
    def __init__(self, network: RoadNetworkWithChargingNodes, charging_node_info: List[Dict],
                 agent_max_battery: float = 16000000.0,
                 agent_move_consumption: float = 1000.0,
                 agent_tx_consumption: float = 100.0,
                 sensor_max_battery: float = 240000.0,
                 gateway_max_battery: float = 800000.0,
                 min_charging_rate: float = 0.7):
        """
        初始化贪心路径规划器
        
        Args:
            network: 完整道路网络对象
            charging_node_info: 从 WSNSimulation 获取的 point_info 数据
            agent_max_battery: 无人车最大电池容量 (J)
            agent_move_consumption: 移动能耗 (J/m)
            agent_tx_consumption: 传输能耗 (J/s)
            sensor_max_battery: 传感器最大电池容量 (J)
            gateway_max_battery: 中继最大电池容量 (J)
        """
        self.network = network
        self.agent_max_battery = agent_max_battery
        self.agent_move_consumption = agent_move_consumption
        self.agent_tx_consumption = agent_tx_consumption
        self.sensor_max_battery = sensor_max_battery
        self.gateway_max_battery = gateway_max_battery
        self.min_charging_rate = min_charging_rate
        
        # 从 WSN 数据中提取需要充电的节点ID和充电站ID
        self.charging_node_ids = []
        self.charging_node_info = {}
        self.station_id = None
        
        for node in charging_node_info:
            if node['battery_info'].get('needs_charge', False):
                # 需要充电的节点
                node_id = node['node_id']
                self.charging_node_ids.append(node_id)
                self.charging_node_info[node_id] = node
            elif node['type'] == 'station':
                # 充电站
                self.station_id = node['node_id']
        
        if self.station_id is None:
            raise ValueError("未找到充电站节点")
        
        # 统计传感器和中继数量
        sensor_count = 0
        relay_count = 0
        for node_id in self.charging_node_ids:
            node_info = network.get_node_by_id(node_id)
            if node_info:
                if node_info['type'] == 'sensor':
                    sensor_count += 1
                elif node_info['type'] == 'relay':
                    relay_count += 1
        
        # print(f"  需要充电的节点数: {len(self.charging_node_ids)}")
        # print(f"  传感器: {sensor_count} 个")
        # print(f"  中继: {relay_count} 个")
        # print(f"  无人车电池容量: {agent_max_battery/1000:.0f} kJ")
        # print(f"  移动能耗: {agent_move_consumption} J/m")
        # print(f"  传输能耗: {agent_tx_consumption} J/s")
        
        # 构建完整的节点列表（包括充电站和所有需要充电的节点）
        self.all_node_ids = self.charging_node_ids + [self.station_id]
        
        # 在完整大地图上计算所有节点之间的最短路径距离
        # print(f"  计算 {len(self.all_node_ids)} 个节点之间的最短路径...")
        self.dist_matrix, self.node_to_idx = network.compute_shortest_paths_full(self.all_node_ids)
        
        # 计算充电时间
        self._calculate_charging_parameters()
        
        # 生成贪心序列（考虑电池容量，分多次往返）
        self.trips = self._greedy_sequence_with_battery()
        self.complete_sequence = self._build_complete_sequence()
    
    def _calculate_charging_parameters(self):
        """计算充电参数"""
        self.tx_rate = 0.70 * self.agent_tx_consumption  # min_charging_rate * agent_tx_consumption
        self.sensor_max_charge = self.sensor_max_battery / self.tx_rate
        self.gateway_max_charge = self.gateway_max_battery / self.tx_rate
        
        self.sensor_charge_time_step = self.sensor_max_charge / 100  # charge_time_levels = 100
        self.gateway_charge_time_step = self.gateway_max_charge / 100
    
    def _get_charging_energy(self, node_id: str) -> float:
        """计算给节点充满电需要的传输能量"""
        node_info = self.network.get_node_by_id(node_id)
        if not node_info:
            return 0
        
        node_type = node_info['type']
        
        # 获取节点的初始能量（从WSN数据中）
        if node_id in self.charging_node_info:
            # current_energy = self.charging_node_info[node_id]['battery_info'].get('current_energy', 0)
            if node_type == 'sensor':
                max_battery = self.sensor_max_battery
            else:
                max_battery = self.gateway_max_battery
            # energy_needed = max_battery - current_energy
            energy_needed = max_battery
            # 充电时间 = 能量需求 / (充电率 * 传输能耗)
            # 充电能量 = 充电时间 * 传输能耗
            # 简化计算：充电能量 = 能量需求 / 充电率
            # charging_rate = self.charging_node_info[node_id]['battery_info'].get('charging_rate', 0.85)
            charging_rate=self.min_charging_rate
            charge_energy = energy_needed / charging_rate
            return max(0, charge_energy)
        
        return 0
    
    def _can_go_and_return(self, start_id: str, target_id: str, remaining_battery: float) -> bool:
        """检查从起点出发到目标节点并返回充电站是否可行（考虑充电能耗）"""
        dist_to_target = self._get_distance(start_id, target_id)
        dist_back_to_station = self._get_distance(target_id, self.station_id)
        
        # 移动能耗
        move_energy = (dist_to_target + dist_back_to_station) * self.agent_move_consumption
        
        # 充电能耗（给目标节点充电）
        charge_energy = self._get_charging_energy(target_id)
        
        total_needed_energy = move_energy + charge_energy
        
        # 预留一些电量作为安全裕度（10%）
        safe_energy = remaining_battery * 0.9
        
        return total_needed_energy <= safe_energy
    
    def _get_distance(self, node1_id: str, node2_id: str) -> float:
        """获取两个节点之间的最短路径距离"""
        idx1 = self.node_to_idx[node1_id]
        idx2 = self.node_to_idx[node2_id]
        return self.dist_matrix[idx1, idx2]
    
    def _greedy_sequence_with_battery(self) -> List[List[str]]:
        """
        考虑电池容量的贪心算法，生成多次往返的充电序列
        返回: [[trip1_nodes], [trip2_nodes], ...] 每个trip是一批可以一次性完成的节点
        """
        if not self.charging_node_ids:
            return []
        
        # 复制未访问节点列表
        unvisited = self.charging_node_ids.copy()
        trips = []
        
        trip_idx = 1
        
        while unvisited:
            # 开始新的一趟行程
            current = self.station_id
            remaining_battery = self.agent_max_battery
            current_trip = []
            trip_distance = 0
            trip_charge_energy = 0
            
            # print(f"  第 {trip_idx} 趟行程:")
            
            # 在当前行程中尽可能多地访问节点
            while unvisited:
                # 找到可以访问且能返回的节点
                feasible_nodes = []
                for node in unvisited:
                    if self._can_go_and_return(current, node, remaining_battery):
                        feasible_nodes.append(node)
                
                if not feasible_nodes:
                    # 没有可行节点，结束当前行程
                    print(f"    无法再访问更多节点，返回充电站")
                    break
                
                # 选择最近的可行节点
                best_node = min(feasible_nodes, key=lambda node: self._get_distance(current, node))
                dist_to_node = self._get_distance(current, best_node)
                charge_energy = self._get_charging_energy(best_node)
                
                # 更新剩余电量
                remaining_battery -= dist_to_node * self.agent_move_consumption
                remaining_battery -= charge_energy
                trip_distance += dist_to_node
                trip_charge_energy += charge_energy
                
                current_trip.append(best_node)
                unvisited.remove(best_node)
                current = best_node
                
                # print(f"    访问节点 {best_node}, 距离: {dist_to_node:.0f}m, 充电能耗: {charge_energy/1000:.0f}kJ, 剩余电量: {remaining_battery/1000:.0f}kJ")
            
            # 返回充电站
            if current != self.station_id:
                dist_back = self._get_distance(current, self.station_id)
                remaining_battery -= dist_back * self.agent_move_consumption
                trip_distance += dist_back
                # print(f"    返回充电站, 距离: {dist_back:.0f}m, 剩余电量: {remaining_battery/1000:.0f}kJ")
            
            trips.append(current_trip)
            # print(f"    本趟完成, 访问 {len(current_trip)} 个节点, 总移动距离: {trip_distance:.0f}m, 总充电能耗: {trip_charge_energy/1000:.0f}kJ")
            trip_idx += 1
        
        return trips
    
    def _build_complete_sequence(self) -> List[str]:
        """构建完整的充电序列（包含充电站）"""
        complete_sequence = []
        
        for i, trip in enumerate(self.trips):
            complete_sequence.extend(trip)
            # 每趟结束后返回充电站（包括最后一趟）
            complete_sequence.append(self.station_id)
        
        return complete_sequence
    
    def get_full_path(self) -> List[str]:
        """在完整大地图上获取完整的路径节点序列（包括返回充电站）"""
        full_path = [self.station_id]
        
        current = self.station_id
        for target_id in self.complete_sequence:
            if target_id == self.station_id:
                # 返回充电站
                path = self.network.get_path_between_full(current, target_id)
                if len(path) > 1:
                    full_path.extend(path[1:])
                current = target_id
            else:
                # 前往充电节点
                path = self.network.get_path_between_full(current, target_id)
                if len(path) > 1:
                    full_path.extend(path[1:])
                current = target_id
        
        return full_path
    
    def get_plan(self) -> Dict:
        """获取规划结果"""
        # 计算总距离
        total_dist = 0
        current = self.station_id
        
        for target_id in self.complete_sequence:
            dist = self._get_distance(current, target_id)
            total_dist += dist
            current = target_id
        
        # 统计所有充电节点
        all_charging_nodes = []
        for trip in self.trips:
            all_charging_nodes.extend(trip)
        
        return {
            'complete_sequence': self.complete_sequence,
            'charging_sequence': all_charging_nodes,
            'trips': self.trips,
            'station_id': self.station_id,
            'total_distance': total_dist,
            'total_nodes': len(all_charging_nodes),
            'num_trips': len(self.trips),
            'full_path': self.get_full_path()
        }


class UAVSimulator:
    """UAV 仿真器 - 使用 UAVSchedulingLimitedEnv 计算性能指标"""
    
    def __init__(self, wsn_data: Dict[str, Any]):
        """
        初始化 UAV 仿真器
        
        Args:
            wsn_data: WSNSimulation.get_data() 返回的数据，包含:
                - graph_G: 充电节点之间的直接连接图（用于环境）
                - point_info: 节点信息列表（包含电池信息）
                - max_min: 归一化所需的最大最小值
        """
        self.graph_G = wsn_data['graph_G']
        self.point_info = wsn_data['point_info']
        self.max_min = wsn_data['max_min']
        
        # 创建环境
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
        
        # 重置环境
        self.env.reset()
        
        # 构建节点ID到全局ID的映射
        self.original_to_global = {}
        self.global_to_original = {}
        for gid, data in self.env.global_to_original_map.items():
            orig_id = data['original_id']
            self.original_to_global[orig_id] = gid
            self.global_to_original[gid] = orig_id
        
        # 预计算充电参数
        self.charge_power = 100.0  # 传输能耗 J/s (agent_tx_consumption)
        self.sensor_max_battery = 240000.0
        self.gateway_max_battery = 800000.0
        
        # 时间步长（基于 min_charging_rate=0.70，用于计算充电等级的分辨率）
        min_charging_rate = 0.70
        tx_rate_min = min_charging_rate * self.charge_power
        self.sensor_charge_time_step = (self.sensor_max_battery / tx_rate_min) / 100
        self.gateway_charge_time_step = (self.gateway_max_battery / tx_rate_min) / 100
    
    def _calculate_charge_level(self, target_id: str) -> int:
        """
        根据节点剩余能量和节点自己的充电效率计算充电等级 (0-99)
        
        充电时间 = 需要电量 / (节点充电效率 × 传输能耗)
        充电等级 = min(99, int(充电时间 / 时间步长))
        
        Args:
            target_id: 节点原始ID
            
        Returns:
            充电等级 (0-99)
        """
        # 查找节点信息
        node_info = None
        for info in self.point_info:
            if info['node_id'] == target_id:
                node_info = info
                break
        
        if node_info is None:
            return 0
        
        node_type = node_info['type']
        current_energy = node_info['battery_info'].get('current_energy', 0)
        # 节点自己的充电效率（根据距离计算得出）
        charging_rate = node_info['battery_info'].get('charging_rate', 0.85)
        
        # 有效充电功率 = 传输能耗 × 节点充电效率
        effective_power = self.charge_power * charging_rate
        
        if node_type == 'sensor':
            max_battery = self.sensor_max_battery
            charge_time_step = self.sensor_charge_time_step
        elif node_type == 'relay':
            max_battery = self.gateway_max_battery
            charge_time_step = self.gateway_charge_time_step
        else:
            return 0
        
        # 需要充的电量
        energy_needed = max_battery - current_energy
        if energy_needed <= 0:
            return 0
        
        # 充电时间 = 需要电量 / 有效充电功率
        charge_time = energy_needed / effective_power if effective_power > 0 else 0
        
        # 充电等级 (0-99)
        charge_level = int(charge_time / charge_time_step)
        charge_level = min(99, max(0, charge_level))
        
        return charge_level
    
    def run_sequence(self, complete_sequence: List[str], charging_sequence: List[str]) -> Dict:
        """
        执行完整的序列（包括返回充电站）
        
        Args:
            complete_sequence: 完整序列，包括充电节点和充电站
            charging_sequence: 仅充电节点序列（用于统计）
        """
        total_reward = 0
        total_distance = 0
        step = 0
        
        for i, target_id in enumerate(complete_sequence):
            # 获取目标节点的全局ID
            if target_id not in self.original_to_global:
                print(f"  警告: 找不到节点 {target_id}")
                continue
            
            target_global_id = self.original_to_global[target_id]
            node_type = self.env.global_to_original_map[target_global_id]["original_type"]
            
            # 计算充电等级
            if node_type == "station":
                charge_level = 0
            else:
                # 根据剩余能量和节点自己的充电效率动态计算充电等级
                charge_level = self._calculate_charge_level(target_id)
            
            # 执行动作
            action = np.array([target_global_id, charge_level])
            obs, reward, terminated, truncated, info = self.env.step(action)
            
            total_reward += reward
            distance = info.get("distance_needed", 0)
            total_distance += distance
            
            step += 1
            
            if terminated:
                break
            
            if truncated:
                print(f"  环境截断")
                break
        
        # 获取最终信息
        final_info = self.env._get_info()
        
        # 计算 EUE
        total_consumed = self.env.tx_energy + self.env.move_energy + self.env.active_energy
        eue = self.env.charg_energy / total_consumed if total_consumed > 0 else 0
        
        return {
            'total_steps': step,
            'total_distance': total_distance,
            'total_reward': total_reward,
            'charged_count': final_info["node_charge_count"],
            'total_nodes': len(charging_sequence),
            'num_trips': final_info.get("return_to_station_count", 0),
            'eue': eue,
            'charge_energy': self.env.charg_energy,
            'move_energy': self.env.move_energy,
            'tx_energy': self.env.tx_energy,
            'active_energy': self.env.active_energy
        }

class MultiVisualizer:
    """多图可视化器"""
    
    def __init__(self, network: RoadNetworkWithChargingNodes):
        self.network = network
    
    def plot_original(self, title="Original Road Network", save_path=None):
        """绘制原始道路网络"""
        fig, ax = plt.subplots(figsize=(14, 12))
        
        # 绘制道路（黑色）
        for edge in self.network.data['edges']:
            from_node = self.network.nodes[edge['from']]
            to_node = self.network.nodes[edge['to']]
            ax.plot([from_node['x'], to_node['x']], [from_node['y'], to_node['y']], 
                   'black', linewidth=1, alpha=0.6, zorder=1)
        
        # 绘制节点
        for node_id, node in self.network.nodes.items():
            x, y = node['x'], node['y']
            node_type = node['type']
            
            if node_type == 'intersection':
                ax.scatter(x, y, c='black', s=20, marker='o', alpha=0.7, zorder=2, edgecolors='gray', linewidth=0.5)
            elif node_type == 'sensor':
                ax.scatter(x, y, c='lightblue', s=25, marker='o', alpha=0.7, zorder=3, edgecolors='deepskyblue', linewidth=0.5)
            elif node_type == 'relay':
                ax.scatter(x, y, c='darkblue', s=120, marker='*', alpha=0.9, zorder=3, edgecolors='navy', linewidth=0.5)
            elif node_type == 'station':
                ax.scatter(x, y, c='green', s=80, marker='s', alpha=0.9, zorder=3, edgecolors='darkgreen', linewidth=1)
        
        # 统计数量
        num_intersections = len([n for n in self.network.nodes.values() if n['type'] == 'intersection'])
        num_sensors = len([n for n in self.network.nodes.values() if n['type'] == 'sensor'])
        num_relays = len([n for n in self.network.nodes.values() if n['type'] == 'relay'])
        num_stations = len([n for n in self.network.nodes.values() if n['type'] == 'station'])
        
        # 图例
        from matplotlib.lines import Line2D
        legend_elements = [
            Line2D([0], [0], color='black', linewidth=1, label='Road'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='black', markersize=8, label=f'Intersection ({num_intersections})'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='lightblue', markeredgecolor='deepskyblue', markersize=8, label=f'Sensor ({num_sensors})'),
            Line2D([0], [0], marker='*', color='w', markerfacecolor='darkblue', markeredgecolor='navy', markersize=10, label=f'Relay ({num_relays})'),
            Line2D([0], [0], marker='s', color='w', markerfacecolor='green', markeredgecolor='darkgreen', markersize=8, label=f'Station ({num_stations})'),
        ]
        
        ax.set_xlabel('X (km)', fontsize=12)
        ax.set_ylabel('Y (km)', fontsize=12)
        
        # 转换坐标显示为公里
        from matplotlib.ticker import FuncFormatter
        ax.xaxis.set_major_formatter(FuncFormatter(lambda x, p: f'{x/1000:.1f}'))
        ax.yaxis.set_major_formatter(FuncFormatter(lambda y, p: f'{y/1000:.1f}'))
        
        ax.set_title(title, fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.2, linestyle='--')
        ax.set_aspect('equal')
        
        plt.subplots_adjust(right=0.85)
        ax.legend(handles=legend_elements, loc='center left', bbox_to_anchor=(1.02, 0.98), fontsize=10, framealpha=0.9)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"  图片已保存: {save_path}")
        plt.show(block=False)
        plt.pause(1)
        plt.close()
        return fig
    
    def plot_charging_nodes(self, charging_node_info: List[Dict], 
                        title="Charging Nodes", save_path=None):
        """绘制需要充电的节点"""
        fig, ax = plt.subplots(figsize=(14, 12))
        
        # 提取需要充电的节点ID
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
        
        # 绘制所有道路（黑色）
        for edge in self.network.data['edges']:
            from_node = self.network.nodes[edge['from']]
            to_node = self.network.nodes[edge['to']]
            ax.plot([from_node['x'], to_node['x']], [from_node['y'], to_node['y']], 
                'black', linewidth=1, alpha=0.5, zorder=1)
        
        # 绘制所有节点
        for node_id, node in self.network.nodes.items():
            x, y = node['x'], node['y']
            node_type = node['type']
            
            if node_type == 'intersection':
                ax.scatter(x, y, c='gray', s=15, marker='o', alpha=0.5, zorder=2)
            elif node_type == 'sensor':
                if node_id in sensor_set:
                    # 需要充电的传感器：红色，大圆点
                    ax.scatter(x, y, c='red', s=80, marker='o', alpha=0.9, zorder=4, 
                            edgecolors='darkred', linewidth=1.5)
                else:
                    # 不需要充电的传感器：浅蓝色，小圆点
                    ax.scatter(x, y, c='lightblue', s=25, marker='o', alpha=0.6, zorder=2, 
                            edgecolors='deepskyblue', linewidth=0.5)
            elif node_type == 'relay':
                if node_id in relay_set:
                    # 需要充电的中继：橙色，大星星
                    ax.scatter(x, y, c='orange', s=150, marker='*', alpha=0.9, zorder=4, 
                            edgecolors='darkorange', linewidth=1.5)
                else:
                    # 不需要充电的中继：深蓝色，小星星
                    ax.scatter(x, y, c='darkblue', s=80, marker='*', alpha=0.6, zorder=2, 
                            edgecolors='navy', linewidth=0.5)
            elif node_type == 'station':
                ax.scatter(x, y, c='green', s=80, marker='s', alpha=0.9, zorder=3, 
                        edgecolors='darkgreen', linewidth=1)
        
        # 图例
        from matplotlib.lines import Line2D
        from matplotlib.patches import Patch
        
        # 创建自定义图例元素
        legend_elements = [
            Line2D([0], [0], color='black', linewidth=1, label='Road'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='gray', 
                markersize=8, label='Intersection'),
            # 传感器图例
            Line2D([0], [0], marker='o', color='w', markerfacecolor='red', 
                markeredgecolor='darkred', markersize=10, label=f'Charging Sensor ({len(sensor_set)})'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='lightblue', 
                markeredgecolor='deepskyblue', markersize=8, label='Normal Sensor'),
            # 中继图例
            Line2D([0], [0], marker='*', color='w', markerfacecolor='orange', 
                markeredgecolor='darkorange', markersize=12, label=f'Charging Relay ({len(relay_set)})'),
            Line2D([0], [0], marker='*', color='w', markerfacecolor='darkblue', 
                markeredgecolor='navy', markersize=10, label='Normal Relay'),
            # 充电站
            Line2D([0], [0], marker='s', color='w', markerfacecolor='green', 
                markeredgecolor='darkgreen', markersize=8, label='Station'),
        ]
        
        ax.set_xlabel('X (km)', fontsize=12)
        ax.set_ylabel('Y (km)', fontsize=12)
        
        from matplotlib.ticker import FuncFormatter
        ax.xaxis.set_major_formatter(FuncFormatter(lambda x, p: f'{x/1000:.1f}'))
        ax.yaxis.set_major_formatter(FuncFormatter(lambda y, p: f'{y/1000:.1f}'))
        
        total_charging = len(sensor_set) + len(relay_set)
        ax.set_title(f"{title} (Total Charging: {total_charging} nodes, Sensor: {len(sensor_set)}, Relay: {len(relay_set)})", 
                    fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.2, linestyle='--')
        ax.set_aspect('equal')
        
        plt.subplots_adjust(right=0.85)
        ax.legend(handles=legend_elements, loc='center left', bbox_to_anchor=(1.02, 0.98), 
                fontsize=10, framealpha=0.9)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"  图片已保存: {save_path}")
        plt.show(block=False)
        plt.pause(1)
        plt.close()
        return fig


    def plot_path(self, path_node_ids: List[str], charging_sequence: List[str], 
                trips: List[List[str]], title="Path Planning Result", save_path=None):
        """绘制路径规划结果"""
        fig, ax = plt.subplots(figsize=(14, 12))
        
        # 绘制所有道路（黑色）
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
        
        # 获取需要充电的节点类型信息（从 charging_sequence 中判断）
        # 由于 charging_sequence 只包含需要充电的节点ID，我们需要从 network 中获取类型
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
                # 如果是需要充电的传感器
                if node_id in charging_node_types:
                    ax.scatter(x, y, c='red', s=35, marker='o', alpha=0.9, zorder=4, 
                            edgecolors='darkred', linewidth=1)
                else:
                    ax.scatter(x, y, c='lightblue', s=25, marker='o', alpha=0.7, zorder=4, 
                            edgecolors='deepskyblue', linewidth=0.5)
            elif node_type == 'relay':
                # 如果是需要充电的中继
                if node_id in charging_node_types:
                    ax.scatter(x, y, c='orange', s=120, marker='*', alpha=0.9, zorder=4, 
                            edgecolors='darkorange', linewidth=1.5)
                else:
                    ax.scatter(x, y, c='darkblue', s=100, marker='*', alpha=0.8, zorder=4, 
                            edgecolors='navy', linewidth=0.5)
            elif node_type == 'station':
                ax.scatter(x, y, c='green', s=80, marker='s', alpha=0.9, zorder=4, 
                        edgecolors='darkgreen', linewidth=1)
        
        # 使用更丰富的颜色方案，动态生成所有轮次的颜色
        import colorsys
        
        # 动态生成颜色：使用HSV色彩空间，确保颜色区分度
        num_trips = len(trips)
        colors = []
        
        if num_trips <= 1:
            colors = ['red']
        elif num_trips <= 7:
            # 预定义颜色方案（区分度好）
            predefined_colors = ['red', 'orange', 'purple', 'brown', 'pink', 'cyan', 'magenta']
            colors = predefined_colors[:num_trips]
        else:
            # 对于更多轮次，使用HSV色彩空间动态生成
            for i in range(num_trips):
                # 使用黄金角（137.5度）在色环上均匀采样，确保颜色区分度
                hue = (i * 137.5) % 360
                saturation = 0.8  # 高饱和度
                value = 0.9       # 高亮度
                rgb = colorsys.hsv_to_rgb(hue / 360, saturation, value)
                colors.append(rgb)
        
        # 为每个轮次分配颜色
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
                
                # 创建标注
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
            # 传感器
            Line2D([0], [0], marker='o', color='w', markerfacecolor='red', markeredgecolor='darkred', 
                markersize=8, label='Charging Sensor'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='lightblue', markeredgecolor='deepskyblue', 
                markersize=8, label='Normal Sensor'),
            # 中继
            Line2D([0], [0], marker='*', color='w', markerfacecolor='orange', markeredgecolor='darkorange', 
                markersize=10, label='Charging Relay'),
            Line2D([0], [0], marker='*', color='w', markerfacecolor='darkblue', markeredgecolor='navy', 
                markersize=10, label='Normal Relay'),
            Line2D([0], [0], marker='s', color='w', markerfacecolor='green', markeredgecolor='darkgreen', 
                markersize=8, label='Station'),
        ]
        
        # 添加轮次图例
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
        plt.show(block=False)
        plt.pause(1)
        plt.close()
        return fig

def main():
    print("="*70)
    print("道路网络贪心路径规划仿真（有限电池容量版本，考虑充电能耗）")
    print("="*70)
    
    # 配置文件路径
    NETWORK_FILE = "./data/road_graph_with_nodes.json"
    OUTPUT_DIR = "./data"
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    
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
    
    # 创建 WSNSimulation 实例，传入道路网络数据
    wsn_sim = WSNSimulation(
        data_input=network.data,
        sensor_battery=240000.0,
        relay_battery=800000.0,
        station_battery=1000000.0,
        seed=42
    )
    
    # 获取 WSN 仿真数据
    wsn_data = wsn_sim.get_data()
    point_info = wsn_data['point_info']  # 只用于知道哪些节点需要充电
    
    # 统计需要充电的节点
    charging_nodes_info = [n for n in point_info if n['battery_info'].get('needs_charge', False)]
    sensors_count = len([n for n in charging_nodes_info if n['type'] == 'sensor'])
    relays_count = len([n for n in charging_nodes_info if n['type'] == 'relay'])
    
    wsn_time = time.time() - start_time
    print(f"  WSN 初始化完成: {wsn_time:.4f}s")
    print(f"  需要充电的传感器: {sensors_count} 个")
    print(f"  需要充电的中继: {relays_count} 个")
    print(f"  充电站: {len([n for n in point_info if n['type'] == 'station'])} 个")
    
    # ==================== 3. 贪心算法基于完整大地图规划路径（考虑电池容量和充电能耗） ====================
    print("\n[3] 贪心算法基于完整大地图规划路径（考虑电池容量和充电能耗）...")
    plan_start_time = time.time()
    
    # 使用完整道路网络和从 WSN 获取的充电节点信息进行贪心规划
    planner = GreedyPathPlannerWithBattery(
        network, 
        point_info,
        agent_max_battery=100000000.0,  
        agent_move_consumption=1000.0,
        agent_tx_consumption=100.0,
        sensor_max_battery=240000.0,
        gateway_max_battery=800000.0
    )
    plan = planner.get_plan()
    
    plan_time = time.time() - plan_start_time
    
    print(f"\n  规划耗时: {plan_time:.4f}s")
    print(f"  需要充电节点数: {plan['total_nodes']}")
    print(f"  总路径长度（含返回充电站）: {plan['total_distance']:.2f} m ({plan['total_distance']/1000:.2f} km)")
    print(f"  行程次数: {plan['num_trips']}")
    
    for i, trip in enumerate(plan['trips']):
        print(f"    第 {i+1} 趟: {len(trip)} 个节点")
        if len(trip) <= 5:
            print(f"      节点: {' -> '.join(trip)}")
        else:
            print(f"      节点: {' -> '.join(trip[:3])} ... -> {trip[-1]}")
    
    # ==================== 4. 使用 UAV 仿真计算性能指标 ====================
    print("\n[4] 使用 UAV 仿真计算性能指标...")
    sim_start_time = time.time()
    
    # 创建仿真器（使用 WSN 生成的 graph_G、point_info 和 max_min）
    simulator = UAVSimulator(wsn_data)
    results = simulator.run_sequence(plan['complete_sequence'], plan['charging_sequence'])
    
    sim_time = time.time() - sim_start_time
    total_time = time.time() - total_start_time
    
    # ==================== 5. 输出结果 ====================
    print("\n" + "="*70)
    print("仿真结果（性能指标）")
    print("="*70)
    print(f"| 规划耗时 (planning_time): {plan_time:.4f} s{'':<43} |")
    print(f"| 仿真耗时 (simulation_time): {sim_time:.4f} s{'':<42} |")
    print(f"| 总耗时 (total_time): {total_time:.4f} s{'':<46} |")
    print(f"| 总步数 (total_steps): {results['total_steps']:<50} |")
    print(f"| 总路径长度 (total_path_length): {results['total_distance']:.2f} m{'':<35} |")
    print(f"| 充电节点数 (charged_node_count): {results['charged_count']:<44} |")
    print(f"| 总节点数 (total_node_count): {results['total_nodes']:<46} |")
    print(f"| 行程次数 (num_trips): {results['num_trips']:<50} |")
    print(f"| 平均能量利用效率 (EUE): {results['eue']:.4f}{'':<43} |")
    print(f"| 累积奖励 (total_reward): {results['total_reward']:.2f}{'':<45} |")
    print(f"| 充电能量 (charge_energy): {results['charge_energy']:.2f} J{'':<43} |")
    print(f"| 移动能量 (move_energy): {results['move_energy']:.2f} J{'':<44} |")
    print(f"| 传输能量 (tx_energy): {results['tx_energy']:.2f} J{'':<44} |")
    print(f"| 激活能量 (active_energy): {results['active_energy']:.2f} J{'':<44} |")
    print("="*70)
    
    # ==================== 6. 生成可视化图片 ====================
    print("\n[5] 生成可视化图片...")
    
    visualizer = MultiVisualizer(network)
    
    # 图1: 原始道路网络
    print("  生成图1: 原始道路网络...")
    visualizer.plot_original(
        title="Original Road Network",
        save_path=os.path.join(IMAGE_DIR, "01_original_road_network_limited.png")
    )
    
    # 图2: 需要充电的节点
    print("  生成图2: 需要充电的节点...")
    visualizer.plot_charging_nodes(
        point_info,
        title=f"Charging Nodes",
        save_path=os.path.join(IMAGE_DIR, "02_charging_nodes_limited.png")
    )
    
    # 图3: 路径规划图
    print("  生成图3: 路径规划图...")
    visualizer.plot_path(
        plan['full_path'], 
        plan['charging_sequence'],
        plan['trips'],
        title=f"Path Planning Result ({plan['num_trips']} trips, Total Dist: {plan['total_distance']/1000:.2f} km)",
        save_path=os.path.join(IMAGE_DIR, "03_path_planning_result_limited.png")
    )
    
    print(f"\n所有图片已保存到: {IMAGE_DIR}")
    print("="*70)
    print("仿真完成!")
    print("="*70)


if __name__ == "__main__":
    main()