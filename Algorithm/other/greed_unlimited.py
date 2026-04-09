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
    from Scenario.uav_scheduling_unlimited import UAVSchedulingUnlimitedEnv
    from Scenario.WSNsimulation import WSNSimulation
except ImportError as e:
    print(f"导入模块失败: {e}")
    sys.exit(1)


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
        # print(f"  中继节点数: {self.statistics['relays']}")
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


class GreedyPathPlanner:
    def __init__(self, network: RoadNetworkWithChargingNodes, charging_node_info: List[Dict]):
        """
        初始化贪心路径规划器
        
        Args:
            network: 完整道路网络对象（可以是 RoadNetworkWithChargingNodes 或 DynamicRoadNetwork）
            charging_node_info: 从 WSNSimulation 获取的 point_info 数据
        """
        self.network = network
        
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
            # 尝试从 charging_node_info 中查找充电站
            for node in charging_node_info:
                if node.get('type') == 'station':
                    self.station_id = node['node_id']
                    break
        
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
        
        # 构建完整的节点列表（包括充电站和所有需要充电的节点）
        self.all_node_ids = self.charging_node_ids + [self.station_id]
        
        # 在完整大地图上计算所有节点之间的最短路径距离
        if hasattr(network, 'compute_shortest_paths_full'):
            self.dist_matrix, self.node_to_idx = network.compute_shortest_paths_full(self.all_node_ids)
        else:
            # 如果网络对象没有这个方法，尝试使用内置方法
            self.dist_matrix, self.node_to_idx = self._compute_dist_matrix(self.all_node_ids)
        
        # 生成贪心序列（不考虑电池容量，一次遍历所有节点）
        self.complete_sequence = self._greedy_sequence()
    
    def _compute_dist_matrix(self, node_ids: List[str]):
        """备用方法：计算节点间距离矩阵"""
        import heapq as hq
        
        n = len(node_ids)
        node_to_idx = {node_id: i for i, node_id in enumerate(node_ids)}
        dist_matrix = np.full((n, n), float('inf'), dtype=np.float32)
        
        for i, start_id in enumerate(node_ids):
            dist_matrix[i, i] = 0.0
            
            distances = {}
            for node in self.network.nodes:
                distances[node] = float('inf')
            distances[start_id] = 0
            pq = [(0, start_id)]
            visited = set()
            
            while pq:
                current_dist, current = hq.heappop(pq)
                if current in visited:
                    continue
                visited.add(current)
                
                if current not in self.network.full_graph:
                    continue
                    
                for neighbor, length in self.network.full_graph[current].items():
                    new_dist = current_dist + length
                    if new_dist < distances.get(neighbor, float('inf')):
                        distances[neighbor] = new_dist
                        hq.heappush(pq, (new_dist, neighbor))
            
            for j, end_id in enumerate(node_ids):
                if end_id in distances:
                    dist_matrix[i, j] = distances[end_id]
        
        return dist_matrix, node_to_idx
    
    def _greedy_sequence(self) -> List[str]:
        """
        贪心算法，每次选择距离当前节点最近的未充电节点
        不考虑电池容量，一次遍历所有节点
        
        返回: [station, node1, node2, ..., nodeN, station] 完整序列
        """
        if not self.charging_node_ids:
            return [self.station_id]
        
        # 复制未访问节点列表
        unvisited = set(self.charging_node_ids)
        sequence = []
        current_node = self.station_id
        
        total_distance = 0
        
        while unvisited:
            # 找到距离当前节点最近的未充电节点
            best_node = min(unvisited, key=lambda node: self._get_distance(current_node, node))
            dist_to_node = self._get_distance(current_node, best_node)
            
            sequence.append(best_node)
            unvisited.remove(best_node)
            current_node = best_node
            total_distance += dist_to_node
        
        # 最后返回充电站
        dist_back = self._get_distance(current_node, self.station_id)
        total_distance += dist_back
        
        # 构建完整序列：从充电站出发，遍历所有节点，最后返回充电站
        complete_sequence = [self.station_id] + sequence + [self.station_id]
        
        return complete_sequence
    
    def _get_distance(self, node1_id: str, node2_id: str) -> float:
        """获取两个节点之间的最短路径距离"""
        idx1 = self.node_to_idx[node1_id]
        idx2 = self.node_to_idx[node2_id]
        return self.dist_matrix[idx1, idx2]
    
    def get_plan(self) -> Dict:
        """获取规划结果"""
        # 计算总距离
        total_dist = 0
        current = self.station_id
        
        for target_id in self.complete_sequence[1:]:  # 跳过起点
            dist = self._get_distance(current, target_id)
            total_dist += dist
            current = target_id
        
        # 充电序列（不包括充电站）
        charging_sequence = [node for node in self.complete_sequence 
                             if node != self.station_id]
        
        return {
            'complete_sequence': self.complete_sequence,
            'charging_sequence': charging_sequence,
            'station_id': self.station_id,
            'total_distance': total_dist,
            'total_nodes': len(charging_sequence),
            'full_path': self.get_full_path()
        }
    
    def get_full_path(self) -> List[str]:
        """在完整大地图上获取完整的路径节点序列（包括返回充电站）"""
        full_path = []
        
        current = self.station_id
        for target_id in self.complete_sequence[1:]:  # 从第一个目标开始
            path = self.network.get_path_between_full(current, target_id)
            if len(path) > 1:
                full_path.extend(path[1:])  # 跳过起点避免重复
            else:
                full_path.extend(path)
            current = target_id
        
        # 加上起点
        full_path = [self.station_id] + full_path
        
        return full_path

    def get_path_between_nodes(self, from_node_id: str, to_node_id: str) -> List[Tuple[float, float]]:
        """
        获取两个充电节点之间的完整路径坐标
        返回: [(x1, y1), (x2, y2), ...]  - 米制坐标（单位：米）
        """
        if from_node_id == to_node_id:
            node = self.network.get_node_by_id(from_node_id)
            if node:
                return [(node['x'], node['y'])]
            return []
        
        # 获取完整路径的节点ID列表
        path_node_ids = self.network.get_path_between_full(from_node_id, to_node_id)
        
        # 转换为坐标（米制）
        coords = []
        for node_id in path_node_ids:
            node = self.network.get_node_by_id(node_id)
            if node:
                coords.append((node['x'], node['y']))
            else:
                # 如果节点不在网络中，尝试从 charging_node_info 中获取
                if node_id in self.charging_node_info:
                    node_info = self.charging_node_info[node_id]
                    coords.append((node_info.get('x', 0), node_info.get('y', 0)))
        
        # 如果仍然没有找到路径，至少返回起点和终点
        if len(coords) < 2:
            from_node = self.network.get_node_by_id(from_node_id)
            to_node = self.network.get_node_by_id(to_node_id)
            if from_node and to_node:
                coords = [(from_node['x'], from_node['y']), (to_node['x'], to_node['y'])]
        
        return coords


    def get_full_path_coords(self) -> List[List[Tuple[float, float]]]:
        """
        获取整个规划路径的坐标段列表
        返回: [ [(x,y),...], [(x,y),...], ... ] - 米制坐标（单位：米）
        """
        path_coords_list = []
        current = self.station_id
        
        for target_id in self.complete_sequence[1:]:
            path_coords = self.get_path_between_nodes(current, target_id)
            if path_coords:
                path_coords_list.append(path_coords)
            current = target_id
        
        return path_coords_list

class UAVSimulator:
    """UAV 仿真器 - 使用 UAVSchedulingUnlimitedEnv 计算性能指标"""
    
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
        
        # ========== 新增：加载道路网络用于路径计算 ==========
        # 加载完整的道路网络数据
        network_file = os.path.join(os.path.dirname(os.path.dirname(__file__)), "data", "road_graph_with_nodes.json")
        self.road_network = RoadNetworkWithChargingNodes(network_file)
        
        # 预计算充电参数
        self.charge_power = 100.0  # 传输能耗 J/s (agent_tx_consumption)
        
        # 传感器和中继的最大电池容量
        self.sensor_max_battery = 240000.0
        self.gateway_max_battery = 800000.0
        
        # 时间步长（基于 min_charging_rate=0.70，但实际充电时会用节点自己的 charging_rate）
        min_charging_rate = 0.70
        tx_rate_min = min_charging_rate * self.charge_power
        self.sensor_charge_time_step = (self.sensor_max_battery / tx_rate_min) / 100
        self.gateway_charge_time_step = (self.gateway_max_battery / tx_rate_min) / 100
    
    def _calculate_charge_level(self, node_id: str) -> int:
        """
        根据节点剩余能量和节点自己的充电效率计算充电等级 (0-99)
        
        充电时间 = 需要电量 / (节点充电效率 × 传输能耗)
        充电等级 = min(99, int(充电时间 / 时间步长))
        """
        # 查找节点信息
        node_info = None
        for info in self.point_info:
            if info['node_id'] == node_id:
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
    
    def _get_path_between(self, from_node_id: str, to_node_id: str) -> List[Tuple[float, float]]:
        """
        获取两个节点之间的路径坐标（经纬度）
        使用道路网络获取真实的最短路径
        返回路径上的点列表，每个点是 (lng, lat)
        """
        if from_node_id == to_node_id:
            node = self.road_network.get_node_by_id(from_node_id)
            if node:
                return [(node['x'], node['y'])]
            return []
        
        # 使用道路网络获取最短路径的节点ID序列
        path_node_ids = self.road_network.get_path_between_full(from_node_id, to_node_id)
        
        if not path_node_ids:
            # 如果找不到路径，直接返回起点和终点
            from_node = self.road_network.get_node_by_id(from_node_id)
            to_node = self.road_network.get_node_by_id(to_node_id)
            if from_node and to_node:
                return [(from_node['x'], from_node['y']), (to_node['x'], to_node['y'])]
            return []
        
        # 将节点ID序列转换为坐标序列
        coords = []
        for node_id in path_node_ids:
            node = self.road_network.get_node_by_id(node_id)
            if node:
                coords.append((node['x'], node['y']))
        
        return coords
    
    def _get_node_coord(self, node_id: str) -> Tuple[float, float]:
        """获取节点坐标"""
        node = self.road_network.get_node_by_id(node_id)
        if node:
            return (node['x'], node['y'])
        return (0, 0)
    
    def run_sequence(self, complete_sequence: List[str], charging_sequence: List[str]) -> Dict:
        total_reward = 0
        total_distance = 0
        step = 0
        return_count = 0
        action_log = []  # 添加 action_log 记录每一步
        
        for i, target_id in enumerate(complete_sequence):
            is_station = (target_id == self.station_original_id)
            
            if i == 0 and is_station:
                continue
            
            if target_id not in self.original_to_global:
                print(f"  警告: 找不到节点 {target_id}")
                continue
            
            target_global_id = self.original_to_global[target_id]
            node_type = self.original_to_type[target_id]
            
            # 获取当前节点和上一个节点
            current_node_id = complete_sequence[i-1] if i > 0 else self.station_original_id
            
            # 获取路径坐标（使用道路网络）
            path_coords = self._get_path_between(current_node_id, target_id)
            
            if node_type == "station":
                charge_level = 0
                return_count += 1
                charge_time = 0
            else:
                charge_level = self._calculate_charge_level(target_id)
                # 计算充电时间
                node_info = next((n for n in self.point_info if n['node_id'] == target_id), None)
                if node_info:
                    current_energy = node_info['battery_info'].get('current_energy', 0)
                    charging_rate = node_info['battery_info'].get('charging_rate', 0.85)
                    max_battery = self.sensor_max_battery if node_type == 'sensor' else self.gateway_max_battery
                    energy_needed = max_battery - current_energy
                    effective_power = self.charge_power * charging_rate
                    charge_time = energy_needed / effective_power if effective_power > 0 else 0
                else:
                    charge_time = 0
            
            action = np.array([target_global_id, charge_level])
            obs, reward, terminated, truncated, info = self.env.step(action)
            
            # 记录这一步的信息
            step_distance = info.get("distance_needed", 0)
            step_time = step_distance / self.env.agent_speed if self.env.agent_speed > 0 else 0
            
            action_log.append({
                "step": step + 1,
                "uav_to": target_id,
                "destination_type": "station" if node_type == "station" else node_type,
                "path_nodes": path_coords,  # 关键：添加路径节点
                "move_time": step_time,
                "charge_time": charge_time
            })
            
            total_reward += reward
            total_distance += step_distance
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
            'charged_count': final_info["node_charge_count"],
            'total_nodes': len(charging_sequence),
            'num_trips': return_count,
            'eue': eue,
            'charge_energy': self.env.charg_energy,
            'move_energy': self.env.move_energy,
            'tx_energy': self.env.tx_energy,
            'active_energy': self.env.active_energy,
            'action_log': action_log  # 添加 action_log 到返回结果
        }
    
def main():
    print("="*70)
    print("道路网络贪心路径规划仿真（无限电池版本）")
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
    
    # ==================== 3. 贪心算法基于完整大地图规划路径 ====================
    print("\n[3] 贪心算法基于完整大地图规划路径（无限电池版本）...")
    plan_start_time = time.time()
    
    # 使用完整道路网络和从 WSN 获取的充电节点信息进行贪心规划
    planner = GreedyPathPlanner(network, point_info)
    plan = planner.get_plan()
    
    plan_time = time.time() - plan_start_time
    
    print(f"\n  规划耗时: {plan_time:.4f}s")
    print(f"  需要充电节点数: {plan['total_nodes']}")
    print(f"  总路径长度（含返回充电站）: {plan['total_distance']:.2f} m ({plan['total_distance']/1000:.2f} km)")
    print(f"  充电顺序: {' -> '.join(plan['charging_sequence'][:5])}" + 
          (f" ... -> {plan['charging_sequence'][-1]}" if len(plan['charging_sequence']) > 5 else ""))
    
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
    print(f"| 返回充电站次数 (num_returns): {results['num_trips']:<49} |")
    print(f"| 平均能量利用效率 (EUE): {results['eue']:.4f}{'':<43} |")
    print(f"| 累积奖励 (total_reward): {results['total_reward']:.2f}{'':<45} |")
    print(f"| 充电能量 (charge_energy): {results['charge_energy']:.2f} J{'':<43} |")
    print(f"| 移动能量 (move_energy): {results['move_energy']:.2f} J{'':<44} |")
    print(f"| 传输能量 (tx_energy): {results['tx_energy']:.2f} J{'':<44} |")
    print(f"| 激活能量 (active_energy): {results['active_energy']:.2f} J{'':<44} |")
    print("="*70)
    

    print("="*70)
    print("仿真完成!")
    print("="*70)


if __name__ == "__main__":
    main()