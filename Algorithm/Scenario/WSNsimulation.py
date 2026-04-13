import json
import random
import math
import numpy as np
from typing import Dict, List, Any, Optional, Union, Tuple
from collections import defaultdict
import heapq
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

class WSNSimulation:
    def __init__(self, data_input: Union[str, Dict[str, Any]], 
                 sensor_battery: float = 240000.0, 
                 relay_battery: float = 800000.0,
                 station_battery: float = 1000000.0,
                 num_sensors: int = 40,
                 num_relays: int = 5,
                 seed: int = 42):
        """
        初始化 WSN 仿真器（基于道路图结构）
        单位：所有距离单位为米（m）
        """
        self.sensor_battery = sensor_battery
        self.relay_battery = relay_battery
        self.station_battery = station_battery
        self.num_sensors = num_sensors
        self.num_relays = num_relays
        self.seed = seed
        
        # 数据存储
        self.data = None
        self.full_graph = {}  # 完整邻接表（距离单位：米）
        self.graph_G = {}  # 过滤后的邻接表（只包含充电节点之间的直接连接）
        self.point_info = []  # 节点信息列表（只包含充电节点）
        self.nodes_dict = {}  # 所有节点信息
        self.charging_node_ids = set()  # 充电节点ID集合
        
        # 统计信息
        self.stats = {
            "total_nodes": 0,
            "sensors": 0,
            "relays": 0,
            "stations": 0,
            "charging_nodes": 0
        }
        
        # 最大最小值
        self.max_min = {
            "distance": {"max": 0.0, "min": float('inf')},
            "x": {"max": -float('inf'), "min": float('inf')},
            "y": {"max": -float('inf'), "min": float('inf')}
        }
        
        # 设置随机种子
        random.seed(seed)
        np.random.seed(seed)
        
        # 加载数据
        self._load_data(data_input)
        self._build_full_graph()
        self._select_charging_nodes()
        self._build_direct_connection_graph()
        # self._calculate_max_min()
    
    def _load_data(self, data_input: Union[str, Dict[str, Any]]) -> None:
        """从文件路径或数据字典加载数据"""
        if isinstance(data_input, str):
            with open(data_input, "r") as f:
                self.data = json.load(f)
        else:
            self.data = data_input
        
        # 检查单位
        # unit = self.data.get('unit', 'meters')
        # print(f"Loaded data unit: {unit}")
    
    def _build_full_graph(self) -> None:
        """构建完整的节点信息和图结构"""
        
        # 构建节点字典
        for node_data in self.data.get('nodes', []):
            node_id = node_data['id']
            node_type = node_data['node_type']
            
            # 兼容不同的坐标字段名
            if 'x_m' in node_data:
                x = node_data['x_m']
                y = node_data['y_m']
            elif 'x' in node_data:
                x = node_data['x']
                y = node_data['y']
            else:
                x = 0
                y = 0
            
            node_info = {
                'node_id': node_id,
                'type': node_type,
                'x': x,
                'y': y,
                'degree': node_data['degree'],
                'neighbors': node_data['neighbors'],
                'battery_info': None
            }
            
            self.nodes_dict[node_id] = node_info
            
            # 统计
            if node_type == 'sensor':
                self.stats['sensors'] += 1
            elif node_type == 'relay':
                self.stats['relays'] += 1
            elif node_type == 'station':
                self.stats['stations'] += 1
            
            # 更新坐标范围
            if x > self.max_min["x"]["max"]:
                self.max_min["x"]["max"] = x
            if x < self.max_min["x"]["min"]:
                self.max_min["x"]["min"] = x
            if y > self.max_min["y"]["max"]:
                self.max_min["y"]["max"] = y
            if y < self.max_min["y"]["min"]:
                self.max_min["y"]["min"] = y
        
        # 构建完整的邻接表
        self.full_graph = {}
        for edge in self.data.get('edges', []):
            from_id = edge['from']
            to_id = edge['to']
            
            # 兼容不同的长度字段名
            if 'length_m' in edge:
                length = edge['length_m']  # 已经是米
            elif 'length_km' in edge:
                length = edge['length_km'] * 1000  # 转换为米
            else:
                length = edge.get('length', 0)  # 默认为米
            
            if from_id not in self.full_graph:
                self.full_graph[from_id] = {}
            if to_id not in self.full_graph:
                self.full_graph[to_id] = {}
            
            self.full_graph[from_id][to_id] = length
            self.full_graph[to_id][from_id] = length
        
        # print(f"Built full graph: {len(self.nodes_dict)} nodes, {len(self.full_graph)} edges")
    
    def _calculate_charging_rate(self, distance_in_road: float, node_type: str) -> float:
        """
        计算充电效率
        distance_in_road: 距离（米）
        node_type: 节点类型
        """
        d_h = distance_in_road
        d_ref = 1.0  # 参考距离1米
        eta_ref = 0.98
        alpha = 6.0
        
        if node_type == 'sensor':
            height = 0.0
        else:
            height = 3.0
        
        beta = 1.0
        d_3d = math.sqrt(d_h**2 + (height + beta)**2)
        
        if node_type == 'sensor':
            raw_rate = alpha * eta_ref * (d_ref / d_3d) ** 3.0
        else:
            raw_rate = alpha * eta_ref * (d_ref / d_3d) ** 1.8
        
        charging_rate = max(0.70, min(0.98, raw_rate))
        
        return charging_rate
    
    def _select_charging_nodes(self) -> None:
        """选择需要充电的节点（传感器和中继节点）"""
        
        all_sensors = [node for node in self.nodes_dict.values() if node['type'] == 'sensor']
        all_relays = [node for node in self.nodes_dict.values() if node['type'] == 'relay']
        all_stations = [node for node in self.nodes_dict.values() if node['type'] == 'station']
        
        # 随机选择需要充电的节点
        num_sensors = min(self.num_sensors, len(all_sensors))
        selected_sensors = random.sample(all_sensors, num_sensors) if all_sensors else []
        
        num_relays = min(self.num_relays, len(all_relays))
        selected_relays = random.sample(all_relays, num_relays) if all_relays else []
        
        # 为选中的传感器设置低电量
        for sensor in selected_sensors:
            current_energy_ratio = random.uniform(0.05, 0.30)
            current_energy = self.sensor_battery * current_energy_ratio
            distance_in_road = random.uniform(0.1, 1)  # 距离0.1-1米
            charging_rate = self._calculate_charging_rate(distance_in_road, 'sensor')
            
            sensor['battery_info'] = {
                'current_energy': current_energy,
                'current_energy_ratio': current_energy_ratio,
                'charging_rate': charging_rate,
                'total_battery': self.sensor_battery,
                'needs_charge': True
            }
        
        # 为选中的中继节点设置低电量
        for relay in selected_relays:
            current_energy_ratio = random.uniform(0.05, 0.30)
            current_energy = self.relay_battery * current_energy_ratio
            distance_in_road = random.uniform(0.1, 1)  # 距离0.1-1米
            charging_rate = self._calculate_charging_rate(distance_in_road, 'relay')
            
            relay['battery_info'] = {
                'current_energy': current_energy,
                'current_energy_ratio': current_energy_ratio,
                'charging_rate': charging_rate,
                'total_battery': self.relay_battery,
                'needs_charge': True
            }
        
        # 为充电站设置电池信息
        for station in all_stations:
            station['battery_info'] = {
                'current_energy': self.station_battery,
                'current_energy_ratio': 1.0,
                'charging_rate': 0.95,
                'total_battery': self.station_battery,
                'needs_charge': False
            }
        
        # 保存选中的节点
        self.selected_sensors = selected_sensors
        self.selected_relays = selected_relays
        self.selected_stations = all_stations
        
        # 构建 point_info，只包含选中的节点
        self.point_info = []
        for sensor in selected_sensors:
            self.point_info.append(sensor)
            self.charging_node_ids.add(sensor['node_id'])
        for relay in selected_relays:
            self.point_info.append(relay)
            self.charging_node_ids.add(relay['node_id'])
        for station in all_stations:
            self.point_info.append(station)
            self.charging_node_ids.add(station['node_id'])
        
        self.stats['charging_nodes'] = len(selected_sensors) + len(selected_relays)
        
        # print(f"Selected charging nodes: {len(selected_sensors)} sensors, {len(selected_relays)} relays")
    
    def _build_direct_connection_graph(self) -> None:
        """
        构建只包含充电节点之间的直接连接图
        规则：如果两个充电节点之间的最短路径上不包含其他充电节点，则它们直接连接
        """
        
        charging_list = list(self.charging_node_ids)
        
        # 计算所有充电节点之间的最短路径
        all_distances = {}
        all_paths = {}
        
        for i, start_id in enumerate(charging_list):
            distances, paths = self._dijkstra_with_path(start_id)
            all_distances[start_id] = distances
            all_paths[start_id] = paths
        
        # 构建直接连接图
        self.graph_G = defaultdict(dict)
        
        # 对每一对充电节点，检查最短路径上是否有其他充电节点
        for i in range(len(charging_list)):
            for j in range(i + 1, len(charging_list)):
                start = charging_list[i]
                end = charging_list[j]
                
                if end not in all_distances[start]:
                    continue
                
                # 获取最短路径
                path = all_paths[start].get(end, [])
                
                if not path:
                    continue
                
                # 检查路径上是否有其他充电节点（不包括起点和终点）
                has_intermediate_charging = False
                for node in path[1:-1]:
                    if node in self.charging_node_ids:
                        has_intermediate_charging = True
                        break
                
                # 如果没有中间充电节点，则直接连接
                if not has_intermediate_charging:
                    distance = all_distances[start][end]
                    self.graph_G[start][end] = distance
                    self.graph_G[end][start] = distance
                    
                    # 更新距离的最大最小值
                    if distance > self.max_min["distance"]["max"]:
                        self.max_min["distance"]["max"] = distance
                    if distance < self.max_min["distance"]["min"]:
                        self.max_min["distance"]["min"] = distance
        
        # 处理没有边的情况
        if self.max_min["distance"]["min"] == float('inf'):
            self.max_min["distance"]["min"] = 0
        if self.max_min["distance"]["max"] == -float('inf'):
            self.max_min["distance"]["max"] = 1
        
        # 更新统计
        self.stats['total_nodes'] = len(self.graph_G)
        
        # print(f"Built direct connection graph: {len(self.graph_G)} nodes, "
        #       f"{sum(len(neighbors) for neighbors in self.graph_G.values()) // 2} edges")
        # print(f"  Distance range: {self.max_min['distance']['min']:.1f} - {self.max_min['distance']['max']:.1f} m")
    
    def _dijkstra_with_path(self, start_id: str) -> Tuple[Dict[str, float], Dict[str, List[str]]]:
        """Dijkstra算法计算从start到所有节点的最短距离和路径"""
        if start_id not in self.full_graph:
            return {}, {}
        
        distances = {start_id: 0}
        paths = {start_id: [start_id]}
        pq = [(0, start_id)]
        visited = set()
        
        while pq:
            current_dist, current = heapq.heappop(pq)
            
            if current in visited:
                continue
            visited.add(current)
            
            if current not in self.full_graph:
                continue
            
            for neighbor, dist in self.full_graph[current].items():
                new_dist = current_dist + dist
                if neighbor not in distances or new_dist < distances[neighbor]:
                    distances[neighbor] = new_dist
                    paths[neighbor] = paths[current] + [neighbor]
                    heapq.heappush(pq, (new_dist, neighbor))
        
        return distances, paths
    
    # def _calculate_max_min(self):
    #     """计算压缩后的图的最大最小值"""
    #     # 更新坐标范围（基于保留的节点）
    #     for node in self.point_info:
    #         x = node['x']
    #         y = node['y']
    #         if x > self.max_min["x"]["max"]:
    #             self.max_min["x"]["max"] = x
    #         if x < self.max_min["x"]["min"]:
    #             self.max_min["x"]["min"] = x
    #         if y > self.max_min["y"]["max"]:
    #             self.max_min["y"]["max"] = y
    #         if y < self.max_min["y"]["min"]:
    #             self.max_min["y"]["min"] = y
    
    def get_data(self) -> Dict[str, Any]:
        """返回图数据和节点信息"""
        return {
            "graph_G": self.graph_G,
            "point_info": self.point_info,
            "stats": self.stats,
            "max_min": self.max_min
        }
    
    def get_statistics(self) -> Dict[str, Any]:
        """返回统计信息"""
        return self.stats
    
    def get_graph_info(self) -> Dict[str, Any]:
        """返回图的基本信息"""
        node_types = defaultdict(int)
        for node in self.point_info:
            node_types[node['type']] += 1
        
        return {
            "total_nodes": len(self.graph_G),
            "total_edges": sum(len(neighbors) for neighbors in self.graph_G.values()) // 2,
            "node_types": dict(node_types),
            "charging_nodes": self.stats['charging_nodes']
        }
    
    def get_max_min(self) -> Dict[str, Any]:
        """返回最大最小值"""
        return self.max_min


# ============= 使用示例 =============

if __name__ == "__main__":
    
    GRAPH_FILE = "./data/road_graph_with_nodes.json"
    
    try:
        with open(GRAPH_FILE, 'r') as f:
            graph_data = json.load(f)

        seed = 42
        
        print("="*60)
        print("WSN Simulation - Charging Node Graph")
        print("="*60)
        
        wsn_sim = WSNSimulation(
            data_input=graph_data,
            sensor_battery=240000.0,
            relay_battery=800000.0,
            station_battery=1000000.0,
            num_sensors=40,
            num_relays=5,
            seed=seed
        )

        sim_data = wsn_sim.get_data()
        stats = wsn_sim.get_statistics()
        graph_info = wsn_sim.get_graph_info()
        max_min = wsn_sim.get_max_min()
            
        print(f"\n{'='*60}")
        print(f"Final Results:")
        print(f"{'='*60}")
        print(f"Total nodes in graph: {graph_info['total_nodes']}")
        print(f"Total edges: {graph_info['total_edges']}")
        print(f"Node types: {graph_info['node_types']}")
        print(f"Charging nodes: {stats['charging_nodes']}")
        print(f"Distance range: {max_min['distance']['min']:.1f} - {max_min['distance']['max']:.1f} m")
        print(f"X range: {max_min['x']['min']/1000:.1f} - {max_min['x']['max']/1000:.1f} km")
        print(f"Y range: {max_min['y']['min']/1000:.1f} - {max_min['y']['max']/1000:.1f} km")
        
        
    except FileNotFoundError:
        print(f"Error: File {GRAPH_FILE} not found!")
        print("Please run insert_sensors.py first to generate the graph data.")
