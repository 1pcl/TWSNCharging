# insert_sensors.py
import numpy as np
from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Optional
from collections import defaultdict
import json
import random
import matplotlib.pyplot as plt
import os

@dataclass
class Node:
    """Base node class for all nodes in the graph"""
    id: str
    x: float  # 单位：米
    y: float  # 单位：米
    node_type: str  # 'intersection', 'sensor', 'station'
    degree: int = 0
    neighbors: List[str] = field(default_factory=list)

class RoadGraphWithSensors:
    """
    Graph with intersections, sensors, and stations all as nodes
    """
    
    def __init__(self):
        self.adjacency: Dict[str, Dict[str, str]] = defaultdict(dict)
        self.nodes: Dict[str, Node] = {}
        self.edges: Dict[str, Tuple[str, str, float]] = {}  # edge_id -> (from, to, length in meters)
        
    def add_node(self, node_id: str, x: float, y: float, node_type: str) -> Node:
        """Add a node to the graph"""
        node = Node(
            id=node_id,
            x=x,
            y=y,
            node_type=node_type
        )
        self.nodes[node_id] = node
        return node
    
    def add_edge(self, edge_id: str, from_id: str, to_id: str, length: float):
        """Add an edge between two nodes (length in meters)"""
        self.adjacency[from_id][to_id] = edge_id
        self.adjacency[to_id][from_id] = edge_id
        self.edges[edge_id] = (from_id, to_id, length)
        
        # Update node degrees
        self.nodes[from_id].degree += 1
        self.nodes[to_id].degree += 1
        self.nodes[from_id].neighbors.append(to_id)
        self.nodes[to_id].neighbors.append(from_id)
    
    def load_road_network(self, network_file: str):
        """Load original road network and convert to graph nodes (coordinates in meters)"""
        with open(network_file, 'r') as f:
            data = json.load(f)
        
        # Add all intersections as nodes
        for inter_data in data['intersections']:
            # 兼容两种字段名，单位转换为米
            if 'x_m' in inter_data:
                x = inter_data['x_m']
                y = inter_data['y_m']
            elif 'x_km' in inter_data:
                x = inter_data['x_km'] * 1000
                y = inter_data['y_km'] * 1000
            else:
                x = inter_data.get('x', 0) * 1000 if 'x' in inter_data else 0
                y = inter_data.get('y', 0) * 1000 if 'y' in inter_data else 0
            
            self.add_node(
                node_id=inter_data['id'],
                x=x,
                y=y,
                node_type='intersection'
            )
        
        # Add all roads as edges
        for road_data in data['roads']:
            # 兼容两种字段名，单位转换为米
            if 'length_m' in road_data:
                length = road_data['length_m']
            elif 'length_km' in road_data:
                length = road_data['length_km'] * 1000
            else:
                length = road_data.get('length', 0) * 1000 if 'length' in road_data else 0
            
            self.add_edge(
                edge_id=road_data['id'],
                from_id=road_data['from'],
                to_id=road_data['to'],
                length=length
            )
        
        total_length_km = sum([l for _, _, l in self.edges.values()]) / 1000
        print(f"Loaded road network: {len(self.nodes)} intersections, {len(self.edges)} roads")
        print(f"  Total road length: {total_length_km:.1f} km")
    
    def insert_sensor_on_road(self, sensor_id: str, road_id: str, position_ratio: float):
        """Insert a sensor node on an existing road, splitting the road into two segments"""
        # Get the original road endpoints
        from_id, to_id, length = self.edges[road_id]
        from_node = self.nodes[from_id]
        to_node = self.nodes[to_id]
        
        # Calculate sensor position (coordinates in meters)
        x = from_node.x + position_ratio * (to_node.x - from_node.x)
        y = from_node.y + position_ratio * (to_node.y - from_node.y)
        
        # Add sensor node
        self.add_node(sensor_id, x, y, 'sensor')
        
        # Calculate segment lengths (in meters)
        length1 = np.sqrt((x - from_node.x)**2 + (y - from_node.y)**2)
        length2 = np.sqrt((to_node.x - x)**2 + (to_node.y - y)**2)
        
        # Remove old road edge
        del self.adjacency[from_id][to_id]
        del self.adjacency[to_id][from_id]
        del self.edges[road_id]
        
        # Create two new edges
        edge1_id = f"{road_id}_seg1"
        edge2_id = f"{road_id}_seg2"
        
        self.add_edge(edge1_id, from_id, sensor_id, length1)
        self.add_edge(edge2_id, sensor_id, to_id, length2)
        
        return sensor_id
    
    def insert_station_on_road(self, station_id: str, road_id: str, position_ratio: float):
        """Insert a charging station node on an existing road, splitting the road into two segments"""
        # Get the original road endpoints
        from_id, to_id, length = self.edges[road_id]
        from_node = self.nodes[from_id]
        to_node = self.nodes[to_id]
        
        # Calculate station position (coordinates in meters)
        x = from_node.x + position_ratio * (to_node.x - from_node.x)
        y = from_node.y + position_ratio * (to_node.y - from_node.y)
        
        # Add station node
        self.add_node(station_id, x, y, 'station')
        
        # Calculate segment lengths (in meters)
        length1 = np.sqrt((x - from_node.x)**2 + (y - from_node.y)**2)
        length2 = np.sqrt((to_node.x - x)**2 + (to_node.y - y)**2)
        
        # Remove old road edge
        del self.adjacency[from_id][to_id]
        del self.adjacency[to_id][from_id]
        del self.edges[road_id]
        
        # Create two new edges
        edge1_id = f"{road_id}_seg1"
        edge2_id = f"{road_id}_seg2"
        
        self.add_edge(edge1_id, from_id, station_id, length1)
        self.add_edge(edge2_id, station_id, to_id, length2)
        
        return station_id


    def deploy_sensors(self, 
                    sensors_per_road: Tuple[int, int] = (2, 5),  # 每条道路2-5个传感器
                    num_stations: int = 1,
                    seed: int = 0):
        """
        部署传感器：每条道路部署固定数量的传感器，均匀分布
        
        Args:
            sensors_per_road: (min_sensors, max_sensors) 每条道路的传感器数量
            num_stations: 充电站数量
            seed: 随机种子
        """
        sensor_counter = 0
        station_counter = 0

        random.seed(seed)
        np.random.seed(seed)

        # Get all current roads
        road_ids = list(self.edges.keys())
        
        # Store original road info for sensor deployment
        roads_to_process = []
        road_lengths_m = []
        for road_id in road_ids:
            from_id, to_id, length = self.edges[road_id]
            # Only deploy on roads between intersections (not already split)
            if self.nodes[from_id].node_type == 'intersection' and self.nodes[to_id].node_type == 'intersection':
                roads_to_process.append(road_id)
                road_lengths_m.append(length)
        
        total_sensors = 0
        
        # 统计道路长度分布
        avg_road_length_m = np.mean(road_lengths_m)
        print(f"  Average road length: {avg_road_length_m:.1f} m ({avg_road_length_m/1000:.2f} km)")
        print(f"  Sensors per road: {sensors_per_road[0]}-{sensors_per_road[1]}")
        print()
        
        # 部署传感器和中继
        for road_id in roads_to_process:
            from_id, to_id, road_length_m = self.edges[road_id]  # length in meters
            
            # 随机选择这条道路的传感器数量
            num_sensors = random.randint(sensors_per_road[0], sensors_per_road[1])
            
            
            total_sensors += num_sensors
            
            road_length_km = road_length_m / 1000
            print(f"  Road {road_id}: {road_length_km:.2f} km ({road_length_m:.0f} m) → {num_sensors} sensors")
            
            # 生成传感器位置：均匀分布在整条道路上（0到1之间）
            sensor_positions = np.linspace(0, 1, num_sensors + 2)[1:-1]  # 不包括端点
            # 添加随机扰动，避免完全均匀
            perturbation = np.random.uniform(-0.05, 0.05, num_sensors)
            sensor_positions += perturbation
            sensor_positions = np.clip(sensor_positions, 0.02, 0.98)  # 稍微避免端点
            sensor_positions.sort()
            
            
            # 部署所有传感器
            current_road_id = road_id
            sensor_info_list = []
            
            for idx, pos in enumerate(sensor_positions):
                sensor_id = f"S{sensor_counter}"
                self.insert_sensor_on_road(sensor_id, current_road_id, pos)
                
                sensor_info_list.append({
                    'id': sensor_id,
                    'position': pos,
                })
                
                sensor_counter += 1
                current_road_id = f"{current_road_id}_seg2"
            
        
        print(f"\n{'='*60}")
        print(f"Deployment Summary:")
        print(f"  Total sensors deployed: {total_sensors}")
        print(f"  Average sensors per road: {total_sensors/len(roads_to_process):.1f}")
        print(f"{'='*60}")
        
        # 部署基站（充电站）- 在中心附近路口部署
        print(f"\nDeploying {num_stations} base station(s) at city center...")
        
        # 获取所有路口节点
        intersections = [node for node in self.nodes.values() if node.node_type == 'intersection']
        
        
        if len(intersections) > 0 and num_stations > 0:
            # 计算城市中心（所有路口的中点）
            center_x = np.mean([inter.x for inter in intersections])
            center_y = np.mean([inter.y for inter in intersections])
            
            print(f"  City center at: ({center_x/1000:.2f} km, {center_y/1000:.2f} km)")
            
            # 计算每个路口到中心的距离（单位：米）
            intersection_distances = []
            for inter in intersections:
                dist_to_center = np.sqrt((inter.x - center_x)**2 + (inter.y - center_y)**2)
                intersection_distances.append((dist_to_center, inter))
            
            # 按距离排序，选择最近的路口
            intersection_distances.sort(key=lambda x: x[0])
            
            # 选择最近的几个路口（根据num_stations）
            num_stations = min(num_stations, len(intersection_distances))
            selected_intersections = [inter for _, inter in intersection_distances[:num_stations]]
            
            # 为每个选中的路口部署基站
            for i, inter in enumerate(selected_intersections):
                # 找到连接该路口的道路
                possible_edges = []
                for edge_id, (from_id, to_id, length) in self.edges.items():
                    if from_id == inter.id or to_id == inter.id:
                        possible_edges.append((edge_id, length))
                
                if possible_edges:
                    # 选择最长的道路作为主干道部署基站
                    possible_edges.sort(key=lambda x: x[1], reverse=True)
                    selected_edge = possible_edges[0][0]
                    edge_length = possible_edges[0][1]
                    
                    # 在路口附近部署基站（100米处）
                    # 计算100米对应的比例
                    station_ratio = min(100.0 / edge_length, 0.3)  # 最多30%处
                    
                    station_id = f"BS{station_counter}"
                    self.insert_station_on_road(station_id, selected_edge, station_ratio)
                    station_counter += 1
                    print(f"  Base Station {station_id} deployed near intersection {inter.id}")
                    print(f"    Distance from center: {intersection_distances[i][0]/1000:.2f} km ({intersection_distances[i][0]:.0f} m)")
                    
        
        print(f"\n  Total base stations deployed: {station_counter}")
        
        return self

    def get_statistics(self) -> Dict:
        """Get network statistics (all lengths in meters)"""
        intersections = [node for node in self.nodes.values() if node.node_type == 'intersection']
        sensors = [node for node in self.nodes.values() if node.node_type == 'sensor']
        stations = [node for node in self.nodes.values() if node.node_type == 'station']
        
        # 计算平均传感器间距
        total_road_length_m = sum([length for _, _, length in self.edges.values()])
        avg_sensor_spacing_m = total_road_length_m / len(sensors) if len(sensors) > 0 else 0
        
        return {
            'total_nodes': len(self.nodes),
            'intersections': len(intersections),
            'sensors': len(sensors),
            'stations': len(stations),
            'total_edges': len(self.edges),
            'total_road_length_km': total_road_length_m / 1000,
            'avg_sensor_spacing_km': avg_sensor_spacing_m / 1000,
            'sensor_density_per_km': len(sensors) / (total_road_length_m / 1000) if total_road_length_m > 0 else 0,
        }
    
    def save_to_json(self, filename: str):
        """Save graph to JSON file (lengths in meters)"""
        data = {
            'statistics': self.get_statistics(),
            'unit': 'meters',
            'nodes': [
                {
                    'id': node.id,
                    'x_m': node.x,
                    'y_m': node.y,
                    'node_type': node.node_type,
                    'degree': node.degree,
                    'neighbors': node.neighbors
                }
                for node in self.nodes.values()
            ],
            'edges': [
                {
                    'id': edge_id,
                    'from': from_id,
                    'to': to_id,
                    'length_m': length
                }
                for edge_id, (from_id, to_id, length) in self.edges.items()
            ],
            'adjacency': {
                node_id: list(neighbors.keys())
                for node_id, neighbors in self.adjacency.items()
            }
        }
        
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
        
        print(f"Graph saved to: {filename}")
    
    def visualize(self, title="Smart Urban Sensor Network"):
        """Visualize the graph with improved colors (coordinates in meters)"""
        from matplotlib.lines import Line2D
        
        fig, ax = plt.subplots(figsize=(14, 10))
        
        # Draw edges (light gray)
        for edge_id, (from_id, to_id, length) in self.edges.items():
            from_node = self.nodes[from_id]
            to_node = self.nodes[to_id]
            ax.plot([from_node.x, to_node.x], [from_node.y, to_node.y], 
                'lightgray', linewidth=1, alpha=0.5, zorder=1)
        
        # Draw nodes by type with improved colors
        for node in self.nodes.values():
            if node.node_type == 'intersection':
                # 黑色路口
                ax.scatter(node.x, node.y, c='black', s=20, marker='o', 
                          alpha=0.8, zorder=5, edgecolors='gray', linewidth=0.5)
            elif node.node_type == 'sensor':
                # 浅蓝色传感器
                ax.scatter(node.x, node.y, c='lightblue', s=25, marker='o', 
                          alpha=0.7, zorder=6, edgecolors='deepskyblue', linewidth=0.8)
            elif node.node_type == 'station':
                # 绿色充电站（基站）
                ax.scatter(node.x, node.y, c='green', s=80, marker='s', 
                          alpha=0.9, zorder=8, edgecolors='darkgreen', linewidth=1.5)
        
        # Count nodes
        num_intersections = len([n for n in self.nodes.values() if n.node_type == 'intersection'])
        num_sensors = len([n for n in self.nodes.values() if n.node_type == 'sensor'])
        num_stations = len([n for n in self.nodes.values() if n.node_type == 'station'])
        
        # Create legend
        legend_elements = [
            Line2D([0], [0], marker='o', color='w', markerfacecolor='black', 
                  markersize=8, label=f'Intersection ({num_intersections})'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='lightblue', 
                  markeredgecolor='deepskyblue', markersize=8, label=f'Sensor ({num_sensors})'),
            Line2D([0], [0], marker='s', color='w', markerfacecolor='green', 
                  markeredgecolor='darkgreen', markersize=8, label=f'Station ({num_stations})'),
        ]
        
        # 将坐标从米转换为公里显示
        ax.set_xlabel('X (km)', fontsize=12)
        ax.set_ylabel('Y (km)', fontsize=12)
        
        # 转换刻度显示
        xticks = ax.get_xticks()
        yticks = ax.get_yticks()
        ax.set_xticklabels([f'{x/1000:.1f}' for x in xticks])
        ax.set_yticklabels([f'{y/1000:.1f}' for y in yticks])
        
        ax.set_title(title, fontsize=16, fontweight='bold')
        ax.grid(True, alpha=0.2, linestyle='--')
        ax.set_aspect('equal')
        
        # 调整布局，为右侧图例留出空间
        plt.subplots_adjust(right=0.85)
        # 图例放在外部右上角
        ax.legend(handles=legend_elements, loc='center left', 
                 bbox_to_anchor=(1.02, 0.98), fontsize=10, framealpha=0.9)
        
        return fig, ax


# ============= Example Usage =============

if __name__ == "__main__":
    
    # Configuration
    NETWORK_FILE = "./data/road_network_169_intersections.json"
    OUTPUT_FILE = "./data/road_graph_with_nodes.json"
    
    # 部署参数配置 - 每条道路固定数量的传感器
    SENSORS_PER_ROAD = (2, 5)      # 每条道路2-5个传感器
    NUM_STATIONS = 1               # 1个基站（在中心附近）
    
    print("="*60)
    print("Smart Urban Sensor Network Deployment")
    print("="*60)
    print(f"\nDeployment Strategy:")
    print(f"  - Sensors per road: {SENSORS_PER_ROAD[0]}-{SENSORS_PER_ROAD[1]}")
    print(f"  - Base stations: {NUM_STATIONS} (at city center)")
    print("="*60)
    
    # Create graph and load road network
    graph = RoadGraphWithSensors()
    graph.load_road_network(NETWORK_FILE)
    
    # Deploy sensors - 每条道路均匀部署固定数量的传感器
    graph.deploy_sensors(
        sensors_per_road=SENSORS_PER_ROAD,
        num_stations=NUM_STATIONS,
        seed=42
    )
    
    # Print statistics
    stats = graph.get_statistics()
    print(f"\n{'='*60}")
    print(f"Final Network Statistics")
    print(f"{'='*60}")
    for key, value in stats.items():
        if isinstance(value, float):
            if 'km' in key:
                print(f"  {key}: {value:.3f}")
            else:
                print(f"  {key}: {value:.2f}")
        else:
            print(f"  {key}: {value}")
    
    # Save to file
    os.makedirs("./data", exist_ok=True)
    graph.save_to_json(OUTPUT_FILE)
    
    # Visualize
    fig, ax = graph.visualize(title=f"Smart Urban Sensor Network")
    plt.show()