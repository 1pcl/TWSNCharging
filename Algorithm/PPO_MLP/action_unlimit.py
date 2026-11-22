import numpy as np
import os
import sys
import time
import json
import torch
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
except ImportError as e:
    print(f"导入模块失败: {e}")
    sys.exit(1)
from attention_model_wrapper import CustomMaskableActorCriticPolicy
# 尝试导入MaskablePPO
try:
    from sb3_contrib import MaskablePPO
except ImportError:
    print("警告: sb3_contrib 未安装，RAWCS算法将不可用")
    MaskablePPO = None

# 模型文件路径映射
MODEL_PATHS = {
    'RN373': './models/RN373_model.zip',
    'RN446': './models/RN446_model.zip',
    'RN528': './models/RN528_model.zip',
    'RN629': './models/RN629_model.zip',
    'default': './models/unlimited_best_model.zip'
}


class RAWCSPlanner:
    """
    RAWCS算法规划器 - 使用预训练的MaskablePPO模型进行路径规划
    """
    
    def __init__(self, network, charging_node_info: List[Dict], dataset_name: str = 'default', max_min: Dict = None):
        """
        初始化RAWCS规划器
        
        Args:
            network: 完整道路网络对象
            charging_node_info: 从 WSNSimulation 获取的 point_info 数据
            dataset_name: 数据集名称，用于加载对应的模型
            max_min: WSN仿真返回的max_min数据（包含distance/x/y的min/max）
        """
        self.network = network
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        
        # 从 WSN 数据中提取需要充电的节点ID和充电站ID
        self.charging_node_ids = []
        self.charging_node_info = {}
        self.station_id = None
        self.all_nodes_info = {}  # 所有节点信息（包括不需要充电的）
        
        for node in charging_node_info:
            node_id = node['node_id']
            self.all_nodes_info[node_id] = node
            if node['battery_info'].get('needs_charge', False):
                self.charging_node_ids.append(node_id)
                self.charging_node_info[node_id] = node
            elif node.get('type') == 'station':
                self.station_id = node['node_id']
        
        if self.station_id is None:
            for node in charging_node_info:
                if node.get('type') == 'station':
                    self.station_id = node['node_id']
                    break
        
        if self.station_id is None:
            raise ValueError("未找到充电站节点")
        
        # 构建完整的节点列表（包括充电站和所有需要充电的节点）
        self.all_node_ids = self.charging_node_ids + [self.station_id]
        
        # 在完整大地图上计算所有节点之间的最短路径距离
        if hasattr(network, 'compute_shortest_paths_full'):
            self.dist_matrix, self.node_to_idx = network.compute_shortest_paths_full(self.all_node_ids)
        else:
            self.dist_matrix, self.node_to_idx = self._compute_dist_matrix(self.all_node_ids)
        
        # 加载模型
        self.model = self._load_model(dataset_name)
        
        # 创建环境（用于模型推理）- 使用传入的max_min
        self.env = self._create_env(charging_node_info, max_min)
        
        # 运行模型推理获取序列
        self.complete_sequence, self.inference_time_ms = self._run_model_inference()
    
    def _compute_dist_matrix(self, node_ids: List[str]):
        """计算节点间距离矩阵"""
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
                current_dist, current = heapq.heappop(pq)
                if current in visited:
                    continue
                visited.add(current)
                
                if current not in self.network.full_graph:
                    continue
                    
                for neighbor, length in self.network.full_graph[current].items():
                    new_dist = current_dist + length
                    if new_dist < distances.get(neighbor, float('inf')):
                        distances[neighbor] = new_dist
                        heapq.heappush(pq, (new_dist, neighbor))
            
            for j, end_id in enumerate(node_ids):
                if end_id in distances:
                    dist_matrix[i, j] = distances[end_id]
        
        return dist_matrix, node_to_idx
    
    def _load_model(self, dataset_name: str):
        """加载对应数据集的模型"""
        if MaskablePPO is None:
            raise ImportError("MaskablePPO未安装，无法加载RAWCS模型")
        
        model_path = MODEL_PATHS.get(dataset_name, MODEL_PATHS['default'])
        
        # 检查模型文件是否存在
        if not os.path.exists(model_path):
            print(f"警告: 模型文件 {model_path} 不存在，尝试使用默认模型")
            model_path = MODEL_PATHS['default']
            
        print(f"加载RAWCS模型: {model_path}")
        model = MaskablePPO.load(model_path, device=self.device)
        return model
    
    def _create_env(self, charging_node_info, max_min: Dict = None):
        """创建UAV环境用于模型推理"""
        # 构建graph_G（充电节点之间的连接图）
        graph_G = defaultdict(dict)
        for i, node1 in enumerate(self.all_node_ids):
            for j, node2 in enumerate(self.all_node_ids):
                if i != j:
                    dist = self.dist_matrix[i, j]
                    if dist < float('inf'):
                        graph_G[node1][node2] = dist
        
        # 转换graph_G为字符串键的字典
        graph_G_str = {}
        for k, v in graph_G.items():
            graph_G_str[str(k)] = {str(nk): nv for nk, nv in v.items()}
        
        # 准备point_info（包含电池信息）
        point_info_for_env = []
        for node_id in self.all_node_ids:
            if node_id in self.charging_node_info:
                point_info_for_env.append(self.charging_node_info[node_id])
            elif node_id in self.all_nodes_info:
                point_info_for_env.append(self.all_nodes_info[node_id])
        
        # 处理max_min：确保格式正确（与贪心算法一致）
        if max_min is None:
            # 如果没有传入，尝试从charging_node_info中提取或使用默认嵌套结构
            print("警告: 未传入max_min，使用默认值")
            max_min = {
                "distance": {"min": 0, "max": 5000},
                "x": {"min": 0, "max": 2000},
                "y": {"min": 0, "max": 2000}
            }
        else:
            # 检查max_min格式，如果是扁平结构则转换为嵌套结构
            if isinstance(max_min.get("distance"), (int, float)):
                # 扁平结构，转换为嵌套结构
                print("检测到扁平结构的max_min，转换为嵌套结构")
                max_min = {
                    "distance": {"min": 0, "max": max_min.get("distance", 5000)},
                    "x": {"min": 0, "max": max_min.get("x", 2000)},
                    "y": {"min": 0, "max": max_min.get("y", 2000)}
                }
        
        # 创建环境
        env = UAVSchedulingUnlimitedEnv(
            graph_G=graph_G_str,
            point_info=point_info_for_env,
            max_min=max_min,
            seed=42,
            is_train=False
        )
        
        return env
    
    def _run_model_inference(self):
        """运行模型推理获取充电序列"""
        inference_start = time.time()
        
        obs, _ = self.env.reset()
        done = False
        
        path_sequence = [self.station_id]
        charging_nodes = []
        
        # 构建映射关系
        global_to_original = {}
        for gid, data in self.env.global_to_original_map.items():
            global_to_original[gid] = data['original_id']
        
        current_node = self.station_id
        
        while not done:
            # 获取动作掩码
            action_masks = self.env.action_masks()
            
            # 模型预测
            action, _ = self.model.predict(obs, action_masks=action_masks, deterministic=True)
            
            # 解码动作
            action_dict = self.env._decode_action(action)
            target_node_id = action_dict["target_node_id"]
            
            # 转换为原始ID
            target_original_id = global_to_original[target_node_id]
            target_type = self.env.global_to_original_map[target_node_id]['original_type']
            
            # 记录序列
            path_sequence.append(target_original_id)
            if target_type != 'station':
                charging_nodes.append(target_original_id)
            
            # 环境步进
            obs, reward, terminated, truncated, info = self.env.step(action)
            current_node = target_original_id
            done = terminated or truncated
        
        # 确保最后返回充电站（如果模型没有返回）
        if path_sequence[-1] != self.station_id:
            path_sequence.append(self.station_id)
        
        inference_time = (time.time() - inference_start) * 1000  # 转换为毫秒
        
        print(f"模型推理完成: {len(charging_nodes)} 个充电节点, 推理时间: {inference_time:.2f}ms")
        
        return path_sequence, inference_time
    
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
        
        for target_id in self.complete_sequence[1:]:
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
            'optimization_time': self.inference_time_ms / 1000,
            'inference_time_ms': self.inference_time_ms
        }
    
    def get_path_between_nodes(self, from_node_id: str, to_node_id: str) -> List[Tuple[float, float]]:
        """获取两个节点之间的完整路径坐标"""
        if from_node_id == to_node_id:
            node = self.network.get_node_by_id(from_node_id)
            if node:
                return [(node['x'], node['y'])]
            return []
        
        path_node_ids = self.network.get_path_between_full(from_node_id, to_node_id)
        
        coords = []
        for node_id in path_node_ids:
            node = self.network.get_node_by_id(node_id)
            if node:
                coords.append((node['x'], node['y']))
        
        if len(coords) < 2:
            from_node = self.network.get_node_by_id(from_node_id)
            to_node = self.network.get_node_by_id(to_node_id)
            if from_node and to_node:
                coords = [(from_node['x'], from_node['y']), (to_node['x'], to_node['y'])]
        
        return coords
    
    def get_full_path_coords(self) -> List[List[Tuple[float, float]]]:
        """获取整个规划路径的坐标段列表"""
        path_coords_list = []
        current = self.station_id
        
        for target_id in self.complete_sequence[1:]:
            path_coords = self.get_path_between_nodes(current, target_id)
            if path_coords:
                path_coords_list.append(path_coords)
            current = target_id
        
        return path_coords_list


class RAWCSSimulator:
    """RAWCS 仿真器 - 计算性能指标"""
    
    def __init__(self, wsn_data: Dict[str, Any], road_network=None):
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
        
        self.road_network = road_network
    
    def _calculate_charge_level(self, node_id: str) -> int:
        """计算充电等级"""
        node_info = None
        for info in self.point_info:
            if info['node_id'] == node_id:
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
        
        for i, target_id in enumerate(complete_sequence):
            is_station = (target_id == self.station_original_id)
            
            if i == 0 and is_station:
                continue
            
            if target_id not in self.original_to_global:
                print(f"  警告: 找不到节点 {target_id}")
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
            
            step_distance = info.get("distance_needed", 0)
            total_reward += reward
            total_distance += step_distance
            step += 1
            
            if terminated or truncated:
                break
        
        final_info = self.env._get_info()
        total_consumed = self.env.tx_energy + self.env.move_energy
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
        }
