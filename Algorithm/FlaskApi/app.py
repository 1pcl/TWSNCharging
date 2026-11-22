import os
import sys
import json
import logging
from flask import Flask, request, jsonify
from typing import Dict, Any,List
import numpy as np
import time
import traceback
from collections import defaultdict
import heapq

# 设置路径
current_script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_script_dir)

# 添加项目根目录到Python路径
if project_root not in sys.path:
    sys.path.insert(0, project_root)
ppo_dir = os.path.join(project_root, "PPO_MLP")
if ppo_dir not in sys.path:
    sys.path.insert(0, ppo_dir)
# 导入WSN仿真模块
try:
    from Scenario.WSNsimulation import WSNSimulation
    logging.info("WSN仿真模块导入成功")
except ImportError as e:
    logging.error(f"WSN仿真模块导入失败: {e}")
    WSNSimulation = None

# 导入贪心算法模块
try:
    from other.greed_unlimited import GreedyPathPlanner, UAVSimulator as GreedyUAVSimulator
    logging.info("贪心算法模块导入成功")
except ImportError as e:
    logging.error(f"贪心算法模块导入失败: {e}")
    GreedyPathPlanner = None
    GreedyUAVSimulator = None

# # 导入ACO算法模块
# try:
#     from other.aco_unlimited import ImprovedAntColonyOptimizer, ACOSimulator
#     logging.info("ACO算法模块导入成功")
# except ImportError as e:
#     logging.error(f"ACO算法模块导入失败: {e}")
#     ACORoadNetwork = None
#     ImprovedAntColonyOptimizer = None
#     ACOSimulator = None

# 导入NSGA算法模块
try:
    from other.ga_unlimited import GeneticAlgorithm, GASimulator
    logging.info("NSGA算法模块导入成功")
except ImportError as e:
    logging.warning(f"NSGA算法模块导入失败: {e}")
    GeneticAlgorithm = None
    GASimulator = None

# 导入RAWCS算法模块
try:
    from PPO_MLP.action_unlimit import RAWCSPlanner, RAWCSSimulator
    logging.info("RAWCS算法模块导入成功")
except ImportError as e:
    logging.error(f"RAWCS算法模块导入失败: {e}")
    RAWCSPlanner = None
    RAWCSSimulator = None

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

app = Flask(__name__)

# 全局变量存储当前WSN仿真数据
current_wsn_data = None


# ========== 动态构建道路网络类 ==========
class DynamicRoadNetwork:
    """
    从Java后端传来的数据动态构建道路网络
    不依赖本地JSON文件
    """
    
    def __init__(self, road_graph_data: Dict[str, Any]):
        """
        初始化道路网络
        
        Args:
            road_graph_data: 包含 'nodes' 和 'edges' 的字典
                nodes: [{"id": "I1", "x": 113.xxx, "y": 23.xxx, "node_type": "intersection", ...}, ...]
                edges: [{"from": "I1", "to": "S1", "length_m": 123.45}, ...]
        """
        start_time = time.time()
        
        # 保存原始数据
        self.data = road_graph_data
        
        # 节点信息
        self.nodes = {}
        for node_data in road_graph_data.get('nodes', []):
            node_id = node_data['id']
            self.nodes[node_id] = {
                'node_id': node_id,
                'type': node_data.get('node_type', 'unknown'),
                'x': node_data.get('x', 0),
                'y': node_data.get('y', 0),
                'degree': node_data.get('degree', 0),
                'neighbors': node_data.get('neighbors', [])
            }
        
        # 构建完整的图邻接表（包含所有节点，用于大地图路径计算）
        self.full_graph = defaultdict(dict)
        for edge in road_graph_data.get('edges', []):
            u = edge['from']
            v = edge['to']
            length = edge.get('length_m', 0)  # 已经是米
            self.full_graph[u][v] = length
            self.full_graph[v][u] = length
        
        # 统计信息
        self.statistics = self._calculate_statistics()
        
        load_time = time.time() - start_time
        logger.info(f"动态加载道路网络成功: {load_time:.4f}s")
        logger.info(f"  总节点数: {self.statistics['total_nodes']}")
        logger.info(f"  道路段数: {self.statistics['total_edges']}")
    
    def _calculate_statistics(self) -> Dict:
        """计算统计信息"""
        node_types = {'intersection': 0, 'sensor': 0, 'relay': 0, 'station': 0, 'unknown': 0}
        for node in self.nodes.values():
            node_type = node['type']
            if node_type in node_types:
                node_types[node_type] += 1
            else:
                node_types['unknown'] += 1
        
        return {
            'total_nodes': len(self.nodes),
            'total_edges': sum(len(neighbors) for neighbors in self.full_graph.values()) // 2,
            'intersections': node_types.get('intersection', 0),
            'sensors': node_types.get('sensor', 0),
            'relays': node_types.get('relay', 0),
            'stations': node_types.get('station', 0)
        }
    
    def compute_shortest_paths_full(self, node_ids: List[str]):
        """
        在完整大地图上计算指定节点之间的最短路径距离
        这是 GreedyPathPlanner 需要的关键方法
        """
        start_time = time.time()
        n = len(node_ids)
        node_to_idx = {node_id: i for i, node_id in enumerate(node_ids)}
        dist_matrix = np.full((n, n), float('inf'), dtype=np.float32)
        
        # 对每个节点运行 Dijkstra
        for i, start_id in enumerate(node_ids):
            dist_matrix[i, i] = 0.0
            
            # 使用字典存储距离，只存储图中的节点
            distances = {}
            for node in self.nodes:
                distances[node] = float('inf')
            distances[start_id] = 0
            pq = [(0, start_id)]
            visited = set()
            
            while pq:
                current_dist, current = heapq.heappop(pq)
                if current in visited:
                    continue
                visited.add(current)
                
                # 确保当前节点在 full_graph 中
                if current not in self.full_graph:
                    continue
                    
                for neighbor, length in self.full_graph[current].items():
                    new_dist = current_dist + length
                    if new_dist < distances.get(neighbor, float('inf')):
                        distances[neighbor] = new_dist
                        heapq.heappush(pq, (new_dist, neighbor))
            
            for j, end_id in enumerate(node_ids):
                if end_id in distances:
                    dist_matrix[i, j] = distances[end_id]
        
        elapsed = time.time() - start_time
        logger.debug(f"  最短路径计算完成: {elapsed:.4f}s, 节点数: {n}")
        
        return dist_matrix, node_to_idx
    
    def get_path_between_full(self, start_id: str, end_id: str) -> List[str]:
        """在完整大地图上获取两个节点之间的最短路径节点序列"""
        if start_id == end_id:
            return [start_id]
        
        # 确保节点存在于图中
        if start_id not in self.nodes or end_id not in self.nodes:
            logger.warning(f"节点不在图中: start={start_id}, end={end_id}")
            return [start_id, end_id]
        
        # 使用字典存储距离
        distances = {}
        previous = {}
        for node in self.nodes:
            distances[node] = float('inf')
            previous[node] = None
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
            
            # 确保当前节点在 full_graph 中
            if current not in self.full_graph:
                continue
                
            for neighbor, length in self.full_graph[current].items():
                new_dist = current_dist + length
                if new_dist < distances.get(neighbor, float('inf')):
                    distances[neighbor] = new_dist
                    previous[neighbor] = current
                    heapq.heappush(pq, (new_dist, neighbor))
        
        # 重建路径
        path = []
        current = end_id
        while current is not None:
            path.append(current)
            current = previous.get(current)
        
        path.reverse()
        
        # 如果找不到路径，返回起点和终点
        if len(path) < 2 or path[0] != start_id:
            logger.warning(f"找不到从 {start_id} 到 {end_id} 的路径，返回直接连接")
            return [start_id, end_id]
        
        return path
    
    def get_node_by_id(self, node_id: str) -> Dict:
        """根据节点ID获取节点信息"""
        if node_id in self.nodes:
            return self.nodes[node_id]
        return None

def load_road_network_from_request(request_data: Dict[str, Any]):
    """
    从请求数据中动态构建道路网络
    不依赖本地JSON文件
    """
    # 请求数据中应该包含 nodes 和 edges
    # 如果数据中没有，尝试从全局的 WSN 数据中获取
    if 'nodes' not in request_data or 'edges' not in request_data:
        # 尝试从全局的 WSN 数据中构建完整的道路图
        # 注意：这里需要 Java 后端在调用时把完整的道路图数据传过来
        raise ValueError("请求数据中缺少道路网络数据（nodes 和 edges），请确保 Java 后端传入了完整的图数据")
    
    return DynamicRoadNetwork(request_data)

def convert_point_info_to_algorithm_format(point_info_list):
    """
    将point_info转换为算法模块期望的格式
    注意：Java后端传来的point_info已经包含米制坐标(x,y)，直接使用即可
    """
    converted = []
    for point in point_info_list:
        node_type = point.get("type")
        
        # 直接使用已有的米制坐标（Java后端已经转换好了）
        x = point.get("x", 0)
        y = point.get("y", 0)
        
        if x == 0 and y == 0:
            # 如果没有米制坐标，尝试从经纬度转换（兼容性）
            lng = point.get("lng") or point.get("longitude", 0)
            lat = point.get("lat") or point.get("latitude", 0)
            print(f"警告: 节点 {point.get('node_id')} 缺少米制坐标，从经纬度转换")
            import math
            ref_lon = 113.34823
            ref_lat = 23.15829
            EARTH_RADIUS = 6371000
            lat_rad = math.radians(lat)
            ref_lat_rad = math.radians(ref_lat)
            x = EARTH_RADIUS * math.cos(ref_lat_rad) * math.radians(lng - ref_lon)
            y = EARTH_RADIUS * math.radians(lat - ref_lat)

        raw_battery = point.get("battery_info", {})
        converted_point = {
            "node_id": point.get("node_id"),
            "type": node_type,
            "x": float(x),  # 米制坐标 X
            "y": float(y),  # 米制坐标 Y
            "degree": point.get("degree", 0),
            "neighbors": point.get("neighbors", []),
            "battery_info": {
                "current_energy": raw_battery.get("current_energy", 0),
                "current_energy_ratio": raw_battery.get("current_energy_ratio", 1.0),
                "charging_rate": raw_battery.get("charging_rate", 0.98),
                "needs_charge": raw_battery.get("needs_charge", False),
                "total_battery": raw_battery.get("total_battery", 0)
            }
        }
        converted.append(converted_point)
    
    station_count = len([p for p in converted if p.get("type") == "station"])
    print(f"转换后: 共 {len(converted)} 个节点, 其中充电站 {station_count} 个")
    
    return converted

def convert_numpy_types(obj):
    """递归地将NumPy类型转换为Python标准类型"""
    if isinstance(obj, dict):
        return {k: convert_numpy_types(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [convert_numpy_types(elem) for elem in obj]
    elif isinstance(obj, (np.float32, np.float64, float)):
        return float(obj)
    elif isinstance(obj, (np.int32, np.int64, int)):
        return int(obj)
    elif isinstance(obj, np.ndarray):
        return obj.tolist()
    else:
        return obj


# ==================== WSN仿真API ====================

@app.route('/api/runWSN', methods=['POST'])
def run_wsn_api():
    """运行WSN仿真，返回仿真数据"""
    global current_wsn_data
    
    try:
        data = request.get_json()
        
        sensor_battery = data.get('sensor_battery', 10000.0)
        relay_battery = data.get('relay_battery', 40000.0)
        station_battery = data.get('station_battery', 1000000.0)
        num_sensors = data.get('num_sensors', 40)
        num_relays = data.get('num_relays', 5)
        seed = data.get('seed', 42)
        
        if WSNSimulation is None:
            return jsonify({
                "status": "error",
                "message": "WSN仿真模块不可用，请检查WSNsimulation.py文件是否存在"
            }), 500
        
        start_time = time.time()
        
        wsn_sim = WSNSimulation(
            data_input=data,
            sensor_battery=sensor_battery,
            relay_battery=relay_battery,
            station_battery=station_battery,
            num_sensors=num_sensors,
            num_relays=num_relays,
            seed=seed
        )
        
        sim_data = wsn_sim.get_data()
        stats = wsn_sim.get_statistics()
        graph_info = wsn_sim.get_graph_info()
        max_min = wsn_sim.get_max_min()
        
        end_time = time.time()
        
        current_wsn_data = {
            "graph_G": sim_data["graph_G"],
            "point_info": sim_data["point_info"],
            "max_min": max_min
        }
        
        # 构建返回给Java的数据
        converted_point_info = []
        needs_charge_count = 0
        
        for point in sim_data["point_info"]:
            battery_info = point.get("battery_info", {})
            needs_charge = battery_info.get("needs_charge", False)
            if needs_charge:
                needs_charge_count += 1
            
            converted_point = {
                "node_id": point["node_id"],
                "type": point["type"],
                "x": point["x"],
                "y": point["y"],
                "degree": point.get("degree", 0),
                "neighbors": point.get("neighbors", []),
                "battery_info": {
                    "current_energy": battery_info.get("current_energy", 0),
                    "current_energy_ratio": battery_info.get("current_energy_ratio", 1.0),
                    "charging_rate": battery_info.get("charging_rate", 0.98),
                    "needs_charge": needs_charge,
                    "total_battery": battery_info.get("total_battery", 0)
                },
                "needs_charge": needs_charge
            }
            converted_point_info.append(converted_point)
        
        stats['charging_nodes'] = needs_charge_count
        
        # 转换 graph_G 的键为字符串
        graph_G_str_keys = {}
        for k, v in sim_data["graph_G"].items():
            graph_G_str_keys[str(k)] = {str(nk): nv for nk, nv in v.items()}
        
        response_data = {
            "point_info": converted_point_info,
            "graph_G": graph_G_str_keys,
            "max_min": {
                "distance": max_min["distance"],
                "x": max_min["x"],
                "y": max_min["y"]
            },
            "stats": stats,
            "graph_info": graph_info,
            "execution_time": end_time - start_time
        }
        
        logger.info(f"WSN仿真完成，处理了 {len(converted_point_info)} 个节点，"
                   f"其中需要充电的节点: {needs_charge_count} 个，"
                   f"耗时 {end_time - start_time:.2f} 秒")
        
        return jsonify({
            "status": "success",
            "message": "WSN仿真完成",
            "data": convert_numpy_types(response_data)
        })
        
    except Exception as e:
        logger.error(f"WSN仿真API处理失败: {e}")
        traceback.print_exc()
        return jsonify({
            "status": "error",
            "message": f"WSN仿真失败: {str(e)}"
        }), 500



# ==================== 贪心算法API ====================
@app.route('/api/runGreedy', methods=['POST'])
def run_greedy():
    """运行贪心算法（使用动态构建的道路网络，不依赖JSON文件）"""
    global current_wsn_data
    
    try:
        data = request.get_json()
        if not data:
            data = current_wsn_data if current_wsn_data else {}
        
        raw_point_info = data.get('point_info', [])
        
        logger.info(f"接收到的point_info数量: {len(raw_point_info)}")
        
        graph_G = data.get('graph_G', {})
        max_min = data.get('max_min', {})
        
        # 从请求数据中获取完整的道路网络数据
        road_graph_data = data.get('road_graph_data', {})
        
        if not road_graph_data:
            logger.warning("请求中没有 road_graph_data，尝试从现有数据构建...")
            road_graph_data = _build_road_graph_from_point_info(raw_point_info, graph_G)
        
        if not road_graph_data.get('nodes') or not road_graph_data.get('edges'):
            return jsonify({
                "status": "error", 
                "message": "缺少道路网络数据，请确保 Java 后端传入了完整的 road_graph_data（包含 nodes 和 edges）"
            }), 400
        
        if GreedyPathPlanner is None:
            return jsonify({"status": "error", "message": "贪心算法模块不可用"}), 500
        
        logger.info("开始执行贪心算法...")
        
        # 转换 point_info 格式
        point_info = convert_point_info_to_algorithm_format(raw_point_info)
        
        # 动态构建道路网络（不依赖JSON文件）
        network = DynamicRoadNetwork(road_graph_data)
        logger.info(f"道路网络构建完成: {len(network.nodes)} 个节点, {len(network.full_graph)} 条边")
        
        # 创建贪心规划器
        planner = GreedyPathPlanner(network, point_info)
        plan = planner.get_plan()
        
        logger.info(f"规划结果: station_id={plan['station_id']}, total_nodes={plan['total_nodes']}, total_distance={plan['total_distance']}")
        
        # 使用 planner 的路径生成方法直接获取路径坐标
        full_path_coords_list = planner.get_full_path_coords()
        logger.info(f"生成路径坐标段数: {len(full_path_coords_list)}")
        
        # 调试：打印前几段路径坐标
        for idx, segment in enumerate(full_path_coords_list[:3]):
            logger.info(f"  段 {idx}: 起点={segment[0] if segment else 'None'}, 终点={segment[-1] if segment else 'None'}, 点数={len(segment)}")
        
        # 准备仿真数据
        wsn_data = {'graph_G': graph_G, 'point_info': point_info, 'max_min': max_min}
        
        # 创建仿真器
        simulator = GreedyUAVSimulator(wsn_data)
        
        # 关键修复：替换 simulator 中的道路网络为动态构建的网络
        simulator.road_network = network
        
        results = simulator.run_sequence(plan['complete_sequence'], plan['charging_sequence'])
        
        logger.info(f"仿真结果: total_steps={results['total_steps']}, total_distance={results['total_distance']}")
        
        # ========== 使用 planner 生成的路径坐标构建 action_log ==========
        complete_sequence = plan['complete_sequence']
        station_id = plan['station_id']
        
        # 获取节点类型映射
        node_type_map = {}
        for point in raw_point_info:
            node_type_map[point.get('node_id')] = point.get('type', 'unknown')
        
        # 构建 action_log
        action_log = []
        
        for i in range(1, len(complete_sequence)):
            target_id = complete_sequence[i]
            is_station = (target_id == station_id)
            
            # 获取这段路径的坐标
            path_coords_idx = i - 1
            if path_coords_idx < len(full_path_coords_list):
                path_coords = full_path_coords_list[path_coords_idx]
            else:
                # 如果 planner 的路径不够，尝试单独计算
                from_node = complete_sequence[i-1]
                to_node = target_id
                path_coords = planner.get_path_between_nodes(from_node, to_node)
            
            # 计算移动距离和时间
            from_idx = planner.node_to_idx.get(complete_sequence[i-1])
            to_idx = planner.node_to_idx.get(target_id)
            if from_idx is not None and to_idx is not None:
                step_distance = float(planner.dist_matrix[from_idx, to_idx])
            else:
                step_distance = 0
            step_time = step_distance / 5.0  # 速度 5 m/s
            
            # 获取节点信息
            node_info = next((p for p in point_info if p.get('node_id') == target_id), None)
            node_type = node_type_map.get(target_id, 'unknown')
            
            # 计算充电时间
            charge_time = 0
            if not is_station and node_info:
                current_energy = node_info['battery_info'].get('current_energy', 0)
                charging_rate = node_info['battery_info'].get('charging_rate', 0.85)
                max_battery = 10000.0 if node_type == 'sensor' else 40000.0
                energy_needed = max_battery - current_energy
                effective_power = 100.0 * charging_rate
                charge_time = energy_needed / effective_power if effective_power > 0 else 0
            
            # 确保 path_coords 是有效的坐标列表
            if not path_coords or len(path_coords) < 2:
                logger.warning(f"步骤 {i}: 路径坐标无效，使用备用方案")
                # 备用方案：使用起点和终点的直线连接
                from_node_info = next((p for p in point_info if p.get('node_id') == complete_sequence[i-1]), None)
                to_node_info = next((p for p in point_info if p.get('node_id') == target_id), None)
                if from_node_info and to_node_info:
                    path_coords = [(from_node_info.get('x', 0), from_node_info.get('y', 0)),
                                   (to_node_info.get('x', 0), to_node_info.get('y', 0))]
            
            action_log.append({
                "step": i,
                "uav_to": target_id,
                "destination_type": "station" if is_station else node_type,
                "path_nodes": path_coords,
                "move_time": step_time,
                "charge_time": charge_time,
                "distance": step_distance
            })
        
        logger.info(f"构建的 action_log 条目数: {len(action_log)}")
        
        # 验证 action_log 中的路径坐标
        valid_path_count = 0
        for idx, log_entry in enumerate(action_log[:5]):
            path_nodes = log_entry.get('path_nodes', [])
            if path_nodes and len(path_nodes) >= 2:
                valid_path_count += 1
                logger.info(f"  步骤 {idx+1}: {log_entry['uav_to']}, 路径点数: {len(path_nodes)}, 起点=({path_nodes[0][0]:.6f},{path_nodes[0][1]:.6f}), 终点=({path_nodes[-1][0]:.6f},{path_nodes[-1][1]:.6f})")
            else:
                logger.warning(f"  步骤 {idx+1}: 路径坐标无效或点数不足, path_nodes={path_nodes}")
        
        if valid_path_count == 0:
            logger.error("所有路径坐标都无效！前端将无法显示路线")
        
        response_data = {
            "point_info": raw_point_info,
            "action_log": action_log,
            "metrics": {
                'total_reward': float(results['total_reward']),
                "total_distance_m": float(results['total_distance']),
                "total_distance_km": float(results['total_distance'] / 1000),
                "total_steps": int(results['total_steps']),
                "charged_count": int(results['charged_count']),
                "total_nodes": int(results['total_nodes']),
                "num_trips": int(results['num_trips']),
                "eue": float(results['eue']),
                "charge_energy_j": float(results['charge_energy']),
                "move_energy_j": float(results['move_energy']),
                "tx_energy_j": float(results['tx_energy']),
            },
            "greedy_sequence": plan['charging_sequence'],
            "total_planned_distance": float(plan['total_distance'])
        }
        
        logger.info(f"贪心算法完成: 总距离={results['total_distance']:.2f}m")

        print(f"贪心算法完成: 总距离={results['total_distance']:.2f}m,EUE={results['eue']:.2f},reward={results['total_reward']:.2f}")

        
        return jsonify({"status": "success", "message": "贪心算法执行完成", "data": convert_numpy_types(response_data)})
        
    except Exception as e:
        logger.error(f"贪心算法执行失败: {e}")
        traceback.print_exc()
        return jsonify({"status": "error", "message": str(e)}), 500

def _build_road_graph_from_point_info(point_info: List[Dict], graph_G: Dict) -> Dict:
    """
    从 point_info 和 graph_G 构建简化的道路网络
    这是一个备用方案，建议 Java 后端直接传入完整的图数据
    """
    nodes = []
    edges = []
    
    # 构建节点
    node_ids = set()
    for point in point_info:
        node_id = point.get('node_id')
        if node_id and node_id not in node_ids:
            node_ids.add(node_id)
            nodes.append({
                'id': node_id,
                'node_type': point.get('type', 'unknown'),
                'x': point.get('lng') or point.get('x', 0),
                'y': point.get('lat') or point.get('y', 0),
                'degree': len(graph_G.get(node_id, {})),
                'neighbors': list(graph_G.get(node_id, {}).keys())
            })
    
    # 构建边
    edge_set = set()
    for from_id, neighbors in graph_G.items():
        for to_id, length in neighbors.items():
            edge_key = f"{from_id}|{to_id}"
            if edge_key not in edge_set and f"{to_id}|{from_id}" not in edge_set:
                edge_set.add(edge_key)
                edges.append({
                    'from': from_id,
                    'to': to_id,
                    'length_m': length
                })
    
    logger.warning(f"备用方案构建的道路网络: {len(nodes)} 个节点, {len(edges)} 条边")
    
    return {
        'nodes': nodes,
        'edges': edges
    }


# # ==================== ACO算法API ====================
# @app.route('/api/runACO', methods=['POST'])
# def run_aco():
#     """运行改进蚁群算法"""
#     global current_wsn_data
    
#     try:
#         data = request.get_json()
#         if not data:
#             data = current_wsn_data if current_wsn_data else {}
        
#         raw_point_info = data.get('point_info', [])
#         graph_G = data.get('graph_G', {})
#         max_min = data.get('max_min', {})
        
#         # 获取道路网络数据
#         road_graph_data = data.get('road_graph_data', {})
        
#         if not road_graph_data.get('nodes') or not road_graph_data.get('edges'):
#             return jsonify({
#                 "status": "error", 
#                 "message": "缺少道路网络数据"
#             }), 400
        
#         if ImprovedAntColonyOptimizer is None or ACOSimulator is None:
#             return jsonify({"status": "error", "message": "ACO算法模块不可用"}), 500
        
#         logger.info("开始执行改进蚁群算法...")
        
#         # 转换 point_info 格式
#         point_info = convert_point_info_to_algorithm_format(raw_point_info)
        
#         # 动态构建道路网络
#         network = DynamicRoadNetwork(road_graph_data)
        
        
#         # 获取ACO参数
#         aco_params = data.get('aco_params', {})
        
#         # 创建 ACO 优化器
#         optimizer = ImprovedAntColonyOptimizer(
#             network=network,  # 传入 DynamicRoadNetwork
#             charging_node_info=point_info,
#             num_ants=aco_params.get('num_ants', 60),
#             max_iterations=aco_params.get('max_iterations', 400),
#             alpha0=aco_params.get('alpha0', 0.7),
#             alpha1=aco_params.get('alpha1', 1.8),
#             K=aco_params.get('K', 0.008),
#             beta=aco_params.get('beta', 1.5),
#             gamma=aco_params.get('gamma', 0.5),
#             rho=aco_params.get('rho', 0.02),
#             Q=aco_params.get('Q', 200.0),
#             tau_min=aco_params.get('tau_min', 0.0005),
#             tau_max=aco_params.get('tau_max', 15.0),
#             distance_threshold=aco_params.get('distance_threshold', 6000.0),
#             elite_ant_ratio=aco_params.get('elite_ant_ratio', 0.25),
#             exploration_rate=aco_params.get('exploration_rate', 0.2),
#             stagnation_limit=aco_params.get('stagnation_limit', 100)
#         )
        
#         # 执行优化
#         plan = optimizer.optimize(verbose=True)
        
#         # 准备仿真数据
#         wsn_data = {'graph_G': graph_G, 'point_info': point_info, 'max_min': max_min}
#         simulator = ACOSimulator(wsn_data)
#         results = simulator.run_sequence(plan['complete_sequence'], plan['charging_sequence'])
        
#         # 获取路径坐标（使用新增的方法）
#         path_coords_list = optimizer.get_full_path_coords()
        
#         # 构建 action_log
#         action_log = []
#         complete_sequence = plan['complete_sequence']
#         station_id = plan['station_id']
        
#         for i in range(1, len(complete_sequence)):
#             target_id = complete_sequence[i]
#             is_station = (target_id == station_id)
            
#             # 获取路径坐标
#             path_coords = path_coords_list[i-1] if i-1 < len(path_coords_list) else []
            
#             # 计算移动时间（使用图中实际距离）
#             # 从 dist_matrix 获取距离
#             from_idx = optimizer.node_to_idx.get(complete_sequence[i-1])
#             to_idx = optimizer.node_to_idx.get(target_id)
#             if from_idx is not None and to_idx is not None:
#                 step_distance = optimizer.dist_matrix[from_idx, to_idx]
#             else:
#                 step_distance = 0
#             step_time = step_distance / 5.0
            
#             # 获取节点信息
#             node_info = next((p for p in point_info if p.get('node_id') == target_id), None)
#             node_type = node_info.get('type', 'unknown') if node_info else 'unknown'
            
#             # 计算充电时间
#             charge_time = 0
#             if not is_station and node_info:
#                 current_energy = node_info['battery_info'].get('current_energy', 0)
#                 charging_rate = node_info['battery_info'].get('charging_rate', 0.85)
#                 max_battery = 240000.0 if node_type == 'sensor' else 800000.0
#                 energy_needed = max_battery - current_energy
#                 effective_power = 100.0 * charging_rate
#                 charge_time = energy_needed / effective_power if effective_power > 0 else 0
            
#             action_log.append({
#                 "step": i,
#                 "uav_to": target_id,
#                 "destination_type": "station" if is_station else node_type,
#                 "path_nodes": path_coords,
#                 "move_time": step_time,
#                 "charge_time": charge_time
#             })
        
#         response_data = {
#             "point_info": raw_point_info,
#             "action_log": action_log,
#             "metrics": {
#                 'total_reward': float(results['total_reward']),
#                 "total_distance_m": float(results['total_distance']),
#                 "total_distance_km": float(results['total_distance'] / 1000),
#                 "total_steps": int(results['total_steps']),
#                 "charged_count": int(results['charged_count']),
#                 "total_nodes": int(results['total_nodes']),
#                 "num_trips": int(results['num_trips']),
#                 "eue": float(results['eue']),
#                 "charge_energy_j": float(results['charge_energy']),
#                 "move_energy_j": float(results['move_energy']),
#                 "tx_energy_j": float(results['tx_energy']),
#                 "optimization_time": float(plan['optimization_time']),
#                 "iterations": int(plan['iterations']),
#                 "best_energy_metric": float(plan.get('best_energy_metric', 0))
#             },
#             "aco_sequence": plan['charging_sequence'],
#             "total_planned_distance": float(plan['total_distance'])
#         }
        
#         logger.info(f"ACO算法完成: 总距离={results['total_distance']:.2f}m, 迭代次数={plan['iterations']}")
#         print(f"ACO算法完成: 总距离={results['total_distance']:.2f}m,EUE={results['eue']:.2f},reward={results['total_reward']:.2f}")
        
#         return jsonify({"status": "success", "message": "ACO算法执行完成", "data": convert_numpy_types(response_data)})
        
#     except Exception as e:
#         logger.error(f"ACO算法执行失败: {e}")
#         traceback.print_exc()
#         return jsonify({"status": "error", "message": str(e)}), 500


# ==================== NSGA算法API ====================
@app.route('/api/runNSGA', methods=['POST'])
def run_nsga():
    """运行NSGA多目标遗传算法"""
    global current_wsn_data
    
    try:
        data = request.get_json()
        if not data:
            data = current_wsn_data if current_wsn_data else {}
        
        raw_point_info = data.get('point_info', [])
        graph_G = data.get('graph_G', {})
        max_min = data.get('max_min', {})
        
        # 获取道路网络数据
        road_graph_data = data.get('road_graph_data', {})
        
        if not road_graph_data.get('nodes') or not road_graph_data.get('edges'):
            return jsonify({
                "status": "error", 
                "message": "缺少道路网络数据"
            }), 400
        
        
        logger.info("开始执行NSGA多目标遗传算法...")
        
        # 转换 point_info 格式
        point_info = convert_point_info_to_algorithm_format(raw_point_info)
        
        # 动态构建道路网络
        network = DynamicRoadNetwork(road_graph_data)
        
        # 获取NSGA参数（从请求中获取，如果没有则使用默认值）
        nsga_params = data.get('nsga_params', {})
        
        # 创建 NSGA 优化器
        optimizer = GeneticAlgorithm(
            network=network,
            charging_node_info=point_info,
            population_size=nsga_params.get('population_size', 100),
            max_iterations=nsga_params.get('max_iterations', 300),
            crossover_rate=nsga_params.get('crossover_rate', 0.9),
            mutation_rate=nsga_params.get('mutation_rate', 0.1)
        )
        
        # 执行优化
        plan = optimizer.optimize(verbose=True)
        
        # 准备仿真数据
        wsn_data = {'graph_G': graph_G, 'point_info': point_info, 'max_min': max_min}
        simulator = GASimulator(wsn_data)
        results = simulator.run_sequence(plan['complete_sequence'], plan['charging_sequence'])
        
        # 获取路径坐标
        path_coords_list = optimizer.get_full_path_coords(plan['complete_sequence'])
        
        # 构建 action_log
        action_log = []
        complete_sequence = plan['complete_sequence']
        station_id = plan['station_id']
        
        for i in range(1, len(complete_sequence)):
            target_id = complete_sequence[i]
            is_station = (target_id == station_id)
            
            # 获取路径坐标
            path_coords = path_coords_list[i-1] if i-1 < len(path_coords_list) else []
            
            # 计算移动距离和时间
            from_idx = optimizer.node_to_idx.get(complete_sequence[i-1])
            to_idx = optimizer.node_to_idx.get(target_id)
            if from_idx is not None and to_idx is not None:
                step_distance = float(optimizer.dist_matrix[from_idx, to_idx])
            else:
                step_distance = 0
            step_time = step_distance / 5.0
            
            # 获取节点信息
            node_info = next((p for p in point_info if p.get('node_id') == target_id), None)
            node_type = node_info.get('type', 'unknown') if node_info else 'unknown'
            
            # 计算充电时间
            charge_time = 0
            if not is_station and node_info:
                current_energy = node_info['battery_info'].get('current_energy', 0)
                charging_rate = node_info['battery_info'].get('charging_rate', 0.85)
                max_battery = 10000.0 if node_type == 'sensor' else 40000.0
                energy_needed = max_battery - current_energy
                effective_power = 100.0 * charging_rate
                charge_time = energy_needed / effective_power if effective_power > 0 else 0
            
            action_log.append({
                "step": i,
                "uav_to": target_id,
                "destination_type": "station" if is_station else node_type,
                "path_nodes": path_coords,
                "move_time": step_time,
                "charge_time": charge_time,
                "distance": step_distance
            })
        
        response_data = {
            "point_info": raw_point_info,
            "action_log": action_log,
            "metrics": {
                'total_reward': float(results['total_reward']),
                "total_distance_m": float(results['total_distance']),
                "total_distance_km": float(results['total_distance'] / 1000),
                "total_steps": int(results['total_steps']),
                "charged_count": int(results['charged_count']),
                "total_nodes": int(results['total_nodes']),
                "num_trips": int(results['num_trips']),
                "eue": float(results['eue']),
                "charge_energy_j": float(results['charge_energy']),
                "move_energy_j": float(results['move_energy']),
                "tx_energy_j": float(results['tx_energy']),
                "optimization_time": float(plan.get('optimization_time', 0)),
                "iterations": int(plan.get('iterations', 0)),
                "best_distance": float(plan.get('total_distance', 0))
            },
            "nsga_sequence": plan['charging_sequence'],
            "total_planned_distance": float(plan['total_distance'])
        }
        
        logger.info(f"NSGA算法完成: 总距离={results['total_distance']:.2f}m, 迭代次数={plan.get('iterations', 0)}")
        print(f"NSGA算法完成: 总距离={results['total_distance']:.2f}m, EUE={results['eue']:.2f}, reward={results['total_reward']:.2f}")
        
        return jsonify({"status": "success", "message": "NSGA算法执行完成", "data": convert_numpy_types(response_data)})
        
    except Exception as e:
        logger.error(f"NSGA算法执行失败: {e}")
        traceback.print_exc()
        return jsonify({"status": "error", "message": str(e)}), 500

# ==================== RAWCS算法API ====================
@app.route('/api/runRAWCS', methods=['POST'])
def run_rawcs():
    """运行RAWCS算法（使用预训练PPO模型）"""
    global current_wsn_data
    
    try:
        data = request.get_json()
        if not data:
            data = current_wsn_data if current_wsn_data else {}
        
        raw_point_info = data.get('point_info', [])
        graph_G = data.get('graph_G', {})
        max_min = data.get('max_min', {})
        
        # 获取道路网络数据
        road_graph_data = data.get('road_graph_data', {})
        
        if not road_graph_data.get('nodes') or not road_graph_data.get('edges'):
            return jsonify({
                "status": "error", 
                "message": "缺少道路网络数据"
            }), 400
        
        # 获取数据集名称，用于加载对应的模型
        dataset_name = data.get('dataset_name', 'default')
        
        logger.info(f"开始执行RAWCS算法，数据集: {dataset_name}...")
        
        # 转换 point_info 格式
        point_info = convert_point_info_to_algorithm_format(raw_point_info)
        
        # 动态构建道路网络
        network = DynamicRoadNetwork(road_graph_data)
        
        # 创建RAWCS规划器
        rawcs_planner = RAWCSPlanner(
            network=network,
            charging_node_info=point_info,
            dataset_name=dataset_name,
            max_min=max_min  
        )
        
        # 执行规划
        plan = rawcs_planner.get_plan()
        
        # 准备仿真数据
        wsn_data = {'graph_G': graph_G, 'point_info': point_info, 'max_min': max_min}
        
        # 创建仿真器
        simulator = RAWCSSimulator(wsn_data, network)
        
        results = simulator.run_sequence(plan['complete_sequence'], plan['charging_sequence'])
        
        # 获取路径坐标
        path_coords_list = rawcs_planner.get_full_path_coords()
        
        # 构建 action_log
        action_log = []
        complete_sequence = plan['complete_sequence']
        station_id = plan['station_id']
        
        # 获取节点类型映射
        node_type_map = {}
        for point in raw_point_info:
            node_type_map[point.get('node_id')] = point.get('type', 'unknown')
        
        for i in range(1, len(complete_sequence)):
            target_id = complete_sequence[i]
            is_station = (target_id == station_id)
            
            # 获取路径坐标
            path_coords = path_coords_list[i-1] if i-1 < len(path_coords_list) else []
            
            # 计算移动距离和时间
            from_idx = rawcs_planner.node_to_idx.get(complete_sequence[i-1])
            to_idx = rawcs_planner.node_to_idx.get(target_id)
            if from_idx is not None and to_idx is not None:
                step_distance = float(rawcs_planner.dist_matrix[from_idx, to_idx])
            else:
                step_distance = 0
            step_time = step_distance / 5.0
            
            # 获取节点信息
            node_info = next((p for p in point_info if p.get('node_id') == target_id), None)
            node_type = node_type_map.get(target_id, 'unknown')
            
            # 计算充电时间
            charge_time = 0
            if not is_station and node_info:
                current_energy = node_info['battery_info'].get('current_energy', 0)
                charging_rate = node_info['battery_info'].get('charging_rate', 0.85)
                max_battery = 10000.0 if node_type == 'sensor' else 40000.0
                energy_needed = max_battery - current_energy
                effective_power = 100.0 * charging_rate
                charge_time = energy_needed / effective_power if effective_power > 0 else 0
            
            # 确保 path_coords 是有效的坐标列表（米制）
            if not path_coords or len(path_coords) < 2:
                logger.warning(f"步骤 {i}: 路径坐标无效，使用备用方案")
                from_node_info = next((p for p in point_info if p.get('node_id') == complete_sequence[i-1]), None)
                to_node_info = next((p for p in point_info if p.get('node_id') == target_id), None)
                if from_node_info and to_node_info:
                    path_coords = [(from_node_info.get('x', 0), from_node_info.get('y', 0)),
                                   (to_node_info.get('x', 0), to_node_info.get('y', 0))]
            
            action_log.append({
                "step": i,
                "uav_to": target_id,
                "destination_type": "station" if is_station else node_type,
                "path_nodes": path_coords,
                "move_time": step_time,
                "charge_time": charge_time,
                "distance": step_distance
            })
        
        response_data = {
            "point_info": raw_point_info,
            "action_log": action_log,
            "metrics": {
                'total_reward': float(results['total_reward']),
                "total_distance_m": float(results['total_distance']),
                "total_distance_km": float(results['total_distance'] / 1000),
                "total_steps": int(results['total_steps']),
                "charged_count": int(results['charged_count']),
                "total_nodes": int(results['total_nodes']),
                "num_trips": int(results['num_trips']),
                "eue": float(results['eue']),
                "charge_energy_j": float(results['charge_energy']),
                "move_energy_j": float(results['move_energy']),
                "tx_energy_j": float(results['tx_energy']),
                "inference_time_ms": float(results.get('inference_time_ms', 0))
            },
            "rawcs_sequence": plan['charging_sequence'],
            "total_planned_distance": float(plan['total_distance']),
            "dataset_name": dataset_name
        }
        
        logger.info(f"RAWCS算法完成: 总距离={results['total_distance']:.2f}m, EUE={results['eue']:.2f}")
        print(f"RAWCS算法完成: 总距离={results['total_distance']:.2f}m, EUE={results['eue']:.2f}, reward={results['total_reward']:.2f}")
        
        return jsonify({"status": "success", "message": "RAWCS算法执行完成", "data": convert_numpy_types(response_data)})
        
    except Exception as e:
        logger.error(f"RAWCS算法执行失败: {e}")
        traceback.print_exc()
        return jsonify({"status": "error", "message": str(e)}), 500

if __name__ == '__main__':
    port = int(os.environ.get('FLASK_PORT', 5000))
    host = os.environ.get('FLASK_HOST', '127.0.0.1')
    
    logger.info(f"启动统一算法API服务: http://{host}:{port}")
    logger.info("可用API端点:")
    logger.info("  POST /api/runWSN - WSN仿真")
    logger.info("  POST /api/runGreedy - 贪心算法")
    
    app.run(host=host, port=port, debug=True)
