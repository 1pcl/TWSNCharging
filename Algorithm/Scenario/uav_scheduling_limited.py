"""
无人车有限电池容量版本 - 电量不足会自动返回充电站充电
特点：
- 电池容量适中（可配置）
- 需要多次往返充电站给所有节点充电
- 最大步数较大（节点数的2倍）
- 保持原始reward计算逻辑不变
- 在充电站时，累计能量但不在中间计算最终奖励
"""

import numpy as np
import gymnasium as gym
from gymnasium import spaces
import os
import sys
import random
import time

# --- 路径配置 ---
current_script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_script_dir)
if project_root not in sys.path:
    sys.path.insert(0, project_root)

try:
    from Scenario.WSNsimulation import WSNSimulation
except ImportError as e:
    print(f"导入自定义模块失败: {e}")
    sys.exit(1)


class UAVSchedulingLimitedEnv(gym.Env):
    """
    无人车有限电池容量版本 - 需要多次往返充电
    """
    
    def __init__(self, 
                 graph_G,
                 point_info,
                 max_min=None,
                 seed=24,
                 agent_max_battery: float = 5000000.0,  # 适中的电池容量
                 agent_move_consumption: float = 50.0,
                 agent_tx_consumption: float = 100,
                 agent_speed: float = 2,
                 min_charging_rate: float = 0.70,
                 sensor_max_battery: float = 10000.0,
                 gateway_max_battery: float = 40000.0,
                 charge_time_levels: int = 100,
                 is_train: bool = False,
                 json_data: str = None,
                 station_charge_time: float = 100.0,  # 新增：充电站充电固定时间
                 ):  
        
        super().__init__()

        self.type = seed
        
        # 1. 存储参数
        self.agent_max_battery = agent_max_battery
        self.agent_move_consumption = agent_move_consumption
        self.agent_tx_consumption = agent_tx_consumption
        self.agent_speed = agent_speed
        self.sensor_max_battery = sensor_max_battery
        self.min_charging_rate = min_charging_rate
        self.gateway_max_battery = gateway_max_battery
        self.charge_time_levels = charge_time_levels
        self.station_charge_time = station_charge_time  # 充电站充电固定时间

        self.is_train = is_train
        self.json_data = json_data

        # 当前状态
        self.current_time = 0
        self.current_step = 0
        self.node_charge_count = 0 
        self.return_to_station_count = 0  # 记录返回充电站的次数

        # 能量 - 累积整个episode的总能量
        self.tx_energy = 0.0
        self.move_energy = 0.0
        self.charg_energy = 0.0

        # 存储图数据
        self.graph_G = graph_G
        self.point_info = point_info
        self.max_min = max_min

        # 充电站充电速率（无人车自身充电）- 不再需要，改为固定时间充满
        self.station_charging_rate = 1
        self.gamma = 0.99

        # 初始化节点映射
        self._initialize_node_mappings()
        
        # 初始化无人车状态
        self._initialize_uav_state()
        
        # 初始化节点动态状态
        self._initialize_node_states()
        
        # 初始化距离矩阵
        self._initialize_distance_matrix()
        
        # 计算充电参数
        self._initialize_charging_parameters()
        
        # 定义观测和动作空间
        self._define_spaces()

        self.total_distance = 0.0
        self.total_overcharge =0.0
        self.cum_cost   = 0.0

        self._compute_norm_constants()
        self._compute_charge_target()

        # 不可达惩罚系数
        self.unreachable_penalty = -5.0
        
        self.max_steps = 2 * self.num_nodes

    def _initialize_node_mappings(self):
        """初始化节点映射关系"""
        x_coords = [p['x'] for p in self.point_info]
        y_coords = [p['y'] for p in self.point_info]
        
        self.x_min, self.x_max = min(x_coords), max(x_coords)
        self.y_min, self.y_max = min(y_coords), max(y_coords)
        self.x_range = max(self.x_max - self.x_min, 1e-6)
        self.y_range = max(self.y_max - self.y_min, 1e-6)        
        
        self.original_to_global_map = {}
        self.global_to_original_map = {}
        new_global_id_counter = 0

        for info in self.point_info:
            node_id = info.get("node_id")
            if node_id is None:
                continue

            node_type = info["type"]
            
            new_global_id = new_global_id_counter
            
            mapping_data = {
                "new_global_id": new_global_id,
                "original_id": node_id,
                "original_type": node_type,
                "x": info.get("x"),
                "y": info.get("y"),
                "battery_info": info.get("battery_info"),
                "point_data": info
            }
            
            self.original_to_global_map[node_id] = mapping_data 
            self.global_to_original_map[new_global_id] = mapping_data 
            
            new_global_id_counter += 1

        self.num_nodes = new_global_id_counter

        self.sensor_global_ids = [
            gid for gid, data in self.global_to_original_map.items() 
            if data["original_type"] == "sensor"
        ]
        self.relay_global_ids = [
            gid for gid, data in self.global_to_original_map.items() 
            if data["original_type"] == "relay"
        ]
        self.station_global_ids = [
            gid for gid, data in self.global_to_original_map.items() 
            if data["original_type"] == "station"
        ]
        
        self.node_global_ids = self.sensor_global_ids + self.relay_global_ids
        
        self.num_sensors = len(self.sensor_global_ids)
        self.num_relays = len(self.relay_global_ids)
        self.num_stations = len(self.station_global_ids)

    def _initialize_uav_state(self):
        """初始化无人车状态"""
        start_node_id = None
        for gid, data in self.global_to_original_map.items():
            if data["original_type"] == "station":
                start_node_id = data["new_global_id"]
                break
        
        if start_node_id is None:
            start_node_id = 0
        
        self.start_node_id = start_node_id
        self.station_node_id = start_node_id
        
        self.initial_uav_state = {
            "node_id": self.start_node_id,
            "remaining_battery": self.agent_max_battery
        }
        self.uav_state = self.initial_uav_state.copy()

    def _initialize_node_states(self):
        """初始化所有节点的动态状态"""
        self.dynamic_node_states = {}

        for global_id, mapping_data in self.global_to_original_map.items():
            node_type = mapping_data["original_type"]
            battery_info = mapping_data.get("battery_info", {})

            if node_type == "sensor" or node_type == "relay":
                initial_energy = battery_info.get("current_energy", 0)
                charging_rate = battery_info.get("charging_rate", 0.85)
                
                self.dynamic_node_states[global_id] = {
                    "current_energy": initial_energy,
                    "charging_rate": charging_rate,
                    "is_charge": False,
                    "is_mask": False,
                    "total_battery": battery_info.get("total_battery", 
                                                      self.sensor_max_battery if node_type == "sensor" else self.gateway_max_battery)
                }
            elif node_type == "station":
                self.dynamic_node_states[global_id] = {
                    "current_energy": self.agent_max_battery,
                    "charging_rate": self.station_charging_rate,
                    "is_charge": False,
                    "is_mask": False,
                    "total_battery": self.agent_max_battery
                }
            else:
                self.dynamic_node_states[global_id] = {
                    "current_energy": 0,
                    "charging_rate": 0,
                    "is_charge": False,
                    "is_mask": False,
                    "total_battery": 0
                }

    def _compute_all_pairs_shortest_paths(self):
        """使用 Floyd-Warshall 计算所有节点对的最短路径"""
        INF = float('inf')
        n = self.num_nodes
        
        dist_matrix = np.full((n, n), INF, dtype=np.float32)
        np.fill_diagonal(dist_matrix, 0.0)
        
        for original_u, neighbors in self.graph_G.items():
            global_u = self.original_to_global_map.get(original_u, {}).get("new_global_id")
            if global_u is None:
                continue
            for original_v, distance in neighbors.items():
                global_v = self.original_to_global_map.get(original_v, {}).get("new_global_id")
                if global_v is not None:
                    dist_matrix[global_u, global_v] = min(dist_matrix[global_u, global_v], distance)
        
        for k in range(n):
            for i in range(n):
                if dist_matrix[i, k] < INF:
                    for j in range(n):
                        if dist_matrix[k, j] < INF:
                            new_dist = dist_matrix[i, k] + dist_matrix[k, j]
                            if new_dist < dist_matrix[i, j]:
                                dist_matrix[i, j] = new_dist
        
        return dist_matrix

    # def _initialize_distance_matrix(self):
    #     """初始化距离矩阵"""
    #     self.node_distance = self._compute_all_pairs_shortest_paths()
    def _initialize_distance_matrix(self):
        """初始化距离矩阵"""
        self.node_distance = self._compute_all_pairs_shortest_paths()
        
        # 取所有可达节点对的距离（去掉对角线 0 和 inf）
        finite_d = self.node_distance[np.isfinite(self.node_distance)]
        finite_d = finite_d[finite_d > 0]
        
        if len(finite_d) > 0:
            # 方案 1：纯 min / max
            self.dist_min = float(finite_d.min())
            self.dist_max = float(finite_d.max())
            
            # # 方案 2：分位数，抗极端值
            # self.dist_min = float(np.percentile(finite_d, 5))
            # self.dist_max = float(np.percentile(finite_d, 95))
        else:
            self.dist_min = 0.0
            self.dist_max = 1.0
        
        self.dist_range = max(self.dist_max - self.dist_min, 1e-6)

    def _initialize_charging_parameters(self):
        """初始化充电参数"""
        self.tx_rate = self.min_charging_rate * self.agent_tx_consumption
        self.sensor_max_charge = self.sensor_max_battery / self.tx_rate
        self.gateway_max_charge = self.gateway_max_battery / self.tx_rate
        
        self.max_charge_time = max(self.sensor_max_charge, self.gateway_max_charge)
        
        self.sensor_charge_time_step = self.sensor_max_charge / self.charge_time_levels
        self.gateway_charge_time_step = self.gateway_max_charge / self.charge_time_levels

    def _define_spaces(self):
        """定义结构化的观测和动作空间"""
        self.observation_space = spaces.Dict({
            "observations": spaces.Box(
                low=0.0,
                high=1.0,
                shape=(self.num_nodes, 10), 
                dtype=np.float32
            ),
            "uav_state": spaces.Box(
                low=0.0,
                high=1.0,
                shape=(8,), 
                dtype=np.float32
            ),
            "action_mask": spaces.Box(
                low=0,
                high=1,
                shape=(self.num_nodes + self.charge_time_levels,),
                dtype=np.int8
            ),
            "last_node_idx": spaces.Box(
                low=0,
                high=self.num_nodes,
                shape=(1,),
                dtype=np.int64
            )
        })
        
        self.action_space = spaces.MultiDiscrete([
            self.num_nodes,          
            self.charge_time_levels  
        ])

    def _get_struct_obs(self):
        """获取结构化的观测值"""
        current_node_id = self.uav_state["node_id"]
        
        node_features = np.zeros((self.num_nodes, 10), dtype=np.float32)
        full_mask = self.action_masks()

        min_dist = self.max_min["distance"]["min"] if self.max_min else 0
        max_dist = self.max_min["distance"]["max"] if self.max_min else 1000
        dist_range = max(max_dist - min_dist, 1e-6)

        x_min = self.max_min["x"]["min"] if self.max_min else self.x_min
        x_range = max((self.max_min["x"]["max"] if self.max_min else self.x_max) - x_min, 1e-6)
        y_min = self.max_min["y"]["min"] if self.max_min else self.y_min
        y_range = max((self.max_min["y"]["max"] if self.max_min else self.y_max) - y_min, 1e-6)

        for i in range(self.num_nodes):
            state = self.dynamic_node_states[i]
            mapping = self.global_to_original_map[i]
            node_type = mapping["original_type"]
            
            norm_x = (mapping["x"] - x_min) / x_range
            norm_y = (mapping["y"] - y_min) / y_range
            
            dist = self.node_distance[current_node_id, i]
            dist_ratio = (dist - min_dist) / dist_range if dist < np.inf else 1.0
            
            type_code = [0.0, 0.0, 0.0]
            energy_ratio = 0.0
            is_charge = 1.0 if state.get("is_charge", False) else 0.0
            
            if node_type == "sensor":
                total = state.get("total_battery", self.sensor_max_battery)
                energy_ratio = state["current_energy"] / total if total > 0 else 0
                type_code = [1.0, 0.0, 0.0]
            elif node_type == "relay":
                total = state.get("total_battery", self.gateway_max_battery)
                energy_ratio = state["current_energy"] / total if total > 0 else 0
                type_code = [0.0, 1.0, 0.0]
            elif node_type == "station":
                energy_ratio = 1.0
                type_code = [0.0, 0.0, 1.0]
            
            charging_rate = state.get("charging_rate", 0.0)
            
            node_features[i, 0] = energy_ratio
            node_features[i, 1] = dist_ratio
            node_features[i, 2] = norm_x
            node_features[i, 3] = norm_y
            node_features[i, 4] = is_charge
            node_features[i, 5:8] = type_code
            node_features[i, 8] = 1.0 if state.get("is_mask", False) else 0.0
            node_features[i, 9] = charging_rate

        uav_x = self.global_to_original_map[current_node_id]["x"]
        uav_y = self.global_to_original_map[current_node_id]["y"]
        
        completion_ratio = self.node_charge_count / max(1, len(self.node_global_ids))
        return_ratio = min(1.0, self.return_to_station_count / max(1, len(self.node_global_ids)))
        
        uav_state = np.array([
            self.uav_state["remaining_battery"] / self.agent_max_battery,
            self.charg_energy / max(self.max_possible_charge, 1e-6),
            self.cum_cost   / max(self.max_possible_cost, 1e-6),
            self.total_distance / self.max_dist_norm,
            completion_ratio,
            return_ratio,
            (uav_x - x_min) / x_range,
            (uav_y - y_min) / y_range
        ], dtype=np.float32)

        action_mask = full_mask.astype(np.int8)

        return {
            "observations": node_features,
            "uav_state": uav_state,
            "action_mask": action_mask,
            "last_node_idx": np.array([current_node_id], dtype=np.int64)
        }
    def _compute_norm_constants(self):
        self.max_possible_charge = (
            self.num_sensors * self.sensor_max_battery
            + self.num_relays  * self.gateway_max_battery
        )
        self.max_possible_cost = (
            self.num_nodes * self.dist_max * self.agent_move_consumption
            + self.max_possible_charge
        )
        self.max_dist_norm = max(self.num_nodes * self.dist_max, 1e-6)
    def action_masks(self) -> np.ndarray:
        """
        有限电池版本的动作掩码
        规则：必须能到达目标节点、给节点充电、并返回充电站
        """
        node_mask = np.zeros(self.num_nodes, dtype=bool)
        level_mask = np.ones(self.charge_time_levels, dtype=bool)

        curr_id = self.uav_state["node_id"]
        curr_batt = self.uav_state["remaining_battery"]
        station_idx = self.station_node_id
        can_go_to_sensor = False

        for i in range(self.num_nodes):
            self.dynamic_node_states[i]["is_mask"] = False
            
            # 跳过充电站
            if i == station_idx:
                continue
            
            # 不能留在原地
            if i == curr_id:
                continue
            
            # 只选择未充电的节点（一次充电）
            if self.dynamic_node_states[i].get("is_charge", False):
                continue
            
            # 检查距离是否可达
            dist_to_node = self.node_distance[curr_id, i]
            dist_to_station = self.node_distance[i, station_idx]
            
            # 不可达检查
            if dist_to_node == float('inf') or dist_to_station == float('inf'):
                continue
            
            # ========== 关键修改：计算总所需能量（移动 + 充电） ==========
            # 1. 移动能耗（去节点 + 返回充电站）
            total_move_dist = dist_to_node + dist_to_station
            move_energy = total_move_dist * self.agent_move_consumption
            
            # 2. 充电能耗（给节点充满电所需的传输能量）
            # 获取节点类型和剩余能量
            node_state = self.dynamic_node_states[i]
            node_type = self.global_to_original_map[i]["original_type"]
            
            if node_type == "sensor":
                max_battery = self.sensor_max_battery
            elif node_type == "relay":
                max_battery = self.gateway_max_battery
            else:
                continue
            
            # 计算需要补充的电量
            current_energy = node_state["current_energy"]
            energy_needed = max_battery - current_energy
            
            # 充电能量 = 需要补充的电量 / 充电效率
            # 因为传输能耗 * 充电时间 = 充电能量，而充电能量 * 充电效率 = 节点获得能量
            # 所以：传输能量 = 节点获得能量 / 充电效率
            charging_rate = node_state["charging_rate"]
            if charging_rate > 0:
                charge_energy = energy_needed / charging_rate
            else:
                charge_energy = energy_needed  # fallback
            
            # 3. 总所需能量
            total_needed_energy = move_energy + charge_energy
            
            # 检查电量是否足够（预留10%安全裕度）
            safe_battery = curr_batt * 0.9
            if total_needed_energy <= safe_battery:
                node_mask[i] = True
                can_go_to_sensor = True
                self.dynamic_node_states[i]["is_mask"] = True

        if can_go_to_sensor:
            # 有传感器可去，屏蔽充电站
            node_mask[station_idx] = False
            self.dynamic_node_states[station_idx]["is_mask"] = False
        else:
            # 没有传感器可去，强制回站
            dist_to_station = self.node_distance[curr_id, station_idx]
            if dist_to_station != float('inf'):
                # 计算返回充电站所需的能量（只移动，不需要充电）
                move_energy_to_station = dist_to_station * self.agent_move_consumption
                if move_energy_to_station <= curr_batt:
                    # 只能去充电站
                    node_mask[:] = False
                    node_mask[station_idx] = True
                    self.dynamic_node_states[station_idx]["is_mask"] = True
                    # 在充电站，充电时间固定，只允许一个充电时间选项
                    level_mask[:] = False
                    level_mask[0] = True
                else:
                    # 连充电站都回不去，找最近的可达节点
                    min_dist = float('inf')
                    min_node = curr_id
                    for i in range(self.num_nodes):
                        if i == curr_id:
                            continue
                        dist = self.node_distance[curr_id, i]
                        if dist != float('inf') and dist < min_dist:
                            min_dist = dist
                            min_node = i
                    if min_node != curr_id:
                        node_mask[min_node] = True
                        self.dynamic_node_states[min_node]["is_mask"] = True
                    else:
                        # 真的没有任何节点可达，允许留在原地作为最后手段
                        node_mask[curr_id] = True
                        self.dynamic_node_states[curr_id]["is_mask"] = True
            else:
                # 充电站不可达
                node_mask[curr_id] = True
                self.dynamic_node_states[curr_id]["is_mask"] = True

        # 确保至少有一个充电时间可选
        if not np.any(level_mask):
            level_mask[0] = True

        return np.concatenate([node_mask, level_mask])

    def _decode_action(self, action):
        """解码动作"""
        action = np.array(action).flatten()
        
        target_node_id = int(action[0])
        charge_level = int(action[1])

        charge_time = charge_level
        node_type = self.global_to_original_map[target_node_id]["original_type"]

        # 如果是充电站，充电时间为0（使用固定充电时间）
        if node_type == "station":
            charge_time = 0.0
        elif node_type == "sensor":
            charge_time = (charge_level + 1) * self.sensor_charge_time_step
        elif node_type == "relay":
            charge_time = (charge_level + 1) * self.gateway_charge_time_step
        
        return {
            "target_node_id": target_node_id,
            "charge_time": np.array([charge_time], dtype=np.float32)
        }

    def reset(self, seed=None, options=None):
        """重置环境"""
        if seed is not None:
            super().reset(seed=seed)
            random.seed(seed)
            np.random.seed(seed)
        
        self.current_time = 0.0
        self.current_step = 0
        self.node_charge_count = 0
        self.return_to_station_count = 0
        
        # 重置所有能量统计
        self.total_distance = 0.0
        self.tx_energy = 0.0
        self.move_energy = 0.0
        self.charg_energy = 0.0
        self.total_overcharge = 0.0
        self.cum_cost   = 0.0

        
        if self.is_train and self.json_data:
            wsn_sim = WSNSimulation(self.json_data, seed=int(time.time()))
            data = wsn_sim.get_data()
            self.graph_G = data["graph_G"]
            self.point_info = data["point_info"]
            self.max_min = data.get("max_min")
            
            self._initialize_node_mappings()
            self._initialize_uav_state()
            self._initialize_node_states()
            self._initialize_distance_matrix()
            self._compute_norm_constants()   
            self._compute_charge_target()
        else:
            self.uav_state = self.initial_uav_state.copy()
            
            self.dynamic_node_states = {}
            for global_id, mapping_data in self.global_to_original_map.items():
                node_type = mapping_data["original_type"]
                battery_info = mapping_data.get("battery_info", {})
                
                if node_type == "sensor" or node_type == "relay":
                    initial_energy = battery_info.get("current_energy", 0)
                    charging_rate = battery_info.get("charging_rate", 0.85)
                    
                    self.dynamic_node_states[global_id] = {
                        "current_energy": initial_energy,
                        "charging_rate": charging_rate,
                        "is_charge": False,
                        "is_mask": False,
                        "total_battery": battery_info.get("total_battery", 
                                                          self.sensor_max_battery if node_type == "sensor" else self.gateway_max_battery)
                    }
                elif node_type == "station":
                    self.dynamic_node_states[global_id] = {
                        "current_energy": self.agent_max_battery,
                        "charging_rate": self.station_charging_rate,
                        "is_charge": False,
                        "is_mask": False,
                        "total_battery": self.agent_max_battery
                    }
                else:
                    self.dynamic_node_states[global_id] = {
                        "current_energy": 0,
                        "charging_rate": 0,
                        "is_charge": False,
                        "is_mask": False,
                        "total_battery": 0
                    }
           
        
        return self._get_struct_obs(), self._get_info()
    def _compute_charge_target(self):
        self.total_charge_needed = 0.0
        self.tx_needed_total = 0.0
        for gid in self.node_global_ids:
            state = self.dynamic_node_states[gid]
            delta = max(0.0, state["total_battery"] - state["current_energy"])
            rate = max(state["charging_rate"], 1e-6)
            self.total_charge_needed += delta
            self.tx_needed_total += delta / rate
        self.total_charge_needed = max(self.total_charge_needed, 1e-6)
        self.tx_needed_total = max(self.tx_needed_total, 1e-6)
    def _get_info(self):
        """获取环境信息"""
        return {
            "current_uav_id": self.uav_state["node_id"],
            "uav_battery": self.uav_state["remaining_battery"],
            "node_num": self.num_nodes - 1,
            "node_charge_count": self.node_charge_count,
            "total_nodes": len(self.node_global_ids),
            "return_to_station_count": self.return_to_station_count
        }
    def _phi(self):
        # dist_to_station = self.node_distance[self.uav_state["node_id"], self.station_node_id]
        phi_charge_progress = min(self.charg_energy / self.total_charge_needed, 1.0)
        # phi_dist = -(self.total_distance+dist_to_station) / self.max_dist_norm
        phi_dist = -(self.total_distance) / self.max_dist_norm
        phi_done = self.node_charge_count / max(1, len(self.node_global_ids))
        phi_overcharge = -self.total_overcharge / self.tx_needed_total
        # phi_battery = self.uav_state["remaining_battery"] / self.agent_max_battery
        # battery_ratio = self.uav_state["remaining_battery"] / self.agent_max_battery
        # phi_battery = -max(0.0, 0.3 - battery_ratio)   # 只在电量<30%时给负
        # return (20.0*phi_charge_progress + 20.0*phi_dist + 2.0*phi_done + 5.0*phi_overcharge + 5.0*phi_battery)
        return (20.0*phi_charge_progress + 20.0*phi_dist + 2.0*phi_done + 5.0*phi_overcharge)
    
    def step(self, action):
        """执行一步环境 - 保持原始reward逻辑不变"""

        phi_prev = self._phi() 

        action_dict = self._decode_action(action)
        target_node_id = action_dict["target_node_id"]
        action_charge_time = action_dict["charge_time"][0]
        
        reward = 0.0
        terminated = False
        truncated = False
        step_charge_energy = 0.0
        step_move_energy = 0.0
        step_tx_energy = 0.0
        step_move_time = 0.0
        step_charge_time = 0.0
        distance_needed = 0.0

        current_node_id = self.uav_state["node_id"]
        
        # 检查目标节点是否可达
        distance_needed = self.node_distance[current_node_id][target_node_id]
        if distance_needed == float('inf'):
            reward = self.unreachable_penalty
            truncated = True
            observation = self._get_struct_obs()
            info = self._get_info()
            info["error"] = "unreachable_node"
            print("unreachable_node")
            return observation, reward, terminated, truncated, info
        
        self.total_distance += distance_needed
        step_move_energy = distance_needed * self.agent_move_consumption
        step_move_time = distance_needed / self.agent_speed

        # 距离惩罚 - 保持原始逻辑
        # reward -= 1.0 * (distance_needed / self.max_min["distance"]["max"])
        
        # 执行移动
        if step_move_energy > self.uav_state["remaining_battery"]:
            reward = self.unreachable_penalty
            truncated = True
            observation = self._get_struct_obs()
            info = self._get_info()
            info["error"] = "insufficient_battery"
            return observation, reward, terminated, truncated, info
        
        self.uav_state["remaining_battery"] -= step_move_energy
        self.uav_state["node_id"] = target_node_id

        self.move_energy += step_move_energy
        
        node_type = self.global_to_original_map[target_node_id]["original_type"]
        self.current_step += 1
         
        # 如果在充电站
        if node_type == "station":
            # 记录返回充电站次数
            if current_node_id != self.station_node_id:
                self.return_to_station_count += 1
            
            # 在充电站，给无人车充满电（固定充电时间）
            # 计算需要补充的电量
            battery_needed = self.agent_max_battery - self.uav_state["remaining_battery"]
            if battery_needed > 0:
                # 使用固定充电时间
                actual_charge_time = self.station_charge_time
                # 恢复无人车电量到满
                self.uav_state["remaining_battery"] = self.agent_max_battery
            else:
                actual_charge_time = 0.0
            
            # 计算总时间（移动时间 + 充电时间）
            current_time_step = step_move_time + actual_charge_time
            self.current_time += current_time_step
            
            # 更新所有节点的能量消耗（在充电期间）
            for global_id in self.node_global_ids:
                node_state = self.dynamic_node_states[global_id]
                node_type_temp = self.global_to_original_map[global_id]["original_type"]
                coum_move_energy = current_time_step / (394 if node_type_temp == "sensor" else 197)
                node_state["current_energy"] = max(0.0, node_state["current_energy"] - coum_move_energy)
            
            # 检查是否所有节点都已充电
            if self.node_charge_count >= len(self.node_global_ids):
                terminated = True
                # # 计算最终奖励 - 使用原始reward计算逻辑
                # if self.uav_state["remaining_battery"] >= 0.0 and (self.tx_energy + self.move_energy) > 0.0:
                #     total_eue = self.charg_energy / (self.tx_energy + self.move_energy)
                #     reward += 1.0*self.num_nodes * total_eue
                #     reward += 1.0*((self.max_min["distance"]["max"]*self.current_step) / max(self.total_distance, 1))
                # else:
                #     reward -= 5.0
            # else:
            #     # 如果还没充完所有节点，给一个小惩罚鼓励尽快完成
            #     reward -= 0.5
                    
        elif node_type == "sensor" or node_type == "relay":
            node_state = self.dynamic_node_states[target_node_id]   

            max_battery = self.sensor_max_battery if node_type == "sensor" else self.gateway_max_battery
            
            # 节点能量消耗
            coum_move_energy = step_move_time / (394 if node_type == "sensor" else 197)
                      
            energy_needed = max_battery - node_state["current_energy"] + coum_move_energy        
                    
            net_tx_rate = self.agent_tx_consumption * node_state["charging_rate"]   
            time_to_full_node = energy_needed / net_tx_rate if net_tx_rate > 0 else 0
            step_charge_time = min(action_charge_time, time_to_full_node)  

            step_charge_energy = step_charge_time * net_tx_rate    
            step_tx_energy = action_charge_time * self.agent_tx_consumption   

            step_overcharge = self.agent_tx_consumption * max(0.0, action_charge_time - step_charge_time)
            self.total_overcharge += step_overcharge

            # 执行充电
            if not node_state["is_charge"]:
                node_state["is_charge"] = True
                self.node_charge_count += 1

            # 检查无人车电量
            if self.uav_state["remaining_battery"] < step_tx_energy and step_tx_energy > 0:
                reward -= self.unreachable_penalty
                truncated = True
                step_tx_energy = self.uav_state["remaining_battery"]
                step_charge_time = step_tx_energy / self.agent_tx_consumption
                step_charge_energy = step_charge_time * net_tx_rate
                self.uav_state["remaining_battery"] = 0.0
            else:                
                self.uav_state["remaining_battery"] -= step_tx_energy
            
            node_state["current_energy"] = min(node_state["current_energy"] + step_charge_energy - coum_move_energy, max_battery)

            # 补偿时间消耗
            current_time_step = step_move_time + action_charge_time
            coum_move_energy = current_time_step / (394 if node_type == "sensor" else 197)
            node_state["current_energy"] += coum_move_energy
        
            # 累积能量统计
            self.tx_energy += step_tx_energy
            self.charg_energy += step_charge_energy
                                            
            # 中间步骤的EUE奖励 - 保持原始逻辑
            # total_energy_consumed = step_move_energy + step_tx_energy
            # if total_energy_consumed > 0 and step_charge_energy > 0:
                # eue_reward = step_charge_energy / (total_energy_consumed)
                # reward += 1.0 * eue_reward

            # 更新全局时间和其他节点能量
            self.current_time += current_time_step
            
            for global_id in self.node_global_ids:
                if global_id == target_node_id:
                    continue
                node_state_temp = self.dynamic_node_states[global_id]
                node_type_temp = self.global_to_original_map[global_id]["original_type"]
                coum_move_energy_temp = current_time_step / (394 if node_type_temp == "sensor" else 197)
                node_state_temp["current_energy"] = max(0.0, node_state_temp["current_energy"] - coum_move_energy_temp)

        self.cum_cost = self.tx_energy + self.move_energy

        phi_next = self._phi()     
        reward += self.gamma * phi_next - phi_prev 

        if self.current_step >= self.max_steps and not terminated:
            truncated = True
            reward -= 5.0

        observation = self._get_struct_obs()
        info = self._get_info()
        info.update({
            "step_charge_energy": step_charge_energy,
            "step_move_energy": step_move_energy,
            "step_tx_energy": step_tx_energy,
            "node_charge_count": self.node_charge_count,
            "distance_needed": distance_needed,
        })
        
        return observation, reward, terminated, truncated, info

    def render(self):
        pass

    def close(self):
        pass
