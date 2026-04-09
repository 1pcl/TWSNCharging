"""
无人车无限电池容量版本 - 可以一次性充完所有节点
特点：
- 无人车电池容量极大（设置为无穷大）
- 最大步数较小（节点数的2倍）
- 不考虑电量限制，一次遍历所有节点
- 保持原始reward计算逻辑不变
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


class UAVSchedulingUnlimitedEnv(gym.Env):
    """
    无人车无限电池容量版本 - 结构化字典观测，适配非连通图
    """
    
    def __init__(self, 
                 graph_G,
                 point_info,
                 max_min=None,
                 seed=24,
                #  agent_max_battery: float = 1e12,  # 极大电池容量，实际无限
                 agent_move_consumption: float = 1000.0,
                 agent_tx_consumption: float = 100,
                 agent_speed: float = 5,
                 min_charging_rate: float = 0.70,
                 sensor_max_battery: float = 240000.0,
                 gateway_max_battery: float = 800000.0,
                 charge_time_levels: int = 100,
                 is_train: bool = False,
                 json_data: str = None,                 
                 ):  
        
        super().__init__()

        self.type = seed
        
        # 1. 存储参数
        # self.agent_max_battery = agent_max_battery
        self.agent_move_consumption = agent_move_consumption
        self.agent_tx_consumption = agent_tx_consumption
        self.agent_speed = agent_speed
        self.sensor_max_battery = sensor_max_battery
        self.min_charging_rate = min_charging_rate
        self.gateway_max_battery = gateway_max_battery
        self.charge_time_levels = charge_time_levels

        self.is_train = is_train
        self.json_data = json_data

        # 当前状态
        self.current_time = 0
        self.current_step = 0
        self.node_charge_count = 0 

        # 能量
        self.tx_energy = 0.0
        self.move_energy = 0.0
        self.charg_energy = 0.0
        self.active_energy = 0.0

        # 存储图数据
        self.graph_G = graph_G
        self.point_info = point_info
        self.max_min = max_min

        # 节点的特征
        self.node_features = 10

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

        # 基准线
        self.base_total_distance = 2600.0
        self.base_total_eue = 0.60     
        self.total_distance = 0.0
        
        # 不可达惩罚系数
        self.unreachable_penalty = -100.0
        
        # 无限电池版本：最大步数设为节点数的2倍
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

        if(self.num_relays==0):
            self.node_features=9

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
            # "remaining_battery": self.agent_max_battery
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
                    "current_energy": 1,
                    "charging_rate": 0.95,
                    "is_charge": False,
                    "is_mask": False,
                    "total_battery": 1
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

    def _initialize_distance_matrix(self):
        """初始化距离矩阵"""
        self.node_distance = self._compute_all_pairs_shortest_paths()

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
                shape=(self.num_nodes, self.node_features), 
                dtype=np.float32
            ),
            "uav_state": spaces.Box(
                low=0.0,
                high=1.0,
                shape=(3,),
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
        
        node_features = np.zeros((self.num_nodes, self.node_features), dtype=np.float32)
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
                if(self.node_features==10):
                    type_code = [1.0, 0.0, 0.0]
                else:
                    type_code = [1.0, 0.0]
            elif node_type == "relay":
                total = state.get("total_battery", self.gateway_max_battery)
                energy_ratio = state["current_energy"] / total if total > 0 else 0
                type_code = [0.0, 1.0, 0.0]
            elif node_type == "station":
                energy_ratio = 1.0
                if(self.node_features==10):
                    type_code = [0.0, 0.0, 1.0]
                else:
                    type_code = [0.0, 1.0]
            
            charging_rate = state.get("charging_rate", 0.0)
            
            node_features[i, 0] = energy_ratio
            node_features[i, 1] = dist_ratio
            node_features[i, 2] = norm_x
            node_features[i, 3] = norm_y
            node_features[i, 4] = is_charge
            if(self.node_features==10):
                node_features[i, 5:8] = type_code
                node_features[i, 8] = 1.0 if state.get("is_mask", False) else 0.0
                node_features[i, 9] = charging_rate
            else:
                node_features[i, 5:7] = type_code
                node_features[i, 7] = 1.0 if state.get("is_mask", False) else 0.0
                node_features[i, 8] = charging_rate

        uav_x = self.global_to_original_map[current_node_id]["x"]
        uav_y = self.global_to_original_map[current_node_id]["y"]
        
        completion_ratio = self.node_charge_count / max(1, len(self.node_global_ids))
        
        uav_state = np.array([
            # self.uav_state["remaining_battery"] / self.agent_max_battery,
            completion_ratio,
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

    def action_masks(self) -> np.ndarray:
        """
        无限电池版本的动作掩码
        只考虑节点是否已充电和是否可达，不考虑电量限制
        """
        node_mask = np.zeros(self.num_nodes, dtype=bool)
        level_mask = np.ones(self.charge_time_levels, dtype=bool)

        curr_id = self.uav_state["node_id"]

        for i in range(self.num_nodes):
            self.dynamic_node_states[i]["is_mask"] = False
            
            # 跳过充电站
            if i == self.station_node_id:
                continue
            
            # 跳过已充电的节点
            if self.dynamic_node_states[i].get("is_charge", False):
                continue
            
            # 不能留在原地
            if i == curr_id:
                continue
            
            # 检查是否可达（由于电池无限，只需要考虑可达性）
            dist_to_node = self.node_distance[curr_id, i]
            
            if dist_to_node != float('inf'):
                node_mask[i] = True
                self.dynamic_node_states[i]["is_mask"] = True

        # 如果所有节点都已充电，允许返回充电站
        if self.node_charge_count >= len(self.node_global_ids):
            dist_to_station = self.node_distance[curr_id, self.station_node_id]
            if dist_to_station != float('inf'):
                node_mask[:] = False
                node_mask[self.station_node_id] = True
                self.dynamic_node_states[self.station_node_id]["is_mask"] = True
                level_mask[:] = False
                level_mask[0] = True

        return np.concatenate([node_mask, level_mask])

    def _decode_action(self, action):
        """解码动作"""
        action = np.array(action).flatten()
        
        target_node_id = int(action[0])
        charge_level = int(action[1])

        charge_time = charge_level
        node_type = self.global_to_original_map[target_node_id]["original_type"]

        if node_type == "sensor":
            charge_time = (charge_level + 1) * self.sensor_charge_time_step
        elif node_type == "relay":
            charge_time = (charge_level + 1) * self.gateway_charge_time_step
        elif node_type == "station":
            charge_time = 0.0
        
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
        
        self.tx_energy = 0.0
        self.move_energy = 0.0
        self.charg_energy = 0.0
        self.active_energy = 0.0
        
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
                        "current_energy": 1,
                        "charging_rate": 0.95,
                        "is_charge": False,
                        "is_mask": False,
                        "total_battery": 1
                    }
                else:
                    self.dynamic_node_states[global_id] = {
                        "current_energy": 0,
                        "charging_rate": 0,
                        "is_charge": False,
                        "is_mask": False,
                        "total_battery": 0
                    }
        
        self.base_total_distance = 2600.0
        self.base_total_eue = 0.60     
        self.total_distance = 0.0
        
        return self._get_struct_obs(), self._get_info()

    def _get_info(self):
        """获取环境信息"""
        return {
            "current_uav_id": self.uav_state["node_id"],
            # "uav_battery": self.uav_state["remaining_battery"],
            "node_num": self.num_nodes - 1,
            "node_charge_count": self.node_charge_count,
            "total_nodes": len(self.node_global_ids)
        }

    def step(self, action):
        """执行一步环境 - 保持原始reward逻辑不变"""
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
        active_time = 0.0
        active_energy = 0.0

        current_node_id = self.uav_state["node_id"]
        
        # 检查目标节点是否可达
        distance_needed = self.node_distance[current_node_id][target_node_id]
        if distance_needed == float('inf'):
            reward = self.unreachable_penalty
            truncated = True
            observation = self._get_struct_obs()
            info = self._get_info()
            info["error"] = "unreachable_node"
            return observation, reward, terminated, truncated, info
        
        self.total_distance += distance_needed
        step_move_energy = distance_needed * self.agent_move_consumption
        step_move_time = distance_needed / self.agent_speed

        # 距离惩罚 - 保持原始逻辑
        reward -= 3.0 * (distance_needed / self.max_min["distance"]["max"])
        
        # 执行移动 - 电池无限，不检查电量
        # self.uav_state["remaining_battery"] -= step_move_energy
        self.uav_state["node_id"] = target_node_id

        node_type = self.global_to_original_map[target_node_id]["original_type"]
                    
        if node_type == "sensor" or node_type == "relay":
            node_state = self.dynamic_node_states[target_node_id]   

            max_battery = self.sensor_max_battery if node_type == "sensor" else self.gateway_max_battery
            
            # 节点能量消耗
            coum_move_energy = step_move_time / (394 if node_type == "sensor" else 197)

            # 检查节点是否已死亡
            if node_state["current_energy"] - coum_move_energy <= 0.0:
                active_time = 1 / node_state["charging_rate"]
                active_energy = active_time * self.agent_tx_consumption
                      
            energy_needed = max_battery - node_state["current_energy"] + coum_move_energy        
                    
            net_tx_rate = self.agent_tx_consumption * node_state["charging_rate"]   
            time_to_full_node = energy_needed / net_tx_rate if net_tx_rate > 0 else 0
            step_charge_time = min(action_charge_time, time_to_full_node)  

            step_charge_energy = step_charge_time * net_tx_rate    
            step_tx_energy = action_charge_time * self.agent_tx_consumption   

            # 执行充电
            if not node_state["is_charge"]:
                node_state["is_charge"] = True
                self.node_charge_count += 1

            # # 检查无人车电量 - 电池无限，所以不会出现电量不足
            # if self.uav_state["remaining_battery"] < step_tx_energy and step_tx_energy > 0:
            #     reward -= 5.0
            #     terminated = True
            #     step_tx_energy = self.uav_state["remaining_battery"]
            #     step_charge_time = step_tx_energy / self.agent_tx_consumption
            #     step_charge_energy = step_charge_time * net_tx_rate
            #     self.uav_state["remaining_battery"] = 0.0
            # else:                
            #     self.uav_state["remaining_battery"] -= step_tx_energy
            
            node_state["current_energy"] = min(node_state["current_energy"] + step_charge_energy - coum_move_energy, max_battery)

            # 补偿时间消耗
            current_time_step = step_move_time + action_charge_time + active_time
            coum_move_energy = current_time_step / (394 if node_type == "sensor" else 197)
            node_state["current_energy"] += coum_move_energy
        
            self.tx_energy += step_tx_energy
            self.move_energy += step_move_energy
            self.charg_energy += step_charge_energy
            self.active_energy += active_energy
                                            
            total_energy_consumed = step_move_energy + step_tx_energy + active_energy
            if total_energy_consumed > 0 and step_charge_energy > 0:
                eue_reward = step_charge_energy / (total_energy_consumed)
                reward += 1.0 * eue_reward

        # 更新全局时间和其他节点能量
        current_time_step = step_move_time + action_charge_time + active_time
        self.current_time += current_time_step
        
        for global_id in self.node_global_ids:
            node_state = self.dynamic_node_states[global_id]
            node_type = self.global_to_original_map[global_id]["original_type"]
            coum_move_energy = current_time_step / (394 if node_type == "sensor" else 197)
            node_state["current_energy"] = max(0.0, node_state["current_energy"] - coum_move_energy)
        
        current_node_type = self.global_to_original_map[self.uav_state["node_id"]]["original_type"]
        
        # 回到充电站检查 - 保持原始逻辑
        if not truncated and current_node_type == "station":
            terminated = True
            if (self.tx_energy + self.move_energy + self.active_energy) > 0.0:
                total_eue = self.charg_energy / (self.tx_energy + self.move_energy + self.active_energy)
                reward += 1.0 * (total_eue / self.base_total_eue) + 2.0 * (self.base_total_distance / max(self.total_distance, 1))
            else:
                reward -= 5.0
        
        self.current_step += 1
        if self.current_step >= self.max_steps and not terminated:
            truncated = True
            reward -= 5.0

        observation = self._get_struct_obs()
        info = self._get_info()
        info.update({
            "step_charge_energy": step_charge_energy,
            "step_move_energy": step_move_energy,
            "step_tx_energy": step_tx_energy,
            "active_energy": active_energy,
            "node_charge_count": self.node_charge_count,
            "distance_needed": distance_needed,
        })
        
        return observation, reward, terminated, truncated, info

    def render(self):
        pass

    def close(self):
        pass