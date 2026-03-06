import itertools
import math
import numpy as np
import gymnasium as gym 
from gymnasium import spaces
from typing import Dict, List, Tuple, Union, Any
import random
import os,sys
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

class UAVSchedulingEnv(gym.Env):
    """
    结构化字典观测
    """
    
    def __init__(self, 
                 graph_G,
                 point_info,
                #  point_len,
                #  topology_info,
                 max_min,
                 seed=24,
                 agent_max_battery: float = 360000000.0,
                 agent_move_consumption: float = 1000.0,
                 agent_tx_consumption: float = 100,
                 agent_speed: float = 5,
                 min_charging_rate: float = 0.70,
                 sensor_max_battery: float = 240000.0,
                 gateway_max_battery: float = 800000.0,
                 charge_time_levels: int = 100,
                 is_train: bool = False, #训练模式
                 json_data: str = None, #json路径                 
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

        # 计算充电参数
        self._initialize_charging_parameters()
        
        # 存储图数据
        self.graph_G = graph_G
        self.point_info = point_info
        # self.point_len = point_len
        # self.point_len_sum = sum(point_len.values())
        # self.topology_info = topology_info
        self.max_min = max_min

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
        self.base_total_ditance = 2600.0
        self.base_total_eue = 0.60     
        # self.candidate_avg_distance = 100.0
        self.total_distance = 0.0

    def _initialize_node_mappings(self):
        """初始化节点映射关系"""

        # 获取所有点的经度和纬度列表
        lngs = [p['lng'] for p in self.point_info]
        lats = [p['lat'] for p in self.point_info]
        
        # 计算极值
        self.lng_min, self.lng_max = min(lngs), max(lngs)
        self.lat_min, self.lat_max = min(lats), max(lats)
        
        # 防止分母为 0（万一所有点都在一条线上）
        self.lng_range = max(self.lng_max - self.lng_min, 1e-6)
        self.lat_range = max(self.lat_max - self.lat_min, 1e-6)        
        
        self.original_to_global_map = {}
        self.global_to_original_map = {}
        new_global_id_counter = 0

        for info in self.point_info:
            old_point_key = info.get("node_id") # originalId
            original_id = info.get("original_id")
            # global_id = 0
            if old_point_key is None: 
                continue

            node_type_str = info["type"]
            # if node_type_str == "station":
            #     global_id = original_id - 1
            # elif node_type_str == "gateWay":
            #     global_id = original_id + self.point_len["station"] -1
            # elif node_type_str == "sensor":
            #     global_id = original_id + self.point_len["station"] + self.point_len["gateWay"] -1
            # else:
            #     continue
            
            new_global_id = new_global_id_counter
            
            mapping_data = {
                "new_global_id": new_global_id,
                "original_id":old_point_key,
                # "global_id":global_id,
                "original_type": node_type_str,
                "lng": info.get("lng"),
                "lat": info.get("lat"),
                "point_data": info
            }
            
            self.original_to_global_map[old_point_key] = mapping_data 
            self.global_to_original_map[new_global_id] = mapping_data 
            
            new_global_id_counter += 1

        self.num_nodes = new_global_id_counter
        self.max_steps = 2 * self.num_nodes

        self.sensor_global_ids = [
            gid for gid, data in self.global_to_original_map.items() 
            if data["original_type"] == "sensor"
        ]
        self.gateWay_global_ids = [
            gid for gid, data in self.global_to_original_map.items() 
            if data["original_type"] == "gateWay"
        ]
        self.node_global_ids = self.sensor_global_ids + self.gateWay_global_ids
        
        self.num_sensors = len(self.sensor_global_ids)
        self.num_gateWay = len(self.gateWay_global_ids)

    def _initialize_uav_state(self):
        """初始化无人车状态"""
        start_key = 0
        start_info = self.original_to_global_map[start_key]
        self.start_node_id = start_info["new_global_id"]
        # global_id = start_info["global_id"]
        
        self.initial_uav_state = {
            "node_id": self.start_node_id,
            # "global_id":global_id,
            "remaining_battery": self.agent_max_battery
        }
        self.uav_state = self.initial_uav_state.copy()

    def _initialize_node_states(self):
        """初始化所有节点的动态状态"""
        self.dynamic_node_states = {}

        for global_id, mapping_data in self.global_to_original_map.items():
            node_type = mapping_data["original_type"]
            point_data = mapping_data["point_data"]

            if node_type == "sensor" or node_type == "gateWay":
                battery_info = point_data.get("battery_info", {}) 
                
                initial_energy = battery_info.get("current_energy")
                charging_rate = battery_info.get("charging_rate") 
                
                self.dynamic_node_states[global_id] = {
                    "current_energy": initial_energy,
                    "charging_rate": charging_rate,
                    "is_charge": False,
                    "is_mask":True,
                }
            else:
                self.dynamic_node_states[global_id] = {
                    "current_energy": 0,
                    "charging_rate": 0,
                    "is_charge": False,
                    "is_mask":False,
                }

    def _initialize_distance_matrix(self):
        """初始化距离矩阵"""
        self.node_distance = np.full((self.num_nodes, self.num_nodes), 
                                    fill_value=np.inf, dtype=np.float32)
        # self.path_node_count = np.full((self.num_nodes, self.num_nodes), 
        #                             fill_value=0, dtype=np.int32)

        for u in self.graph_G:
            for v in self.graph_G[u]:
                original_u = u 
                original_v = v
                
                global_u = self.original_to_global_map.get(original_u, {}).get("new_global_id")
                global_v = self.original_to_global_map.get(original_v, {}).get("new_global_id")

                if global_u is not None and global_v is not None:
                    distance = self.graph_G[original_u][original_v]
                    self.node_distance[global_u, global_v] = distance

        np.fill_diagonal(self.node_distance, 0.0)
        # for (start_original, end_original), path_list in self.topology_info.items():
        #     # 获取对应的全局索引
        #     start_info = self.original_to_global_map.get(start_original)
        #     end_info = self.original_to_global_map.get(end_original)
            
        #     if start_info is None or end_info is None:
        #         continue
                
        #     start_idx = start_info["new_global_id"]
        #     end_idx = end_info["new_global_id"]
            
        #     # 计算途径点个数（减去起点和终点）
        #     path_count = len(path_list) - 2
        #     if path_count >= 0:  # 确保非负
        #         self.path_node_count[start_idx, end_idx] = path_count

    def _initialize_charging_parameters(self):
        """初始化充电参数"""
        self.tx_rate = self.min_charging_rate * self.agent_tx_consumption
        self.sensor_max_charge = self.sensor_max_battery / self.tx_rate
        self.gateway_max_charge = self.gateway_max_battery / self.tx_rate
        
        # 计算最大充电时间
        self.max_charge_time = max(self.sensor_max_charge, self.gateway_max_charge)
        
        # 充电时间步长
        self.snesor_charge_time_step = self.sensor_max_charge / self.charge_time_levels
        self.gateway_charge_time_step = self.gateway_max_charge / self.charge_time_levels

    def _define_spaces(self):
        """定义结构化的观测和动作空间（适配 Attention Model）"""
        
        # 1. 定义观测空间为字典结构
        # 维度说明：每个节点包含 9 个特征 (energy, dist, lng, lat, is_charge, type_one_hot*3, is_mask)
        self.observation_space = spaces.Dict({
            "observations": spaces.Box(
                low=0.0,
                high=1.0,
                shape=(self.num_nodes, 10), 
                dtype=np.float32
            ),
            # "node_ids": spaces.Box(
            #     low=0,
            #     high=self.point_len_sum,
            #     shape=(self.num_nodes,),
            #     dtype=np.int64
            # ),
            "uav_state": spaces.Box(
                low=0.0,
                high=1.0,
                shape=(4,), # battery_ratio, completion_ratio, norm_lng, norm_lat
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
        
        # 2. 动作空间保持 MultiDiscrete，但逻辑上由模型处理
        self.action_space = spaces.MultiDiscrete([
            self.num_nodes,          
            self.charge_time_levels  
        ])
        

    def _get_struct_obs(self):
        """获取结构化的观测值"""
        current_node_id = self.uav_state["node_id"]

        # current_global_id = self.uav_state["global_id"]
        
        # 1. 构造所有节点的特征矩阵 [num_nodes, 9]
        # 特征顺序定义: 0:energy, 1:dist, 2:lng, 3:lat, 4:is_charge, 5-7:type_one_hot, 8:is_mask
        node_features = np.zeros((self.num_nodes, 10), dtype=np.float32)
        # node_ids_seq = np.zeros(self.num_nodes, dtype=np.int64)

        full_mask = self.action_masks()

        self.min_distance = self.max_min['distance']['min']
        self.max_distance = self.max_min['distance']['max']
        self.distance_rang = self.max_distance - self.min_distance

        self.lng_min = self.max_min['longitude']['min']
        self.lng_range = self.max_min['longitude']['max'] - self.lng_min

        self.lat_min = self.max_min['latitude']['min']
        self.lat_range = self.max_min['latitude']['max'] - self.lat_min

        # # 获取全局最大最小拓扑路径长度
        # max_path_len = self.max_min["topology_path_length"]["max"]
        # min_path_len = self.max_min["topology_path_length"]["min"]
        # self.path_len_range = max(1, max_path_len - min_path_len)

        for i in range(self.num_nodes):
            state = self.dynamic_node_states[i]
            mapping = self.global_to_original_map[i]
            node_type = mapping["original_type"]
            # node_ids_seq[i] = mapping["global_id"]
            
            # 基础坐标归一化
            norm_lng = (mapping["lng"] - self.lng_min) / self.lng_range
            norm_lat = (mapping["lat"] - self.lat_min) / self.lat_range
            
            # 距离特征
            dist = self.node_distance[current_node_id, i]
            dist_ratio = (dist - self.min_distance) / self.distance_rang if self.distance_rang > 0 else 0.0

            # raw_count = self.path_node_count[current_node_id, i]
            # normalized_count = (raw_count - min_path_len) / self.path_len_range
            
            # 类型与能量逻辑
            type_code = [0.0, 0.0, 0.0]
            energy_ratio = 0.0
            is_charge = 1.0 if state.get("is_charge", False) else 0.0
            
            if node_type == "sensor":
                energy_ratio = (state["current_energy"]) / self.sensor_max_battery
                type_code = [1.0, 0.0, 0.0]
            elif node_type == "gateWay":
                energy_ratio = (state["current_energy"]) / self.gateway_max_battery 
                type_code = [0.0, 1.0, 0.0]
            elif node_type == "station":
                energy_ratio = 1.0 # 充电站能量视为满
                type_code = [0.0, 0.0, 1.0] 
            charging_rate = state.get("charging_rate", 0.0)
            # 填充矩阵
            node_features[i, 0] = energy_ratio
            node_features[i, 1] = dist_ratio
            node_features[i, 2] = norm_lng
            node_features[i, 3] = norm_lat
            node_features[i, 4] = is_charge
            node_features[i, 5:8] = type_code
            node_features[i, 8] = 1.0 if state.get("is_mask", False) else 0.0
            # node_features[i, 9] = normalized_count
            node_features[i, 9] = charging_rate

        # 2. 构造 UAV 状态向量
        uav_lng = self.global_to_original_map[current_node_id]["lng"]
        uav_lat = self.global_to_original_map[current_node_id]["lat"]
        uav_state = np.array([
            self.uav_state["remaining_battery"] / self.agent_max_battery,
            self.node_charge_count / self.num_nodes - 1,
            (uav_lng - self.lng_min) / self.lng_range,
            (uav_lat - self.lat_min) / self.lat_range
        ], dtype=np.float32)

        # node_mask = full_mask[:self.num_nodes].astype(np.int8)
        action_mask = full_mask.astype(np.int8)

        return {
            "observations": node_features,  # 送入 Backbone.encoder
            # "node_ids": node_ids_seq,
            "uav_state": uav_state,        # 用于 Context 拼接
            "action_mask": action_mask,       # 用于 Decoder 的掩码计算
            "last_node_idx":np.array([current_node_id], dtype=np.int64)
        }

    def reset(self, seed=None, options=None):
        """重置环境"""
        if seed is not None:
            super().reset(seed=seed)
        
        # 重置状态
        self.current_time = 0.0
        self.current_step = 0
        self.node_charge_count = 0
        # 能量
        self.tx_energy = 0.0
        self.move_energy = 0.0
        self.charg_energy = 0.0
        self.active_energy = 0.0
        
        if self.is_train:
            # 存储图数据
            json_data = self.json_data
            wsn_sim = WSNSimulation(json_data, seed = int(time.time()))
            data = wsn_sim.get_data()
            graph_G = data["graph_G"]
            point_info = data["point_info"]
            # point_len = data['len']
            # topology_info = data['topology_info']            
            self.graph_G = graph_G
            self.point_info = point_info
            # self.point_len = point_len
            # self.point_len_sum = sum(point_len.values())
            # self.topology_info = topology_info

            # 初始化节点映射
            self._initialize_node_mappings()
            
            # 初始化无人车状态
            self._initialize_uav_state()
            
            # 初始化节点动态状态
            self._initialize_node_states()
            
            # 初始化距离矩阵
            self._initialize_distance_matrix()
        else:        
            # 重置无人车
            self.uav_state = self.initial_uav_state.copy()
            
            # 重置节点状态
            self.dynamic_node_states = {}
            
            for global_id, mapping_data in self.global_to_original_map.items():
                node_type = mapping_data["original_type"]
                point_data = mapping_data["point_data"]

                if node_type == "sensor" or node_type == "gateWay":
                    battery_info = point_data.get("battery_info", {}) 
                    initial_energy = battery_info.get("current_energy")
                    charging_rate = battery_info.get("charging_rate") 

                    self.dynamic_node_states[global_id] = {
                        "current_energy": initial_energy,
                        "charging_rate": charging_rate,
                        "is_charge": False,
                        "is_mask"  : True
                    }
                else:
                    self.dynamic_node_states[global_id] = {
                        "current_energy": 0,
                        "charging_rate": 0,
                        "is_charge": False,
                        "is_mask":False,
                    }
        # 基准线
        self.base_total_ditance = 2600.0
        self.base_total_eue = 0.60     
        # self.candidate_avg_distance = 100.0
        self.total_distance = 0.0
        # 返回观测
        return self._get_struct_obs(), self._get_info()

    def _get_info(self):
        """获取环境信息"""
        return {
            "current_uav_id": self.uav_state["node_id"],
            "uav_battery": self.uav_state["remaining_battery"],
            "node_num": self.num_nodes - 1,
        }

    def _decode_action(self, action):
        """解码MultiDiscrete动作为原始格式"""
        action = np.array(action).flatten()
        
        target_node_id = int(action[0])
        charge_level = int(action[1])

        charge_time = charge_level

        node_type = self.global_to_original_map[target_node_id]["original_type"]

        # 将充电级别转换为实际充电时间
        if node_type == "sensor":
            charge_time = (charge_level + 1) * self.snesor_charge_time_step
        elif node_type == "gateWay":
            charge_time = (charge_level + 1) * self.gateway_charge_time_step
        elif node_type == "station":
            charge_time = 0.0 # 回服务站依然保持 0
        
        return {
            "target_node_id": target_node_id,
            "charge_time": np.array([charge_time], dtype=np.float32)
        }

    def step(self, action):
        """
        执行一步环境
        
        Args:
            action: MultiDiscrete动作 [target_node_id, charge_time_level]
        """
        # 解码动作
        action_dict = self._decode_action(action)
        target_node_id = action_dict["target_node_id"]
        action_charge_time = action_dict["charge_time"][0]
        
        # 初始化
        reward = 0.0
        terminated = False
        truncated = False
        step_charge_energy = 0.0
        step_move_energy = 0.0
        step_tx_energy = 0.0
        step_move_time = 0.0
        step_charge_time = 0.0
        distance_needed = 0.0
        # current_eue = 0.0

        active_time = 0.0
        active_energy = 0.0

        current_node_id = self.uav_state["node_id"]
        
        distance_needed = self.node_distance[current_node_id][target_node_id]
        self.total_distance += distance_needed
        step_move_energy = distance_needed * self.agent_move_consumption
        step_move_time = distance_needed / self.agent_speed

        # 距离奖励，越接近目标点越好
        # distance_ratio = (distance_needed-self.min_distance) / (self.max_distance-self.min_distance)
        # reward -= 5.0 * (distance_ratio) 
        # reward -= 5.0 * ((distance_needed)/(self.candidate_avg_distance)) 
        reward -= 5.0 * ((distance_needed/self.max_distance)) 
        
        # 执行移动
        self.uav_state["remaining_battery"] -= step_move_energy
        self.uav_state["node_id"] = target_node_id        

        node_type = self.global_to_original_map[target_node_id]["original_type"]
                    
        if node_type == "sensor" or node_type == "gateWay":            
            node_state = self.dynamic_node_states[target_node_id]   

            max_battery = self.sensor_max_battery
            coum_move_energy = 0
            if node_type == "sensor":
                max_battery = self.sensor_max_battery
                #394s消耗1J能量
                coum_move_energy = step_move_time / 394
            else:
                max_battery = self.gateway_max_battery
                coum_move_energy = step_move_time / 197             

            # 检查节点电量 (节点电量在无人车到达时已经为0，已经死亡需要激活)
            if node_state["current_energy"] - coum_move_energy <= 0.0:
                active_time = 1 / node_state["charging_rate"]
                active_energy = active_time * self.agent_tx_consumption
                      
            energy_needed = (max_battery - node_state["current_energy"] + coum_move_energy )        
                    
            net_tx_rate = self.agent_tx_consumption * node_state["charging_rate"]   
            time_to_full_node = energy_needed / net_tx_rate     
            step_charge_time = min(action_charge_time, time_to_full_node)  

            step_charge_energy = step_charge_time * net_tx_rate    
            step_tx_energy = action_charge_time * self.agent_tx_consumption   

            # 执行充电
            node_state["is_charge"] = True
            self.node_charge_count += 1
            # reward += 10.0

            # 检查充电电量 (无人车不够电量充电)
            if self.uav_state["remaining_battery"] < step_charge_energy and step_charge_energy > 0:
                reward -= 5.0
                terminated = True
                step_tx_energy = self.uav_state["remaining_battery"]
                step_charge_time = step_tx_energy / self.agent_tx_consumption
                step_charge_energy = step_charge_time * net_tx_rate
                self.uav_state["remaining_battery"] = 0.0
            else:                
                self.uav_state["remaining_battery"] -= step_tx_energy
            
            node_state["current_energy"] = min(node_state["current_energy"] + step_charge_energy - coum_move_energy, max_battery)

            #后面会减回来，不然容易出错
            current_time_step = step_move_time + action_charge_time + active_time
            if node_type == "sensor":
                #394s消耗1J能量
                coum_move_energy = current_time_step / 394
            else:
                coum_move_energy = current_time_step / 197 

            node_state["current_energy"] += coum_move_energy
        
            self.tx_energy += step_tx_energy
            self.move_energy += step_move_energy
            self.charg_energy += step_charge_energy
            self.active_energy += active_energy
                                            
            total_energy_consumed = step_move_energy + step_tx_energy + active_energy
            if total_energy_consumed > 0 and step_charge_energy > 0:
                current_eue = (step_charge_energy) / (total_energy_consumed )
                # max_battery在分母加上的话可以降低eue的reward计算比例
                eue_reward =  (step_charge_energy) / (total_energy_consumed + max_battery)
                reward += 1.0 * eue_reward

        # 更新全局时间
        current_time_step = step_move_time + action_charge_time + active_time
        self.current_time += current_time_step
        
        for global_id in self.node_global_ids:
            node_state = self.dynamic_node_states[global_id]
            node_type = self.global_to_original_map[global_id]["original_type"]
            if node_type == "sensor":
                #394s消耗1J能量
                coum_move_energy = current_time_step / 394
                node_state["current_energy"] -= coum_move_energy
            else:
                coum_move_energy = current_time_step / 197 
                node_state["current_energy"] -= coum_move_energy                      
            
            node_state["current_energy"] = max(0.0, node_state["current_energy"])
        
        current_node_type = self.global_to_original_map[self.uav_state["node_id"]]["original_type"]
        
        if not terminated and current_node_type == "station":
            terminated = True
            if  self.uav_state["remaining_battery"] >= 0.0 and (self.tx_energy + self.move_energy + self.active_energy) > 0.0:  #回去服务站
                total_eue = (self.charg_energy)/(self.tx_energy + self.move_energy + self.active_energy)         
                reward += 1.0 * (total_eue/self.base_total_eue)  + 2.0*(self.base_total_ditance/self.total_distance)
            else: #回不去服务站了
                reward -= 5.0
        
        self.current_step += 1
        if (self.current_step >= self.max_steps) and not terminated:
            truncated = True
            reward -= 5.0

        # 获取观测和信息
        observation = self._get_struct_obs()
        info = self._get_info()
        info.update({
            "step_charge_energy": step_charge_energy,
            "step_move_energy": step_move_energy,
            "step_tx_energy": step_tx_energy,
            "active_energy": active_energy,
            "node_charge_count": self.node_charge_count,
            "distance_needed": distance_needed,
            "charge_time": action_charge_time+step_charge_time
            # "current_eue": current_eue
        })
        
        return observation, reward, terminated, truncated, info

    def render(self):
        """渲染环境（暂不实现）"""
        pass

    def close(self):
        """关闭环境"""
        pass

    
    def action_masks(self) -> np.ndarray:
        """
        动作掩码逻辑：
        True 表示可以去，False 表示禁止去。
        """
        # Node Mask: 针对节点选择维度的掩码
        node_mask = np.zeros(self.num_nodes, dtype=bool)
        level_mask = np.ones(self.charge_time_levels, dtype=bool)

        curr_id = self.uav_state["node_id"]
        # original_cur_id = self.global_to_original_map[curr_id]["original_id"]
        curr_batt = self.uav_state["remaining_battery"]
        station_idx = self.original_to_global_map[0]["new_global_id"]
        can_go_to_sensor = False


        for i in range(self.num_nodes):
            # 一开始所有都是0
            self.dynamic_node_states[i]["is_mask"] = False
            # 跳过充电站，后面统一逻辑处理
            if i == station_idx:
                continue
            # 规则：不能留在原地
            if i == curr_id:
                continue
                
            dist_to_node = self.node_distance[curr_id, i]
            # 这里的计算参考你原来的逻辑：去节点 + 回站的距离
            dist_to_station = self.node_distance[i, station_idx]
            total_needed_dist = dist_to_node + dist_to_station
                
            # 检查条件：
            if (total_needed_dist != np.inf and total_needed_dist < 3.4028235e38 and # 距离不为无穷大（距离可达）
                (total_needed_dist * self.agent_move_consumption) <= curr_batt and # 移动无人车电量足够移动(去到目的地+服务站)
                not self.dynamic_node_states[i].get("is_charge", False)): # 节点未充电 （只去服务一次）              
                node_mask[i] = True
                can_go_to_sensor = True
                self.dynamic_node_states[i]["is_mask"] = True


        if can_go_to_sensor:
            # 有传感器可去，屏蔽充电站
            node_mask[station_idx] = False
            self.dynamic_node_states[station_idx]["is_mask"] = False
        else:
            # 没有传感器可去，强制回站
            node_mask[:] = False # 清空所有
            node_mask[station_idx] = True
            self.dynamic_node_states[station_idx]["is_mask"] = True
            # 回站时，level 必须选 0
            level_mask[:] = False
            level_mask[0] = True

        return np.concatenate([node_mask, level_mask])
    
    