import json
import random
from typing import Dict, List, Any, Optional, Union
import math

class WSNSimulation:
    def __init__(self, data_input: Union[str, Dict[str, Any]], sensorBattery=240000.0, gatewayBattery=800000.0, seed: int = 42):
        """
        初始化 WSN 仿真器
        
        Args:
            data_input: 可以是 JSON 文件路径或已读取的数据字典
            seed: 随机种子，确保可复现
        """
        self.sensorBattery = sensorBattery
        self.gatewayBattery = gatewayBattery
        self.seed = seed
        self.data = None
        self.graph_G = {}
        self.point_info = []
        # self.original_lens = {} # 用于存储原始长度
        self.max_min = {
            "distance": {"max": 0.0, "min": float('inf')},
            "longitude": {"max": -180.0, "min": 180.0},
            "latitude": {"max": -90.0, "min": 90.0},
        }
        
        # 设置随机种子
        random.seed(seed)
        
        # 加载数据
        self._load_data(data_input)
        self._calculate_global_max_min()  # 计算最大最小值
        self._convert_data_format()
        self._select_low_battery_nodes()
        self._build_filtered_graph()
        
    
    def _load_data(self, data_input: Union[str, Dict[str, Any]]) -> None:
        """从文件路径或数据字典加载数据"""
        if isinstance(data_input, str):
            # 如果是文件路径，从文件读取
            with open(data_input, "r") as f:
                self.data = json.load(f)
        else:
            # 如果是字典，直接使用
            self.data = data_input

    def _calculate_global_max_min(self) -> None:
        """计算原始数据的全局最大最小值"""
        # 1. 计算距离的最大最小值（从原始的 graph_G 中）
        if "graph_G" in self.data:
            for edge in self.data["graph_G"]:
                distance = edge["distance"]
                if distance > self.max_min["distance"]["max"]:
                    self.max_min["distance"]["max"] = distance
                if distance < self.max_min["distance"]["min"]:
                    self.max_min["distance"]["min"] = distance
        
        # 2. 计算经纬度的最大最小值（从所有节点中）
        all_points = []
        if "stations" in self.data:
            all_points.extend(self.data["stations"])
        if "gateWay" in self.data:
            all_points.extend(self.data["gateWay"])
        if "sensors" in self.data:
            all_points.extend(self.data["sensors"])
        
        for point in all_points:
            lng = point.get("lng", 0)
            lat = point.get("lat", 0)
            
            if lng > self.max_min["longitude"]["max"]:
                self.max_min["longitude"]["max"] = lng
            if lng < self.max_min["longitude"]["min"]:
                self.max_min["longitude"]["min"] = lng
            
            if lat > self.max_min["latitude"]["max"]:
                self.max_min["latitude"]["max"] = lat
            if lat < self.max_min["latitude"]["min"]:
                self.max_min["latitude"]["min"] = lat
        
        
        # 如果没有找到有效的距离，设置默认值
        if self.max_min["distance"]["min"] == float('inf'):
            self.max_min["distance"]["min"] = 0

    def _convert_data_format(self) -> None:
        """将原始数据转换为统一的 point_info 格式"""
        self.point_info = []
        station_count = 0
        gateWay_count = 0

        # 转换基站数据
        if "stations" in self.data:
            for station in self.data["stations"]:
                station_count += 1
                self.point_info.append({
                    "node_id": station["id"]-1,
                    "type": "station",
                    "original_id": station["id"],
                    "lng": station["lng"],
                    "lat": station["lat"],
                    "distanceInRoad": station["distanceInRoad"],
                    "battery_info": None
                })
        
        # 转换中继节点数据
        if "gateWay" in self.data:
            for gateWay in self.data["gateWay"]:
                gateWay_count += 1
                self.point_info.append({
                    "node_id": gateWay["id"]-1+station_count,
                    "type": "gateWay",
                    "original_id": gateWay["id"],
                    "lng": gateWay["lng"],
                    "lat": gateWay["lat"],
                    "distanceInRoad": gateWay["distanceInRoad"],
                    "battery_info": None
                })
        
        # 转换传感器数据
        if "sensors" in self.data:
            for sensor in self.data["sensors"]:
                self.point_info.append({
                    "node_id": sensor["id"]-1+station_count+gateWay_count,
                    "type": "sensor",
                    "original_id": sensor["id"],
                    "lng": sensor["lng"],
                    "lat": sensor["lat"],
                    "distanceInRoad": sensor["distanceInRoad"],
                    "battery_info": None  # 稍后设置
                })
    
    def _select_low_battery_nodes(self) -> None:
        """选择几个低电量节点，并设置电量信息"""
        # 在过滤前，统计所有节点的原始数量
        # self.original_lens = {
        #     "sensor": sum(1 for p in self.point_info if p["type"] == "sensor"),
        #     "gateWay": sum(1 for p in self.point_info if p["type"] == "gateWay"),
        #     "station": sum(1 for p in self.point_info if p["type"] == "station")
        # }
        # 获取所有传感器节点
        all_sensors = [point for point in self.point_info if point["type"] == "sensor"]
        # 获取所有中继节点
        all_gateWays = [point for point in self.point_info if point["type"] == "gateWay"]
        
        # 随机选择几个节点作为低电量节点
        # num_low_battery = random.randint(17, min(40, len(all_sensors)))
        snesor_low_battery = 30
        gateWays_low_battery = 3
        selected_sensors = random.sample(all_sensors, snesor_low_battery)
        selected_gateWays = random.sample(all_gateWays, gateWays_low_battery)
        
        
        for sensor in selected_sensors:
            # 设置低电量
            current_energy_ratio = random.uniform(0.3, 0.5)
            current_energy = self.sensorBattery * current_energy_ratio

            # 充电效率
            d_h = sensor["distanceInRoad"]
            d_ref = 1.0
            eta_ref = 0.98
            alpha = 6

            #距离偏置，防止分母为0
            beta = 1.0

            d_3d = math.sqrt(math.pow(d_h, 2) + math.pow(0.0 + beta, 2))

            # 原始计算
            raw_rate = alpha * eta_ref * math.pow(d_ref / d_3d, 3.0)

            charging_rate = max(0.70, min(0.98, raw_rate))
            
            
            sensor["battery_info"] = {
                "current_energy": current_energy,
                "current_energy_ratio": current_energy_ratio,
                "charging_rate": charging_rate
            }

        for gateWays in selected_gateWays:
            # 设置低电量（10%-40%）
            current_energy_ratio = random.uniform(0.3, 0.5)
            current_energy = self.gatewayBattery * current_energy_ratio
            
            # 充电效率
            d_h = gateWays["distanceInRoad"]
            d_ref = 1.0
            eta_ref = 0.98
            alpha = 6.0

            d_3d = math.sqrt(math.pow(d_h, 2) + math.pow(3.0, 2))

            # 原始计算
            raw_rate = alpha * eta_ref * math.pow(d_ref / d_3d, 1.8)

            charging_rate = max(0.70, min(0.98, raw_rate))            
            
            gateWays["battery_info"] = {
                "current_energy": current_energy,
                "current_energy_ratio": current_energy_ratio,
                "charging_rate": charging_rate
            }
        
        # 构建最终的 point_info，只包含充电站、基站和选中的低电量传感器
        self.point_info = [
            point for point in self.point_info
            if point["type"] in ["station"] or 
            (point["type"] == "sensor" and point in selected_sensors) or 
            (point["type"] == "gateWay" and point in selected_gateWays)
        ]
    
    def _build_filtered_graph(self) -> None:
        """构建过滤后的图结构，只包含有效节点之间的连接"""
        valid_node_ids = {point["node_id"] for point in self.point_info}
        
        self.graph_G = {}
        if "graph_G" in self.data:
            for edge in self.data["graph_G"]:
                start = edge["nodeStartId"]
                end = edge["nodeEndId"]
                dist = edge["distance"]
                
                # 只保留两个端点都在有效节点列表中的边
                if start in valid_node_ids and end in valid_node_ids:
                    if start not in self.graph_G:
                        self.graph_G[start] = {}
                    self.graph_G[start][end] = dist

    
    def get_data(self) -> Dict[str, Any]:
        """返回 graph_G 和 point_info 数据"""
        return {
            "graph_G": self.graph_G,
            "point_info": self.point_info,
            # "len": self.original_lens, # 返回过滤前的原始长度
            "max_min": self.max_min  # 返回全局最大最小值
        }


# 使用示例
if __name__ == "__main__":
    # 方式1：直接传入文件路径（保持向后兼容）
    # wsnSimulation = WSNSimulation("./data/chargingScheduling_data.json", sensorBattery=240000.0, gatewayBattery=800000.0, seed=42)
    
    # 方式2：先读取文件，然后传入数据（推荐，避免重复读取）
    with open("./data/chargingScheduling_data.json", "r") as f:
        data = json.load(f)
    
    # 创建多个仿真实例，共享同一个数据
    for seed in range(5):
        wsnSimulation = WSNSimulation(data, sensorBattery=240000.0, gatewayBattery=800000.0, seed=seed)
        simulation_data = wsnSimulation.get_data()
        print(f"仿真 {seed} - 节点数量: {len(simulation_data['point_info'])}")
    
    pass