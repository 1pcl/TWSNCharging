from WSNsimulation import WSNSimulation
from UAVSchedulingEnv import UAVSchedulingEnv

# 使用示例
if __name__ == "__main__":

    # 创建调度器实例(需要输入传感器节点容量、中继节点容量和种子)
    wsnSimulation = WSNSimulation("./data/chargingScheduling_data.json", sensorBattery=10800.0, gatewayBattery=10800*2.0, seed=42)
    
    # 获取数据
    data = wsnSimulation.get_data()

    UAVSchedulingEnv(data["graph_G"],data["point_info"],data["len"])

