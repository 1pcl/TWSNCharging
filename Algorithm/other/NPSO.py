import numpy as np
import random
import time
import os
import sys
from copy import deepcopy
import math

# --- 环境导入逻辑，确保路径正确 ---
current_script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_script_dir)
if project_root not in sys.path:
    sys.path.insert(0, project_root)

try:
    from Scenario.UAVSchedulingEnv import UAVSchedulingEnv
    from Scenario.WSNsimulation import WSNSimulation 
except ImportError as e:
    print(f"导入自定义模块失败: {e}")
    sys.exit(1)

class NPSOSolver:
    """
    方案 C: 路径(交换子) + 充电时长(连续空间映射) 联合优化算法
    """
    def __init__(self, env, n_particles=100, max_iterations=1000):
        self.env = env
        self.n_particles = n_particles
        self.max_iterations = max_iterations
            
        # 获取所有可充电节点ID（排除基站/充电站）
        self.all_nodes = list(env.node_global_ids)
        self.station_idx = env.start_node_id
        self.sensor_nodes = [n for n in self.all_nodes if n != self.station_idx]
        self.dim = len(self.sensor_nodes)  # 维度 = 传感器节点数量
        
        # NPSO 论文核心参数
        self.omega_max = 0.8
        self.omega_min = 0.2
        self.c1 = 1.2 # 个体学习因子
        self.c2 = 2.0 # 社会学习因子
        self.K_threshold = 10 # 新生代触发阈值
        self.early_stop_patience = 100  # X 代没有改进就停
        
        # 初始化粒子群
        self.particles = []
        self.gbest_path = None
        self.gbest_times = None # 映射：node_id -> float_ratio
        self.gbest_fit = -float('inf')
        
        self._init_swarm()

    def _init_swarm(self):
        """初始化粒子：5个启发式种子 + 45个随机粒子"""
        
        # --- 辅助函数：生成不同策略的种子 ---
        def generate_heuristic_seed(strategy="nearest"):
            unvisited = list(self.sensor_nodes)
            path = []
            curr = self.station_idx
            
            while unvisited:
                if strategy == "nearest":
                    # 策略1：最近邻
                    next_node = min(unvisited, key=lambda x: self.env.node_distance[curr, x])
                elif strategy == "balanced":
                    # 策略2：平衡距离与电量 (得分 = 距离 / 剩余电量百分比)
                    # 剩余电量越少，分母越小，得分越高
                    def get_score(n_id):
                        dist = self.env.node_distance[curr, n_id]
                        state = self.env.dynamic_node_states[n_id]
                        info = self.env.global_to_original_map[n_id]
                        max_b = self.env.sensor_max_battery if info["original_type"] == "sensor" else self.env.gateway_max_battery
                        life_ratio = max(0.01, state["current_energy"] / max_b)
                        return dist * life_ratio # 值越小表示越值得去
                    next_node = min(unvisited, key=get_score)
                else:
                    # 策略3：轻微扰动的最近邻 (在最近的3个中随机挑)
                    candidates = sorted(unvisited, key=lambda x: self.env.node_distance[curr, x])[:3]
                    next_node = random.choice(candidates)
                
                path.append(next_node)
                unvisited.remove(next_node)
                curr = next_node
            
            # 关键修改：路径最后添加回站节点
            path.append(self.station_idx)
            
            # 时间初始化：统一使用"补满"逻辑
            node_times = {}
            for n_id in self.sensor_nodes:
                info = self.env.global_to_original_map[n_id]
                max_b = self.env.sensor_max_battery if info["original_type"] == "sensor" else self.env.gateway_max_battery
                state = self.env.dynamic_node_states[n_id]
                needed = max_b - state["current_energy"]
                pwr = self.env.agent_tx_consumption * state["charging_rate"]
                ratio = (needed / pwr) / self.env.charge_time_levels
                node_times[n_id] = max(0.0, min(1.0, ratio))
            
            # 服务站的时间设为0
            node_times[self.station_idx] = 0.0
            
            return path, node_times

        # --- 1. 生成 5 个种子 ---
        seeds_configs = ["nearest", "nearest", "balanced", "balanced", "random_near"]
        for config in seeds_configs:
            h_path, h_times = generate_heuristic_seed(config)
            self.particles.append({
                'path': h_path,
                'node_times': h_times,
                'pbest_path': deepcopy(h_path),
                'pbest_times': deepcopy(h_times),
                'pbest_fit': -float('inf')
            })
            # print(f"已注入启发式种子策略: {config}")

        # --- 2. 其余粒子保持随机 ---
        remaining = self.n_particles - len(self.particles)
        for _ in range(remaining):
            path = random.sample(self.sensor_nodes, self.dim)
            # 关键修改：随机路径最后添加回站节点
            path.append(self.station_idx)
            node_times = {node: random.random() for node in self.sensor_nodes}
            # 服务站的时间设为0
            node_times[self.station_idx] = 0.0
            self.particles.append({
                'path': path,
                'node_times': node_times,
                'pbest_path': deepcopy(path),
                'pbest_times': deepcopy(node_times),
                'pbest_fit': -float('inf')
            })

    def calculate_fitness(self, path, node_times):
        """计算适应度：在仿真环境中执行该粒子的决策"""
        self.env.reset()
        total_reward = 0
        
        for node_id in path:
            # 方案C核心：根据节点ID取回对应的充电比例，映射为 Level
            ratio = node_times[node_id]
            time_level = int(ratio * (self.env.charge_time_levels - 1))
            time_level = max(0, min(self.env.charge_time_levels - 1, time_level))
            
            # 检查 Action Mask (安全机制)
            mask = self.env.action_masks()
            if not mask[node_id]:
                total_reward -= 100 # 如果该点不可达，给予大幅惩罚
                break
                
            action = (node_id, time_level)
            _, reward, terminated, truncated, _ = self.env.step(action)
            total_reward += reward
            
            if terminated or truncated:
                break
        
        # 注意：现在路径已经包含服务站，所以不需要额外回站
        # 服务站节点的time_level为0，所以不会充电
        return total_reward

    def _generate_newborns(self):
        """NPSO 论文核心：新生代粒子逻辑 (基于全局最优的小范围变异)"""
        # 随机选择 20% 的粒子进行新生操作
        for i in range(int(self.n_particles * 0.2)):
            idx = random.randint(0, self.n_particles - 1)
            # 路径变异：交换两个节点（不交换最后的服务站）
            new_path = deepcopy(self.gbest_path)
            a, b = random.sample(range(self.dim), 2)  # 只在传感器节点中交换
            new_path[a], new_path[b] = new_path[b], new_path[a]
            
            # 充电时间变异：随机扰动
            new_times = deepcopy(self.gbest_times)
            target_node = random.choice(self.sensor_nodes)
            new_times[target_node] = max(0.0, min(1.0, new_times[target_node] + random.uniform(-0.2, 0.2)))
            
            self.particles[idx]['path'] = new_path
            self.particles[idx]['node_times'] = new_times

    def update_particle(self, p, omega):
        """更新粒子位置（双空间同步学习）"""
        # 1. 路径空间更新 (使用交换子向 Gbest 靠拢)
        new_path = deepcopy(p['path'])
        for i in range(self.dim):  # 只对前dim个节点（传感器节点）进行交换
            if random.random() < self.c2 * random.random():
                target_node = self.gbest_path[i]
                # 在new_path中查找目标节点（排除最后一个服务站）
                curr_idx = new_path.index(target_node) if target_node in new_path[:-1] else -1
                if curr_idx != -1 and curr_idx < self.dim:  # 确保找到且在有效范围内
                    new_path[i], new_path[curr_idx] = new_path[curr_idx], new_path[i]
        # 确保路径最后一个节点是服务站
        new_path[-1] = self.station_idx
        p['path'] = new_path

        # 2. 时间空间更新 (连续变量更新公式)
        for node_id in self.sensor_nodes:
            r1, r2 = random.random(), random.random()
            curr_val = p['node_times'][node_id]
            pbest_val = p['pbest_times'][node_id]
            gbest_val = self.gbest_times[node_id]
            
            # 计算更新步长
            new_val = curr_val + self.c1 * r1 * (pbest_val - curr_val) + self.c2 * r2 * (gbest_val - curr_val)
            # 增加基于惯性权重的微量随机扰动
            new_val += (random.random() - 0.5) * omega
            
            p['node_times'][node_id] = max(0.0, min(1.0, new_val))
        
        # 服务站的时间保持为0
        p['node_times'][self.station_idx] = 0.0

    def solve(self):
        """主优化循环"""
        # print(f"开始 NPSO 方案 C 优化，共 {self.max_iterations} 代...")
        no_improve_count = 0
        for it in range(self.max_iterations):
            # 动态权重计算
            omega = self.omega_max - (self.omega_max - self.omega_min) * (it / self.max_iterations)

            # 记录本代是否找到了更好的解
            found_better = False
            
            for p in self.particles:
                fit = self.calculate_fitness(p['path'], p['node_times'])
                
                # 更新个体最优
                if fit > p['pbest_fit']:
                    p['pbest_fit'] = fit
                    p['pbest_path'] = deepcopy(p['path'])
                    p['pbest_times'] = deepcopy(p['node_times'])
                
                # 更新全局最优
                if fit > self.gbest_fit:
                    # 只有显著提升才认为有改进 (例如提升大于 0.001)
                    if fit - self.gbest_fit > 0.001:
                        found_better = True
                    self.gbest_fit = fit
                    self.gbest_path = deepcopy(p['path'])
                    self.gbest_times = deepcopy(p['node_times'])
            

            # 早停逻辑判断
            if found_better:
                no_improve_count = 0
            else:
                no_improve_count += 1

            # 更新粒子状态
            for p in self.particles:
                self.update_particle(p, omega)
                
            # 触发新生代逻辑
            if it > self.K_threshold:
                self._generate_newborns()
                
            if (it + 1) % 20 == 0:
                print(f"Iteration {it+1}/{self.max_iterations}, Best Reward: {self.gbest_fit:.2f}")

            if no_improve_count >= self.early_stop_patience:
                print(f"--- 触发早停: 连续 {no_improve_count} 代未优化 ---")
                break
        
        return self.gbest_fit

def run_detailed_simulation_npso(env, best_path, best_node_times):
    """仿真执行并统计数据 (方案 C)"""
    env.reset()
    total_reward = 0
    total_path_length = 0
    charge_e, move_e, tx_e, active_e = 0.0, 0.0, 0.0, 0.0
    total_stay_time = 0
    charged_nodes = set()
    logs = []
    
    for i, node_id in enumerate(best_path):
        ratio = best_node_times[node_id]
        time_level = max(0, min(env.charge_time_levels - 1, int(ratio * (env.charge_time_levels - 1))))
        
        from_id = env.uav_state["node_id"]
        from_key = env.global_to_original_map[from_id]["point_data"]["node_id"]
        target_key = env.global_to_original_map[node_id]["point_data"]["node_id"]
        
        action = (node_id, time_level)
        decoded = env._decode_action(action)
        actual_charge_time = decoded["charge_time"][0]
        
        obs, reward, terminated, truncated, info = env.step(action)
        total_reward += reward
        
        total_path_length += info["distance_needed"]
        total_stay_time += actual_charge_time
        charge_e += info["step_charge_energy"]
        move_e += info["step_move_energy"]
        tx_e += info["step_tx_energy"]
        active_e += info["active_energy"]
        
        if time_level > 0 and node_id != env.start_node_id:  # 服务站不统计为充电节点
            charged_nodes.add(node_id)
        
        logs.append({
            "step": i + 1,
            "from_key": from_key,
            "target_key": target_key,
            "stay_level": time_level,
            "actual_charge_time": actual_charge_time,
            "distance": info["distance_needed"],
            "reward": reward
        })
        
        if terminated or truncated:
            break

    stats = {
        "total_steps": len(logs),  # 注意：这里改为 len(logs)，因为不需要+1了
        "total_path_length": total_path_length,
        "total_stay_time": total_stay_time,
        "charged_nodes": len(charged_nodes),
        "avg_eue": charge_e / (move_e + tx_e + active_e) if (move_e + tx_e + active_e) > 0 else 0
    }
    return total_reward, logs, stats

def print_solution_details_npso(env, best_path, best_node_times, total_reward, exec_time, stats):
    """可视化打印结果"""
    print("\n" + "="*75)
    print(f"{'NPSO 方案 C 执行轨迹 (路径 + 时间映射)':^75}")
    print("="*75)
    print(f"| {'Step':<4} | {'From':<8} | {'To':<8} | {'Lvl':<3} | {'Time(s)':<8} | {'Dist(m)':<8} | {'Reward':<7} |")
    print("-" * 75)
    
    # 模拟一遍用于打印
    env.reset()
    step_count = 0
    for i, node_id in enumerate(best_path):
        ratio = best_node_times[node_id]
        time_level = max(0, min(env.charge_time_levels - 1, int(ratio * (env.charge_time_levels - 1))))
        from_id = env.uav_state["node_id"]
        from_key = env.global_to_original_map[from_id]["point_data"]["node_id"]
        target_key = env.global_to_original_map[node_id]["point_data"]["node_id"]
        
        action = (node_id, time_level)
        decoded = env._decode_action(action)
        obs, reward, _, _, info = env.step(action)
        
        step_count += 1
        print(f"| {step_count:<4} | {from_key:<8} | {target_key:<8} | {time_level:<3} | {decoded['charge_time'][0]:<8.1f} | {info['distance_needed']:<8.1f} | {reward:<7.2f} |")
    
    print("-" * 75)
    print(f"算法总耗时: {exec_time:.2f}s | 累积奖励: {total_reward:.2f}")
    print(f"总路径长度: {stats['total_path_length']:.2f}m | 能量利用率(EUE): {stats['avg_eue']:.4f}")
    print(f"访问节点数: {stats['charged_nodes']}/{len(best_path)-1} | 总步数: {stats['total_steps']}")
    print("="*75)

if __name__ == "__main__":
    # 1. 环境初始化
    try:
        wsnSimulation = WSNSimulation(data_input="./data/chargingScheduling_data.json", seed=42)
        data = wsnSimulation.get_data()
        env = UAVSchedulingEnv(
            graph_G=data["graph_G"], 
            point_info=data["point_info"], 
            max_min=data['max_min']
        )
    except Exception as e:
        print(f"初始化失败: {e}")
        sys.exit(1)

    # 2. 运行 NPSO 求解
    start_t = time.time()
    solver = NPSOSolver(env, n_particles=100, max_iterations=1000)
    best_fit = solver.solve()
    exec_t = time.time() - start_t

    # 3. 详细仿真与统计
    total_reward, logs, stats = run_detailed_simulation_npso(env, solver.gbest_path, solver.gbest_times)
    
    # 4. 打印结果
    print_solution_details_npso(env, solver.gbest_path, solver.gbest_times, total_reward, exec_t, stats)