import os
import sys
import json
import logging
from flask import Flask, request, jsonify
from typing import Dict, Any, List
import numpy as np
import time

from sb3_contrib import MaskablePPO
import torch
import logging

# --- 路径设置 ---
current_script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_script_dir)
sys.path = [path for path in sys.path if path != '../']
if project_root not in sys.path:
    sys.path.insert(0, project_root)

ppo_dir = os.path.join(project_root, "PPO_MLP")
if ppo_dir not in sys.path:
    sys.path.insert(0, ppo_dir)

# 尝试导入自定义模块
try:
    from other.naive_greed import NaiveHeuristicGreedyAgent,run_greedy_simulation
    from other.ACO import ACOSolver, run_detailed_simulation_aco
    from other.NPSO import NPSOSolver, run_detailed_simulation_npso, print_solution_details_npso
    # 导入环境和模拟类 
    from Scenario.UAVSchedulingEnv import UAVSchedulingEnv
    from PPO_MLP.attention_model_wrapper import CustomMaskableActorCriticPolicy
except ImportError as e:
    logging.error(f"导入自定义模块失败: {e}")
    # 打印 sys.path 以帮助调试导入问题
    print("Python path:", sys.path)
    # exit() # 生产环境建议退出，此处为示例不退出
    
app = Flask(__name__)

# --- 辅助函数 JSON 编码器辅助函数 (解决 float32 错误) ---
def convert_numpy_types(obj):
    """递归地将 NumPy/float32 等类型转换为 Python 标准类型，以便 JSON 序列化。"""
    if isinstance(obj, dict):
        return {k: convert_numpy_types(v) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [convert_numpy_types(elem) for elem in obj]
    # elif isinstance(obj, (np.float32, np.float64, np.float)):
    elif isinstance(obj, (np.float32, np.float64, float)): 
        return float(obj)
    # elif isinstance(obj, (np.int32, np.int64, np.int)):
    elif isinstance(obj, (np.int32, np.int64, int)):
        return int(obj)
    elif isinstance(obj, np.ndarray):
        return obj.tolist()
    else:
        return obj


@app.route('/api/runGreedySimulation', methods=['POST'])
def run_greedy_api():
    """
    接收 Java 传输的仿真初始数据，调用贪心算法，并返回路径信息和关键指标。
    """
    try:
        # 1. 获取 Java 传输的 JSON 数据 (POST 请求体)
        data = request.get_json()
        if not data or 'graph_G' not in data or 'point_info' not in data:
            return jsonify({"status": "error", "message": "请求体缺少 'graph_G' 或 'point_info' 字段。"}), 400

        # graph_G = data['graph_G']
        # point_info = data['point_info']
        graph_G = {int(k): {int(nk): v for nk, v in nv.items()} 
               for k, nv in data['graph_G'].items()}
    
        # 强制转换 point_info 里的 node_id 为 int
        point_info = data['point_info']
        for p in point_info:
            p['node_id'] = int(p['node_id'])
        
        logging.info("成功接收 Java 传输的数据，开始初始化环境...")
        
        
        # 2. 初始化环境 (使用传入的数据)
        env = UAVSchedulingEnv(
            graph_G=graph_G,
            point_info=point_info,
            max_min= data['max_min']
        )
        
        # 3. 初始化贪心代理
        agent = NaiveHeuristicGreedyAgent(env)

        # 4. 运行仿真并获取结果
        start_time = time.time()
        simulation_results = run_greedy_simulation(env, agent, print_log=False)
        end_time = time.time()
        
        # 5. 过滤和提取结果
        action_log = simulation_results.get('action_log', [])
        
        # 提取关键指标 
        metrics = {
            "totalSteps": simulation_results.get('total_steps', 0),
            "totalReward": simulation_results.get('total_reward', 0),
            "totalEUE": simulation_results.get('total_eue', 0.0),
            "totalPathLength": simulation_results.get('total_path_length', 0.0),
            "chargedNodeCount":simulation_results.get('charged_node_count', 0),
            "needChargedCount":simulation_results.get('total_node_count', 0),
            "totalTime":end_time-start_time
        }
        
        # 6. 组合最终返回数据，并进行类型清理 (解决 float32 问题)
        final_data = {
            "action_log": action_log, # 过滤后的路径信息
            "metrics": metrics # 关键指标
        }
        
        cleaned_data = convert_numpy_types(final_data)
        
        logging.info(f"算法运行完成，返回 {len(action_log)} 条路径步骤。")
        
        # 7. 成功响应
        return jsonify({
            "status": "success",
            "message": "Greedy scheduling completed.",
            "data": cleaned_data
        })

    except Exception as e:
        # 错误处理：捕获所有异常并返回 500
        logging.exception("Greedy simulation failed during processing.")
        return jsonify({
            "status": "error", 
            "message": f"Greedy simulation failed: {type(e).__name__}: {str(e)}"
        }), 500
    
# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)
# 全局模型变量
ppo_model = None
device = None
def make_test_env_from_data(graph_G, point_info, max_min_data, seed=None):
    """从数据创建测试环境"""
    if seed is None:
        seed = int(time.time())
    
    env = UAVSchedulingEnv(
        graph_G=graph_G,
        point_info=point_info,
        max_min=max_min_data,
        seed=seed,
        is_train=False,
        json_data=None  # 不从JSON文件加载，因为数据已经传入
    )
    return env
def initialize_model(model_path: str):
    """初始化PPO模型"""
    global ppo_model, device
    
    if not os.path.exists(model_path):
        raise FileNotFoundError(f"找不到模型文件: {model_path}")
    
    # 设置设备
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    logger.info(f"使用设备: {device}")
    
    # 加载模型
    logger.info(f"正在加载PPO模型: {model_path}")
    ppo_model = MaskablePPO.load(
        model_path, 
        device=device,
        custom_objects={
            "policy_class": CustomMaskableActorCriticPolicy
        }
    )
    logger.info("PPO模型加载完成")
    return ppo_model

def run_ppo_simulation(env, model, print_log=False):
    """
    运行PPO模型仿真
    """
    # 重置环境
    obs, info = env.reset()
    done = False
    
    # 初始化指标
    total_reward = 0.0
    total_path_length = 0.0
    charge_e = 0.0
    move_e = 0.0
    tx_e = 0.0
    active_e = 0.0
    action_log = []
    
    while not done:
        from_id = env.uav_state["node_id"]
        from_key = env.global_to_original_map[from_id]["point_data"]["node_id"]
        
        # 使用模型预测动作 (使用动作掩码保证合法性)
        action_masks = env.action_masks()
        action, _ = model.predict(obs, action_masks=action_masks, deterministic=True)
        
        # 解码动作 - PPO模型输出的是二维数组[[target_id, charge_level]]
        action = np.array(action).flatten()
        
        # 解码目标节点名称与充电时间
        target_id = int(action[0])
        target_key = env.global_to_original_map[target_id]["point_data"]["node_id"]
        decoded = env._decode_action(action)
        actual_charge_time = decoded["charge_time"][0]
        
        # 执行动作
        obs, reward, terminated, truncated, info = env.step(action)
        
        # 累计数据
        total_reward += reward
        dist = info.get("distance_needed", 0)
        total_path_length += dist
        charge_e += info["step_charge_energy"]
        move_e += info["step_move_energy"]
        tx_e += info["step_tx_energy"]
        active_e += info["active_energy"]
        
        # 记录动作日志
        action_log.append({
            "step": env.current_step,
            "uav_from": from_key,
            "uav_to": target_key,
            "charge_time": actual_charge_time,
            "distance": dist,
            "move_time":info['distance_needed']/env.agent_speed,
            "reward": reward
        })
        
        done = terminated or truncated
        
        if print_log:
            charge_tag = "<--" if actual_charge_time > 0 else "---"
            logger.info(f"Step {env.current_step:<4} | {from_key:<10} -> {target_key:<10} | "
                       f"Charge: {actual_charge_time:.2f}s{charge_tag:<5} | Dist: {dist:.2f}m | Reward: {reward:.2f}")
    
    # 计算EUE
    denom = move_e + tx_e + active_e
    total_eue = charge_e / denom if denom > 0 else 0.0
    
    # 返回结果
    simulation_results = {
        'total_steps': env.current_step,
        'total_reward': total_reward,
        'charged_node_count': info.get('node_charge_count', 0),
        'total_node_count': len(env.sensor_global_ids) + len(env.gateWay_global_ids),
        'total_eue': total_eue,
        'total_path_length': total_path_length,
        'action_log': action_log,
    }
    
    return simulation_results

def build_model_path(dataset_name):
    """根据数据集名称构建模型文件路径"""
    # 基本模型文件名
    base_model_name = "ppo_sb_mask_mlp_best_model.zip"
    
    # 构建完整路径：数据集名 + _ + 基础模型名
    # 例如：chargingScheduling_data_ppo_sb_mask_mlp_best_model.zip
    model_filename = f"{dataset_name}_{base_model_name}"
    model_path = f"./models/{model_filename}"
    
    return model_path

@app.route('/api/runPPOSimulation', methods=['POST'])
def run_ppo_api():
    """
    接收 Java 传输的仿真初始数据，调用PPO算法，并返回路径信息和关键指标。
    """
    try:
        # 1. 获取 Java 传输的 JSON 数据 (POST 请求体)
        data = request.get_json()
        if not data or 'graph_G' not in data or 'point_info' not in data:
            return jsonify({"status": "error", "message": "请求体缺少 'graph_G' 或 'point_info' 字段。"}), 400

        # graph_G = data['graph_G']
        # point_info = data['point_info']

        graph_G = {int(k): {int(nk): v for nk, v in nv.items()} 
               for k, nv in data['graph_G'].items()}
    
        # 强制转换 point_info 里的 node_id 为 int
        point_info = data['point_info']
        for p in point_info:
            p['node_id'] = int(p['node_id'])

        data_name = data['dataset_name']
        model_path=build_model_path(data_name)
        
        
        logger.info("成功接收 Java 传输的数据，开始初始化环境...")
        
        # 2. 初始化环境 (使用传入的数据)
        env = make_test_env_from_data(
            graph_G=graph_G,
            point_info=point_info,
            max_min_data=data['max_min']
        )
        
        # 3. 检查并初始化模型
        global ppo_model
        if ppo_model is None:
            ppo_model = initialize_model(model_path)
        
        # 4. 运行仿真并获取结果
        start_time = time.time()
        simulation_results = run_ppo_simulation(env, ppo_model, print_log=False)
        end_time = time.time()
        
        # 5. 过滤和提取结果
        action_log = simulation_results.get('action_log', [])
        
        # 提取关键指标 
        metrics = {
            "totalSteps": simulation_results.get('total_steps', 0),
            "totalReward": simulation_results.get('total_reward', 0),
            "totalEUE": simulation_results.get('total_eue', 0.0),
            "totalPathLength": simulation_results.get('total_path_length', 0.0),
            "chargedNodeCount": simulation_results.get('charged_node_count', 0),
            "needChargedCount": simulation_results.get('total_node_count', 0),
            "totalTime": end_time - start_time
        }
        
        # 6. 组合最终返回数据，并进行类型清理 (解决 float32 问题)
        final_data = {
            "action_log": action_log,  # 动作序列
            "metrics": metrics  # 关键指标
        }
        
        cleaned_data = convert_numpy_types(final_data)
        
        logger.info(f"PPO算法运行完成，返回 {len(action_log)} 条路径步骤。")
        logger.info(f"总奖励: {metrics['totalReward']:.2f}, EUE: {metrics['totalEUE']:.4f}, 路径长度: {metrics['totalPathLength']:.2f}m")
        
        # 7. 成功响应
        return jsonify({
            "status": "success",
            "message": "PPO scheduling completed.",
            "data": cleaned_data
        })

    except Exception as e:
        # 错误处理：捕获所有异常并返回 500
        logger.exception("PPO simulation failed during processing.")
        return jsonify({
            "status": "error", 
            "message": f"PPO simulation failed: {type(e).__name__}: {str(e)}"
        }), 500

# --- ACO 算法接口 ---
@app.route('/api/runACOSimulation', methods=['POST'])
def run_aco_api():
    """
    接收 Java 传输的仿真初始数据，调用ACO算法，并返回路径信息和关键指标。
    参考贪心算法的接口设计。
    """
    
    try:
        # 1. 获取 Java 传输的 JSON 数据 (POST 请求体)
        data = request.get_json()
        if not data or 'graph_G' not in data or 'point_info' not in data:
            return jsonify({"status": "error", "message": "请求体缺少 'graph_G' 或 'point_info' 字段。"}), 400

        # 转换graph_G中的键为整数
        graph_G = {int(k): {int(nk): v for nk, v in nv.items()} 
                   for k, nv in data['graph_G'].items()}
        
        # 强制转换 point_info 里的 node_id 为 int
        point_info = data['point_info']
        for p in point_info:
            p['node_id'] = int(p['node_id'])
        
        logger.info("成功接收 Java 传输的数据，开始初始化环境并运行ACO算法...")
        
        # 2. 初始化环境 (使用传入的数据)
        env = UAVSchedulingEnv(
            graph_G=graph_G,
            point_info=point_info,
            max_min=data['max_min']
        )
        
        # 3. 初始化ACO求解器
        # 使用默认参数，可以根据需要从请求中获取参数
        solver_params = data.get('aco_params', {})
        n_ants = solver_params.get('n_ants', 100)
        n_iterations = solver_params.get('n_iterations', 1000)
        alpha = solver_params.get('alpha', 1.0)
        beta = solver_params.get('beta', 1.5)
        rho = solver_params.get('rho', 0.1)
        Q = solver_params.get('Q', 100)
        
        solver = ACOSolver(
            env, 
            n_ants=n_ants, 
            n_iterations=n_iterations,
            alpha=alpha, 
            beta=beta, 
            rho=rho, 
            Q=Q
        )
        
        # 4. 运行ACO算法
        start_time = time.time()
        path, durations, total_reward, exec_time = solver.solve()
        end_time = time.time()
        
        # 5. 运行详细仿真以获取动作序列
        # 创建新的环境实例用于仿真
        sim_env = UAVSchedulingEnv(
            graph_G=graph_G,
            point_info=point_info,
            max_min=data['max_min']
        )
        
        # 运行仿真
        sim_env.reset()
        action_log = []
        total_path_length = 0.0
        charge_e = 0.0
        move_e = 0.0
        tx_e = 0.0
        active_e = 0.0
        charged_node_count = 0
        
        # 执行ACO生成的路径
        current_step = 1
        curr_id = sim_env.start_node_id
        
        for node_id in path:
            # 获取充电时长
            level = int(np.clip(round(durations.get(node_id, 0)), 0, sim_env.charge_time_levels - 1))
            
            # 记录动作前的状态
            from_id = sim_env.uav_state["node_id"]
            from_key = sim_env.global_to_original_map[from_id]["point_data"]["node_id"]
            target_key = sim_env.global_to_original_map[node_id]["point_data"]["node_id"]

            # 执行动作
            action = (node_id, level)
            decoded = sim_env._decode_action(action)
            actual_charge_time = decoded["charge_time"][0]
            
            # 执行动作
            obs, reward, terminated, truncated, info = sim_env.step(action)
            
            # 累计数据
            dist = info.get("distance_needed", 0)
            total_path_length += dist
            
            # 记录动作日志
            action_log.append({
                "step": current_step,
                "uav_from": from_key,
                "uav_to": target_key,
                "charge_time": actual_charge_time,
                "distance": dist,
                "move_time": dist / sim_env.agent_speed,
                "reward": reward
            })
            
            charge_e += info.get("step_charge_energy", 0)
            move_e += info.get("step_move_energy", 0)
            tx_e += info.get("step_tx_energy", 0)
            active_e += info.get("active_energy", 0)
            
            if node_id != sim_env.start_node_id:
                charged_node_count += 1
            
            current_step += 1
            curr_id = node_id
            
            if terminated or truncated:
                break
        
        # 计算EUE
        denom = move_e + tx_e + active_e
        total_eue = charge_e / denom if denom > 0 else 0.0
        
        # 6. 提取关键指标
        metrics = {
            "totalSteps": len(action_log),
            "totalReward": total_reward,
            "totalEUE": total_eue,
            "totalPathLength": total_path_length,
            "chargedNodeCount": charged_node_count,
            "needChargedCount": len(sim_env.sensor_global_ids) + len(sim_env.gateWay_global_ids),
            "totalTime": end_time - start_time,
            "acoExecutionTime": exec_time
        }
        
        # 7. 组合最终返回数据，并进行类型清理
        final_data = {
            "action_log": action_log,  # 动作序列
            "metrics": metrics,  # 关键指标
            "aco_path": [sim_env.global_to_original_map[nid]["point_data"]["node_id"] for nid in path if nid != sim_env.start_node_id]  # ACO优化路径
        }
        
        cleaned_data = convert_numpy_types(final_data)
        
        logger.info(f"ACO算法运行完成，返回 {len(action_log)} 条路径步骤。")
        logger.info(f"ACO路径长度: {len(path)}个节点，总奖励: {metrics['totalReward']:.2f}")
        logger.info(f"EUE: {metrics['totalEUE']:.4f}, 路径长度: {metrics['totalPathLength']:.2f}m")
        
        # 8. 成功响应
        return jsonify({
            "status": "success",
            "message": "ACO scheduling completed.",
            "data": cleaned_data
        })

    except Exception as e:
        # 错误处理：捕获所有异常并返回 500
        logger.exception("ACO simulation failed during processing.")
        return jsonify({
            "status": "error", 
            "message": f"ACO simulation failed: {type(e).__name__}: {str(e)}"
        }), 500

# --- NPSO 算法接口 ---
@app.route('/api/runNPSOSimulation', methods=['POST'])
def run_npso_api():
    """
    接收 Java 传输的仿真初始数据，调用NPSO算法，并返回路径信息和关键指标。
    参考贪心算法的接口设计。
    """
    
    try:
        # 1. 获取 Java 传输的 JSON 数据 (POST 请求体)
        data = request.get_json()
        if not data or 'graph_G' not in data or 'point_info' not in data:
            return jsonify({"status": "error", "message": "请求体缺少 'graph_G' 或 'point_info' 字段。"}), 400

        # 转换graph_G中的键为整数
        graph_G = {int(k): {int(nk): v for nk, v in nv.items()} 
                   for k, nv in data['graph_G'].items()}
        
        # 强制转换 point_info 里的 node_id 为 int
        point_info = data['point_info']
        for p in point_info:
            p['node_id'] = int(p['node_id'])
        
        logger.info("成功接收 Java 传输的数据，开始初始化环境并运行NPSO算法...")
        
        # 2. 初始化环境 (使用传入的数据)
        env = UAVSchedulingEnv(
            graph_G=graph_G,
            point_info=point_info,
            max_min=data['max_min']
        )
        
        # 3. 初始化NPSO求解器
        # 使用默认参数，可以根据需要从请求中获取参数
        solver_params = data.get('npso_params', {})
        n_particles = solver_params.get('n_particles', 100)
        max_iterations = solver_params.get('max_iterations', 1000)
        
        start_time = time.time()
        solver = NPSOSolver(env, n_particles=n_particles, max_iterations=max_iterations)
        
        # 4. 运行NPSO算法
        best_fit = solver.solve()
        exec_time = time.time() - start_time
        
        # 5. 运行详细仿真以获取动作序列
        sim_env = UAVSchedulingEnv(
            graph_G=graph_G,
            point_info=point_info,
            max_min=data['max_min']
        )
        
        # 运行NPSO详细仿真
        total_reward, logs, stats = run_detailed_simulation_npso(sim_env, solver.gbest_path, solver.gbest_times)
        
        # 6. 转换日志格式以匹配贪心算法的action_log格式
        action_log = []
        total_path_length = 0.0
        charge_e = 0.0
        move_e = 0.0
        tx_e = 0.0
        active_e = 0.0
        
        # 创建新环境用于详细仿真
        sim_env2 = UAVSchedulingEnv(
            graph_G=graph_G,
            point_info=point_info,
            max_min=data['max_min']
        )
        sim_env2.reset(seed=42)
        
        # 执行最优路径
        for i, node_id in enumerate(solver.gbest_path):
            # 获取充电比例并映射为level
            ratio = solver.gbest_times[node_id]
            time_level = max(0, min(sim_env2.charge_time_levels - 1, 
                                  int(ratio * (sim_env2.charge_time_levels - 1))))
            
            # 记录动作前的状态
            from_id = sim_env2.uav_state["node_id"]
            from_key = sim_env2.global_to_original_map[from_id]["point_data"]["node_id"]
            target_key = sim_env2.global_to_original_map[node_id]["point_data"]["node_id"]
            
            # 执行动作
            action = (node_id, time_level)
            decoded = sim_env2._decode_action(action)
            actual_charge_time = decoded["charge_time"][0]
            
            obs, reward, terminated, truncated, info = sim_env2.step(action)
            
            # 累计数据
            dist = info.get("distance_needed", 0)
            total_path_length += dist
            charge_e += info.get("step_charge_energy", 0)
            move_e += info.get("step_move_energy", 0)
            tx_e += info.get("step_tx_energy", 0)
            active_e += info.get("active_energy", 0)
            
            # 记录动作日志（格式与贪心算法一致）
            action_log.append({
                "step": i + 1,
                "uav_from": from_key,
                "uav_to": target_key,
                "charge_time": actual_charge_time,
                "distance": dist,
                "move_time": dist / sim_env2.agent_speed,
                "reward": reward
            })
            
            if terminated or truncated:
                break
        
        # 计算EUE
        denom = move_e + tx_e + active_e
        total_eue = charge_e / denom if denom > 0 else 0.0
        
        # 7. 提取关键指标
        metrics = {
            "totalSteps": len(action_log),
            "totalReward": total_reward,
            "totalEUE": total_eue,
            "totalPathLength": stats.get("total_path_length", total_path_length),
            "chargedNodeCount": stats.get("charged_nodes", 0),
            "needChargedCount": len(sim_env2.sensor_global_ids) + len(sim_env2.gateWay_global_ids),
            "totalTime": time.time() - start_time,
            "npsoExecutionTime": exec_time,
            "totalStayTime": stats.get("total_stay_time", 0)
        }
        
        # 8. 组合最终返回数据，并进行类型清理
        # 转换路径格式：使用原始节点ID而不是全局ID
        npso_path = []
        for nid in solver.gbest_path:
            if nid != sim_env2.start_node_id:
                original_node_id = sim_env2.global_to_original_map[nid]["point_data"]["node_id"]
                npso_path.append(str(original_node_id))
        
        final_data = {
            "action_log": action_log,  # 动作序列（与贪心算法格式一致）
            "metrics": metrics,  # 关键指标
            "npso_path": npso_path,  # NPSO优化路径（原始节点ID）
            "node_times": {str(sim_env2.global_to_original_map[nid]["point_data"]["node_id"]): 
                          float(solver.gbest_times[nid]) 
                          for nid in solver.gbest_times}  # 每个节点的充电时间比例
        }
        
        cleaned_data = convert_numpy_types(final_data)
        
        logger.info(f"NPSO算法运行完成，返回 {len(action_log)} 条路径步骤。")
        logger.info(f"NPSO适应度: {best_fit:.2f}, 总奖励: {metrics['totalReward']:.2f}")
        logger.info(f"EUE: {metrics['totalEUE']:.4f}, 路径长度: {metrics['totalPathLength']:.2f}m")
        logger.info(f"充电节点数: {metrics['chargedNodeCount']}/{metrics['needChargedCount']}")
        
        # 9. 成功响应
        return jsonify({
            "status": "success",
            "message": "NPSO scheduling completed.",
            "data": cleaned_data
        })

    except Exception as e:
        # 错误处理：捕获所有异常并返回 500
        logger.exception("NPSO simulation failed during processing.")
        return jsonify({
            "status": "error", 
            "message": f"NPSO simulation failed: {type(e).__name__}: {str(e)}"
        }), 500

if __name__ == '__main__':
    # 运行 Flask 应用
    # 确保 Java 后端可以访问到这个端口 (例如 5000)
    # app.run(host='0.0.0.0', port=5000)
    app.run(host='127.0.0.1', port=5000)
