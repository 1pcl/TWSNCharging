# RoadNetwork.py
import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Optional
from collections import defaultdict
import json

@dataclass
class Intersection:
    """Road intersection node"""
    id: str
    x: float  # meters
    y: float  # meters
    degree: int = 0
    roads: List[str] = field(default_factory=list)
    
@dataclass
class Road:
    """Road segment between two intersections"""
    id: str
    from_intersection: str
    to_intersection: str
    length: float  # meters

class RoadNetwork:
    """Road network with all distances in meters"""
    
    def __init__(self):
        self.adjacency: Dict[str, Dict[str, str]] = defaultdict(dict)
        self.intersections: Dict[str, Intersection] = {}
        self.roads: Dict[str, Road] = {}
        
    def add_intersection(self, inter_id: str, x: float, y: float) -> Intersection:
        inter = Intersection(id=inter_id, x=x, y=y)
        self.intersections[inter_id] = inter
        return inter
    
    def add_road(self, road_id: str, from_id: str, to_id: str, length: float) -> Road:
        road = Road(id=road_id, from_intersection=from_id, to_intersection=to_id, length=length)
        self.roads[road_id] = road
        
        self.adjacency[from_id][to_id] = road_id
        self.adjacency[to_id][from_id] = road_id
        
        self.intersections[from_id].degree += 1
        self.intersections[to_id].degree += 1
        self.intersections[from_id].roads.append(road_id)
        self.intersections[to_id].roads.append(road_id)
        
        return road
    
    def get_statistics(self) -> Dict:
        degrees = [inter.degree for inter in self.intersections.values()]
        road_lengths = [road.length for road in self.roads.values()]
        
        total_length_km = sum(road_lengths) / 1000
        
        # 计算区域面积
        if self.intersections:
            x_coords = [inter.x for inter in self.intersections.values()]
            y_coords = [inter.y for inter in self.intersections.values()]
            area_width_km = (max(x_coords) - min(x_coords)) / 1000
            area_height_km = (max(y_coords) - min(y_coords)) / 1000
            area_km2 = area_width_km * area_height_km
        else:
            area_km2 = 0
        
        return {
            'total_intersections': len(self.intersections),
            'total_roads': len(self.roads),
            'avg_degree': np.mean(degrees) if degrees else 0,
            'max_degree': max(degrees) if degrees else 0,
            'min_degree': min(degrees) if degrees else 0,
            'total_road_length_m': sum(road_lengths),
            'total_road_length_km': total_length_km,
            'avg_road_length_m': np.mean(road_lengths) if road_lengths else 0,
            'avg_road_length_km': np.mean(road_lengths) / 1000 if road_lengths else 0,
            'area_km2': area_km2,
            'road_density_km_per_km2': total_length_km / area_km2 if area_km2 > 0 else 0,
        }
    
    def visualize(self, title="Road Network", figsize=(14, 10)):
        fig, ax = plt.subplots(figsize=figsize)
        
        # Draw roads
        for road in self.roads.values():
            from_inter = self.intersections[road.from_intersection]
            to_inter = self.intersections[road.to_intersection]
            ax.plot([from_inter.x, to_inter.x], [from_inter.y, to_inter.y], 
                   'gray', linewidth=1, alpha=0.6)
        
        # Draw intersections
        x_coords = [inter.x for inter in self.intersections.values()]
        y_coords = [inter.y for inter in self.intersections.values()]
        ax.scatter(x_coords, y_coords, c='black', s=10, marker='o', zorder=5, alpha=0.7)
        
        # 坐标轴显示公里
        ax.set_xlabel('X (km)', fontsize=12)
        ax.set_ylabel('Y (km)', fontsize=12)
        
        # 将刻度转换为公里
        xticks = ax.get_xticks()
        yticks = ax.get_yticks()
        ax.set_xticklabels([f'{x/1000:.1f}' for x in xticks])
        ax.set_yticklabels([f'{y/1000:.1f}' for y in yticks])
        
        ax.set_title(title, fontsize=14)
        ax.grid(True, alpha=0.2, linestyle='--')
        ax.set_aspect('equal')
        
        return fig, ax
    
    def save_to_json(self, filename: str):
        """Save network to JSON file"""
        data = {
            'statistics': self.get_statistics(),
            'unit': 'meters',
            'intersections': [
                {
                    'id': inter.id,
                    'x_m': inter.x,
                    'y_m': inter.y,
                    'degree': inter.degree,
                    'roads': inter.roads
                }
                for inter in self.intersections.values()
            ],
            'roads': [
                {
                    'id': road.id,
                    'from': road.from_intersection,
                    'to': road.to_intersection,
                    'length_m': road.length
                }
                for road in self.roads.values()
            ],
            'adjacency': {
                inter_id: list(neighbors.keys())
                for inter_id, neighbors in self.adjacency.items()
            }
        }
        
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
        
        print(f"Network saved to: {filename}")
    
    def load_from_json(self, filename: str):
        """Load network from JSON file"""
        with open(filename, 'r') as f:
            data = json.load(f)
        
        self.adjacency.clear()
        self.intersections.clear()
        self.roads.clear()
        
        # Load intersections
        for inter_data in data['intersections']:
            if 'x_m' in inter_data:
                x = inter_data['x_m']
                y = inter_data['y_m']
            else:
                x = inter_data.get('x', 0) * 1000
                y = inter_data.get('y', 0) * 1000
            
            inter = Intersection(
                id=inter_data['id'],
                x=x,
                y=y,
                degree=inter_data['degree'],
                roads=inter_data['roads']
            )
            self.intersections[inter.id] = inter
        
        # Load roads
        for road_data in data['roads']:
            if 'length_m' in road_data:
                length = road_data['length_m']
            else:
                length = road_data.get('length', 0) * 1000
            
            road = Road(
                id=road_data['id'],
                from_intersection=road_data['from'],
                to_intersection=road_data['to'],
                length=length
            )
            self.roads[road.id] = road
        
        # Rebuild adjacency
        for inter_id, neighbors in data['adjacency'].items():
            for neighbor_id in neighbors:
                road = self.get_road_between(inter_id, neighbor_id)
                if road:
                    self.adjacency[inter_id][neighbor_id] = road.id
        
        print(f"Network loaded from: {filename}")
        print(f"  - Intersections: {len(self.intersections)}")
        print(f"  - Roads: {len(self.roads)}")
        print(f"  - Unit: meters")
    
    def get_road_between(self, inter1: str, inter2: str) -> Optional[Road]:
        if inter2 in self.adjacency[inter1]:
            road_id = self.adjacency[inter1][inter2]
            return self.roads[road_id]
        return None


def generate_urban_network(num_intersections: int = 400, area_size_km: float = 20.0, seed: int = 0):
    """
    Generate a realistic urban road network
    
    Parameters:
    -----------
    num_intersections : int
        Number of intersections to generate
    area_size_km : float
        Size of the urban area in kilometers (e.g., 20 km x 20 km)
    seed : int
        Random seed for reproducibility
    
    Returns:
    --------
    RoadNetwork : Generated road network (all units in meters)
    """
    network = RoadNetwork()
    
    # Convert to meters for internal calculations
    area_size_m = area_size_km * 1000
    
    # Calculate grid size
    grid_size = int(np.sqrt(num_intersections))
    spacing_m = area_size_m / (grid_size - 1)
    
    np.random.seed(seed)
    
    # Create intersections with slight deformations
    intersection_ids = {}
    inter_counter = 0
    
    # Urban block size: 100-200m, perturbation up to 30m
    perturbation_m = min(30.0, spacing_m * 0.2)
    
    for i in range(grid_size):
        for j in range(grid_size):
            x_base = j * spacing_m
            y_base = i * spacing_m
            
            # Small random offset for realistic irregular layout
            x = x_base + np.random.uniform(-perturbation_m, perturbation_m)
            y = y_base + np.random.uniform(-perturbation_m, perturbation_m)
            
            # Keep within bounds with margin
            margin_m = 50.0
            x = np.clip(x, margin_m, area_size_m - margin_m)
            y = np.clip(y, margin_m, area_size_m - margin_m)
            
            inter_id = f"I{inter_counter}"
            network.add_intersection(inter_id, x, y)
            intersection_ids[(i, j)] = inter_id
            inter_counter += 1
    
    # Connect intersections
    road_counter = 0
    
    # Horizontal connections (east-west streets)
    for i in range(grid_size):
        for j in range(grid_size - 1):
            if np.random.random() < 0.90:  # 90% connectivity
                from_id = intersection_ids[(i, j)]
                to_id = intersection_ids[(i, j + 1)]
                from_inter = network.intersections[from_id]
                to_inter = network.intersections[to_id]
                length = np.sqrt((from_inter.x - to_inter.x)**2 + (from_inter.y - to_inter.y)**2)
                
                road_id = f"R{road_counter}"
                network.add_road(road_id, from_id, to_id, length)
                road_counter += 1
    
    # Vertical connections (north-south streets)
    for i in range(grid_size - 1):
        for j in range(grid_size):
            if np.random.random() < 0.90:
                from_id = intersection_ids[(i, j)]
                to_id = intersection_ids[(i + 1, j)]
                from_inter = network.intersections[from_id]
                to_inter = network.intersections[to_id]
                length = np.sqrt((from_inter.x - to_inter.x)**2 + (from_inter.y - to_inter.y)**2)
                
                road_id = f"R{road_counter}"
                network.add_road(road_id, from_id, to_id, length)
                road_counter += 1
    
    # Add diagonal connections for realism (optional, 5%)
    for i in range(grid_size - 1):
        for j in range(grid_size - 1):
            if np.random.random() < 0.05:
                from_id = intersection_ids[(i, j)]
                to_id = intersection_ids[(i + 1, j + 1)]
                from_inter = network.intersections[from_id]
                to_inter = network.intersections[to_id]
                length = np.sqrt((from_inter.x - to_inter.x)**2 + (from_inter.y - to_inter.y)**2)
                
                road_id = f"R{road_counter}"
                network.add_road(road_id, from_id, to_id, length)
                road_counter += 1
    
    return network


# ============= Example Usage =============

if __name__ == "__main__":
    # 真实城市尺度配置
    # 小城市: 10km × 10km (100 km²),100
    # 中城市: 15km × 15km (900 km²),175
    # 大城市: 20km × 20km (400 km²),250
    
    
    AREA_SIZE_KM = 15.0  
    NUM_INTERSECTIONS = 175 
    
    print("="*70)
    print(f"Urban Road Network Generation")
    print(f"  - Area: {AREA_SIZE_KM} km × {AREA_SIZE_KM} km = {AREA_SIZE_KM**2:.0f} km²")
    print(f"  - Intersections: ~{NUM_INTERSECTIONS}")
    print("="*70)
    
    # Generate network
    network = generate_urban_network(
        num_intersections=NUM_INTERSECTIONS,
        area_size_km=AREA_SIZE_KM,
        seed=42
    )
    
    if len(network.intersections) > 0:
        stats = network.get_statistics()
        print(f"\nNetwork Statistics:")
        print(f"  Total intersections: {stats['total_intersections']}")
        print(f"  Total roads: {stats['total_roads']}")
        print(f"  Average degree: {stats['avg_degree']:.2f}")
        print(f"  Total road length: {stats['total_road_length_km']:.1f} km")
        print(f"  Average road length: {stats['avg_road_length_km']:.2f} km")
        print(f"  Area: {stats['area_km2']:.1f} km²")
        print(f"  Road density: {stats['road_density_km_per_km2']:.2f} km/km²")
        
        # Save to file
        import os
        os.makedirs("./data", exist_ok=True)
        network.save_to_json(f"./data/road_network_{stats['total_intersections']}_intersections.json")
        
        # Visualize
        fig, ax = network.visualize(title=f"Urban Road Network - {AREA_SIZE_KM}km × {AREA_SIZE_KM}km")
        plt.show()
    else:
        print("Failed to generate network")