from typing import List, Tuple
import random
from scipy.spatial.distance import euclidean
import json
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose
import rospy

def genetate_exp_points_assignment(map_size, block_size, exp_ids: List[int]):
    uav_ids = exp_ids

    def generate_exploration_points(map_size: Tuple[int, int, int], block_size: float) -> List[Tuple[float, float, float]]:
        exploration_points = []
        transform = (map_size[0] / 2, map_size[1] / 2, map_size[2] / 2)
        for x in range(int(block_size/2), int(map_size[0]), int(block_size)):
            for y in range(int(block_size/2), int(map_size[1]), int(block_size)):
                exploration_points.append((x-transform[0], y-transform[1], map_size[2] - 1))
        return exploration_points

    # 随机选择初始点
    def initialize_centers(grid_points, uav_ids):
        return {uav: random.choice(grid_points) for uav in uav_ids}

    # 分配点给无人机
    def assign_points(grid_points, uav_ids, centers):
        assignments = {uav: [centers[uav]] for uav in uav_ids}
        remaining_points = set(grid_points) - set(centers.values())
        
        while remaining_points:
            for uav in uav_ids:
                if not remaining_points:
                    break
                # 找最近的点
                last_point = assignments[uav][-1]
                nearest_point = min(remaining_points, key=lambda p: euclidean(last_point, p))
                assignments[uav].append(nearest_point)
                remaining_points.remove(nearest_point)
        
        return assignments

    # 路径优化（可选）
    def optimize_path(points):
        if not points:
            return []
        visited = [points[0]]
        points = points[1:]
        while points:
            last_point = visited[-1]
            next_point = min(points, key=lambda p: euclidean(last_point, p))
            visited.append(next_point)
            points.remove(next_point)
        return visited

    # 主流程
    grid_points = generate_exploration_points(map_size, block_size)
    centers = initialize_centers(grid_points, uav_ids)
    assignments = assign_points(grid_points, uav_ids, centers)
    optimized_paths = {uav: optimize_path(points) for uav, points in assignments.items()}

    return optimized_paths

def check_if_detect_feature(feature_database, detect_func) -> List:
    dynamic_feature_database = json.loads(feature_database)
    detected_features = []
    for feature in dynamic_feature_database.values():
        if not feature['detected'] and not feature['finished']:
            if detect_func(tuple(feature['position'])):
                detected_features.append(feature)
    return detected_features

def show_fov(publisher, id, dist, width, hight, pose: Pose):
    marker = Marker()
    marker.id = 0
    marker.ns = f"{id}_fov"
    marker.header.stamp = rospy.Time.now()
    marker.header.frame_id = "world"
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.scale.x = 0.1
    marker.pose = pose
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0

    half_width = width / 2
    half_height = hight / 2
    points = [
        (0, 0, 0),
        (dist, -half_width, -half_height),
        (0, 0, 0),
        (dist, half_width, -half_height),
        (0, 0, 0),
        (dist, half_width, half_height),
        (0, 0, 0),
        (dist, -half_width, half_height),
        (dist, -half_width, -half_height),
        (dist, half_width, -half_height),
        (dist, half_width, -half_height),
        (dist, half_width, half_height),
        (dist, half_width, half_height),
        (dist, -half_width, half_height),
        (dist, -half_width, half_height),
        (dist, -half_width, -half_height)
    ]

    for point in points:
        p = Point()
        p.x, p.y, p.z = point
        marker.points.append(p)
    publisher.publish(marker)