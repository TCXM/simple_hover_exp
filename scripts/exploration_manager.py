#! /usr/bin/env python
import rospy, utils
from typing import Dict, Tuple, List
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose
from visualization_msgs.msg import Marker
from std_msgs.msg import String

REACHED_THRESHOLD = 1
BLOCK_SIZE = 5


class Explorer:
    def __init__(self, id, odom_topic):
        self.id = id

        # planning
        self.waypoint_pub = rospy.Publisher(
            f"/drone_{id}_planning/waypoint", PoseStamped, queue_size=10
        )
        self.odom_sub = rospy.Subscriber(
            f"/drone_{id}_{odom_topic}", Odometry, self.odom_cb
        )
        self.has_odom = False

        # state machine
        self.state = "idle"
        self.state_timer = rospy.Timer(rospy.Duration(1), self.state_timer_cb)

        # visualization
        self.vis_pub = rospy.Publisher(f"/extended_vis", Marker, queue_size=10)
        self.visualize_timer = rospy.Timer(rospy.Duration(0.1), self.visualize_timer_cb)

        # world interface
        self.detect_objects_pub = rospy.Publisher(
            "/detect_region", String, queue_size=10
        )

    def odom_cb(self, msg: Odometry):
        self.current_pose = msg.pose.pose
        self.has_odom = True

    def assign_exp_points(self, points: List[Tuple[float, float, float]]):
        self.exploration_points = points
        self.state = "exploring"
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = self.exploration_points[0][0]
        pose.pose.position.y = self.exploration_points[0][1]
        pose.pose.position.z = self.exploration_points[0][2]
        pose.pose.orientation.w = 1
        self.waypoint_pub.publish(pose)

    def check_reached(self, current_point, target_point) -> bool:
        distance_square = (current_point[0] - target_point[0]) ** 2 + (
            current_point[1] - target_point[1]
        ) ** 2
        if distance_square < REACHED_THRESHOLD:
            return True
        else:
            return False

    def state_timer_cb(self, event):
        if not self.has_odom:
            return
        if self.state == "idle":
            return
        elif self.state == "exploring":
            if self.check_reached(
                (
                    self.current_pose.position.x,
                    self.current_pose.position.y,
                    self.current_pose.position.z,
                ),
                self.exploration_points[0],
            ):
                self.detect_feature(self.exploration_points[0], BLOCK_SIZE)
                self.exploration_points.append(self.exploration_points.pop(0))
                pose = PoseStamped()
                pose.header.frame_id = "world"
                pose.pose.position.x = self.exploration_points[0][0]
                pose.pose.position.y = self.exploration_points[0][1]
                pose.pose.position.z = self.exploration_points[0][2]
                pose.pose.orientation.w = 1
                self.waypoint_pub.publish(pose)

    def visualize_timer_cb(self, event):
        if not self.has_odom:
            return
        pose = Pose()
        pose.position = self.current_pose.position
        pose.orientation.x = 0
        pose.orientation.y = 0.707
        pose.orientation.z = 0
        pose.orientation.w = 0.707
        utils.show_fov(self.vis_pub, self.id, 5, BLOCK_SIZE, BLOCK_SIZE, pose)

    def detect_feature(self, current_exp_point, block_size) -> list:
        min_x = current_exp_point[0] - block_size / 2
        max_x = current_exp_point[0] + block_size / 2
        min_y = current_exp_point[1] - block_size / 2
        max_y = current_exp_point[1] + block_size / 2
        self.detect_objects_pub.publish(f"{min_x},{min_y}~{max_x},{max_y}")


class ExplorationManager:
    def __init__(
        self, explorer_uav_ids: list, odom_topic: str, map_size: Tuple[int, int, int]
    ):

        # initialize explorers
        self.explorers: Dict[int, Explorer] = {}
        for id in explorer_uav_ids:
            self.explorers[id] = Explorer(id, odom_topic)

        # assign exploration points
        assignments = utils.genetate_exp_points_assignment(
            map_size, BLOCK_SIZE, explorer_uav_ids
        )
        rospy.sleep(2)  # 必须等，不然太快了roscore反应不过来
        for id, points in assignments.items():
            self.explorers[id].assign_exp_points(points)
        rospy.loginfo("Exploration points assigned")


if __name__ == "__main__":

    rospy.init_node("exploration_manager")
    explorer_uav_ids = str(rospy.get_param("~explorer_uav_ids")).split(",")
    odom_topic = rospy.get_param("~odom_topic", "odom")
    map_size_x = rospy.get_param("~map_size_x", 100)
    map_size_y = rospy.get_param("~map_size_y", 100)
    map_size_z = rospy.get_param("~map_size_z", 100)
    map_size = (map_size_x, map_size_y, map_size_z)

    ExplorationManager(explorer_uav_ids, odom_topic, map_size)

    rospy.spin()
