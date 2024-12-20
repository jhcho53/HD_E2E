#!/usr/bin/env python3

import rospy
from collections import deque
import numpy as np
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from morai_msgs.msg import EgoVehicleStatus

class RoutePlanner:
    def __init__(self, min_distance, max_distance):
        self.route = deque()
        self.route_distances = deque()
        self.min_distance = min_distance
        self.max_distance = max_distance
        self.is_last = False

    def set_route(self, global_plan):
        self.route.clear()
        for pos, _ in global_plan:
            pos = np.array([pos.x, pos.y, pos.z])
            self.route.append((pos, None))

        self.route_distances.clear()
        self.route_distances.append(0.0)
        for i in range(1, len(self.route)):
            diff = self.route[i][0] - self.route[i - 1][0]
            distance = np.linalg.norm(diff)
            self.route_distances.append(distance)

    def run_step(self, gps):
        if len(self.route) <= 2:
            self.is_last = True
            return list(self.route)

        to_pop = 0
        farthest_in_range = -np.inf
        cumulative_distance = 0.0

        for i in range(1, len(self.route)):
            if cumulative_distance > self.max_distance:
                break

            cumulative_distance += self.route_distances[i]
            diff = self.route[i][0] - gps
            distance = np.linalg.norm(diff)

            if distance <= self.min_distance and distance > farthest_in_range:
                farthest_in_range = distance
                to_pop = i

        for _ in range(to_pop):
            if len(self.route) > 2:
                self.route.popleft()
                self.route_distances.popleft()

        updated_route = list(self.route)
        return updated_route

def convert_route_to_path(route):
    """
    경로 리스트를 nav_msgs/Path 메시지로 변환합니다.
    :param route: 경로 리스트 [(numpy array)].
    :return: Path 메시지
    """
    path_msg = Path()
    path_msg.header.stamp = rospy.Time.now()
    path_msg.header.frame_id = "map"  # 경로 기준 좌표계 설정

    for pos, _ in route:
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = pos[0]
        pose.pose.position.y = pos[1]
        pose.pose.position.z = pos[2]
        pose.pose.orientation.w = 1.0  # 단순한 경로로 방향은 설정하지 않음
        path_msg.poses.append(pose)

    return path_msg

def route_planner_node():
    rospy.init_node('route_planner', anonymous=True)

    min_distance = rospy.get_param('~min_distance', 5.0)
    max_distance = rospy.get_param('~max_distance', 50.0)
    file_path = rospy.get_param('~global_path_file', 'hmg_mission11_global_path.txt')

    planner = RoutePlanner(min_distance, max_distance)

    # Path Publisher 추가
    path_pub = rospy.Publisher('/global_path', Path, queue_size=20)

    # 글로벌 경로 파일에서 읽기
    def load_global_path_from_file(file_path):
        global_plan = []
        try:
            with open(file_path, 'r') as file:
                for line in file:
                    parts = line.strip().split()
                    if len(parts) >= 4:
                        x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                        position = Point(x, y, z)
                        global_plan.append((position, None))
            rospy.loginfo("Global path loaded successfully from file.")
        except Exception as e:
            rospy.logerr(f"Error loading global path file: {e}")
        return global_plan

    global_plan = load_global_path_from_file(file_path)
    if not global_plan:
        rospy.logerr("No global plan loaded. Exiting...")
        return

    planner.set_route(global_plan)

    def gps_callback(data):
        gps_position = np.array([data.position.x, data.position.y, data.position.z])
        updated_route = planner.run_step(gps_position)

        # Path 메시지로 변환 및 퍼블리시
        path_msg = convert_route_to_path(updated_route)
        path_pub.publish(path_msg)

        rospy.loginfo("Published updated path.")

    # 차량 상태 토픽 구독
    rospy.Subscriber('/Ego_topic', EgoVehicleStatus, gps_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        route_planner_node()
    except rospy.ROSInterruptException:
        pass
