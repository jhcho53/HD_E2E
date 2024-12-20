#!/usr/bin/env python
import rospy
import math
import numpy as np
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from time import time


class VehicleController:
    def __init__(self):
        # ROS 노드 초기화
        rospy.init_node('vehicle_controller', anonymous=True)

        # Publisher 설정
        self.ctrl_cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=10)

        # Subscriber 설정
        rospy.Subscriber('/global_path', Path, self.path_callback)
        rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.ego_callback)

        # 차량 상태 및 경로 데이터 초기화
        self.global_path = []
        self.current_pose = None
        self.current_heading = 0.0
        self.previous_pose = None
        self.previous_time = None
        self.current_speed = 0.0

        rospy.loginfo("Vehicle Controller Node Initialized.")

    def path_callback(self, msg):
        """ 글로벌 경로를 구독하는 콜백 함수 """
        self.global_path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        rospy.loginfo_once(f"Global path received with {len(self.global_path)} waypoints.")

    def ego_callback(self, msg):
        """ 차량 상태를 구독하는 콜백 함수 """
        self.current_pose = (msg.position.x, msg.position.y)
        self.current_heading = math.radians(msg.heading)

        # 속도 계산
        if self.previous_pose and self.previous_time:
            current_time = time()
            time_delta = current_time - self.previous_time
            if time_delta > 0:
                distance = np.linalg.norm(np.array(self.current_pose) - np.array(self.previous_pose))
                self.current_speed = distance / time_delta
                rospy.loginfo(f"Current speed: {self.current_speed:.2f} m/s")
            self.previous_time = current_time
        else:
            self.previous_time = time()

        self.previous_pose = self.current_pose

    def find_target_waypoint(self):
        """ 경로 상에서 가장 가까운 웨이포인트를 찾아 반환 """
        if not self.global_path or self.current_pose is None:
            return None

        closest_distance = float('inf')
        closest_index = 0

        # 가장 가까운 웨이포인트 찾기
        for i, waypoint in enumerate(self.global_path):
            distance = np.linalg.norm(np.array(waypoint) - np.array(self.current_pose))
            if distance < closest_distance:
                closest_distance = distance
                closest_index = i

        # 몇 개 앞의 웨이포인트를 목표로 설정
        target_index = min(closest_index + 5, len(self.global_path) - 1)
        return self.global_path[target_index]

    def control_vehicle(self, target_speed):
        """ 차량 제어 명령을 계산하고 발행 """
        target_point = self.find_target_waypoint()
        if target_point is None:
            rospy.logwarn("No valid target waypoint found.")
            return

        # 목표점과 현재 위치 차이 계산
        dx = target_point[0] - self.current_pose[0]
        dy = target_point[1] - self.current_pose[1]
        target_heading = math.atan2(dy, dx)

        # 헤딩 에러 계산
        heading_error = target_heading - self.current_heading
        heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi  # 각도 정규화

        # 조향 및 가속도 계산
        steering = np.clip(heading_error, -0.5, 0.5)  # 조향각 제한

        # 조건에 따른 가속도 및 브레이크 조절
        if self.current_speed >= 60:
            accel = 0.0
            brake = 0.0
        elif self.current_speed >= 20 and abs(steering) > 0.1:
            accel = 0.0
            brake = 0.4
        else:
            accel = 0.2 if abs(steering) < 0.1 else 0.1
            brake = 0.0

        # CtrlCmd 메시지 생성 및 발행
        ctrl_cmd = CtrlCmd()
        ctrl_cmd.longlCmdType = 1
        ctrl_cmd.velocity = target_speed
        ctrl_cmd.accel = accel
        ctrl_cmd.steering = steering
        ctrl_cmd.brake = brake

        self.ctrl_cmd_pub.publish(ctrl_cmd)
        rospy.loginfo(f"Control -> Steering: {steering:.2f}, Accel: {accel:.2f}, Brake: {brake:.2f}")

    def run(self):
        """ 노드 실행 루프 """
        rate = rospy.Rate(10)  # 10 Hz
        target_speed = 5.0  # 목표 속도 (m/s)

        while not rospy.is_shutdown():
            if self.current_pose:
                self.control_vehicle(target_speed)
            else:
                rospy.logwarn("Waiting for ego vehicle position...")
            rate.sleep()


if __name__ == '__main__':
    try:
        controller = VehicleController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
