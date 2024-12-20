#!/usr/bin/env python3


import os
import json
import datetime
import pathlib
import time
import cv2
from collections import deque

import torch
import torchvision
import numpy as np
from PIL import Image
from pyquaternion import Quaternion

from stp3.utils.geometry import (
    update_intrinsics,
    mat2pose_vec,
    invert_matrix_egopose_numpy,
)
import stp3.utils.sampler as trajectory_sampler

# from leaderboard.autoagents import autonomous_agent
# from team_code.planner import RoutePlanner
from stp3.trainer import TrainingModule
from stp3.datas.CarlaData import scale_and_crop_image

#모라이 ROS 메세지 import
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus
from sensor_msgs.msg import CompressedImage
import rospy
from cv_bridge import CvBridge

class PIDController(object):
    def __init__(self, K_P=1.0, K_I=0.0, K_D=0.0, n=20):
        self._K_P = K_P
        self._K_I = K_I
        self._K_D = K_D

        self._window = deque([0 for _ in range(n)], maxlen=n)
        self._max = 0.0
        self._min = 0.0

    def step(self, error):
        if isinstance(error, np.ndarray):
            error = np.mean(error)  # error가 배열일 경우 평균값을 사용
        self._window.append(error)
        self._max = max(self._max, abs(error))
        self._min = -abs(self._max)

        if len(self._window) >= 2:
            integral = np.mean(self._window)
            derivative = (self._window[-1] - self._window[-2])
        else:
            integral = 0.0
            derivative = 0.0

        return self._K_P * error + self._K_I * integral + self._K_D * derivative

class RoutePlanner:
    def __init__(self, min_distance, max_distance):
        self.route = deque()
        self.route_distances = deque()
        self.min_distance = min_distance
        self.max_distance = max_distance
        self.is_last = False

    def set_route(self, global_plan):
        self.route.clear()
        for pos in global_plan:
            # pos는 {'x': value, 'y': value, 'z': value} 형식으로 되어 있다고 가정
            pos = np.array([pos['x'], pos['y'], pos['z']])
            self.route.append((pos, None))
        #rospy.loginfo(f"Route set with {len(self.route)} waypoints.")
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
    
def load_global_plan(file_path):
    if not os.path.exists(file_path):
        rospy.logerr(f"File not found: {file_path}")
        return []

    global_plan = []
    
    with open(file_path, 'r') as f:
        for line in f:
            # 각 라인을 공백으로 구분하여 데이터를 처리
            parts = line.split()
            if len(parts) >= 4:
                # ID는 무시하고, x, y, z 값을 가져옴
                x = float(parts[1])
                y = float(parts[2])
                z = float(parts[3])
                global_plan.append({'x': x, 'y': y, 'z': z})

    #rospy.loginfo(f"Loaded global plan from {file_path} with {len(global_plan)} waypoints.")
    return global_plan


# GPS 콜백 함수
def gps_callback(data, agent):
    # ROS 토픽에서 차량의 위치와 heading 값 가져오기
    gps_position = np.array([data.position.x, data.position.y, data.position.z])  # 차량 위치 (x, y, z)
    heading = data.heading  # 차량의 heading (예: 138.7890625)

    # 경로 업데이트
    next_wp = agent.route_planner.run_step(gps_position)  # next_wp는 경로에서 다음 웨이포인트

    if not next_wp or len(next_wp) == 0:
        rospy.logwarn("No waypoint returned by route planner.")
        return

    # next_wp는 [(array([x, y, z]), None)] 형태로 반환되므로, 첫 번째 요소만 사용
    next_wp = next_wp[0][0]  # numpy 배열 (x, y, z)

    # heading 값을 theta로 사용하여 각도 계산
    theta = heading + np.pi / 2  # 북쪽을 기준으로 90도 회전 (로컬 좌표계에서 사용)

    # 회전 행렬
    R = np.array([
        [np.cos(theta), -np.sin(theta)],
        [np.sin(theta), np.cos(theta)]
    ])

    # pos는 3D 좌표(x, y, z)이므로, x, y만 사용하여 local_command_point 계산
    local_command_point = np.array([next_wp[0] - gps_position[0], next_wp[1] - gps_position[1]])

    # 회전 적용
    local_command_point = R.T.dot(local_command_point)

    # y축 반전
    local_command_point = local_command_point * [1.0, -1.0]

    # torch로 변환하여 반환 (CUDA로 이동)
    target_point = torch.from_numpy(local_command_point).to('cuda', dtype=torch.float32).unsqueeze(0)

    # target_point를 agent의 run_step에 전달할 수 있도록 업데이트
    agent.update_target_point(target_point)
class MVPAgent:
    def __init__(self, checkpoint_path, global_plan_file):
        # Model Loading
        trainer = TrainingModule.load_from_checkpoint(checkpoint_path, strict=False)
        trainer.eval()
        trainer.cuda()
        self.model = trainer.model
        self.cfg = self.model.cfg

        # Route Planner
        self.route_planner = RoutePlanner(min_distance=1.0, max_distance=10.0)

        # Load global plan from file
        global_plan = load_global_plan(global_plan_file)
        self.route_planner.set_route(global_plan)

        # PID Controllers
        self.turn_controller = PIDController(K_P=1.25, K_I=0.75, K_D=0.3, n=40)
        self.speed_controller = PIDController(K_P=5.0, K_I=0.5, K_D=1.0, n=40)
        self.last_steer = 0

        # ROS Setup
        rospy.init_node('mvp_agent_node', anonymous=True)
        self.cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=10)

        # Image Normalization
        self.normalise_image = torchvision.transforms.Compose([
            torchvision.transforms.ToTensor(),
            torchvision.transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ])

        # Store target points
        self.target_points = None

    def update_target_point(self, target_point):
        """ 업데이트된 target_point를 저장 """
        self.target_points = target_point

    def control_pid(self, waypoints, velocity):
        waypoints = waypoints[0].detach().cpu().numpy()
        speed = velocity[0].cpu().numpy()
        aim = (waypoints[1] + waypoints[0]) / 2.0
        angle = np.degrees(np.pi / 2 - np.arctan2(aim[1], aim[0])) / 90

        steer = self.turn_controller.step(angle)
        steer = np.clip(steer, -1.0, 1.0)

        desired_speed = np.linalg.norm(waypoints[0] - waypoints[1]) * 2.0
        brake = (speed / desired_speed) > 1.2

        delta = np.clip(desired_speed - speed, 0.0, 0.25)
        accel = self.speed_controller.step(delta)
        accel = np.clip(accel, 0.0, 0.75)
        accel = accel if not brake else 0.0

        return steer, accel, brake

    def run_step(self, images, intrinsics, extrinsics, future_egomotion, trajs, gt_velocity, command, target_points):
        # cam_front에 사용할 더미 텐서 (형태: batch_size x 64 x 60 x 28)
        dummy_cam_front = torch.zeros((images.size(0), 64, 60, 28)).cuda()

        # hd_map에 사용할 더미 텐서 (형태: batch_size x 2 x 200 x 200)
        dummy_hd_map = torch.zeros((images.size(0), 2, 200, 200)).cuda()

        # trajs에 사용할 더미 텐서 (형태: batch_size x N x n_future x 3)
        dummy_trajs = torch.zeros((images.size(0), 30, 20, 3)).cuda()  # N=30, n_future=20

        # gt_trajs에 사용할 더미 텐서 (형태: batch_size x n_future x 3)
        dummy_gt_trajs = torch.zeros((images.size(0), 20, 3)).cuda()  # n_future=20

        # semantic_pred에 사용할 더미 텐서 (형태: batch_size x n_future x 200 x 200)
        dummy_semantic_pred = torch.zeros((images.size(0), 20, 200, 200)).cuda()

        # trajs의 최대 범위에 맞춰 cost_volume H, W 설정
        max_x = int(torch.max(dummy_trajs[..., 0]).item()) + 1  # 최대 X값
        max_y = int(torch.max(dummy_trajs[..., 1]).item()) + 1  # 최대 Y값

        H, W = max(200, max_y), max(200, max_x)  # 최소 200, 최대는 trajs 범위에 맞춤

        # cost_volume에 사용할 더미 텐서 (형태: batch_size x n_future x H x W)
        dummy_cost_volume = torch.zeros((images.size(0), 20, H, W), device=images.device)

        # commands: 더미 리스트 생성 (형태: batch_size)
        dummy_commands = [1] * images.size(0)

        # target_points에 사용할 더미 텐서 (형태: batch_size x 2)
        target_points = self.target_points if self.target_points is not None else torch.zeros((images.size(0), 2)).cuda()
                
        _, final_traj = self.model.planning(
            cam_front=dummy_cam_front,  # 64채널 더미 텐서
            trajs=dummy_trajs,          # 더미 trajs 텐서
            gt_trajs=None,
            cost_volume=dummy_cost_volume,
            semantic_pred=dummy_semantic_pred,
            hd_map=dummy_hd_map,        # 더미 텐서 사용
            commands=dummy_commands,
            target_points=target_points  # target_points 전달
        )
        rospy.loginfo(f"Final Trajectory: {final_traj}")  # final_traj 값 출력
        rospy.loginfo(f"Ground Truth Velocity: {gt_velocity}")  # gt_velocity 값 출력
        steer, accel, brake = self.control_pid(final_traj, gt_velocity)
        self.last_steer = steer

        # 안전 처리
        if brake < 0.05: brake = 0.0
        if accel > brake: brake = 0.0
        rospy.loginfo(f"Control Command: accel={float(accel)}, brake={float(brake)}, steering={float(steer)}")

        # ROS 메시지 생성 및 발행
        ctrl_msg = CtrlCmd()
        ctrl_msg.longlCmdType = 1
        ctrl_msg.accel = float(accel)
        ctrl_msg.brake = float(brake)
        ctrl_msg.steering = float(steer)
        self.cmd_pub.publish(ctrl_msg)

# 차량 상태 토픽 구독
def main():
    checkpoint_path = "/home/viplab/hd/hd_ws/src/morai_stp3/src/morai.ckpt"
    global_plan_file = "/home/viplab/hd/hd_ws/hmg_mission11_global_path.txt"
    agent = MVPAgent(checkpoint_path, global_plan_file)

    batch_size = 1
    seq_length = 2
    dummy_image = torch.zeros((batch_size, seq_length, 1, 3, 224, 224)).cuda()
    dummy_intrinsics = torch.eye(3).unsqueeze(0).unsqueeze(0).unsqueeze(0).repeat(batch_size, seq_length, 4, 1, 1).cuda()
    dummy_extrinsics = torch.eye(4).unsqueeze(0).unsqueeze(0).unsqueeze(0).repeat(batch_size, seq_length, 4, 1, 1).cuda()
    dummy_future_egomotion = torch.zeros((batch_size, seq_length, 3, 6)).cuda()
    dummy_trajs = torch.zeros((batch_size, 10, 2)).cuda()
    dummy_velocity = torch.tensor([[0.0]]).cuda()
    dummy_command = torch.tensor([1]).cuda()

    rospy.Subscriber('/Ego_topic', EgoVehicleStatus, gps_callback, agent)

    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        try:
            rospy.loginfo("Running Agent Step...")
            agent.run_step(
                dummy_image, dummy_intrinsics, dummy_extrinsics, dummy_future_egomotion,
                dummy_trajs, dummy_velocity, dummy_command, agent.target_points
            )
            rate.sleep()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down MVPAgent.")
            break

if __name__ == "__main__":
    main()

