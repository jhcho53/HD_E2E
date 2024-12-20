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
                x = float(parts[1])
                y = float(parts[2])
                z = float(parts[3])
                global_plan.append({'x': x, 'y': y, 'z': z})

    #rospy.loginfo(f"Loaded global plan from {file_path} with {len(global_plan)} waypoints.")
    return global_plan

    '''
    [
    {'x': 12.34, 'y': 56.78, 'z': 9.01},
    {'x': 23.45, 'y': 67.89, 'z': 10.11},   Example of Global plan
    {'x': 34.56, 'y': 78.90, 'z': 11.12}
    ]
    '''

class MVPAgent:
    def __init__(self):
        
        # Route Planner
        self.route_planner = RoutePlanner(min_distance=1.0, max_distance=10.0)

        # 카메라 이미지 저장용 딕셔너리
        self.rgb_images = {
            'rgb': None,
            'rgb_left': None,
            'rgb_right': None,
            'rgb_rear': None
        }

        # 입력 버퍼: Torch 텐서 저장
        self.input_buffer = {
            'rgb': deque(maxlen=4),
            'rgb_left': deque(maxlen=4),
            'rgb_right': deque(maxlen=4),
            'rgb_rear': deque(maxlen=4),
            'gps': deque(maxlen=4),
            'thetas': deque(maxlen=4)
        }

        # 초기화 플래그 및 스텝
        self.initialized = False
        self.step = 0

        checkpoint_path = "/home/viplab/hd/hd_ws/src/morai_stp3/src/morai.ckpt"

        # 경로 설정을 위한 global_plan 로드
        global_plan_file = "/home/viplab/hd/hd_ws/hmg_mission11_global_path.txt"  # 실제 경로 파일 경로로 변경
        global_plan = load_global_plan(global_plan_file)
        if global_plan:
            self.route_planner.set_route(global_plan)
        else:
            rospy.logwarn("Global plan could not be loaded.")
        
        # Model Loading
        trainer = TrainingModule.load_from_checkpoint(checkpoint_path, strict=False)
        trainer.eval()
        trainer.cuda()
        self.model = trainer.model
        self.cfg = self.model.cfg

        # PID Controllers
        self.turn_controller = PIDController(K_P=1.25, K_I=0.75, K_D=0.3, n=40)
        self.speed_controller = PIDController(K_P=5.0, K_I=0.5, K_D=1.0, n=40)
        self.last_steer = 0

        # ROS Setup
        rospy.init_node('mvp_agent_node', anonymous=True)
        self.cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=10)
        self.bridge = CvBridge()

        # 카메라 토픽 구독
        self.cm1 = rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.camera_callback, 'rgb')
        self.cm2 = rospy.Subscriber('/image_jpeg/compressed2', CompressedImage, self.camera_callback, 'rgb_left')
        self.cm3 = rospy.Subscriber('/image_jpeg/compressed3', CompressedImage, self.camera_callback, 'rgb_right')
        self.cm4 = rospy.Subscriber('/image_jpeg/compressed4', CompressedImage, self.camera_callback, 'rgb_rear')
        
        # Compass와 GPS 데이터 토픽 구독
        self.ego_sub = rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.ego_callback)

        # 이미지 정규화
        self.normalise_image = torchvision.transforms.Compose([
            torchvision.transforms.ToTensor(),
            torchvision.transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
        ])

        # Store target points
        self.target_points = None

    def camera_callback(self, msg, camera_name):
        try:
            # 압축된 이미지를 OpenCV에서 읽을 수 있도록 디코딩
            np_arr = np.frombuffer(msg.data, np.uint8)
            bgr_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # BGR을 RGB로 변환
            rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
            self.rgb_images[camera_name] = rgb_image

        except Exception as e:
            rospy.logerr(f"Error in camera_callback: {str(e)}")

    def update_target_point(self, target_point):
                """ 업데이트된 target_point를 저장 """
                self.target_points = target_point

    def ego_callback(self, msg):
        """
        /Ego_topic에서 position 데이터를 수신하고 입력 버퍼에 추가합니다.
        """
        try:
            gps_position = np.array([msg.position.x, msg.position.y, msg.position.z])
            ego_position = (msg.position.x, msg.position.y)  # x와 y 좌표 추출
            heading = msg.heading

            self.input_buffer['gps'].append(ego_position)  # GPS 버퍼에 추가
            self.input_buffer['thetas'].append(heading)
            #rospy.loginfo(f"GPS Position: {ego_position}, Heading: {heading}")

            # 경로 업데이트
            next_wp = self.route_planner.run_step(gps_position)

            if not next_wp or len(next_wp) == 0:
                rospy.logwarn("No waypoint returned by route planner.")
                return

            # next_wp는 [(array([x, y, z]), None)] 형태로 반환되므로, 첫 번째 요소만 사용
            # None => High Command

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

            target_point = torch.from_numpy(local_command_point).to('cuda', dtype=torch.float32).unsqueeze(0)

            # target_point를 agent의 run_step에 전달할 수 있도록 업데이트
            self.update_target_point(target_point)
            #rospy.loginfo(f"/Ego_topic position data received: {target_point}")            
            #rospy.loginfo(f"/Ego_topic position data received: {ego_position}")
            #rospy.loginfo(f"/Ego_topic position data received: {heading}")
        except Exception as e:
            rospy.logerr(f"Error in ego_callback: {str(e)}")

    def get_future_egomotion(self, seq_x, seq_y, seq_theta):
        future_egomotions = []

        def convert_to_matrix_numpy(x, y, theta):
            matrix = np.zeros((4,4), dtype=np.float32)
            matrix[:2, :2] = np.array([
                [np.cos(theta), -np.sin(theta)],
                [np.sin(theta), np.cos(theta)]
            ])
            matrix[2,2] = 1
            matrix[0,3] = x
            matrix[1,3] = y
            matrix[3,3] = 1
            return matrix

        for i in range(len(seq_x)-1):
            egopose_t0 = convert_to_matrix_numpy(seq_x[i], seq_y[i], seq_theta[i])
            egopose_t1 = convert_to_matrix_numpy(seq_x[i+1], seq_y[i+1], seq_theta[i+1])

            future_egomotion = invert_matrix_egopose_numpy(egopose_t1).dot(egopose_t0)
            future_egomotion[3, :3] = 0.0
            future_egomotion[3, 3] = 1.0

            future_egomotion = torch.Tensor(future_egomotion).float()
            future_egomotion = mat2pose_vec(future_egomotion)
            future_egomotions.append(future_egomotion.unsqueeze(0))

        return torch.cat(future_egomotions, dim=0)
    
    def tick(self):
        """
        현재까지 수신된 RGB 이미지들을 반환.
        """
        return self.rgb_images

    def update_target_point(self, target_point):
        """ 업데이트된 target_point를 저장 """
        # rospy.loginfo(f"target_point shape: {target_point.shape}")
        # torch.Size([1, 2])
        self.target_points = target_point

    def get_cam_para_from_json(self, camera_data):
        def get_cam_to_ego(pos, yaw):
            yaw_rad = np.radians(float(yaw))
            rotation = Quaternion(scalar=np.cos(yaw_rad / 2), vector=[0, 0, np.sin(yaw_rad / 2)])
            translation = np.array([[float(pos['x'])], [float(pos['y'])], [float(pos['z'])]])
            cam_to_ego = np.vstack([
                np.hstack((rotation.rotation_matrix, translation)),
                np.array([0, 0, 0, 1])
            ])
            return torch.from_numpy(cam_to_ego).float().unsqueeze(0)

        extrinsic_list = []
        intrinsic_list = []

        for cam in camera_data['cameraList']:
            pos, rot, fov = cam['pos'], cam['rot'], cam['cameraFOV']
            res_width, res_height = cam['cameraResWidth'], cam['cameraResHeight']

            extrinsic_list.append(get_cam_to_ego(pos, rot['yaw']))
            f = res_width / (2 * np.tan(np.radians(fov / 2)))
            intrinsic = torch.Tensor([
                [f, 0, res_width / 2],
                [0, f, res_height / 2],
                [0, 0, 1]
            ]).unsqueeze(0)
            intrinsic_list.append(intrinsic)

        extrinsic = torch.cat(extrinsic_list, dim=0)
        intrinsic = torch.cat(intrinsic_list, dim=0)
        return extrinsic, intrinsic
    
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

    def _init(self):
        """
        초기화 함수.
        """
        rospy.loginfo("Initializing CameraSubscriber...")
        self.initialized = True

    def run_step(self, intrinsics, extrinsics, trajs, gt_velocity, command, target_points):
        """
        RGB 이미지들을 Torch 텐서로 변환하고 입력 버퍼에 저장.
        버퍼를 병합해 최종 (1, 3, 4, 256, 256) 형태.
        """
        if not self.initialized:
            self._init()

        # tick()에서 이미지를 가져오기
        tick_data = self.tick()
        images = []

        for key in ['rgb', 'rgb_left', 'rgb_right', 'rgb_rear']:
            image = tick_data.get(key)
            if image is not None:
                try:
                    # scale_and_crop_image 적용 및 차원 변환 (H, W, C) -> (C, H, W)
                    cropped_image = scale_and_crop_image(Image.fromarray(image), scale=1, crop=256)
                    chw_image = np.transpose(np.array(cropped_image), (2, 0, 1))  # (C, H, W)

                    # Torch 텐서 변환 및 정규화
                    tensor_image = self.normalise_image(chw_image).to('cuda', dtype=torch.float32)  # (3, 256, 256)

                    # 버퍼에 최신 이미지 추가 (최대 길이 3)
                    self.input_buffer[key].append(tensor_image)
                    if len(self.input_buffer[key]) > 3:
                        self.input_buffer[key].popleft()

                    # 버퍼에서 최신 3개 이미지를 가져와 시퀀스로 추가
                    sequence_images = torch.stack(list(self.input_buffer[key]), dim=0)  # (3, 3, 256, 256)
                    images.append(sequence_images)

                except Exception as e:
                    rospy.logerr(f"Error processing {key}: {e}")
            else:
                rospy.logwarn(f"No image data for {key}. Skipping...")
                # 데이터가 없을 경우 빈 시퀀스 삽입
                fallback = torch.zeros((3, 3, 256, 256), dtype=torch.float32).to('cuda')
                images.append(fallback)

        # 모든 시퀀스 이미지를 병합하여 최종 형태 생성: (1, 3, 4, 256, 256)
        if len(images) == 4:  # 모든 카메라 뷰가 준비된 경우
            #torch.Size([1, 3, 4, 3, 256, 256]
            stacked_images = torch.stack(images, dim=1).unsqueeze(0)  # (1, 3, 4, 256, 256)
            #rospy.loginfo(f"Final stacked images shape: {stacked_images.shape}")
        else:
            rospy.logwarn("Incomplete image data. Skipping processing.")
            stacked_images = torch.zeros((1, 3, 4, 256, 256), dtype=torch.float32).to('cuda')  # Fallback

        self.step += 1
        #rospy.loginfo(f"Step: {self.step}, images processed and added to input buffer.")

        #torch.Size([1, 3, 4, 4, 4])
        extrinsics = extrinsics.unsqueeze(0).expand(3,4,4,4).unsqueeze(0).to('cuda', dtype=torch.float32)
        #torch.Size([1, 3, 4, 4, 4])
        intrinsics = intrinsics.unsqueeze(0).expand(3,4,3,3).unsqueeze(0).to('cuda', dtype=torch.float32)
        rospy.loginfo(f"intrinsic tensor shape: {intrinsics.shape}")
        # 미래 egomotion 계산을 위한 데이터가 충분한지 확인
        if len(self.input_buffer['gps']) >= 2 and len(self.input_buffer['thetas']) >= 2:
            future_egomotion = self.get_future_egomotion(
                seq_x=[p[0] for p in self.input_buffer['gps']],
                seq_y=[p[1] for p in self.input_buffer['gps']],
                seq_theta=[p for p in self.input_buffer['thetas']]
            ).to('cuda', dtype=torch.float32).unsqueeze(0)
        else:
            rospy.logwarn("Not enough data for future egomotion computation.")
            future_egomotion = torch.zeros(1, 3)  # 기본값 또는 대체값

        output = self.model(
            stacked_images, intrinsics, extrinsics, future_egomotion,
        )
        torch.cuda.empty_cache()
        n_present = self.model.receptive_field
        seg_prediction = output['segmentation'].detach()
        seg_prediction = torch.argmax(seg_prediction, dim=2, keepdim=True)
        pedestrian_prediction = output['pedestrian'].detach()
        pedestrian_prediction = torch.argmax(pedestrian_prediction, dim=2, keepdim=True)
        occupancy = torch.logical_or(seg_prediction, pedestrian_prediction)

        _, final_traj = self.model.planning(
            cam_front=output['cam_front'].detach(),
            trajs=trajs[:, :, 1:],
            gt_trajs=None,
            cost_volume=output['costvolume'][:, n_present:].detach(),
            semantic_pred=occupancy[:, n_present:].squeeze(2),
            hd_map=output['hdmap'].detach(),
            commands=command,
            target_points=target_points
        )

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

def main():
    
    #global_plan_file = "/home/viplab/hd/hd_ws/hmg_mission11_global_path.txt"
    json_file_path = "/home/viplab/hd/hd_ws/camera_config.json"
    os.environ['PYTORCH_CUDA_ALLOC_CONF'] = 'max_split_size_mb:32'
    with open(json_file_path, "r") as file:
        camera_data = json.load(file)
    agent = MVPAgent()

    extrinsic, intrinsic = agent.get_cam_para_from_json(camera_data)
    

    # 더미 데이터 준비
    batch_size = 1
    # seq_length = 2
    dummy_trajs = torch.zeros((batch_size, 10, 2)).cuda()
    dummy_velocity = torch.tensor([[5.0]]).cuda()
    dummy_command = torch.tensor([1]).cuda()


    # ROS 루프 실행
    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        try:
            rospy.loginfo("Running Agent Step...")

            # Run step without pre-fetching images
            agent.run_step(
                intrinsics=intrinsic,
                extrinsics=extrinsic,
                trajs=dummy_trajs,
                gt_velocity=dummy_velocity,
                command=dummy_command,
                target_points=agent.target_points
            )

            rate.sleep()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down MVPAgent.")
            break

if __name__ == "__main__":
    os.environ['PYTORCH_CUDA_ALLOC_CONF'] = 'max_split_size_mb:32'

    main()








    # dummy_image = torch.zeros((batch_size, seq_length, 1, 3, 224, 224)).cuda()
    # dummy_intrinsics = torch.eye(3).unsqueeze(0).unsqueeze(0).unsqueeze(0).repeat(batch_size, seq_length, 1, 1, 1).cuda()
    # dummpy_intrinsics = [batch, seq_length, 4,3,3]
    # dummy_extrinsics = torch.eye(4).unsqueeze(0).unsqueeze(0).unsqueeze(0).repeat(batch_size, seq_length, 1, 1, 1).cuda()
    # dummpy_extrinsics = [batch, seq_length, 4,4,4]
    # dummy_future_egomotion = torch.zeros((batch_size, seq_length, 4, 4)).cuda()
    # dummy_trajs = torch.zeros((batch_size, 10, 2)).cuda()
    # dummy_velocity = torch.tensor([[5.0]]).cuda()
    # dummy_command = torch.tensor([1]).cuda()
    # dummy_target_points = torch.zeros((batch_size, 2)).cuda()






        #######################################################
        # # cam_front에 사용할 더미 텐서 (형태: batch_size x 64 x 60 x 28)
        # dummy_cam_front = torch.zeros((images.size(0), 64, 60, 28)).cuda()

        # # hd_map에 사용할 더미 텐서 (형태: batch_size x 2 x 200 x 200)
        # dummy_hd_map = torch.zeros((images.size(0), 2, 200, 200)).cuda()

        # # trajs에 사용할 더미 텐서 (형태: batch_size x N x n_future x 3)
        # dummy_trajs = torch.zeros((images.size(0), 30, 20, 3)).cuda()  # N=30, n_future=20

        # # gt_trajs에 사용할 더미 텐서 (형태: batch_size x n_future x 3)
        # dummy_gt_trajs = torch.zeros((images.size(0), 20, 3)).cuda()  # n_future=20

        # # semantic_pred에 사용할 더미 텐서 (형태: batch_size x n_future x 200 x 200)
        # dummy_semantic_pred = torch.zeros((images.size(0), 20, 200, 200)).cuda()

        # # trajs의 최대 범위에 맞춰 cost_volume H, W 설정
        # max_x = int(torch.max(dummy_trajs[..., 0]).item()) + 1  # 최대 X값
        # max_y = int(torch.max(dummy_trajs[..., 1]).item()) + 1  # 최대 Y값

        # H, W = max(200, max_y), max(200, max_x)  # 최소 200, 최대는 trajs 범위에 맞춤

        # # cost_volume에 사용할 더미 텐서 (형태: batch_size x n_future x H x W)
        # dummy_cost_volume = torch.zeros((images.size(0), 20, H, W), device=images.device)

        # # commands: 더미 리스트 생성 (형태: batch_size)
        # dummy_commands = [1] * images.size(0)

        # # target_points에 사용할 더미 텐서 (형태: batch_size x 2)
        # target_points = self.target_points if self.target_points is not None else torch.zeros((images.size(0), 2)).cuda()

        # _, final_traj = self.model.planning(
        #     cam_front=dummy_cam_front,  # 64채널 더미 텐서
        #     trajs=dummy_trajs,          # 더미 trajs 텐서
        #     gt_trajs=None,
        #     cost_volume=dummy_cost_volume,
        #     semantic_pred=dummy_semantic_pred,
        #     hd_map=dummy_hd_map,        # 더미 텐서 사용
        #     commands=dummy_commands,
        #     target_points=target_points  # target_points 전달
        # )

        #######################################################