#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
import cv2
import numpy as np
import torch
from PIL import Image
from torchvision import transforms
from collections import deque
from stp3.datas.CarlaData import scale_and_crop_image  # 함수 임포트
from morai_msgs.msg import EgoVehicleStatus
from stp3.utils.geometry import (
    update_intrinsics,
    mat2pose_vec,
    invert_matrix_egopose_numpy,
)
import os

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

class CameraSubscriber:
    def __init__(self):
        rospy.init_node('camera_subscriber', anonymous=True)
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
            'thetas': deque(maxlen=4),
            'gps': deque(maxlen=4)
        }

        # 초기화 플래그 및 스텝
        self.initialized = False
        self.step = 0

        # 경로 설정을 위한 global_plan 로드
        global_plan_file = "/home/viplab/hd/hd_ws/hmg_mission11_global_path.txt"  # 실제 경로 파일 경로로 변경
        global_plan = load_global_plan(global_plan_file)
        if global_plan:
            self.route_planner.set_route(global_plan)
        else:
            rospy.logwarn("Global plan could not be loaded.")

        # 카메라 토픽 구독
        self.cm1 = rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.camera_callback, 'rgb')
        self.cm2 = rospy.Subscriber('/image_jpeg/compressed2', CompressedImage, self.camera_callback, 'rgb_left')
        self.cm3 = rospy.Subscriber('/image_jpeg/compressed3', CompressedImage, self.camera_callback, 'rgb_right')
        self.cm4 = rospy.Subscriber('/image_jpeg/compressed4', CompressedImage, self.camera_callback, 'rgb_rear')

        # Compass와 GPS 데이터 토픽 구독
        self.ego_sub = rospy.Subscriber('/Ego_topic', EgoVehicleStatus, self.ego_callback)


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
            rospy.loginfo(f"GPS Position: {ego_position}, Heading: {heading}")

            # 경로 업데이트
            next_wp = self.route_planner.run_step(gps_position)

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
            self.update_target_point(target_point)
            rospy.loginfo(f"/Ego_topic position data received: {target_point}")

            
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

    @torch.no_grad()
    def run_step(self, input_data, timestamp):
        """
        RGB 이미지들을 Torch 텐서로 변환하고 입력 버퍼에 저장.
        버퍼를 병합해 최종 (1, 3, 4, 256, 256) 형태로 만듭니다.
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
                    tensor_image = self.normalise_image(chw_image).unsqueeze(0).to('cuda', dtype=torch.float32)  # (1, 3, 256, 256)

                    # 버퍼에 최신 이미지 추가
                    self.input_buffer[key].append(tensor_image)
                    if len(self.input_buffer[key]) > 1:
                        self.input_buffer[key].popleft()

                    # 버퍼에서 가장 최신 이미지를 추가
                    images.append(self.input_buffer[key][-1])

                except Exception as e:
                    rospy.logerr(f"Error processing {key}: {e}")
            else:
                rospy.logwarn(f"No image data for {key}. Skipping...")

        # 이미지 병합
        if len(images) == 4:  # 모든 카메라 이미지가 준비된 경우에만 병합
            try:
                # Concatenate 4 images along a new dimension (dim=0)
                stacked_images = torch.stack(images, dim=2)  # shape: (1, 3, 4, 256, 256)
                rospy.loginfo(f"Final images tensor shape: {stacked_images.shape}")
            except RuntimeError as e:
                rospy.logerr(f"Error during final torch.stack: {e}")
        else:
            rospy.logwarn("Not all images are available for processing.")
        
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
    def normalise_image(self, image):
        """
        이미지를 정규화하고 Torch 텐서로 변환.
        """
        image = torch.from_numpy(image).float() / 255.0
        image = (image - 0.5) / 0.5  # [-1, 1]로 정규화
        return image

    def _init(self):
        """
        초기화 함수.
        """
        rospy.loginfo("Initializing CameraSubscriber...")
        self.initialized = True

if __name__ == '__main__':
    try:
        cam_sub = CameraSubscriber()
        while not rospy.is_shutdown():
            cam_sub.run_step(input_data=None, timestamp=None)
    except rospy.ROSInterruptException:
        pass






# import torch
# import numpy as np
# from pyquaternion import Quaternion
# import json

# def get_cam_para_from_json(camera_data):
#     def get_cam_to_ego(pos, yaw):
#         yaw_rad = np.radians(float(yaw))
#         rotation = Quaternion(scalar=np.cos(yaw_rad / 2), vector=[0, 0, np.sin(yaw_rad / 2)])
#         translation = np.array([[float(pos['x'])], [float(pos['y'])], [float(pos['z'])]])
#         cam_to_ego = np.vstack([
#             np.hstack((rotation.rotation_matrix, translation)),
#             np.array([0, 0, 0, 1])
#         ])
#         return torch.from_numpy(cam_to_ego).float().unsqueeze(0)

#     extrinsic_list = []
#     intrinsic_list = []

#     for cam in camera_data['cameraList']:
#         pos, rot, fov = cam['pos'], cam['rot'], cam['cameraFOV']
#         res_width, res_height = cam['cameraResWidth'], cam['cameraResHeight']

#         extrinsic_list.append(get_cam_to_ego(pos, rot['yaw']))
#         f = res_width / (2 * np.tan(np.radians(fov / 2)))
#         intrinsic = torch.Tensor([
#             [f, 0, res_width / 2],
#             [0, f, res_height / 2],
#             [0, 0, 1]
#         ]).unsqueeze(0)
#         intrinsic_list.append(intrinsic)

#     extrinsic = torch.cat(extrinsic_list, dim=0)
#     intrinsic = torch.cat(intrinsic_list, dim=0)
#     return extrinsic, intrinsic

# if __name__ == "__main__":
#     json_file_path = "/home/viplab/hd/hd_ws/camera_config.json"
#     with open(json_file_path, "r") as file:
#         camera_data = json.load(file)
    
#     extrinsic, intrinsic = get_cam_para_from_json(camera_data)
#     print("Extrinsic Matrix:")
#     print(extrinsic)
#     print("\nIntrinsic Matrix:")
#     print(intrinsic)
#     print("Extrinsic Tensor Shape:", extrinsic.shape)
#     print("Intrinsic Tensor Shape:", intrinsic.shape)


# Extrinsic Tensor Shape: torch.Size([4, 4, 4])
# Intrinsic Tensor Shape: torch.Size([4, 3, 3])
