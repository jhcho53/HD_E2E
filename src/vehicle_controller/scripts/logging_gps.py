#!/usr/bin/env python3

import rospy
from morai_msgs.msg import GPSMessage  # GPS 메시지 타입 import

def gps_callback(data):
    # latitude, longitude, altitude 데이터를 로깅합니다.
    rospy.loginfo(f"Latitude: {data.latitude}, Longitude: {data.longitude}, Altitude: {data.altitude}")
    # 파일에 기록
    with open("/home/viplab/hd/hd_ws/src/global_path_demo/src/morai_gps_log.txt", "a") as log_file:
        log_file.write(f"{rospy.get_time()}, {data.latitude}, {data.longitude}, {data.altitude}\n")

def gps_logger():
    # ROS 노드 초기화
    rospy.init_node('morai_gps_logger', anonymous=True)
    # /gps 토픽 구독
    rospy.Subscriber("/gps", GPSMessage, gps_callback)
    # 종료되지 않도록 유지
    rospy.spin()

if __name__ == '__main__':
    try:
        gps_logger()
    except rospy.ROSInterruptException:
        pass
