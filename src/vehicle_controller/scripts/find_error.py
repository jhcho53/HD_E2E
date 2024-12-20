import numpy as np

def check_global_path(file_path):
    """
    글로벌 경로 파일의 문제 있는 줄을 검사하여 출력.
    Args:
        file_path (str): 경로 파일의 경로
    """
    with open(file_path, 'r') as file:
        for line_number, line in enumerate(file, start=1):
            parts = line.strip().split()  # 공백 기준으로 분리
            if len(parts) < 4:
                print(f"Line {line_number}: Malformed line - {line.strip()}")
                continue
            try:
                # 각 부분을 float으로 변환 시도 (ID 제외)
                float(parts[1])  # easting
                float(parts[2])  # northing
                float(parts[3])  # up
            except ValueError as ex:
                print(f"Line {line_number}: Error parsing line - {line.strip()} ({ex})")


def load_global_path(file_path):
    """
    글로벌 경로 파일을 읽어와 경로 리스트를 반환.
    Args:
        file_path (str): 경로 파일의 경로
    Returns:
        list: [(easting, northing, up), ...] 형태의 경로점 리스트
    """
    waypoints = []
    with open(file_path, 'r') as file:
        for line_number, line in enumerate(file, start=1):
            parts = line.strip().split()  # 공백 기준으로 분리
            if len(parts) < 4:
                print(f"Warning: Skipping malformed line {line_number}: {line.strip()}")
                continue
            try:
                e = float(parts[1])  # easting
                n = float(parts[2])  # northing
                u = float(parts[3])  # up
                waypoints.append((e, n, u))
            except ValueError as ex:
                print(f"Error parsing line {line_number}: {line.strip()} - {ex}")
    return waypoints


if __name__ == '__main__':
    # 경로 파일 경로
    global_path_file = "/home/vip-lab/morai_ws/src/vehicle_controller/scripts/global_path.txt"

    # 1. 문제 있는 줄 검사
    print("Checking global path for issues...")
    check_global_path(global_path_file)

    # 2. 유효한 경로 로드
    print("\nLoading global path...")
    global_path = load_global_path(global_path_file)

    # 3. 로드된 경로 출력
    print("\nLoaded global path waypoints:")
    for waypoint in global_path:
        print(waypoint)
