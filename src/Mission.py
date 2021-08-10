import os
import sys
import warnings

# 차선(왼쪽: 1, 오른쪽: 2), 색
mode1 = [1, ['y', 'w']]  # 왼쪽 + (흰, 노란) 차선 검출
mode2 = [2, ['w']]  # 오른쪽 + 흰 차선 검출
mode3 = [1, ['b']]  # 왼쪽 + 파란 차선 검출
mode4 = [1, ['yy', 'w']] # 파킹 시 왼쪽 + 노란 차선 검출
mode5 = [1, ['y']]
mode6 = [1, ['w']]

sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

from abc import abstractclassmethod, ABCMeta
from src.Database import Database
from Path import Path
from Control.Control import Control
import threading
import time
import cv2
import numpy as np
from Path.polygonparking import polygonparking
from Path.lidarparking import lidarparking

warnings.simplefilter("always")


class Mission(metaclass=ABCMeta):
    def __init__(self, db: Database, control: Control, path: Path):
        self.db = db
        self.control = control
        self.path = path
        self.key = None

    @abstractclassmethod
    def main(self):  # 미션 수행 함수 구현해야함.
        '''
        while not self.mission_end():
            (params) = self.line.some_func_for_specific_mission()
            self.control.some_func_for_specific_mission(*params)
            time.sleep(0.1)
        '''

    def __str__(self):
        if self.key is None:
            return "None"
        else:
            return self.key + " Mission"


# 현재미션과 다음미션을 잇는 클래스
class MissionManager:
    def __init__(self, db: Database = None):
        self.missions = dict()
        self.db = db
        self.mission_keys = list()
        self.mission_idx = None
        self.current_mission_key = None
        self.manager_thread = threading.Thread(target=self.main)

        self.mission_manager = None

    def add_mission(self, key, mission: Mission):
        if key not in self.mission_keys:
            warnings.warn("The new key %s is not registered.\
                 Therefore, NO mission will be added." % key)
        else:
            mission.key = key
            self.missions[key] = mission

    def main(self):
        while not self.db.flag.system_stop:
            current_mission = self.missions[self.current_mission_key]  # 미션 번호를 수신함
            current_mission.main()  # 미션 탈출조건이 만족될 때까지 해당 미션 수행
            if not self.db.flag.system_stop:
                # 정상적으로 미션이 끝났을 때만 다음 미션으로 넘어감.
                self.next_mission()

    def next_mission(self):
        if self.mission_idx >= len(self.mission_keys):
            print("All missions are complete. System will stop.")
            self.db.flag.system_stop = True
        else:
            self.mission_idx += 1
            self.current_mission_key = self.mission_keys[self.mission_idx]

    def add_database(self, db):
        self.db = db

    def start(self):
        self.manager_thread.start()

    def join(self):
        self.manager_thread.join()


# Example for specific mission class
class SampleMission(Mission):
    def main(self):
        while not self.mission_end():
            time.sleep(0.1)
            print(self.mission_end())


class ParkingMission(Mission):
    def __init__(self, db, control, path):
        super().__init__(db, control, path)
        self.start_time = 0
        self.parking_start_time = 0
        self.brake_start_time = 0
        self.is_parking = 0
        self.parking_count = 0
        self.parking_sign = 0

        self.wall_tracking = False
        self.parking_start = False

    def main(self):
        start_time = time.time()

        while True:
            # print('buffer', self.parking_count)
            # 정지선 검출시 탈출
            if self.control.parking_mission_end:
                break

            # if lidarparking(self.db.lidar.data) and polygonparking(self.db.parking_cam.data):
            # if polygonparking(self.db.parking_cam.data):

            # lidarpakring.py에서 True를 반환하면 count를 증가시킴.
            if lidarparking(self.db.lidar.data):
                self.parking_count += 1
                print('empty')
            else:
                self.parking_count = 0
                print('parking counting reset')

            # 주차카운트가 연속해서 True가 뜨면 주차구역이라고 플래그 변경(버퍼)
            if self.parking_count >= 2:
                self.is_parking = True

            # 주차 매크로 시작
            if self.is_parking:
                print('start')
                if self.brake_start_time == 0:
                    self.brake_start_time = time.time()

                # 주차
                if time.time() - self.brake_start_time < 3:  # 정지
                    self.control.stop(brake=80)
                else:
                    if self.parking_start_time == 0:
                        self.parking_start_time = time.time()

                    self.control.parking_macro(self.parking_start_time)

            # Default
            else:
                # self.control.line_tracking(mode4, speed=90)
                self.control.simple_straight_forward(speed=90)
                # if 6 < time.time() - start_time < 8:
                #     self.control.stop(brake=20)

            time.sleep(0.1)


class StaticSmallObstacleMission(Mission):
    def __init__(self, db, control, path):
        super().__init__(db, control, path)
        # 현재 차선. 1차선 or 2차선
        self.current_line = 1
        self.mode = mode3

        # 사용하지 않을 수도 있음
        self.obstacle_start_time = 0
        self.obstacle_cnt = 0

        self.obs_detect = 0

        self.obstacle_location_left = False
        self.obstacle_location_right = False
        self.obs_detect = False
        self.start_range_error_time = 0
        self.running_time = 0
        self.start_range_error = 90 * 2
        self.obs_cnt = 0

    def main(self):
        while True:
            # 소형 장애물 위치 확인
            if not self.obs_detect and self.control.distance() < 200:
                self.obs_detect = True
                if self.control.distance(180, 360) < self.control.distance(0, 180):
                    self.obstacle_location_left = True
                    print("Obstacle is on the left.\n")
                else:
                    self.obstacle_location_right = True
                    print("Obstacle is on the right.\n")
                if self.start_range_error_time == 0 and self.running_time == 0:
                    self.start_range_error_time = time.time()
                    self.running_time = time.time()
                # self.control.control_code(0, 0, 0x00, 60)  # 확인용 Brake

            if self.obs_detect and self.obstacle_location_left:  # 장애물 : 왼 -> 오
                if time.time() - self.start_range_error_time > 1 and self.start_range_error > 60:
                    self.start_range_error -= 10 * 2  # 목표 : start_range = 300
                    self.start_range_error_time = time.time()
                if time.time() - self.running_time <= 6:  # 6초 동안 왼쪽 장애물 타고 가기
                    self.control.left_wall_tracking(start_range=360 - self.start_range_error, end_range=360)
                    print("Left obstacle tracking.\n")
                    # print("error : ", self.start_range_error, "\n")
                else:
                    self.control.line_tracking(mode1, speed=50)
                    print("Go straight little bit using line tracking.\n")

                    # 초기화
                    if self.control.distance() < 200:
                        self.obs_detect = False
                        self.obstacle_location_right = False
                        self.start_range_error_time = 0
                        self.running_time = 0
                        self.obs_cnt += 1
                    if self.obs_cnt == 1:
                        self.obs_cnt += 1

            elif self.obs_detect and self.obstacle_location_right:  # 장애물 : 오 -> 왼
                if time.time() - self.start_range_error_time > 1 and self.start_range_error > 60:
                    self.start_range_error -= 10 * 2  # 목표 : start_range = 60
                    self.start_range_error_time = time.time()
                if time.time() - self.running_time <= 6:  # 6초 동안 오른쪽 장애물 타고 가기
                    self.control.right_wall_tracking(start_range=0, end_range=0 + self.start_range_error)
                    print("Right obstacle tracking.\n")
                    # print("error : ", self.start_range_error, "\n")
                else:
                    self.control.line_tracking(mode1, speed=50)
                    print("Go straight little bit using line tracking.\n")

                    # 초기화
                    if self.control.distance() < 200:
                        self.obs_detect = False
                        self.obstacle_location_right = False
                        self.start_range_error_time = 0
                        self.running_time = 0
                        self.obs_cnt += 1
                    if self.obs_cnt == 1:
                        self.obs_cnt += 1

            # Default
            else:
                # self.control.control_code(60, 0, 0x00, 0)
                self.control.line_tracking(mode1, speed=50)
                # print("Go straight.\n")

            # 탈출 조건
            if self.obs_cnt == 2:
                break

            time.sleep(0.1)


class StaticBigObstacleMission(Mission):
    def __init__(self, db, control, path):
        super().__init__(db, control, path)
        # 현재 차선. 1차선 or 2차선
        self.current_line = 1
        self.mode = mode3

        # 사용하지 않을 수도 있음
        self.obstacle_cnt = 0

    def main(self):
        while True:
            # if self.path.add_total_gps_idx():
            #     break

            if self.obstacle_cnt >= 2:
                break

            # 도심부 대형 장애물, 현재 위치 -> 오른쪽 -> 왼쪽 매크로
            if self.control.distance() < 400:
                if self.current_line == 2:
                    self.control.lane_change_to_left(time.time())
                    self.current_line = 1
                    self.mode = mode3

                    self.obstacle_cnt += 1

                elif self.current_line == 1:
                    self.control.lane_change_to_right(time.time())
                    self.current_line = 2
                    self.mode = mode2

                    self.obstacle_cnt += 1

            else:
                self.control.line_tracking(self.mode, speed=70)

            time.sleep(0.1)


class DynamicObstacleMission(Mission):
    def __init__(self, db, control, path):
        super().__init__(db, control, path)
        self.dynamic_time = 0
        self.speed = 90

    def main(self):
        self.control.gps_mission_end = False

        filename = "gps1030_20.txt"
        filepath = os.path.join(os.getcwd(), 'Database', 'gps_logging', filename)

        with open(filepath) as f:
            datas = f.readlines()
            # gps data preprocessing
            parsed_datas = []
            for data in datas:
                single_degree = list(map(float, data[:-1].split(',')))
                single_degree = [single_degree[0] * 110000, single_degree[1] * 88800]
                parsed_datas.append(single_degree)

        while True:
            if self.control.gps_mission_end:  # 미션 탈출
                break

            if self.control.distance() <= 450:  # 장애물과의 거리가 4.5m 이내이면 계속 시간 갱신
                print(self.control.distance())
                self.dynamic_time = time.time()

            if time.time() - self.dynamic_time < 2:  # 장애물이 마지막으로 감지된지 2초 이내이면 정지
                self.control.stop()
            else:  # 장애물 감지 X
                self.control.GPS_tracking(parsed_datas, speed=self.speed)

            time.sleep(0.1)


class StraightTrafficLightMission(Mission):
    def __init__(self, mode, logging_data_idx, db, control, path):
        super().__init__(db, control, path)
        self.logging_data_idx = logging_data_idx  # 순서대로 하면 1로 시작
        self.mode = mode
        self.is_stopline = False
        self.traffic_flag = False
        self.gps_flag = False

        # 1 : 스쿨존 진입 직진
        # 2 : 정적장애물(대형) 이후 직진
        # 3 : 돌아오는 직진 - 1
        # 4 : 돌아오는 직진 - 2
        self.logging_filename_dict = {1: "gps1030_18.txt",
                                      2: "after_large_obstacle.txt",
                                      3: "comeback1.txt",
                                      4: "comeback2.txt"}

    def main(self):
        self.control.gps_mission_end = False

        filename = self.logging_filename_dict[self.logging_data_idx]
        filepath = os.path.join(os.getcwd(), 'Database', 'gps_logging', filename)

        stop_line_detected_time = 0

        with open(filepath) as f:
            datas = f.readlines()
            # gps data 전처리
            parsed_datas = []
            for data in datas:
                parsed_datas.append(list(map(float, data[:-1].split(','))))
                single_degree = [single_degree[0] * 110000, single_degree[1] * 88800]
                parsed_datas.append(single_degree)

        while True:
            # 미션을 강제로 종료하기 위한 장치, 추후 확인
            # if self.path.add_total_gps_idx():
            #     break

            # 정지선이 O green X -> 멈춤
            # 정지선이 O green O -> gps tracking
            # 정지선 X green X -> line tracking
            # 정지선 X green O -> line tracking

            # gps tracking에 진입하기 전까지는 신호등 값 계속 체크
            if not self.gps_flag and self.db.trafficlight == 'green':
                self.traffic_flag = True
            else:
                self.traffic_flag = False

            # default 직진
            # 신호를 계속 보고 있다가, 초록불 + 정지선 인식 -> 그대로 GPS trarcking
            # 초록불 X + 정지선 인식 -> 정지, 추후 신호 바뀌면 GPS trarcking

            # GPS tracking
            if self.gps_flag:
                if self.control.gps_mission_end:  # 미션 탈출
                    break
                self.control.GPS_tracking(parsed_datas)

            elif self.is_stopline:
                if self.traffic_flag:
                    self.gps_flag = True
                else:
                    # stop
                    self.control.stop()

                    if time.time() - stop_line_detected_time > 10:  # 10초 이상이면 그냥 GPS tracking
                        self.gps_flag = True

            else:  # line tracking
                self.control.line_tracking(self.mode)
                if self.path.line.check_stop_line():
                    if stop_line_detected_time == 0:
                        stop_line_detected_time = time.time()

                    self.is_stopline = True

            time.sleep(0.1)


class GpsBaseTrafficMission(Mission):
    def __init__(self, logging_data_idx, db, control, path):
        super().__init__(db, control, path)
        self.logging_data_idx = logging_data_idx
        self.speed = 120
        self.brake = 0

        self.traffic_flag = False
        self.gps_flag = False
        self.is_stopline = False

        # 1 : 스쿨존 좌회전
        # 2 : 좌회전 + 차선변경 before strange left
        # 3 : strange left
        self.logging_filename_dict = {1: "gps1030_19.txt", 2: "before_strange_trafficlight.txt",
                                      3: "strange_left.txt"}

    def main(self):
        filename = self.logging_filename_dict[self.logging_data_idx]
        if self.logging_data_idx == 1:
            self.speed = 70
            self.brake = 20
        if self.logging_data_idx == 2:
            self.is_stopline = True

        filepath = os.path.join(os.getcwd(), 'Database', 'gps_logging', filename)

        with open(filepath) as f:
            datas = f.readlines()
            # 전처리
            parsed_datas = []
            for data in datas:
                single_degree = list(map(float, data[:-1].split(',')))
                single_degree = [single_degree[0] * 110000, single_degree[1] * 88800]
                parsed_datas.append(single_degree)

        while True:
            if not self.gps_flag and self.db.trafficlight == 'arrow':
                self.traffic_flag = True
            else:
                self.traffic_flag = False

            if self.gps_flag:  # GPS tracking
                if self.control.gps_mission_end:  # 미션 탈출
                    break
                self.control.GPS_tracking(parsed_datas, speed=self.speed, brake=self.brake)
                self.brake = 0

            elif self.is_stopline:
                if self.traffic_flag:
                    self.gps_flag = True
                else:  # stop
                    self.control.stop()

                    if time.time() - stop_line_detected_time > 10:
                        self.gps_flag = True

            else:
                # GPS tracking
                self.control.GPS_tracking(parsed_datas, speed=self.speed, brake=self.brake)
                self.brake = 0

                if self.path.line.check_stop_line():
                    self.is_stopline = True

                    if stop_line_detected_time == 0:
                        stop_line_detected_time = time.time()


class GpsTrackingMission(Mission):
    def __init__(self, logging_data_idx, db, control, path):
        super().__init__(db, control, path)
        self.logging_data_idx = logging_data_idx
        self.speed = 100

        # 1 : 시작 ~ 주차구간 이전
        # 2 : 주차 이후 ~ 좌회전
        # 3 : 소형장애물 이후 ~ 우회번 2번
        # 4 : 동적장애물 -> 대형장애물 쪽으로 가는 우회전
        # 5 : 차선 변경
        # 6 : linetracking 시 차선이탈 발생하는 좌회전
        # 7 : 돌아오는 우회전
        # 8 : 마지막 비신호 직진
        # real
        # self.logging_filename_dict = {1: "gps1030_14.txt", 2: "gps1030_15.txt",
        #                               3: "gps1030_33.txt",
        #                               4: "before_large_obstacle.txt",
        #                               5: "after_large_obstacle.txt",
        #                               6: "before_strange_trafficlight.txt",
        #                               7: "comeback_right.txt", 8: "the_end.txt"}

        # 앞부분만 신호보기
        self.logging_filename_dict = {1: "gps1030_14.txt", 2: "gps1030_15.txt",
                                      3: "gps1030_33.txt",
                                      4: "gps1030_21.txt",
                                      5: "change_lane.txt",
                                      6: "option_left.txt",
                                      7: "comeback_right.txt", 8: "the_end.txt"}

        # 비상용 - all gps
        # self.logging_filename_dict = {1: "gps1030_14.txt", 2: "gps1030_15.txt",
        #                               3: "gps1030_33.txt",
        #                               4: "gps1030_18.txt",
        #                               5: "gps1030_19.txt",
        #                               6: "gps1030_21.txt"}

    def main(self):
        start_time = time.time()

        self.control.gps_mission_end = False

        filename = self.logging_filename_dict[self.logging_data_idx]
        filepath = os.path.join(os.getcwd(), 'Database', 'gps_logging', filename)

        with open(filepath) as f:
            datas = f.readlines()
            # 전처리
            parsed_datas = []
            for data in datas:
                single_degree = list(map(float, data[:-1].split(',')))
                single_degree = [single_degree[0] * 110000, single_degree[1] * 88800]
                parsed_datas.append(single_degree)

        self.control.gps_tracking_idx = self.control.GPS_first_index(parsed_datas)

        while True:
            if self.logging_data_idx == 1 and time.time() - start_time < 8:  # 초반에는 단순 직진
                self.control.simple_straight_forward(speed=50)
                time.sleep(0.1)
                continue

            # 시작 인덱스 보정
            elif self.logging_data_idx == 1 and time.time() - start_time >= 4:
                self.control.gps_tracking_idx = self.control.GPS_first_index(parsed_datas)

            if self.control.gps_mission_end:  # 미션 탈출
                break

            # GPS Tracking
            self.control.GPS_tracking(parsed_datas, speed=self.speed)
            brake = 0
            time.sleep(0.1)


class LineTrackingMission(Mission):
    def __init__(self, mode, db, control, path, end_location=None):
        super().__init__(db, control, path)
        self.mode = mode
        self.is_stopline = False
        self.end_location = end_location

    def main(self):
        self.control.default_start_time = time.time()

        while True:
            # 미션 강제종료를 위한 장치. 추후 확인
            # if self.path.add_total_gps_idx():
            #     break

            # line tracking 시 gps 특정 위치에 도달하면 break 조건(특정 상황에서만 썼음)
            if self.end_location is not None:
                gps_data = self.db.gps.data

                latitude = gps_data[6]
                longitude = gps_data[7]

                current_position = [latitude * 110000, longitude * 88800]

                distance = (current_position[0] - self.end_location[0]) ** 2 + \
                           (current_position[1] - self.end_location[1]) ** 2

                # Distance Square 9
                if distance < 9:
                    break

            # 정지선flag가 True면 정지
            if self.is_stopline:
                break
            # 라인 트래킹
            else:
                self.control.line_tracking(self.mode, speed=120)
                # 정지선 검출 함수
                if self.path.line.check_stop_line():
                    self.is_stopline = True

            time.sleep(0.1)


class EndMission(Mission):
    def main(self):
        while True:
            self.control.stop()


# StrainghtTraffic이랑 동일
class LeftTrafficLightMission(Mission):
    def __init__(self, logging_data_idx, db, control, path):
        super().__init__(db, control, path)
        self.logging_data_idx = logging_data_idx  # 순서대로 하면 1로 시작
        self.logging_filename_dict = {1: "first_left.txt", 2: "strange_left.txt"}

    def main(self):
        self.control.gps_mission_end = False
        filename = self.logging_filename_dict[self.logging_data_idx]
        filepath = os.path.join(os.getcwd(), 'Database', 'gps_logging', filename)
        idx = 0
        stop_line_detected_time = 0
        self.is_stopline = False

        with open(filepath) as f:
            datas = f.readlines()
            # 전처리
            parsed_datas = []
            for data in datas:
                parsed_datas.append(list(map(float, data[:-1].split(','))))

        while True:
            if self.path.add_total_gps_idx():
                break

            # 정지선이 O arrow X -> 멈춤
            # 정지선이 O arrow O -> gps tracking
            # 정지선 X arrow X -> line tracking
            # 정지선 X arrow O -> line tracking

            if self.path.line.check_stop_line():
                self.is_stopline = True

            if self.is_stopline == True:
                if self.db.trafficlight == 'arrow':
                    # GPS tracking
                    if self.control.gps_mission_end:
                        # 미션 탈출 조건
                        break
                    self.control.GPS_tracking(parsed_datas)
                else:  # stop
                    if stop_line_detected_time == 0:
                        print("STOP LINE DETECTED")
                        stop_line_detected_time = time.time()

                    if time.time() - stop_line_detected_time > 10:
                        # for test
                        if self.control.gps_mission_end:  # 미션 탈출 조건
                            break
                        self.control.GPS_tracking(parsed_datas)
                    else:
                        print("STOP LINE")
                        self.control.stop()
            else:  # line tracking
                self.control.line_tracking(mode1, speed=60)

            time.sleep(0.1)


class EmergencyMission(Mission):
    def main(self):
        self.control.simple_straight_forward(speed=80)


if __name__ == "__main__":
    db = Database(gps=False, imu=False, platform=False, cam=False, lidar=False)
    db.start()

    time.sleep(1)

    path = Path(db=db)
    control = Control(db=db, path=path)

    # obstacle = StaticSmallObstacleMission(db=db, control=control, path=path)
    # left_traffic = LeftTrafficLightMission(db=db, control=control, path=path)
    # straight_traffic = StraightTrafficLightMission(db=db, control=control, path=path)
    # gps_tracking = GpsTrackingMission(db=db, control=control, path=path)
    # default = DefaultMission(db=db, control=control, path=path)
    # parking = ParkingMission(db=db, control=control, path=path)
    #
    # obstacle.main()
    # left_traffic.main()
    # straight_traffic.main()
    # gps_tracking.main()
    # default.main()
    # parking.main()
