import os
import sys

sys.path.append(".")
sys.path.append("C:\\Users\\HEVEN_A\\darknet\\build\\darknet\\x64")
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

from src.Database import Database
from Control.Control import Control
from Path import Path
from src.Mission import MissionManager, StaticSmallObstacleMission, StaticBigObstacleMission,\
    DynamicObstacleMission, LeftTrafficLightMission, \
    StraightTrafficLightMission, GpsTrackingMission, LineTrackingMission, ParkingMission, GpsBaseTrafficMission, EndMission, EmergencyMission, mode1, mode2, mode3, mode4
# from YOLO.threading_yolo import YOLO
import time
import cv2


def main():
    # 미션 매니저 생성
    mission_manager = MissionManager()
    db = Database()
    db.start()
    time.sleep(0.1)

    mission_manager.add_database(db)

    # yolo = YOLO(db=db)
    # yolo.start()

    path = Path(db=db)
    control = Control(db=db, path=path)

    # 수행할 미션 순서

    # VER 1. 최대한 차선 보는 미션
    # LeftTrafficLight - 신호 보는 gps
    # StraightTrafficLight - 신호 보는 gps
    # GpsTracking - 신호 안보는 gps
    # LineTracking - 차선 인식 주행
    # 전체 신호봄
    # mission_manager.mission_keys = ["GpsTracking1",  # 주차 전 GPS tracking
    #                                 "Parking",  # 주차
    #                                 "GpsTracking2",  # 비신호 좌회전 이전 정지선까지
    #                                 "GpsTracking3",  # 좌회전 + ㄷ자
    #                                 "StraightTrafficLight1", # 스쿨존 진입하는 직진
    #                                 "GpsBaseTraffic1",  # 스쿨존 좌회전
    #                                 "DynamicObstacle", # 동적 장애물
    #                                 "StraightTrafficLight2", # 정적 대형 이후 직진
    #                                 ]

    # 앞부분만 신호봄
    # mission_manager.mission_keys = ["GpsTracking1",  # 주차 전 GPS tracking
    #                                 "Parking",  # 주차
    #                                 "GpsTracking2",  # 비신호 좌회전 이전 정지선까지
    #                                 "GpsTracking3",  # 좌회전 + ㄷ자
    #                                 "StraightTrafficLight1", # 스쿨존 진입하는 직진
    #                                 "GpsBaseTraffic1",  # 스쿨존 좌회전
    #                                 "DynamicObstacle", # 동적 장애물
    #                                 "GpsTracking4"]

    # VER 2. 최대한 GPS 보는 미션
    # GPS tracking mission 키 변경하고 시작하기
    # mission_manager.mission_keys = ["GpsTracking1", # 단순직진 n초 + gpstracking
    #                                 "Parking", "GpsTracking2", # 처음에 로깅한 것 두번째 -> 정지선까지
    #                                 "GpsTracking3", # ㄷ자
    #                                 "GpsTracking4", # 스쿨존 진입
    #                                 "GpsTracking5",  # 스쿨존 좌회전
    #                                 "DynamicObstacle", # 동적 장애물
    #                                 "GpsTracking6" # Dynamic 종료 후부터 끝까지
    #                                 ]

    # full tracking 아닐 때 미션키

    # ["StaticBigObstacle",  # 정적 장애물(대형)
    # "StraightTrafficLight2",
    # "LineTrackingMode1",
    # "GpsTracking5", # 차선 변경
    # "LineTrackingMode3", # 차선 변경 이후 align 맞추기용
    # "GpsBaseTraffic2",  # 좌회전 + 차선변경
    # "LineTrackingMode1", # 차선 변경 이후 align 맞추기용
    # "GpsBaseTraffic3", # strange 좌회전
    # "LineTrackingEndCondition",
    # "GpsTracking6", # 좌회전 - linetracking 가능할까?
    # "LineTrackingMode1",
    # "GpsTracking7", # 돌아오는 우회전
    # "StraightTrafficLight3", # 돌아오는 직진 - 1
    # "StraightTrafficLight4", # 돌아오는 직진 - 2
    # "LineTrackingMode1", "GpsTracking8",
    # "LineTrackingMode1",
    # "EndMission"] # 정지

    # VER 3. 연습주행용
    mission_manager.mission_keys = ["DynamicObstacle"]

    # 시작 미션 할당(일반적인 경우 0번째 미션부터 시작)
    mission_manager.mission_idx = 0
    mission_manager.current_mission_key = mission_manager.mission_keys[mission_manager.mission_idx]

    # 수행할 미션들을 생성
    static_small_obstacle_mission = StaticSmallObstacleMission(db=db, control=control, path=path)
    static_big_obstacle_mission = StaticBigObstacleMission(db=db, control=control, path=path)

    dynamic_obstacle_mission = DynamicObstacleMission(db=db, control=control, path=path)

    # 직진 미션별 객체
    # mode1 : left, (yellow, white)
    # mode2 : right, (white)
    # mode3 : left, (blue)
    straighttraffic_mission1 = StraightTrafficLightMission(mode=mode1, logging_data_idx=1, db=db, control=control, path=path)
    straighttraffic_mission2 = StraightTrafficLightMission(mode=mode2, logging_data_idx=2, db=db, control=control, path=path)
    straighttraffic_mission3 = StraightTrafficLightMission(mode=mode1, logging_data_idx=3, db=db, control=control, path=path)
    straighttraffic_mission4 = StraightTrafficLightMission(mode=mode1, logging_data_idx=4, db=db, control=control, path=path)

    # gps tracking 객체
    gps_tracking_mission1 = GpsTrackingMission(logging_data_idx=1, db=db, control=control, path=path)
    gps_tracking_mission2 = GpsTrackingMission(logging_data_idx=2, db=db, control=control, path=path)
    gps_tracking_mission3 = GpsTrackingMission(logging_data_idx=3, db=db, control=control, path=path)
    gps_tracking_mission4 = GpsTrackingMission(logging_data_idx=4, db=db, control=control, path=path)
    gps_tracking_mission5 = GpsTrackingMission(logging_data_idx=5, db=db, control=control, path=path)
    gps_tracking_mission6 = GpsTrackingMission(logging_data_idx=6, db=db, control=control, path=path)
    gps_tracking_mission7 = GpsTrackingMission(logging_data_idx=7, db=db, control=control, path=path)
    gps_tracking_mission8 = GpsTrackingMission(logging_data_idx=8, db=db, control=control, path=path)

    # gps tracking + 신호등 객체
    gps_base_traffic_mission1 = GpsBaseTrafficMission(logging_data_idx=1, db=db, control=control, path=path)
    gps_base_traffic_mission2 = GpsBaseTrafficMission(logging_data_idx=2, db=db, control=control, path=path)
    gps_base_traffic_mission3 = GpsBaseTrafficMission(logging_data_idx=3, db=db, control=control, path=path)

    # Line tracking 객체
    # mode1 : left, (yellow, white)
    # mode2 : right, (white)
    # mode3 : left, (blue)
    line_tracking_left_yw_mission = LineTrackingMission(mode=mode1, db=db, control=control, path=path)
    line_tracking_right_w_mission = LineTrackingMission(mode=mode2, db=db, control=control, path=path)
    line_tracking_left_b_mission = LineTrackingMission(mode=mode3, db=db, control=control, path=path)
    line_tracking_break_condition_mission = LineTrackingMission(mode=mode1, db=db, control=control, path=path,
                                                                end_location=[37.24186683, 126.77373416])

    line_tracking_mode4_mission = LineTrackingMission(mode=mode4, db=db, control=control, path=path)

    # 기타 미션 객체
    parking_mission = ParkingMission(db=db, control=control, path=path)
    end_mission = EndMission(db=db, control=control, path=path)

    emergency_mission = EmergencyMission(db=db, control=control, path=path)

    # NOT USED : 좌회전 미션별 객체
    lefttraffic_mission1 = LeftTrafficLightMission(logging_data_idx=1, db=db, control=control, path=path)
    lefttraffic_mission2 = LeftTrafficLightMission(logging_data_idx=2, db=db, control=control, path=path)

    # 미션 매니저에 수행할 미션들을 추가.
    # mission_manager.add_mission(key="StaticSmallObstacle", mission=static_small_obstacle_mission)
    # mission_manager.add_mission(key="StaticBigObstacle", mission=static_big_obstacle_mission)
    mission_manager.add_mission(key="DynamicObstacle", mission=dynamic_obstacle_mission)
    #
    # mission_manager.add_mission(key="StraightTrafficLight1", mission=straighttraffic_mission1)
    # mission_manager.add_mission(key="StraightTrafficLight2", mission=straighttraffic_mission2)
    # mission_manager.add_mission(key="StraightTrafficLight3", mission=straighttraffic_mission3)
    # mission_manager.add_mission(key="StraightTrafficLight4", mission=straighttraffic_mission4)
    #
    # mission_manager.add_mission(key="GpsTracking1", mission=gps_tracking_mission1)
    # mission_manager.add_mission(key="GpsTracking2", mission=gps_tracking_mission2)
    # mission_manager.add_mission(key="GpsTracking3", mission=gps_tracking_mission3)
    # mission_manager.add_mission(key="GpsTracking4", mission=gps_tracking_mission4)
    # mission_manager.add_mission(key="GpsTracking5", mission=gps_tracking_mission5)
    # mission_manager.add_mission(key="GpsTracking6", mission=gps_tracking_mission6)
    # mission_manager.add_mission(key="GpsTracking7", mission=gps_tracking_mission7)
    # mission_manager.add_mission(key="GpsTracking8", mission=gps_tracking_mission8)
    #
    # mission_manager.add_mission(key="GpsBaseTraffic1", mission=gps_base_traffic_mission1)
    # mission_manager.add_mission(key="GpsBaseTraffic2", mission=gps_base_traffic_mission2)
    # mission_manager.add_mission(key="GpsBaseTraffic3", mission=gps_base_traffic_mission3)
    #
    # mission_manager.add_mission(key="LineTrackingMode1", mission=line_tracking_left_yw_mission)
    # mission_manager.add_mission(key="LineTrackingMode2", mission=line_tracking_right_w_mission)
    # mission_manager.add_mission(key="LineTrackingMode3", mission=line_tracking_left_b_mission)
    # mission_manager.add_mission(key="LineTrackingEndCondition", mission=line_tracking_break_condition_mission)
    #
    # mission_manager.add_mission(key="Parking", mission=parking_mission)
    # mission_manager.add_mission(key="EndMission", mission=end_mission)
    #
    # mission_manager.add_mission(key="Emergeny", mission=emergency_mission)

    # NOT USED
    # mission_manager.add_mission(key="LeftTrafficLight1", mission=lefttraffic_mission1)
    # mission_manager.add_mission(key="LeftTrafficLight2", mission=lefttraffic_mission2)

    mission_manager.start()

    while True:
        if db.flag.system_stop:
            break
        else:
            try:
                print(mission_manager.current_mission_key)
                print(db.mission)
                time.sleep(1)
            except KeyboardInterrupt:
                print("Keyboard Interrupt detected!")
                db.flag.system_stop = True
                break

    # yolo.join()
    mission_manager.join()
    db.join()

    return 0


if __name__ == "__main__":
    if main() == 0:
        print("\nAutonomous-Car-System terminated successfully!")
    else:
        print("\nThere is something wrong. I recommend you to kill every processes which is related to this program.")
