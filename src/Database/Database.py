import sys
import os
sys.path.append(os.path.dirname(__file__))

from src.Database.CAM import CAM
from src.Database.Flag import Flag
from src.Database.GPS import GPS
from src.Database.LiDAR import LiDAR
from src.Database.Platform import Platform
from src.Database.imu import Imu
from src.Database.Screens import Screens
from src.Database.ControlData import ControlData

import threading
import time

class Database:
    def __init__(self, gps=True, platform=True, cam=True, lidar=True, imu=True):
        self.__gps_on = gps
        self.__platform_on = platform
        self.__cam_on = cam
        self.__lidar_on = lidar
        self.__imu_on = imu

        self.mission = ''
        self.flag = Flag()
        self.screen = Screens()
        self.control_data = ControlData()
        self.mission_packet = [0, None, None, None, None]
        self.now_idx = 0
        self.position = 0
        self.degree = 0
        self.trafficlight = ''
        self.yolo_sign_list = []
        self.yolo_buffer_sign = ''

        # for total gps idx
        self.total_gps_idx = 0
        # read total track gps tracking file
        self.total_track = []

        #캠을 제외한 센서는 Threading처리
        print("\nIntializing Database")
        print("──────────────────────────────────────────────────────────────────────────────────────────────────────────────────")
        if self.__gps_on:
            self.gps = GPS('COM8', 9600, flag=self.flag)
            self.__gps_thread = threading.Thread(target=self.gps.main)

        if self.__platform_on:
            self.platform  = Platform('COM21', 115200, flag=self.flag)
            self.__platform_thread = threading.Thread(target=self.platform.main)

        if self.__cam_on:
            self.main_cam = CAM(0, 'Main', flag=self.flag)
            self.sub_cam = CAM(1, 'Sub', flag=self.flag)
            self.parking_cam = CAM(2, 'Parking', flag=self.flag)
            self.front_cam = CAM(3, 'Front', flag=self.flag)
            self.__main_cam_thread = threading.Thread(target=self.main_cam.main)
            self.__sub_cam_thread = threading.Thread(target=self.sub_cam.main)
            self.__parking_cam_thread = threading.Thread(target=self.parking_cam.main)
            self.__front_cam_thread = threading.Thread(target=self.front_cam.main)

        if self.__lidar_on:
            self.lidar = LiDAR('169.254.204.42', 2111, 57600, flag=self.flag)
            self.__lidar_thread = threading.Thread(target=self.lidar.main)

        if self.__imu_on:
            self.imu = Imu("COM3"
                           "", 115200, flag=self.flag)
            self.__imu_thread = threading.Thread(target=self.imu.main)

        print("──────────────────────────────────────────────────────────────────────────────────────────────────────────────────")
        print("Database is ready to run!")

    def start(self):
        print("\nStart to run Database...")
        print("──────────────────────────────────────────────────────────────────────────────────────────────────────────────────")
        if self.__gps_on:
            self.__gps_thread.start()
            time.sleep(0.1)

        if self.__platform_on:
            self.__platform_thread.start()
            time.sleep(0.1)

        if self.__cam_on:
            self.__main_cam_thread.start()
            time.sleep(0.1)
            self.__sub_cam_thread.start()
            time.sleep(0.1)
            self.__parking_cam_thread.start()
            time.sleep(0.1)
            self.__front_cam_thread.start()
            time.sleep(0.1)
        
        if self.__lidar_on:
            self.__lidar_thread.start()
            time.sleep(0.1)
        
        if self.__imu_on:
            self.__imu_thread.start()
            time.sleep(1)
        print("──────────────────────────────────────────────────────────────────────────────────────────────────────────────────")
        print("Database is running!\n")
    
    def join(self):
        print("\nTerminating Database...")
        print("──────────────────────────────────────────────────────────────────────────────────────────────────────────────────")
        if self.__gps_on:
            self.__gps_thread.join()

        if self.__platform_on:
            self.__platform_thread.join()

        if self.__cam_on:
            self.__main_cam_thread.join()
            self.__sub_cam_thread.join()
            self.__parking_cam_thread.join()
            self.__front_cam_thread.join()

        if self.__lidar_on:
            self.__lidar_thread.join()
            time.sleep(0.1)

        if self.__imu_on:
            self.__imu_thread.join()
        print("──────────────────────────────────────────────────────────────────────────────────────────────────────────────────")
        print("Database termination complete!")


