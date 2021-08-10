#-*- coding:utf-8 -*-
import socket
import time
import copy
import sys
import os
sys.path.append(os.path.dirname(__file__))

from Flag import Flag

class LiDAR:
    def __init__(self, host, port, buff, flag: Flag):
        self.__data = None
        self.flag = flag
        self.__lidar_initializing_success = False
        self.__buff = buff

        INIT_MESG = chr(2) + 'sEN LMDscandata 1' + chr(3)

        #소켓 모듈을 이용한 통신
        try:
            self.__socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.__socket.connect((host, port))
            self.__socket.send(str.encode(INIT_MESG))
            self.__lidar_initializing_success = True
            print("[LiDAR Intializing \tOk  ]")
        except Exception as e:
            print("[LiDAR Intializing \tFail] \tError occurred while opening the socket:", e)

    def main(self):
        if self.__lidar_initializing_success:
            print("Start LiDAR\t- Success\n")
            time.sleep(1)
            self.__read_lidar()
        else:
            print("Start LiDAR\t- Fail:\t\tLiDAR doesn't initialize succeessfully. Therefore, LiDAR will not run.")
        print("\t\t\t\t-->\tTerminate LiDAR")

    def __read_lidar(self):
        while not self.flag.system_stop:
            if self.flag.lidar_stop:
                time.sleep(0.1)
            else:
                data = str(self.__socket.recv(self.__buff))
                #data=[7611, 7591, 7609, 7618, 7599, 7598, 6773, 1000000, 3362, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 1000000, 7586, 7542, 7487, 7439, 7432, 7355, 7345, 7304, 7329, 7289, 7067, 6991, 6932, 6763, 6976, 6731, 6435, 6102, 6019, 5972, 5770, 5682, 5575, 5417, 5345, 5253, 5256, 5242, 5157, 5023, 5050, 4989, 4919, 4847, 4842, 4811, 4744, 4722, 4328, 3962, 3875, 3816, 3770, 1000000, 1000000, 1000000, 1000000, 4043, 4226, 4221, 4175, 4073, 4110, 4107, 4085, 4094, 4099, 4070, 4059, 4032, 4011, 4042, 4025, 4014, 3995, 3970, 3945, 3927, 3920, 3905, 3906, 3862, 3871, 3886, 3863, 3863, 3850, 3862, 3841, 3841, 3835, 3835, 3832, 3814, 3819, 3811, 3801, 3788, 3796, 3792, 3787, 3788, 3789, 3776, 3757, 3755, 3763, 3772, 3757, 3747, 3739, 3768, 3755, 3763, 3760, 3761, 3749, 3779, 3769, 3756, 3748, 3726, 3776, 3779, 3782, 3789, 3801, 3781, 3798, 3802, 3820, 3846, 3842, 3855, 3877, 3895, 3909, 3921, 3927, 3933, 3926, 3976, 3983, 3969, 3969, 4001, 3991, 3994, 3987, 3989, 3986, 3997, 4035, 4035, 4059, 4072, 4067, 4081, 4115, 4117, 4115, 4133, 4133, 4152, 4161, 4162, 4140, 4158, 4153, 4163, 4190, 4214, 4215, 4236, 4240, 4253, 4282, 4295, 4327, 4356, 4371, 4399, 4418, 4423, 4445, 4479, 4507, 4547, 4572, 4592, 4654, 4644, 4591, 4546, 4567, 4587, 4709, 4816, 4856, 4905, 4911, 4952, 4996, 5033, 5057, 5115, 5146, 5193, 5247, 5295, 5360, 5442, 5530, 5588, 5640, 5726, 5763, 5849, 5905, 5912, 5892, 5929, 6025, 6169, 6255, 6330, 6444, 6501, 6566, 6632, 6819, 6862, 7072, 7500, 7463, 7443, 7394, 7346, 7348, 7356, 7358, 7377, 7419, 7667, 8028, 8172, 8297, 8381, 8485, 8574, 8689, 8796, 8921, 9051, 9194, 9384, 9598, 9796, 10107, 10106, 10083, 1000000, 1000000, 1000000, 14262, 14159, 14034, 13942, 13887, 13737, 13683, 13599, 13494, 13386, 13351, 13269, 13211, 13239, 13250, 13158, 13133, 13069, 13214, 13101, 15201, 15117, 15067, 14998, 14891, 14812, 14700, 14651, 14556, 14461, 14410, 14343, 14233, 14167, 14111, 14028, 13978, 13951, 13873, 13827, 13773, 13702, 13634, 13605, 13658, 15769, 15762, 15762, 15700, 15675, 15710, 13746, 13617, 13612, 13669, 13677, 13742, 15571, 15525, 15550, 12937, 12881, 12831, 12811, 12791, 12762, 12774, 12730, 12723, 12684, 12670, 12672, 12666, 12672, 12678, 12637, 12632, 12631, 12645, 12589, 1706, 1592, 1573, 1593, 1599, 1596, 1592, 1606, 1613, 1610]
                if data.__contains__('sEA'):
                    pass
                else:
                    try:
                        data = data.split(' ')[116:477] #Platform Lidar 범위 경험값(전방 180도)
                        temp_data = [1000000] * 361
                        for i in range(0, 361):
                            r = int(data[i], 16)
                            if r > 3: #outlier filtering
                                temp_data[i] = r
                        self.__data = copy.deepcopy(temp_data)
                        temp_data.sort()
                        # print(temp_data)
                    except Exception as e:
                        print("[LiDAR Running \tError] \t Error occured while parsing data from LiDAR:", e)
        
        time.sleep(0.1)
        print("Terminating LiDAR")
        self.__socket.send(str.encode(chr(2) + 'sEN LMDscandata 0' + chr(3)))
        self.__socket.close()


    @property
    def data(self):
        return copy.deepcopy(self.__data)


if __name__ == "__main__":
    gps = LiDAR('169.254.204.42', 2111, 57600, flag=Flag())
    gps.main()