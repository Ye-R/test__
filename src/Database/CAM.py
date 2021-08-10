import time
import copy
import cv2
import sys
import os
sys.path.append(os.path.dirname(__file__))
from Flag import Flag
class CAM:
    def __init__(self, num, name, flag: Flag):
        self.__data = None
        self.__name = name
        self.flag = flag
        self.__cam_initializing_success = False
        self.__capture = cv2.VideoCapture(num, cv2.CAP_DSHOW) #캡쳐시작
        if self.__capture.read()[0]:    # If cam works well, it return (True, Data)
            self.__cam_initializing_success = True
            print("[%s CAM Intializing \tOk  ]" % self.__name)
        else:
            self.__capture.release()
            print("[%s CAM Intializing \tFail] \tCan not read image from cam successfully. " % self.__name)

    def main(self):
        if self.__cam_initializing_success:
            print("Start %s CAM\t- Success\n" % self.__name)
            time.sleep(1)
            self.__read_cam()
        else:
            print("Start %s CAM\t- Fail:\t\t%s CAM doesn't initialize succeessfully. Therefore, %s CAM will not run." % (self.__name, self.__name, self.__name))
        print("\t\t\t\t-->\tTerminate %s CAM" % self.__name)

    def __read_cam(self):
        global out2, out3, out1,out4
        '''
        if self.__name == "Main":
             out1 = cv2.VideoWriter(os.path.join(os.getcwd(),'Database','cam_logging','Main.avi'), cv2.VideoWriter_fourcc('D', 'I', 'V', 'X'), 20,
                               (640, 480))
        elif self.__name == "Sub":
             out2 = cv2.VideoWriter(os.path.join(os.getcwd(),'Database', 'cam_logging', 'Sub.avi'), cv2.VideoWriter_fourcc('D', 'I', 'V', 'X'), 20,
                               (int(self.__capture.get(3)), int(self.__capture.get(4))))
        elif self.__name == "Parking":
             out3 = cv2.VideoWriter(os.path.join(os.getcwd(),'Database', 'cam_logging', 'Parking.avi'), cv2.VideoWriter_fourcc('D', 'I', 'V', 'X'), 20,
                               (int(self.__capture.get(3)), int(self.__capture.get(4))))
        elif self.__name == "Front":
             out4 = cv2.VideoWriter(os.path.join(os.getcwd(),'Database', 'cam_logging', 'Front.avi'), cv2.VideoWriter_fourcc('D', 'I', 'V', 'X'), 20,
                               (int(self.__capture.get(3)), int(self.__capture.get(4))))
        '''
        #database.py에 각 이미지를 parameter화해서 넣어줌
        while not self.flag.system_stop:
            if self.flag.cam_stop:
                time.sleep(0.1)
            else:
                success, frame = self.__capture.read()
                if success:
                    #print(frame)
                    if self.__name=="Main":
                        # cv2.imshow("MainCamera",frame)
                        #out1.write(frame)
                        #cv2.putText(frame," alala ", (20,20), cv2.FONT_HERSHEY_PLAIN, 1,
                        #(0, 255, 0))
                        #cv2.imshow("MainCamera", frame)
                        cv2.waitKey(1)
                        self.__data = frame
                    elif self.__name=="Sub":
                        # cv2.imshow("SubCamera",frame)
                        #out2.write(frame)
                        cv2.waitKey(1)
                        self.__data = frame
                    elif self.__name=="Parking":
                        # cv2.imshow("ParkCamera",frame)
                        #out3.write(frame)
                        cv2.waitKey(1)
                        self.__data = frame
                    elif self.__name == 'Front':
                        # cv2.imshow("Front",frame)
                        #out4.write(frame)
                        cv2.waitKey(1)
                        self.__data = frame
                else:
                    pass
        time.sleep(0.1)
        self.__capture.release()
        print("Terminating %s CAM" % self.__name)


    @property
    def data(self):
        return copy.deepcopy(self.__data)

if __name__ == '__main__':
    import cv2

    cap = cv2.VideoCapture(0)

    while True:
        ret, fram = cap.read()

        if ret:
            gray = cv2.cvtColor(fram, cv2.COLOR_BGR2GRAY)
            cv2.imshow('video', gray)
            k = cv2.waitKey(1) & 0xFF
            if k == 27:
                break
        else:
            print('error')

    cap.release()
    cv2.destroyAllWindows()
