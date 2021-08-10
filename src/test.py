import os

import cv2
from Database import Database
from Path import Path
from Record import gpsmap, imu_show
import time
import threading

if __name__ == "__main__":
    db = Database(cam=True, platform=True, gps=True, lidar=True, imu=True)
    db.start()
    # gpsimage = gpsmap.make_default_img()
    # lis=list()
    #     gpsimage = gpsmap.make_default_img()

    lis = list()
    r = 0
    fourcc_codec = cv2.VideoWriter_fourcc(*'XVID')
    #out_lidar = cv2.VideoWriter('Lidar.avi', fourcc_codec, 2, (600, 300))
    
   
    #out_gps = cv2.VideoWriter('GPS.avi', cv2.VideoWriter_fourcc('D', 'I', 'V', 'X'), 2,
    #                          (gpsimage.shape[0], gpsimage.shape[1]))
    
    out_main = cv2.VideoWriter(os.path.join(os.getcwd(), 'Database', 'cam_logging', 'Main.avi'),
                               cv2.VideoWriter_fourcc('D', 'I', 'V', 'X'), 60,
                               (640,480))
    out_sub = cv2.VideoWriter(os.path.join(os.getcwd(), 'Database', 'cam_logging', 'Sub.avi'),
                              cv2.VideoWriter_fourcc('D', 'I', 'V', 'X'), 60,
                              (640, 480))
    out_parking = cv2.VideoWriter(os.path.join(os.getcwd(), 'Database', 'cam_logging', 'Parking.avi'),
                                  cv2.VideoWriter_fourcc('D', 'I', 'V', 'X'), 60,
                                  (640, 480))
    out_front = cv2.VideoWriter(os.path.join(os.getcwd(), 'Database', 'cam_logging', 'Front.avi'),
                                  cv2.VideoWriter_fourcc('D', 'I', 'V', 'X'), 60,
                                  (640,480))

    # with open("gps_degree1.txt", 'w') as df, open("gps_nmea1.txt", 'w') as nf:
    #     time.sleep(0.3)

    while (True):
        try:

            # if db.lidar.data is not None:
            #     var = Path(db=db).narrow()
            #     #out_lidar.write(var[2])
            #     cv2.imshow("lidar",var[2])
            #     cv2.waitKey(1)

            # if db.gps.data is not None:
            #     with open(os.path.join(os.getcwd(), 'Database', 'gps_logging', 'gps_degree.txt'), 'a') as df:
            #         df.writelines(str(db.gps.data[6]) + "," + str(db.gps.data[7]) + "\n")
            #     with open(os.path.join(os.getcwd(), 'Database', 'gps_logging', 'gps_nmea.txt'), 'a') as nf:
            #         nf.writelines(str(db.gps.data[1]) + " " + str(db.gps.data[2]) + "," + str(db.gps.data[3]) + " " + str(db.gps.data[4]) + "\n")
                #datas = [[db.gps.data[1], db.gps.data[3]]]
                #gps = gpsmap.update(gpsimage, datas, lis)
                #out_gps.write(gps[0])
                #cv2.imshow("gps", gps[0])
                #cv2.waitKey(1)

            if db.main_cam.data is not None:
                frame=db.main_cam.data
                cv2.putText(frame, str(db.gps.data[6])+" "+ str(db.gps.data[7]) , (20, 20),cv2.FONT_HERSHEY_PLAIN, 1,(255,0,0))
                out_main.write(frame)

                cv2.imshow("main_Cam", frame)
                cv2.waitKey(1)
            if db.sub_cam.data is not None:
                out_sub.write(db.sub_cam.data)
                cv2.imshow("sub_Cam", db.sub_cam.data)
                cv2.waitKey(1)
            if db.parking_cam.data is not None:
                out_parking.write(db.parking_cam.data)
                cv2.imshow("parking_cam",db.parking_cam.data)
                cv2.waitKey(1)
            if db.front_cam.data is not None:
                out_front.write(db.front_cam.data)
                cv2.imshow("front_Cam", db.front_cam.data)
                cv2.waitKey(1)
            # if db.imu.data is not None:
            #     imu_image = imu_show.imu_show(db.imu.data)
            #     #cv2.imshow("imu",imu_image)
            #     cv2.waitKey(1)
        except KeyboardInterrupt:
            #out_lidar.release()
            #out_gps.release()
            #out_main.release()
            #out_sub.release()
            #out_parking.release()
            cv2.destroyAllWindows()
            stop = True
            break
