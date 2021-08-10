from src.Database import Database
import os
import time
import sys

if __name__ == "__main__":
    db = Database(cam=False, platform=True, gps=True, lidar=False, imu=False)
    db.start()

    while (True):

        if db.gps.data is not None:
            with open(os.path.join(os.getcwd(), 'Database', 'gps_logging', '0917_school_whoop.txt'), 'a') as df:
                df.writelines(str(db.gps.data[6]) + "," + str(db.gps.data[7]) + "\n")
                print(db.gps.data[6],db.gps.data[7])
                time.sleep(1)
