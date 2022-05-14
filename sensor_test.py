# In settings.json first activate computer vision mode:
# https://github.com/Microsoft/AirSim/blob/master/docs/image_apis.md#computer-vision-mode

import airsim

# requires Python 3.5.3 :: Anaconda 4.4.0
# pip install opencv-python
import cv2
import time
import sys
from threading import Thread
import numpy as np
import pprint
from matplotlib import pyplot as plt

cameraType = "depth"

client = airsim.MultirotorClient()
client_2 = airsim.MultirotorClient()
client_3 = airsim.MultirotorClient()
help = False
client.confirmConnection()
client.enableApiControl(True)

gpsData = client_3.getGpsData()
start_point_altitude = gpsData.gnss.geo_point.altitude

print("arming the drone...")
client_2.armDisarm(True)

client_2.takeoffAsync().join()
altitude = -10
client_2.moveToPositionAsync(0, 0, altitude, 5).join()

controller = 0


def camera():
    global controller
    fontFace = cv2.FONT_HERSHEY_SIMPLEX
    fontScale = 0.5
    thickness = 2
    textSize, baseline = cv2.getTextSize("FPS", fontFace, fontScale, thickness)
    print(textSize)
    textOrg = (10, 10 + textSize[1])
    frameCount = 0
    startTime = time.time()
    fps = 0
    counter = 0
    imgCounter = 1

    while True:
        if controller:
            cv2.destroyAllWindows()
            break
        # because this method returns std::vector<uint8>, msgpack decides to encode it as a string unfortunately.
        rawImage = client.simGetImage("0", airsim.ImageType.Scene)
        if (rawImage == None):
            print("Camera is not returning image, please check airsim for error messages")
            sys.exit(0)
        else:
            png = cv2.imdecode(airsim.string_to_uint8_array(rawImage), cv2.IMREAD_UNCHANGED)
            # cv2.putText(png,'FPS ' + str(fps),textOrg, fontFace, fontScale,(255,0,255),thickness)
            cv2.imshow("Depth", png)
            if counter % 2 == 0:
                print("WROTE.")
                cv2.imwrite(f"/home/semooww/PycharmProjects/DesignProject480/multirotor/images/{imgCounter}.png", png)
                imgCounter += 1
                counter += 1
        frameCount = frameCount + 1
        endTime = time.time()
        diff = endTime - startTime
        imuData=client.getImuData()
        print(imuData)
        if (diff > 1):
            counter += 1
            fps = frameCount
            frameCount = 0
            startTime = endTime

        key = cv2.waitKey(1) & 0xFF
        if (key == 27 or key == ord('q') or key == ord('x')):
            break


def movement():
    global controller
    for i in range(5):
        client_2.moveToPositionAsync(5, 5, altitude, 3).join()

        client_2.moveToPositionAsync(5, -5, altitude, 3).join()

        client_2.moveToPositionAsync(-5, -5, altitude, 3).join()

        client_2.moveToPositionAsync(-5, 5, altitude, 3).join()

    controller = 1


def parse_lidarData(data):
    # reshape array of floats to array of [X,Y,Z]
    points = np.array(data.point_cloud, dtype=np.dtype('f4'))
    points = np.reshape(points, (int(points.shape[0] / 3), 3))

    return points


x_axis = []
lidar = []
dist = []
gps = []


def getData():
    global controller
    counter = 1
    while True:
        if controller:
            break

        z_d = client_3.simGetVehiclePose().position.z_val
        lidarData = client_3.getLidarData()
        distanceSensorData = client_3.getDistanceSensorData()
        gpsData = client_3.getGpsData()
        if (len(lidarData.point_cloud) < 3):
            time.sleep(1)
            continue
        else:
            points = parse_lidarData(lidarData)
            print("\t\tlidar position: \n%s" % (pprint.pformat(lidarData.pose.position.z_val)))
            print("\t\tDS position: \n", distanceSensorData.distance)
            print("\t\tGPS position: \n", gpsData.gnss.geo_point.altitude - start_point_altitude)
            print("-------------------------------------------")
            print("\t\tlidar difference: \n", z_d - lidarData.pose.position.z_val)
            lidar.append(z_d - lidarData.pose.position.z_val)
            print("\t\tDS difference: \n", distanceSensorData.distance + z_d)
            dist.append(distanceSensorData.distance + z_d)
            print("\t\tGPS difference: \n", gpsData.gnss.geo_point.altitude - start_point_altitude + z_d)
            gps.append(gpsData.gnss.geo_point.altitude - start_point_altitude + z_d)
            x_axis.append(counter)
            print("*******************************************")
        time.sleep(1)
        counter += 1


t1 = Thread(target=camera)
t2 = Thread(target=movement)
t3 = Thread(target=getData)

# t1.start()
t2.start()
t3.start()

# t1.join()
t2.join()
t3.join()

client.reset()

plt.scatter(x_axis, lidar, color="red", label="lidar")
plt.scatter(x_axis, gps, color="blue", label="gps")
plt.scatter(x_axis, dist, color="green", label="distance ")
plt.legend(loc="upper left")
plt.show()
