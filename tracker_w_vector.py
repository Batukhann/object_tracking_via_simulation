import math
import time
import numpy as np
import airsim
import threading
import cv2
from helper_functions import calc_velocity, calc_difs, calc_euler, calc_arrays, plot_all, bounding,calc_measurement
# connect to the AirSim simulator
from airsim import DrivetrainType

client = airsim.MultirotorClient()
client_2 = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
timer_controller = False
tracking_time = 140
initializer = True
distance_x = 0
distance_y = 0


def yaw_controller(distance):
    if -2.5 < distance < 2.5:
        return 0
    else:
        return distance


def altitude_controller(altitude):
    if altitude > 20.5:
        return 2
    elif altitude < 19.5:
        return -2
    else:
        return 0


def commander():
    global initializer
    global timer_controller
    global distance_x
    global distance_y
    print("Taking off...")
    client.armDisarm(True)
    client.takeoffAsync().join()
    time_arr = []
    altitudes = [[],[]]
    start_time = time.time()

    json_array_d = []  # gt
    json_array_v = []  # gt
    cv_array = []

    # initialization
    client.moveByVelocityBodyFrameAsync(0, 0, -3, 6, drivetrain=DrivetrainType.ForwardOnly).join()
    client.moveByVelocityBodyFrameAsync(2, 0, 0, 2, drivetrain=DrivetrainType.ForwardOnly).join()
    time.sleep(2)

    initializer = False
    while True:
        if timer_controller:
            break
        json_d = client.simGetVehiclePose('my_drone')
        json_v = client.simGetObjectPose("my_car")
        json_array_d.append(json_d)
        json_array_v.append(json_v)
        distance_data = client.getDistanceSensorData().distance
        altitudes[0].append(distance_data)
        altitudes[1].append(abs(json_d.position.z_val-json_v.position.z_val))
        cv_array.append([distance_x, distance_y, distance_data])

        distance_x = yaw_controller(distance_x)

        vx = -distance_x / 25
        vy = distance_y / 3.5 + 1.5
        vz = altitude_controller(distance_data)
        client.moveByVelocityBodyFrameAsync(vy, vx, vz, 0.1, drivetrain=DrivetrainType.ForwardOnly,
                                            yaw_mode=airsim.YawMode(False, 0)).join()

        current_time = time.time()
        time_arr.append(current_time - start_time)

    ################
    poses_drone, poses_vehicle, orientation_drone, orientation_vehicle = calc_arrays(json_array_v, json_array_d)
    ################
    pose_difs, orientation_difs = calc_difs(poses_drone, poses_vehicle, orientation_vehicle, orientation_drone)
    ################
    distance_gt,distance_cv= calc_measurement(pose_difs,cv_array)
    ################
    euler_drone, euler_vehicle = calc_euler(orientation_drone, orientation_vehicle)
    ################
    velocities_d, velocities_v = calc_velocity(poses_drone, poses_vehicle, time_arr)
    ################
    array = [poses_drone, poses_vehicle, orientation_drone, orientation_vehicle, pose_difs, orientation_difs,
             velocities_d, velocities_v, euler_drone, euler_vehicle, altitudes,distance_gt,distance_cv]
    plot_all(time_arr, array)
    ################


def image_reader():
    global initializer
    global timer_controller
    global distance_x
    global distance_y
    is_bounded = False
    tracker = cv2.TrackerCSRT_create()
    font_face = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.5
    thickness = 2
    text_size, baseline = cv2.getTextSize("FPS", font_face, font_scale, thickness)
    text_org = (10, 10 + text_size[1])
    frame_count = 0
    fps = 0
    start_time = time.time()
    while True:
        if initializer:
            continue
        if timer_controller:
            break
        json_d = client_2.simGetVehiclePose(vehicle_name='my_drone')
        json_v = client_2.simGetObjectPose("my_car")

        responses = client_2.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])
        response = responses[0]
        img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8)
        img_rgb = img1d.reshape(response.height, response.width, 3)
        png = cv2.cvtColor(img_rgb, cv2.COLOR_BGRA2BGR)
        ch, cw = 240, 320
        y_center, x_center = png.shape[0] // 2, png.shape[1] // 2

        if bounding(json_v, json_d) and is_bounded == False:
            is_bounded = True
            y, x = png.shape[0] // 2, png.shape[1] // 2
            bbox = (x - 20, y - 80, 40, 80)
            bounded = cv2.rectangle(png, (x - 25, y - 100), (x + 25, y - 5), (0, 0, 255), 2)
            print("Initialized the bounding box")
            tracker.init(png, bbox)
            print("Initialized the tracker")

        if is_bounded:
            ok, bbox = tracker.update(png)
            if ok:
                p1 = (int(bbox[0]), int(bbox[1]))
                p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                png = cv2.rectangle(png, p1, p2, (0, 0, 255), 2, 1)
                center = (int(bbox[0] + bbox[2] / 2), int(bbox[1] + bbox[3] / 2))
                cv2.line(png, (cw, ch), center, (255, 0, 255), 2)
                pixel_distances = x_center - (bbox[0] + bbox[2] / 2), y_center - (bbox[1] + bbox[3] / 2)
                distance_data = client_2.getDistanceSensorData().distance
                distance_x = distance_data * pixel_distances[0] / 315
                distance_y = max(distance_data * pixel_distances[1] / 315, 0)
        end_time = time.time()
        diff = end_time - start_time
        frame_count += 1
        if diff > 1:
            fps = frame_count
            frame_count = 0
            start_time = end_time
        cv2.putText(png, 'FPS ' + str(fps), text_org, font_face, font_scale, (255, 0, 255), thickness)
        cv2.line(png, (cw, 0), (cw, 480), (0, 200, 0), 2)
        cv2.line(png, (0, ch), (640, ch), (0, 200, 0), 2)
        cv2.imshow("Drone", png)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('x'):
            break


def timer():
    global tracking_time
    global timer_controller
    counter = 0
    while 1:
        if counter >= tracking_time:
            timer_controller = True
            print("Time finished")
            break
        counter += 1
        time.sleep(1)


t0 = threading.Thread(target=image_reader)
t1 = threading.Thread(target=commander)
t2 = threading.Thread(target=timer)

t0.start()
t1.start()
t2.start()

t0.join()
t1.join()
t2.join()

client.reset()
client.armDisarm(False)

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False)
