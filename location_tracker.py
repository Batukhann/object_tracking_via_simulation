import math
import matplotlib.pyplot as plt
import time
import airsim
import threading
import cv2

# connect to the AirSim simulator
from airsim import DrivetrainType

client_2 = airsim.MultirotorClient()
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
finish_controller = False
tracking_time = 1000
difarr = []
desiredV = []
velocityVehicle = []


def commander():
    global finish_controller
    print("Taking off...")
    client.armDisarm(True)
    client.takeoffAsync().join()
    counter = 0
    timearr = []
    start_time = time.time()
    previous_time = start_time

    json_array_d = []
    json_array_v = []

    while True:
        json_d = client.simGetVehiclePose(vehicle_name='my_drone')
        json_v = client.simGetObjectPose("my_car")
        json_array_d.append(json_d)
        json_array_v.append(json_v)

        x_v, y_v, z_v = json_v.position.x_val, json_v.position.y_val, json_v.position.z_val

        current_time = time.time()

        v_desired = calc_desired_velocity(json_array_v, json_array_d, counter, current_time, previous_time)

        timearr.append(current_time - start_time)

        client.moveToPositionAsync(x_v, y_v, z_v - 20, v_desired, timeout_sec=0.2,
                                   drivetrain=DrivetrainType.ForwardOnly,
                                   yaw_mode=airsim.YawMode(False, 0)).join()
        previous_time = current_time
        counter += 1

        if counter == tracking_time:
            break
    finish_controller = True
    ################
    poses_drone, poses_vehicle, orientation_drone, orientation_vehicle = calc_arrays(json_array_v, json_array_d)
    ################
    pose_difs, orientation_difs = calc_difs(poses_drone, poses_vehicle, orientation_vehicle, orientation_drone)
    ################
    euler_drone, euler_vehicle = calc_euler(orientation_drone, orientation_vehicle)
    ################
    velocities_d, velocities_v = calc_velocity(poses_drone, poses_vehicle, timearr)
    ################
    array = [poses_drone, poses_vehicle, orientation_drone, orientation_vehicle, pose_difs, orientation_difs,
             velocities_d, velocities_v, difarr, euler_drone, euler_vehicle, desiredV, velocityVehicle]
    # plot_all(timearr, array)
    ################


def calc_desired_velocity(json_arr_v, json_arr_d, counter, current_time, previous_time):
    global desiredV
    global velocityVehicle
    if counter == 0:
        return 0

    if current_time - previous_time == 0:
        current_time += 0.2
    x_d, y_d, z_d = json_arr_d[counter].position.x_val, json_arr_d[counter].position.y_val, json_arr_d[
        counter].position.z_val
    x_v, y_v, z_v = json_arr_v[counter].position.x_val, json_arr_v[counter].position.y_val, json_arr_v[
        counter].position.z_val
    x_v_prev, y_v_prev, z_v_prev = json_arr_v[counter - 1].position.x_val, json_arr_v[counter - 1].position.y_val, \
                                   json_arr_v[counter - 1].position.z_val

    velocity_v = (math.sqrt((x_v - x_v_prev) ** 2 + (y_v - y_v_prev) ** 2 + (z_v - z_v_prev) ** 2) / (current_time - previous_time))
    x_dif = math.sqrt((x_d - x_v) ** 2 + (y_d - y_v) ** 2 + (z_d - (z_v - 20)) ** 2)
    v_desired = velocity_v + (x_dif / ((current_time - previous_time) * 50))

    desiredV.append(v_desired)
    velocityVehicle.append(velocity_v)
    difarr.append(x_dif)
    return v_desired


def calc_arrays(json_arr_v, json_arr_d):
    poses_drone = [[], [], []]
    poses_vehicle = [[], [], []]
    orientation_drone = [[], [], [], []]
    orientation_vehicle = [[], [], [], []]
    for i in range(len(json_arr_v)):
        poses_drone[0].append(json_arr_d[i].position.x_val)
        poses_drone[1].append(json_arr_d[i].position.y_val)
        poses_drone[2].append(json_arr_d[i].position.z_val)
        ###########
        poses_vehicle[0].append(json_arr_v[i].position.x_val)
        poses_vehicle[1].append(json_arr_v[i].position.y_val)
        poses_vehicle[2].append(json_arr_v[i].position.z_val)
        ############
        orientation_drone[0].append(json_arr_d[i].orientation.w_val)
        orientation_drone[1].append(json_arr_d[i].orientation.x_val)
        orientation_drone[2].append(json_arr_d[i].orientation.y_val)
        orientation_drone[3].append(json_arr_d[i].orientation.z_val)
        ############
        orientation_vehicle[0].append(json_arr_v[i].orientation.w_val)
        orientation_vehicle[1].append(json_arr_v[i].orientation.x_val)
        orientation_vehicle[2].append(json_arr_v[i].orientation.y_val)
        orientation_vehicle[3].append(json_arr_v[i].orientation.z_val)
    return poses_drone, poses_vehicle, orientation_drone, orientation_vehicle


def calc_velocity(poses_drone, poses_vehicle, timearr):
    velocity_d = [[0.0], [0.0], [0.0]]
    velocity_v = [[0.0], [0.0], [0.0]]
    for i in range(len(poses_drone)):
        for j in range(1, len(poses_drone[i])):
            try:
                pose_dif = poses_drone[i][j] - poses_drone[i][j - 1]
                time_dif = timearr[j] - timearr[j - 1]
                velocity_d[i].append(pose_dif / time_dif)
                #########################################
                pose_dif = poses_vehicle[i][j] - poses_vehicle[i][j - 1]
                time_dif = timearr[j] - timearr[j - 1]
                velocity_v[i].append(pose_dif / time_dif)
            except ZeroDivisionError:
                pose_dif = poses_drone[i][j] - poses_drone[i][j - 1]
                velocity_d[i].append(pose_dif / 0.2)
                #########################################
                pose_dif = poses_vehicle[i][j] - poses_vehicle[i][j - 1]
                velocity_v[i].append(pose_dif / 0.2)
    return velocity_d, velocity_v


def calc_difs(poses_drone, poses_vehicle, orientation_vehicle, orientation_drone):
    orientation_difs = [[], [], [], []]
    pose_difs = [[], [], []]
    for i in range(len(poses_drone)):
        for j in range(len(poses_drone[i])):
            pose_difs[i].append(poses_drone[i][j] - poses_vehicle[i][j])

    for i in range(len(orientation_drone)):
        for j in range(len(orientation_drone[i])):
            orientation_difs[i].append(orientation_drone[i][j] - orientation_vehicle[i][j])
    return pose_difs, orientation_difs


def calc_euler(orientation_drone, orientation_vehicle):
    euler_drone = [[], [], []]
    euler_vehicle = [[], [], []]
    for i in range(len(orientation_drone[0])):
        euler_d = euler_from_quaternion(orientation_drone[1][i], orientation_drone[2][i], orientation_drone[3][i],
                                        orientation_drone[0][i])
        euler_v = euler_from_quaternion(orientation_vehicle[1][i], orientation_vehicle[2][i], orientation_vehicle[3][i],
                                        orientation_vehicle[0][i])
        euler_drone[0].append(euler_d[0])
        euler_drone[1].append(euler_d[1])
        euler_drone[2].append(euler_d[2])

        euler_vehicle[0].append(euler_v[0])
        euler_vehicle[1].append(euler_v[1])
        euler_vehicle[2].append(euler_v[2])
    return euler_drone, euler_vehicle


# array-> PosDrone,PosVehicle,OriDrone,OriVehicle,PosDiffs,OriDiffs,VelDrone,VelVehicle,dif
def plot_all(timearr, array):
    positions = ["X Positions", "Y Positions", "Z Positions"]
    orientations = ["X Orientation", "Y Orientation", "Z Orientation"]
    diffPos = ["Position-Differentiation X",
               "Position-Differentiation Y", "Position-Differentiation Z"]
    diffOri = ["Orientation-Differentiation X", "Orientation-Differentiation Y", "Orientation-Differentiation Z",
               "Orientation-Differentiation W"]
    velocity = ["X Velocity", "Y Velocity", "Z Velocity"]
    dif = ["Distance to Desired Position"]
    euler = ["X Euler", "Y Euler", "Z Euler"]
    vel = ["Velocity"]
    titleArray = [positions, orientations, diffPos, diffOri, velocity, dif, euler, vel]

    for i in range(len(titleArray)):
        for j in range(len(titleArray[i])):
            if i == 0:
                path = r"C:\Users\ayvaz\Desktop\Resources\dataset\\" + titleArray[i][j] + ".jpg"
                plot2(timearr, array[0][j], array[1][j], titleArray[i][j], "Position", "Time(s)", path)
            elif i == 1:
                path = r"C:\Users\ayvaz\Desktop\Resources\dataset\\" + titleArray[i][j] + ".jpg"
                plot2(timearr, array[2][j], array[3][j], titleArray[i][j], "Orientation", "Time(s)", path)
            elif i == 2:
                path = r"C:\Users\ayvaz\Desktop\Resources\dataset\\" + titleArray[i][j] + ".jpg"
                plot_img(timearr, array[4][j], titleArray[i][j], "Differentiaton", "Time(s)", path)
            elif i == 3:
                path = r"C:\Users\ayvaz\Desktop\Resources\dataset\\" + titleArray[i][j] + ".jpg"
                plot_img(timearr, array[5][j], titleArray[i][j], "Orientation", "Time(s)", path)
            elif i == 4:
                path = r"C:\Users\ayvaz\Desktop\Resources\dataset\\" + titleArray[i][j] + ".jpg"
                plot2(timearr, array[6][j], array[7][j], titleArray[i][j], "Velocity(m/s)", "Time(s)", path)
            elif i == 5:
                path = r"C:\Users\ayvaz\Desktop\Resources\dataset\\" + titleArray[i][j] + ".jpg"
                plot_img(timearr[1::], array[8], titleArray[i][j], "Difference", "Time(s)", path)
            elif i == 6:
                path = r"C:\Users\ayvaz\Desktop\Resources\dataset\\" + titleArray[i][j] + ".jpg"
                plot2(timearr, array[9][j], array[10][j], titleArray[i][j], "Euler", "Time(s)", path)
            elif i == 7:
                path = r"C:\Users\ayvaz\Desktop\Resources\dataset\\" + titleArray[i][j] + ".jpg"
                plot2(timearr[1::], array[11], array[12], titleArray[i][j], "Velocity", "Time(s)", path)


def plot_img(array1, array2, title, y_label, x_label, path):
    plt.plot(array1, array2, color='r')
    plt.title(title)
    plt.ylabel(y_label)
    plt.xlabel(x_label)
    plt.savefig(path)
    plt.clf()


def plot2(array1, array2, array3, title, y_label, x_label, path):
    plt.plot(array1, array2, color='r', label='Drone')
    plt.plot(array1, array3, color='g', label='Vehicle')
    plt.legend(loc="upper right")
    plt.title(title)
    plt.ylabel(y_label)
    plt.xlabel(x_label)
    plt.savefig(path)
    plt.clf()


def image_reader():
    global finish_controller
    image_counter = 0
    while True:
        if finish_controller:
            break
        camera_position = client_2.simGetObjectPose("my_car")
        camera_position.position.x_val += 1
        camera_position.position.z_val -= 2
        client_2.simSetCameraPose("external_camera", camera_position, external=True)
        rawImage = client_2.simGetImage("external_camera", airsim.ImageType.Scene, external=True)
        png = cv2.imdecode(airsim.string_to_uint8_array(rawImage), cv2.IMREAD_UNCHANGED)
        filename = r"C:\Users\ayvaz\PycharmProjects\DesignProject\dataset\car_dataset\%d.jpg" % image_counter
        cv2.imwrite(filename, png)

        rawImage = client_2.simGetImage("0", airsim.ImageType.Scene)
        png = cv2.imdecode(airsim.string_to_uint8_array(rawImage), cv2.IMREAD_UNCHANGED)
        filename = r"C:\Users\ayvaz\PycharmProjects\DesignProject\dataset\drone_dataset\%d.jpg" % image_counter
        cv2.imwrite(filename, png)
        image_counter += 1

        cv2.imshow("Drone", png)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('x'):
            break


def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return [roll_x, pitch_y, yaw_z]  # in radians


t0 = threading.Thread(target=image_reader)
t1 = threading.Thread(target=commander)

t0.start()
t1.start()

t0.join()
t1.join()

client.reset()
client.armDisarm(False)

# that's enough fun for now. let's quit cleanly
client.enableApiControl(False)
