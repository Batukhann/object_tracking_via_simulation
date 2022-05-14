import math
import matplotlib.pyplot as plt
import numpy as np


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


def calc_measurement(pose_difs, cv_array):
    distance_gt = []
    distance_cv = []
    for i in range(len(cv_array)):
        distance_gt.append(np.sqrt(pose_difs[0][i] ** 2 + pose_difs[1][i] ** 2))
        distance_cv.append(np.sqrt(cv_array[i][0] ** 2 + cv_array[i][1] ** 2))
    return distance_gt, distance_cv


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


# array-> PosDrone,PosVehicle,OriDrone,OriVehicle,PosDiffs,OriDiffs,VelDrone,VelVehicle,EulerDrone,EulerVehicle
def plot_all(timearr, array):
    positions = ["X Positions", "Y Positions", "Z Positions"]
    orientations = ["X Orientation", "Y Orientation", "Z Orientation"]
    diffPos = ["Position-Differentiation X",
               "Position-Differentiation Y", "Position-Differentiation Z"]
    diffOri = ["Orientation-Differentiation X", "Orientation-Differentiation Y", "Orientation-Differentiation Z",
               "Orientation-Differentiation W"]
    velocity = ["X Velocity", "Y Velocity", "Z Velocity"]
    euler = ["X Euler", "Y Euler", "Z Euler"]
    altitude = ["Altitude Data from Distance Sensor"]
    distance = ["Distance"]
    titleArray = [positions, orientations, diffPos, diffOri, velocity, euler, altitude, distance]
    path_text = 'C:/Users/ayvaz/PycharmProjects/DesignProject/dataset/graph/'
    for i in range(len(titleArray)):
        for j in range(len(titleArray[i])):
            if i == 0:
                path = path_text + titleArray[i][j] + ".jpg"
                plot2(timearr, array[0][j], array[1][j], titleArray[i][j], "Position", "Time(s)", path)
            elif i == 1:
                path = path_text + titleArray[i][j] + ".jpg"
                plot2(timearr, array[2][j], array[3][j], titleArray[i][j], "Orientation", "Time(s)", path)
            elif i == 2:
                path = path_text + titleArray[i][j] + ".jpg"
                plot_img(timearr, array[4][j], titleArray[i][j], "Position Differentiation", "Time(s)", path)
            elif i == 3:
                path = path_text + titleArray[i][j] + ".jpg"
                plot_img(timearr, array[5][j], titleArray[i][j], "Orientation Differentiation", "Time(s)", path)
            elif i == 4:
                path = path_text + titleArray[i][j] + ".jpg"
                plot2(timearr, array[6][j], array[7][j], titleArray[i][j], "Velocity(m/s)", "Time(s)", path)
            elif i == 5:
                path = path_text + titleArray[i][j] + ".jpg"
                plot2(timearr, array[8][j], array[9][j], titleArray[i][j], "Euler", "Time(s)", path)
            elif i == 6:
                path = path_text + titleArray[i][j] + ".jpg"
                plot2(timearr, array[10][0], array[10][1], titleArray[i][j], "Altitude", "Time(s)", path,
                      "Distance Sensor", "Groundtruth")
            elif i == 7:
                path = path_text + titleArray[i][j] + ".jpg"
                plot2(timearr, array[11], array[12], titleArray[i][j], "Distance", "Time(s)", path, "Real Distance",
                      "Camera Distance")


def plot_img(array1, array2, title, y_label, x_label, path):
    plt.plot(array1, array2, color='r')
    plt.title(title)
    plt.ylabel(y_label)
    plt.xlabel(x_label)
    plt.savefig(path)
    plt.clf()


def plot2(array1, array2, array3, title, y_label, x_label, path, label1="Drone", label2="Vehicle"):
    plt.plot(array1, array2, color='r', label=label1)
    plt.plot(array1, array3, color='g', label=label2)
    plt.legend(loc="best")
    plt.title(title)
    plt.ylabel(y_label)
    plt.xlabel(x_label)
    plt.savefig(path)
    plt.clf()


def bounding(json_v, json_d, threshold=4):
    x_d, y_d, z_d = json_d.position.x_val, json_d.position.y_val, json_d.position.z_val
    x_v, y_v, z_v = json_v.position.x_val, json_v.position.y_val, json_v.position.z_val

    if (np.absolute(x_v - x_d) < threshold) and (np.absolute(y_v - y_d) < threshold):
        return True
    else:
        return False
