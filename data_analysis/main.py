import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

hololens_data = "C:\dev\SpotDockerImage\data\experiments\P900\gestures_stationary_hololens.csv"
odometry_data = "C:\dev\SpotDockerImage\data\experiments\P900\gestures_stationary_odom.csv"

start_position_person = [0.0, 0.0, 0.0]
start_position_robot = [0.0, -1.0, 0.0]

df_holo = pd.read_csv(hololens_data)
df_odom = pd.read_csv(odometry_data)

df_holo.rename(columns={
    "camera position [x,y,z]": "position",
    "camera orientation [w,x,y,z]": "orientation",
}, inplace=True)

df_odom.rename(columns={
    "position_odom_spot [x,y,z]": "position",
    "orientation_odom_spot [yaw,pitch,roll]": "orientation",
}, inplace=True)

def calibrate_starting_position_quaternion(truth, data):
    data["position"] = data["position"].apply(lambda x: list(eval(x)))
    x_cor = truth[0] - data["position"].iloc[0][0]
    y_cor = truth[1] - data["position"].iloc[0][1]
    z_cor = truth[2] - data["position"].iloc[0][2]
    data["position"] = data["position"].apply(lambda x: [x[0] - x_cor,  x[1] - y_cor,  x[2] - z_cor])
    return data

def calibrate_starting_position_euler(truth, data):
    data["position"] = data["position"].apply(lambda x: list(eval(x)))
    x_cor = truth[0] - data["position"].iloc[0][0]
    y_cor = truth[1] - data["position"].iloc[0][1]
    z_cor = truth[2] - data["position"].iloc[0][2]
    data["position"] = data["position"].apply(lambda x: [x[0] - x_cor,  x[1] - y_cor,  x[2] - z_cor])
    return data


df_holo_cali =  calibrate_starting_position_quaternion(start_position_person, df_holo)
df_odom_cali =  calibrate_starting_position_euler(start_position_person, df_odom)

positions_holo = df_holo_cali["position"].values
positions_odom = df_odom_cali["position"].values

positions_holo = np.array([np.array(x) for x in positions_holo])
positions_odom = np.array([np.array(x) for x in positions_odom])

fig = plt.figure() 

plt.scatter(positions_holo[:, 0], positions_holo[:, 1], c="blue")
plt.scatter(positions_odom[:, 0], positions_odom[:, 1], c="red")

plt.show()