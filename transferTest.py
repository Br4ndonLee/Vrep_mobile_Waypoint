
from pydoc import cli
import time
import numpy as np
import yaml
import argparse
import math



# Paht for robots
output_path = []
output_time=[]
radian = []
degree = []
radian_list = []
degree_list = []
z = 0.15
dt = 0.05
scale = 1

# Get coordinate from output.yaml
with open("output.yaml", 'r') as output_file:
    data = yaml.load(output_file, Loader=yaml.FullLoader)
    agent1 = data['agent1']    
    agent0 = data['agent0']

# set agent
# for element in agent0:
for element in agent1:
    x = element['x'] * scale
    y = element['y'] * scale
    z = 0.15
    t = element['t']
    coordinates = [x, y, z]
    element_time = t
    output_time.append(element_time)
    output_path.append(coordinates)
print(output_time)

for i in range(len(output_path)-1):
    radian = math.atan2(output_path[i+1][1]-output_path[i][1], output_path[i+1][0]-output_path[i][0])
    degree = math.degrees(radian)
    radian_list.append(radian)
    degree_list.append(degree)

for j in range(len(output_path)-1):
    output_path[j].append(degree_list[j])
output_path[-1].append(degree_list[-1])
# output_path[len(output_path)-1].append()
    

# [t, x, y, theta]
# input_path = [[0, 0, 0, 0],[1, 2, 0, 0], [2, 2, 0, 90], [3, 2, 2, 90], [4, 2, 2, -90], [5, 2, 0, -90], [6, 2, -2, -90],]
# input_path = [[0, 0, 0, 0],[0.4, 2, 0, 0], [0.6, 2, 0, 90], [1.6, 2, 2, 90], [2.0, 2, 2, -90], [4, 2, 0, -90], [8, 2, -2, -90]]

print(output_path)

# robot_path = [x, y, z, yaw]
robot_path = []
robot_orient = []
robot_time = []

radian_to_degree =[]

# robot_path.append([input_path[0][1], input_path[0][2], z])
for i in range(len(output_path)-1):
    
    t = output_time[i]
    t_prime = output_time[i+1]
    tdt = (t_prime - t)
    
    # robot_time.append(tdt)

    # get x coordinate & calculate difference with x and x+1
    x = output_path[i][0]
    x_prime = output_path[i+1][0]
    xdt = (x_prime - x)*dt/tdt

    # get y coordinate & calculate difference with y and y+1
    y = output_path[i][1]
    y_prime = output_path[i+1][1]
    ydt = (y_prime - y)*dt/tdt

    # get yaw coordinate & calculate difference with yaw and yaw+1
    yaw = math.radians(output_path[i][3])
    yaw_prime = math.radians(output_path[i+1][3])
    yawdt = (yaw_prime - yaw)*dt/tdt


    n = int(tdt/dt)
    for a in range (n):
        x += xdt
        y += ydt
        yaw += yawdt

        coordinate = [x, y, z]
        robot_path.append(coordinate)
        # yaw = np.mod((yaw+np.pi), 2*np.pi)-np.pi
        direction = [0,0, yaw]
        robot_orient.append(direction)
        radian_to_degree.append(math.degrees(yaw))
        # robot_time.append(dt)
# print(robot_path)
    # robot_coordinate = [x, y, z]
    # robot_direction = [0, 0, yaw]
    # robot_time.append(dt)
    # robot_path.append(robot_coordinate)
    # robot_orient.append(robot_direction)


print("##################################")
print(radian_to_degree)
# print(robot_path)
# print("##################################")
# print(robot_orient)
