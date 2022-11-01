try:
    import vrep
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')

from pydoc import cli
import time
import numpy as np
import yaml
import argparse
import math

number_of_robot = 4
total_input_path = []
with open("output.yaml", 'r') as output_file:
    data = yaml.load(output_file, Loader=yaml.FullLoader)


for n in range(1, number_of_robot+1):
    output_path = []
    for element in data['schedule']['AMR_LIFT'+str(n)]:
        x = float(element["x"])
        y = float(element["y"])
        t = float(element["t"])
        coordinates = [t, x, y]
        output_path.append(coordinates)
    total_input_path.append(output_path)

# print(total_input_path)
# total_input_path.append([[0.0, 0.0, 0.0], [0.5, 2.0, 0.0], [1.5, 2.0, 2.0], [3.0, 2.0, 0.0], [3.4, 0.0, 0.0]]) # [t, x, y]
# total_input_path.append ([[0.0, 0.0, 2.0], [0.5, 2.0, 2.0], [1.5, 2.0, 4.0], [3.0, 2.0, 2.0], [3.4, 0.0, 2.0]]) # [t, x, y]
# input_path_ = [[0.0, 0.0, 0.0], [0.5, 2.0, 0.0], [1.5, 2.0, 2.0], [3.0, 2.0, 0.0], [3.4, 0.0, 0.0]]
# input_path = [[0, 0, 0, 0], [0.4, 2, 0, 0], [0.6, 2, 0, 90], [1.6, 2, 2, 90], [2.0, 2, 2, -90], [4, 2, 0, -90], [8, 2, -2, -90]]

turn_ratio = 0.5
n = 0
total_final_path = []
total_robot_path = []
total_robot_orient = []
for n in range(number_of_robot):

    input_path = total_input_path[n]
    final_path = []
    print("input path is",input_path)
    # current_head = [1, 0]
    # if robot is standing without move, robot's heading is [1, 0]
    if len(input_path) == 1:
        current_head = [1, 0]
    else:
        current_head = [input_path[1][1] - input_path[0][1], input_path[1][2] - input_path[0][2]]
    current_angle = 0
    print("current head is ",current_head)

    for i in range(len(input_path)-1):
        current_node = input_path[i]
        next_node = input_path[i+1]
        next_head = [input_path[i+1][1] - input_path[i][1], input_path[i+1][2] - input_path[i][2]]
        if (np.linalg.norm(current_head)*np.linalg.norm(next_head)) == 0:
            angle = 0
        else:
            angle = math.acos(np.dot(current_head, next_head)/(np.linalg.norm(current_head)*np.linalg.norm(next_head)))*180/math.pi

        # print(angle)
        if abs(angle) < 0.1:
            current_node = [current_node[0], current_node[1], current_node[2], current_angle]
            final_path.append(current_node)
        else:
            current_t = input_path[i][0]
            current_node = [current_t, current_node[1], current_node[2], current_angle]
            final_path.append(current_node)
            current_angle += angle
            current_t = input_path[i+1][0] - (input_path[i+1][0] - input_path[i][0])*turn_ratio
            current_node = [current_t, current_node[1], current_node[2], current_angle]
            final_path.append(current_node)
            
        current_head = next_head
    final_node = [input_path[-1][0], input_path[-1][1], input_path[-1][2], current_angle]
    final_path.append(final_node)

    # print(final_path)