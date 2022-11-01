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


vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to V-REP
if clientID!=-1:

    print ('Connected to remote API server')

    # 2.a: Handle the Pioneer:
    res , pioneerHandle0 = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx#0", vrep.simx_opmode_blocking)
    res , pioneerHandle1 = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx#1", vrep.simx_opmode_blocking)
    res , pioneerHandle2 = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx#2", vrep.simx_opmode_blocking)
    res , pioneerHandle3 = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx#3", vrep.simx_opmode_blocking)
    
    pioneerHandle = [pioneerHandle0, pioneerHandle1, pioneerHandle2, pioneerHandle3]

    ### Step 3: Start the Simulation: Keep printing out status messages!!!
    
    res = vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot_wait)

    
    if res == vrep.simx_return_ok:

        print ("---!!! Started Simulation !!! ---")
        
    
    z = 0.15
    dt = 0.03
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
        # current_head = [1, 0]

        # if robot is standing without move, robot's heading is [1, 0]
        if len(input_path) == 1:
            current_head = [1, 0]
        else:
            current_head = [input_path[1][1] - input_path[0][1], input_path[1][2] - input_path[0][2]]
        current_angle = 0

        for i in range(len(input_path)-1):
            current_node = input_path[i]
            next_node = input_path[i+1]
            next_head = [input_path[i+1][1] - input_path[i][1], input_path[i+1][2] - input_path[i][2]]
            if (np.linalg.norm(current_head)*np.linalg.norm(next_head)) == 0:
                angle = 0
            else:
                angle = math.acos(np.dot(current_head, next_head)/(np.linalg.norm(current_head)*np.linalg.norm(next_head)))*180/math.pi

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

        total_final_path.append(final_path)

        robot_path = []
        robot_orient = []
        robot_time = []
        robot_path.append([final_path[0][1], final_path[0][2], z])
        robot_orient.append([0, 0, final_path[0][3]])
        for i in range(len(final_path)-1):
            
            t = final_path[i][0]
            t_prime = final_path[i+1][0]
            tdt = (t_prime - t)
            # robot_time.append(tdt)

            # get x coordinate & calculate difference with x and x+1
            x = final_path[i][1]
            x_prime = final_path[i+1][1]
            xdt = (x_prime - x)*dt/tdt

            # get y coordinate & calculate difference with y and y+1
            y = final_path[i][2]
            y_prime = final_path[i+1][2]
            ydt = (y_prime - y)*dt/tdt

            # get yaw coordinate & calculate difference with yaw and yaw+1
            yaw = final_path[i][3] * (math.pi/180)
            yaw_prime = final_path[i+1][3]* (math.pi/180)
            yawdt = (yaw_prime - yaw)*dt/tdt

            n = int(tdt/dt)
            for a in range (n):
                x += xdt
                y += ydt
                yaw += yawdt

                coordinate = [x, y, z]
                robot_path.append(coordinate)
                direction = [0,0, yaw]
                robot_orient.append(direction)

        total_robot_path.append(robot_path)
        total_robot_orient.append(robot_orient)

    length_of_each_path = []
    for i in range(number_of_robot):
        
        number = len(total_robot_path[i])
        length_of_each_path.append(number)
    
    maximum_path_length = max(length_of_each_path)
    print(maximum_path_length)

    for p in range(maximum_path_length):
        for n in range(number_of_robot):
            if len(total_robot_path[n]) <= p:
                pass
            else:
                vrep.simxSetObjectPosition(clientID, pioneerHandle[n], -1, total_robot_path[n][p], vrep.simx_opmode_oneshot)
                vrep.simxSetObjectOrientation(clientID, pioneerHandle[n], -1, total_robot_orient[n][p], vrep.simx_opmode_oneshot)

        time.sleep(dt)

    # Stop the simulation
    # res = vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait)
    # if res == vrep.simx_return_ok:
    #     print("Stop Simulation") 
