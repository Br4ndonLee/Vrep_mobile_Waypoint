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
from math import pi

vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to V-REP
if clientID!=-1:

    print ('Connected to remote API server')

    # 2.a: Handle the Pioneer:
    res, pioneerHandle = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx", vrep.simx_opmode_blocking)
    res, pioneerLeftMotor = vrep.simxGetObjectHandle(clientID, "left", vrep.simx_opmode_blocking)
    res, pioneerRightMotor = vrep.simxGetObjectHandle(clientID, "right", vrep.simx_opmode_blocking)
    res, pioneerOrientation = vrep.simxGetObjectOrientation(clientID, pioneerHandle, -1, vrep.simx_opmode_streaming)


    # Get the position of the Pioneer for the first time in streaming mode: Still have to understand why this is done!
    res , pioneerPosition = vrep.simxGetObjectPosition(clientID, pioneerHandle, 1 , vrep.simx_opmode_streaming)

    # Deactivate joint actuations:Make sure Pioneer is stationary:
    res = vrep.simxSetJointTargetVelocity(clientID, pioneerLeftMotor, 0, vrep.simx_opmode_streaming)
    res = vrep.simxSetJointTargetVelocity(clientID, pioneerRightMotor, 0, vrep.simx_opmode_streaming)


    ### Step 3: Start the Simulation: Keep printing out status messages!!!
    res = vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot_wait)

    
    if res == vrep.simx_return_ok:

        print ("---!!! Started Simulation !!! ---")
        
    
    # # Paht for robots
    # robotPath = []
    # output_time=[]
    # scale = 10

    # # Get coordinate from output.yaml
    # with open("output.yaml", 'r') as output_file:
    #     data = yaml.load(output_file, Loader=yaml.FullLoader)
    #     agent1 = data['agent1']    
    #     agent0 = data['agent0']
    
    # # set agent
    # for element in agent0:
    #     x = element['x'] * scale
    #     y = element['y'] * scale
    #     z = 0.15
    #     t = element['t']
    #     coordinates = [x, y, z]
    #     element_time = [t]
    #     output_time.append(element_time)
    #     robotPath.append(coordinates)

    # [t, x, y, theta]
    # input_path = [[0, 0, 0, 0],[1, 2, 0, 0], [2, 2, 0, 90], [3, 2, 2, 90], [4, 2, 2, -90], [5, 2, 0, -90], [6, 2, -2, -90],]
    input_path = [[0, 0, 0, 0],[0.4, 2, 0, 0], [0.6, 2, 0, 90], [1.6, 2, 2, 90], [2.0, 2, 2, -90], [4, 2, 0, -90], [8, 2, -2, -90]]

    z = 0.15
    dt = 0.05
    wheelRadius = 0.0975   # meters - From VREP model of left wheel
    trackWidth = 0.4       # meters - This is approximate

    robot_path = []
    robot_orient = []
    robot_time = []
    robot_path.append([input_path[0][1], input_path[0][2], z])
    for i in range(len(input_path)-1):
        
        t = input_path[i][0]
        t_prime = input_path[i+1][0]
        tdt = (t_prime - t)
        # robot_time.append(tdt)

        # get x coordinate & calculate difference with x and x+1
        x = input_path[i][1]
        x_prime = input_path[i+1][1]
        xdt = (x_prime - x)/tdt
        print("xdt :", xdt)

        # get y coordinate & calculate difference with y and y+1
        y = input_path[i][2]
        y_prime = input_path[i+1][2]
        ydt = (y_prime - y)/tdt
        print("ydt :", ydt)

        # get yaw coordinate & calculate difference with yaw and yaw+1
        yaw = input_path[i][3] * (pi/180)
        yaw_prime = input_path[i+1][3]* (pi/180)
        yawdt = (yaw_prime - yaw)/tdt
        print("yawdt :", yawdt)


        n = int(tdt/dt)
        # for a in range (n):
        #     x += xdt
        #     y += ydt
        #     yaw += yawdt

        #     coordinate = [x, y, z]
        #     robot_path.append(coordinate)
        #     direction = [0,0, yaw]
        #     robot_orient.append(direction)

        robot_coordinate = [x, y, z]
        robot_direction = [0, 0, yaw]
        # robot_time.append(dt)
        # robot_path.append(robot_coordinate)
        # robot_orient.append(robot_direction)
        if xdt == 0:
            omega = ydt
            if ydt < 0:
                omega = -ydt
            omega_right = (0.5*trackWidth*omega)/wheelRadius
            omega_left = (0.5*trackWidth*omega)/wheelRadius
            print(omega)

            if ydt == 0:
                omega = yawdt
                omega_left = -(0.5*trackWidth*omega)/wheelRadius
                omega_right = (0.5*trackWidth*omega)/wheelRadius
                print(omega)

        else:
            omega = xdt
            omega_right = (0.5*trackWidth*omega)/wheelRadius
            omega_left = (0.5*trackWidth*omega)/wheelRadius
            print(omega)

        res = vrep.simxSetJointTargetVelocity(clientID, pioneerLeftMotor, omega_left, vrep.simx_opmode_oneshot_wait)
        res = vrep.simxSetJointTargetVelocity(clientID, pioneerRightMotor, omega_right, vrep.simx_opmode_oneshot_wait)

        # left_vel = vrep.simxGetObjectVelocity(clientID, pioneerLeftMotor, vrep.simx_opmode_oneshot_wait)
        # right_vel = vrep.simxGetObjectVelocity(clientID, pioneerRightMotor, vrep.simx_opmode_oneshot_wait)

        # print(left_vel, right_vel)

        time.sleep(tdt)
        res = vrep.simxSetJointTargetVelocity(clientID, pioneerLeftMotor, 0, vrep.simx_opmode_oneshot_wait)
        res = vrep.simxSetJointTargetVelocity(clientID, pioneerRightMotor, 0, vrep.simx_opmode_oneshot_wait)
    
    # robot stop
    res = vrep.simxSetJointTargetVelocity(clientID, pioneerLeftMotor, 0, vrep.simx_opmode_oneshot_wait)
    res = vrep.simxSetJointTargetVelocity(clientID, pioneerRightMotor, 0, vrep.simx_opmode_oneshot_wait)

    # Stop simulation
    res = vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait)
    if res == vrep.simx_return_ok:
        print("---!!! Stop Simulation !!!---")            

