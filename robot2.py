
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

import time
import numpy as np
import params
import yaml
import argparse
from   utilities import *
from math import sqrt

global angErrHist
global distErrHist

angErrHist = []
distErrHist = []

print ('Program started')



# 3. Routine to get the Distance and angular errors for the go-to-goal PID controller:
def g2g_xy_errors(clientID, pioneerHandle, goal):

    # a: Stream current position and orientation:
    res , pioneerPosition = vrep.simxGetObjectPosition(clientID, pioneerHandle, -1 , vrep.simx_opmode_buffer)
    res , pioneerOrientation = vrep.simxGetObjectOrientation(clientID, pioneerHandle, -1 , vrep.simx_opmode_buffer)

    xp = pioneerPosition[0]
    yp = pioneerPosition[1]
    thetap = pioneerOrientation[2]



    # # VREP orientation returns a value between -pi to +pi. Trnasform this to the range o to 2*pi
    # if thetap < 0:
    #
    #     thetap = 2*np.pi + thetap
    #
    # print ("Thetap -> " , thetap)

    xg = goal[0]
    yg = goal[1]

    diff_x = xg - xp
    diff_y = yg - yp
 
    # b. Compute orientation error:
    targetOrientation = np.arctan2( diff_y , diff_x)


    # # Arctan2 returns a value between -pi to +pi. Trnasform this to the range o to 2*pi
    # if targetOrientation < 0:
    #
    #     targetOrientation = 2*np.pi + targetOrientation
    #
    # print ("Target -> " , thetap)

    errAng = targetOrientation - thetap


    # Restrict to -pi and +pi:
    errAng = np.mod((errAng+ np.pi), 2*np.pi) - np.pi


    if abs(errAng) < params.angErrThresh:

        reachedOrientation = True
    else:
        reachedOrientation = False

    # c. Compute distance error:
    errDist = np.sqrt( (xg - xp)**2 + (yg - yp)**2)

    if errDist < params.distErrThresh:

        reachedGoal = True
    else:

        reachedGoal = False


    # Return the values:
    return errAng, errDist, reachedOrientation , reachedGoal



# 4. Function to map unicycle commands to differential drive commands:
def diff_drive_map(v,omega, trackWidth,wheelRadius):

    # TO DO: Document the formulas in detail here:

    # limit the meximun velocity
    meximumVelocity = 6

    # 1. Left wheel angular velocity:
    omegaLeft = (v + 0.5*trackWidth*omega)/wheelRadius
    if omegaLeft <= meximumVelocity:
        omegaLeft = (v + 0.5*trackWidth*omega)/wheelRadius
        # omegaLeft = sqrt(x**2+y**2)/t
    else:
        omegaLeft = meximumVelocity

    # 2. Right wheel angular velocity:
    omegaRight = (v - 0.5*trackWidth*omega)/wheelRadius
    if omegaRight <= meximumVelocity:
        omegaRight = (v - 0.5*trackWidth*omega)/wheelRadius
        # omegaRight = sqrt(x**2+y**2)/t
    else:
        omegaRight = meximumVelocity

    return omegaLeft , omegaRight


# 5. Go to Goal PID control from a 2-D differential drive robot:
def g2g_controller_PID(clientID, pioneerHandle, pioneerLeftMotorHandle, pioneerRightMotorHandle, goal):

    global angErrHist , distErrHist

    # Set the sleep time in While function
    sleepTime = 0.2

    # a. Setup internal parameters of the PID controller:
    reachedGoal = False
    reachedOrientation = False
    wheelRadius = 0.0975   # meters - From VREP model of left wheel
    trackWidth = 0.4       # meters - This is approximate
    v= 0
    omega = 0

    timer = 0

    # Continue this as long as the Robot does not reach its goal:
    while not reachedGoal:

        errAng, errDist, reachedOrientation , reachedGoal = g2g_xy_errors(clientID, pioneerHandle, goal)


        # print (" Time = " , timer , " sec" , end= " -> ")
        # print ("Angular Error =  " , errAng, end= " -> ")
        # print (" Distance Error = " , errDist)

        # Append error values to the cache variabes:
        angErrHist.append(errAng)
        distErrHist.append(errDist)

        # Orientation correction:
        if not reachedOrientation:

            # PID control law for Orientation:
            v = 0

            if len(angErrHist) > params.integralWindow:
                omega = (-params.kpTheta*errAng - params.kdTheta*(errAng - angErrHist[-2]) - params.kiTheta*sum(angErrHist[-params.integralWindow:]))

            elif len(angErrHist) > 2:

                omega = (-params.kpTheta*errAng - params.kdTheta*(errAng - angErrHist[-2]) - params.kiTheta*sum(angErrHist))

            else:

                omega =  (-params.kpTheta*errAng)


            omegaLeft , omegaRight  = diff_drive_map(v,omega,trackWidth,wheelRadius)

            res = vrep.simxSetJointTargetVelocity(clientID, pioneerLeftMotorHandle, omegaLeft, vrep.simx_opmode_oneshot_wait)
            res = vrep.simxSetJointTargetVelocity(clientID, pioneerRightMotorHandle, omegaRight, vrep.simx_opmode_oneshot_wait)

        # Distance correction:
        else:

            # Clear the angular error history cache:
            angErrHist = []

            omega = 0.0

            if  len(distErrHist) > params.integralWindow:
                v = 8*(params.kpDist*errDist + params.kdDist*(errDist - distErrHist[-2]) + params.kiDist*sum(distErrHist[-params.integralWindow:]))

            elif len(distErrHist) > 2:

                v = 8*(params.kpDist*errDist + params.kdDist*(errDist - distErrHist[-2]) + params.kiDist*sum(distErrHist))

            else:
                v = 8*(params.kpDist*errDist)


            omegaLeft , omegaRight  = diff_drive_map(v,omega,trackWidth, wheelRadius)

            res = vrep.simxSetJointTargetVelocity(clientID, pioneerLeftMotorHandle, omegaLeft, vrep.simx_opmode_oneshot_wait)
            res = vrep.simxSetJointTargetVelocity(clientID, pioneerRightMotorHandle, omegaRight, vrep.simx_opmode_oneshot_wait)


        # Set the frequency of the controller:
        time.sleep(sleepTime)


    # Notify the user:
    #print(" Robot reached its Goal!")

    # Clear the error history caches:
    angErrHist = []
    distErrHist = []

    v= v/2
    omega = 0

    omegaLeft , omegaRight  = diff_drive_map(v,omega,trackWidth,wheelRadius)

    # Deactivate joint actuations:
    res = vrep.simxSetJointTargetVelocity(clientID, pioneerLeftMotorHandle, omegaLeft, vrep.simx_opmode_oneshot_wait)
    res = vrep.simxSetJointTargetVelocity(clientID, pioneerRightMotorHandle, omegaRight, vrep.simx_opmode_oneshot_wait)


    return reachedGoal




# ---------------------------------- MAIN SCRIPT BEGINS HERE -----------------------------------------------------------------------------#

vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to V-REP
if clientID!=-1:

    print ('Connected to remote API server')


    # 2.a: Handle the Pioneer:
    res , pioneerHandle = vrep.simxGetObjectHandle(clientID, "Pioneer_p3dx", vrep.simx_opmode_blocking)
    res,  pioneerLeftMotorHandle = vrep.simxGetObjectHandle(clientID, "left", vrep.simx_opmode_blocking)
    res,  pioneerRightMotorHandle = vrep.simxGetObjectHandle(clientID, "right", vrep.simx_opmode_blocking)

    # Get the position of the Pioneer for the first time in streaming mode: Still have to understand why this is done!
    res , pioneerPosition = vrep.simxGetObjectPosition(clientID, pioneerHandle, -1 , vrep.simx_opmode_streaming)
    res , pioneerOrientation = vrep.simxGetObjectOrientation(clientID, pioneerHandle, -1 , vrep.simx_opmode_streaming)

    # Deactivate joint actuations:Make sure Pioneer is stationary:
    res = vrep.simxSetJointTargetVelocity(clientID, pioneerLeftMotorHandle, 0, vrep.simx_opmode_streaming)
    res = vrep.simxSetJointTargetVelocity(clientID, pioneerRightMotorHandle, 0, vrep.simx_opmode_streaming)



    ### Step 3: Start the Simulation: Keep printing out status messages!!!
    res = vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot_wait)

    
    if res == vrep.simx_return_ok:

        print ("---!!! Started Simulation !!! ---")



    ### Step 3: Perception:  TO BE DONE:

    # # First call:
    # res,resolution,image=vrep.simxGetVisionSensorImage(clientID, quadCameraHandle,0,vrep.simx_opmode_streaming)
    # print ('firts call',res)
    # time.sleep(5)
    # res,resolution,image=vrep.simxGetVisionSensorImage(clientID, quadCameraHandle,0,vrep.simx_opmode_buffer)
    # print ('second call',res)
    #
    # if res == vrep.simx_return_ok:
    #
    #     print (" successful first image!")
    #
    # time.sleep(5)
    #
    # while (vrep.simxGetConnectionId(clientID) != -1):
    #
    #      res, resolution, image = vrep.simxGetVisionSensorImage(clientID, quadCameraHandle, 0, vrep.simx_opmode_buffer)
    #
    #      print (res)
    #
    #      if res == vrep.simx_return_ok:
    #          print (len(image))

    # # 3. Setup for vectorization:
    # setup_vec()

    # 4. Run the RRT algorithm and directly plot the path:
    startTime = time.time()
    # RRT_path = RRT(start, goal)
    endTime = time.time()

    # Paht for robots
    revPath = []
    output_time=[]
    scale = 1

    # 5. Get the true path from start to goal:
    with open("output.yaml", 'r') as output_file:
        data = yaml.load(output_file, Loader=yaml.FullLoader)
        agent1 = data['agent1']
        agent0 = data['agent0']
    
    # set agent
    for element in agent0:
        x = element['x'] * scale
        y = element['y'] * scale
        t = element['t']
        coordinates = [x, y]
        element_time = [t]
        output_time.append(element_time)
        revPath.append(coordinates)
  
    # revPath = reverse_path(RRT_path)
    # revPath = [[10.0,0.0],[10.0, -10.0],[6,-4],[8,7],[-4,-4.0],[0,0]]

    timeRRT= endTime - startTime

    print (" Time taken time is : " , timeRRT ,  "secs")
    print (" The RRT path is : ")
    print (revPath)
    # ----------------------------------------------------- #


    # Start a timer:
    sTime = time.time()

    pathPointCount = 0

    # Loop to traverse RRT Path:
    for immGoal in revPath:

        # Run the Go-to-goal control function:
        reached = g2g_controller_PID(clientID, pioneerHandle, pioneerLeftMotorHandle, pioneerRightMotorHandle, immGoal)

        if reached:

            # Increment the path point count:
            pathPointCount +=1

            print (" Reached node # " , pathPointCount )
            print (" ---------------------------------")

        else:

            print(" There was an ERROR! Please investigate!!!")


    eTime = time.time()

    travelTime = eTime - sTime

    # Display successful path planning message!:
    print ("--- Robot has Successfully reached its final Goal --- ")
    print (" The toal time taken is : " , travelTime , " secs")
    print ("The robot position is : ", pioneerPosition)

    # Set the joint velocities back to zero:
    res = vrep.simxSetJointTargetVelocity(clientID, pioneerLeftMotorHandle, 0, vrep.simx_opmode_streaming)
    res = vrep.simxSetJointTargetVelocity(clientID, pioneerRightMotorHandle, 0, vrep.simx_opmode_streaming)


    # Stop the simulation:
    res = vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot_wait)

    if res == vrep.simx_return_ok:

        print (" --- !!!Stopped Simulation!!! ---")


    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)

else:

    print ('Failed connecting to remote API server')



print (' --- Python Remote API Program ended ---')
