# -*- coding: utf-8 -*-

# Multiple Footnote Command is "Ctrl+1" in SPYDER / "Ctrl + /" in VSCode

import vrep
import sys
import time
import numpy as np
import math
import params
import random
from utilities import *

global angErrHist
global distErrHist

angErrHist = []
distErrHist = []

print ('Program started')

# 3. Routine to get the Distance and angular errors for the go-to-goal PID controller:
def g2g_xy_errors(clientID, pioneerHandle, goal):

    # Stream current position and orientation:
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

    # Compute orientation error
    targetOrientation = np.arctan2(yg - yp, xg - xp)

    # # Arctan2 returns a value between -pi to +pi. Trnasform this to the range o to 2*pi
    # if targetOrientation < 0:
    #
    #     targetOrientation = 2*np.pi + targetOrientation
    #
    # print ("Target -> " , thetap)

    errAng = targetOrientation - thetap

    #Restrict to -pi ~ +pi
    errAng = np.mod((errAng + np.pi), 2*np.pi) - np.piecewise

    if abs(errAng) < params.angErrThresh:
        reachedOrientation = True
    else:
        reachedOrientation = False

    # Compute distance error
    errDist = np.sqrt((xg - xp)**2 + (yg - yp)**2)

    if errDist < params.distErrThresh:
        reachedGoal = True
    else:
        reachedGoal = False
    
    # Return the values
    return errAng, errDist, reachedOrientation, reachedGoal

# 4. Function to map unicycle commands to differential drive commands
def diff_drive_map(v, omega, trackWidth, wheelRadius):

    # TO DO : Document the formulas in detail here

    # 1. Left & Right wheel angular velocity 
    omegaLeft = (v + 0.5*trackWidth*omega)/wheelRadius
    omegaRight = (v + 0.5*trackWidth*omega)/wheelRadius

    return omegaLeft, omegaRight

# 5. Go to Goal PID control from a 2-D differential drive robot
def g2g_controller_PID(clientID, pioneerHandle, pioneerLeftMotorHandle, pioneerRightMotorHandle, goal):
    global angErrHist, distErrHist

    # Setup internal parameters of the PID controller
    reachedGoal = False
    reachedOrientation = False
    wheelRadius = 0.0975            # meters - from VREP model of left wheel
    trackWidth = 0.4                # meters - This is approximate
    v = 0.5
    omega = 0

    timer = 0

    # Continue this as long as the Robot does not reach its goal
    while not reachedGoal:
        errAng, errDist, reachedOrientation, reachedGoal = g2g_xy_errors(clientID, pioneerHandle, goal)

        # print (" Time = " , timer , " sec" , end= " -> ")
        # print ("Angular Error =  " , errAng, end= " -> ")
        # print (" Distance Error = " , errDist)

        # Append error values to the cache variabes
        angErrHist.append(errAng)
        distErrHist.append(errDist)

        # Orientation correction
        if not reachedOrientation:

            # PID control law for Orientation
            v = 0

            if len(angErrHist) > params.integralWindow:
                omega = -params.kpTheta*errAng - params.kpTheta*(errAng - angErrHist[-2]) - params.kiTheta*sum(angErrHist[-params.integralWindow:])

            elif len(angErrHist) > 2:
                omega = -params.kpTheta*errAng - params.kdTheta*(errAng - angErrHist[-2]) - params.kiTheta*sum(angErrHist)
            
            else:
                omega = -params.kpThepa*errAng
            
            omegaLeft, omegaRight = diff_drive_map(v, omega, trackWidth, wheelRadius)

            res = vrep.simxSetJointTargetVelocity(clientID, pioneerLeftMotorHandle, omegaLeft, vrep.simx_opmode_oneshot_wait)
            res = vrep.simxSetJointTargetVelocity(clientID, pioneerRightMotorHandle, omegaRight, vrep.simx_opmode_oneshot_wait)
        
        # Distance correction
        else:

            # Clear the angular error history cache
            angErrHist = []

            omega = 0.0
            
            if len(distErrHist) > params.integralWindow:
                v = params.kpDist*errDist + params.kdDist*(errDist - distErrHist[-2]) + params.kiDist*sum(distErrHist[-params.integralWindow:])

            elif len(distErrHist) > 2:
                v = params.kpDist*errDist + params.kdDist*(errDist - distErrHist[-2]) + params.kiDist*sum(distErrHist)
            
            else:
                v = params.kpDist*errDist
            
            omegaLeft, omegaRight = diff_drive_map(v, omega, trackWidth, wheelRadius)

            res = vrep.simxSetJointTargetVelocity(clientID, pioneerLeftMotorHandle, omegaLeft, vrep.simx_opmode_oneshot_wait)
            res = vrep.simxSetJointTargetVelocity(clientID, pioneerRightMotorHandle, omegaRight, vrep.simx_opmode_oneshot_wait)

        # Set the frequency of the controller
        time.sleep(0.5)

    # Clear the error history
    angErrHist = []
    distErrHist = []

    v = v/2
    omega = 0

    omegaLeft, omegaRight = diff_drive_map(v, omega, trackWidth, wheelRadius)

    # Deactivate joint actuations:
    res = vrep.simxSetJointTargetVelocity(clientID, pioneerLeftMotorHandle, omegaLeft, vrep.simx_opmode_oneshot_wait)
    res = vrep.simxSetJointTargetVelocity(clientID, pioneerRightMotorHandle, omegaRight, vrep.simx_opmode_oneshot_wait)

    return reachedGoal




# ---------------------------------- MAIN SCRIPT BEGINS HERE -----------------------------------------------------------------------------#    

vrep.simxFinish(-1)
clientID=vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

if clientID!=(-1):
    print ('Connected to remote API server')

    # Handle the Pioneer
    res, pioneerHandle = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx', vrep.simx_opmode_blocking)    
    res, pioneerLeftMotorHandle=vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_oneshot_wait)
    res, pioneerRightMotorHandle=vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_oneshot_wait)

    # Get the position of the Pioneer for the first time in streaming mode: Still have to understand why this is done
    res, pioneerPosition = vrep.simxGetObjectPosition(clientID, pioneerHandle, -1, vrep.simx_opmode_streaming)
    res, pioneerOrientation = vrep.simxGetObjectOrientation(clientID, pioneerHandle, -1, vrep.simx_opmode_streaming)

    # Deactivate joint actuations : Make sure Pioneer is stationary
    res = vrep.simxSetJointTargetVelocity(clientID, pioneerLeftMotorHandle, 0, vrep.simx_opmode_streaming)
    res = vrep.simxSetJointTargetVelocity(clientID, pioneerRightMotorHandle, 0, vrep.simx_opmode_streaming)



else:
    print('Connection not succesful')
    sys.exit('Could not connect')



# Routine to get the Distance and angular errors for the go-to-goal PID controller:



#vrep.simxSetJointTargetVelocity(clientID, pioneerLeftMotorHandle, 0.2, vrep.simx_opmode_streaming)
#vrep.simxSetJointTargetVelocity(clientID, pioneerRightMotorHandle, 0.2, vrep.simx_opmode_streaming)

error= vrep.simxSetObjectOrientation (clientID, pioneerLeftMotorHandle, 1 ,[ 0,0,1.254 ],vrep.simx_opmode_oneshot_wait)
error= vrep.simxSetObjectOrientation (clientID, pioneerRightMotorHandle, 1 ,[ 0,0,1.254],vrep.simx_opmode_oneshot_wait)

#error=vrep.simxSetJointTargetVelocity(clientID,pioneerLeftMotorHandle,1, vrep.simx_opmode_oneshot_wait)
#error=vrep.simxSetJointTargetVelocity(clientID,pioneerRightMotorHandle,1, vrep.simx_opmode_oneshot_wait)

# time.sleep(5 )
#error=vrep.simxSetJointTargetVelocity(clientID,pioneerRightMotorHandle,0.2, vrep.simx_opmode_oneshot_wait)
#error=vrep.simxSetJointTargetVelocity(clientID,pioneerLeftMotorHandle,0, vrep.simx_opmode_oneshot_wait)
