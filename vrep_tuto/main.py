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
