#!/usr/bin/env python3

import time
import keyboard
import numpy as np
import sys
import sim

# === Function Definition ===
# ---------------------------
def connectSimulator():
    # Close all opened connections
    sim.simxFinish(-1)
    # Connect to CoppeliaSim
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    if clientID != -1:
        print("Connected to remote API server")
    else:
        print("Connection not successful")
        sys.exit("Could not connect")
    return clientID

def setObjectHandler(clientID, path):
    objectHandler = sim.simxGetObjectHandle(clientID, path, sim.simx_opmode_blocking)[1]
    return objectHandler

def getObjectPose(clientID, objectHandle, block=None):
    if(block == None):
        position = sim.simxGetObjectPosition(clientID, objectHandle, -1, sim.simx_opmode_oneshot)[1]
        orientation = sim.simxGetObjectOrientation(clientID, objectHandle, -1, sim.simx_opmode_oneshot)[1]
    else:
        position = sim.simxGetObjectPosition(clientID, objectHandle, -1, sim.simx_opmode_blocking)[1]
        orientation = sim.simxGetObjectOrientation(clientID, objectHandle, -1, sim.simx_opmode_blocking)[1]

    # print("Pose of: x = {:.2f} y = {:.2f} theta = {:.2f}".format(position[0], position[1], orientation[2]*(180/np.pi)))
    return position[0], position[1], orientation[2]

def setRobotMotion(clientID, motors, velocity):
    sim.simxSetJointTargetVelocity(clientID, motors[0], velocity[0], sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID, motors[1], velocity[1], sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID, motors[2], velocity[2], sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID, motors[3], velocity[3], sim.simx_opmode_oneshot)


def inverseKinematics(local_speed):

    L = 0.5 # distance to wheel
    alpha_1 = np.deg2rad(-135)
    alpha_2 = np.deg2rad(-45)
    alpha_3 = np.deg2rad(45)
    alpha_4 = np.deg2rad(135)

    V = np.array([[local_speed[0]],
                  [local_speed[1]],
                  [local_speed[2]]])

    A = np.array([[-np.sin(alpha_1), np.cos(alpha_1), L],
                  [-np.sin(alpha_2), np.cos(alpha_2), L],
                  [-np.sin(alpha_3), np.cos(alpha_3), L],
                  [-np.sin(alpha_4), np.cos(alpha_4), L]])

    Phi = np.matmul(A, V)

    print("Phi0 = {} Phi1 = {} Phi2 = {} Phi3 = {}".format(Phi[0], Phi[1], Phi[2], Phi[3]))    
    return float(Phi[0]), float(Phi[1]), float(Phi[2]), float(Phi[3])

def keyboardRoutine():

    localSpeed = [0, 0, 0]
    # Horizontal and Vertical
    if keyboard.is_pressed('w'):
        localSpeed = [0, 4, 0]
    elif keyboard.is_pressed('s'):
        localSpeed = [0, -4, 0]
    elif keyboard.is_pressed('a'):
        localSpeed = [-4, 0, 0]
    elif keyboard.is_pressed('d'):
        localSpeed = [4, 0, 0]

    # Diagonal
    elif keyboard.is_pressed('q'):
        localSpeed = [-4, 4, 0]
    elif keyboard.is_pressed('e'):
        localSpeed = [4, 4, 0]
    elif keyboard.is_pressed('z'):
        localSpeed = [-4, -4, 0]
    elif keyboard.is_pressed('c'):
        localSpeed = [4, -4, 0]

    # Rotation
    elif keyboard.is_pressed('r'):
        localSpeed = [0, 0, -4]
    elif keyboard.is_pressed('t'):
        localSpeed = [0, 0, 4]

    return localSpeed

# === Main Program ===
# --------------------
print("Program started")
# --- Connection to Coppelia Simulator ---
client_id = connectSimulator()
 
# --- Get Object Handler ---
robotHandler = setObjectHandler(client_id, "/OmniPlatform")
wheelHandler = [None]*4
for i in range(4):
    wheelHandler[i] = setObjectHandler(client_id, "/OmniPlatform/link[" + str(i) + "]/regularRotation")

waypoint = [None]*2
for i in range(2):
    waypoint[i] = setObjectHandler(client_id, "/Cylinder[" + str(i+1) + "]")

# --- Get initial Pose ---
robotPose = getObjectPose(client_id, robotHandler, block=True)

# --- Initial Param ---
samp_time = 0.1
n = 1
time_start = time.time()

wheelsPhi = [0, 0, 0, 0]
localSpeed = [0, 0, 0]
mode = 1

while True:
    t_now = time.time() - time_start
    if t_now >= samp_time*n:
        # update time
        n += 1

        robotPose = getObjectPose(client_id, robotHandler)

        if(mode ==1):
            localSpeed = keyboardRoutine()
            wheelsPhi = inverseKinematics(localSpeed)

        setRobotMotion(client_id, wheelHandler, wheelsPhi)
        
        print("Wheels Phi: {:.2f}, {:.2f}, {:.2f}, {:.2f}".format(wheelsPhi[0], wheelsPhi[1], wheelsPhi[2], wheelsPhi[3]))
        print("Time = {} Robot Pose: {:.2f}, {:.2f}, {:.2f}\n".format(round(t_now,2), robotPose[0], robotPose[1], robotPose[2]*(180/np.pi)))
        # print("ya")
    
# --- End of Simulation ---
sim.simxFinish(client_id)
print("Program ended")