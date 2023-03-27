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


samp_time = 0.1
n = 1
time_start = time.time()

while True:
    t_now = time.time() - time_start
    if t_now >= samp_time*n:
        setRobotMotion(client_id, wheelHandler, [3, 3, 3, 3])
        print("ya")
    
# --- End of Simulation ---
sim.simxFinish(client_id)
print("Program ended")