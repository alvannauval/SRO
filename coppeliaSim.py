# Importing libraries
from pydoc import cli
from turtle import pos, position
import sim
import sys
import time
import numpy as np

# Variables and Constants
veloCmd_left, veloCmd_right = 1.0 , 1.0
trapezoidTime_left = (1, 10, 20, 30)
trapezoidTime_right = (1, 15, 25, 35)
position = (.0, .0, .0)
angle = (.0, .0, .0)

def connectSim():
    # Close all connections
    sim.simxFinish(-1)
    # Connect to CoppeliaSim
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
    if clientID != -1:
        print('Connect to remote API server')
    else:
        print('Connection unsuccesful, program closed')
        sys.exit()
    
    return clientID

def getRobotHandle(clientID):
    robotHandle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx', sim.simx_opmode_blocking)
    
    return robotHandle

# def getFloorHandle(clientID):
#     floorHandle = sim.simxGetObjectHandle(clientID, 'ReziableFloor_5_25', sim.simx_opmode_blocking)

#     return floorHandle

# def getRobotPosAngle(clientID, robotHandle, floorHandle, firstTime):

#     if firstTime:
#         ~, pos = sim.simxGetObjectPosition(clientID, robotHandle, -1, sim.simx_opmode_streaming)
#         ~, ori = sim.simxGetObjectOrientation(clientID, robotHandle, -1, sim.simx_opmode_streaming)
#         firstTime = False
#     else:
#         ~, pos = sim.simxGetObjectPosition(clientID, robotHandle, -1, sim.simx_opmode_buffer)
#         ~, ori = sim.simxGetObjectOrientation(clientID, robotHandle, -1, sim.simx_opmode_buffer)
    

#     return position, orientation

def getSensorsHandle(clientID):
    sensorsHandle = np.array([])
    for i in range(16):
        sensorHandle = sim.simxGetObjectHandle(
            clientID, 'Pioneer_p3dx_ultrasonicSensor'+str(i+1), sim.simx_opmode_blocking)[1]
            # First call to proximity sensor must use opmode_streaming
        
        _,_,_,_,_ = sim.simxReadProximitySensor(clientID, sensorHandle, sim.simx_opmode_streaming)
        
        sensorsHandle = np.append(sensorsHandle, sensorHandle)
            
    return sensorsHandle

def getMotorHandle(clientID):
    motorLeft = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)[1]
    motorRight = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)[1]
    
    return motorLeft, motorRight

def getDistance(clientID, sensors):
    distances = np.array([])
    for i in range(16):
        _, detectionState, detectedPoint, _, _ =  sim.simxReadProximitySensor(clientID, np.int(sensors[i]), sim.simx_opmode_buffer)
        
        distance = detectedPoint[2]

        if detectionState == False:
            distance = 10.0
        distances = np.append(distances, distance)
    
    return distances

def trapezoidalNotion(veloCmd, trapezoidTime, t):
    if t <= trapezoidTime[0] or t > trapezoidTime[3]:
        vt=0
    if t > trapezoidTime[0] and t <= trapezoidTime[1]:
        vt = veloCmd*(t-trapezoidTime[0])/(trapezoidTime[1]-trapezoidTime[0])
    if t > trapezoidTime[1] and t <= trapezoidTime[2]:
        vt = veloCmd
    if t > trapezoidTime[2] and t <= trapezoidTime[3]:
        vt = veloCmd*(trapezoidTime[3]-t)/(trapezoidTime[3]-trapezoidTime[2])
    return vt

def setRobotVelocity(t):
    veloLeft = trapezoidalNotion(veloCmd_left, trapezoidTime_left, t)
    veloRight = trapezoidalNotion(veloCmd_right, trapezoidTime_right, t)
    return (veloLeft, veloRight)

def simRobotMotion(clientID, motor, veloCmd):
    _ = sim.simxSetJointTargetVelocity(clientID, motor[0], veloCmd[0][0], sim.simx_opmode_oneshot)
    _ = sim.simxSetJointTargetVelocity(clientID, motor[1], veloCmd[1][0], sim.simx_opmode_oneshot)

def kinematrix(velocity, angular):
    matrix = np.array([[0.04875, 0.04875],
                       [0.0002559, -0.0002559]])
    # matrix = np.array([[48.75, 48.75],
    #                    [0.256, -0.256]])

    input = np.array([[velocity],[angular]])
    
    invmatrix = np.linalg.inv(matrix)
    
    result = np.dot(invmatrix, input)
    
    return result

# MAIN PROGRAM
def main():
    print('Program started')

    # Connecting to CoppeliaSim
    client_id = connectSim()

    # Object Handle
    robot_handle = getRobotHandle(client_id)
    # floor_handle = getFloorHandle(client_id)
    motor_left, motor_right = getMotorHandle(client_id)
    sensors_handle = getSensorsHandle(client_id)
    # Simulation Process
    firstTime=True
    samp_time = 0.1
    n = 1
    velo_zero = (0,0)
    time_start = time.time()

    while(True):
        t_now = time.time() - time_start
        # Command processing
        if t_now >= samp_time*n:
            # Motion Command
            velo_cmd = setRobotVelocity(t_now)

            # robot_pos_angle = getRobotPosAngle(client_id, robot_handle, floor_handle, firstTime)
            object_distance = getDistance(client_id, sensors_handle)

            #Create error
            error_forward = ((object_distance[3] + object_distance[4])/2)-0.5
            error_angular = (object_distance[2]-object_distance[5])
            value_forward =  error_forward * 0.1
            value_angular = error_angular * 0.001

            # Give some restraint to the value
            if(value_forward >= 100): value_forward = 100
            if(value_forward <= -50): value_forward =  -50
            if(value_angular >= 2): value_angular = 2
            if(value_angular <= -2): value_angular = -2

            # Enter the value to kinematik
            kinematik = kinematrix(value_forward, value_angular)
            # Robot motion simulation
            # simRobotMotion(client_id, (motor_left,motor_right), velo_cmd)
            simRobotMotion(client_id, (motor_left,motor_right), kinematik)
            # update sample
            n += 1.0
            # show info
            # print('t = ', round(t_now, 2), '--- velocity command = ', velo_cmd)
            # print('t = ', round(t_now, 2), '--- velocity command = ', kinematik[0][0], kinematik[1][0])
            print('t = ', round(t_now, 2), 'front object distance= ', object_distance[3], object_distance[4])
            # print(value_forward, value_angular, robot_pos_angle[0])
        if 0xFF == ord('q'):
            simRobotMotion(client_id, (motor_left, motor_right), velo_zero)
            break

    # Simulation Finished
    sim.simxFinish(client_id)
    print('program ended\n')

if __name__ == '__main__':
    main()