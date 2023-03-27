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

def getSensorHandle(clientID):
    sensorsHandle = np.array([])
    for i in range(16):
        sensorHandle = sim.simxGetObjectHandle(
            clientID, "Pioneer_p3dx_ultrasonicSensor" + str(i + 1), sim.simx_opmode_blocking)[1]
        # First call proximity sensor must use opmode_streaming
        _, _, _, _, _ = sim.simxReadProximitySensor(clientID, sensorHandle, sim.simx_opmode_streaming)
        sensorsHandle = np.append(sensorsHandle, sensorHandle)
    return sensorsHandle

def getMotorHandle(clientID):
    motorLeftHandle = sim.simxGetObjectHandle(clientID, "Pioneer_p3dx_leftMotor", sim.simx_opmode_blocking)[1]
    motorRightHandle = sim.simxGetObjectHandle(clientID, "Pioneer_p3dx_rightMotor", sim.simx_opmode_blocking)[1]
    return motorLeftHandle, motorRightHandle

def getDistance(clientID, sensors):
    distances = np.array([])
    for i in range(16):
        _, detectionState, detectedPoint, _, _ = sim.simxReadProximitySensor(clientID, np.int(sensors[i]), sim.simx_opmode_buffer)
        distance = detectedPoint[2]
        if detectionState == False:
            distance = 10.0
        distances = np.append(distances,distance)
    return distances

def setRobotMotion(clientID, motors, veloCmd):
    sim.simxSetJointTargetVelocity(clientID, motors[0], veloCmd[0], sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID, motors[1], veloCmd[1], sim.simx_opmode_oneshot)
    
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
    
def keyboardRoutine():

    if keyboard.is_pressed('w'):
        vel = ( 1, 1)
    elif keyboard.is_pressed('s'):
        vel = (-1,-1)
    elif keyboard.is_pressed('q'):
        vel = (-2, -1.5)
    elif keyboard.is_pressed('e'):
        vel = (-1.5, -2)
    elif keyboard.is_pressed('a'):
        vel = (-1, 1)
    elif keyboard.is_pressed('d'):
        vel = ( 1,-1)
    else:
        vel = (0,0)

    setRobotMotion(client_id, [motor_left_handle, motor_right_handle], vel)

def followObject(s3, s4, s5, s6):

    distance_ref = 0.25
    angle_ref = 0
    Kp_trans = 2
    Kp_rot = 0.2

    distance = min(s4, s5)
    angle = s6 - s3

    error_trans = -(distance_ref - distance)
    error_rot = (angle_ref - angle)

    v_trans = Kp_trans * error_trans
    v_rot = Kp_rot * error_rot

    # print("distance = {:.2f} angle = {:.2f} v_trans = {:.2f} v_rot = {:.2f}".format(distance, angle, v_trans, v_rot))

    return v_trans, v_rot

def inverseKinematics(v_trans, v_rot):

    r = 0.195 # wheel radii
    l = 0.190 # distance to wheel
    
    V = np.array([[v_trans],
                  [v_rot]])

    A = np.array([[r/2,      r/2],
                  [r/(2*l), -r/(2*l)]])
                  
    A_ = np.linalg.inv(A)

    Phi = np.dot(A_, V)

    # print("v_1 = {} v_2 = {}".format(Phi[0], Phi[1]))

    return Phi[0]*0.5, Phi[1]*0.5

def inverseKinematics3D(speed, angle):

    r = 0.195 # wheel radii
    l = 0.190 # distance to wheel   


    print("speed[0] = {:.2f} speed[1] = {:.2f} speed[2] = {:.2f} angle = {:.2f}".format(speed[0], speed[1], speed[2], angle*(180/np.pi)))

    V = np.array([[speed[0]],
                  [speed[1]],
                  [speed[2]]])
    
    A = np.array([[r/2*np.cos(angle),  r/2*np.cos(angle)],
                  [r/2*np.sin(angle),  r/2*np.sin(angle)],
                  [r/(2*l),           -r/(2*l)]])

    A_ = np.linalg.pinv(A)

    Phi = np.matmul(A_, V)

    # print(V)
    # print(A)
    # print(Phi)

    return Phi

def moveToPoint(pose, reference):

    error = [0, 0, 0]
    output = [0, 0, 0]
    speed = [0, 0, 0]
    kp = [1, 1, 0.1]

    error[0] = reference[0] - pose[0]
    error[1] = reference[1] - pose[1]
    distance = np.sqrt(error[0]**2 + error[1]**2)
    referenceAngle = np.arctan2(error[1], error[0])
    print("error[0] = {:.2f} error[1] = {:.2f} ref_angle = {:.2f}".format(error[0], error[1], referenceAngle * (180 / np.pi)))

    error[2] = referenceAngle - pose[2]

    for i in range(2):
        output[i] = kp[i] * error[i]
        # to do I/D controller
        speed[i] = output[i]
    # print("speed[0] = {:.2f} speed[1] = {:.2f} speed[2] = {:.2f}".format(speed[0], speed[1], speed[2]))

    return speed

def moveToPoint2(pose, reference):

    error = [0, 0, 0]
    kp = [1, 2] # 1, 2 | 0.75, 1.5
    v_trans = 0
    v_rot = 0
    eps = 0.1
    eps2 = 0.05

    # reference_coba = -200*np.pi/180

    error[0] = reference[0] - pose[0]
    error[1] = reference[1] - pose[1]
    error[2] = reference[2] - pose[2]

    print("reference[2] = {:.2f} pose[2] = {:.2f} error[2] = {:.2f}".format(reference[2] * (180 / np.pi), pose[2] * (180 / np.pi), error[2] * (180 / np.pi)))

    distance = np.sqrt(error[0]**2 + error[1]**2)
    referenceAngle = np.arctan2(error[1], error[0])
    error_positioning = referenceAngle - pose[2]

    # Search nearest angle
    if(abs(error[2]) >= np.pi):
        if(error[2] > 0):
            error[2] = error[2] - 2*np.pi
        else:
            error[2] = error[2] + 2*np.pi

    if(abs(error_positioning) >= np.pi):
        if(error_positioning > 0):
            error_positioning = error_positioning - 2*np.pi
        else:
            error_positioning = error_positioning + 2*np.pi

    # Checking whether position or orientation control
    if(abs(distance) >= eps):
        v_trans = kp[0] * distance
        # v_trans = kp[0] * (error[0]/np.cos(pose[2]))
        v_rot = -kp[1] * error_positioning
        print("error[0] = {:.2f} error[1] = {:.2f} distance = {:.2f}".format(error[0], error[1], distance))
        
    else:
        if(abs(error[2]) >= eps2):
            v_rot = -kp[1] * error[2]
        else:
            # berhenti
            pass

    print("prevPose = {:.2f} reference[2] = {:.2f} pose[2] = {:.2f}".format(moveToPoint2.prevPose * (180 / np.pi), reference[2] * (180 / np.pi), pose[2] * (180 / np.pi)))
    # if(moveToPoint2.prevPose>0 and reference[2]<0):
    #     print("ya,masuk")
    #     v_rot = -kp[1] * (pose[2] - abs(reference[2]))
    # elif(moveToPoint2.prevPose<0 and reference[2]>=0):
    #     v_rot = kp[1] * (reference[2] - abs(pose[2]))

    moveToPoint2.prevPose = pose[2]

    print("referenceAngle = {:.2f} error[2] = {:.2f} pose[2] = {:.2f}".format(referenceAngle * (180 / np.pi), error[2] * (180 / np.pi) ,pose[2] * (180 / np.pi)))
    print("v_trans = {:.2f} v_rot = {:.2f}".format(v_trans, v_rot))

    return v_trans, v_rot

moveToPoint2.prevPose = 0


def limitSpeed(v1, v2, lim1, lim2):

    if(v1 >= lim1):
       v1 = lim1
    elif(v1 <= -lim1):
         v1 = -lim1

    if(v2 >= lim2):
       v2 = lim2
    elif(v2 <= -lim2):
         v2 = -lim2

    return v1, v2

# === Main Program ===
# --------------------
print("Program started")
# --- Connection to Coppelia Simulator ---
client_id = connectSimulator()
# --- object handle ---
motor_left_handle, motor_right_handle = getMotorHandle(client_id)
sensor_handle = getSensorHandle(client_id)
robotHandle = setObjectHandler(client_id, "/PioneerP3DX")
waypoint1 = setObjectHandler(client_id, "/Cylinder[0]")
# --- Simulation Process ---
samp_time = 0.1
n = 1
velo_zero = (0,0)
cmd_vel = velo_zero
time_start = time.time()

mode = 4
robotPose = getObjectPose(client_id, robotHandle, block=True)
waypoint1_pose = getObjectPose(client_id, waypoint1, block=True)
 

while True:
    t_now = time.time() - time_start
    if t_now >= samp_time*n:
        # Sensors
        object_distance = getDistance(client_id, sensor_handle)
        # Robot motion simulation
        setRobotMotion(client_id, [motor_left_handle, motor_right_handle], cmd_vel)
        # update time
        n += 1
        
        if(mode == 1):
            keyboardRoutine()

        elif(mode == 2):
            v_trans, v_rot = followObject(object_distance[2], object_distance[3], object_distance[4], object_distance[5])
            v_1, v_2 = inverseKinematics(v_trans, v_rot)
            v_1, v_2 = limitSpeed(v_1, v_2, 4, 4)
            cmd_vel = (v_1, v_2)

            print("v_1 = {:.2f} v_2 = {:.2f} v_trans = {:.2f} v_rot {:.2f}".format(float(v_1), float(v_2), float(v_trans), float(v_rot)))

        elif(mode == 3):
            # BELUM WORKING HEHE#
            robotPose = getObjectPose(client_id, robotHandle)
            waypoint1_pose = getObjectPose(client_id, waypoint1)
            robotSpeed = moveToPoint(robotPose, waypoint1_pose)
            v_1, v_2 = inverseKinematics3D(robotSpeed, robotPose[2])
            # v_1, v_2 = limitSpeed(v_1, v_2, 8, 8)
            cmd_vel = (v_1, v_2)

            print("v1 = {} v2 = {}".format(v_1, v_2))
            print("Robot Pose: x = {:.2f} y = {:.2f} theta = {:.2f}".format(robotPose[0], robotPose[1], robotPose[2] * (180 / np.pi)))
            print("Waypoint Pose: x = {:.2f} y = {:.2f} theta = {:.2f}".format(waypoint1_pose[0], waypoint1_pose[1], waypoint1_pose[2] * (180 / np.pi)))

        elif(mode == 4):
            robotPose = getObjectPose(client_id, robotHandle)
            waypoint1_pose = getObjectPose(client_id, waypoint1)
            v_trans, v_rot = moveToPoint2(robotPose, waypoint1_pose)  
            v_1, v_2 = inverseKinematics(v_trans, v_rot)      
            # v_1, v_2 = limitSpeed(v_1, v_2, 8, 8)
            cmd_vel = (float(v_1), float(v_2))

            print("Robot Pose: x = {:.2f} y = {:.2f} theta = {:.2f}".format(robotPose[0], robotPose[1], robotPose[2] * (180 / np.pi)))
            print("Waypoint Pose: x = {:.2f} y = {:.2f} theta = {:.2f}".format(waypoint1_pose[0], waypoint1_pose[1], waypoint1_pose[2] * (180 / np.pi)))
            print("v_1 = {:.2f} v_2 = {:.2f}".format(float(v_1), float(v_2)))
            # print(v_1)
            # print(v_2)
        
        elif(mode == 5):
            pass
        # show info
        print("Time: ", round(t_now, 2), "front side object distance = ", object_distance[3], object_distance[4], "\n")
    
    # if keyboard.is_pressed('esc'):
    #     setRobotMotion(client_id, [motor_left_handle, motor_right_handle], velo_zero)
    #     break
    
# --- End of Simulation ---
sim.simxFinish(client_id)
print("Program ended")