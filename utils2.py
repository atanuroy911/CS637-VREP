import math
import time

import vrep

# Close all the connections.
vrep.simxFinish(-1)
# Connect the V-REP
clientID = vrep.simxStart("127.0.0.1", 19997, True, True, 5000, 5)

Vision_Sensors = [1, 1, 1]

if clientID == -1:
    raise Exception("Fail to connect remote API server.")

_, l_motor = vrep.simxGetObjectHandle(clientID, 'MyBot_FLwheel_Motor', vrep.simx_opmode_oneshot_wait)
_, l_steer = vrep.simxGetObjectHandle(clientID, 'MyBot_FLwheel_Steering', vrep.simx_opmode_oneshot_wait)
_, r_motor = vrep.simxGetObjectHandle(clientID, 'MyBot_FRwheel_Motor', vrep.simx_opmode_oneshot_wait)
_, r_steer = vrep.simxGetObjectHandle(clientID, 'MyBot_FRwheel_Steering', vrep.simx_opmode_oneshot_wait)
_, Vision_Sensors[0] = vrep.simxGetObjectHandle(clientID, 'MyBot_Left_Sensor', vrep.simx_opmode_oneshot_wait)
_, Vision_Sensors[1] = vrep.simxGetObjectHandle(clientID, 'MyBot_Middle_Sensor', vrep.simx_opmode_oneshot_wait)
_, Vision_Sensors[2] = vrep.simxGetObjectHandle(clientID, 'MyBot_Right_Sensor', vrep.simx_opmode_oneshot_wait)

# Display processed image, just for debugging!
_, debug = vrep.simxGetObjectHandle(clientID, 'Debug_Display', vrep.simx_opmode_oneshot_wait)

# The distance between left wheels and right wheels, 28 is the truth distance(cm), 0.2 is a factor.
d = 28 * 0.2
# The distance between front wheels and rear wheels, 30 is the truth distance(cm), 0.2 is a factor.
l = 30 * 0.2


def calc_pid_angle(kp: float, ki: float, kd: float, results: list):
    prev_error = 0
    integral = 0
    derivative = 0

    error = results[2] - results[0]
    integral = integral + error
    derivative = error - prev_error
    prev_error = error

    steeringAngle = kp * error + ki * integral + kd * derivative

    return steeringAngle


def steer(angle: float):
    """Steer for specific angle.

    :param angle: the desired angle we want the car to steer
    """
    if angle == 0:
        angle = 1
    angle = angle * math.pi / 180
    leftAngle = math.atan(l / (-d + l / math.tan(angle)))
    rightAngle = math.atan(l / (d + l / math.tan(angle)))
    _ = vrep.simxSetJointTargetPosition(clientID, l_steer, leftAngle, vrep.simx_opmode_oneshot)
    _ = vrep.simxSetJointTargetPosition(clientID, r_steer, rightAngle, vrep.simx_opmode_oneshot)


def left_motor(speed: float):
    """Set the power of the motor.

    :param speed: the desired speed
    """
    _ = vrep.simxSetJointTargetVelocity(clientID, l_motor, speed, vrep.simx_opmode_oneshot)


def right_motor(speed: float):
    """Set the power of the motor.

    :param speed: the desired speed
    """
    _ = vrep.simxSetJointTargetVelocity(clientID, r_motor, speed, vrep.simx_opmode_oneshot)


def main_loop():
    results = [-1, -1, -1]
    # data = []
    time.sleep(1)
    for i in range(3):
        result, flag, data = vrep.simxReadVisionSensor(clientID, Vision_Sensors[i], vrep.simx_opmode_oneshot)
        # print(data)
        try:
            if result >= 0:
                results[i] = data[0][10]
                # print(data)
                # print(f'This is {i}')
                # print(float(results[i]))
        except:
            continue
    steeringAngle = calc_pid_angle(50, 0.01, 1, results)
    # print(steeringAngle)
    return steeringAngle;
