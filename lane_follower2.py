from utils2 import *

left_motor_power = 2
right_motor_power = 2


def main():

    # PID Parameters
    # Motor Kp Ki Kd
    # 5 0.01 0 0
    # 10 0.0001 0.0005

    # pid = pid_controller(1, 0.0001, 0.0005)
    # Set the Power!
    left_motor(left_motor_power)
    right_motor(right_motor_power)

    # Start the main loop.
    while vrep.simxGetConnectionId(clientID) != -1:
        # Retrieve the image from Lane Vision Sensor.
        get_angle = main_loop()
        # print(get_results)
        if get_angle is None:
            continue
        print(get_angle)
        steer(get_angle)


if __name__ == '__main__':
    main()
