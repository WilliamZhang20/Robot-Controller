from spike import PrimeHub, MotorPair, DistanceSensor
from spike.control import wait_for_seconds
from time import sleep
from math import *

hub = PrimeHub()

# Reset the yaw angle on startup
hub.motion_sensor.reset_yaw_angle()

# Show an image to confirm the program has started
hub.light_matrix.show_image('HAPPY')

# Distance Settings to control speed

wall_detector = DistanceSensor('A')

desired_distance = 10

K_p_spd = 10
K_i_spd = 0.001
K_d_spd = 9

prev_error_spd = 0
sum_error_spd = 0

# Desired heading and PID constants

motor_pair = MotorPair('D', 'C')
motor_pair.set_stop_action('brake')

DH = 0
K_p_dir = 2.4
K_i_dir = 0.001
K_d_dir = 2

# PID variables
prev_error = 0
sum_error_dir = 0 # Accumulated integral

# Output limits for the motors
MIN_OUTPUT = -100
MAX_OUTPUT = 100

# Feedforward control constant for steering
K_ff = 1.0

def clamp(value, min_value, max_value):
    """Clamp the value to the specified range."""
    return max(min_value, min(value, max_value))

def get_distance():
    val = wall_detector.get_distance_cm()
    if val is not None:
        dist_cm = desired_distance - val
    else:
        dist_cm = 1000 # too far to detect
    dist_cm = abs(dist_cm)
    return dist_cm

while True:
    # Calculate error (desired - current)
    curr_error = DH - hub.motion_sensor.get_yaw_angle()

    dist_cm = get_distance()    

    print(dist_cm)

    while dist_cm <= 6:
        # if obstacle in front, then turn until there is no obstacle
        print('rotating')
        motor_pair.start((hub.motion_sensor.get_yaw_angle()-90) % 180, 20)
        dist_cm = get_distance()
        if(dist_cm >= 10):
            motor_pair.start((hub.motion_sensor.get_yaw_angle()-90) % 180, 20)
            motor_pair.start(hub.motion_sensor.get_yaw_angle(), 50)
            sleep(1)

    # Update integral and derivative terms
    sum_error_dir += curr_error
    sum_error_spd += dist_cm

    # Implementing anti-windup: only integrate if output is within bounds
    if abs(curr_error) < 10: # Threshold for integrating
        sum_error_dir = clamp(sum_error_dir, MIN_OUTPUT / K_i_dir, MAX_OUTPUT / K_i_dir)

    d_error = curr_error - prev_error
    d_error_spd = dist_cm - prev_error_spd 

    # PID output calculation with feedforward
    feedforward = K_ff * DH # Assuming linear feedforward based on desired heading
    out = int(feedforward + curr_error * K_p_dir + sum_error_dir * K_i_dir + d_error * K_d_dir)
    out_spd = int(dist_cm * K_p_spd + sum_error_spd * K_i_spd + d_error_spd * K_d_spd)
    out_spd = abs(out_spd)
    out_spd = min(out_spd, MAX_OUTPUT - 10) # clamp speed

    # Clamp the output to prevent saturation
    out = clamp(out, MIN_OUTPUT, MAX_OUTPUT)

    # Start the motors with the calculated correction
    motor_pair.start(out, out_spd)

    # Update previous error for the next iteration
    prev_error = curr_error

# Stop the motors after the loop (though this loop runs indefinitely)
motor_pair.stop()
