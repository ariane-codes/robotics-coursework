"""robotics_project controller."""
import random
from controller import Robot, Motor, DistanceSensor, Camera

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
TIMESTEP = 32
DS_THRESHOLD = 90
ANGULAR_SPEED = 500 # Tried to approximate this, but isn't very accurate
MAX_SPEED = 6.28
INFINITY = float('inf')
LEFT = "left"
RIGHT = "right"

# Get all necessary devices and sensors
motor_left = robot.getDevice('left wheel motor')
motor_right = robot.getDevice('right wheel motor')
motor_left.setPosition(INFINITY)
motor_right.setPosition(INFINITY)
ds_6 = robot.getDevice('ps6')
ds_7 = robot.getDevice('ps7')
ds_0 = robot.getDevice('ps0')
ds_1 = robot.getDevice('ps1')
camera = robot.getDevice('camera')

# Enable sensors
ds_6.enable(TIMESTEP)
ds_7.enable(TIMESTEP)
ds_0.enable(TIMESTEP)
ds_1.enable(TIMESTEP)
camera.enable(10)
camera.recognitionEnable(True)

def move_forward():
    motor_left.setVelocity(MAX_SPEED)
    motor_right.setVelocity(MAX_SPEED)

def stop_motors():
    motor_left.setVelocity(0.0)
    motor_right.setVelocity(0.0)

def turn_left():
    stop_motors()
    motor_left.setVelocity(-MAX_SPEED)
    motor_right.setVelocity(MAX_SPEED)

def turn_right():
    stop_motors()
    motor_left.setVelocity(MAX_SPEED)
    motor_right.setVelocity(-MAX_SPEED)

def calculate_rotation_time(angle):
    return abs(angle) / ANGULAR_SPEED

def rotate_by_angle(angle, direction):
    duration = calculate_rotation_time(angle)
    start_time = robot.getTime()
    while robot.getTime() < start_time + duration:
        if direction == LEFT:
            turn_left()
        elif direction == RIGHT:
            turn_right()
        robot.step(TIMESTEP)


def get_sensor_condition(sensor):
    if sensor.getValue() > DS_THRESHOLD:
        return True
    else:
        return False

def is_camera_recognizing_objects(camera_recognition_objects):
    if len(camera.getRecognitionObjects()) > 0:
        return True
    else:
        return False

def is_ball(camera_recognition_object):
    if camera_recognition_object.getModel() == "soccer ball":
        return True
    else:
        return False

def act_on_camera_recognition(camera_recognition_objects):
    for recognized_object in camera_recognition_objects:
        if is_ball(recognized_object):
            size_on_img = recognized_object.getSizeOnImage()
            x_size = size_on_img[0]
            y_size = size_on_img[1]

            if x_size > 400 or y_size > 400:
                print("Ball is too close, turning right")
                rotate_by_angle(180, RIGHT)

def turn_if_ball_is_in_front():
    camera_recognition_objects = camera.getRecognitionObjects()
    if is_camera_recognizing_objects(camera_recognition_objects):
        act_on_camera_recognition(camera_recognition_objects)
            
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(TIMESTEP) != -1:


    # Sensors to the left
    ds_6_is_active = get_sensor_condition(ds_6)
    ds_7_is_active = get_sensor_condition(ds_7)

    # Sensors to the right
    ds_0_is_active = get_sensor_condition(ds_0)
    ds_1_is_active = get_sensor_condition(ds_1)
    
    if (ds_6_is_active or ds_7_is_active):
        # There's an obstacle to the left, so turn right
        print("Obstacle to the left, turning right")
        rotate_by_angle(90, RIGHT)
    elif (ds_0_is_active or ds_1_is_active):
        # There's an obstacle to the right, so turn left
        print("Obstacle to the right, turning left")
        rotate_by_angle(90, LEFT)
    else:
        # No obstacles, so move forward
        turn_if_ball_is_in_front()
        move_forward()
