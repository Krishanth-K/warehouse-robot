from controller import (
    Robot,
    Motor,
    PositionSensor,
    TouchSensor,
    Camera,
    Lidar
)
import math
import sys

# ---------------- Constants ----------------
TIME_STEP = 16

MAX_WHEEL_SPEED = 3.0
WHEELS_DISTANCE = 0.4492
SUB_WHEELS_DISTANCE = 0.098
WHEEL_RADIUS = 0.08

TOLERANCE = 0.05

def almost_equal(a, b):
    return (b - TOLERANCE) < a < (b + TOLERANCE)

# ---------------- Enums ----------------
FLL_WHEEL, FLR_WHEEL, FRL_WHEEL, FRR_WHEEL, BLL_WHEEL, BLR_WHEEL, BRL_WHEEL, BRR_WHEEL = range(8)
FL_ROTATION, FR_ROTATION, BL_ROTATION, BR_ROTATION = range(4)
SHOULDER_ROLL, SHOULDER_LIFT, UPPER_ARM_ROLL, ELBOW_LIFT, WRIST_ROLL = range(5)
LEFT_FINGER, RIGHT_FINGER = range(2)

# ---------------- Robot ----------------
robot = Robot()

# ---------------- Devices ----------------
wheel_motors = [None] * 8
wheel_sensors = [None] * 8
rotation_motors = [None] * 4
rotation_sensors = [None] * 4

left_arm_motors = [None] * 5
left_arm_sensors = [None] * 5
right_arm_motors = [None] * 5
right_arm_sensors = [None] * 5

left_finger_motor = None
right_finger_motor = None
left_finger_sensor = None
right_finger_sensor = None

left_finger_contacts = [None, None]
right_finger_contacts = [None, None]

torso_motor = None
torso_sensor = None

right_head_cam = None
base_lidar = None  # <<< LIDAR >>>

# ---------------- Step ----------------
def step():
    if robot.step(TIME_STEP) == -1:
        sys.exit(0)

# ---------------- Initialization ----------------
def initialize_devices():
    global torso_motor, torso_sensor
    global left_finger_motor, right_finger_motor
    global left_finger_sensor, right_finger_sensor
    global right_head_cam, base_lidar

    wheel_names = [
        "fl_caster_l_wheel_joint", "fl_caster_r_wheel_joint",
        "fr_caster_l_wheel_joint", "fr_caster_r_wheel_joint",
        "bl_caster_l_wheel_joint", "bl_caster_r_wheel_joint",
        "br_caster_l_wheel_joint", "br_caster_r_wheel_joint"
    ]

    for i, name in enumerate(wheel_names):
        wheel_motors[i] = robot.getDevice(name)
        wheel_sensors[i] = wheel_motors[i].getPositionSensor()

    rotation_names = [
        "fl_caster_rotation_joint", "fr_caster_rotation_joint",
        "bl_caster_rotation_joint", "br_caster_rotation_joint"
    ]

    for i, name in enumerate(rotation_names):
        rotation_motors[i] = robot.getDevice(name)
        rotation_sensors[i] = rotation_motors[i].getPositionSensor()

    left_arm_names = [
        "l_shoulder_pan_joint", "l_shoulder_lift_joint",
        "l_upper_arm_roll_joint", "l_elbow_flex_joint",
        "l_wrist_roll_joint"
    ]

    right_arm_names = [
        "r_shoulder_pan_joint", "r_shoulder_lift_joint",
        "r_upper_arm_roll_joint", "r_elbow_flex_joint",
        "r_wrist_roll_joint"
    ]

    for i in range(5):
        left_arm_motors[i] = robot.getDevice(left_arm_names[i])
        left_arm_sensors[i] = left_arm_motors[i].getPositionSensor()

        right_arm_motors[i] = robot.getDevice(right_arm_names[i])
        right_arm_sensors[i] = right_arm_motors[i].getPositionSensor()

    left_finger_motor = robot.getDevice("l_finger_gripper_motor::l_finger")
    right_finger_motor = robot.getDevice("r_finger_gripper_motor::l_finger")
    left_finger_sensor = left_finger_motor.getPositionSensor()
    right_finger_sensor = right_finger_motor.getPositionSensor()

    left_finger_contacts[LEFT_FINGER] = robot.getDevice("l_gripper_l_finger_tip_contact_sensor")
    left_finger_contacts[RIGHT_FINGER] = robot.getDevice("l_gripper_r_finger_tip_contact_sensor")
    right_finger_contacts[LEFT_FINGER] = robot.getDevice("r_gripper_l_finger_tip_contact_sensor")
    right_finger_contacts[RIGHT_FINGER] = robot.getDevice("r_gripper_r_finger_tip_contact_sensor")

    torso_motor = robot.getDevice("torso_lift_joint")
    torso_sensor = robot.getDevice("torso_lift_joint_sensor")

    right_head_cam = robot.getDevice("wide_stereo_r_stereo_camera_sensor")
    base_lidar = robot.getDevice("base_laser")  # <<< LIDAR >>>

# ---------------- Enable ----------------
def enable_devices():
    for i in range(8):
        wheel_sensors[i].enable(TIME_STEP)
        wheel_motors[i].setPosition(float("inf"))
        wheel_motors[i].setVelocity(0.0)

    for i in range(4):
        rotation_sensors[i].enable(TIME_STEP)

    left_finger_sensor.enable(TIME_STEP)
    right_finger_sensor.enable(TIME_STEP)

    for i in range(5):
        left_arm_sensors[i].enable(TIME_STEP)
        right_arm_sensors[i].enable(TIME_STEP)

    for i in range(2):
        left_finger_contacts[i].enable(TIME_STEP)
        right_finger_contacts[i].enable(TIME_STEP)

    torso_sensor.enable(TIME_STEP)

    right_head_cam.enable(TIME_STEP)

    base_lidar.enable(TIME_STEP)  # <<< LIDAR >>>

# ---------------- Wheels ----------------
def set_wheels_speed(speed):
    for m in wheel_motors:
        m.setVelocity(speed)

def stop_wheels():
    set_wheels_speed(0.0)

# ---------------- Rotation Wheels ----------------
def set_rotation_wheels_angles(fl, fr, bl, br, wait):
    rotation_motors[FL_ROTATION].setPosition(fl)
    rotation_motors[FR_ROTATION].setPosition(fr)
    rotation_motors[BL_ROTATION].setPosition(bl)
    rotation_motors[BR_ROTATION].setPosition(br)

    if wait:
        targets = [fl, fr, bl, br]
        while True:
            done = True
            for i in range(4):
                if not almost_equal(rotation_sensors[i].getValue(), targets[i]):
                    done = False
                    break
            if done:
                break
            step()

# ---------------- Motion ----------------
def robot_go_forward(distance):
    speed = MAX_WHEEL_SPEED if distance > 0 else -MAX_WHEEL_SPEED
    set_wheels_speed(speed)

    start = wheel_sensors[FLL_WHEEL].getValue()

    while True:
        current = wheel_sensors[FLL_WHEEL].getValue()
        traveled = abs(WHEEL_RADIUS * (current - start))
        if traveled >= abs(distance):
            break
        if abs(distance) - traveled < 0.025:
            set_wheels_speed(0.1 * speed)
        step()

    stop_wheels()

def robot_rotate(angle):
    stop_wheels()
    set_rotation_wheels_angles(
        3 * math.pi / 4, math.pi / 4,
        -3 * math.pi / 4, -math.pi / 4,
        True
    )

    speed = MAX_WHEEL_SPEED if angle > 0 else -MAX_WHEEL_SPEED
    set_wheels_speed(speed)

    start = wheel_sensors[FLL_WHEEL].getValue()
    expected = abs(angle * 0.5 * (WHEELS_DISTANCE + SUB_WHEELS_DISTANCE))

    while True:
        current = wheel_sensors[FLL_WHEEL].getValue()
        traveled = abs(WHEEL_RADIUS * (current - start))
        if traveled >= expected:
            break
        if expected - traveled < 0.025:
            set_wheels_speed(0.1 * speed)
        step()

    set_rotation_wheels_angles(0, 0, 0, 0, True)
    stop_wheels()

# ---------------- Arms ----------------
def set_arm(motors, sensors, values, wait):
    for i in range(5):
        motors[i].setPosition(values[i])
    if wait:
        while True:
            if all(almost_equal(sensors[i].getValue(), values[i]) for i in range(5)):
                break
            step()

# ---------------- Gripper ----------------
def set_gripper(left, open_grip, torque, wait):
    motor = left_finger_motor if left else right_finger_motor
    sensor = left_finger_sensor if left else right_finger_sensor
    contacts = left_finger_contacts if left else right_finger_contacts

    if open_grip:
        motor.setPosition(0.5)
        if wait:
            while not almost_equal(sensor.getValue(), 0.5):
                step()
    else:
        motor.setPosition(0.0)
        if wait:
            while (
                (contacts[LEFT_FINGER].getValue() == 0.0 or
                 contacts[RIGHT_FINGER].getValue() == 0.0)
                and not almost_equal(sensor.getValue(), 0.0)
            ):
                step()
            motor.setAvailableTorque(torque)

# ---------------- Torso ----------------
def set_torso(height, wait):
    torso_motor.setPosition(height)
    if wait:
        while not almost_equal(torso_sensor.getValue(), height):
            step()

# ---------------- Main ----------------
initialize_devices()
enable_devices()

set_torso(0.2, True)

while True:
    # ---------- CAMERA ----------
    image = right_head_cam.getImage()
    if image is not None:
        w = right_head_cam.getWidth()
        h = right_head_cam.getHeight()
        r = right_head_cam.imageGetRed(image, w, w // 2, h // 2)
        g = right_head_cam.imageGetGreen(image, w, w // 2, h // 2)
        b = right_head_cam.imageGetBlue(image, w, w // 2, h // 2)
        if int(robot.getTime()) % 1 == 0:
            print("Right Head Camera Center RGB:", r, g, b)

    # ---------- LIDAR ----------
    ranges = base_lidar.getRangeImage()
    if ranges:
        n = len(ranges)
        front = ranges[n // 2]
        left = ranges[int(0.75 * n)]
        right = ranges[int(0.25 * n)]
        print(f"LIDAR | Front: {front:.2f} m | Left: {left:.2f} m | Right: {right:.2f} m")

    set_gripper(True, False, 20.0, True)
    set_gripper(False, False, 20.0, True)

    set_arm(left_arm_motors, left_arm_sensors, [0, 0.5, 0, -1, 0], True)
    set_arm(right_arm_motors, right_arm_sensors, [0, 0.5, 0, -1, 0], True)

    robot_go_forward(-0.35)
    robot_rotate(math.pi)
    robot_go_forward(0.35)

    set_arm(left_arm_motors, left_arm_sensors, [0, 0.5, 0, -0.5, 0], True)
    set_arm(right_arm_motors, right_arm_sensors, [0, 0.5, 0, -0.5, 0], True)

    set_gripper(True, True, 0.0, True)
    set_gripper(False, True, 0.0, True)
