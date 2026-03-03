from controller import Supervisor, Keyboard, Camera, Lidar, Motor
import math

TIME_STEP = 16

MAX_WHEEL_SPEED = 3.0
ROTATE_SPEED = 2.5

PIXEL_STEP = 4
FRONT_IMAGE_RATIO = 0.6
CENTER_WIDTH_RATIO = 0.8

OBJECT_DISTANCE_THRESHOLD = 1.5
FRONT_FOV = 0.5

HUMAN_CONFIRM_FRAMES = 3
HUMAN_LOST_FRAMES = 10
FRONT_STABLE_FRAMES = 8
HUMAN_DISTANCE_THRESHOLD = 2.0


robot = Supervisor()
keyboard = Keyboard()
keyboard.enable(TIME_STEP)


human_nodes = {}

root = robot.getRoot()
children = root.getField("children")

for i in range(children.getCount()):
    node = children.getMFNode(i)
    if node.getTypeName() == "Pedestrian":
        name = node.getField("name").getSFString()
        human_nodes[name] = node
        print(f"Found Pedestrian: {name}")


wheel_motors = []
rotation_motors = []

left_finger_motor = None
right_finger_motor = None

right_head_cam = None
base_lidar = None


last_state = None
last_human_name = None

human_seen_frames = 0
human_lost_frames = 0
human_confirmed = False
front_stable_counter = 0


def initialize_devices():
    global right_head_cam, base_lidar
    global left_finger_motor, right_finger_motor

    wheel_names = [
        "fl_caster_l_wheel_joint","fl_caster_r_wheel_joint",
        "fr_caster_l_wheel_joint","fr_caster_r_wheel_joint",
        "bl_caster_l_wheel_joint","bl_caster_r_wheel_joint",
        "br_caster_l_wheel_joint","br_caster_r_wheel_joint"
    ]

    for name in wheel_names:
        m = robot.getDevice(name)
        m.setPosition(float("inf"))
        m.setVelocity(0.0)
        wheel_motors.append(m)

    rotation_names = [
        "fl_caster_rotation_joint",
        "fr_caster_rotation_joint",
        "bl_caster_rotation_joint",
        "br_caster_rotation_joint"
    ]

    for name in rotation_names:
        rotation_motors.append(robot.getDevice(name))

    left_finger_motor = robot.getDevice("l_finger_gripper_motor::l_finger")
    right_finger_motor = robot.getDevice("r_finger_gripper_motor::l_finger")

    right_head_cam = robot.getDevice("wide_stereo_r_stereo_camera_sensor")
    right_head_cam.enable(TIME_STEP)

    base_lidar = robot.getDevice("base_laser")
    base_lidar.enable(TIME_STEP)
    base_lidar.enablePointCloud()

def set_wheels(speed):
    for m in wheel_motors:
        m.setVelocity(speed)

def stop_wheels():
    set_wheels(0.0)

def set_caster_angles(fl, fr, bl, br):
    rotation_motors[0].setPosition(fl)
    rotation_motors[1].setPosition(fr)
    rotation_motors[2].setPosition(bl)
    rotation_motors[3].setPosition(br)

def set_gripper(left, open_grip, torque):
    motor = left_finger_motor if left else right_finger_motor
    if open_grip:
        motor.setPosition(0.5)
    else:
        motor.setPosition(0.0)
        motor.setAvailableTorque(torque)

def get_front_distance(ranges):
    c = len(ranges) // 2
    w = int(len(ranges) * FRONT_FOV / 2)
    return min(ranges[c - w : c + w])

initialize_devices()

print("Robot           : PR2")
print("RoboticSystem   : Warehouse")
print("Process         : Idle")

rotate_mode = None


while robot.step(TIME_STEP) != -1:
    key = keyboard.getKey()
    rotating = False

    if key == Keyboard.UP:
        rotate_mode = None
        set_caster_angles(0, 0, 0, 0)
        set_wheels(MAX_WHEEL_SPEED)
    elif key == Keyboard.DOWN:
        rotate_mode = None
        set_caster_angles(0, 0, 0, 0)
        set_wheels(-MAX_WHEEL_SPEED)
    elif key == Keyboard.LEFT:
        rotating = True
        set_caster_angles(
            3 * math.pi / 4, math.pi / 4,
           -3 * math.pi / 4, -math.pi / 4
        )
        set_wheels(ROTATE_SPEED)
    elif key == Keyboard.RIGHT:
        rotating = True
        set_caster_angles(
            3 * math.pi / 4, math.pi / 4,
           -3 * math.pi / 4, -math.pi / 4
        )
        set_wheels(-ROTATE_SPEED)
    else:
        stop_wheels()

    front_dist = get_front_distance(base_lidar.getRangeImage())
    if front_dist < OBJECT_DISTANCE_THRESHOLD and not rotating:
        front_stable_counter += 1
    else:
        front_stable_counter = 0

    front_close = front_stable_counter >= FRONT_STABLE_FRAMES

    img = right_head_cam.getImage()
    w, h = right_head_cam.getWidth(), right_head_cam.getHeight()

    y0 = int(h * (1 - FRONT_IMAGE_RATIO))
    x0 = int(w * (0.5 - CENTER_WIDTH_RATIO / 2))
    x1 = int(w * (0.5 + CENTER_WIDTH_RATIO / 2))

    red = green = yellow = False

    for y in range(y0, h, PIXEL_STEP):
        for x in range(x0, x1, PIXEL_STEP):
            r = right_head_cam.imageGetRed(img, w, x, y)
            g = right_head_cam.imageGetGreen(img, w, x, y)
            b = right_head_cam.imageGetBlue(img, w, x, y)

            lum = 0.2126*r + 0.7152*g + 0.0722*b

            if r > g * 1.3 and r > b * 1.3 and lum > 30:
                red = True
            elif g > r * 1.3 and g > b * 1.3 and lum > 30:
                green = True
            elif r > 120 and g > 120 and b < 120:
                yellow = True

    human_camera = (red and green) or (green and yellow) or (red and green and yellow)

    if human_camera and front_close:
        human_seen_frames += 1
        human_lost_frames = 0
    else:
        human_lost_frames += 1
        human_seen_frames = 0

    if human_seen_frames >= HUMAN_CONFIRM_FRAMES:
        human_confirmed = True
    if human_lost_frames >= HUMAN_LOST_FRAMES:
        human_confirmed = False
        last_human_name = None

    if human_confirmed:
        state = "Human detected"
    elif front_close:
        state = "Object detected"
    else:
        state = "Idle"

    if state != last_state:
        if state == "Human detected":
            print("Agent           : Human")
            print("Physical        : HumanBody")
            print("Process         : Interaction")

            pr2_pos = robot.getSelf().getPosition()
            closest_name = None
            closest_dist = float("inf")
            closest_pos = None

            for name, human in human_nodes.items():
                pos = human.getPosition()
                dist = math.dist(pr2_pos, pos)
                if dist < closest_dist:
                    closest_dist = dist
                    closest_name = name
                    closest_pos = pos

            if closest_name:
                print("Human name      :", closest_name)
                print("Distance (m)    :", round(closest_dist, 2))
                print("Position        :", [round(x, 2) for x in closest_pos])
                last_human_name = closest_name

        elif state == "Object detected":
            print("Physical        : Object")
            print("Process         : Motion")
        else:
            print("Process         : Idle")

        last_state = state

    if key == ord('Q'):
        set_gripper(True, False, 20.0)
    elif key == ord('Z'):
        set_gripper(True, True, 0.0)
    elif key == ord('E'):
        set_gripper(False, False, 20.0)
    elif key == ord('C'):
        set_gripper(False, True, 0.0)
