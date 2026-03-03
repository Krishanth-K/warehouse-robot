from controller import Supervisor, Camera, Lidar, Motor
import math, cv2, socket
import numpy as np

TIME_STEP = 16
MAX_WHEEL_SPEED = 6.0
ROTATE_SPEED = 2.5
QR_SKIP_N = 6

MOVE_1_DISTANCE = 3.0
MOVE_2_DISTANCE = 6.3
TURN_ANGLE = math.pi
HUMAN_CONFIRM_FRAMES = 2
HUMAN_LOST_FRAMES = 10
OBJECT_STOP_DISTANCE = 2.0

DELAY_BEFORE_TURN = 1.0
delay_start_time = None

robot = Supervisor()

# ===== Supervisor nodes =====
pr2_node = robot.getSelf()
cube_node = robot.getFromDef("cube_with_stands")

cyl_left = robot.getFromDef("cyl_left")
cyl_right = robot.getFromDef("cyl_right")

left_gripper = robot.getFromDef("l_gripper_tool_frame")
right_gripper = robot.getFromDef("r_gripper_tool_frame")

cube_locked = False
relative_offset = None

# ===== ORBIT SETTINGS =====
orbit_angle = 0.0
orbit_start_angle = 0.0
orbit_radius = 0.0
current_offset_xy = [0.0, 0.0]

ORBIT_SPEED = -0.017
ORBIT_TARGET_ROT = math.pi

orbit_mode = True

ORBIT_START_DELAY = 1.3
ORBIT_END_DELAY = 1.0

orbit_start_timer = None
orbit_end_timer = None
orbit_started = False
orbit_finished = False

# ===== NEW: CUBE X-AXIS TRAVEL =====
cube_x_mode = False
cube_x_start = None
CUBE_X_DISTANCE = 10.0
CUBE_X_SPEED = 0.04   # adjust speed here

wheel_motors = []
rotation_motors = []

left_finger_motor = None
right_finger_motor = None

right_head_cam = None
base_lidar = None

phase = 0
start_pos = None
start_yaw = None

qr_frame_skip = 0
human_present = False
gripper_opened_at_end = False

human_seen_frames = 0
human_lost_frames = 0

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(("127.0.0.1", 5005))
sock.setblocking(False)


def get_robot_position():
    return pr2_node.getPosition()

def get_robot_yaw():
    o = pr2_node.getOrientation()
    return math.atan2(o[2], o[0])

def distance_from_start():
    return math.dist(start_pos, get_robot_position())

def angle_turned():
    return abs(get_robot_yaw() - start_yaw)


def send_frame(camera):
    try:
        img = camera.getImage()
        w, h = camera.getWidth(), camera.getHeight()
        image = np.frombuffer(img, np.uint8).reshape((h, w, 4))
        image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        _, enc = cv2.imencode(".jpg", image)
        data = enc.tobytes()
        sock.sendall(len(data).to_bytes(4, "big"))
        sock.sendall(data)
        size_data = sock.recv(4)
        if not size_data:
            return None
        size = int.from_bytes(size_data, "big")
        buf = b""
        while len(buf) < size:
            part = sock.recv(size - len(buf))
            if not part:
                return None
            buf += part
        return buf.decode()
    except BlockingIOError:
        return None

def get_front_object_distance():
    ranges = base_lidar.getRangeImage()
    c = len(ranges)//2
    w = int(len(ranges)*0.25)
    front = ranges[c-w:c+w]
    vals = [r for r in front if not math.isinf(r)]
    if not vals:
        return None
    return min(vals)

def initialize_devices():
    global right_head_cam, base_lidar
    global left_finger_motor, right_finger_motor

    for n in [
        "fl_caster_l_wheel_joint","fl_caster_r_wheel_joint",
        "fr_caster_l_wheel_joint","fr_caster_r_wheel_joint",
        "bl_caster_l_wheel_joint","bl_caster_r_wheel_joint",
        "br_caster_l_wheel_joint","br_caster_r_wheel_joint"
    ]:
        m = robot.getDevice(n)
        m.setPosition(float("inf"))
        m.setVelocity(0.0)
        wheel_motors.append(m)

    for n in [
        "fl_caster_rotation_joint","fr_caster_rotation_joint",
        "bl_caster_rotation_joint","br_caster_rotation_joint"
    ]:
        rotation_motors.append(robot.getDevice(n))

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

initialize_devices()

while robot.step(TIME_STEP) != -1:

    # ===== CUBE ORBIT FOLLOW =====
    if cube_locked and cube_node:
        pr2_pos = pr2_node.getPosition()
        now = robot.getTime()

        if not orbit_started:
            if orbit_start_timer is not None and (now - orbit_start_timer) >= ORBIT_START_DELAY:
                orbit_started = True

        if orbit_started and orbit_mode:
            orbit_angle += ORBIT_SPEED
            ox = orbit_radius * math.cos(orbit_angle)
            oy = orbit_radius * math.sin(orbit_angle)
            current_offset_xy = [ox, oy]

            if abs(orbit_angle - orbit_start_angle) >= ORBIT_TARGET_ROT:
                orbit_mode = False
                orbit_finished = True
                orbit_end_timer = now

        ox = current_offset_xy[0]
        oy = current_offset_xy[1]

        cube_node.getField("translation").setSFVec3f([
            pr2_pos[0] + ox,
            pr2_pos[1] + oy,
            pr2_pos[2] + relative_offset[2]
        ])

    # ===== DETACHED CUBE X AXIS MOTION =====
    if cube_x_mode and cube_node:
        pos = cube_node.getPosition()

        if cube_x_start is None:
            cube_x_start = pos[0]

        if (pos[0] - cube_x_start) < CUBE_X_DISTANCE:
            cube_node.getField("translation").setSFVec3f([
                pos[0] + CUBE_X_SPEED,
                pos[1],
                pos[2]
            ])

    # ===== CYLINDERS FOLLOW GRIPPERS =====
    if cyl_left and left_gripper:
        cyl_left.getField("translation").setSFVec3f(left_gripper.getPosition())
    if cyl_right and right_gripper:
        cyl_right.getField("translation").setSFVec3f(right_gripper.getPosition())

    if phase == 0:
        set_gripper(True, True, 0.0)
        set_gripper(False, True, 0.0)
        start_pos = get_robot_position()
        phase = 1

    elif phase == 1:
        set_caster_angles(0,0,0,0)
        set_wheels(MAX_WHEEL_SPEED)
        if distance_from_start() >= MOVE_1_DISTANCE:
            stop_wheels()
            set_gripper(True, False, 20.0)
            set_gripper(False, False, 20.0)
            delay_start_time = robot.getTime()
            phase = 1.5

    elif phase == 1.5:
        if not cube_locked and cube_node:
            cube_pos = cube_node.getPosition()
            pr2_pos = pr2_node.getPosition()

            relative_offset = [
                cube_pos[0]-pr2_pos[0],
                cube_pos[1]-pr2_pos[1],
                cube_pos[2]-pr2_pos[2]
            ]

            orbit_radius = math.sqrt(relative_offset[0]**2 + relative_offset[1]**2)
            orbit_angle = math.atan2(relative_offset[1], relative_offset[0])
            orbit_start_angle = orbit_angle
            current_offset_xy = [relative_offset[0], relative_offset[1]]
            orbit_start_timer = robot.getTime()
            cube_locked = True

        if robot.getTime() - delay_start_time >= DELAY_BEFORE_TURN:
            start_yaw = get_robot_yaw()
            phase = 2

    elif phase == 2:
        set_caster_angles(
            3*math.pi/4, math.pi/4,
           -3*math.pi/4, -math.pi/4
        )
        set_wheels(-ROTATE_SPEED)
        if angle_turned() >= TURN_ANGLE:
            stop_wheels()
            start_pos = get_robot_position()
            phase = 3

    elif phase == 3:
        qr_frame_skip += 1
        if qr_frame_skip % (QR_SKIP_N*2) == 0:
            vision_result = send_frame(right_head_cam)
            if vision_result:
                detected = (vision_result.split(";")[0] == "HUMAN")
                if detected:
                    human_seen_frames += 1
                    human_lost_frames = 0
                else:
                    human_lost_frames += 1
                    human_seen_frames = 0
                if human_seen_frames >= HUMAN_CONFIRM_FRAMES:
                    human_present = True
                if human_lost_frames >= HUMAN_LOST_FRAMES:
                    human_present = False

        front_dist = get_front_object_distance()

        if human_present and front_dist is not None and front_dist <= OBJECT_STOP_DISTANCE:
            stop_wheels()
        else:
            set_caster_angles(0,0,0,0)
            set_wheels(MAX_WHEEL_SPEED)
            if distance_from_start() >= MOVE_2_DISTANCE:
                stop_wheels()
                if not gripper_opened_at_end:
                    set_gripper(True, True, 0.0)
                    set_gripper(False, True, 0.0)
                    gripper_opened_at_end = True

                # ⭐ DETACH MESH FROM PR2
                cube_locked = False
                orbit_mode = False
                cube_x_mode = True

                phase = 4
