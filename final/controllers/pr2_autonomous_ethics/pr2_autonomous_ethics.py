from controller import Robot, Camera, Lidar, Motor, Supervisor
import math
from owlready2 import *
import threading
import time

# --- Actuation Constants ---
TIME_STEP = 16
SPEED_NORMAL = 15.0
SPEED_REDUCED = 3.0
SPEED_STOP = 0.0
SPEED_ROTATE = 2.0
PROXIMITY_THRESHOLD = 4.0  
CHECKPOINT_MARGIN = 0.3    
CLEARANCE_TIME = 1.5       # Seconds to stay slow after human passes the side

# --- Navigation Waypoints ---
WAYPOINTS_OUTBOUND = [
    {"x": 1.0, "y": 0.0, "label": "Warehouse Middle"},
    {"x": 5.2, "y": 0.0, "label": "Final Stop Before Shelf"}
]
WAYPOINTS_INBOUND = [
    {"x": 1.0, "y": 0.0, "label": "Warehouse Middle (Return)"},
    {"x": -4.2, "y": 0.0, "label": "Starting Position"}
]

# --- Helper Functions ---
def normalize_angle(angle):
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle

# --- Formal NEP-Compatible Ethics Engine ---
class FormalEthicsEngine(threading.Thread):
    def __init__(self, owl_path):
        super().__init__()
        self.daemon = True 
        self.onto = get_ontology(owl_path).load()
        with self.onto:
            sync_reasoner_pellet(infer_property_values=True, debug=0)
        
        self.human_near = False
        self.target_velocity = SPEED_NORMAL
        self.behavior_desc = "Normal Operation"
        self.running = True
        self.start()

    def update_human_proximity(self, is_near):
        self.human_near = is_near
        if not is_near:
            self.target_velocity = SPEED_NORMAL
            self.behavior_desc = "Normal Velocity (Clear Path)"

    def run(self):
        while self.running:
            try:
                with self.onto:
                    r = self.onto["pr2_robot"]
                    red_act = self.onto["reduced_velocity_action"]
                    if self.human_near:
                        s = self.onto.HumanNearSituation("current_human_situation")
                        r.recognizes = [s]
                    else:
                        r.recognizes = []
                    sync_reasoner_pellet(infer_property_values=True, debug=0)
                    if self.human_near:
                        if red_act in r.executes:
                            self.target_velocity = SPEED_REDUCED
                            self.behavior_desc = "NEP: Executing ReducedVelocityAction (Inferred)"
                        else:
                            self.behavior_desc = "NEP: Reasoning..."
                    else:
                        self.target_velocity = SPEED_NORMAL
                        self.behavior_desc = "NEP: Normal Operation"
            except Exception: pass
            time.sleep(0.1)

# --- Robot Setup ---
robot = Supervisor()
wheel_motors = []
rotation_motors = []
arm_motors = {}
torso_motor = None
left_finger_motor = None
head_pan_motor = None
camera = None

def initialize_devices():
    global head_pan_motor, camera, torso_motor, left_finger_motor
    wheel_names = ["fl_caster_l_wheel_joint","fl_caster_r_wheel_joint","fr_caster_l_wheel_joint","fr_caster_r_wheel_joint","bl_caster_l_wheel_joint","bl_caster_r_wheel_joint","br_caster_l_wheel_joint","br_caster_r_wheel_joint"]
    for name in wheel_names:
        m = robot.getDevice(name); m.setPosition(float("inf")); m.setVelocity(0.0); wheel_motors.append(m)
    rotation_names = ["fl_caster_rotation_joint", "fr_caster_rotation_joint", "bl_caster_rotation_joint", "br_caster_rotation_joint"]
    for name in rotation_names:
        m = robot.getDevice(name); m.setPosition(0.0); rotation_motors.append(m)
    torso_motor = robot.getDevice("torso_lift_joint")
    arm_names = ["r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", "r_wrist_roll_joint"]
    for name in arm_names: arm_motors[name] = robot.getDevice(name)
    left_finger_motor = robot.getDevice("l_finger_gripper_motor::l_finger")
    head_pan_motor = robot.getDevice("head_pan_joint")
    camera = robot.getDevice("wide_stereo_r_stereo_camera_sensor"); camera.enable(TIME_STEP)
    lidar = robot.getDevice("base_laser"); lidar.enable(TIME_STEP)
    return lidar

lidar = initialize_devices()
self_node = robot.getSelf()
ethics = FormalEthicsEngine("formal_warehouse_ethics.owl")

def is_human_in_view():
    img = camera.getImage()
    if not img: return False
    w, h = camera.getWidth(), camera.getHeight()
    for y in range(h // 4, 3 * h // 4, 15):
        for x in range(0, w, 15):
            r, g, b = camera.imageGetRed(img, w, x, y), camera.imageGetGreen(img, w, x, y), camera.imageGetBlue(img, w, x, y)
            if (r > 140 and g < 110 and b < 110) or (g > 140 and r < 110 and b < 110): return True
    return False

def set_caster_angles(fl, fr, bl, br):
    rotation_motors[0].setPosition(fl); rotation_motors[1].setPosition(fr)
    rotation_motors[2].setPosition(bl); rotation_motors[3].setPosition(br)

# --- Main Loop ---
state = "NAV_OUTBOUND"
checkpoint_index = 0
step_count = 0
currently_tracking = False
is_rotating = False
clearance_timer = 0.0
pickup_sequence_start = 0

print("DEBUG: Starting Robust Path-Aware Mission...")

while robot.step(TIME_STEP) != -1:
    step_count += 1
    pos = self_node.getPosition()
    rot = self_node.getOrientation()
    yaw = math.atan2(rot[3], rot[0]) 
    
    # 1. Sensing & Clearance Tracking
    ranges = lidar.getRangeImage()
    valid_ranges = [r for r in ranges if not math.isinf(r) and not math.isnan(r) and r > 0.1]
    
    lidar_sees_human = False
    if valid_ranges:
        min_dist = min(valid_ranges); idx = ranges.index(min_dist); fov = lidar.getFov()
        angle = (idx / (len(ranges)-1)) * fov - (fov / 2.0)
        
        if not currently_tracking:
            head_pan_motor.setPosition(0.0)
            if min_dist < PROXIMITY_THRESHOLD and abs(angle) < 0.5 and is_human_in_view():
                currently_tracking = True; lidar_sees_human = True
        else:
            head_pan_motor.setPosition(angle)
            lidar_sees_human = True
            # Check if human has moved behind the side-arc (1.45 rad is ~83 deg)
            if min_dist > PROXIMITY_THRESHOLD or abs(angle) > 1.45:
                currently_tracking = False; lidar_sees_human = False; head_pan_motor.setPosition(0.0)
                clearance_timer = CLEARANCE_TIME # Start 1.5s countdown
    else:
        if currently_tracking:
            currently_tracking = False; head_pan_motor.setPosition(0.0); clearance_timer = CLEARANCE_TIME
        lidar_sees_human = False

    if clearance_timer > 0:
        clearance_timer -= (TIME_STEP / 1000.0)
    
    # Final Ethical Status: Slow if seen OR if still in clearance buffer
    human_active = lidar_sees_human or (clearance_timer > 0)
    ethics.update_human_proximity(human_active)
    limit_speed, behavior_name = ethics.target_velocity, ethics.behavior_desc
    
    # 2. Navigation State Machine
    target_speed = SPEED_STOP; current_rot_speed = 0.0
    active_waypoints = WAYPOINTS_OUTBOUND if state == "NAV_OUTBOUND" else WAYPOINTS_INBOUND
    
    if state in ["NAV_OUTBOUND", "NAV_INBOUND"]:
        if checkpoint_index < len(active_waypoints):
            wp = active_waypoints[checkpoint_index]
            dist_to_wp = math.sqrt((wp['x'] - pos[0])**2 + (wp['y'] - pos[1])**2)
            target_yaw = math.atan2(wp['y'] - pos[1], wp['x'] - pos[0])
            angle_error = normalize_angle(target_yaw - yaw)
            
            # Hysteresis Logic for Rotation
            if not is_rotating:
                if abs(angle_error) > 0.25: is_rotating = True
            else:
                if abs(angle_error) < 0.05: is_rotating = False

            if dist_to_wp < CHECKPOINT_MARGIN:
                checkpoint_index += 1; is_rotating = False
            elif is_rotating:
                set_caster_angles(3*math.pi/4, math.pi/4, -3*math.pi/4, -math.pi/4)
                current_rot_speed = SPEED_ROTATE if angle_error > 0 else -SPEED_ROTATE
                target_speed = SPEED_STOP
            else:
                set_caster_angles(0, 0, 0, 0); target_speed = limit_speed
        else:
            if state == "NAV_OUTBOUND": state = "PICKING_UP"; pickup_sequence_start = step_count
            else: state = "FINISHED"

    elif state == "PICKING_UP":
        target_speed = SPEED_STOP
        elapsed = step_count - pickup_sequence_start
        if elapsed == 1: torso_motor.setPosition(0.2)
        elif elapsed == 50: arm_motors["r_shoulder_lift_joint"].setPosition(-0.5); arm_motors["r_elbow_flex_joint"].setPosition(-1.2)
        elif elapsed == 100: left_finger_motor.setPosition(0.2)
        elif elapsed == 150: left_finger_motor.setPosition(0.0)
        elif elapsed == 200: arm_motors["r_shoulder_lift_joint"].setPosition(-0.8)
        elif elapsed == 250: state = "NAV_INBOUND"; checkpoint_index = 0

    # 3. Actuation
    if is_rotating:
        for m in wheel_motors: m.setVelocity(current_rot_speed)
    else:
        for m in wheel_motors: m.setVelocity(target_speed)

    if step_count % 100 == 0:
        status = "CLEARANCE" if (clearance_timer > 0 and not lidar_sees_human) else behavior_name
        print(f"STATE: {state} | Ethics: {status} | Speed: {target_speed:.1f}")
