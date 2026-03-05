from controller import Robot, Camera, Lidar, Motor, Supervisor
import math
from owlready2 import *
import threading
import time

# --- Actuation Constants ---
TIME_STEP = 16
SPEED_NORMAL = 10.0
SPEED_REDUCED = 3.0
SPEED_STOP = 0.0
SPEED_ROTATE = 2.0
PROXIMITY_THRESHOLD = 4.0  
CHECKPOINT_MARGIN = 0.3    
CLEARANCE_TIME = 1.5       

# --- Task Configuration ---
TARGET_BOX_HEIGHT = 0.62  # Known height of the box in the world

# --- Navigation Waypoints ---
WAYPOINTS_OUTBOUND = [
    {"x": 1.0, "y": 0.0, "label": "Warehouse Middle"},
    {"x": 5.25, "y": 0.0, "label": "Final Stop Before Shelf"}
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

def calculate_reach_params(box_height):
    """Calculates arm positions using a V-down pose for shelf clearance."""
    # PR2 constants (approximate)
    SHOULDER_BASE_Z = 0.73
    ELBOW_FLEX = -1.2 
    
    # 1. Torso Lift: Try to keep shoulder ~15cm above box
    target_torso = box_height - SHOULDER_BASE_Z + 0.15
    torso_pos = max(0.0, min(0.3, target_torso))
    
    # 2. Shoulder & Elbow Geometry
    shoulder_z = SHOULDER_BASE_Z + torso_pos
    dz = box_height - shoulder_z # vertical distance to box
    dx = 0.52 # horizontal reach
    
    angle_to_box = math.atan2(-dz, dx) # positive if box is below shoulder
    # V-down pose: shoulder points down, forearm points back up
    # midpoint of upper/forearm should align with box
    shoulder_lift = angle_to_box - (ELBOW_FLEX / 2.0)
    shoulder_lift = max(-0.5, min(1.2, shoulder_lift))
    
    # 3. Wrist Flex to stay level
    # Forearm angle is shoulder_lift + ELBOW_FLEX. We want it 0.
    # wrist_flex is relative to forearm. PR2 limit is approx [-2.1, 0.0].
    wrist_flex = -(shoulder_lift + ELBOW_FLEX)
    wrist_flex = max(-2.0, min(0.0, wrist_flex))
    
    retract_shoulder = shoulder_lift - 0.25 # Lift up for clearance
    
    return torso_pos, shoulder_lift, ELBOW_FLEX, wrist_flex, retract_shoulder

# --- EWHR Ethics Engine ---
class EWHREthicsEngine(threading.Thread):
    def __init__(self, owl_path):
        super().__init__()
        self.daemon = True 
        self.onto = get_ontology(owl_path).load()
        
        self.human_near = False
        self.mission_situations = set() # Set of active mission situation names
        self.target_velocity = SPEED_NORMAL
        self.behavior_desc = "EWHR: Initializing"
        self.running = True
        self.start()

    def update_human_proximity(self, is_near):
        self.human_near = is_near

    def set_active_situations(self, sit_names):
        """Pass a list of mission situations that are currently active."""
        self.mission_situations = set(sit_names)

    def run(self):
        while self.running:
            try:
                # 1. Reasoning Phase (CPU Intensive - do less frequently)
                with self.onto:
                    robot_ind = self.onto.search_one(iri="*PR2_001")
                    if robot_ind:
                        # Clear recognitions definitively
                        robot_ind.recognizes = []
                        # CRITICAL: Clear inferred actions so reasoner can re-evaluate from scratch
                        robot_ind.executes = []

                        # Assert Situations
                        for sit_name in self.mission_situations:
                            sit = self.onto.search_one(iri=f"*{sit_name}")
                            if sit: robot_ind.recognizes.append(sit)
                        
                        proximity_sit = self.onto.search_one(iri="*Human_Proximity_Sit")
                        if self.human_near and proximity_sit:
                            robot_ind.recognizes.append(proximity_sit)
                    
                    sync_reasoner_pellet(infer_property_values=True, debug=0)
                    
                    # 2. Update Phase (Thread-safe)
                    if robot_ind:
                        inferred_actions = [a.name for a in robot_ind.executes]
                        self.behavior_desc = f"EWHR Actions: {', '.join(inferred_actions) if inferred_actions else 'None'}"
                        
                        reduce_speed_act = self.onto.search_one(iri="*Set_Motor_Velocity_0.5")
                        if reduce_speed_act in robot_ind.executes:
                            self.target_velocity = SPEED_REDUCED
                        else:
                            self.target_velocity = SPEED_NORMAL
                            
            except Exception as e: pass
            time.sleep(0.5) # Reduced frequency to save CPU for Webots physics

# --- Robot Setup ---
robot = Supervisor()
wheel_motors = []
rotation_motors = []
arm_motors = {}
torso_motor = None
r_finger_motor = None
l_finger_motor = None
head_pan_motor = None
camera = None

def get_motor(name):
    m = robot.getDevice(name)
    if not m: print(f"WARNING: Motor '{name}' not found!")
    return m

def initialize_devices():
    global head_pan_motor, camera, torso_motor, r_finger_motor, l_finger_motor
    # Wheels
    wheel_names = ["fl_caster_l_wheel_joint","fl_caster_r_wheel_joint","fr_caster_l_wheel_joint","fr_caster_r_wheel_joint","bl_caster_l_wheel_joint","bl_caster_r_wheel_joint","br_caster_l_wheel_joint","br_caster_r_wheel_joint"]
    for name in wheel_names:
        m = get_motor(name)
        if m: m.setPosition(float("inf")); m.setVelocity(0.0); wheel_motors.append(m)
    # Casters
    rotation_names = ["fl_caster_rotation_joint", "fr_caster_rotation_joint", "bl_caster_rotation_joint", "br_caster_rotation_joint"]
    for name in rotation_names:
        m = get_motor(name)
        if m: m.setPosition(0.0); rotation_motors.append(m)
    # Torso & Arms
    torso_motor = get_motor("torso_lift_joint")
    
    # Initialize BOTH arms to prevent collisions
    for side in ['r', 'l']:
        joints = ["_shoulder_pan_joint", "_shoulder_lift_joint", "_upper_arm_roll_joint", "_elbow_flex_joint", "_forearm_roll_joint", "_wrist_flex_joint", "_wrist_roll_joint"]
        for j in joints:
            name = side + j
            m = get_motor(name)
            if m: arm_motors[name] = m

    r_finger_motor = get_motor("r_finger_gripper_motor::l_finger")
    l_finger_motor = get_motor("l_finger_gripper_motor::l_finger")
    head_pan_motor = get_motor("head_pan_joint")
    camera = robot.getDevice("wide_stereo_r_stereo_camera_sensor"); camera.enable(TIME_STEP)
    lidar = robot.getDevice("base_laser"); lidar.enable(TIME_STEP)
    return lidar

def tuck_arms():
    """Moves arms to a safe position to avoid hitting shelves."""
    # Tuck Left Arm (completely out of the way)
    if "l_shoulder_lift_joint" in arm_motors: arm_motors["l_shoulder_lift_joint"].setPosition(1.2)
    if "l_elbow_flex_joint" in arm_motors: arm_motors["l_elbow_flex_joint"].setPosition(-2.0)
    if "l_shoulder_pan_joint" in arm_motors: arm_motors["l_shoulder_pan_joint"].setPosition(0.1)
    
    # Initial Right Arm Tuck
    if "r_shoulder_lift_joint" in arm_motors: arm_motors["r_shoulder_lift_joint"].setPosition(1.2)
    if "r_elbow_flex_joint" in arm_motors: arm_motors["r_elbow_flex_joint"].setPosition(-2.0)

lidar = initialize_devices()
tuck_arms()
self_node = robot.getSelf()
ethics = EWHREthicsEngine("EWHR_integrated.owl")

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
    if len(rotation_motors) == 4:
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

print(f"DEBUG: Starting Mission. Target Height: {TARGET_BOX_HEIGHT}m")

while robot.step(TIME_STEP) != -1:
    step_count += 1
    pos = self_node.getPosition()
    rot = self_node.getOrientation()
    yaw = math.atan2(rot[3], rot[0]) 
    
    # 1. Sensing & Tracking
    ranges = lidar.getRangeImage()
    valid_ranges = [r for r in ranges if not math.isinf(r) and not math.isnan(r) and r > 0.1]
    lidar_sees_human = False
    
    if valid_ranges:
        min_dist = min(valid_ranges); idx = ranges.index(min_dist); fov = lidar.getFov()
        angle = (idx / (len(ranges)-1)) * fov - (fov / 2.0)
        if not currently_tracking:
            if head_pan_motor: head_pan_motor.setPosition(0.0)
            if min_dist < PROXIMITY_THRESHOLD and abs(angle) < 0.5 and is_human_in_view():
                currently_tracking = True; lidar_sees_human = True
        else:
            if head_pan_motor: head_pan_motor.setPosition(angle)
            lidar_sees_human = True
            if min_dist > PROXIMITY_THRESHOLD or abs(angle) > 1.45:
                currently_tracking = False; lidar_sees_human = False
                if head_pan_motor: head_pan_motor.setPosition(0.0)
                clearance_timer = CLEARANCE_TIME
    else:
        if currently_tracking:
            currently_tracking = False; lidar_sees_human = False; clearance_timer = CLEARANCE_TIME
            if head_pan_motor: head_pan_motor.setPosition(0.0)

    if clearance_timer > 0: clearance_timer -= (TIME_STEP / 1000.0)
    ethics.update_human_proximity(lidar_sees_human or (clearance_timer > 0))
    limit_speed, behavior_name = ethics.target_velocity, ethics.behavior_desc
    
    # 2. State Machine & Ontological Synchronization
    target_speed = SPEED_STOP; current_rot_speed = 0.0
    active_waypoints = WAYPOINTS_OUTBOUND if state == "NAV_OUTBOUND" else WAYPOINTS_INBOUND
    
    # Verify Situations based on State
    if state == "NAV_OUTBOUND":
        if checkpoint_index == 0: ethics.set_active_situations(["At_Base_Station_Sit"])
        else: ethics.set_active_situations(["At_Base_Station_Sit"]) # Could add Movement_To specifically
    elif state == "PICKING_UP":
        elapsed = step_count - pickup_sequence_start
        if elapsed < 150: ethics.set_active_situations(["Box_Lift_Ready_Sit"])
        else: ethics.set_active_situations(["Holding_Box_Sit"])
    elif state == "NAV_INBOUND":
        ethics.set_active_situations(["Holding_Box_Sit"])
    elif state == "FINISHED":
        ethics.set_active_situations(["Mission_Complete_Sit"])

    if state in ["NAV_OUTBOUND", "NAV_INBOUND"]:
        if checkpoint_index < len(active_waypoints):
            wp = active_waypoints[checkpoint_index]
            dist_to_wp = math.sqrt((wp['x'] - pos[0])**2 + (wp['y'] - pos[1])**2)
            target_yaw = math.atan2(wp['y'] - pos[1], wp['x'] - pos[0])
            angle_error = normalize_angle(target_yaw - yaw)
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
        # Generalized Height Sequence
        torso_pos, shoulder_pos, elbow_pos, wrist_pos, retract_shoulder = calculate_reach_params(TARGET_BOX_HEIGHT)
        
        if elapsed == 1:
            print(f"PICKUP: Target height {TARGET_BOX_HEIGHT}m. Lifting Torso to {torso_pos:.2f}..."); 
            if torso_motor: torso_motor.setPosition(torso_pos)
        elif elapsed == 50:
            print(f"PICKUP: Deploying Arm (S:{shoulder_pos:.2f}, E:{elbow_pos:.2f}, W:{wrist_pos:.2f})...");
            if "r_shoulder_lift_joint" in arm_motors: arm_motors["r_shoulder_lift_joint"].setPosition(shoulder_pos)
            if "r_elbow_flex_joint" in arm_motors: arm_motors["r_elbow_flex_joint"].setPosition(elbow_pos)
            if "r_wrist_flex_joint" in arm_motors: arm_motors["r_wrist_flex_joint"].setPosition(wrist_pos)
        elif elapsed == 100:
            print("PICKUP: Opening Gripper...");
            if r_finger_motor: r_finger_motor.setPosition(0.5)
        elif elapsed == 150:
            print("PICKUP: Closing Gripper...");
            if r_finger_motor: r_finger_motor.setPosition(0.0)
        elif elapsed == 200:
            print(f"PICKUP: Lifting to {retract_shoulder:.2f} and Retracting...");
            if "r_shoulder_lift_joint" in arm_motors: arm_motors["r_shoulder_lift_joint"].setPosition(retract_shoulder)
        elif elapsed == 250:
            print("PICKUP: Complete. Initiating Return."); state = "NAV_INBOUND"; checkpoint_index = 0

    # 3. Actuation
    for m in wheel_motors:
        m.setVelocity(current_rot_speed if is_rotating else target_speed)

    if step_count % 100 == 0:
        status = "CLEARANCE" if (clearance_timer > 0 and not lidar_sees_human) else behavior_name
        # Log both target and limit to verify restoration
        print(f"STATE: {state} | Ethics: {status} | CurrentV: {target_speed:.1f} | Limit: {limit_speed:.1f}")
