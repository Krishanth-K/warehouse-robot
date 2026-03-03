from controller import Robot, Camera, Lidar, Motor, Supervisor
import math
from owlready2 import *
import threading
import time

# --- Constants ---
TIME_STEP = 16
MAX_WHEEL_SPEED = 10.0
REDUCED_SPEED = 3.0
STOP_SPEED = 0.0
PROXIMITY_THRESHOLD = 3.5  
CHECKPOINT_MARGIN = 0.3    # Increased slightly for smoother high-speed transitions

# --- Checkpoints (X, Y) ---
CHECKPOINTS = [
    {"x": 1.0, "y": 0.0, "label": "Warehouse Middle"},
    {"x": 4.5, "y": 0.0, "label": "Shelf Arrival Zone"},
    {"x": 5.2, "y": 0.0, "label": "Final Stop Before Shelf"}
]

# --- Threaded Ontology Logic ---
class EthicsEngine(threading.Thread):
    def __init__(self, owl_path):
        super().__init__()
        self.daemon = True 
        print(f"DEBUG: Loading ontology {owl_path}...")
        self.onto = get_ontology(owl_path).load()
        with self.onto:
            sync_reasoner_pellet(infer_property_values=True, debug=0)
        
        self.human_near = False
        self.inferred_behavior = (MAX_WHEEL_SPEED, "NormalSpeed")
        self.running = True
        self.start()

    def update_human_near(self, status):
        self.human_near = status
        # FAST PATH: Instant speed recovery
        if not status:
            self.inferred_behavior = (MAX_WHEEL_SPEED, "NormalSpeed")

    def run(self):
        while self.running:
            try:
                with self.onto:
                    r = self.onto.search_one(name="pr2_robot")
                    if not r: r = self.onto.Robot("pr2_robot")
                    
                    p_new = self.onto.Near("human_proximity") if self.human_near else self.onto.Far("human_proximity")
                    r.has_proximity = [p_new]
                    sync_reasoner_pellet(infer_property_values=True, debug=0)
                    
                    if self.human_near:
                        if self.onto.EmergencyStop in r.is_a:
                            self.inferred_behavior = (STOP_SPEED, "EmergencyStop")
                        elif self.onto.ReducedSpeed in r.is_a:
                            self.inferred_behavior = (REDUCED_SPEED, "ReducedSpeed")
                        else:
                            self.inferred_behavior = (MAX_WHEEL_SPEED, "NormalSpeed")
                    else:
                        self.inferred_behavior = (MAX_WHEEL_SPEED, "NormalSpeed")
            except Exception: pass
            time.sleep(0.05)

# --- Robot Setup ---
robot = Supervisor()
wheel_motors = []
rotation_motors = []
head_pan_motor = None
camera = None

def initialize_devices():
    global head_pan_motor, camera
    wheel_names = ["fl_caster_l_wheel_joint","fl_caster_r_wheel_joint","fr_caster_l_wheel_joint","fr_caster_r_wheel_joint","bl_caster_l_wheel_joint","bl_caster_r_wheel_joint","br_caster_l_wheel_joint","br_caster_r_wheel_joint"]
    for name in wheel_names:
        m = robot.getDevice(name); m.setPosition(float("inf")); m.setVelocity(0.0)
        wheel_motors.append(m)

    rotation_names = ["fl_caster_rotation_joint", "fr_caster_rotation_joint", "bl_caster_rotation_joint", "br_caster_rotation_joint"]
    for name in rotation_names:
        m = robot.getDevice(name); m.setPosition(0.0)
        rotation_motors.append(m)

    head_pan_motor = robot.getDevice("head_pan_joint")
    camera = robot.getDevice("wide_stereo_r_stereo_camera_sensor")
    camera.enable(TIME_STEP)
    lidar = robot.getDevice("base_laser")
    lidar.enable(TIME_STEP)
    return lidar

lidar = initialize_devices()
self_node = robot.getSelf()
ethics = EthicsEngine("warehouse_ethics.owl")

def is_human_in_view():
    img = camera.getImage()
    if not img: return False
    w, h = camera.getWidth(), camera.getHeight()
    for y in range(h // 3, 2 * h // 3, 10):
        for x in range(w // 3, 2 * w // 3, 10):
            r, g, b = camera.imageGetRed(img, w, x, y), camera.imageGetGreen(img, w, x, y), camera.imageGetBlue(img, w, x, y)
            if (r > 140 and g < 110 and b < 110) or (g > 140 and r < 110 and b < 110):
                return True
    return False

# --- Main Loop ---
checkpoint_index = 0
step_count = 0
currently_tracking = False

print("DEBUG: Starting Continuous Autonomous Navigation...")

while robot.step(TIME_STEP) != -1:
    step_count += 1
    pos = self_node.getPosition()
    
    # 1. Sensing & Tracking
    ranges = lidar.getRangeImage()
    valid_ranges = [r for r in ranges if not math.isinf(r) and not math.isnan(r) and r > 0.1]
    human_in_sight = False
    
    if valid_ranges:
        min_dist = min(valid_ranges)
        idx = ranges.index(min_dist)
        fov = lidar.getFov()
        angle = (idx / (len(ranges)-1)) * fov - (fov / 2.0)
        
        if not currently_tracking:
            head_pan_motor.setPosition(0.0)
            if min_dist < PROXIMITY_THRESHOLD and is_human_in_view():
                currently_tracking = True
                human_in_sight = True
        else:
            head_pan_motor.setPosition(-angle)
            if min_dist < PROXIMITY_THRESHOLD:
                human_in_sight = True
            else:
                currently_tracking = False
                head_pan_motor.setPosition(0.0)
    else:
        currently_tracking = False
        head_pan_motor.setPosition(0.0)

    # 2. Update Ethics
    ethics.update_human_near(human_in_sight)
    limit_speed, behavior_name = ethics.inferred_behavior
    
    # 3. Continuous Checkpoint Navigation Logic
    target_speed = STOP_SPEED
    
    if checkpoint_index < len(CHECKPOINTS):
        target = CHECKPOINTS[checkpoint_index]
        dist_to_target = math.sqrt((target['x'] - pos[0])**2 + (target['y'] - pos[1])**2)
        
        if dist_to_target < CHECKPOINT_MARGIN:
            print(f"STATUS: Passed {target['label']}.")
            checkpoint_index += 1
            # NO STOP - Continue to next waypoint immediately
            if checkpoint_index < len(CHECKPOINTS):
                target_speed = limit_speed 
            else:
                target_speed = STOP_SPEED
        else:
            target_speed = limit_speed
    else:
        target_speed = STOP_SPEED

    # 4. Actuation
    for m in wheel_motors:
        m.setVelocity(target_speed)

    if step_count % 100 == 0:
        print(f"POS: ({pos[0]:.2f}, {pos[1]:.2f}) | WP: {checkpoint_index}/{len(CHECKPOINTS)} | Ethics: {behavior_name} | Speed: {target_speed:.1f}")
