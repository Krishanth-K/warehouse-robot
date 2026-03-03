from controller import Robot, Camera, Lidar, Motor
import math
from owlready2 import *
import time

# --- Constants ---
TIME_STEP = 16
MAX_WHEEL_SPEED = 10.0
REDUCED_SPEED = 3.0
STOP_SPEED = 0.0
PROXIMITY_THRESHOLD = 2.5  # Increase detection range for faster robot
SHELF_STOP_THRESHOLD = 1.0  # Stop 1m before the shelf
REASONING_INTERVAL = 30    

# --- Modular Ontology Logic ---
class EthicsEngine:
    def __init__(self, owl_path):
        print(f"DEBUG: Loading ontology {owl_path}...")
        self.onto = get_ontology(owl_path).load()
        with self.onto:
            sync_reasoner_pellet(infer_property_values=True, debug=0)
        print(f"Ontology '{owl_path}' loaded and ready.")
        self.last_behavior = (MAX_WHEEL_SPEED, "NormalSpeed")

    def get_behavior(self, sensor_data, step_count):
        if step_count % REASONING_INTERVAL != 0:
            return self.last_behavior

        dist = sensor_data['lidar_min']
        try:
            with self.onto:
                r = self.onto.Robot("pr2_robot")
                if dist < PROXIMITY_THRESHOLD:
                    p = self.onto.Near("human_proximity")
                else:
                    p = self.onto.Far("human_proximity")
                r.has_proximity = [p]
                
                sync_reasoner_pellet(infer_property_values=True, debug=0)
                
                if self.onto.EmergencyStop in r.is_a:
                    self.last_behavior = (STOP_SPEED, "EmergencyStop")
                elif self.onto.ReducedSpeed in r.is_a:
                    self.last_behavior = (REDUCED_SPEED, "ReducedSpeed")
                else:
                    self.last_behavior = (MAX_WHEEL_SPEED, "NormalSpeed")
        except Exception as e:
            print(f"Ontology Error: {e}")
            
        return self.last_behavior

# --- Robot Setup ---
robot = Robot()
wheel_motors = []
rotation_motors = []

def initialize_devices():
    wheel_names = [
        "fl_caster_l_wheel_joint","fl_caster_r_wheel_joint",
        "fr_caster_l_wheel_joint","fr_caster_r_wheel_joint",
        "bl_caster_l_wheel_joint","bl_caster_r_wheel_joint",
        "br_caster_l_wheel_joint","br_caster_r_wheel_joint"
    ]
    for name in wheel_names:
        m = robot.getDevice(name)
        if m:
            m.setPosition(float("inf"))
            m.setVelocity(0.0)
            wheel_motors.append(m)

    rotation_names = ["fl_caster_rotation_joint", "fr_caster_rotation_joint", "bl_caster_rotation_joint", "br_caster_rotation_joint"]
    for name in rotation_names:
        m = robot.getDevice(name)
        if m:
            m.setPosition(0.0)
            rotation_motors.append(m)

    lidar = robot.getDevice("base_laser")
    if lidar:
        lidar.enable(TIME_STEP)
    
    return lidar

lidar = initialize_devices()
ethics = EthicsEngine("warehouse_ethics.owl")

# --- Main Loop ---
state = "NAVIGATE_TO_SHELF"
step_count = 0

print("DEBUG: Starting main loop...")

while robot.step(TIME_STEP) != -1:
    step_count += 1
    
    # 1. Sensing
    ranges = lidar.getRangeImage() if lidar else []
    valid_ranges = [r for r in ranges if not math.isinf(r) and not math.isnan(r)]
    min_dist = min(valid_ranges) if valid_ranges else 10.0
    sensor_data = {'lidar_min': min_dist}
    
    # 2. Ethical Reasoning
    onto_speed, behavior_name = ethics.get_behavior(sensor_data, step_count)
    
    # 3. Navigation State Machine
    target_speed = 0.0
    
    if state == "NAVIGATE_TO_SHELF":
        # Check if we should stop at the shelf (Threshold-based)
        if min_dist <= SHELF_STOP_THRESHOLD:
            state = "AT_SHELF"
            target_speed = STOP_SPEED
            print(f"Status: Reached shelf (Dist: {min_dist:.2f}m). Stopping.")
        else:
            # Use the speed determined by the ontology
            target_speed = onto_speed

    elif state == "AT_SHELF":
        target_speed = STOP_SPEED

    # 4. Actuation
    for m in wheel_motors:
        m.setVelocity(target_speed)

    # Logging
    if step_count % 100 == 0:
        print(f"Heartbeat: State={state}, MinDist={min_dist:.2f}m, Behavior={behavior_name}, Speed={target_speed:.2f}")

    if behavior_name != "NormalSpeed" and step_count % REASONING_INTERVAL == 0:
        print(f"[ETHICS] Inferred: {behavior_name} (Dist: {min_dist:.2f}m) -> Speed: {target_speed:.2f}")
