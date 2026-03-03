from controller import Robot, Camera, Lidar, Motor
import math
from owlready2 import *
import time

# --- Constants ---
TIME_STEP = 16
MAX_WHEEL_SPEED = 3.0
REDUCED_SPEED = 1.0
STOP_SPEED = 0.0
PROXIMITY_THRESHOLD = 2.0  # meters

# --- Pathing Constants ---
TARGET_X = 5.8  # Shelf distance in ethics_test.wbt

# --- Modular Ontology Logic ---
class EthicsEngine:
    def __init__(self, owl_path):
        self.onto = get_ontology(owl_path).load()
        with self.onto:
            # Pre-sync to verify the file
            sync_reasoner_pellet(infer_property_values=True, debug=0)
        print(f"Ontology '{owl_path}' loaded and ready.")

    def get_behavior(self, sensor_data):
        """
        This method is designed to be easily swappable when a more 
        complex ontology file is provided.
        """
        dist = sensor_data['lidar_min']
        
        with self.onto:
            # 1. Map Sensors to Ontology Individuals
            # (Using a generic Robot/Proximity mapping for now)
            r = self.onto.Robot("pr2_robot")
            if dist < PROXIMITY_THRESHOLD:
                p = self.onto.Near("human_proximity")
            else:
                p = self.onto.Far("human_proximity")
            
            r.has_proximity = [p]
            
            # 2. Run Reasoning
            sync_reasoner_pellet(infer_property_values=True, debug=0)
            
            # 3. Extract Inferred Behavior
            # If the new ontology uses different classes, only this section needs changing
            if self.onto.EmergencyStop in r.is_a:
                return STOP_SPEED, "EmergencyStop"
            elif self.onto.ReducedSpeed in r.is_a:
                return REDUCED_SPEED, "ReducedSpeed"
            else:
                return MAX_WHEEL_SPEED, "NormalSpeed"

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
        m.setPosition(float("inf"))
        m.setVelocity(0.0)
        wheel_motors.append(m)

    rotation_names = ["fl_caster_rotation_joint", "fr_caster_rotation_joint", "bl_caster_rotation_joint", "br_caster_rotation_joint"]
    for name in rotation_names:
        rotation_motors.append(robot.getDevice(name))

    lidar = robot.getDevice("base_laser")
    lidar.enable(TIME_STEP)
    return lidar

lidar = initialize_devices()

# Initialize the Ethics Engine
# Note: Swap 'warehouse_ethics.owl' for the new file when available
ethics = EthicsEngine("warehouse_ethics.owl")

# --- Main Loop ---
state = "DRIVING_FORWARD"
start_time = robot.getTime()

while robot.step(TIME_STEP) != -1:
    # 1. Sensing
    ranges = lidar.getRangeImage()
    min_dist = min(ranges) if ranges else 10.0
    sensor_data = {'lidar_min': min_dist}
    
    # 2. Ethical Reasoning
    target_speed, behavior_name = ethics.get_behavior(sensor_data)
    
    # 3. Autonomous State Machine (Simple Straight Path)
    current_time = robot.getTime() - start_time
    
    # Logic: Drive for 10 seconds (approx distance to shelf) then stop
    if state == "DRIVING_FORWARD":
        if current_time > 10.0:
            state = "PICKING_UP"
            print("Status: Reached shelf. Starting pick-up sequence.")
    
    if state == "PICKING_UP":
        target_speed = STOP_SPEED
        # (Gripper code will go here next)
    
    # 4. Actuation
    for m in wheel_motors:
        m.setVelocity(target_speed)

    # Logging as per requirement
    if behavior_name != "NormalSpeed":
        print(f"[ETHICS] Inferred: {behavior_name} (Dist: {min_dist:.2f}m) -> Speed: {target_speed}")
