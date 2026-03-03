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

# --- Load Ontology ---
onto = get_ontology("warehouse_ethics.owl").load()
with onto:
    sync_reasoner_pellet(infer_property_values=True, debug=0)

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

    camera = robot.getDevice("wide_stereo_r_stereo_camera_sensor")
    camera.enable(TIME_STEP)

    lidar = robot.getDevice("base_laser")
    lidar.enable(TIME_STEP)
    return camera, lidar

camera, lidar = initialize_devices()

# --- Ontology Reasoning Function ---
def reason_behavior(distance_to_human):
    with onto:
        # Create/Update Individuals
        r1 = onto.Robot("pr2_robot")
        if distance_to_human < PROXIMITY_THRESHOLD:
            p1 = onto.Near("human_proximity")
        else:
            p1 = onto.Far("human_proximity")
        
        r1.has_proximity = [p1]
        
        # Run Reasoner
        sync_reasoner_pellet(infer_property_values=True, debug=0)
        
        # Check inferred behavior
        if onto.EmergencyStop in r1.is_a:
            return STOP_SPEED, "EmergencyStop"
        elif onto.ReducedSpeed in r1.is_a:
            return REDUCED_SPEED, "ReducedSpeed"
        else:
            return MAX_WHEEL_SPEED, "NormalSpeed"

# --- Main Loop ---
while robot.step(TIME_STEP) != -1:
    # 1. Sensing
    ranges = lidar.getRangeImage()
    min_dist = min(ranges) if ranges else 10.0
    
    # 2. Reasoning (Ontology)
    target_speed, behavior_name = reason_behavior(min_dist)
    
    # 3. Logging (Requirement 5)
    print(f"Lidar: {min_dist:.2f}m -> Ontology inferred: {behavior_name} -> Action: Set Speed {target_speed}")

    # 4. Actuation
    for m in wheel_motors:
        m.setVelocity(target_speed)

    # Note: Pathing and Manipulation to be added next.
