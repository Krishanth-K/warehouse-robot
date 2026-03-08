from controller import Supervisor
import math
from kinematics import normalize_angle, calculate_reach_params, set_caster_angles
from robot_config import initialize_devices, tuck_arms, is_human_in_view, set_mission_pose
from ethics_engine import EWHREthicsEngine, load_waypoints

# --- Configuration & Constants ---
TIME_STEP = 16
SPEED_NORMAL = 15.0
SPEED_ROTATE = 2.0
SPEED_BACKWARD = -3.0 
SPEED_STOP = 0.0
CHECKPOINT_MARGIN = 0.75    
CLEARANCE_TIME = 1.5       
TARGET_BOX_HEIGHT = 0.62  
BACKUP_STEPS = 40

ONTOLOGY_PATH = "EWHR_integrated.owl"

# --- Initialization ---
robot = Supervisor()

# SUPERVISOR OVERRIDE
pr2_node = robot.getSelf()
wheel_motor_names = ["fl_caster_l_wheel_joint","fl_caster_r_wheel_joint","fr_caster_l_wheel_joint","fr_caster_r_wheel_joint","bl_caster_l_wheel_joint","bl_caster_r_wheel_joint","br_caster_l_wheel_joint","br_caster_r_wheel_joint"]

if pr2_node:
    for name in wheel_motor_names:
        motor_node = pr2_node.getFromProtoDef(name)
        if motor_node:
            max_vel_field = motor_node.getField("maxVelocity")
            if max_vel_field:
                max_vel_field.setSFFloat(25.0)

hw = initialize_devices(robot, TIME_STEP)
set_mission_pose(hw, TARGET_BOX_HEIGHT)
self_node = robot.getSelf()

ethics = EWHREthicsEngine(ONTOLOGY_PATH)
onto = ethics.onto

WAYPOINTS_OUTBOUND = load_waypoints(onto, "OutboundWaypoint")
WAYPOINTS_INBOUND  = load_waypoints(onto, "InboundWaypoint")

def get_ind(name):
    return onto.search_one(iri=f"*{name}")

# Initial Authorization
with ethics._lock:
    with onto:
        robot_ind = get_ind("PR2_001")
        authorized_fact = get_ind("Authorized_Fact")
        if robot_ind and authorized_fact:
            robot_ind.establishes.append(authorized_fact)

# --- State Management ---
state = "NAV_OUTBOUND"
checkpoint_index = 0
step_count = 0
currently_tracking = False
is_rotating = False
clearance_timer = 0.0
pickup_sequence_start = 0
backup_start_step = 0
did_backup = False

# --- Mission Context ---
TARGET_SHELF_X = 11.0 

# --- Main Control Loop ---
while robot.step(TIME_STEP) != -1:
    step_count += 1
    pos = self_node.getPosition()
    rot = self_node.getOrientation()
    yaw = math.atan2(rot[3], rot[0]) 
    
    ethics.robot_position = (pos[0], pos[1])
    ethics.current_state = state
    elapsed = step_count - pickup_sequence_start if state == "PICKING_UP" else 0
    ethics.brakes_engaged = (state == "PICKING_UP" and elapsed >= 50)

    # 1. Perception & Tracking
    lidar = hw["lidar"]
    ranges = lidar.getRangeImage()
    valid_ranges = [r for r in ranges if not math.isinf(r) and not math.isnan(r) and r > 0.1]
    lidar_sees_human = False
    
    if valid_ranges:
        min_dist = min(valid_ranges); idx = ranges.index(min_dist); fov = lidar.getFov()
        angle = (idx / (len(ranges)-1)) * fov - (fov / 2.0)
        
        if not currently_tracking:
            if hw["head_pan"]: hw["head_pan"].setPosition(0.0)
            prox_t = robot.getSelf().getField("proximity_threshold")
            threshold = prox_t.getSFFloat() if prox_t else 3.0
            if min_dist < threshold and abs(angle) < 0.5 and is_human_in_view(hw["camera"]):
                currently_tracking = True; lidar_sees_human = True
        else:
            if hw["head_pan"]: hw["head_pan"].setPosition(angle)
            lidar_sees_human = True
            if min_dist > 3.0 or abs(angle) > 1.45:
                currently_tracking = False; lidar_sees_human = False
                if hw["head_pan"]: hw["head_pan"].setPosition(0.0)
                clearance_timer = CLEARANCE_TIME
    else:
        if currently_tracking:
            currently_tracking = False; lidar_sees_human = False; clearance_timer = CLEARANCE_TIME
            if hw["head_pan"]: hw["head_pan"].setPosition(0.0)

    if clearance_timer > 0: clearance_timer -= (TIME_STEP / 1000.0)
    ethics.update_human_proximity(lidar_sees_human or (clearance_timer > 0))
    limit_speed = ethics.target_velocity
    
    # 2. Mission Logic
    target_speed = SPEED_STOP
    current_rot_speed = 0.0
    active_waypoints = WAYPOINTS_OUTBOUND if state == "NAV_OUTBOUND" else WAYPOINTS_INBOUND
    
    if state == "NAV_OUTBOUND":
        ethics.set_active_situations(["At_Base_Station_Sit"])
    elif state == "PICKING_UP":
        if elapsed < 150: ethics.set_active_situations(["Box_Lift_Ready_Sit"])
        else: ethics.set_active_situations(["Holding_Box_Sit"])
    elif state == "NAV_INBOUND":
        ethics.set_active_situations(["Holding_Box_Sit"])
    elif state == "BACKING_UP":
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
                set_caster_angles(hw["rotation"], 3*math.pi/4, math.pi/4, -3*math.pi/4, -3*math.pi/4)
                current_rot_speed = SPEED_ROTATE if angle_error > 0 else -SPEED_ROTATE
                target_speed = SPEED_STOP
            else:
                set_caster_angles(hw["rotation"], 0, 0, 0, 0)
                if state == "NAV_INBOUND" and checkpoint_index == 0 and not did_backup:
                    target_speed = SPEED_BACKWARD
                else:
                    target_speed = limit_speed
        else:
            if state == "NAV_OUTBOUND":
                with ethics._lock:
                    with onto:
                        shelf_fact = get_ind("Robot_At_Shelf_Fact")
                        if robot_ind and shelf_fact:
                            robot_ind.establishes.append(shelf_fact)
                state = "PICKING_UP"; pickup_sequence_start = step_count
            else:
                with ethics._lock:
                    with onto:
                        mission_fact = get_ind("Mission_Logged_Fact")
                        if robot_ind and mission_fact:
                            robot_ind.establishes.append(mission_fact)
                state = "FINISHED"

    elif state == "PICKING_UP":
        target_speed = SPEED_STOP
        current_reach = (TARGET_SHELF_X - pos[0]) - 0.25
        torso_p, shoulder_p, elbow_p, wrist_p, _ = calculate_reach_params(TARGET_BOX_HEIGHT, current_reach)
        
        if elapsed == 50:
            # 1. EXTEND: Move arms straight forward (No pan)
            for side in ['r', 'l']:
                if f"{side}_shoulder_lift_joint" in hw["arms"]: hw["arms"][f"{side}_shoulder_lift_joint"].setPosition(shoulder_p)
                if f"{side}_elbow_flex_joint" in hw["arms"]: hw["arms"][f"{side}_elbow_flex_joint"].setPosition(elbow_p)
                if f"{side}_wrist_flex_joint" in hw["arms"]: hw["arms"][f"{side}_wrist_flex_joint"].setPosition(wrist_p)
        elif elapsed == 100:
            # 2. GRASP: Bring grippers together to grasp the box
            # Sign Logic: Right positive (+), Left negative (-) to move inward
            if "r_shoulder_pan_joint" in hw["arms"]: hw["arms"]["r_shoulder_pan_joint"].setPosition(0.4)
            if "l_shoulder_pan_joint" in hw["arms"]: hw["arms"]["l_shoulder_pan_joint"].setPosition(-0.4)
        elif elapsed == 150:
            # Ensure fingers are tight
            if hw["r_finger"]: hw["r_finger"].setPosition(0.0)
            if hw["l_finger"]: hw["l_finger"].setPosition(0.0)
            with ethics._lock:
                with onto:
                    held_fact = get_ind("Box_X_Is_Held_Fact")
                    if robot_ind and held_fact:
                        robot_ind.establishes.append(held_fact)
        elif elapsed == 200:
            # 3. LIFT: Raise very slightly
            if hw["torso"]: hw["torso"].setPosition(torso_p + 0.05)
        elif elapsed == 250:
            # 4. RETRACT: Transition to backward movement
            state = "BACKING_UP"
            backup_start_step = step_count

    elif state == "BACKING_UP":
        set_caster_angles(hw["rotation"], 0, 0, 0, 0)
        target_speed = SPEED_BACKWARD
        if step_count - backup_start_step >= BACKUP_STEPS:
            did_backup = True
            state = "NAV_INBOUND"
            checkpoint_index = 0

    # 3. Actuation
    for m in hw["wheels"]:
        m.setVelocity(current_rot_speed if is_rotating else target_speed)

    if step_count % 100 == 0:
        print(f"STATE: {state} | Speed: {target_speed:.1f}/{limit_speed:.1f}")
