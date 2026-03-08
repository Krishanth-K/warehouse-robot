from controller import Robot, Camera, Lidar, Motor, Supervisor

def get_motor(robot, name):
    m = robot.getDevice(name)
    if not m: print(f"WARNING: Motor '{name}' not found!")
    return m

def initialize_devices(robot, time_step):
    """Initializes and returns all robot hardware components."""
    wheel_motors = []
    rotation_motors = []
    arm_motors = {}
    
    # Wheels
    wheel_names = ["fl_caster_l_wheel_joint","fl_caster_r_wheel_joint","fr_caster_l_wheel_joint","fr_caster_r_wheel_joint","bl_caster_l_wheel_joint","bl_caster_r_wheel_joint","br_caster_l_wheel_joint","br_caster_r_wheel_joint"]
    for name in wheel_names:
        m = get_motor(robot, name)
        if m: 
            m.setPosition(float("inf"))
            m.setVelocity(0.0)
            wheel_motors.append(m)
            
    # Casters
    rotation_names = ["fl_caster_rotation_joint", "fr_caster_rotation_joint", "bl_caster_rotation_joint", "br_caster_rotation_joint"]
    for name in rotation_names:
        m = get_motor(robot, name)
        if m: 
            m.setPosition(0.0)
            rotation_motors.append(m)
            
    # Torso & Arms
    torso_motor = get_motor(robot, "torso_lift_joint")
    
    for side in ['r', 'l']:
        joints = ["_shoulder_pan_joint", "_shoulder_lift_joint", "_upper_arm_roll_joint", "_elbow_flex_joint", "_forearm_roll_joint", "_wrist_flex_joint", "_wrist_roll_joint"]
        for j in joints:
            name = side + j
            m = get_motor(robot, name)
            if m: arm_motors[name] = m

    r_finger_motor = get_motor(robot, "r_finger_gripper_motor::l_finger")
    l_finger_motor = get_motor(robot, "l_finger_gripper_motor::l_finger")
    head_pan_motor = get_motor(robot, "head_pan_joint")
    
    camera = robot.getDevice("wide_stereo_r_stereo_camera_sensor")
    camera.enable(time_step)
    
    lidar = robot.getDevice("base_laser")
    lidar.enable(time_step)
    
    return {
        "wheels": wheel_motors,
        "rotation": rotation_motors,
        "arms": arm_motors,
        "torso": torso_motor,
        "r_finger": r_finger_motor,
        "l_finger": l_finger_motor,
        "head_pan": head_pan_motor,
        "camera": camera,
        "lidar": lidar
    }

def tuck_arms(arm_motors):
    """Positions arms in a safe tuck pose."""
    if "l_shoulder_lift_joint" in arm_motors: arm_motors["l_shoulder_lift_joint"].setPosition(1.2)
    if "l_elbow_flex_joint" in arm_motors: arm_motors["l_elbow_flex_joint"].setPosition(-2.0)
    if "l_shoulder_pan_joint" in arm_motors: arm_motors["l_shoulder_pan_joint"].setPosition(0.1)
    
    if "r_shoulder_lift_joint" in arm_motors: arm_motors["r_shoulder_lift_joint"].setPosition(1.2)
    if "r_elbow_flex_joint" in arm_motors: arm_motors["r_elbow_flex_joint"].setPosition(-2.0)

def set_mission_pose(hw, box_height):
    """Sets extremely narrow arm positions to ensure they are centered on the box."""
    from kinematics import calculate_reach_params
    torso_p, shoulder_p, elbow_p, wrist_p, _ = calculate_reach_params(box_height, 0.5)
    
    if hw["torso"]: hw["torso"].setPosition(torso_p)
    
    for side, roll in [('r', 0.7), ('l', -0.7)]:
        if f"{side}_shoulder_lift_joint" in hw["arms"]: hw["arms"][f"{side}_shoulder_lift_joint"].setPosition(shoulder_p)
        if f"{side}_elbow_flex_joint" in hw["arms"]: hw["arms"][f"{side}_elbow_flex_joint"].setPosition(elbow_p)
        if f"{side}_wrist_flex_joint" in hw["arms"]: hw["arms"][f"{side}_wrist_flex_joint"].setPosition(wrist_p)
        if f"{side}_upper_arm_roll_joint" in hw["arms"]: hw["arms"][f"{side}_upper_arm_roll_joint"].setPosition(roll)
        
        # EXTREMELY TIGHT: 0.2 rad inward brings hands close together
        pan_val = 0.2 if side == 'r' else -0.2
        if f"{side}_shoulder_pan_joint" in hw["arms"]: hw["arms"][f"{side}_shoulder_pan_joint"].setPosition(pan_val)

def is_human_in_view(camera):
    """Heuristic to detect human presence."""
    img = camera.getImage()
    if not img: return False
    w, h = camera.getWidth(), camera.getHeight()
    match_count = 0
    for y in range(h // 4, 3 * h // 4, 5):
        for x in range(0, w, 5):
            r, g, b = camera.imageGetRed(img, w, x, y), camera.imageGetGreen(img, w, x, y), camera.imageGetBlue(img, w, x, y)
            is_warm = (r > 150 and g > 100 and r > b + 40)
            is_clothing = (b > 150 and b > r + 30) or (g > 150 and g > r + 30)
            is_brick = (r > 140 and abs(g - b) < 15)
            if (is_warm or is_clothing) and not is_brick:
                match_count += 1
    return match_count > 15
