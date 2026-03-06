import math

def normalize_angle(angle):
    """Normalizes an angle to the range [-pi, pi]."""
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle

def calculate_reach_params(box_height):
    """Calculates arm positions using a V-down pose for shelf clearance."""
    SHOULDER_BASE_Z = 0.73
    ELBOW_FLEX = -1.2 
    
    target_torso = box_height - SHOULDER_BASE_Z + 0.15
    torso_pos = max(0.0, min(0.3, target_torso))
    
    shoulder_z = SHOULDER_BASE_Z + torso_pos
    dz = box_height - shoulder_z
    dx = 0.52
    
    angle_to_box = math.atan2(-dz, dx)
    shoulder_lift = angle_to_box - (ELBOW_FLEX / 2.0)
    shoulder_lift = max(-0.5, min(1.2, shoulder_lift))
    
    wrist_flex = -(shoulder_lift + ELBOW_FLEX)
    wrist_flex = max(-2.0, min(0.0, wrist_flex))
    
    retract_shoulder = shoulder_lift - 0.25
    
    return torso_pos, shoulder_lift, ELBOW_FLEX, wrist_flex, retract_shoulder

def set_caster_angles(rotation_motors, fl, fr, bl, br):
    """Sets the positions for the four caster rotation joints."""
    if len(rotation_motors) == 4:
        rotation_motors[0].setPosition(fl)
        rotation_motors[1].setPosition(fr)
        rotation_motors[2].setPosition(bl)
        rotation_motors[3].setPosition(br)
