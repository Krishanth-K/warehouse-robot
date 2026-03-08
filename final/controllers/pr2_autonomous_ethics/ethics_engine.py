import threading
import time
import math
from owlready2 import *

# --- Constants copied from main to maintain logic ---
SPEED_NORMAL = 10.0
SPEED_REDUCED = 3.0

def load_waypoints(onto, class_name):
    """Queries the ontology for waypoint individuals and returns them sorted by order."""
    cls = getattr(onto, class_name)
    if not cls: return []
    waypoints = []
    for ind in cls.instances():
        waypoints.append({
            "x":     float(ind.hasX[0]),
            "y":     float(ind.hasY[0]),
            "label": ind.hasLabel[0],
            "order": int(ind.hasOrder[0]),
        })
    return sorted(waypoints, key=lambda w: w["order"])

class EWHREthicsEngine(threading.Thread):
    def __init__(self, owl_path):
        super().__init__()
        self.daemon = True 
        self.onto = get_ontology(owl_path).load()
        self._lock = threading.Lock()
        
        self.human_near = False
        self.mission_situations = set() 
        self.target_velocity = SPEED_NORMAL
        self.behavior_desc = "EWHR: Initializing"
        
        # Attributes for normative enforcement
        self.robot_position = (0.0, 0.0)
        self.current_state = "NAV_OUTBOUND"
        self.brakes_engaged = False
        
        self.running = True
        self.start()

    def get_ind(self, name):
        """Robustly find an individual or class by name."""
        return self.onto.search_one(iri=f"*{name}")

    def update_human_proximity(self, is_near):
        self.human_near = is_near

    def set_active_situations(self, sit_names):
        self.mission_situations = set(sit_names)

    def run(self):
        while self.running:
            try:
                with self._lock:
                    with self.onto:
                        robot_ind = self.get_ind("PR2_001")
                        if robot_ind:
                            robot_ind.recognizes = []
                            robot_ind.executes = []

                            for sit_name in self.mission_situations:
                                sit = self.get_ind(sit_name)
                                if sit: robot_ind.recognizes.append(sit)
                            
                            proximity_sit = self.get_ind("Human_Proximity_Sit")
                            if self.human_near and proximity_sit:
                                robot_ind.recognizes.append(proximity_sit)
                        
                        sync_reasoner_pellet(infer_property_values=True, debug=0)
                        
                        if robot_ind:
                            inferred_actions = [a.name for a in robot_ind.executes]
                            self.behavior_desc = f"EWHR Actions: {', '.join(inferred_actions) if inferred_actions else 'None'}"
                            
                            # Ontology action inference mappings
                            reduce_speed_act    = self.get_ind("Set_Motor_Velocity_0.5")
                            engage_brakes_act   = self.get_ind("Engage_Wheel_Brakes")
                            reroute_act         = self.get_ind("Calculate_New_Trajectory")
                            auth_act            = self.get_ind("Authorization_Verification")

                            if reduce_speed_act and reduce_speed_act in robot_ind.executes:
                                self.target_velocity = SPEED_REDUCED
                            else:
                                self.target_velocity = SPEED_NORMAL

                            if engage_brakes_act and engage_brakes_act in robot_ind.executes:
                                self.brakes_engaged = True
                                print("[ONTOLOGY] Engaging wheel brakes for stability")

                            if reroute_act and reroute_act in robot_ind.executes:
                                print("[ONTOLOGY] Rerouting path due to restricted zone")

                            if auth_act and auth_act in robot_ind.executes:
                                print("[ONTOLOGY] Authorization check triggered")

                    # Enforce Norms
                    self._check_norms()
                            
            except Exception as e:
                print(f"[EWHR Ethics Engine ERROR] {e}")
                import traceback
                traceback.print_exc()
            time.sleep(0.5)

    def _check_norms(self):
        """Internal method for normative verification."""
        # Yield_To_Human_Norm
        if self.human_near and self.target_velocity == SPEED_NORMAL:
            print("[NORM VIOLATION] Yield_To_Human_Norm: Human nearby but speed not reduced")

        # Do_Not_Enter_Restricted_Area_Norm
        restricted_type = self.get_ind("restricted_area")
        if self.onto.WarehouseArea:
            for area in self.onto.WarehouseArea.instances():
                if restricted_type and restricted_type in area.hasAreaType:
                    dist = math.sqrt((self.robot_position[0] - float(area.hasX[0]))**2 + 
                                   (self.robot_position[1] - float(area.hasY[0]))**2)
                    if dist < 1.0:
                        print("[NORM VIOLATION] Do_Not_Enter_Restricted_Area_Norm: Robot in restricted area")

        # Lock_Base_Stability_Norm
        if self.current_state == "PICKING_UP" and self.brakes_engaged is False:
            print("[NORM VIOLATION] Lock_Base_Stability_Norm: Picking up without brakes engaged")
