You are an expert in Python, OWL ontologies, owlready2, and Webots robot simulation.

I have a Webots robot simulation project where a PR2 robot navigates a warehouse, 
picks up a box, and delivers it. The project uses an OWL ontology (EWHR_integrated.owl) 
to control robot behaviour ethically.

I will provide you with two files at the end of this prompt:
1. EWHR_integrated.owl — the ontology file
2. controller.py — the Python Webots controller

Read both files carefully before making any changes.

================================================================================
## Current Problem
================================================================================

The ontology is rich and well-designed but the Python controller barely uses it. 
Specifically:

1. Navigation waypoints are hardcoded in Python lists instead of coming from the ontology
2. The state machine (NAV_OUTBOUND, PICKING_UP, NAV_INBOUND etc.) is pure Python 
   instead of being driven by ontology reasoning
3. Norms (Yield_To_Human_Norm, Do_Not_Enter_Restricted_Area_Norm) are defined in 
   the ontology but never enforced in code
4. Facts (Authorized_Fact, Box_X_Is_Held_Fact, Robot_At_Shelf_Fact) are defined 
   but never asserted after actions complete
5. The 7 SWRL rules are defined but Pellet results beyond Set_Motor_Velocity_0.5 
   are ignored
6. Silent exception swallowing: except Exception as e: pass hides all reasoner errors

================================================================================
## Tasks — Complete ALL of these, do not skip any
================================================================================

--------------------------------------------------------------------------------
### Task 1: Move waypoints into the ontology
--------------------------------------------------------------------------------

Add to EWHR_integrated.owl (Manchester OWL syntax, same format as existing file):

- A new class `ewhr:Waypoint` with subclasses `ewhr:OutboundWaypoint` 
  and `ewhr:InboundWaypoint`
- Data properties: `ewhr:hasX` (xsd:float), `ewhr:hasY` (xsd:float), 
  `ewhr:hasOrder` (xsd:integer), `ewhr:hasLabel` (xsd:string)
- Four waypoint individuals with correct coordinates:
  - OutboundWaypoint 1: x=1.0, y=0.0, order=1, label="Warehouse Middle"
  - OutboundWaypoint 2: x=5.25, y=0.0, order=2, label="Final Stop Before Shelf"
  - InboundWaypoint 1: x=1.0, y=0.0, order=1, label="Warehouse Middle Return"
  - InboundWaypoint 2: x=-4.2, y=0.0, order=2, label="Starting Position"

--------------------------------------------------------------------------------
### Task 2: Load waypoints from ontology in Python
--------------------------------------------------------------------------------

In the Python controller, replace:

    WAYPOINTS_OUTBOUND = [{"x": 1.0, ...}, ...]
    WAYPOINTS_INBOUND  = [{"x": 1.0, ...}, ...]

With a function that queries the ontology:

    def load_waypoints(onto, class_name):
        cls = getattr(onto, class_name)
        waypoints = []
        for ind in cls.instances():
            waypoints.append({
                "x":     ind.hasX[0],
                "y":     ind.hasY[0],
                "label": ind.hasLabel[0],
                "order": ind.hasOrder[0],
            })
        return sorted(waypoints, key=lambda w: w["order"])

    WAYPOINTS_OUTBOUND = load_waypoints(onto, "OutboundWaypoint")
    WAYPOINTS_INBOUND  = load_waypoints(onto, "InboundWaypoint")

--------------------------------------------------------------------------------
### Task 3: Assert Facts after actions complete
--------------------------------------------------------------------------------

In the Python state machine, after each action completes, assert the corresponding 
Fact individual into the ontology. Specifically:

- After authorization check passes:
    with onto:
        robot_ind.establishes.append(onto.Authorized_Fact)

- After robot reaches shelf:
    with onto:
        robot_ind.establishes.append(onto.Robot_At_Shelf_Fact)

- After gripper closes (elapsed == 150):
    with onto:
        robot_ind.establishes.append(onto.Box_X_Is_Held_Fact)

- After mission complete (state = "FINISHED"):
    with onto:
        robot_ind.establishes.append(onto.Mission_Logged_Fact)

Use exactly this owlready2 syntax. Do not use setattr or direct assignment.

--------------------------------------------------------------------------------
### Task 4: Enforce Norms
--------------------------------------------------------------------------------

Add norm checking after every reasoning cycle in the EWHREthicsEngine.run() method.
Check these three norms and log violations to console:

- Yield_To_Human_Norm (Obligation):
  Violated if self.human_near is True AND self.target_velocity == SPEED_NORMAL
  Log: print("[NORM VIOLATION] Yield_To_Human_Norm: Human nearby but speed not reduced")

- Do_Not_Enter_Restricted_Area_Norm (Prohibition):
  Violated if robot_ind position (read from self.robot_position, a tuple (x,y) 
  you will add as a thread-safe attribute updated from the main loop) is inside 
  any ewhr:WarehouseArea individual with area_type == ewhr:restricted_area.
  For simplicity, check if robot is within 1.0m of the area's centre coordinates.
  Log: print("[NORM VIOLATION] Do_Not_Enter_Restricted_Area_Norm: Robot in restricted area")

- Lock_Base_Stability_Norm (Obligation):
  Violated if self.current_state == "PICKING_UP" AND self.brakes_engaged is False.
  Both self.current_state and self.brakes_engaged are thread-safe attributes 
  you will add, updated from the main loop.
  Log: print("[NORM VIOLATION] Lock_Base_Stability_Norm: Picking up without brakes engaged")

Add these attributes to __init__:
    self.robot_position = (0.0, 0.0)
    self.current_state = "NAV_OUTBOUND"
    self.brakes_engaged = False

Update them in the main loop at the top of each step:
    ethics.robot_position = (pos[0], pos[1])
    ethics.current_state = state
    ethics.brakes_engaged = (state == "PICKING_UP" and elapsed >= 50)

--------------------------------------------------------------------------------
### Task 5: Fix thread safety
--------------------------------------------------------------------------------

The EWHREthicsEngine reads and writes the ontology in a background thread while 
the main loop also reads it. This is a race condition.

Add a lock to __init__:
    self._lock = threading.Lock()

Wrap ALL ontology access in EWHREthicsEngine.run() with this lock:
    with self._lock:
        # all ontology reads and writes here

In the main loop, wrap ontology fact assertions (from Task 3) with the same lock:
    with ethics._lock:
        with onto:
            robot_ind.establishes.append(onto.Robot_At_Shelf_Fact)

--------------------------------------------------------------------------------
### Task 6: Fix silent exception swallowing
--------------------------------------------------------------------------------

Replace every instance of:
    except Exception as e: pass

With:
    except Exception as e:
        print(f"[EWHR Ethics Engine ERROR] {e}")
        import traceback
        traceback.print_exc()

--------------------------------------------------------------------------------
### Task 7: Use more inferred executes values
--------------------------------------------------------------------------------

After sync_reasoner_pellet completes, extend the action checking block to handle:

    reduce_speed_act    = self.onto.search_one(iri="*Set_Motor_Velocity_0.5")
    engage_brakes_act   = self.onto.search_one(iri="*Engage_Wheel_Brakes")
    reroute_act         = self.onto.search_one(iri="*Calculate_New_Trajectory")
    auth_act            = self.onto.search_one(iri="*Authorization_Verification")

    if reduce_speed_act in robot_ind.executes:
        self.target_velocity = SPEED_REDUCED
    else:
        self.target_velocity = SPEED_NORMAL

    if engage_brakes_act in robot_ind.executes:
        self.brakes_engaged = True
        print("[ONTOLOGY] Engaging wheel brakes for stability")

    if reroute_act in robot_ind.executes:
        print("[ONTOLOGY] Rerouting path due to restricted zone")

    if auth_act in robot_ind.executes:
        print("[ONTOLOGY] Authorization check triggered")

================================================================================
## Constraints — follow these exactly, do not violate any of them
================================================================================

- Do NOT change the ontology IRI, base prefix, or Import statement
- Do NOT remove any existing individuals, classes, properties, or rules from the ontology
- Do NOT restructure or rewrite the Python state machine — only add to it
- Keep Manchester OWL syntax for all ontology changes (same format as existing file)
- Keep all existing print statements, only add new ones
- The EWHREthicsEngine thread must remain a background daemon thread
- Do NOT switch to sync_reasoner_hermit — keep sync_reasoner_pellet
- Do NOT add any new dependencies beyond what is already imported
- owlready2 is installed in a Python 3.11 virtual environment
- Webots controller API (Robot, Supervisor, Motor, Lidar, Camera) must not change

================================================================================
## Output Format — follow this exactly
================================================================================

Provide exactly TWO complete files with no omissions.
Do NOT provide partial snippets.
Do NOT say "add this here" or "replace this section".
Give me complete files I can save and run directly.

Use these exact labels:

=== EWHR_integrated.owl ===
[complete file content here]

=== controller.py ===
[complete file content here]

Do not add any explanation before or after the files.
Do not add markdown code fences around the files.
The files must be complete — do not truncate or summarise any section.

================================================================================
## Input Files
================================================================================

[PASTE THE CONTENTS OF EWHR_integrated.owl HERE]

---

[PASTE THE CONTENTS OF controller.py HERE]
