# Technical Report: Procedure for Building the PR2 Robot and Warehouse World in Webots

## 1. Introduction
This report describes the implementation procedure used to build an ontology-driven warehouse robot scenario in Webots. The objective was to deploy a PR2 mobile manipulator that can autonomously navigate from a base position to a shelf, pick a box, and return to the origin while applying ethical constraints inferred from an OWL ontology (`EWHR.owl`) through real-time reasoning.

The final system combines three layers:
1. A structured Webots world model for reproducible task execution.
2. A Python controller for sensing, navigation, manipulation, and actuation.
3. An ontology reasoning layer (Owlready2 + Pellet) for ethical behavior modulation.

## 2. World Construction in Webots
The simulation world was defined in `final/worlds/ethics_test.wbt` using Webots R2025a assets and PR2-related PROTOs. The scene was intentionally designed to be minimal and deterministic so that behavioral changes could be attributed to ontology reasoning rather than environmental randomness.

The world contains:
1. A `RectangleArena` floor (`20 m x 20 m`) as the warehouse workspace.
2. A `Pr2` robot initialized near `x = -4.99` and configured with `supervisor TRUE`.
3. A `Shelves` unit at the pickup destination (`x = 6.0`).
4. A `WoodenBox` on the shelf (`z = 0.62`) as the target object.
5. A `Pedestrian` positioned near the travel route to trigger proximity-aware ethics.

This layout supports end-to-end testing of outbound navigation, constrained-space grasping, and inbound return under human-presence conditions.

## 3. Robot Configuration and Device Initialization
Controller development was implemented in `final/controllers/pr2_autonomous_ethics/pr2_autonomous_ethics.py`. Initialization included:
1. Eight wheel joints for PR2 omnidirectional base motion.
2. Four caster rotation joints for heading control and in-place rotation.
3. Torso lift, full arm joint access, finger grippers, and head pan.
4. Sensors: `base_laser` (lidar) and `wide_stereo_r_stereo_camera_sensor` (camera).

A dedicated arm-tucking routine was applied at startup to avoid early collisions with shelves and to ensure safe kinematic posture before manipulation.

## 4. World-to-Controller Task Procedure
The execution pipeline was implemented as a finite state machine with four task states:
1. `NAV_OUTBOUND`
2. `PICKING_UP`
3. `NAV_INBOUND`
4. `FINISHED`

### 4.1 Navigation
Navigation uses ontology-sourced waypoints instead of hardcoded coordinate lists. Waypoints are loaded by querying `OutboundWaypoint` and `InboundWaypoint` individuals and sorting by `hasOrder`. The controller tracks heading error to each waypoint, rotates when needed, then advances using straight caster alignment.

### 4.2 Manipulation
The pickup sequence is time-phased and height-aware. A reach heuristic computes torso and right-arm target positions from the known box height, maintaining shelf clearance and gripper alignment. The manipulation stages include torso lift, arm deployment, gripper open/close, and retraction before return navigation.

## 5. Ontology Integration Procedure (`EWHR.owl`)
Ethical and mission semantics were encoded in `EWHR.owl` and consumed at runtime by Owlready2. The ontology provides:
1. Situation and action abstractions (`Situation`, `PlanAction`).
2. Route entities (`Waypoint`, `OutboundWaypoint`, `InboundWaypoint`).
3. Mission facts (`Authorized_Fact`, `Robot_At_Shelf_Fact`, `Box_X_Is_Held_Fact`, `Mission_Logged_Fact`).
4. Safety semantics, including restricted-area typing and ethics-trigger actions.
5. SWRL rules that infer actions from recognized situations.

A dedicated background daemon thread (`EWHREthicsEngine`) performs repeated reasoning cycles. In each cycle, the controller writes current situations to ontology assertions, executes `sync_reasoner_pellet`, and reads inferred `executes` actions to update control constraints (for example, speed reduction under human proximity).

## 6. Human Detection and Ethical Response Pipeline
Human-awareness was implemented by fusing camera-based color detection with lidar distance/angle filtering:
1. Camera identifies candidate human signatures in sampled image pixels.
2. Lidar verifies nearest target distance and relative direction.
3. A short clearance timer stabilizes transitions to avoid rapid oscillation.
4. The final proximity flag is sent to the ethics thread.

When proximity is inferred as ethically relevant, ontology rules can trigger behavior such as reduced velocity. This ensures that safety modulation is reasoner-driven rather than only hardcoded conditional logic.

## 7. Norm Enforcement, Fact Assertion, and Concurrency Control
Three runtime norm checks were implemented and logged:
1. `Yield_To_Human_Norm`: violation if human is near and speed is not reduced.
2. `Do_Not_Enter_Restricted_Area_Norm`: violation if robot enters restricted zone radius.
3. `Lock_Base_Stability_Norm`: violation if pickup proceeds without stability condition.

Mission facts are asserted at key milestones:
1. Authorization established at startup.
2. Shelf arrival confirmed at outbound completion.
3. Box-hold fact asserted when the grasp closes.
4. Mission-logged fact asserted at final completion.

Because ontology access occurs in both the main loop and the ethics thread, all ontology reads/writes are protected with a shared `threading.Lock` to prevent race conditions.

## 8. Outcome
The final implementation demonstrates a full simulation stack in which world design, physical robot control, and ontology reasoning are operationally connected. The PR2 can execute a warehouse pickup-and-return mission while adapting behavior to ethically significant context (human proximity and norm constraints), with explicit reasoning traces and milestone fact assertions suitable for research reporting and reproducibility.
