# Project Goal: Ontology-Driven Ethical Warehouse Robot

## 1. Objective
Create a fully autonomous PR2 robot simulation in Webots that navigates a warehouse, retrieves a box, and returns to start. The robot's movement must be governed by an ontology (`fullaxiom.owl`) using real-time reasoning (`owlready2`) to ensure ethical behavior (e.g., speed adjustments) when humans are present.

## 2. Navigation & Pathing (Hardcoded)
- **Method:** The robot follows a strictly hardcoded sequence of waypoints/actions (Timed or Coordinate-based). 
- **No Mapping:** No SLAM, ROS, or dynamic pathfinding is used.
- **Workflow:**
    1. **Start:** Begin at a fixed coordinate (0,0,0).
    2. **Outbound:** Follow a set path to the target shelf.
    3. **Action:** Physically pick up the target box using the PR2 grippers (hardcoded arm/gripper sequence).
    4. **Inbound:** Follow the return path back to the starting coordinate.

## 3. Human Detection (Sensing)
- **Method:** Color-based detection (simplest approach).
- **Implementation:** The robot identifies humans by detecting specific RGB values (e.g., a "Safety Orange" or "Bright Red" vest) in the camera image.
- **Lidar Integration:** Use the `base_laser` to determine the exact distance to the detected human for proximity reasoning.

## 4. Ontology Integration (The Ethics Engine)
- **Library:** Use `owlready2` for real-time interaction with `fullaxiom.owl`.
- **Reasoning Loop:**
    1. **Sensing:** Capture Lidar (distance) and Camera (human detection) data.
    2. **Fact Insertion:** Map sensor data into the ontology (e.g., create an individual `DetectedHuman` in the ontology if a person is seen).
    3. **Inference:** Run a reasoner (HermiT or Pellet via Owlready2) to check for "Ethical Violations" or "Required Behaviors".
    4. **Requirement:** If a Human is within a certain threshold (e.g., 2 meters), the ontology must infer a `ReducedSpeed` or `SafetyStop` state.
    5. **Actuation:** The Python controller reads the inferred state from the ontology and modifies the robot's hardware velocity (e.g., `MAX_SPEED` vs `CAUTIOUS_SPEED`).

## 5. Interaction & Behavior
- **Demonstration:** Pre-placed humans will be located along the hardcoded path.
- **Ethical Response:** The robot must demonstrate a clear change in behavior (slowing down or stopping) specifically because the ontology's rules dictated it, not because of a simple `if` statement in Python.
- **Verification:** The console MUST log the reasoning process (e.g., "Ontology inferred: HumanProximity -> Action: ReduceSpeed").

## 6. Hardware Constraints (Webots)
- **Robot:** PR2.
- **Sensors:** `base_laser` (Lidar), `wide_stereo_r_stereo_camera_sensor` (Camera).
- **Actuators:** Omni-directional wheel motors and finger grippers.
