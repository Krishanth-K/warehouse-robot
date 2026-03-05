# Development Log - Ontology-Driven Ethical Warehouse Robot

## [2026-03-03] - Initial Setup
... (previous logs preserved) ...

## [2026-03-05] - Enhanced Generalized Picking & Gripping
- **Improvement:** Expanded the height-aware sequence to include dynamic shoulder adjustment.
- **Action:**
    - Replaced `calculate_torso_pos` with `calculate_reach_params(box_height)`.
    - Implemented heuristic mapping to adjust `r_shoulder_lift_joint` when the desired height exceeds the `torso_lift_joint` range (0.0 - 0.3m).
    - Added height-dependent `retract_shoulder` position to ensure safe lifting after pickup regardless of shelf height.
    - Updated `PICKING_UP` state machine to use these dynamic parameters.
- **Status:** Gripping motion now depends fully on `TARGET_BOX_HEIGHT`.

## [2026-03-05] - Bug Fix: Shelf Collision & Physics Instability
- **Issue:** The left arm was hitting shelves because it was not initialized or tucked. The right arm pose was pointing the elbow/forearm upwards, causing it to hit the ceiling of the shelf and trigger physics errors.
- **Action:**
    - Updated `initialize_devices` to track all joints for both `r_` and `l_` arms.
    - Added `tuck_arms()` to safely position the left arm and provide a starting pose for the right.
    - Redesigned `calculate_reach_params` to use "V-down" geometry: the shoulder now points down while the forearm points back up, keeping the "elbow hump" low and away from the shelf ceiling.
    - Added `r_wrist_flex_joint` control to keep the gripper horizontal regardless of the arm's angle.
- **Status:** Improved reach and clearance. The robot now avoids the shelf ceiling and keeps the gripper level for better box retrieval.

## [2026-03-05] - Full EWHR Ontology Integration
- **Requirement:** Transition from the simplified `formal_warehouse_ethics.owl` to the comprehensive `EWHR.owl` ontology.
- **Action:**
    - Implemented `EWHREthicsEngine` to replace `FormalEthicsEngine`.
    - Mapped Webots sensing to `ewhr:Human_Proximity_Sit` individual.
    - Updated reasoning loop to use `ewhr:PR2_001` (Robot) and monitor the inference of `ewhr:Set_Motor_Velocity_0.5` (AgentAction).
    - Handled namespace mapping for NEP properties (e.g., `nep:recognizes` and `nep:executes`) within the EWHR framework.
- **Status:** The robot's behavior is now governed by the complete EWHR ontology and its associated NEP rules.
