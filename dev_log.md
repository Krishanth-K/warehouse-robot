# Development Log - Ontology-Driven Ethical Warehouse Robot

## [2026-03-03] - Initial Setup
- **Task:** Finalized `goal.md` and established the project roadmap.
- **Goal:** Transform the manual PR2 controller into an autonomous, ontology-driven system.
- **Log Created:** Initialized `dev_log.md` to track all changes, issues, and solutions.

### Current Status:
- Controller: `pr2_py_controller.py` is currently manual (keyboard-controlled) with basic color-based human detection.
- Ontology: `fullaxiom.owl` is present but only loaded as an XML tree, not used for reasoning.
- Environment: Webots 2023b (assumed) with PR2 robot.

## [2026-03-03] - World Creation & Ontology Clarification
- **Action:** Created `FN Robotics/worlds/ethics_test.wbt` with a straight-line setup (Robot -> Human -> Shelf).
- **Note:** `PR2` is set to use the `pr2_autonomous_ethics` controller (which we will move to the correct folder).
- **Clarification:** The existing `fullaxiom.owl` is a standard IEEE 1872 foundation. It provides the "vocabulary" (what is a Robot, what is a System) but not the "moral rules." My prototype `warehouse_ethics.owl` adds the specific ethical logic (Speed Control rules). In the future, these will be merged so the robot uses a single, robust ontology.

### Next Steps:
1. Move the new controller and ontology files into the `FN Robotics/controllers/pr2_autonomous_ethics/` folder so Webots can find them.
2. Refine the controller to handle the "straight line" movement.

## [2026-03-03] - Implementation Phase 1: Modular Ethics Engine
- **Action:** Created `FN Robotics/controllers/pr2_autonomous_ethics/` and copied `pr2_autonomous_ethics.py`, `warehouse_ethics.owl`, and `fullaxiom.owl` into it.
- **Action:** Refactored the controller into a modular `EthicsEngine` class. 
- **Reasoning:** To ensure that when the "full ethics" ontology is provided, we only need to update the `get_behavior` mapping logic, rather than rewriting the whole controller.
- **Progress:** 
    - Robot now has a basic state machine (`DRIVING_FORWARD`, `PICKING_UP`).
    - Lidar-based proximity is piped into the ontology.
    - Speed is automatically reduced if the ontology infers `Near` proximity.

### Technical Notes:
- The path is currently time-based (10 seconds forward) for simplicity. This will be tuned for the `ethics_test.wbt` world.
- The `EthicsEngine` uses `sync_reasoner_pellet` which is robust but can be slow if the ontology is huge. This is something to monitor.

## [2026-03-03] - Migration to `final/` and Proto Fixes
- **Action:** Moved all active project files to `/home/krish/webots/latest/final/`.
- **Reason:** To maintain a clean, separated project structure.
- **Action:** Updated `final/worlds/ethics_test.wbt` to use `R2023b` URLs (likely matching the local install) and switched `Shelf` to `RoCKInShelf` to fix download errors.
- **Action:** Changed floor to `VarnishedPine` for a better visual experience.

## [2026-03-03] - Case Sensitivity & Proto Path Fix
- **Issue:** Webots threw download errors for `PR2`.
- **Solution:** Found that the existing world files used `robots/clearpath/pr2/protos/Pr2.proto` (case sensitive `Pr2`) instead of `robots/willow_garage/pr2/protos/PR2.proto`.
- **Action:** Corrected `final/worlds/ethics_test.wbt` with the correct `Pr2` and `RoCKInShelf` paths.
- **Action:** Reverted to `R2025a` URLs as they were used in the working "old" files.

## [2026-03-03] - Larger Shelf & User Adjustments
- **Action:** Swapped `RoCKInShelf` for the larger `Shelves` proto as requested.
- **Action:** Preserved user's manual change to human rotation (`-1.5707953071795862`).
- **Action:** Cleaned up the `ethics_test.wbt` file by removing unnecessary "hidden" simulation state fields.

### Next Steps:
1. Implement the gripper picking sequence in the controller.
