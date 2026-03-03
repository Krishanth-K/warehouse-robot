# Future Enhancements & Roadmap

## 1. Dynamic Human Detection (Vision-Based)
- **Current:** Supervisor-based tracking (getting exact coordinates of human nodes).
- **Future:** Replace Supervisor tracking with robust AI vision (e.g., YOLOv8) and Lidar-to-Vision mapping. This will allow the robot to "see" humans without having access to the world's ground-truth data.

## 2. Advanced Ontology Integration
- **Current:** Simple `warehouse_ethics.owl` prototype.
- **Future:** Integrate the full `fullaxiom.owl` and additional moral reasoning rules. This will enable more complex ethical decisions (e.g., priority-based navigation, social distance maintenance).

## 3. Dynamic Pathfinding & Mapping
- **Current:** Hardcoded checkpoints and waypoints.
- **Future:** Implement SLAM (Simultaneous Localization and Mapping) and a pathfinder (e.g., A* or Dijkstra's) so the robot can navigate a changing warehouse autonomously.

## 4. Intelligent Manipulation
- **Current:** Hardcoded gripper sequence.
- **Future:** Use camera/sensor feedback for dynamic grasping. This will enable the robot to pick up objects that are slightly out of position or have different shapes.

## 5. Multi-Robot Ethics
- **Current:** Single-robot simulation.
- **Future:** Extend the ontology to handle interactions between multiple robots, ensuring they cooperate ethically in shared spaces.
