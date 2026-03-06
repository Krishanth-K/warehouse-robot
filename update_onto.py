from owlready2 import *
import os

onto_path = "final/controllers/pr2_autonomous_ethics/EWHR_integrated.owl"
onto = get_ontology(onto_path).load()

with onto:
    # Task 1: Waypoints
    class Waypoint(Thing): pass
    class OutboundWaypoint(Waypoint): pass
    class InboundWaypoint(Waypoint): pass
    
    class hasX(DataProperty): range = [float]
    class hasY(DataProperty): range = [float]
    class hasOrder(DataProperty): range = [int]
    class hasLabel(DataProperty): range = [str]
    
    # Individuals
    ow1 = OutboundWaypoint("OutboundWaypoint1")
    ow1.hasX = [1.0]
    ow1.hasY = [0.0]
    ow1.hasOrder = [1]
    ow1.hasLabel = ["Warehouse Middle"]
    
    ow2 = OutboundWaypoint("OutboundWaypoint2")
    ow2.hasX = [5.25]
    ow2.hasY = [0.0]
    ow2.hasOrder = [2]
    ow2.hasLabel = ["Final Stop Before Shelf"]
    
    iw1 = InboundWaypoint("InboundWaypoint1")
    iw1.hasX = [1.0]
    iw1.hasY = [0.0]
    iw1.hasOrder = [1]
    iw1.hasLabel = ["Warehouse Middle Return"]
    
    iw2 = InboundWaypoint("InboundWaypoint2")
    iw2.hasX = [-4.2]
    iw2.hasY = [0.0]
    iw2.hasOrder = [2]
    iw2.hasLabel = ["Starting Position"]

    # Task 3 Prerequisites: Facts and establishes property
    class Fact(Thing): pass
    class establishes(ObjectProperty):
        domain = [onto.Robot]
        range = [Fact]
    
    Authorized_Fact = Fact("Authorized_Fact")
    Robot_At_Shelf_Fact = Fact("Robot_At_Shelf_Fact")
    Box_X_Is_Held_Fact = Fact("Box_X_Is_Held_Fact")
    Mission_Logged_Fact = Fact("Mission_Logged_Fact")

    # Task 4 Prerequisites: WarehouseArea and restricted_area
    class WarehouseArea(Thing): pass
    class AreaType(Thing): pass
    class hasAreaType(ObjectProperty):
        domain = [WarehouseArea]
        range = [AreaType]
    
    restricted_area = AreaType("restricted_area")
    
    # Adding a sample restricted area for Task 4 testing
    RestrictedZone1 = WarehouseArea("RestrictedZone1")
    RestrictedZone1.hasAreaType = [restricted_area]
    RestrictedZone1.hasX = [2.0] # Sample coordinates
    RestrictedZone1.hasY = [2.0]

    # Task 7 Prerequisites: Additional PlanActions
    Engage_Wheel_Brakes = onto.PlanAction("Engage_Wheel_Brakes")
    Calculate_New_Trajectory = onto.PlanAction("Calculate_New_Trajectory")
    Authorization_Verification = onto.PlanAction("Authorization_Verification")

onto.save(file=onto_path, format="rdfxml")
print("Ontology updated successfully.")
