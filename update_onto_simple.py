from owlready2 import *
import os

onto_path = "final/controllers/pr2_autonomous_ethics/EWHR_formal.owl"
onto = get_ontology(onto_path).load()

with onto:
    # Clear existing waypoints to avoid confusion
    for wp in onto.Waypoint.instances():
        destroy_entity(wp)

    # --- Simple Mission Waypoints ---
    # Outbound: Drive straight to the shelf (X=0, Y=-5)
    # Target: Pick up position (-1.0, -5.0)
    ow1 = onto.OutboundWaypoint("OutboundWaypoint1")
    ow1.hasX = [-1.0]
    ow1.hasY = [-5.0]
    ow1.hasOrder = [1]
    ow1.hasLabel = ["Target Shelf"]

    # Inbound: Return to start (-8, -5)
    iw1 = onto.InboundWaypoint("InboundWaypoint1")
    iw1.hasX = [-8.0]
    iw1.hasY = [-5.0]
    iw1.hasOrder = [1]
    iw1.hasLabel = ["Start Position"]

onto.save(file=onto_path, format="rdfxml")
print("Ontology updated for simple_test.wbt (straight line mission).")
