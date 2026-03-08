from owlready2 import *
import os

onto_path = "final/controllers/pr2_autonomous_ethics/EWHR_integrated.owl"
onto = get_ontology(onto_path).load()

with onto:
    # --- Update Waypoints for Spaced Aisle Layout ---
    # Outbound 1: Move along southern corridor to the entrance of Aisle 2 (now at X=5)
    ow1 = onto.search_one(iri="*OutboundWaypoint1")
    if ow1:
        ow1.hasX = [4.2] 
        ow1.hasY = [-5.0] # New southern starting Y

    # Outbound 2: Move into Aisle 2 to the box (X=5, Y=4.4)
    ow2 = onto.search_one(iri="*OutboundWaypoint2")
    if ow2:
        ow2.hasX = [4.2] 
        ow2.hasY = [4.4]

    # Inbound 1: Exit aisle back to south
    iw1 = onto.search_one(iri="*InboundWaypoint1")
    if iw1:
        iw1.hasX = [4.2]
        iw1.hasY = [-5.0]

    # Inbound 2: Return to the robot start position (-8, -5)
    iw2 = onto.search_one(iri="*InboundWaypoint2")
    if iw2:
        iw2.hasX = [-7.5]
        iw2.hasY = [-5.0]

onto.save(file=onto_path, format="rdfxml")
print("Ontology waypoints synchronized with spaced aisles and parallel belts.")
