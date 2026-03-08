from owlready2 import *
import os

onto_path = "final/controllers/pr2_autonomous_ethics/EWHR_integrated.owl"
onto = get_ontology(onto_path).load()

with onto:
    # --- Update Waypoints for Quad Setup & South Shift ---
    # Outbound 1: Movement along southern baseline (Y=-8) to Aisle 2 entry (X=5)
    ow1 = onto.search_one(iri="*OutboundWaypoint1")
    if ow1:
        ow1.hasX = [4.2] 
        ow1.hasY = [-8.0]

    # Outbound 2: Move north into Aisle 2 to the box (X=5, Y=4.4)
    ow2 = onto.search_one(iri="*OutboundWaypoint2")
    if ow2:
        ow2.hasX = [4.2] 
        ow2.hasY = [4.4]

    # Inbound 1: Exit aisle back to southern corridor (Y=-8)
    iw1 = onto.search_one(iri="*InboundWaypoint1")
    if iw1:
        iw1.hasX = [4.2]
        iw1.hasY = [-8.0]

    # Inbound 2: Return to robot start position (-10, -8)
    iw2 = onto.search_one(iri="*InboundWaypoint2")
    if iw2:
        iw2.hasX = [-9.5]
        iw2.hasY = [-8.0]

onto.save(file=onto_path, format="rdfxml")
print("Ontology waypoints synchronized with professional Quad Setup and Y=-8 baseline.")
