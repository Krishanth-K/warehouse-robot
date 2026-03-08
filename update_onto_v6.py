from owlready2 import *
import os

onto_path = "final/controllers/pr2_autonomous_ethics/EWHR_integrated.owl"
onto = get_ontology(onto_path).load()

with onto:
    # --- Update Waypoints for Southern Shift ---
    # Outbound 1: Move along southern corridor to Aisle 2 entrance
    ow1 = onto.search_one(iri="*OutboundWaypoint1")
    if ow1:
        ow1.hasX = [2.2] 
        ow1.hasY = [-3.0] # Stay south before entering aisle

    # Outbound 2: Move into Aisle 2 to pick up the box
    ow2 = onto.search_one(iri="*OutboundWaypoint2")
    if ow2:
        ow2.hasX = [2.2] 
        ow2.hasY = [4.4]

    # Inbound 1: Exit Aisle 2 back to southern corridor
    iw1 = onto.search_one(iri="*InboundWaypoint1")
    if iw1:
        iw1.hasX = [2.2]
        iw1.hasY = [-3.0]

    # Inbound 2: Return to the new start position (-6, -3)
    iw2 = onto.search_one(iri="*InboundWaypoint2")
    if iw2:
        iw2.hasX = [-5.5]
        iw2.hasY = [-3.0]

onto.save(file=onto_path, format="rdfxml")
print("Ontology waypoints synchronized with southern shifted robot and belts.")
