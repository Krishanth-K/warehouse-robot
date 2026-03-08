from owlready2 import *
import os

onto_path = "final/controllers/pr2_autonomous_ethics/EWHR_integrated.owl"
onto = get_ontology(onto_path).load()

with onto:
    # --- Synchronize Ontology with User's Final World Layout ---
    # Robot Start: (-4.05, -5.0)
    # Target Shelf: (11.0, -5.0)
    
    # Outbound 1: Move forward to a midpoint or just before the shelf
    ow1 = onto.search_one(iri="*OutboundWaypoint1")
    if ow1:
        ow1.hasX = [5.0] 
        ow1.hasY = [-5.0]

    # Outbound 2: Reach the pickup position at the mission shelf
    ow2 = onto.search_one(iri="*OutboundWaypoint2")
    if ow2:
        ow2.hasX = [10.2] # Stopping just before the shelf at X=11
        ow2.hasY = [-5.0]

    # Inbound 1: Back away slightly
    iw1 = onto.search_one(iri="*InboundWaypoint1")
    if iw1:
        iw1.hasX = [5.0]
        iw1.hasY = [-5.0]

    # Inbound 2: Return to the final start position (-4.05, -5.0)
    iw2 = onto.search_one(iri="*InboundWaypoint2")
    if iw2:
        iw2.hasX = [-3.5] # Safe return point
        iw2.hasY = [-5.0]

onto.save(file=onto_path, format="rdfxml")
print("Ontology waypoints synchronized with USER FINAL world layout.")
