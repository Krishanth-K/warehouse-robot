from owlready2 import *
import os

onto_path = "final/controllers/pr2_autonomous_ethics/EWHR_integrated.owl"
onto = get_ontology(onto_path).load()

with onto:
    # --- Update Waypoints for New Layout ---
    # Outbound 1: Entry point for Aisle 2 (X=3)
    ow1 = onto.search_one(iri="*OutboundWaypoint1")
    if ow1:
        ow1.hasX = [2.2] 
        ow1.hasY = [0.0]

    # Outbound 2: Box location in Aisle 2
    ow2 = onto.search_one(iri="*OutboundWaypoint2")
    if ow2:
        ow2.hasX = [2.2] 
        ow2.hasY = [4.4] # New box location Y

    # Inbound 1: Exit aisle
    iw1 = onto.search_one(iri="*InboundWaypoint1")
    if iw1:
        iw1.hasX = [2.2]
        iw1.hasY = [0.0]

    # Inbound 2: Return to start at -6
    iw2 = onto.search_one(iri="*InboundWaypoint2")
    if iw2:
        iw2.hasX = [-5.5]
        iw2.hasY = [0.0]

    # --- Add Restricted Zone in Aisle 1 ---
    RestrictedZoneA1_v5 = onto.WarehouseArea("RestrictedZoneA1_v5")
    RestrictedZoneA1_v5.hasAreaType = [onto.restricted_area]
    RestrictedZoneA1_v5.hasX = [0.0]
    RestrictedZoneA1_v5.hasY = [4.0]

onto.save(file=onto_path, format="rdfxml")
print("Ontology waypoints and zones updated for High Fidelity layout.")
