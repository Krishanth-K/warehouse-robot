from owlready2 import *
import os

onto_path = "final/controllers/pr2_autonomous_ethics/EWHR_integrated.owl"
onto = get_ontology(onto_path).load()

with onto:
    # Update InboundWaypoint 2 to match new world start at -10
    iw2 = onto.search_one(iri="*InboundWaypoint2")
    if iw2:
        iw2.hasX = [-9.5]
    
    # Add a Restricted Zone in the middle of Aisle 2
    RestrictedZone3 = onto.WarehouseArea("RestrictedZone3")
    RestrictedZone3.hasAreaType = [onto.restricted_area]
    RestrictedZone3.hasX = [10.0]
    RestrictedZone3.hasY = [-2.0]

onto.save(file=onto_path, format="rdfxml")
print("Ontology waypoints and zones updated for enclosed warehouse layout.")
