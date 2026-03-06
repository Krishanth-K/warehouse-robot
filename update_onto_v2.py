from owlready2 import *
import os

onto_path = "final/controllers/pr2_autonomous_ethics/EWHR_integrated.owl"
onto = get_ontology(onto_path).load()

with onto:
    # Update InboundWaypoint 2 to match new world start
    iw2 = onto.search_one(iri="*InboundWaypoint2")
    if iw2:
        iw2.hasX = [-7.5]
    
    # Add another Restricted Zone for testing
    RestrictedZone2 = onto.WarehouseArea("RestrictedZone2")
    RestrictedZone2.hasAreaType = [onto.restricted_area]
    RestrictedZone2.hasX = [6.0]
    RestrictedZone2.hasY = [3.0]

onto.save(file=onto_path, format="rdfxml")
print("Ontology waypoints and zones updated for new world layout.")
