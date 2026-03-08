from owlready2 import *
import os

onto_path = "final/controllers/pr2_autonomous_ethics/EWHR_integrated.owl"
onto = get_ontology(onto_path).load()

with onto:
    # Target Shelf is at X=11.0
    # Move close enough to effectively reach the shelf before pickup.
    
    ow2 = onto.search_one(iri="*OutboundWaypoint2")
    if ow2:
        ow2.hasX = [10.9] # Final outbound checkpoint near shelf
        ow2.hasY = [-5.0] # Keeping it in the straight line path

onto.save(file=onto_path, format="rdfxml")
print("Ontology waypoint 2 adjusted to reach the shelf line at X=10.9.")
