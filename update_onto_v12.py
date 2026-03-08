from owlready2 import *
import os

onto_path = "final/controllers/pr2_autonomous_ethics/EWHR_integrated.owl"
onto = get_ontology(onto_path).load()

with onto:
    # --- Update Waypoints for Straight Line Mission ---
    # Outbound 1: Stop before reaching the shelf at (0, -5)
    ow1 = onto.search_one(iri="*OutboundWaypoint1")
    if ow1:
        ow1.hasX = [-2.0] 
        ow1.hasY = [-5.0]

    # Outbound 2: Reach the pickup slot at (0, -5)
    ow2 = onto.search_one(iri="*OutboundWaypoint2")
    if ow2:
        ow2.hasX = [-0.8] 
        ow2.hasY = [-5.0]

    # Inbound 1: Back away from shelf
    iw1 = onto.search_one(iri="*InboundWaypoint1")
    if iw1:
        iw1.hasX = [-2.0]
        iw1.hasY = [-5.0]

    # Inbound 2: Return to start (-10, -5)
    iw2 = onto.search_one(iri="*InboundWaypoint2")
    if iw2:
        iw2.hasX = [-9.5]
        iw2.hasY = [-5.0]

onto.save(file=onto_path, format="rdfxml")
print("Ontology waypoints synchronized with straight-line mission shelf.")
