from owlready2 import *
import os

onto_path = "final/controllers/pr2_autonomous_ethics/EWHR_integrated.owl"
onto = get_ontology(onto_path).load()

with onto:
    # --- Update Waypoints for New Functional Mission Shelf ---
    # Outbound 1: Movement along southern corridor baseline (Y=-5) to mission entry X
    ow1 = onto.search_one(iri="*OutboundWaypoint1")
    if ow1:
        ow1.hasX = [-6.8] # Approaching from the south side of the shelf
        ow1.hasY = [-5.0]

    # Outbound 2: Move north to the mission shelf at (-6, 4.4)
    ow2 = onto.search_one(iri="*OutboundWaypoint2")
    if ow2:
        ow2.hasX = [-6.8] 
        ow2.hasY = [4.4]

    # Inbound 1: Exit mission area back to southern corridor
    iw1 = onto.search_one(iri="*InboundWaypoint1")
    if iw1:
        iw1.hasX = [-6.8]
        iw1.hasY = [-5.0]

    # Inbound 2: Return to start (-10, -5)
    iw2 = onto.search_one(iri="*InboundWaypoint2")
    if iw2:
        iw2.hasX = [-9.5]
        iw2.hasY = [-5.0]

onto.save(file=onto_path, format="rdfxml")
print("Ontology waypoints synchronized with singular functional mission shelf.")
