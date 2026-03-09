from owlready2 import *
import os

onto_path = "final/controllers/pr2_autonomous_ethics/EWHR_integrated.owl"
onto = get_ontology(onto_path).load()

with onto:
    # World-aligned straight-line mission values in ethics_test.wbt
    # Robot start: (-1.46, -5.0), shelf: (11.0, -5.0)
    
    ow1 = onto.search_one(iri="*OutboundWaypoint1")
    if ow1:
        ow1.hasX = [-1.46]
        ow1.hasY = [-5.0]

    ow2 = onto.search_one(iri="*OutboundWaypoint2")
    if ow2:
        ow2.hasX = [10.9] # Final outbound checkpoint near shelf
        ow2.hasY = [-5.0] # Keeping it in the straight line path

    iw1 = onto.search_one(iri="*InboundWaypoint1")
    if iw1:
        iw1.hasX = [-1.46]
        iw1.hasY = [-5.0]

    iw2 = onto.search_one(iri="*InboundWaypoint2")
    if iw2:
        iw2.hasX = [-1.46]
        iw2.hasY = [-5.0]

onto.save(file=onto_path, format="rdfxml")
print("Ontology waypoints synchronized to robot start (-1.46, -5.0) and shelf line (10.9, -5.0).")
