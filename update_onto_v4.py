from owlready2 import *
import os

onto_path = "final/controllers/pr2_autonomous_ethics/EWHR_integrated.owl"
onto = get_ontology(onto_path).load()

with onto:
    # --- Update Waypoints for Northern Aisle Pickup ---
    # Outbound 1: Move to the entrance of the target aisle
    ow1 = onto.search_one(iri="*OutboundWaypoint1")
    if ow1:
        ow1.hasX = [4.2] # Entry point for Aisle 2
        ow1.hasY = [0.0]
        ow1.hasLabel = ["Aisle 2 Entrance"]

    # Outbound 2: Move into the aisle to the box
    ow2 = onto.search_one(iri="*OutboundWaypoint2")
    if ow2:
        ow2.hasX = [4.2] 
        ow2.hasY = [6.0] # Box location Y
        ow2.hasLabel = ["Aisle 2 Target Slot"]

    # Inbound 1: Exit aisle
    iw1 = onto.search_one(iri="*InboundWaypoint1")
    if iw1:
        iw1.hasX = [4.2]
        iw1.hasY = [0.0]
        iw1.hasLabel = ["Aisle 2 Exit"]

    # Inbound 2: Return to start
    iw2 = onto.search_one(iri="*InboundWaypoint2")
    if iw2:
        iw2.hasX = [-9.5]
        iw2.hasY = [0.0]

    # --- Add Restricted Zones in Aisles ---
    # Zone in Aisle 1
    RestrictedZoneA1 = onto.WarehouseArea("RestrictedZoneA1")
    RestrictedZoneA1.hasAreaType = [onto.restricted_area]
    RestrictedZoneA1.hasX = [-0.8] # Center of Aisle 1
    RestrictedZoneA1.hasY = [5.0]

onto.save(file=onto_path, format="rdfxml")
print("Ontology waypoints and zones updated for Northern Aisle layout.")
