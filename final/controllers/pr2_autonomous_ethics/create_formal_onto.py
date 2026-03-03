from owlready2 import *

# Create the formal NEP-based Warehouse Ontology with explicit naming
onto = get_ontology("http://eras-nep.org/formal_warehouse.owl")

with onto:
    # --- NEP CLASSES ---
    class Agent(Thing): pass
    class Robot(Agent): pass
    class Situation(Thing): pass
    class HumanNearSituation(Situation): pass
    class PlanAction(Thing): pass
    class ReducedVelocityAction(PlanAction): pass
    class NormalVelocityAction(PlanAction): pass
    class EmergencyStopAction(PlanAction): pass

    # --- NEP PROPERTIES ---
    class recognizes(ObjectProperty):
        domain = [Agent]
        range = [Situation]
    class executes(ObjectProperty):
        domain = [Agent]
        range = [PlanAction]

    # --- INDIVIDUALS ---
    # We define these explicitly so the Controller can find them by name
    pr2 = Robot("pr2_robot")
    act_red = ReducedVelocityAction("reduced_velocity_action")
    act_norm = NormalVelocityAction("normal_velocity_action")
    act_stop = EmergencyStopAction("emergency_stop_action")

    # --- THE FORMAL NEP RULE ---
    # If Robot recognizes a HumanNearSituation -> It executes ReducedVelocityAction
    rule = Imp()
    rule.set_as_rule("""Robot(?r), recognizes(?r, ?s), HumanNearSituation(?s) -> executes(?r, reduced_velocity_action)""")

onto.save(file="formal_warehouse_ethics.owl", format="rdfxml")
print("SUCCESS: formal_warehouse_ethics.owl generated with explicit NEP rules.")
