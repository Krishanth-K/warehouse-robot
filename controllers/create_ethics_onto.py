from owlready2 import *

# Create a new ontology for Warehouse Ethics
onto = get_ontology("http://test.org/warehouse_ethics.owl")

with onto:
    class Entity(Thing): pass
    class Human(Entity): pass
    class Robot(Entity): pass
    
    class Proximity(Thing): pass
    class Near(Proximity): pass
    class Far(Proximity): pass
    
    class Behavior(Thing): pass
    class NormalSpeed(Behavior): pass
    class ReducedSpeed(Behavior): pass
    class EmergencyStop(Behavior): pass
    
    class has_proximity(ObjectProperty):
        domain    = [Robot]
        range     = [Proximity]

    # Rule: If Robot has_proximity Near, then it must have Behavior ReducedSpeed
    rule = Imp()
    rule.set_as_rule("""Robot(?r), has_proximity(?r, ?p), Near(?p) -> ReducedSpeed(?r)""")

onto.save(file="warehouse_ethics.owl", format="rdfxml")
print("Warehouse Ethics ontology created successfully.")
