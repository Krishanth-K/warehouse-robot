from owlready2 import *

def convert_manchester_to_rdf():
    # We create a new RDF/XML ontology that mimics the structure of EWHR.owl
    onto = get_ontology("http://ewhr.org/formal.owl")
    
    with onto:
        # TLO & NEP Base Classes
        class Agent(Thing): pass
        class Robot(Agent): pass
        class Situation(Thing): pass
        class PlanAction(Thing): pass
        class AgentAction(Thing): pass
        class Fact(Thing): pass
        class Norm(Thing): pass
        class DeontologicalNorm(Norm): pass
        class Obligation(DeontologicalNorm): pass
        class Prohibition(DeontologicalNorm): pass
        class WarehouseArea(Thing): pass
        class AreaType(Thing): pass

        # Properties
        class recognizes(ObjectProperty):
            domain = [Agent]
            range = [Situation]
        class executes(ObjectProperty):
            domain = [Agent]
            range = [PlanAction]
        class applies(ObjectProperty):
            domain = [Agent]
            range = [Thing] # AgentPlan in formal
        class establishes(ObjectProperty):
            domain = [AgentAction]
            range = [Fact]
        class has_precondition(ObjectProperty):
            domain = [AgentAction]
            range = [Situation]
        class is_implemented_by(ObjectProperty):
            domain = [PlanAction]
            range = [AgentAction]
        class hasAreaType(ObjectProperty):
            domain = [WarehouseArea]
            range = [AreaType]
        
        # Data Properties for Waypoints (needed for the controller)
        class hasX(DataProperty): range = [float]
        class hasY(DataProperty): range = [float]
        class hasOrder(DataProperty): range = [int]
        class hasLabel(DataProperty): range = [str]
        
        # Waypoint Classes
        class Waypoint(Thing): pass
        class OutboundWaypoint(Waypoint): pass
        class InboundWaypoint(Waypoint): pass

        # Individuals from EWHR.owl
        pr2 = Robot("PR2_001")
        
        # Situations
        startup_sit = Situation("System_Startup_Sit")
        base_sit = Situation("At_Base_Station_Sit")
        human_sit = Situation("Human_Proximity_Sit")
        lift_ready_sit = Situation("Box_Lift_Ready_Sit")
        holding_sit = Situation("Holding_Box_Sit")
        complete_sit = Situation("Mission_Complete_Sit")
        
        # Actions
        health_check = PlanAction("System_Health_Check")
        move_to = PlanAction("Movement_To")
        reduce_speed = PlanAction("Reduce_Speed_Instruction")
        pickup = PlanAction("Pickup_Target")
        move_from = PlanAction("Movement_From")
        log_data = PlanAction("Log_Mission_Data")
        
        # Implementation mappings (Agent Actions)
        set_vel = AgentAction("Set_Motor_Velocity_0.5")
        reduce_speed.is_implemented_by = [set_vel]
        set_vel.has_precondition = [human_sit]
        
        # Norms
        yield_norm = Obligation("Yield_To_Human_Norm")
        
        # Rules (mimicking Manchester Rules)
        rule1 = Imp()
        rule1.set_as_rule("Robot(?r), recognizes(?r, Human_Proximity_Sit) -> executes(?r, Reduce_Speed_Instruction)")
        
        # Add basic waypoints to ensure load_waypoints doesn't crash initially
        # These will be updated by update_onto_v8.py later
        ow1 = OutboundWaypoint("OutboundWaypoint1")
        ow1.hasX = [0.0]; ow1.hasY = [0.0]; ow1.hasOrder = [1]; ow1.hasLabel = ["Start"]

    onto.save(file="final/controllers/pr2_autonomous_ethics/EWHR_formal.owl", format="rdfxml")
    print("Created EWHR_formal.owl (RDF/XML version of formal EWHR.owl)")

if __name__ == "__main__":
    convert_manchester_to_rdf()
