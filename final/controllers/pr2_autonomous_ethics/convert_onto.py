from owlready2 import *

def create_integrated_ewhr(output_file):
    onto = get_ontology("http://ewhr.org/integrated.owl")
    
    with onto:
        class Agent(Thing): pass
        class Robot(Agent): pass
        class Situation(Thing): pass
        class PlanAction(Thing): pass
        
        class recognizes(ObjectProperty): pass
        class executes(ObjectProperty): pass
            
        # EWHR Individual: Robot
        pr2 = Robot("PR2_001")
        
        # Mapping Situations to Actions
        mappings = {
            "System_Startup_Sit": "System_Health_Check",
            "At_Base_Station_Sit": "Movement_To",
            "Human_Proximity_Sit": "Set_Motor_Velocity_0.5",
            "Box_Lift_Ready_Sit": "Pickup_Target",
            "Holding_Box_Sit": "Movement_From",
            "At_Assembly_Line_Sit": "Drop_off",
            "Mission_Complete_Sit": "Log_Mission_Data"
        }
        
        for sit_name, act_name in mappings.items():
            sit = Situation(sit_name)
            act = PlanAction(act_name)
            
            # Create Rule: If Robot recognizes Situation -> It executes PlanAction
            rule = Imp()
            rule.set_as_rule(f"Robot(?r), recognizes(?r, {sit_name}) -> executes(?r, {act_name})")

    onto.save(file=output_file, format="rdfxml")
    print(f"Created VERIFIED EWHR RDF/XML at {output_file}")

if __name__ == "__main__":
    create_integrated_ewhr("EWHR_integrated.owl")
