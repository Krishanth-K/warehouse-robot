from owlready2 import *

try:
    onto = get_ontology("../FN Robotics/controllers/pr2_py_controller/fullaxiom.owl").load()
    print("--- Classes ---")
    for c in list(onto.classes()):
        print(c.name)
    print("\n--- Properties ---")
    for p in list(onto.properties()):
        print(p.name)
except Exception as e:
    print(f"Error: {e}")
