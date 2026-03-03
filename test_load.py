from owlready2 import *

# Try to load the Manchester syntax file
try:
    onto = get_ontology("ERAS-NEP-owl-manchester-axioms.owl").load()
    print("Successfully loaded Manchester syntax file.")
except Exception as e:
    print(f"Failed to load: {e}")
