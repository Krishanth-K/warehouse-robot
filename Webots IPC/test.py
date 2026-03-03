from rdflib import Graph, RDF, RDFS, OWL, URIRef, BNode
import networkx as nx
import os


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OWL_FILE = os.path.join(BASE_DIR, "fullaxiom.owl")

CMS_NS = "http://example.org/cms#"


g = Graph()
g.parse(OWL_FILE)

print("OWL file loaded")
print("Total triples:", len(g))


class_graph = nx.DiGraph()

classes = set()

for c in g.subjects(RDF.type, OWL.Class):
    classes.add(c)

for s, _, _ in g.triples((None, RDFS.subClassOf, None)):
    classes.add(s)

print("Detected classes:", len(classes))

for c in classes:
    class_graph.add_node(str(c))

for s, _, o in g.triples((None, RDFS.subClassOf, None)):
    s_uri = str(s)

    if isinstance(o, URIRef):
        class_graph.add_edge(s_uri, str(o), label="subClassOf")

    elif isinstance(o, BNode):
        class_graph.add_edge(s_uri, str(o), label="restriction")

print("Class graph nodes:", class_graph.number_of_nodes())
print("Class graph edges:", class_graph.number_of_edges())


print("\n--- GRAPH PREVIEW (first 25 edges) ---")
for i, (u, v, data) in enumerate(class_graph.edges(data=True)):
    print(f"{u.split('#')[-1]} --[{data['label']}]--> {v.split('#')[-1]}")
    if i >= 24:
        break


def find_class(name):
    """Find ontology class URI by short name"""
    for node in class_graph.nodes:
        if node.endswith("#" + name):
            return node
    return None

def has_path(child, parent):
    """Check if child ⊑ parent"""
    try:
        return nx.has_path(class_graph, child, parent)
    except nx.NodeNotFound:
        return False


print("\n--- SYMBOL LOOKUP (CORA CLASSES) ---")
symbols = ["Human", "Agent", "Process", "Interaction", "Physical"]

for s in symbols:
    print(f"{s} →", "FOUND" if find_class(s) else "NOT FOUND")


print("\n--- DOMAIN CLASS EXTENSION (CMS → CORA) ---")

domain_mappings = {
    "Human": "Agent",
    "HumanBody": "Physical",
    "Interaction": "Process"
}

cms_classes = {}

for child, parent in domain_mappings.items():
    parent_uri = find_class(parent)

    if parent_uri:
        child_uri = CMS_NS + child
        class_graph.add_node(child_uri)
        class_graph.add_edge(child_uri, parent_uri, label="subClassOf")
        cms_classes[child] = child_uri

        print(f"{child} ⊑ {parent}")
    else:
        print(f"FAILED: Parent class {parent} not found")


print("\n--- SEMANTIC RELATION CHECKS ---")

if "Human" in cms_classes:
    print(
        "Human ⊑ Agent:",
        has_path(cms_classes["Human"], find_class("Agent"))
    )

if "Interaction" in cms_classes:
    print(
        "Interaction ⊑ Process:",
        has_path(cms_classes["Interaction"], find_class("Process"))
    )

if "HumanBody" in cms_classes:
    print(
        "HumanBody ⊑ Physical:",
        has_path(cms_classes["HumanBody"], find_class("Physical"))
    )


print("\n--- FINAL GRAPH SUMMARY ---")
print("Total nodes:", class_graph.number_of_nodes())
print("Total edges:", class_graph.number_of_edges())
