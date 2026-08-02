import maya.cmds as cmds


def print_hierarchy(node, dash_count=0):
    """
    Recursively print the DAG hierarchy under 'node'.
    'dash_count' controls how many dashes to prefix (increases per level).
    """
    # Print current node with dashes (no leading dash at root level, or add dash if desired)
    # To make it look like a tree, we'll use dashes and a space
    print("-" * dash_count + node if dash_count > 0 else node)

    # Get all child transforms (exclude shapes by default)
    children = cmds.listRelatives(node, children=True, type="transform") or []
    for child in children:
        print_hierarchy(child, dash_count + 1)


def print_selected_hierarchy():
    """Print the hierarchy for all currently selected transform nodes using dashes."""
    selected = cmds.ls(selection=True, type="transform")
    if not selected:
        print("No transform selected. Please select an object.")
        return

    for obj in selected:
        print(f"\n--- Hierarchy for: {obj} ---")
        print_hierarchy(obj)


# Run the function
print_selected_hierarchy()
