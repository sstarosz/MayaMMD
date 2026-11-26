"""Build an additive bone/body/constraint gallery in Maya with MayaMMD loaded.

Execute this file in Maya's Python Script Editor, or use runpy.run_path and
build(show_ui=False). Distances below are centimeters. No scene is cleared,
saved, or connected to the timeline. Step/reset affect only this gallery.
See docs/RigidBodyArchitecture.md and joint_glyph_demo.py for limit examples.
"""

from __future__ import annotations

from maya import cmds
from maya.api import OpenMaya as om

MODES = ("followBone", "physics", "physicsBone")
CASES = [(f"{a} -> {b}", (a, b), True, False) for a in MODES for b in MODES] + [
    ("Boneless pin -> physics", ("followBone", "physics"), False, False),
    ("Boneless physics -> physics", ("physics", "physics"), False, False),
    ("Bone chain", ("followBone", "physics", "physicsBone"), True, True),
]


def _distance(value):
    return om.MDistance(value, om.MDistance.kCentimeters).asUnits(om.MDistance.uiUnit())


def _shape(solver, array, index):
    return cmds.listConnections(
        f"{solver}.{array}[{index}]", source=True, destination=False, plugs=True
    )[0].rsplit(".", 1)[0]


def _bone(name, parent, index, local):
    bone = cmds.createNode("joint", name=name, parent=parent)
    cmds.setAttr(bone + ".translate", *map(_distance, local), type="double3")
    cmds.setAttr(bone + ".radius", _distance(0.18))
    cmds.addAttr(bone, longName="pmxBoneIndex", attributeType="long")
    cmds.setAttr(bone + ".pmxBoneIndex", index)
    for attr, values in (
        ("pmxRestTranslate", local),
        ("pmxRestRotate", (0, 0, 0)),
        ("pmxRestJointOrient", (0, 0, 0)),
    ):
        cmds.addAttr(bone, longName=attr, attributeType="double3")
        for axis in "XYZ":
            cmds.addAttr(
                bone, longName=attr + axis, attributeType="double", parent=attr
            )
        cmds.setAttr(bone + "." + attr, *values, type="double3")
    tip = cmds.createNode("joint", name=name + "_tip", parent=bone)
    cmds.setAttr(tip + ".translateY", _distance(1.2))
    cmds.setAttr(tip + ".radius", _distance(0.1))
    return bone


def build(show_ui=True):
    """Create 12 independent cases and return their node names.

    Args:
        show_ui: Open a small case selector with step/reset controls.

    Returns:
        Dictionary containing root, cases, and per-case solver/body/bone nodes.
    """
    if not cmds.pluginInfo("MayaMMD", query=True, loaded=True):
        cmds.loadPlugin("MayaMMD")
    previous = cmds.ls(selection=True, long=True) or []
    cmds.undoInfo(openChunk=True, chunkName="MMD physics flow gallery")
    try:
        root = cmds.createNode("transform", name="MMD_FlowGallery#")
        entries = []
        for number, (label, modes, with_bones, chain) in enumerate(CASES):
            prefix = f"{root}_case{number + 1:02d}"
            group = cmds.createNode("transform", name=prefix, parent=root)
            solver = cmds.createNode(
                "pmxRigidBodyNode", name=prefix + "_solver", parent=group
            )
            cmds.setAttr(solver + ".gravity", 0, -9.8, 0, type="double3")
            cmds.addAttr(group, longName="description", dataType="string")
            cmds.setAttr(group + ".description", label, type="string")
            x, y = (number % 3) * 12.0, 32.0 - (number // 3) * 8.0
            bones, bodies, constraints = [], [], []
            for index, mode in enumerate(modes):
                position = (x + index * 2.5, y - index, 0)
                bone = None
                if with_bones:
                    parent = bones[-1] if chain and index else group
                    local = (2.5, -1, 0) if chain and index else position
                    bone = _bone(prefix + f"_bone{index}", parent, index, local)
                    bones.append(bone)
                # A visible 0.5 cm body/bone offset demonstrates rest-pose K.
                body_index = cmds.pmxRigidBody(
                    solver,
                    name=f"body{index}_{mode}",
                    physicsMode=mode,
                    shape=("box", "sphere", "capsule")[(number + index) % 3],
                    size=(0.45, 0.7, 0.45),
                    position=(position[0], position[1] + 0.5, 0),
                    mass=1,
                    linearDamping=0.3,
                    angularDamping=0.3,
                    mask=0,
                    **({"bone": bone} if bone else {}),
                )
                body = _shape(solver, "bodyShapes", body_index)
                cmds.setAttr(body + ".drawMode", 1)
                bodies.append(body)
                if index:
                    joint_index = cmds.pmxRigidBodyConstraint(
                        solver,
                        name=f"link{index - 1}_{index}",
                        bodyA=index - 1,
                        bodyB=index,
                        position=(position[0] - 1.25, position[1] + 1, 0),
                        angularMin=(0, 0, -0.65),
                        angularMax=(0, 0, 0.65),
                        angularStiffness=(0, 0, 3),
                    )
                    constraints.append(_shape(solver, "jointShapes", joint_index))
            entries.append(
                {
                    "name": label,
                    "group": group,
                    "solver": solver,
                    "bones": bones,
                    "bodies": bodies,
                    "constraints": constraints,
                    "modes": modes,
                }
            )
        gallery = {"root": root, "cases": entries}
        reset(gallery)
    finally:
        cmds.select(previous, replace=True) if previous else cmds.select(clear=True)
        cmds.undoInfo(closeChunk=True)
    if show_ui and not cmds.about(batch=True):
        _show_ui(gallery)
    print(f"Created {root}: {len(entries)} cases. Select constraints to see limits.")
    return gallery


def _evaluate(entry):
    for index in range(len(entry["bodies"])):
        cmds.getAttr(f"{entry['solver']}.outGuideTranslate[{index}]")


def step(gallery, frames=30):
    """Advance this gallery by fixed 1/60-second ticks, leaving the timeline alone."""
    tick = om.MTime(1 / 60, om.MTime.kSeconds).asUnits(om.MTime.uiUnit())
    for _ in range(frames):
        for entry in gallery["cases"]:
            plug = entry["solver"] + ".time"
            cmds.setAttr(plug, cmds.getAttr(plug) + tick)
            _evaluate(entry)
    if not cmds.about(batch=True):
        cmds.refresh()


def reset(gallery):
    """Rewind each solver to its rest setup at the current anchor-bone pose."""
    for entry in gallery["cases"]:
        # Evaluate a distant time then rewind, even when already at zero.
        cmds.setAttr(entry["solver"] + ".time", 100000)
        _evaluate(entry)
        cmds.setAttr(entry["solver"] + ".time", 0)
        _evaluate(entry)


def inspect(gallery, index):
    """Select a case's constraint and return nodes for Node Editor exploration."""
    entry = gallery["cases"][index]
    cmds.select(entry["constraints"], replace=True)
    print(entry)
    return entry


def _show_ui(gallery):
    window = cmds.window(
        title=gallery["root"] + " — bone / body flow", widthHeight=(540, 420)
    )
    cmds.columnLayout(adjustableColumn=True)
    cmds.text(label="Bone tips show rotation. Wire colliders show the simulated pose.")
    listing = cmds.textScrollList(
        append=[entry["name"] for entry in gallery["cases"]], height=230
    )

    def select_case(*_):
        selected = cmds.textScrollList(listing, query=True, selectIndexedItem=True)
        if selected:
            inspect(gallery, selected[0] - 1)

    cmds.textScrollList(listing, edit=True, selectCommand=select_case)
    cmds.button(
        label="Select all gallery nodes (then Frame Selected)",
        command=lambda *_: cmds.select(gallery["root"], replace=True),
    )
    cmds.button(label="Step 30 ticks (0.5 seconds)", command=lambda *_: step(gallery))
    cmds.button(label="Reset simulation", command=lambda *_: reset(gallery))
    cmds.text(label="Gravity ON; collisions OFF; no ground. Free pairs fall together.")
    cmds.text(
        label="Move followBone bones; inspect physics bones' incoming connections."
    )
    cmds.showWindow(window)


if __name__ == "__main__":
    gallery = build()
