"""Additive MayaMMD constraint gallery (Maya 2026/2027).

Run in Maya's Python Script Editor:
    exec(compile(open(r"<repo>/scripts/maya/joint_glyph_demo.py",
                      encoding="utf-8").read(), "joint_glyph_demo.py", "exec"))

Creates a uniquely named group and optional dedicated viewport. Never opens,
clears or saves a scene. Repeated runs create independent galleries.
All distances are centimeters; angular limits below are radians (converted to
Maya's UI angle unit when written). UI names are not
viewport labels. Select a case to reveal its motion guides (Ranges draw mode).
"""
from __future__ import annotations

import math
from maya import cmds
from maya.api import OpenMaya as om


def case(name, note="", **settings):
    return dict(name=name, note=note, **settings)


# PMX 2.0 joints are all Spring6DOF; zero stiffness gives a plain 6DOF joint.
CASES = [
    case("01 Welded", note="All six axes locked at zero; both attachments coincide."),
    case("02 Line", linearMin=(-1.5, 0, 0), linearMax=(1.5, 0, 0),
         note="Only X translates. The white capped ruler measures the allowed segment; short witness lines connect it to the real endpoints."),
    case("03 Plane", linearMin=(-1.2, -.8, 0), linearMax=(1.2, .8, 0),
         note="X and Y translate; Z is locked. White rectangle."),
    case("04 Volume", linearMin=(-1, -.8, -.6), linearMax=(1, .8, .6),
         note="All three translation axes limited. Orbit to inspect the box."),
    case("05 Fixed offset", linearMin=(1.2, .6, 0), linearMax=(1.2, .6, 0),
         note="White cube = required B attachment position (1.2, 0.6, 0) relative to A. Gold diamond = current B. Dashed line = error, not permitted travel. Step to approach the target."),
    case("06 Offset plane", linearMin=(-1, -.8, 1), linearMax=(1, .8, 1),
         note="A plane offset by one centimeter in Z; not centered on attachment A."),
    case("07 Free translation", linearMin=(1, 1, 1), linearMax=(-1, -1, -1),
         note="Inverted endpoints mean unlimited motion. Open arrows are indicators, not boundaries."),
    case("08 Mixed axes", linearMin=(1, -.8, .5), linearMax=(-1, .8, .5),
         note="X free, Y limited, Z fixed at an offset; the segment is an unbounded-region cross-section."),
    case("09 Angular limits", angularMin=(-.6, -.35, -.9), angularMax=(.6, .35, .9),
         note="Red/green/blue per-axis angle guides; they are not one combined rotation envelope."),
    case("10 Angular offset", angularMin=(.5, 0, 0), angularMax=(.5, 0, 0),
         note="X rotation locked at 28.65 degrees. Solid spoke/square = target; dashed sweep measures its offset from zero, NOT an allowed range."),
    case("11 Free rotation", angularMin=(1, 1, 1), angularMax=(-1, -1, -1),
         note="Rotation free on every axis; full rings without end stops."),
    case("12 Linear springs", linearMin=(-1, -1, -1), linearMax=(1, 1, 1),
         linearStiffness=(20, 35, 50), note="Straight zigzags indicate spring axes. Zero cross is equilibrium."),
    case("13 Angular springs", angularMin=(-.7, -.7, -.7), angularMax=(.7, .7, .7),
         angularStiffness=(10, 20, 30), note="Sprung rotation axes draw their arc as a zigzag across the limits; size does not represent stiffness."),
    case("14 Combined springs", linearMin=(-1, 0, 0), linearMax=(1, 0, 0),
         angularMin=(0, 0, -.6), angularMax=(0, 0, .6), linearStiffness=(25, 0, 0),
         angularStiffness=(0, 0, 15), note="X translation spring and Z rotation spring with separate limits."),
    case("15 Rotated attachment frame", linearMin=(-1.3, -.5, 0), linearMax=(1.3, .5, 0),
         rotation=(25, 35, 40), note="Motion region rotates with attachment A; tests local/world-space alignment."),
    case("16 Two simulated bodies", modes=("physics", "physics"),
         linearMin=(1, 0, 0), linearMax=(1, 0, 0), note="Initial offset target is unsatisfied. Step: both simulated bodies move to share the correction; compare case 05 where A stays put."),
    case("17 Hybrid B", modes=("followBone", "physicsBone"),
         angularMin=(-.5, -.5, -.5), angularMax=(.5, .5, .5),
         note="Hybrid B writes simulated rotation to its visible driver bone, while that bone's translation remains animation-driven. This is a body mode, not a different joint type."),
    case("18 Separated attachments diagnostic", modes=("followBone", "followBone"),
         separated=True, note="Both bodies are animation-driven; B's driver is displaced deliberately. Bullet cannot correct this pair."),
]


_LIMITS = ("linearMin", "linearMax", "angularMin", "angularMax", "linearStiffness", "angularStiffness")


def _shape(solver, array, index):
    return cmds.listConnections(f"{solver}.{array}[{index}]", s=True, d=False, plugs=True)[0].rsplit(".", 1)[0]


def _distance(value):
    return om.MDistance(value, om.MDistance.kCentimeters).asUnits(om.MDistance.uiUnit())


def _angle(value):
    return om.MAngle(value, om.MAngle.kDegrees).asUnits(om.MAngle.uiUnit())


def _vec_attr(node, name, value):
    attr = "joint" + name[0].upper() + name[1:]
    if name.startswith("angular") and not name.endswith("Stiffness"):
        unit = om.MAngle.uiUnit()
        cmds.setAttr(f"{node}.{attr}", *(om.MAngle(v, om.MAngle.kRadians).asUnits(unit) for v in value))
    else:
        cmds.setAttr(f"{node}.{attr}", *value, type="double3")


def build(show_ui=True):
    """Return a gallery dictionary; no existing nodes are replaced or deleted."""
    if not cmds.pluginInfo("MayaMMD", query=True, loaded=True):
        cmds.loadPlugin("MayaMMD")
    previous = cmds.ls(selection=True, long=True) or []
    cmds.undoInfo(openChunk=True, chunkName="MMD constraint gallery")
    try:
        root = cmds.createNode("transform", name="MMD_ConstraintGallery#")
        entries = []
        for i, spec in enumerate(CASES):
            # Groups remain identity: native commands store body rest poses in world space.
            group = cmds.createNode("transform", name=f"case_{i + 1:02d}", parent=root)
            solver = cmds.createNode("pmxRigidBodyNode", name=f"gallerySolver_{i + 1:02d}", parent=group)
            cmds.setAttr(solver + ".gravity", 0, 0, 0, type="double3")
            x, y = (i % 6) * 8.0, (math.ceil(len(CASES) / 6) - 1 - i // 6) * 8.0 + 4.0
            modes = spec.get("modes", ("followBone", "physics"))
            bones = []
            bodies = []
            for end in range(2):
                px = x + (-1.8 if end == 0 else 1.8)
                bone = cmds.createNode("joint", name=f"driver_{i + 1:02d}_{end}", parent=group)
                cmds.setAttr(bone + ".translate", _distance(px), _distance(y), 0, type="double3")
                # Persist an explicit rest transform for the plugin's anchor/write-back logic.
                for name, values in (("pmxRestTranslate", (px, y, 0)),
                                     ("pmxRestRotate", (0, 0, 0)),
                                     ("pmxRestJointOrient", (0, 0, 0))):
                    cmds.addAttr(bone, longName=name, attributeType="double3")
                    for axis in "XYZ":
                        cmds.addAttr(bone, longName=name + axis, attributeType="double", parent=name)
                    cmds.setAttr(bone + "." + name, *values, type="double3")
                cmds.setAttr(bone + ".visibility", modes[end] == "physicsBone")
                shape_kind = ("box", "sphere", "capsule")[(i + end) % 3]
                idx = cmds.pmxRigidBody(solver, name=f"case{i + 1:02d}_body{'AB'[end]}",
                    bone=bone, physicsMode=modes[end], shape=shape_kind,
                    size=(.55, .9, .55), position=(px, y, 0), mass=1,
                    linearDamping=.3, angularDamping=.3, group=(5 if end == 0 else 6), mask=0)
                body = _shape(solver, "bodyShapes", idx)
                cmds.setAttr(body + ".drawMode", 1)  # wire; no filled collider obscures attachments
                bodies.append(body)
                bones.append(bone)
            index = cmds.pmxRigidBodyConstraint(solver, name=spec["name"], bodyA=0, bodyB=1,
                                                type=0, position=(x, y, 0))
            joint = _shape(solver, "jointShapes", index)
            # Set Maya-space attributes explicitly; command flags convert MMD handedness.
            for attr in _LIMITS:
                _vec_attr(joint, attr, spec.get(attr, (0, 0, 0)))
            transform = cmds.listRelatives(joint, parent=True, fullPath=True)[0]
            cmds.setAttr(transform + ".rotate", *map(_angle, spec.get("rotation", (0, 0, 0))), type="double3")
            cmds.addAttr(group, longName="description", dataType="string")
            cmds.setAttr(group + ".description", spec["note"], type="string")
            # Gallery time is independent of the user's timeline; Step/Reset controls only these solvers.
            cmds.setAttr(solver + ".time", 1)
            cmds.getAttr(solver + ".outGuideTranslate[0]")
            if spec.get("separated"):
                cmds.setAttr(bones[1] + ".translateY", _distance(y + 1.5))
                cmds.setAttr(bones[1] + ".rotateZ", _angle(25))
                cmds.getAttr(solver + ".outGuideTranslate[1]")
            entries.append(dict(spec=spec, group=group, solver=solver, joint=joint,
                                bodies=bodies, bones=bones))
        result = dict(root=root, cases=entries, panel=None, camera=None)
    finally:
        cmds.select(previous, replace=True) if previous else cmds.select(clear=True)
        cmds.undoInfo(closeChunk=True)
    if show_ui and not cmds.about(batch=True):
        _show_browser(result)
    print(f"Created {len(entries)} constraint cases under {root}; no existing scene content replaced.")
    return result


def step(gallery, frames=30):
    """Advance just this gallery sequentially; keep Maya's current timeline unchanged."""
    for _ in range(frames):
        for entry in gallery["cases"]:
            solver = entry["solver"]
            cmds.setAttr(solver + ".time", cmds.getAttr(solver + ".time") + 1)
            cmds.getAttr(solver + ".outGuideTranslate[1]")
    if not cmds.about(batch=True):
        cmds.refresh()


def reset(gallery):
    for entry in gallery["cases"]:
        cmds.setAttr(entry["solver"] + ".time", 0)
        cmds.getAttr(entry["solver"] + ".outGuideTranslate[1]")


def inspect(gallery, index):
    entry = gallery["cases"][index]
    cmds.select(entry["group"], replace=True)
    if gallery["camera"]:
        cmds.viewFit(gallery["camera"], fitFactor=.75)
    cmds.select(entry["joint"], replace=True)
    return entry


def describe_joint(joint):
    """Read current attributes; display effective modes without altering inputs."""
    values = {key: cmds.getAttr(joint + ".joint" + key[0].upper() + key[1:])[0]
              for key in _LIMITS}
    unit = om.MAngle.uiUnit()
    for key in ("angularMin", "angularMax"):
        values[key] = tuple(om.MAngle(v, unit).asRadians() for v in values[key])
    lines = ["Spring 6DOF", "Distances: cm | angles: degrees", ""]

    def mode(lo, hi, angular=False):
        original = (lo, hi)
        if angular:
            lo, hi = (math.atan2(math.sin(v), math.cos(v)) for v in (lo, hi))
        warning = " [wrapped from outside +/-180 deg]" if angular and any(abs(v) > math.pi for v in original) else ""
        scale = 180 / math.pi if angular else 1
        if lo > hi:
            return "FREE" + warning
        if lo == hi:
            return f"LOCKED at {lo * scale:.2f}" + warning
        return f"LIMITED {lo * scale:.2f} to {hi * scale:.2f}" + warning

    for axis, name in enumerate("XYZ"):
        lines.append(f"Move {name}: " + mode(values["linearMin"][axis], values["linearMax"][axis]))
    for axis, name in enumerate("XYZ"):
        lines.append(f"Rotate {name}: " + mode(values["angularMin"][axis], values["angularMax"][axis], True))
    lines.append("Angular dials show settings, not measured current angles.")
    for key, label in (("linearStiffness", "Translation springs"), ("angularStiffness", "Rotation springs")):
        lines.append(label + ": " + ", ".join(f"{axis}={v:g}" for axis, v in zip("XYZ", values[key])))
    return "\n".join(lines)


def _show_browser(gallery):
    window = cmds.window(title="MMD constraint gallery", widthHeight=(1180, 750))
    split = cmds.paneLayout(configuration="vertical2", paneSize=(1, 28, 100))
    cmds.columnLayout(adjustableColumn=True)
    cmds.text(label="Select a case to show its motion guides", align="left")
    listing = cmds.textScrollList(height=300, append=[c["spec"]["name"] for c in gallery["cases"]])
    description = cmds.scrollField(editable=False, wordWrap=True, height=300)
    def refresh_description(*_):
        indexes = cmds.textScrollList(listing, query=True, selectIndexedItem=True) or [1]
        entry = gallery["cases"][indexes[0] - 1]
        cmds.scrollField(description, edit=True, text=entry["spec"]["note"] + "\n\n" + describe_joint(entry["joint"]))
    def changed(*_):
        indexes = cmds.textScrollList(listing, query=True, selectIndexedItem=True) or [1]
        inspect(gallery, indexes[0] - 1)
        refresh_description()
    cmds.textScrollList(listing, edit=True, selectCommand=changed)
    cmds.button(label="Refresh parameter explanation after editing", command=refresh_description)
    cmds.button(label="Step simulation 30 frames", command=lambda *_: step(gallery))
    cmds.button(label="Reset simulation", command=lambda *_: reset(gallery))
    cmds.button(label="Show entire gallery", command=lambda *_: _overview(gallery))
    cmds.setParent(split)
    camera, camera_shape = cmds.camera(name="galleryCamera#")
    cmds.parent(camera, gallery["root"])
    cmds.setAttr(camera_shape + ".nearClipPlane", _distance(.01))
    cmds.viewPlace(camera, eye=list(map(_distance, (27, 22, 65))),
                   lookAt=list(map(_distance, (20, 16, 0))), up=(0, 1, 0))
    panel = cmds.modelPanel(camera=camera, menuBarVisible=False)
    cmds.modelEditor(panel, edit=True, rendererName="vp2Renderer", displayAppearance="smoothShaded",
                     grid=False, cameras=False)
    gallery.update(camera=camera, panel=panel)
    cmds.showWindow(window)
    cmds.select(gallery["root"], replace=True)
    cmds.isolateSelect(panel, state=True)
    cmds.isolateSelect(panel, loadSelected=True)
    cmds.select(clear=True)
    cmds.textScrollList(listing, edit=True, selectIndexedItem=1)
    changed()


def _overview(gallery):
    cmds.select([c["group"] for c in gallery["cases"]], replace=True)
    cmds.viewFit(gallery["camera"], fitFactor=.9)
    cmds.select(clear=True)


if __name__ == "__main__":
    gallery = build()
