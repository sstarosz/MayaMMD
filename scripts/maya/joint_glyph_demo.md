# Joint visualization gallery

Run this in Maya's Python Script Editor with the current MayaMMD plugin installed:

```python
path = r"<repo>/scripts/maya/joint_glyph_demo.py"
exec(compile(open(path, encoding="utf-8").read(), path, "exec"))
```

This adds a new `MMD_ConstraintGallery#` group and opens a dedicated isolated
viewport. It never clears, replaces, or saves your scene. Choose a case in the
list to select its joint and frame its two bodies. Alt-drag to orbit;
**Show entire gallery** frames the complete layout and clears selection.

**Step simulation 30 frames** and **Reset simulation** change only gallery solver
time, not your scene timeline. Gravity and collisions are disabled so examples
remain readable. Fixed-offset targets begin unsatisfied and move toward their
target when stepped. The separated-pair diagnostic has two kinematic bodies;
it intentionally cannot be corrected by Bullet.

The 18 cases are all PMX 2.0 Spring6Dof joints (zero stiffness behaves as a plain
6DOF joint). They cover line/plane/box translation regions, fixed offsets, free
and mixed axes, angular limits and offsets, linear/angular springs, a rotated
frame, and dynamic/dynamic and hybrid body pairs. Box, sphere, and capsule
colliders cycle through the examples. This is a representative set, not every
possible numeric configuration.

Unselected hubs show the joint's freedom state by colour. A/B use the
violet-ring / gold-diamond display, and motion guide colours mean local X/Y/Z
(see `docs/JointVisualization.md`). Select the joint, not its body, to reveal its
guides and springs. Hidden driver joints can be shown through the
Outliner to inspect bone write-back. Case names and descriptions live in the
browser and on each case group's `description`.

The explanation panel reads the selected case's current attributes and reports
LOCKED / LIMITED / FREE with angles in degrees. Use **Refresh parameter explanation
after editing** to update it without reframing. Values outside +/-180 degrees are
reported as wrapped; the script does not clamp or rewrite parameters.

Case 05 uses a white cube for the required B attachment position. The gold
diamond is its actual position; the dashed line measures the error. Case 10's
dashed angular sweep measures a fixed angle and does not mean an allowed range.
Case 16 starts with a fixed offset error so stepping shows both bodies share the
correction. Case 17 exposes its hybrid driver bone. Case 18 is intentionally
unsatisfiable because both bodies are driven by animation.

For scripting without the browser:

```python
import runpy
api = runpy.run_path(path, run_name="gallery_library")
gallery = api["build"](show_ui=False)
api["inspect"](gallery, 8)  # zero-based: angular limits
api["step"](gallery, 30)
api["reset"](gallery)
```

Repeated runs create independent galleries. Close the browser and delete its
`MMD_ConstraintGallery#` group when finished. Building a gallery is one undo chunk;
scene selection changes and later simulation stepping are separate actions.

`tests/integration/maya/cmds/test_pmx_rigid_body_constraint_cmd_integration.py`
builds and steps the gallery in Maya standalone. The browser's rendered
appearance needs interactive verification.
