import importlib
import logging
import os
import sys

import maya.api.OpenMaya as om
from maya import cmds
from PySide6 import QtCore, QtWidgets
from PySide6.QtGui import QAction
from PySide6.QtWidgets import QComboBox, QLabel, QWidgetAction
from shiboken6 import isValid as is_qt_obj_valid

from mmd.core.pmx_importer import dump_pmx_to_json, parse_pmx
from mmd.core.pmx_validate import validate_pmx_model
from mmd.core.vmd_importer import parse_vmd_file
from mmd.core.vpd_importer import parse_vpd_file
from mmd.maya.maya_data_types import ResolvedModelData
from mmd.maya.model_context import ModelContext
from mmd.maya.model_visibility import (
    PART_LABELS,
    apply_default_visibility,
    get_part_visibility,
    hidden_descendant_counts,
    reveal_all,
    set_all_parts_visibility,
    set_part_visibility,
)
from mmd.maya.pmx_model_utils import (
    discover_model_roots_in_scene,
    find_all_model_roots_from_selection,
    reset_model_to_bind_pose,
)
from mmd.maya.rigid_body_info import read_physics

log = logging.getLogger(__name__)
from mmd.maya.pmx_scene_builder import build_pmx_scene
from mmd.maya.vmd_scene_builder import apply_vmd_to_scene
from mmd.maya.vpd_scene_builder import apply_vpd_pose_to_scene
from mmd.ui.collapsible_section import CollapsibleSection
from mmd.ui.maya_section import MayaSection, embed_maya_section
from mmd.ui.morph_tree_widget import MorphTreeWidget
from mmd.ui.physics_panel import PhysicsPanel
from mmd.ui.visibility_panel import VisibilityPanel

_OPEN_PANEL: "MMMToolWidget | None" = None
"""The panel Maya's ``frameLayout`` commands call back into, if one is open."""


def _is_section_collapsed(section: object) -> bool:
    """Whether ``section`` is collapsed, for either section implementation.

    ``MayaSection`` clamps itself to its header; the Qt fallback expands and
    collapses on its own.  Both expose their state for lazy panel refreshes.
    """
    if section is None:
        return False
    if hasattr(section, "isCollapsed"):
        return bool(section.isCollapsed)
    if hasattr(section, "isExpanded"):
        return not bool(section.isExpanded())
    return False


PHYSICS_EXPAND_COMMAND = (
    'python("import mmd.ui.tool_main_widget as tw; tw.refresh_open_panel()")'
)
"""MEL run by the Physics frame when it is expanded (see refresh_open_panel)."""

MORPHS_SECTION_HEIGHT = 440
"""Fixed body height for six 40px morph rows plus model row, tabs and headers."""

PHYSICS_SECTION_HEIGHT = 680
"""Fixed expanded body height of the Physics section."""

COLLAPSED_SECTION_HEIGHT = 24
"""What a collapsed section is clamped to: just its Maya title bar."""


def refresh_open_panel() -> None:
    """Re-read the physics data of the open panel.

    Called from the Physics frame's ``expandCommand``, so collapsing and
    expanding the section is what picks up hand edits made in the Attribute
    Editor.
    """
    panel = _OPEN_PANEL
    if panel is None:
        return
    try:
        if is_qt_obj_valid(panel):
            panel._refresh_physics_panel()
    except RuntimeError:  # pragma: no cover - panel closed mid-call
        pass


# Force reload modules during development (set MAYAMMD_DEV=0 to disable;
# reloading re-executes module globals, so tests run with it off).
_DEV_RELOAD = os.environ.get("MAYAMMD_DEV", "1") == "1"
if _DEV_RELOAD and "mmd.maya.vmd_scene_builder" in sys.modules:
    importlib.reload(sys.modules["mmd.maya.vmd_scene_builder"])
    from mmd.maya.vmd_scene_builder import apply_vmd_to_scene
if _DEV_RELOAD and "mmd.maya.pmx_scene_builder" in sys.modules:
    importlib.reload(sys.modules["mmd.maya.pmx_scene_builder"])
    from mmd.maya.pmx_scene_builder import build_pmx_scene
if _DEV_RELOAD and "mmd.maya.pmx_model_utils" in sys.modules:
    importlib.reload(sys.modules["mmd.maya.pmx_model_utils"])
    from mmd.maya.pmx_model_utils import reset_model_to_bind_pose
if _DEV_RELOAD and "mmd.maya.vpd_scene_builder" in sys.modules:
    importlib.reload(sys.modules["mmd.maya.vpd_scene_builder"])
    from mmd.maya.vpd_scene_builder import apply_vpd_pose_to_scene
if _DEV_RELOAD and "mmd.maya.maya_data_types" in sys.modules:
    importlib.reload(sys.modules["mmd.maya.maya_data_types"])
if _DEV_RELOAD and "mmd.maya.model_context" in sys.modules:
    importlib.reload(sys.modules["mmd.maya.model_context"])
    from mmd.maya.model_context import ModelContext
if _DEV_RELOAD and "mmd.maya.model_visibility" in sys.modules:
    importlib.reload(sys.modules["mmd.maya.model_visibility"])
    from mmd.maya.model_visibility import (
        PART_LABELS,
        apply_default_visibility,
        get_part_visibility,
        hidden_descendant_counts,
        reveal_all,
        set_all_parts_visibility,
        set_part_visibility,
    )
if _DEV_RELOAD and "mmd.ui.morph_tree_widget" in sys.modules:
    importlib.reload(sys.modules["mmd.ui.morph_tree_widget"])
    from mmd.ui.morph_tree_widget import MorphTreeWidget
if _DEV_RELOAD and "mmd.ui.visibility_panel" in sys.modules:
    importlib.reload(sys.modules["mmd.ui.visibility_panel"])
    from mmd.ui.visibility_panel import VisibilityPanel
if _DEV_RELOAD and "mmd.maya.rigid_body_info" in sys.modules:
    importlib.reload(sys.modules["mmd.maya.rigid_body_info"])
    from mmd.maya.rigid_body_info import read_physics
if _DEV_RELOAD and "mmd.ui.collapsible_section" in sys.modules:
    importlib.reload(sys.modules["mmd.ui.collapsible_section"])
    from mmd.ui.collapsible_section import CollapsibleSection
if _DEV_RELOAD and "mmd.ui.physics_controls" in sys.modules:
    importlib.reload(sys.modules["mmd.ui.physics_controls"])
if _DEV_RELOAD and "mmd.ui.physics_panel" in sys.modules:
    importlib.reload(sys.modules["mmd.ui.physics_panel"])
    from mmd.ui.physics_panel import PhysicsPanel

if _DEV_RELOAD and "mmd.ui.maya_section" in sys.modules:
    importlib.reload(sys.modules["mmd.ui.maya_section"])
    from mmd.ui.maya_section import MayaSection, embed_maya_section


# Simple PySide widget (no MayaQWidgetDockableMixin)
class MMMToolWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent=parent)
        self.debug_dump_json = False
        self._selection_callback_id = None  # Maya API 2.0 callback handle
        self._import_in_progress = False  # Suppresses callback during ops
        self._pending_selection_refresh = False  # Selection was skipped during import

        # ── Selection-driven model context ────────────────────────────────
        self.ctx = ModelContext()
        self.ctx.modelChanged.connect(self._on_model_changed)

        # Menu Bar
        menu_bar = QtWidgets.QMenuBar(self)
        self.file_menu = menu_bar.addMenu("File")
        self.debug_menu = menu_bar.addMenu("Debug")
        self.about_menu = menu_bar.addMenu("About")

        # Log level drop-down
        log_levels = ["DEBUG", "INFO", "WARNING", "ERROR"]
        self.log_level_combo = QComboBox()
        self.log_level_combo.addItems(log_levels)
        self.log_level_combo.setCurrentText("INFO")
        log.setLevel(logging.INFO)  # Set initial log level
        self.log_level_combo.currentTextChanged.connect(self.set_log_level)

        logLabel = QtWidgets.QLabel("Log Level:")
        log_layout = QtWidgets.QHBoxLayout()
        log_layout.addWidget(logLabel)
        log_layout.addWidget(self.log_level_combo)

        # Add combo box to debug menu using QWidgetAction
        combo_action = QWidgetAction(self)
        combo_action.setDefaultWidget(QtWidgets.QWidget())
        combo_action.defaultWidget().setLayout(log_layout)
        self.debug_menu.addAction(combo_action)

        self.debug_action = QAction("Dump JSON", self, checkable=True)
        self.debug_action.setChecked(self.debug_dump_json)
        self.debug_action.toggled.connect(self.set_debug_dump_json)
        self.debug_menu.addAction(self.debug_action)

        # UI Elements

        self.main_layout = QtWidgets.QVBoxLayout()
        self.main_layout.setMenuBar(menu_bar)

        # The body scrolls: with both trees open the panel is taller than most
        # docks, and a section that is collapsed must not leave a gap — inside
        # a scroll area each section simply takes the height it needs.
        self._scroll_area = QtWidgets.QScrollArea(self)
        self._scroll_area.setWidgetResizable(True)
        self._scroll_area.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        self._scroll_area.setVerticalScrollBarPolicy(
            QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOn
        )
        self._scroll_area.setHorizontalScrollBarPolicy(
            QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff
        )
        scroll_content = QtWidgets.QWidget(self._scroll_area)
        self.content_layout = QtWidgets.QVBoxLayout(scroll_content)
        self.content_layout.setContentsMargins(0, 0, 0, 0)
        self.content_layout.setSpacing(4)
        self.content_layout.setSizeConstraint(
            QtWidgets.QLayout.SizeConstraint.SetMinAndMaxSize
        )
        self._scroll_area.setWidget(scroll_content)

        # Use Qt sections inside QScrollArea. Native frameLayout/paneLayout
        # controls keep Maya-managed geometry after reparenting and can clip
        # their content and scrollbars at the bottom of the viewport.

        # Buttons layout

        # Pmx Import/Export
        self.pmx_layout = QtWidgets.QVBoxLayout()
        self.pmx_label = QLabel("PMX Importer:")
        self.pmx_import_button = QtWidgets.QPushButton("Import PMX")
        self.pmx_import_button.clicked.connect(self.import_pmx)
        self.pmx_export_button = QtWidgets.QPushButton("Export PMX")
        self.pmx_export_button.setDisabled(True)  # Placeholder, not implemented yet
        self.pmx_export_button.setToolTip("PMX export not implemented yet")

        self.pmx_layout.addWidget(self.pmx_label)
        self.pmx_layout.addWidget(self.pmx_import_button)
        self.pmx_layout.addWidget(self.pmx_export_button)

        # Motion Data Import/Export (placeholder for future features)
        self.vmd_layout = QtWidgets.QVBoxLayout()
        self.vmd_label = QLabel("VMD Motion Data:")

        self.vmd_import_button = QtWidgets.QPushButton("Import VMD")
        self.vmd_import_button.clicked.connect(self.import_vmd)
        self.vmd_import_button.setEnabled(False)  # Disabled until PMX is imported
        self.vmd_import_button.setToolTip(
            "Import VMD motion data and apply to PMX model (requires PMX import first)"
        )

        self.vmd_export_button = QtWidgets.QPushButton("Export VMD")
        self.vmd_export_button.setDisabled(True)  # Placeholder, not implemented yet
        self.vmd_export_button.setToolTip("VMD export not implemented yet")

        self.vmd_layout.addWidget(self.vmd_label)
        self.vmd_layout.addWidget(self.vmd_import_button)
        self.vmd_layout.addWidget(self.vmd_export_button)

        # Pose Data Import/Export (placeholder for future features)
        self.pose_layout = QtWidgets.QVBoxLayout()
        self.pose_label = QLabel("Pose Data:")

        self.vpd_import_button = QtWidgets.QPushButton("Import Pose")
        self.vpd_import_button.clicked.connect(self.import_vpd)
        self.vpd_import_button.setEnabled(False)  # Disabled until PMX is imported
        self.vpd_import_button.setToolTip(
            "Import VPD pose data and apply to PMX model (requires PMX import first)"
        )

        self.vpd_export_button = QtWidgets.QPushButton("Export Pose")
        self.vpd_export_button.setDisabled(True)  # Placeholder, not implemented yet
        self.vpd_export_button.setToolTip("Pose export not implemented yet")

        self.pose_layout.addWidget(self.pose_label)
        self.pose_layout.addWidget(self.vpd_import_button)
        self.pose_layout.addWidget(self.vpd_export_button)

        # Model Operations
        self.model_ops_layout = QtWidgets.QVBoxLayout()
        self.model_ops_label = QLabel("Model Operations:")

        self.reset_pose_button = QtWidgets.QPushButton("Reset to Bind Pose")
        self.reset_pose_button.clicked.connect(self.reset_to_bind_pose)
        self.reset_pose_button.setEnabled(False)  # Disabled until PMX is imported
        self.reset_pose_button.setToolTip(
            "Reset model to its original bind pose (requires PMX import first)"
        )

        self.model_ops_layout.addWidget(self.model_ops_label)
        self.model_ops_layout.addWidget(self.reset_pose_button)

        # Import/Export layout
        self.import_export_layout = QtWidgets.QHBoxLayout()
        self.import_export_layout.addLayout(self.pmx_layout)
        self.import_export_layout.addLayout(self.vmd_layout)
        self.import_export_layout.addLayout(self.pose_layout)

        # Visibility section (self-contained widget inside a collapsible header,
        # so it matches the Morphs / Physics sections and can scroll).
        self.visibility_panel = VisibilityPanel()
        self.visibility_panel.match_row_height(
            self.pmx_import_button.sizeHint().height()
        )
        self.visibility_panel.partToggled.connect(self._on_visibility_toggled)
        self.visibility_panel.allToggled.connect(self._on_visibility_all_toggled)

        self.content_layout.addLayout(self.import_export_layout)

        self.visibility_section = embed_maya_section(
            self.content_layout,
            "Visibility",
            self.visibility_panel,
            fallback_factory=CollapsibleSection,
        )

        self.content_layout.addLayout(self.model_ops_layout)

        # Fixed-height sections share the outer scroll area. Their own trees
        # retain internal scrolling; unused dock space stays below the panels.
        self.morph_tree_widget = MorphTreeWidget()
        self.morphs_section = embed_maya_section(
            self.content_layout,
            "Morphs",
            self.morph_tree_widget,
            fallback_factory=self._fallback_section,
            height=MORPHS_SECTION_HEIGHT,
        )
        self._bind_section(self.morphs_section)

        # Physics section: the rigid-body / joint list + inspector.  The
        # panel owns no scene logic — it renders what `read_physics` returns
        # and asks the widget to select or delete.
        self.physics_panel = PhysicsPanel()
        self._physics_edit_refresh_pending = False
        self.physics_panel.inspector.attributeEdited.connect(
            self._on_physics_attribute_edited
        )
        self.physics_panel.selectionChanged.connect(self._on_physics_selection_changed)
        self.physics_panel.deleteRequested.connect(self._on_physics_delete_requested)

        # Guard against the select -> Maya callback -> sync -> select loop.
        self._applying_physics_selection = False

        self.physics_section = embed_maya_section(
            self.content_layout,
            "Physics",
            self.physics_panel,
            fallback_factory=self._fallback_section,
            height=PHYSICS_SECTION_HEIGHT,
        )
        self._bind_section(self.physics_section)

        self.content_layout.addStretch(1)

        self.main_layout.addWidget(self._scroll_area)
        # Tree starts empty - will be populated when a PMX model is imported
        log.debug("MorphTreeWidget created - waiting for model import")

        self.setLayout(self.main_layout)
        self.setWindowTitle("MayaMMD")

        # Let Maya's frameLayout expandCommand find the open panel.
        global _OPEN_PANEL
        _OPEN_PANEL = self

        # Install the selection-changed callback.  This is safe now because
        # _on_maya_selection_changed guards against both in-progress imports
        # (_import_in_progress) and destroyed C++ wrappers (_qt_valid()).
        self._start_selection_listener()
        self._physics_callbacks = []
        self._physics_scene_timer = QtCore.QTimer(self)
        self._physics_scene_timer.setSingleShot(True)
        self._physics_scene_timer.setInterval(100)
        self._physics_scene_timer.timeout.connect(self._refresh_physics_when_idle)
        self._start_physics_listener()

        # Populate from whatever is already selected: the panel is often opened
        # on a model that was targeted before, and Maya reports no selection
        # change for that.
        self.ctx.refresh_from_selection()
        self._refresh_button_states()
        self._refresh_physics_panel()

    def __del__(self) -> None:
        """Clean up the Maya callback when the Python wrapper is destroyed."""
        global _OPEN_PANEL
        if _OPEN_PANEL is self:
            _OPEN_PANEL = None
        self._stop_selection_listener()

    def _fallback_section(self, title: str, content: QtWidgets.QWidget):
        """A fixed-height Qt section when Maya's native frame is unavailable."""
        section = CollapsibleSection(title, content)
        section.setContentHeight(
            MORPHS_SECTION_HEIGHT if title == "Morphs" else PHYSICS_SECTION_HEIGHT
        )
        section.setSizePolicy(
            QtWidgets.QSizePolicy.Policy.Expanding,
            QtWidgets.QSizePolicy.Policy.Fixed,
        )
        return section

    def _bind_section(self, section: object) -> None:
        """Refresh the scene snapshot when Physics is expanded."""
        if getattr(section, "title", "") != "Physics":
            return
        if isinstance(section, MayaSection):
            section.on_toggle = self._on_physics_section_toggled_soon
        elif isinstance(section, CollapsibleSection):
            section.toggled.connect(self._on_physics_section_toggled)

    # ── Qt validity guard ─────────────────────────────────────────────────

    def _qt_valid(self) -> bool:
        """Check if the widget's C++ wrappers are still alive.

        Maya may destroy the underlying QWidget (e.g. on workspace control
        reset or scene close) while the Python object still exists.  All
        Qt-accessing methods must check this first.
        """
        return is_qt_obj_valid(self.morph_tree_widget) and is_qt_obj_valid(self)

    # ── Context-driven helpers ────────────────────────────────────────────

    def _refresh_button_states(self) -> None:
        """Enable/disable VMD/VPD/reset buttons based on context validity.

        Uses ``self.ctx.isValid`` (fast — checks cached root).
        """
        if not self._qt_valid():
            return
        has_models = self.ctx.isValid
        self.vmd_import_button.setEnabled(has_models)
        self.vpd_import_button.setEnabled(has_models)
        self.reset_pose_button.setEnabled(has_models)
        self._refresh_visibility_buttons()

    def _refresh_visibility_buttons(self) -> None:
        """Sync the visibility panel with the active model's part visibility.

        Toggles are disabled for parts the model does not have (e.g. rigid
        bodies on a model imported without physics), and the panel is told how
        many nodes are hidden individually so it can say why Show does not
        reveal them.
        """
        if not self._qt_valid():
            return
        root = self.ctx.rootName
        self.visibility_panel.sync(
            get_part_visibility(root), hidden_descendant_counts(root)
        )

    def _refresh_physics_panel(self) -> None:
        """Re-read all models' rigid bodies and joints into the panel.

        The panel reads the *scene*, not the PMX file, so this is also how a
        hand edit made in the Attribute Editor or the Node Editor reaches it.
        """
        if not self._qt_valid():
            return
        self.physics_panel.showScene(
            [read_physics(root) for root in discover_model_roots_in_scene()]
        )
        self.physics_panel.syncSelection(cmds.ls(selection=True, long=True) or [])

    def _schedule_physics_refresh(self, *_args: object) -> None:
        if self._qt_valid():
            self._physics_scene_timer.start()

    def _refresh_physics_when_idle(self) -> None:
        if not self._qt_valid():
            return
        if self._import_in_progress:
            self._physics_scene_timer.start()
            return
        self._refresh_physics_panel()

    def _start_physics_listener(self) -> None:
        """Coalesce structural edits; ordinary selection never reloads physics."""
        try:
            for node_type in ("pmxRigidBodyShape", "pmxRigidBodyJoint", "pmxRigidBodyNode"):
                for register in (om.MDGMessage.addNodeAddedCallback, om.MDGMessage.addNodeRemovedCallback):
                    self._physics_callbacks.append(
                        register(self._schedule_physics_refresh, node_type)
                    )
            for event in ("SceneOpened", "NewSceneOpened", "Undo", "Redo", "NameChanged"):
                self._physics_callbacks.append(
                    om.MEventMessage.addEventCallback(event, self._schedule_physics_refresh)
                )
        except (RuntimeError, TypeError, AttributeError) as exc:
            log.warning("Could not install all physics scene callbacks: %s", exc)

    def _on_physics_attribute_edited(
        self, node: str, attribute: str, value: object
    ) -> None:
        """Commit one editor gesture through Maya's undoable setAttr command."""
        plug = f"{node}.{attribute}"
        try:
            if not cmds.objExists(plug):
                raise RuntimeError(f"Attribute no longer exists: {plug}")
            if not cmds.getAttr(plug, settable=True):
                raise RuntimeError(f"Attribute is locked or connected: {plug}")
            current = cmds.getAttr(plug)
            if isinstance(value, tuple):
                if current and tuple(current[0]) == value:
                    return
                cmds.setAttr(plug, *value, type="double3")
            elif isinstance(value, str):
                if current == value:
                    return
                cmds.setAttr(plug, value, type="string")
            else:
                if current == value:
                    return
                cmds.setAttr(plug, value)
            self.physics_panel.applyAttributeEdit(node, attribute, value)
            return
        except (RuntimeError, ValueError, TypeError) as exc:
            log.warning("Could not edit %s: %s", plug, exc)
            cmds.warning(str(exc))
        # Only failed writes need a scene refresh to restore the authoritative
        # value. Successful edits retain their controls, focus and geometry.
        if not self._physics_edit_refresh_pending:
            self._physics_edit_refresh_pending = True
            QtCore.QTimer.singleShot(0, self._refresh_after_physics_edit)

    def _refresh_after_physics_edit(self) -> None:
        self._physics_edit_refresh_pending = False
        if self._qt_valid():
            self._refresh_physics_panel()

    def _on_physics_section_toggled(self, expanded: bool) -> None:
        """Re-read when the Physics section is expanded.

        The panel shows a snapshot, and there is no Refresh button: collapsing
        and re-expanding is the gesture that picks up hand edits.
        """
        if expanded:
            self._refresh_physics_panel()

    def _on_physics_section_toggled_soon(self) -> None:
        """:meth:`_on_physics_section_toggled` for a section with ``on_toggle``.

        ``MayaSection`` reports the new state through its own clamp, so the
        expansion is read back from the section instead of passed in.
        """
        section = self.physics_section
        if section is not None and not _is_section_collapsed(section):
            self._refresh_physics_panel()

    def _on_physics_selection_changed(self, node: str) -> None:
        """Select the node picked in the panel (switching model if needed)."""
        if not self._qt_valid() or self._applying_physics_selection or not node:
            return
        if not cmds.objExists(node):
            # Deleted behind our back: re-read rather than select nothing.
            self._refresh_physics_panel()
            return
        self._applying_physics_selection = True
        try:
            cmds.select(node, replace=True)
        except Exception as exc:
            log.warning("Could not select %s: %s", node, exc)
        finally:
            self._applying_physics_selection = False

    def _on_physics_delete_requested(self, paths: list[str]) -> None:
        """Delete the picked rigid bodies / joints after confirming.

        Only the picked nodes are removed — a body's joints are left alone, so
        they start reporting ``missing body`` in the same list rather than
        silently disappearing.  That keeps the delete surgical, which is what
        a read-only panel should do.
        """
        if not self._qt_valid() or not paths:
            return
        existing = [path for path in paths if cmds.objExists(path)]
        if not existing:
            self._refresh_physics_panel()
            return
        what = "node" if len(existing) == 1 else "nodes"
        answer = cmds.confirmDialog(
            title="Delete Physics Nodes",
            message=(
                f"Delete {len(existing)} physics {what} from the scene?\n\n"
                "Joints that referenced a deleted body are kept and will show "
                "as 'missing body'."
            ),
            button=["Delete", "Cancel"],
            defaultButton="Delete",
            cancelButton="Cancel",
            dismissString="Cancel",
        )
        if answer != "Delete":
            return
        self._import_in_progress = True
        try:
            cmds.delete(existing)
            log.debug("Deleted %d physics %s", len(existing), what)
        except Exception as exc:
            log.warning("Could not delete physics nodes: %s", exc)
        finally:
            self._import_in_progress = False
        self._refresh_physics_panel()

    def _on_visibility_toggled(self, part: str, visible: bool) -> None:
        """Show or hide one model part on every targeted model."""
        if not self._qt_valid():
            return
        roots = self._get_target_model_roots()
        if not roots:
            # The targeted model may have been deleted under us: put the button
            # back and disable it rather than leaving it flipped.
            self._refresh_visibility_buttons()
            return
        toggled = sum(set_part_visibility(root, part, visible) for root in roots)
        if not toggled:
            log.debug("No model had a %s part to toggle", part)
        else:
            log.debug(
                "%s %s on %d model(s)",
                "Showing" if visible else "Hiding",
                PART_LABELS[part],
                toggled,
            )
        # Re-read the scene: a model that lacks the part, or a rejected write,
        # would otherwise leave the button showing a state the scene is not in.
        self._refresh_visibility_buttons()

    def _on_visibility_all_toggled(self, visible: bool) -> None:
        """Show or hide every part of the targeted models (master button).

        Showing uses :func:`reveal_all`, which also clears per-node hiding -
        "Show All" must leave nothing invisible.  Hiding only gates the
        containers, so the nodes' own visibility survives for later.
        """
        if not self._qt_valid():
            return
        roots = self._get_target_model_roots()
        if roots:
            if visible:
                revealed = sum(reveal_all(root) for root in roots)
                log.debug(
                    "Revealed %d individually hidden node(s) across %d model(s)",
                    revealed,
                    len(roots),
                )
            else:
                hidden = sum(set_all_parts_visibility(root, False) for root in roots)
                log.debug("Hid every part on %d model(s)", hidden)
        self._refresh_visibility_buttons()

    # ── Maya API 2.0 selection change callback ────────────────────────────

    def _on_maya_selection_changed(self, client_data: object) -> None:
        """Callback fired by Maya whenever the active selection list changes."""
        if not self._qt_valid():
            return
        if self._import_in_progress:
            # Import is modifying the scene — don't re-query now, but flag so
            # we replay the selection after the import finishes.
            # Intermediate selection changes during import are intentionally
            # collapsed: the flag is consumed in the finally block of each
            # import method (import_pmx/import_vmd/import_vpd), which calls
            # _on_maya_selection_changed(None) to read the final state.
            self._pending_selection_refresh = True
            return
        self.ctx.refresh_from_selection()
        # The scene-wide list remains intact; only its selection changes.
        if not self._applying_physics_selection:
            self.physics_panel.syncSelection(cmds.ls(selection=True, long=True) or [])

    def _start_selection_listener(self) -> None:
        """Install the selection-changed callback via Maya API 2.0."""
        if self._selection_callback_id is not None:
            return  # already registered
        try:
            cid = om.MModelMessage.addCallback(
                om.MModelMessage.kActiveListModified,
                self._on_maya_selection_changed,
                None,
            )
            self._selection_callback_id = cid
            log.debug("Selection listener started (callback %s)", cid)
        except Exception as exc:
            log.warning("Failed to install selection callback: %s", exc)

    def _stop_selection_listener(self) -> None:
        """Remove the selection-changed callback."""
        cid = self._selection_callback_id
        if cid is None:
            return
        try:
            om.MModelMessage.removeCallback(cid)
            log.debug("Selection listener stopped (removed %s)", cid)
        except Exception as exc:
            log.warning("Failed to remove selection callback %s: %s", cid, exc)
        finally:
            self._selection_callback_id = None

    def cleanup(self) -> None:
        """Release callbacks before Maya deletes the workspace control."""
        self._stop_selection_listener()
        if hasattr(self, "_physics_scene_timer") and is_qt_obj_valid(self._physics_scene_timer):
            self._physics_scene_timer.stop()
        for callback in getattr(self, "_physics_callbacks", []):
            try:
                om.MMessage.removeCallback(callback)
            except (RuntimeError, TypeError) as exc:
                log.debug("Could not remove physics callback %s: %s", callback, exc)
        self._physics_callbacks = []
        try:
            self.morph_tree_widget._cleanup_callbacks()
        except Exception:
            pass
        try:
            # Qt already disconnects slots when their receiver is destroyed.
            if is_qt_obj_valid(self) and self.ctx is not None and is_qt_obj_valid(self.ctx):
                self.ctx.modelChanged.disconnect(self._on_model_changed)
        except (RuntimeError, TypeError):
            pass
        for name in ("visibility_section", "morphs_section", "physics_section"):
            section = getattr(self, name, None)
            if isinstance(section, MayaSection):
                section.dispose()
        global _OPEN_PANEL
        if _OPEN_PANEL is self:
            _OPEN_PANEL = None

    def closeEvent(self, event) -> None:
        self.cleanup()
        super().closeEvent(event)

    def _on_model_changed(self, root_name: str) -> None:
        """React to active model change from context."""
        if not self._qt_valid():
            return
        if self._import_in_progress:
            # _resolve_multi swaps the active root which emits modelChanged;
            # we must not refresh the tree mid-import.
            self._pending_selection_refresh = True
            return
        # Refresh morph tree with all models in the scene
        self.morph_tree_widget.refresh()
        # Update button states
        # buttons should disable; when a new model is selected they enable.
        self._refresh_button_states()
        log.debug("Active model changed: %s", root_name or "(none)")

    def _get_target_model_roots(self) -> list[str]:
        """Resolve which PMX model roots to operate on.

        Returns roots from the current selection (allows multi-model
        operations when the user selects joints from multiple models).
        Falls back to the context's active root.  Returns empty list if
        no PMX models are available at all.
        """
        roots = find_all_model_roots_from_selection()
        if roots:
            return roots
        if self.ctx.isValid:
            return [self.ctx.rootName]
        return []

    def _resolve_multi(self, roots: list[str]) -> list["ResolvedModelData"]:
        """Resolve model data for one or more roots."""

        results = []
        for root in roots:
            self.ctx.set_active_root(root)
            results.append(self.ctx.resolve())
        return results

    def import_pmx(self):
        pmx_file = cmds.fileDialog2(
            fileFilter="PMX Files (*.pmx)", dialogStyle=2, fileMode=1
        )

        if pmx_file:
            for file in pmx_file:
                log.debug("Selected PMX file: %s", file)

                # Suspend selection polling while we modify the scene
                self._import_in_progress = True

                try:
                    pmx_data = parse_pmx(file)

                    # Validate pmx_data before building scene
                    validation_result = validate_pmx_model(pmx_data)

                    # Display validation issues to the user (for now, just log them)
                    # TODO: Implement a UI panel to show these issues in a user-friendly way, with options to apply fixes
                    if validation_result.issues:
                        for issue in validation_result.issues:
                            log.warning("Validation issue - %s", issue)

                    maya_pmx_data = build_pmx_scene(pmx_data)

                    # Start from the panel's default part visibility so the
                    # toggles and the viewport agree right after import.
                    apply_default_visibility(maya_pmx_data.root_name)

                    # TODO(viewport-transparency): MMD models often contain
                    # near-invisible transparent overlay materials (e.g. the
                    # `face00` nose/cheek piece in YYB Hatsune Miku NT, whose
                    # face2.png is ~92% transparent).  Maya's default
                    # single-pass viewport transparency sorts these overlays
                    # wrong, making them render as see-through holes into the
                    # hollow head.  Confirmed fix (manual, works):
                    #   Viewport 2.0 -> Renderer settings ->
                    #     Transparency Algorithm = Depth Peeling
                    #     + enable Alpha Cut prepass.
                    # Task: apply these automatically on import via
                    # `hardwareRenderingGlobals` (verify exact attribute
                    # names/values for Maya 2026), ideally behind a toggle in
                    # the MMD UI since it changes global viewport settings.
                    if self.debug_dump_json:
                        dump_pmx_to_json(
                            pmx_data=pmx_data, output_path=pmx_data.absolute_path
                        )
                        log.info(
                            "Dumped PMX data to JSON at: %s", pmx_data.absolute_path
                        )

                    # Refresh context & UI for the newly imported model
                    self.ctx.set_active_root(maya_pmx_data.root_name)
                    self._refresh_button_states()

                    # Refresh morph tree with all models
                    self.morph_tree_widget.refresh()

                    log.info("Successfully imported PMX model: %s", pmx_data.model_name)
                except Exception as e:
                    log.error("Failed to import PMX file: %s", e)
                    import traceback

                    log.error(traceback.format_exc())
                    self.ctx.clear()
                    self._refresh_button_states()
                finally:
                    self._import_in_progress = False
                    if self._pending_selection_refresh:
                        self._pending_selection_refresh = False
                        self._on_maya_selection_changed(None)
        else:
            log.debug("No PMX file selected.")

    def import_vmd(self):
        roots = self._get_target_model_roots()
        if not roots:
            if not discover_model_roots_in_scene():
                cmds.confirmDialog(
                    title="No PMX Model",
                    message="No PMX models found in the scene.\n"
                    "Please import a PMX model first.",
                    button=["OK"],
                    defaultButton="OK",
                )
            return

        vmd_file = cmds.fileDialog2(
            fileFilter="VMD Files (*.vmd)", dialogStyle=2, fileMode=1
        )
        if vmd_file:
            log.debug("Selected VMD file: %s", vmd_file[0])

            # Suspend selection polling during scene modification
            self._import_in_progress = True
            try:
                # Parse VMD file once
                vmd_data = parse_vmd_file(vmd_file[0])

                # TODO display warning if vmd_data.model_name do not match the PMX model name(s) in the scene
                # This can couse issues if the VMD is not intended for the PMX model(s) in the scene.
                log.debug("Successfully parsed VMD file: %s", vmd_file[0])

                # Apply VMD animation to each targeted model
                for model in self._resolve_multi(roots):
                    apply_vmd_to_scene(
                        vmd_data,
                        model=model,
                        start_frame=1,
                    )
                    log.debug(
                        "VMD animation %s applied to model: %s",
                        vmd_file[0],
                        model.root_name,
                    )

                # Refresh morph tree to update keyframe indicators for
                # newly created animation curves on morph weights.
                if self._qt_valid():
                    self.morph_tree_widget.refresh()

            except Exception as e:
                log.error("Failed to import VMD file: %s", e)
                import traceback

                log.error(traceback.format_exc())
            finally:
                self._import_in_progress = False
                if self._pending_selection_refresh:
                    self._pending_selection_refresh = False
                    self._on_maya_selection_changed(None)
        else:
            log.debug("No VMD file selected.")

    def import_vpd(self):
        roots = self._get_target_model_roots()
        if not roots:
            if not discover_model_roots_in_scene():
                cmds.confirmDialog(
                    title="No PMX Model",
                    message="No PMX models found in the scene.\n"
                    "Please import a PMX model first.",
                    button=["OK"],
                    defaultButton="OK",
                )
            return

        vpd_file = cmds.fileDialog2(
            fileFilter="VPD Files (*.vpd)", dialogStyle=2, fileMode=1
        )

        if vpd_file:
            log.debug("Selected VPD file: %s", vpd_file[0])

            self._import_in_progress = True
            try:
                # Parse VPD file once
                vpd_data = parse_vpd_file(vpd_file[0])

                log.debug("Successfully parsed VPD file: %s", vpd_file[0])

                # Ask user if they want to create a keyframe
                result = cmds.confirmDialog(
                    title="Apply Pose",
                    message="Do you want to create a keyframe at the current frame?",
                    button=["Yes (Keyframe)", "No (Just Set Pose)", "Cancel"],
                    defaultButton="Yes (Keyframe)",
                    cancelButton="Cancel",
                    dismissString="Cancel",
                )

                if result == "Cancel":
                    log.debug("User cancelled VPD import")
                    return

                create_keyframe = result == "Yes (Keyframe)"

                # Apply VPD pose to each targeted model
                for model in self._resolve_multi(roots):
                    apply_vpd_pose_to_scene(
                        vpd_data=vpd_data,
                        model=model,
                        create_keyframe=create_keyframe,
                    )

                    log.debug(
                        "VPD pose %s applied to model: %s (create_keyframe=%s)",
                        vpd_file[0],
                        model.root_name,
                        create_keyframe,
                    )

            except Exception as e:
                log.error("Failed to import VPD file: %s", e)
                import traceback

                log.error(traceback.format_exc())
            finally:
                self._import_in_progress = False
                if self._pending_selection_refresh:
                    self._pending_selection_refresh = False
                    self._on_maya_selection_changed(None)
        else:
            log.debug("No VPD file selected.")

    def reset_to_bind_pose(self):
        """Reset the current PMX model(s) to their bind pose."""
        roots = self._get_target_model_roots()
        if not roots:
            if not discover_model_roots_in_scene():
                cmds.confirmDialog(
                    title="No PMX Model",
                    message="No PMX models found in the scene.\n"
                    "Please import a PMX model first.",
                    button=["OK"],
                    defaultButton="OK",
                )
            return

        self._import_in_progress = True
        try:
            total_bones = 0
            total_ik = 0
            for model in self._resolve_multi(roots):
                stats = reset_model_to_bind_pose(
                    model=model,
                )
                total_bones += stats.get("bones_reset", 0)
                total_ik += stats.get("ik_handles_reset", 0)

            log.debug(
                "Model reset to bind pose: %d bones, %d IK handles across %d model(s)",
                total_bones,
                total_ik,
                len(roots),
            )

            # Refresh morph tree with all models
            self.morph_tree_widget.refresh()
            log.debug("Refreshed morph tree widget after reset")

        except Exception as e:
            log.error("Failed to reset model to bind pose: %s", e)
            cmds.confirmDialog(
                title="Error",
                message=f"Failed to reset to bind pose:\n{e!s}",
                button=["OK"],
                defaultButton="OK",
            )
            import traceback

            log.error(traceback.format_exc())
        finally:
            self._import_in_progress = False
            if self._pending_selection_refresh:
                self._pending_selection_refresh = False
                self._on_maya_selection_changed(None)

    def set_debug_dump_json(self, checked: bool) -> None:
        self.debug_dump_json = checked

    def set_log_level(self, level_name: str) -> None:
        level_map = {
            "DEBUG": logging.DEBUG,
            "INFO": logging.INFO,
            "WARNING": logging.WARNING,
            "ERROR": logging.ERROR,
        }
        log.info("Setting log level to: %s", level_name)
        log.setLevel(level_map.get(level_name, logging.INFO))
