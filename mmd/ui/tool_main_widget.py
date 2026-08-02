import importlib
import logging
import sys

import maya.api.OpenMaya as om
import maya.OpenMayaUI as omui
from maya import cmds
from PySide6 import QtWidgets
from PySide6.QtGui import QAction
from PySide6.QtWidgets import QComboBox, QLabel, QWidgetAction
from shiboken6 import isValid as is_qt_obj_valid
from shiboken6 import wrapInstance

from mmd.core.pmx_importer import dump_pmx_to_json, parse_pmx
from mmd.core.pmx_validate import validate_pmx_model
from mmd.core.vmd_importer import parse_vmd_file
from mmd.core.vpd_importer import parse_vpd_file
from mmd.maya.model_context import ModelContext
from mmd.maya.pmx_model_utils import (
    discover_model_roots_in_scene,
    find_all_model_roots_from_selection,
    reset_model_to_bind_pose,
)

log = logging.getLogger(__name__)
from mmd.maya.pmx_scene_builder import build_pmx_scene
from mmd.maya.vmd_scene_builder import apply_vmd_to_scene
from mmd.maya.vpd_scene_builder import apply_vpd_pose_to_scene
from mmd.ui.morph_tree_widget import MorphTreeWidget

# Force reload modules during development
if "mmd.maya.vmd_scene_builder" in sys.modules:
    importlib.reload(sys.modules["mmd.maya.vmd_scene_builder"])
    from mmd.maya.vmd_scene_builder import apply_vmd_to_scene
if "mmd.maya.pmx_scene_builder" in sys.modules:
    importlib.reload(sys.modules["mmd.maya.pmx_scene_builder"])
    from mmd.maya.pmx_scene_builder import build_pmx_scene
if "mmd.maya.pmx_model_utils" in sys.modules:
    importlib.reload(sys.modules["mmd.maya.pmx_model_utils"])
    from mmd.maya.pmx_model_utils import reset_model_to_bind_pose
if "mmd.maya.vpd_scene_builder" in sys.modules:
    importlib.reload(sys.modules["mmd.maya.vpd_scene_builder"])
    from mmd.maya.vpd_scene_builder import apply_vpd_pose_to_scene
if "mmd.maya.maya_data_types" in sys.modules:
    importlib.reload(sys.modules["mmd.maya.maya_data_types"])
if "mmd.maya.model_context" in sys.modules:
    importlib.reload(sys.modules["mmd.maya.model_context"])
    from mmd.maya.model_context import ModelContext
if "mmd.ui.morph_tree_widget" in sys.modules:
    importlib.reload(sys.modules["mmd.ui.morph_tree_widget"])
    from mmd.ui.morph_tree_widget import MorphTreeWidget


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

        self.main_layout.addWidget(QLabel("MayaMMD Main Widget"))
        self.main_layout.addLayout(self.import_export_layout)
        self.main_layout.addLayout(self.model_ops_layout)

        # Create Qt-based pose tree widget (similar to Shape Editor)
        # Wrap in Maya's frameLayout for native Maya appearance

        # Convert Maya UI to Qt widget helper function
        def mayaToQtObject(inMayaUI):
            ptr = omui.MQtUtil.findControl(inMayaUI)
            if ptr is None:
                ptr = omui.MQtUtil.findLayout(inMayaUI)
            if ptr is None:
                ptr = omui.MQtUtil.findMenuItem(inMayaUI)
            if ptr is not None:
                return wrapInstance(int(ptr), QtWidgets.QWidget)

        # Create a Maya frameLayout (native collapsible section)
        frame_name = cmds.frameLayout(label="Morphs", collapsable=True, collapse=False)

        # paneLayout with configuration="single" makes its single child expand
        # to fill all available space — unlike columnLayout which respects
        # fixed control heights and ignores Qt size policies.
        cmds.paneLayout(configuration="single", backgroundColor=[0.286, 0.286, 0.286])

        # Create a plain Maya text control as a placeholder.
        # Qt will own the *inside* of this single control, while Maya continues
        # to manage the paneLayout and frameLayout geometry.  This avoids the
        # unsupported pattern of nesting a Qt layout inside a Maya layout.
        placeholder = cmds.text(label="")

        # Wrap the placeholder control as a Qt widget
        qt_placeholder = mayaToQtObject(placeholder)

        # Create the Qt tree widget
        self.morph_tree_widget = MorphTreeWidget()

        # Embed the tree widget inside the placeholder control via a QVBoxLayout.
        # Qt only manages the interior of one Maya control — a clean boundary.
        if qt_placeholder:
            layout = QtWidgets.QVBoxLayout(qt_placeholder)
            layout.addWidget(self.morph_tree_widget)

        # Now exit the layouts
        cmds.setParent("..")  # Exit paneLayout
        cmds.setParent("..")  # Exit frameLayout

        # Wrap the Maya frameLayout as a Qt widget and add to main layout
        qt_frame = mayaToQtObject(frame_name)
        if qt_frame:
            self.main_layout.addWidget(qt_frame, stretch=1)

        # Tree starts empty - will be populated when a PMX model is imported
        log.debug("MorphTreeWidget created - waiting for model import")

        self.main_layout.addStretch()
        self.setLayout(self.main_layout)
        self.setWindowTitle("Custom Maya WorkspaceControl Example")

        # Install the selection-changed callback.  This is safe now because
        # _on_maya_selection_changed guards against both in-progress imports
        # (_import_in_progress) and destroyed C++ wrappers (_qt_valid()).
        self._start_selection_listener()

    def __del__(self) -> None:
        """Clean up the Maya callback when the Python wrapper is destroyed."""
        self._stop_selection_listener()

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

    def closeEvent(self, event) -> None:
        """Clean up Maya callbacks + signal connections when the widget closes."""
        self._stop_selection_listener()
        try:
            self.morph_tree_widget._cleanup_callbacks()
        except Exception:
            pass
        try:
            if self.ctx is not None:
                self.ctx.modelChanged.disconnect(self._on_model_changed)
        except (RuntimeError, TypeError):
            pass
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

    def _get_model_roots_for_import(self) -> list[str]:
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
                            log.warning(
                                "Validation issue - Severity: %s, Category: %s, Message: %s",
                                issue.severity.name,
                                issue.category.name,
                                issue.message,
                            )

                    maya_pmx_data = build_pmx_scene(pmx_data)

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
        roots = self._get_model_roots_for_import()
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
        roots = self._get_model_roots_for_import()
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
        roots = self._get_model_roots_for_import()
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
