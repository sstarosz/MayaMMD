"""
PMX validation functions.
"""

import os
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Optional

from mmd.core.data_types import PmxModel, WeightType


class IssueSeverity(Enum):
    """Severity levels for validation issues."""

    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"
    CRITICAL = "CRITICAL"


class IssueCategory(Enum):
    """Categories for grouping validation issues."""

    STRUCTURE = "Structure"
    TEXTURES = "Textures"
    MATERIALS = "Materials"
    BONES = "Bones"
    MORPHS = "Morphs"
    PHYSICS = "Physics"
    PERFORMANCE = "Performance"
    NOT_SUPPORTED = "Not Supported"


@dataclass
class ValidationIssue:
    """Represents a single validation issue with optional fix."""

    severity: IssueSeverity
    category: IssueCategory
    message: str
    details: Optional[str] = None
    autofix: Optional[Callable[[PmxModel], PmxModel]] = None
    proposed_fix: Optional[str] = None
    affected_items: list[str] = field(default_factory=list[str])

    def __str__(self) -> str:
        result = f"[{self.severity.value}] {self.category.value}: {self.message}"
        if self.details:
            result += f"\n  Details: {self.details}"
        if self.proposed_fix:
            result += f"\n  Proposed fix: {self.proposed_fix}"
        if self.affected_items:
            result += f"\n  Affected items: {', '.join(self.affected_items[:5])}"
            if len(self.affected_items) > 5:
                result += f" ... and {len(self.affected_items) - 5} more"
        return result


@dataclass
class ValidationResult:
    """Contains all validation results for a PMX model."""

    issues: list[ValidationIssue] = field(default_factory=list[ValidationIssue])

    def add_issue(self, issue: ValidationIssue):
        """Add a validation issue."""
        self.issues.append(issue)

    def has_errors(self) -> bool:
        """Check if there are any errors or critical issues."""
        return any(
            issue.severity in (IssueSeverity.ERROR, IssueSeverity.CRITICAL)
            for issue in self.issues
        )

    def has_warnings(self) -> bool:
        """Check if there are any warnings."""
        return any(issue.severity == IssueSeverity.WARNING for issue in self.issues)

    def get_by_severity(self, severity: IssueSeverity) -> list[ValidationIssue]:
        """Get all issues of a specific severity."""
        return [issue for issue in self.issues if issue.severity == severity]

    def get_by_category(self, category: IssueCategory) -> list[ValidationIssue]:
        """Get all issues of a specific category."""
        return [issue for issue in self.issues if issue.category == category]

    def get_fixable_issues(self) -> list[ValidationIssue]:
        """Get all issues that have an autofix available."""
        return [issue for issue in self.issues if issue.autofix is not None]

    def get_summary(self) -> str:
        """Get a summary of all issues."""
        critical = len(self.get_by_severity(IssueSeverity.CRITICAL))
        errors = len(self.get_by_severity(IssueSeverity.ERROR))
        warnings = len(self.get_by_severity(IssueSeverity.WARNING))
        info = len(self.get_by_severity(IssueSeverity.INFO))

        fixable = len(self.get_fixable_issues())

        summary = "Validation Summary: "
        parts: list[str] = []
        if critical > 0:
            parts.append(f"{critical} critical")
        if errors > 0:
            parts.append(f"{errors} errors")
        if warnings > 0:
            parts.append(f"{warnings} warnings")
        if info > 0:
            parts.append(f"{info} info")

        summary += ", ".join(parts) if parts else "No issues"

        if fixable > 0:
            summary += f" ({fixable} fixable)"

        return summary

    def apply_autofixes(self, pmx_model: PmxModel) -> tuple[PmxModel, int]:
        """
        Apply all available autofixes to the model.
        Returns the fixed model and the number of fixes applied.
        """
        fixed_model = pmx_model
        fixes_applied = 0

        for issue in self.get_fixable_issues():
            if issue.autofix:
                try:
                    fixed_model = issue.autofix(fixed_model)
                    fixes_applied += 1
                except Exception as e:
                    # If autofix fails, create a new issue
                    self.add_issue(
                        ValidationIssue(
                            severity=IssueSeverity.ERROR,
                            category=issue.category,
                            message=f"Autofix failed for: {issue.message}",
                            details=str(e),
                        )
                    )

        return fixed_model, fixes_applied


def validate_pmx_model(pmx_model: PmxModel) -> ValidationResult:
    """
    Detect common issues in the PMX model data.
    Collect validation issues with potential fixes.
    """
    result = ValidationResult()

    # Structure validation
    _validate_structure(pmx_model, result)

    # Vertex validation (e.g. check for invalid bone weights, missing UVs, etc.)
    _validate_vertices(pmx_model, result)

    # Texture validation
    _validate_textures(pmx_model, result)

    # Material validation
    _validate_materials(pmx_model, result)

    # Bone validation
    _validate_bones(pmx_model, result)

    # Morph validation
    _validate_morphs(pmx_model, result)

    # Performance checks
    _validate_performance(pmx_model, result)

    return result


def _validate_structure(pmx_model: PmxModel, result: ValidationResult):
    """Validate basic model structure."""
    if not pmx_model.vertices:
        result.add_issue(
            ValidationIssue(
                severity=IssueSeverity.CRITICAL,
                category=IssueCategory.STRUCTURE,
                message="Model has no vertices",
                details="A valid PMX model must have at least one vertex",
            )
        )

    if not pmx_model.indices:
        result.add_issue(
            ValidationIssue(
                severity=IssueSeverity.CRITICAL,
                category=IssueCategory.STRUCTURE,
                message="Model has no indices",
                details="A valid PMX model must have at least one index",
            )
        )

    if len(pmx_model.indices) % 3 != 0:
        result.add_issue(
            ValidationIssue(
                severity=IssueSeverity.ERROR,
                category=IssueCategory.STRUCTURE,
                message="Index count is not a multiple of 3",
                details=f"Found {len(pmx_model.indices)} indices",
                proposed_fix="Remove extra indices or add missing ones to complete triangles",
            )
        )


def _validate_vertices(pmx_model: PmxModel, result: ValidationResult):
    """Validate vertex data, including bone weights and UVs."""

    # Check for not supported vertex weights types (e.g. SDEF, QDEF).
    # Collect (index, vertex) in a single O(V) pass — a later membership test
    # against a plain list of vertices would be O(V x U) and use PmxVertex.__eq__,
    # which is extremely slow for large models (e.g. tens of thousands of SDEF
    # vertices x hundreds of thousands of total vertices = hours).
    unsupported_vertices = [
        (i, vertex)
        for i, vertex in enumerate(pmx_model.vertices)
        if vertex.weight_type
        not in (WeightType.BDEF1, WeightType.BDEF2, WeightType.BDEF4)
    ]

    if unsupported_vertices:
        result.add_issue(
            ValidationIssue(
                severity=IssueSeverity.WARNING,
                category=IssueCategory.NOT_SUPPORTED,
                message=f"{len(unsupported_vertices)} vertices use unsupported weight types (SDEF/QDEF)",
                details="These weight types may not import correctly into all tools",
                proposed_fix="Consider converting these vertices to use BDEF1, BDEF2, or BDEF4 weights for better compatibility",
                affected_items=[f"Vertex {i}" for i, _ in unsupported_vertices],
            )
        )


def _validate_textures(pmx_model: PmxModel, result: ValidationResult):
    """Validate textures and their references."""
    missing_textures: list[str] = []
    for texture_path in pmx_model.textures_paths:
        # Skip empty texture paths (some models use empty strings for unused texture slots)
        if not texture_path:
            continue

        # Check if texture is a file. (Some models might reference directories)
        if os.path.isdir(os.path.join(pmx_model.absolute_path, texture_path)):
            continue

        full_path = os.path.join(pmx_model.absolute_path, texture_path)
        if not os.path.exists(full_path):
            missing_textures.append(texture_path)

    if missing_textures:
        result.add_issue(
            ValidationIssue(
                severity=IssueSeverity.WARNING,
                category=IssueCategory.TEXTURES,
                message=f"{len(missing_textures)} texture(s) not found",
                affected_items=missing_textures,
                proposed_fix="Ensure texture files are in the correct directory relative to the model",
            )
        )

    # Check for non-ASCII texture paths
    non_ascii_textures = [
        path for path in pmx_model.textures_paths if any(ord(c) > 127 for c in path)
    ]

    if non_ascii_textures:
        result.add_issue(
            ValidationIssue(
                severity=IssueSeverity.WARNING,
                category=IssueCategory.TEXTURES,
                message=f"{len(non_ascii_textures)} texture(s) have non-ASCII characters in their paths",
                affected_items=non_ascii_textures,
                proposed_fix="Consider renaming texture files to use only ASCII characters for better compatibility",
            )
        )


def _validate_bones(pmx_model: PmxModel, result: ValidationResult):
    # Plugin v1.0 only supports basic bone hierarchy, position, and parent index
    for idx, bone in enumerate(pmx_model.bones):
        unsupported: list[str] = []
        # External parent not supported
        if bone.externalParent is not None:
            unsupported.append("external parent")
        # Flags: warn for unsupported PMXBoneFlagBits
        unsupported_flags: list[str] = []
        for flag in bone.flags.__class__:
            if bone.flags & flag:
                if flag in [
                    bone.flags.__class__.ROTATABLE,
                    bone.flags.__class__.TRANSLATABLE,
                    bone.flags.__class__.VISIBLE,
                    bone.flags.__class__.ENABLED,
                    bone.flags.__class__.IK,
                    bone.flags.__class__.FIXED_AXIS,
                    bone.flags.__class__.LOCAL_COORDINATE,
                    bone.flags.__class__.INHERIT_ROTATION,
                    bone.flags.__class__.INHERIT_TRANSLATION,
                    bone.flags.__class__.INDEXED_TAIL_POSITION,
                ]:
                    continue  # Supported by default
                unsupported_flags.append(flag.name or str(flag))
        if unsupported_flags:
            unsupported.append(f"flags ({', '.join(unsupported_flags)})")
        # Only warn if unsupported features are present
        if unsupported:
            result.add_issue(
                ValidationIssue(
                    severity=IssueSeverity.WARNING,
                    category=IssueCategory.BONES,
                    message=f"Bone '{bone.nameLocal or bone.nameUniversal or f'Bone {idx}'}' uses unsupported features in plugin v1.0: {', '.join(unsupported)}",
                    details="Only basic bone hierarchy, position, and parent index are supported in this version.",
                    affected_items=[
                        bone.nameLocal or bone.nameUniversal or f"Bone {idx}"
                    ],
                )
            )

    # Check for unsupported bone features (e.g. invisible bones, non-standard bone types)


def _validate_materials(pmx_model: PmxModel, result: ValidationResult):
    """
    List of potential material issues to check for:
    - Unsupported material properties (e.g. specular, ambient, edge, sphere texture, toon, flags)
    - Materials referencing missing textures
    - Materials with invalid texture indices (e.g. out of bounds)
    - Materials with indices to texture that is actually a directory instead of a file
    """

    # Plugin v1.0 only supports basic diffuse color and texture
    for idx, mat in enumerate(pmx_model.materials):
        unsupported: list[str] = []
        # Ambient color not supported
        if mat.ambient_color != mat.ambient_color.__class__(0.0, 0.0, 0.0):
            unsupported.append("ambient")
        # Edge color/size not supported
        if mat.edge_size != 0.0 or mat.edge_color != mat.edge_color.__class__(
            0.0, 0.0, 0.0, 1.0
        ):
            unsupported.append("edge")
        # Sphere texture not supported
        if mat.sphere_texture_index != -1:
            unsupported.append("sphere texture")
        # Toon not supported
        if mat.toon_flag != 0 or mat.toon_value != 0:
            unsupported.append("toon")
        # Material flags: Only NO_CULL is supported by default in Maya, others are not
        unsupported_flags: list[str] = []
        for flag in mat.draw_flag.__class__:
            if mat.has_flag(flag):
                # NO_CULL is supported by default (back-face culling off)
                # GROUND_SHADOW, DRAW_SHADOW, RECEIVE_SHADOW are also supported by default in Maya,
                # so we can ignore those flags as well. ()
                if (
                    flag == mat.draw_flag.__class__.NO_CULL
                    or flag == mat.draw_flag.__class__.GROUND_SHADOW
                    or flag == mat.draw_flag.__class__.DRAW_SHADOW
                    or flag == mat.draw_flag.__class__.RECEIVE_SHADOW
                ):
                    continue
                unsupported_flags.append(flag.name or str(flag))
        if unsupported_flags:
            unsupported.append(f"material flags ({', '.join(unsupported_flags)})")
        # Metadata not supported
        if mat.meta_data:
            unsupported.append("meta data")
        # Only warn if unsupported features are present
        if unsupported:
            result.add_issue(
                ValidationIssue(
                    severity=IssueSeverity.WARNING,
                    category=IssueCategory.MATERIALS,
                    message=f"Material '{mat.name_local or mat.name_universal or f'Material {idx}'}' uses unsupported features in plugin v1.0: {', '.join(unsupported)}",
                    details="Only diffuse color and texture are supported in this version.",
                    affected_items=[
                        mat.name_local or mat.name_universal or f"Material {idx}"
                    ],
                )
            )

    # Check for textures that are referenced by materials, but missing from the texture list or have invalid indices
    for idx, mat in enumerate(pmx_model.materials):
        if mat.texture_index >= 0:
            if mat.texture_index >= len(pmx_model.textures_paths):
                result.add_issue(
                    ValidationIssue(
                        severity=IssueSeverity.ERROR,
                        category=IssueCategory.MATERIALS,
                        message=f"Material '{mat.name_local or mat.name_universal or f'Material {idx}'}' references texture index {mat.texture_index} which is out of bounds",
                        details=f"Model has {len(pmx_model.textures_paths)} textures defined",
                        proposed_fix="Update the material to reference a valid texture index or set it to -1 for no texture",
                        affected_items=[
                            mat.name_local or mat.name_universal or f"Material {idx}"
                        ],
                    )
                )
            else:
                # Check if the referenced texture is missing (empty path or file not found)
                tex_path = pmx_model.textures_paths[mat.texture_index]
                full_tex_path = os.path.join(pmx_model.absolute_path, tex_path)
                if not tex_path or not os.path.isfile(full_tex_path):
                    result.add_issue(
                        ValidationIssue(
                            severity=IssueSeverity.WARNING,
                            category=IssueCategory.MATERIALS,
                            message=f"Material '{mat.name_local or mat.name_universal or f'Material {idx}'}' references texture '{tex_path}' which is missing",
                            details="Ensure the texture file exists at the specified path relative to the model",
                            proposed_fix="Provide the missing texture file or update the material to reference an existing texture",
                            affected_items=[
                                mat.name_local
                                or mat.name_universal
                                or f"Material {idx}"
                            ],
                        )
                    )
                # Check is texture_path referenced to directory instead of file
                elif os.path.isdir(full_tex_path):
                    result.add_issue(
                        ValidationIssue(
                            severity=IssueSeverity.WARNING,
                            category=IssueCategory.MATERIALS,
                            message=f"Material '{mat.name_local or mat.name_universal or f'Material {idx}'}' references texture '{tex_path}' which is a directory",
                            details="Texture paths should reference image files, not directories",
                            proposed_fix="Update the material to reference a valid texture file instead of a directory",
                            affected_items=[
                                mat.name_local
                                or mat.name_universal
                                or f"Material {idx}"
                            ],
                        )
                    )


def _validate_morphs(pmx_model: PmxModel, result: ValidationResult):
    # Plugin v1.0 only supports basic vertex morphs
    for idx, morph in enumerate(pmx_model.morphs):
        unsupported: list[str] = []
        # Only vertex morphs and bone morphs are supported, warn if other types are present
        # supported, warn if other types are present
        if morph.morph_type not in (
            morph.morph_type.__class__.VERTEX,
            morph.morph_type.__class__.BONE,
        ):
            unsupported.append(f"type {morph.morph_type.name}")
        if unsupported:
            result.add_issue(
                ValidationIssue(
                    severity=IssueSeverity.WARNING,
                    category=IssueCategory.MORPHS,
                    message=f"Morph '{morph.name_local or morph.name_universal or f'Morph {idx}'}' uses unsupported features in plugin v1.0: {', '.join(unsupported)}",
                    details="Only basic vertex morphs are supported in this version.",
                    affected_items=[
                        morph.name_local or morph.name_universal or f"Morph {idx}"
                    ],
                )
            )


def _validate_performance(pmx_model: PmxModel, result: ValidationResult):
    pass
