cmake_minimum_required(VERSION 3.21)
project(mayammd-native LANGUAGES CXX)

# ── C++ standard ─────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Maya version ────────────────────────────────────────────────────────
# Set via CMakePresets.json (see MAYA_VERSION cache variable).
if(NOT DEFINED MAYA_VERSION)
    set(MAYA_VERSION "2026" CACHE STRING "Maya version to target")
endif()
message(STATUS "Targeting Maya ${MAYA_VERSION}")

# ── Python (for release packaging scripts) ──────────────────────────────
# Any Python ≥ 3.10 works — source .py files are Maya-version-agnostic.
set(Python_FIND_STRATEGY LOCATION)
find_package(Python 3.10 QUIET COMPONENTS Interpreter)
if(Python_FOUND)
    message(STATUS "Python ${Python_VERSION} (for packaging scripts)")
endif()

# ── Resolve project version from git tags ────────────────────────────
# Git is the single source of truth (replaces hatch-vcs).
set(MMD_PROJECT_VERSION "0.1.0")  # last-resort fallback
include("${CMAKE_SOURCE_DIR}/cmake/GitVersion.cmake")
get_git_version(MMD_PROJECT_VERSION)

if(NOT MMD_PROJECT_VERSION)
    # No git available — try reading the last generated _version.py
    set(_version_py "${CMAKE_SOURCE_DIR}/mmd/_version.py")
    if(EXISTS "${_version_py}")
        file(STRINGS "${_version_py}" _version_line REGEX "^__version__[ \t]*=")
        if(_version_line)
            string(REGEX REPLACE ".*\"([^\"]+)\".*" "\\1" MMD_PROJECT_VERSION "${_version_line}")
        endif()
    endif()
endif()

if(NOT MMD_PROJECT_VERSION)
    set(MMD_PROJECT_VERSION "0.1.0")
endif()
message(STATUS "MayaMMD version: ${MMD_PROJECT_VERSION}")

# ── Generate version files from templates ────────────────────────────
# Python: mmd/_version.py  (read by mmd.plugin at import time)
configure_file(
    "${CMAKE_SOURCE_DIR}/cmake/_version.py.in"
    "${CMAKE_SOURCE_DIR}/mmd/_version.py"
    @ONLY
)
# C++: version.hpp  (included by MayaMMD.cpp)
configure_file(
    "${CMAKE_SOURCE_DIR}/cmake/version.hpp.in"
    "${CMAKE_BINARY_DIR}/generated/version.hpp"
    @ONLY
)

# ── Ensure SDK via custom target ────────────────────────────────────────
# (SDK is auto-ensured before CMake runs — this target is for reference)
add_custom_target(ensure-sdk
    COMMAND ${CMAKE_COMMAND} -E echo "  SDK was ensured during configure"
)

# ── Options ──────────────────────────────────────────────────────────────
option(BUILD_NATIVE_MODULE "Build the C++ native .mll plugin (MayaMMD.mll)" ON)
option(BUILD_TESTS "Build C++ unit tests (requires Catch2, no Maya SDK)" OFF)
option(BUILD_PACKAGE "Build release package (assembles Python source + zip, requires Maya SDK + Python)" OFF)
option(BUILD_DEV_INSTALL "Generate dev-install layout with .mod file for local development" ON)

# Enable CTest (used by both Catch2 unit tests and Maya integration tests)
enable_testing()

# ── SDK setup ────────────────────────────────────────────────────────────
# Must run BEFORE find_package(Maya) so FindMaya.cmake can discover it.
#
# The Maya SDK is proprietary Autodesk software and is NOT included in this repo.
# CMake will auto-download it via scripts/ci/strip_maya_sdk.py ensure.
#
# Options:
#   cmake --preset maya2026-release                              # auto-download from Autodesk
#   cmake --preset maya2026-release -DSDK_DIR=C:/Maya2026/devkitBase  # use existing SDK
#
# For faster downloads, set MAYAMMD_SDK_CACHE_URL to a pre-stripped SDK cache:
#   $env:MAYAMMD_SDK_CACHE_URL = "https://github.com/.../releases/download/latest"
#
# Get the SDK from Autodesk:
#   https://www.autodesk.com/developer-network/platform-technologies/maya
if(NOT DEFINED SDK_DIR)
    set(SDK_CACHE_DIR "${CMAKE_SOURCE_DIR}/out/.sdk")
    set(SDK_DIR "${SDK_CACHE_DIR}/sdk-maya${MAYA_VERSION}")

    if(NOT EXISTS "${SDK_DIR}/include/maya/MFnPlugin.h")
        find_package(Python 3.10 QUIET COMPONENTS Interpreter)
        if(Python_FOUND)
            message(STATUS "Maya ${MAYA_VERSION} SDK not found — running ensure...")
            execute_process(COMMAND ${Python_EXECUTABLE}
                "${CMAKE_SOURCE_DIR}/scripts/ci/strip_maya_sdk.py" ensure
                --version ${MAYA_VERSION}
                RESULT_VARIABLE _ensure_rc
                WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
            )
            if(NOT _ensure_rc EQUAL 0)
                message(FATAL_ERROR
                    "Failed to download Maya ${MAYA_VERSION} SDK.\n\n"
                    "  Provide it manually:\n"
                    "    -DSDK_DIR=/path/to/maya-sdk\n\n"
                    "  Download from Autodesk:\n"
                    "    https://www.autodesk.com/developer-network/platform-technologies/maya\n"
                    "  See docs/CPPDevelopment.md for detailed setup instructions."
                )
            endif()
        else()
            message(FATAL_ERROR
                "Maya ${MAYA_VERSION} SDK not found and Python not available for auto-download.\n\n"
                "  Provide the SDK manually:\n"
                "    -DSDK_DIR=/path/to/maya-sdk\n\n"
                "  Or install Python ≥ 3.10 and re-run cmake for auto-download.\n"
                "  See docs/CPPDevelopment.md for detailed setup instructions."
            )
        endif()
    else()
        message(STATUS "Maya ${MAYA_VERSION} SDK found at: ${SDK_DIR}")
    endif()
else()
    message(STATUS "Using user-provided Maya SDK: ${SDK_DIR}")
    if(NOT EXISTS "${SDK_DIR}/include/maya/MFnPlugin.h")
        message(FATAL_ERROR "MFnPlugin.h not found in SDK_DIR=${SDK_DIR}. "
            "Download the Maya SDK from Autodesk: "
            "https://www.autodesk.com/developer-network/platform-technologies/maya")
    endif()
endif()

# ── Find Maya SDK ────────────────────────────────────────────────────────
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
find_package(Maya REQUIRED)
message(STATUS "Maya ${Maya_VERSION} SDK: ${Maya_INCLUDE_DIR} (libs: ${Maya_LIBRARY_DIR})")

# ── Bullet (physics engine) — resolved ONCE at the top level ────────────
# Used by the Maya-free core (mmd_core) and the plugin.  Provided by vcpkg
# (see vcpkg.json, package `bullet3` 3.25) with FLOAT precision — do NOT
# enable vcpkg's `double-precision` feature, btScalar must stay float to
# match the consumers in mmd/core/simulation.cpp.
if(NOT CMAKE_TOOLCHAIN_FILE)
    message(FATAL_ERROR
        "No CMake toolchain configured.\n\n"
        "  Use a CMake preset to activate the vcpkg toolchain:\n"
        "    cmake --preset maya2026-release\n\n"
        "  Set VCPKG_ROOT to a bootstrapped vcpkg first — see docs/CPPDevelopment.md."
    )
endif()
find_package(Bullet CONFIG REQUIRED)

# =========================================================================
# Native .mll plugin — single entry point for MayaMMD
# =========================================================================
if(BUILD_NATIVE_MODULE AND Maya_FOUND)
    message(STATUS "Maya ${Maya_VERSION} SDK: ${Maya_INCLUDE_DIR}")

    # =====================================================================
    # MayaMMD.mll — SINGLE Maya plugin entry point
    #
    # This is the MAIN plugin for MayaMMD.  Its initializePlugin:
    #   1. Registers native C++ nodes via MFnPlugin::registerNode
    #   2. Calls Python (via MGlobal::executePythonCommand) to register
    #      Python-based nodes and set up the UI — all on the same handle.
    #
    # Source:
    #   - mmd/MayaMMD.cpp               → entry point (initializePlugin/uninitializePlugin)
    #   - mmd/maya/nodes/ccd_ik_solver_node.h/cpp  → C++ CCD IK solver node
    # =====================================================================
    add_library(MayaMMD SHARED
        mmd/MayaMMD.cpp
        mmd/maya/nodes/ccd_ik_solver_node.cpp
    )
    target_include_directories(MayaMMD PRIVATE
        "${CMAKE_BINARY_DIR}/generated"
    )
    target_link_libraries(MayaMMD PRIVATE Maya::Maya)

    # Maya plugin extension — .mll (Windows), .so (Linux), .bundle (macOS)
    if(WIN32)
        set(_MAYA_PLUGIN_SUFFIX ".mll")
    elseif(APPLE)
        set(_MAYA_PLUGIN_SUFFIX ".bundle")
    else()
        set(_MAYA_PLUGIN_SUFFIX ".so")
    endif()
    set_target_properties(MayaMMD PROPERTIES
        PREFIX ""
        SUFFIX "${_MAYA_PLUGIN_SUFFIX}"
    )

    # Normalize path separators for embedding in C++ string literal
    string(REPLACE "\\" "/" _normalized_root "${CMAKE_SOURCE_DIR}")

    # PROJECT_ROOT_DIR is used ONLY for sys.path.insert() before
    # importing mmd.plugin — it is NOT used for plugin discovery.
    target_compile_definitions(MayaMMD PRIVATE
        PROJECT_ROOT_DIR="${_normalized_root}"
    )

    # Platform-specific extras (beyond what Maya::Maya provides)
    if(WIN32)
        target_compile_definitions(MayaMMD PRIVATE
            $<$<CONFIG:Debug>:_DEBUG>
            _HAS_ITERATOR_DEBUGGING=0
        )
        target_compile_options(MayaMMD PRIVATE /Zi /wd4996)
    elseif(APPLE)
        # Maya::Maya already provides _BOOL, MAC_PLUGIN
    else()
        target_compile_options(MayaMMD PRIVATE
            -Wextra -Wno-unused-parameter -Wno-unused-variable
            -Wno-deprecated-declarations
            -fpermissive
        )
    endif()

    # ── Dev-install: post-build copy + .mod generation ──
    # The .mll is copied to <prefix>/plug-ins/ after every build so that
    # Maya and tests always pick up the latest version — no separate
    # cmake --install step required during development.
    if(BUILD_DEV_INSTALL AND NOT BUILD_PACKAGE)
        # Create the plug-ins directory at configure time
        file(MAKE_DIRECTORY "${CMAKE_INSTALL_PREFIX}/plug-ins")

        # Post-build: copy .mll → <prefix>/plug-ins/
        add_custom_command(TARGET MayaMMD POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "$<TARGET_FILE:MayaMMD>"
                "${CMAKE_INSTALL_PREFIX}/plug-ins/"
            COMMENT "Deploying MayaMMD.mll → ${CMAKE_INSTALL_PREFIX}/plug-ins/"
        )

        # Generate MayaMMD.mod directly into the install prefix.
        # Module root = CMAKE_INSTALL_PREFIX, so Maya finds plug-ins/
        # automatically — no MAYA_PLUG_IN_PATH or PYTHONPATH override needed.
        configure_file(
            "${CMAKE_SOURCE_DIR}/cmake/MayaMMD.mod.in"
            "${CMAKE_INSTALL_PREFIX}/MayaMMD.mod"
            @ONLY
        )

        # Also generate into build dir (for cmake --install / packaging)
        configure_file(
            "${CMAKE_SOURCE_DIR}/cmake/MayaMMD.mod.in"
            "${CMAKE_BINARY_DIR}/MayaMMD.mod"
            @ONLY
        )
        install(FILES "${CMAKE_BINARY_DIR}/MayaMMD.mod"
            DESTINATION "."
        )

        # Keep install(TARGETS) for cmake --install / packaging support
        install(TARGETS MayaMMD
            RUNTIME DESTINATION "plug-ins"
            LIBRARY DESTINATION "plug-ins"
        )

        message(STATUS "Dev-install prefix: ${CMAKE_INSTALL_PREFIX}")
    endif()

    message(STATUS "Target: MayaMMD → .mll (single entry point)")

elseif(BUILD_NATIVE_MODULE AND NOT Maya_FOUND)
    message(WARNING
        "Maya SDK not found — skipping native module build.\n"
        "  Run: python scripts/ci/strip_maya_sdk.py ensure")
endif()

# =========================================================================
# Tests — Catch2 C++ unit tests & Maya integration tests via CTest
# =========================================================================
add_subdirectory(tests)

# =========================================================================
# Package build — assemble release directory (opt-in via -DBUILD_PACKAGE=ON)
# =========================================================================
if(BUILD_PACKAGE)
    # Packaging requires Python for the build script.
    if(NOT Python_EXECUTABLE)
        message(FATAL_ERROR "Python ≥ 3.10 not found — required for packaging")
    endif()

    # ── Detect OS display name ─────────────────────────────────────────
    if(WIN32)
        set(OUTPUT_OS "Windows")
    elseif(APPLE)
        set(OUTPUT_OS "macOS")
    else()
        set(OUTPUT_OS "Linux")
    endif()

    set(RELEASE_OUTPUT_DIR "${CMAKE_INSTALL_PREFIX}/MayaMMD-Maya${MAYA_VERSION}-${OUTPUT_OS}/MayaMMD")

    # ── Python source assembly ─────────────────────────────────────────
    # Copy mmd/ source tree into the release staging directory.
    # Open-source: source .py files are distributed directly.
    set(PYTHON_STAGING "${CMAKE_BINARY_DIR}/python-staging/mmd")
    add_custom_command(
        OUTPUT "${PYTHON_STAGING}/_version.py"
        COMMAND ${Python_EXECUTABLE}
            "${CMAKE_SOURCE_DIR}/scripts/release/build_release.py"
            --output-dir "${CMAKE_BINARY_DIR}/python-staging"
            --version "${MMD_PROJECT_VERSION}"
        COMMENT "Assembling Python source for Maya ${MAYA_VERSION}..."
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    )
    add_custom_target(python_stage ALL
        DEPENDS "${PYTHON_STAGING}/_version.py"
    )

    # ── Install rules ──────────────────────────────────────────────────
    # MayaMMD.mll → plug-ins/ (only if target was built)
    if(TARGET MayaMMD)
        install(TARGETS MayaMMD
            RUNTIME DESTINATION "${RELEASE_OUTPUT_DIR}/plug-ins"
            LIBRARY DESTINATION "${RELEASE_OUTPUT_DIR}/plug-ins"
        )
    endif()

    # Python mmd/ package → plug-ins/mmd/
    install(DIRECTORY "${CMAKE_BINARY_DIR}/python-staging/mmd/"
        DESTINATION "${RELEASE_OUTPUT_DIR}/plug-ins/mmd"
        USE_SOURCE_PERMISSIONS
    )

    # Icons (e.g., the Maya shelf button icon) ship inside mmd/ via
    # build_release.py and are installed together with the mmd package above.

    # Documentation files
    set(_docs README.md LICENSE.txt CHANGELOG.md)
    foreach(_doc ${_docs})
        if(EXISTS "${CMAKE_SOURCE_DIR}/${_doc}")
            install(FILES "${CMAKE_SOURCE_DIR}/${_doc}"
                DESTINATION "${RELEASE_OUTPUT_DIR}")
        endif()
    endforeach()

    # ── Generate MayaMMD.mod ──────────────────────────────────────────
    # Reuse the version already resolved at configure time.
    set(_pkg_version "${MMD_PROJECT_VERSION}")

    set(MOD_CONTENT "+ MayaMMD ${_pkg_version} MayaMMD\n")
    file(WRITE "${CMAKE_BINARY_DIR}/MayaMMD.mod" "${MOD_CONTENT}")
    install(FILES "${CMAKE_BINARY_DIR}/MayaMMD.mod"
        DESTINATION "${CMAKE_INSTALL_PREFIX}/MayaMMD-Maya${MAYA_VERSION}-${OUTPUT_OS}")

    message(STATUS "Release output: ${CMAKE_INSTALL_PREFIX}/MayaMMD-Maya${MAYA_VERSION}-${OUTPUT_OS}/")
endif()
