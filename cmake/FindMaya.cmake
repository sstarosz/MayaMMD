# FindMaya.cmake — Locate a Maya SDK installation for plugin development.
#
# This module searches for Maya SDK headers and libraries in the following
# order (see below).  On success it sets:
#
#   Maya_FOUND          - True if Maya SDK was found
#   Maya_INCLUDE_DIR    - Path to Maya headers (e.g. .../include)
#   Maya_LIBRARY_DIR    - Path to Maya libraries (e.g. .../lib)
#   Maya_VERSION        - Version string, e.g. "2026"
#   Maya_LIBRARIES      - List of Maya libraries to link against
#   Maya_PLUGIN_SUFFIX  - Platform-specific plugin extension (.mll/.so/.bundle)
#
# Usage in CMakeLists.txt:
#   find_package(Maya REQUIRED)
#   target_include_directories(myplugin PRIVATE ${Maya_INCLUDE_DIR})
#   target_link_directories(myplugin PRIVATE ${Maya_LIBRARY_DIR})
#   target_link_libraries(myplugin ${Maya_LIBRARIES})
#
# ============================================================================
# Search order:
#   1. MAYA_LOCATION  environment variable (set to Maya install root)
#   2. MAYA_LOCATION  CMake variable (passed as -DMAYA_LOCATION=...)
#   3. Windows:  Registry  HKLM\SOFTWARE\Autodesk\Maya\<ver>\Setup\InstallPath
#   4. Common install paths per platform

# -----------------------------------------------------------------------
# Helper: test whether a candidate directory is a valid Maya SDK root
# -----------------------------------------------------------------------
function(_maya_check_root path out_var)
    set(valid FALSE)
    if(EXISTS "${path}/include/maya/MFnPlugin.h" AND
       EXISTS "${path}/lib/Foundation.lib" OR
       EXISTS "${path}/lib/libFoundation.so" OR
       EXISTS "${path}/lib/libFoundation.dylib")
        set(valid TRUE)
    endif()
    set(${out_var} ${valid} PARENT_SCOPE)
endfunction()

# -----------------------------------------------------------------------
# Candidate paths
# -----------------------------------------------------------------------
set(_maya_candidates "")

# 1. Environment variable
if(DEFINED ENV{MAYA_LOCATION})
    list(APPEND _maya_candidates "$ENV{MAYA_LOCATION}")
endif()

# 2. CMake variable
if(DEFINED MAYA_LOCATION)
    list(APPEND _maya_candidates "${MAYA_LOCATION}")
endif()

# 3. out/.sdk/ — auto-downloaded stripped SDKs (preferred cache)
# Prefer MAYA_VERSION first (a Maya-2027 build must never resolve the 2026
# SDK just because it was cached earlier), then scan the remaining cached
# SDKs newest-first as a fallback.  file(GLOB without wildcards does NOT
# match directories, so use foreach+exists.
if(DEFINED MAYA_VERSION AND NOT MAYA_VERSION STREQUAL "")
    set(_sdk_dir "${CMAKE_SOURCE_DIR}/out/.sdk/sdk-maya${MAYA_VERSION}")
    if(EXISTS "${_sdk_dir}/include/maya/MFnPlugin.h")
        list(APPEND _maya_candidates "${_sdk_dir}")
    endif()
endif()
foreach(_sdk_ver 2027 2026 2025 2024)
    if(DEFINED MAYA_VERSION AND _sdk_ver STREQUAL "${MAYA_VERSION}")
        continue()  # Already added above
    endif()
    set(_sdk_dir "${CMAKE_SOURCE_DIR}/out/.sdk/sdk-maya${_sdk_ver}")
    if(EXISTS "${_sdk_dir}/include/maya/MFnPlugin.h")
        list(APPEND _maya_candidates "${_sdk_dir}")
    endif()
endforeach()

# Also check out/.sdk/ (legacy) and thirdy-party/ (legacy) and raw DevKit directories
foreach(_legacy_pattern
    "${CMAKE_SOURCE_DIR}/out/.sdk/sdk-maya*"
    "${CMAKE_SOURCE_DIR}/thirdy-party/sdk-maya*"
    "${CMAKE_SOURCE_DIR}/thirdy-party/Autodesk_Maya_*/devkitBase"
)
    file(GLOB _legacy_match ${_legacy_pattern})
    list(APPEND _maya_candidates ${_legacy_match})
endforeach()

# 4. Windows registry — prefer MAYA_VERSION if set
if(WIN32)
    # If MAYA_VERSION is specified, try it first.
    if(DEFINED MAYA_VERSION AND NOT MAYA_VERSION STREQUAL "")
        get_filename_component(
            _maya_reg_path
            "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Autodesk\\Maya\\${MAYA_VERSION}\\Setup\\InstallPath]"
            ABSOLUTE
        )
        if(_maya_reg_path AND NOT _maya_reg_path STREQUAL "/registry")
            list(APPEND _maya_candidates "${_maya_reg_path}")
        endif()
    endif()
    # Then search all installed versions (newest first) as fallback.
    foreach(_ver RANGE 2027 2024 -1)
        if(DEFINED MAYA_VERSION AND _ver STREQUAL "${MAYA_VERSION}")
            continue()  # Already added above
        endif()
        get_filename_component(
            _maya_reg_path
            "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Autodesk\\Maya\\${_ver}\\Setup\\InstallPath]"
            ABSOLUTE
        )
        if(_maya_reg_path AND NOT _maya_reg_path STREQUAL "/registry")
            list(APPEND _maya_candidates "${_maya_reg_path}")
        endif()
    endforeach()
endif()

# 5. Common install paths — prefer MAYA_VERSION if set
if(WIN32)
    if(DEFINED MAYA_VERSION AND NOT MAYA_VERSION STREQUAL "")
        list(APPEND _maya_candidates "C:/Program Files/Autodesk/Maya${MAYA_VERSION}")
    endif()
    foreach(_ver 2027 2026 2025 2024)
        if(DEFINED MAYA_VERSION AND _ver STREQUAL "${MAYA_VERSION}")
            continue()  # Already added above
        endif()
        list(APPEND _maya_candidates "C:/Program Files/Autodesk/Maya${_ver}")
    endforeach()
elseif(APPLE)
    if(DEFINED MAYA_VERSION AND NOT MAYA_VERSION STREQUAL "")
        list(APPEND _maya_candidates "/Applications/Autodesk/Maya${MAYA_VERSION}/Maya.app/Contents")
    endif()
    foreach(_ver 2027 2026 2025 2024)
        if(DEFINED MAYA_VERSION AND _ver STREQUAL "${MAYA_VERSION}")
            continue()
        endif()
        list(APPEND _maya_candidates "/Applications/Autodesk/Maya${_ver}/Maya.app/Contents")
    endforeach()
elseif(UNIX)
    if(DEFINED MAYA_VERSION AND NOT MAYA_VERSION STREQUAL "")
        list(APPEND _maya_candidates "/usr/autodesk/maya${MAYA_VERSION}")
        list(APPEND _maya_candidates "/opt/autodesk/maya${MAYA_VERSION}")
    endif()
    foreach(_ver 2027 2026 2025 2024)
        if(DEFINED MAYA_VERSION AND _ver STREQUAL "${MAYA_VERSION}")
            continue()
        endif()
        list(APPEND _maya_candidates "/usr/autodesk/maya${_ver}")
        list(APPEND _maya_candidates "/opt/autodesk/maya${_ver}")
    endforeach()
endif()

# -----------------------------------------------------------------------
# Search
# -----------------------------------------------------------------------
find_path(Maya_INCLUDE_DIR
    NAMES maya/MFnPlugin.h
    PATHS ${_maya_candidates}
    PATH_SUFFIXES include
    NO_DEFAULT_PATH
)

# Find the library directory near the include dir
if(Maya_INCLUDE_DIR)
    get_filename_component(_maya_root "${Maya_INCLUDE_DIR}" DIRECTORY)

    # Check common lib locations
    foreach(_libdir "${_maya_root}/lib" "${_maya_root}/lib64" "${_maya_root}/Maya.app/Contents/MacOS")
        if(EXISTS "${_libdir}/Foundation.lib" OR
           EXISTS "${_libdir}/libFoundation.so" OR
           EXISTS "${_libdir}/libFoundation.dylib")
            set(Maya_LIBRARY_DIR "${_libdir}" CACHE PATH "Maya library directory")
            break()
        endif()
    endforeach()
endif()

# -----------------------------------------------------------------------
# Extract version from include path.
# Matches directory names like:
#   Maya2026              (installed Maya)
#   sdk-maya2026          (official SDK cache naming)
#   sdk-maya2026-windows  (platform-specific SDK cache)
#   Autodesk_Maya_2026    (raw devkit)
# -----------------------------------------------------------------------
if(Maya_INCLUDE_DIR)
    get_filename_component(_maya_parent "${Maya_INCLUDE_DIR}" DIRECTORY)
    get_filename_component(_maya_dirname "${_maya_parent}" NAME)
    # Normalize to lowercase for case-insensitive matching
    string(TOLOWER "${_maya_dirname}" _maya_dirname_lower)
    if(_maya_dirname_lower MATCHES "maya[_-]?([0-9]+)")
        set(Maya_VERSION "${CMAKE_MATCH_1}" CACHE STRING "Maya version")
    else()
        # Fallback: check for a version header
        if(EXISTS "${Maya_INCLUDE_DIR}/maya/MTypes.h")
            file(STRINGS "${Maya_INCLUDE_DIR}/maya/MTypes.h" _maya_ver REGEX "#define MAYA_API_VERSION")
            if(_maya_ver)
                string(REGEX REPLACE ".*([0-9][0-9][0-9][0-9]).*" "\\1" Maya_VERSION "${_maya_ver}")
            endif()
        endif()
    endif()
endif()

# -----------------------------------------------------------------------
# Set required libraries
# -----------------------------------------------------------------------
set(Maya_LIBRARIES
    Foundation
    OpenMaya
    OpenMayaAnim
    OpenMayaFX
    OpenMayaRender
    OpenMayaUI
    CACHE STRING "Maya libraries to link against"
)

# Platform-specific linker settings
if(WIN32)
    set(Maya_PLUGIN_SUFFIX ".mll" CACHE STRING "Maya plugin extension")
elseif(APPLE)
    set(Maya_PLUGIN_SUFFIX ".bundle" CACHE STRING "Maya plugin extension")
else()
    set(Maya_PLUGIN_SUFFIX ".so" CACHE STRING "Maya plugin extension")
endif()

# Platform compile definitions required by Maya plugins.
# Sourced from the official SDK: devkit.cmake + pluginEntry.cmake
if(WIN32)
    set(_maya_defines "_BOOL;NT_PLUGIN;USERDLL;CRT_SECURE_NO_DEPRECATE;_CRT_SECURE_NO_WARNINGS")
elseif(APPLE)
    set(_maya_defines "_BOOL;MAC_PLUGIN")
else()
    set(_maya_defines "LINUX;_BOOL;_GLIBCXX_USE_CXX11_ABI=1")
endif()

# -----------------------------------------------------------------------
# Standard CMake finder handling
# -----------------------------------------------------------------------
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Maya
    REQUIRED_VARS Maya_INCLUDE_DIR Maya_LIBRARY_DIR
    VERSION_VAR Maya_VERSION
)

mark_as_advanced(Maya_INCLUDE_DIR Maya_LIBRARY_DIR Maya_VERSION Maya_LIBRARIES)

# Imported target for modern CMake usage.
# Consumers only need:  target_link_libraries(myPlugin PRIVATE Maya::Maya)
if(Maya_FOUND AND NOT TARGET Maya::Maya)
    add_library(Maya::Maya INTERFACE IMPORTED)
    target_include_directories(Maya::Maya INTERFACE "${Maya_INCLUDE_DIR}")
    target_link_directories(Maya::Maya INTERFACE "${Maya_LIBRARY_DIR}")
    target_link_libraries(Maya::Maya INTERFACE ${Maya_LIBRARIES})
    target_compile_definitions(Maya::Maya INTERFACE ${_maya_defines})
endif()
