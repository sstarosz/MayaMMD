# GitVersion.cmake — Read version from git tags (replaces hatch-vcs)
#
# Git is the single source of truth.  No Python, no external tools needed
# at CMake configure time — just Git.
#
# Usage:
#   include(GitVersion)
#   get_git_version(MMD_PROJECT_VERSION)
#
# Output:
#   Tagged release:     v1.2.0             →  1.2.0
#   Dev after tag:      v1.2.0-7-g83abef1  →  1.2.0.dev7+g83abef1
#   Dirty working tree: (appends)           →  …dirty
#   No tags at all:                         →  0.0.0+g<commit>
#   No git available:                       →  "" (caller handles fallback)

function(get_git_version out_var)
    find_package(Git QUIET)

    if(NOT GIT_FOUND)
        set(${out_var} "" PARENT_SCOPE)
        return()
    endif()

    # --tags                    include lightweight tags
    # --match "v[0-9]*"         only tags starting with v<digit> (version tags)
    # --dirty                   append -dirty when working tree has changes
    # --always                  fall back to commit hash when no tags exist
    # --abbrev=7                short commit hash length
    execute_process(
        COMMAND ${GIT_EXECUTABLE} describe
                --tags --match "v[0-9]*" --dirty --always --abbrev=7
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_DESCRIBE
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
        RESULT_VARIABLE _git_result
    )

    if(_git_result OR NOT GIT_DESCRIBE)
        set(${out_var} "" PARENT_SCOPE)
        return()
    endif()

    # ── Transform git-describe → project version ─────────────────────
    # v1.2.3                   →  1.2.3
    # v1.2.3-7-g83abef1        →  1.2.3.dev7+g83abef1
    # v1.2.3-7-g83abef1-dirty  →  1.2.3.dev7+g83abef1.dirty
    # 83abef1-dirty            →  0.0.0+g83abef1.dirty   (no tags)

    set(_version "${GIT_DESCRIBE}")

    # Check if a version tag was found (starts with v<digit>)
    string(REGEX MATCH "^v[0-9]" _has_tag "${_version}")

    if(_has_tag)
        # Strip 'v' prefix
        string(REGEX REPLACE "^v" "" _version "${_version}")

        # Replace "-N-gXXXX" with ".devN+gXXXX"  (commits since tag)
        string(REGEX REPLACE "-([0-9]+)-g" ".dev\\1+g" _version "${_version}")

        # Replace trailing "-dirty" with ".dirty"
        string(REGEX REPLACE "-dirty$" ".dirty" _version "${_version}")
    else()
        # No version tag — prefix with 0.0.0+g
        string(REGEX REPLACE "-dirty$" ".dirty" _version "${_version}")
        set(_version "0.0.0+g${_version}")
    endif()

    set(${out_var} "${_version}" PARENT_SCOPE)
endfunction()
