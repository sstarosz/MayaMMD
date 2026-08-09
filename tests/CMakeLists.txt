# =========================================================================
# tests/CMakeLists.txt
#
# Test definitions for MayaMMD — split from the main CMakeLists.txt.
# Included via add_subdirectory(tests) from the project root.
# =========================================================================

# =========================================================================
# Catch2 C++ unit tests (no Maya SDK required)
#
# NOTE: No C++ unit tests exist yet. When they are added:
#   1. Place .cpp test files in tests/
#   2. Uncomment the add_executable + catch_discover_tests block below
# =========================================================================
if(BUILD_TESTS)
    include(FetchContent)
    FetchContent_Declare(
        Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG        v3.7.1
    )
    FetchContent_MakeAvailable(Catch2)

    # TODO: Add test source files and uncomment when ready.
    # add_executable(mmd_tools_tests
    #     test_example.cpp
    # )
    # target_link_libraries(mmd_tools_tests PRIVATE Catch2::Catch2WithMain)
    # target_compile_features(mmd_tools_tests PRIVATE cxx_std_17)
    # include(Catch)
    # catch_discover_tests(mmd_tools_tests)

    message(STATUS "Catch2 available — no C++ tests registered yet")
endif()

# =========================================================================
# Maya integration tests via CTest (requires Maya SDK + mayapy)
# =========================================================================
# Only registered when the native .mll plugin is being built and mayapy
# can be found alongside the Maya SDK.
if(BUILD_NATIVE_MODULE AND Maya_FOUND)
    # Find mayapy — the Maya SDK (build/.sdk/) is stripped and doesn't
    # include mayapy, so we also search the real Maya install directory
    # and the system PATH (which has Maya's bin/ via terminal config).
    get_filename_component(_maya_root "${Maya_INCLUDE_DIR}" DIRECTORY)
    find_program(MAYAPY_EXECUTABLE
        NAMES mayapy mayapy.exe
        PATHS "${_maya_root}/bin"
              "C:/Program Files/Autodesk/Maya${MAYA_VERSION}/bin"
    )

    if(MAYAPY_EXECUTABLE)
        message(STATUS "mayapy: ${MAYAPY_EXECUTABLE}")
        set(_MAYA_INTEGRATION_SUITES
            import bone morph vmd vpd node ccd cmd multi context pose-tree
        )

        foreach(_suite ${_MAYA_INTEGRATION_SUITES})
            add_test(NAME "maya-integration-${_suite}"
                COMMAND "${MAYAPY_EXECUTABLE}"
                        "${CMAKE_SOURCE_DIR}/tests/integration/maya/run_all_integration_tests.py"
                        "${_suite}"
            )
            set_tests_properties("maya-integration-${_suite}" PROPERTIES
                WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
                TIMEOUT 600
                LABELS "maya;integration"
                ENVIRONMENT "PYTHONIOENCODING=utf-8;PYTHONUNBUFFERED=1;MAYA_PLUG_IN_PATH=${CMAKE_INSTALL_PREFIX}/plug-ins;PYTHONPATH=${CMAKE_SOURCE_DIR}"
            )
        endforeach()

        # Full suite (all tests) — longer timeout
        add_test(NAME "maya-integration-all"
            COMMAND "${MAYAPY_EXECUTABLE}"
                    "${CMAKE_SOURCE_DIR}/tests/integration/maya/run_all_integration_tests.py"
                    "all"
        )
        set_tests_properties("maya-integration-all" PROPERTIES
            WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
            TIMEOUT 3600
            LABELS "maya;integration"
            ENVIRONMENT "PYTHONIOENCODING=utf-8;PYTHONUNBUFFERED=1;MAYA_PLUG_IN_PATH=${CMAKE_INSTALL_PREFIX}/plug-ins;PYTHONPATH=${CMAKE_SOURCE_DIR}"
        )

        message(STATUS "Maya integration tests: ${_MAYA_INTEGRATION_SUITES} suites via CTest")
    else()
        message(WARNING
            "mayapy not found — Maya integration tests disabled.\n"
            "  Looked in: ${_maya_root}/bin/")
    endif()
    unset(_maya_root)
endif()
