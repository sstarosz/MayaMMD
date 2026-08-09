# =========================================================================
# tests/core — C++ unit tests for the Maya-free core (mmd_core)
#
# Pure-logic tests (no Maya SDK).  Include dirs and Bullet come from the
# mmd_core target, so only Catch2 needs to be linked explicitly.
# =========================================================================

add_executable(mmd_core_tests
    test_physics_math.cpp
    test_simulation.cpp
)
target_link_libraries(mmd_core_tests PRIVATE
    Catch2::Catch2WithMain
    mmd_core
)
target_compile_features(mmd_core_tests PRIVATE cxx_std_17)
include(Catch)
catch_discover_tests(mmd_core_tests)

message(STATUS "Catch2 available — registered mmd_core_tests (tests/core)")
