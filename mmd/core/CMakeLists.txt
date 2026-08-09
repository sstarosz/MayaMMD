# =========================================================================
# mmd/core — Maya-free C++ core (static library)
#
# Pure C++ logic, no Maya SDK required.
# =========================================================================

add_library(mmd_core STATIC
    simulation.cpp
)
target_include_directories(mmd_core PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
    ${BULLET_INCLUDE_DIRS}
)

target_link_libraries(mmd_core PRIVATE ${BULLET_LIBRARIES})
target_compile_features(mmd_core PUBLIC cxx_std_17)
