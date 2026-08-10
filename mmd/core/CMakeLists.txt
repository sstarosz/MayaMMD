# =========================================================================
# mmd/core — Maya-free C++ core (static library)
#
# Pure C++ logic, no Maya SDK required.
# =========================================================================

add_library(mmd_core STATIC
    simulation.cpp
)

# mmd_core is linked into the shared Maya plugin (MayaMMD.so on Linux), so its
# objects must be position-independent code — without this the static archive
# fails to link into the shared object on ELF targets (R_X86_64_PC32 relocation
# error).  Harmless on Windows/macOS.
set_target_properties(mmd_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(mmd_core PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
)

target_include_directories(mmd_core PRIVATE ${BULLET_INCLUDE_DIRS})
target_link_libraries(mmd_core PRIVATE ${BULLET_LIBRARIES})
target_compile_features(mmd_core PUBLIC cxx_std_17)
