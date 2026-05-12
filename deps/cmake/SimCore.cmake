# Shared core + GLFW target definitions.
# Included by CMakeLists.txt (ament) and by the CLI trampoline (non-ROS host build).
#
# Guarded with if(NOT TARGET ...) to support duplicate inclusion.

if(NOT DEFINED _SIMCORE_GUARD)
  set(_SIMCORE_GUARD TRUE)

  # Resolve project root relative to this file (deps/cmake/ → ../..)
  get_filename_component(SIM_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)

  # MuJoCo library (guard against double-definition)
  if(NOT TARGET mujoco)
    add_library(mujoco SHARED IMPORTED)
    set_target_properties(mujoco PROPERTIES
      IMPORTED_LOCATION ${SIM_ROOT}/deps/lib/libmujoco.so)
  endif()

  # ---- quadrotor_sim_core (headless physics) ----
  if(NOT TARGET quadrotor_sim_core)
    add_executable(quadrotor_sim_core
      ${SIM_ROOT}/src/core/main.cc
      ${SIM_ROOT}/src/core/sim_core.cc
    )
    target_include_directories(quadrotor_sim_core PRIVATE
      ${SIM_ROOT}/include
      ${SIM_ROOT}/src/core
    )
    target_compile_options(quadrotor_sim_core PRIVATE -Wall -Wextra)
    target_link_libraries(quadrotor_sim_core PRIVATE
      mujoco
      pthread
      rt
      ${CMAKE_DL_LIBS}
    )
  endif()

  # ---- quadrotor_sim_glfw_adapter (render viewer) ----
  if(NOT TARGET quadrotor_sim_glfw_adapter)
    find_package(PkgConfig QUIET)
    pkg_check_modules(GLFW glfw3)
    if(GLFW_FOUND)
      add_executable(quadrotor_sim_glfw_adapter
        ${SIM_ROOT}/src/glfw_adapter/glfw_adapter_main.cc
      )
      target_include_directories(quadrotor_sim_glfw_adapter PRIVATE
        ${SIM_ROOT}/include
        ${SIM_ROOT}/src/core
        ${GLFW_INCLUDE_DIRS}
      )
      target_compile_options(quadrotor_sim_glfw_adapter PRIVATE -Wall -Wextra)
      target_link_libraries(quadrotor_sim_glfw_adapter PRIVATE
        mujoco
        ${GLFW_LIBRARIES}
        pthread
        rt
        ${CMAKE_DL_LIBS}
      )
    endif()
  endif()

endif()
