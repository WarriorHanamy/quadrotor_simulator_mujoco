# Shared MuJoCo library definition.
# Included by CMakeLists.txt (ament) for ROS targets that link mujoco.
#
# Standalone targets are now built via xmake (see xmake.lua).

if(NOT DEFINED _SIMCORE_GUARD)
  set(_SIMCORE_GUARD TRUE)

  get_filename_component(SIM_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)

  if(NOT TARGET mujoco)
    add_library(mujoco SHARED IMPORTED)
    set_target_properties(mujoco PROPERTIES
      IMPORTED_LOCATION ${SIM_ROOT}/deps/lib/libmujoco.so)
  endif()

endif()
