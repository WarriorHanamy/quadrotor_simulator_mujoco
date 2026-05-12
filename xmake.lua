add_rules("mode.debug", "mode.release")
set_languages("cxx17")

-- Output binaries to flat directory matching the existing CLI convention
set_targetdir("build_standalone")

-- Bundled MuJoCo 2.3.2 (no xmake-repo package available)
add_linkdirs("deps/lib")
add_links("mujoco")

-- External dependencies with pinned versions
add_requires("yaml-cpp 0.8.0")
add_requires("eigen 3.4.0")
add_requires("glfw3")

-- ---------------------------------------------------------------------------
-- Target: quadrotor_sim_core (headless physics engine)
-- ---------------------------------------------------------------------------
target("quadrotor_sim_core")
    set_kind("binary")
    add_files("src/core/main.cc", "src/core/sim_core.cc")
    add_includedirs("include", "src/core")
    add_links("mujoco")
    add_syslinks("pthread", "rt", "dl")

-- ---------------------------------------------------------------------------
-- Target: quadrotor_sim_se3_direct (SE3 controller via shared memory)
-- ---------------------------------------------------------------------------
target("quadrotor_sim_se3_direct")
    set_kind("binary")
    add_files("src/se3_controller/main_direct.cc",
              "src/se3_controller/se3_controller.cc")
    add_includedirs("include", "src/core", "src/se3_controller")
    add_packages("yaml-cpp", "eigen")
    add_syslinks("pthread", "rt", "dl")

-- ---------------------------------------------------------------------------
-- Target: quadrotor_sim_glfw_adapter (GLFW render viewer)
-- ---------------------------------------------------------------------------
target("quadrotor_sim_glfw_adapter")
    set_kind("binary")
    add_files("src/glfw_adapter/glfw_adapter_main.cc")
    add_includedirs("include", "src/core")
    add_packages("glfw3")
    add_links("mujoco")
    add_syslinks("pthread", "rt", "dl")
