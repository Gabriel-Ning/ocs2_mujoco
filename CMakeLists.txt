cmake_minimum_required(VERSION 3.16)
project(ocs2_mujoco_tutorial)

# Add the OCS2 library submodule
# Enable examples to build ocs2_mobile_manipulator
set(BUILD_EXAMPLES ON CACHE BOOL "" FORCE)
set(BUILD_PYTHON_INTERFACE ON CACHE BOOL "" FORCE)

add_subdirectory(external/ocs2_lib)

# Examples
add_subdirectory(examples/example_cartpole)
