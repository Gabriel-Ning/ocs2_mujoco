# =============================================================================
# OCS2 + MuJoCo Integration Tutorial
# =============================================================================
# This is a minimal top-level CMake configuration that:
# 1. Includes the OCS2 library (as a submodule)
# 2. Builds the example projects
#
# For teaching purposes, each example is self-contained and can be built
# independently once OCS2 is available.
# =============================================================================

cmake_minimum_required(VERSION 3.16)
project(ocs2_mujoco_tutorial)

# Options
option(BUILD_EXAMPLES "Build example projects" ON)
option(BUILD_PYTHON_INTERFACE "Build Python bindings" ON)

# Include OCS2 library (required)
add_subdirectory(external/ocs2_lib)

# Build examples
if(BUILD_EXAMPLES)
  add_subdirectory(examples/cartpole)
endif()
