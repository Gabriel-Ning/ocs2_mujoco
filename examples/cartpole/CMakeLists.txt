# =============================================================================
# Cartpole Example
# =============================================================================

cmake_minimum_required(VERSION 3.16)
project(ocs2_cartpole)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# =============================================================================
# Dependencies
# =============================================================================

# OCS2
if(NOT TARGET ocs2::ocs2_core)
  find_package(ocs2 REQUIRED)
endif()

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)

# Python bindings
if(BUILD_PYTHON_INTERFACE)
  find_package(pybind11 REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
endif()

# =============================================================================
# Subdirectories
# =============================================================================

add_subdirectory(ocs2_cartpole_interface)

# =============================================================================
# Install Models
# =============================================================================

# Install model and config files
install(DIRECTORY config models
  DESTINATION share/ocs2_cartpole_interface
)
