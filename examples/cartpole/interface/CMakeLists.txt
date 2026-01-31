cmake_minimum_required(VERSION 3.16)

# =============================================================================
# Cartpole Interface Library
# =============================================================================
#
# This builds:
#   1. ocs2_cartpole_interface - C++ library
#   2. CartpolePyBindings - Python module (if BUILD_PYTHON_INTERFACE is ON)
#
# The simplified structure uses:
#   - Cartpole.h: Single header with all definitions
#   - Cartpole.cpp: Implementation
#   - PyBindModule.cpp: Python bindings
#

set(LIB_NAME "ocs2_cartpole_interface")

# -----------------------------------------------------------------------------
# C++ Library
# -----------------------------------------------------------------------------

add_library(${LIB_NAME}
  src/Cartpole.cpp
)
add_library(ocs2::${LIB_NAME} ALIAS ${LIB_NAME})

target_include_directories(${LIB_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${LIB_NAME}
  PUBLIC
    ocs2::ocs2_core
    ocs2::ocs2_ddp
    ocs2::ocs2_mpc
    ocs2::ocs2_robotic_tools
    ocs2::ocs2_python_interface
    Eigen3::Eigen
    Boost::filesystem
    Boost::system
    dl
)

target_compile_options(${LIB_NAME} PUBLIC ${OCS2_CXX_FLAGS})

# -----------------------------------------------------------------------------
# Python Bindings
# -----------------------------------------------------------------------------

if(BUILD_PYTHON_INTERFACE)
  pybind11_add_module(CartpolePyBindings
    src/PyBindModule.cpp
  )
  target_link_libraries(CartpolePyBindings PRIVATE
    ${LIB_NAME}
    ocs2::ocs2_python_interface
  )

  # Install to Python packages directory
  install(TARGETS CartpolePyBindings
    LIBRARY DESTINATION lib/python3/dist-packages/ocs2_cartpole
  )
endif()

# -----------------------------------------------------------------------------
# Install
# -----------------------------------------------------------------------------

install(TARGETS ${LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${LIB_NAME}
)
