cmake_minimum_required(VERSION 3.16)
project(ocs2_cartpole)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Python bindings are optional
if(BUILD_PYTHON_INTERFACE)
  find_package(pybind11 REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
endif()

if(NOT TARGET ocs2::ocs2_core)
  find_package(ocs2 REQUIRED)
endif()

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)

# Resolve for the package path at compile time.
configure_file(
  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h.in"
  "${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/package_path.h"
  @ONLY
)

add_library(${PROJECT_NAME}
  src/CartPoleInterface.cpp
)
add_library(ocs2::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  ocs2::ocs2_core
  ocs2::ocs2_ddp
  ocs2::ocs2_mpc
  ocs2::ocs2_robotic_tools
  Eigen3::Eigen
  Boost::filesystem
  Boost::system
  dl
)

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

# python bindings (optional)
if(BUILD_PYTHON_INTERFACE)
  pybind11_add_module(CartpolePyBindings
    src/pyBindModule.cpp
  )
  target_link_libraries(CartpolePyBindings PRIVATE
    ${PROJECT_NAME}
    ocs2::ocs2_python_interface
  )

  # Install bindings
  install(TARGETS CartpolePyBindings
    LIBRARY DESTINATION lib/python3/dist-packages/${PROJECT_NAME}
  )
endif()

#############
## Install ##
#############

include(CMakePackageConfigHelpers)
configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/ocs2_cartpoleConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION share/${PROJECT_NAME}/cmake
  NO_SET_AND_CHECK_MACRO
  NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/cmake/${PROJECT_NAME}Config.cmake"
  DESTINATION share/${PROJECT_NAME}/cmake
)

############
# Testing ##
############

if(BUILD_TESTING)
  include(GoogleTest)
  find_package(GTest REQUIRED)

  add_executable(test_cartpole test/testCartpole.cpp)
  target_include_directories(test_cartpole PRIVATE
    ${PROJECT_BINARY_DIR}/include
  )
  target_link_libraries(test_cartpole
    GTest::gtest_main
    ${PROJECT_NAME}
  )
  gtest_discover_tests(test_cartpole DISCOVERY_MODE PRE_TEST)
endif()
