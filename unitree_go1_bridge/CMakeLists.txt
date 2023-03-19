cmake_minimum_required(VERSION 3.15)
project(unitree_go1_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(UnitreeLeggedSDK REQUIRED)

message(${UnitreeLeggedSDK_LIBRARY})

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

generate_parameter_library(unitree_go1_bridge_node_parameters
  src/unitree_go1_bridge_node_parameters.yaml
)

add_library(unitree_go1_communicator SHARED)
target_sources(unitree_go1_communicator
  PUBLIC
    src/control_communicator.cpp
    src/utility.cpp
)
target_include_directories(unitree_go1_communicator
  PUBLIC
    include
)
target_link_libraries(unitree_go1_communicator
  PUBLIC
    UnitreeLeggedSDK::UnitreeLeggedSDK
    ${UnitreeLeggedSDK_LIBRARY}
)
set_property(
  TARGET unitree_go1_communicator
  PROPERTY
    POSITION_INDEPENDENT_CODE ON
)

ament_auto_add_library(unitree_go1_bridge SHARED
  src/unitree_go1_bridge_node.cpp
)
target_link_libraries(unitree_go1_bridge
  unitree_go1_communicator
  ${UnitreeLeggedSDK_LIBRARY}
  unitree_go1_bridge_node_parameters
)
target_compile_features(unitree_go1_bridge
  PUBLIC
    cxx_std_17
)
rclcpp_components_register_node(unitree_go1_bridge
  PLUGIN "unitree_go1_bridge::UnitreeGo1BridgeNode"
  EXECUTABLE unitree_go1_bridge_node
)

install(
  TARGETS
    unitree_go1_communicator
    unitree_go1_bridge
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
    DIRECTORY include/
    DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
