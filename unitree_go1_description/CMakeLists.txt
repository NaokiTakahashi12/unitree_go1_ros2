cmake_minimum_required(VERSION 3.8)
project(unitree_go1_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY
    launch
    urdf
    meshes
    config
    rviz
  DESTINATION
    "share/${PROJECT_NAME}"
)
if(DEFINED ENV{GZ_VERSION})
    ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.gz.dsv.in")
elseif(DEFINED ENV{IGN_VERSION})
    ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.ign.dsv.in")
else()
    message(WARNING "Not defined GZ_VERSION or IGN_VERSION")
endif()

ament_package()
