cmake_minimum_required(VERSION 3.8)
project(unitree_legged_sdk_go1_vendor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(UnitreeLeggedSDK QUIET)

if(UnitreeLeggedSDK_FOUND AND UnitreeLeggedSDK_INCLUDE_DIR AND UnitreeLeggedSDK_LIBRARY)
  message(STATUS "Ignore build unitree_legged_sdk")
else()
  set(unitree_legged_sdk_version 3.8.0)
  set(unitree_legged_sdk_install_dir "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-install")
  set(extra_cmake_args)
  list(APPEND extra_cmake_args "-D CMAKE_BUILD_TYPE=Release")
  list(APPEND extra_cmake_args "-D CMAKE_INSTALL_PREFIX=${unitree_legged_sdk_install_dir}")
  include(ExternalProject)
  externalproject_add(unitree_legged_sdk_${unitree_legged_sdk_version}
    GIT_REPOSITORY https://github.com/NaokiTakahashi12/unitree_legged_sdk.git
    GIT_TAG go1-latest
    CMAKE_ARGS ${extra_cmake_args}
  )
  install(
    DIRECTORY ${unitree_legged_sdk_install_dir}/
    DESTINATION ${CMAKE_INSTALL_PREFIX}
    USE_SOURCE_PERMISSIONS
  )
endif()

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
