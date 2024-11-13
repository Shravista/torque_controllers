cmake_minimum_required(VERSION 3.8)
project(torque_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=conversion -Werror=unused-but-set-variable
  -Werror=return-type -Werror=shadow -Werror=format)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  torque_msgs
)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
find_package(${Dependency} REQUIRED)
endforeach()

# generate parameter library
generate_parameter_library(
  torque_controller_parameters
  src/torque_controller_parameters.yaml
)
include_directories(include)

# add libraries
add_library(torque_controller SHARED
            src/torque_controller.cpp)
target_compile_features(torque_controller PUBLIC cxx_std_17)
target_link_libraries(torque_controller PUBLIC torque_controller_parameters)
ament_target_dependencies(torque_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(torque_controller PRIVATE "TORQUE_CONTROLLER_BUILDING_DLL")
target_include_directories(torque_controller PUBLIC $<INSTALL_INTERFACE:include/torque_controller>)

# install 
install(
  DIRECTORY include/
  DESTINATION include/torque_controller
)

# install(
#   TARGETS torque_controller
# )

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

install(
  TARGETS
    torque_controller
  EXPORT export_torque_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(export_torque_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
