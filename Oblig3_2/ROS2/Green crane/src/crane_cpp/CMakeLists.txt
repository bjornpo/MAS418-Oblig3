cmake_minimum_required(VERSION 3.8)
project(crane_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(crane_interfaces  REQUIRED)

add_executable(Crane_commander src/Crane_commander.cpp)
ament_target_dependencies(Crane_commander rclcpp crane_interfaces)

install(TARGETS
  Crane_commander
  DESTINATION lib/${PROJECT_NAME})

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)



ament_package()
