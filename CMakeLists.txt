cmake_minimum_required(VERSION 3.8)
project(turtle_motion)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)

# C++ 
add_executable(control src/control.cpp)
ament_target_dependencies(control rclcpp geometry_msgs)

# add_executable(transform src/transform.cpp)
# ament_target_dependencies(transform rclcpp geometry_msgs)

install(TARGETS 
  control 
  # transform 
  DESTINATION lib/${PROJECT_NAME}
  )

# Python 
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  turtle_motion/mouse.py
  turtle_motion/window.py
  DESTINATION lib/${PROJECT_NAME}
  )

install(DIRECTORY launch turtle_motion
  DESTINATION share/${PROJECT_NAME}
  )


ament_package()
