cmake_minimum_required(VERSION 3.5)
project(luma_link)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(uuv_msgs REQUIRED)
find_package(dvl_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(luma_msgs REQUIRED)



include_directories(include
  ${rclcpp_INCLUDE_DIRS}
)

ament_python_install_package(scripts)

add_executable(${PROJECT_NAME}_vehicle src/luma_link_vehicle.cpp)
ament_target_dependencies(${PROJECT_NAME}_vehicle rclcpp geometry_msgs dvl_msgs sensor_msgs uuv_msgs serial)

add_executable(${PROJECT_NAME}_surface src/luma_link_surface.cpp)
ament_target_dependencies(${PROJECT_NAME}_surface rclcpp sensor_msgs dvl_msgs luma_msgs uuv_msgs serial)


#Install C++ targets
install(TARGETS
  ${PROJECT_NAME}_vehicle
  ${PROJECT_NAME}_surface
  DESTINATION lib/${PROJECT_NAME}
)

#Install Python scripts
#file(GLOB SCRIPT_FILES scripts/*)
install(PROGRAMS
  #${SCRIPT_FILES}
  #scripts/imu_sensor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
