cmake_minimum_required(VERSION 3.8)
project(es_car)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(es_car_inter REQUIRED)

include_directories(include)

add_executable(carstartup
               src/carstartup.cpp
               src/carFront.cpp
               src/spidriver.cpp
)
ament_target_dependencies(carstartup rclcpp std_msgs)

add_executable(dummy
               src/dummy.cpp
)
ament_target_dependencies(dummy rclcpp es_car_inter)

add_executable(steering
	       src/steering.cpp
               src/carFront.cpp
               src/spidriver.cpp
)
ament_target_dependencies(steering rclcpp es_car_inter)

add_executable(throttle
               src/throttle.cpp
               src/carBack.cpp
               src/spidriver.cpp
)
ament_target_dependencies(throttle rclcpp es_car_inter)

add_executable(ultrasonic_front
               src/ulson_front.cpp
               src/carFront.cpp
               src/spidriver.cpp
)
ament_target_dependencies(ultrasonic_front rclcpp std_msgs)

add_executable(wasd_control
               src/wasd_control.cpp
)
ament_target_dependencies(wasd_control rclcpp std_msgs es_car_inter)

install(TARGETS
  dummy
  steering
  throttle
  ultrasonic_front
  carstartup
  wasd_control
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
