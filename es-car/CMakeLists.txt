cmake_minimum_required(VERSION 3.8)
project(es-car)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(steering
	       src/steering.cpp
               src/carFront.cpp
               src/spidriver.cpp
)
ament_target_dependencies(steering rclcpp std_msgs)

install(TARGETS
  steering
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
