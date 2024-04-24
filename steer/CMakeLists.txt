cmake_minimum_required(VERSION 3.8)
project(steer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#set (CMAKE_CXX_FLAGS "-g -std=c++0x -pthread")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++11 -pthread")
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -pthread")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(carFront
               src/carFront.cpp
	       src/can.cpp
	       src/com.cpp
               src/rt_task.cpp
               src/spidriver.cpp
               src/udp.cpp
)

ament_target_dependencies(carFront rclcpp std_msgs)
#target_link_libraries(carFront pthread)

install(TARGETS
  carFront
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
