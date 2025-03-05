cmake_minimum_required(VERSION 3.8)
project(jet_walking)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${dynamixel_sdk_INCLUDE_DIRS})
include_directories(include/jet_walking)

# motion, sensor, input을 라이브러리로 빌드 (main() 없음)
add_library(motion_lib src/motion.cpp)
target_include_directories(motion_lib PRIVATE include)
ament_target_dependencies(motion_lib rclcpp geometry_msgs sensor_msgs)

add_executable(sensor src/sensor.cpp)
target_include_directories(sensor PRIVATE include)
ament_target_dependencies(sensor rclcpp sensor_msgs)

add_library(input_lib src/input.cpp)
target_include_directories(input_lib PRIVATE include)
ament_target_dependencies(input_lib rclcpp std_msgs)

# controller 실행 파일 생성 & 라이브러리 연결 (main() 있음)
add_executable(controller src/controller.cpp)
target_include_directories(controller PRIVATE include)
#target_link_libraries(controller motion_lib sensor_lib input_lib)
target_link_libraries(controller motion_lib input_lib)
ament_target_dependencies(controller rclcpp std_msgs sensor_msgs geometry_msgs dynamixel_sdk)

# 설치할 대상 지정
install(TARGETS
  motion_lib
  sensor
  input_lib
  controller
  DESTINATION lib/${PROJECT_NAME}
)

# 헤더 파일 설치
install(DIRECTORY include/
  DESTINATION include/
)

# Launch 파일 설치 (추가)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()



