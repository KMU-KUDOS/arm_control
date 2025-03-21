cmake_minimum_required(VERSION 3.5)
project(joystick_control)

#ROS 2 의 필수 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

#실행 파일 생성 (타겟 이름을 고유하게 지정)
add_executable(joystick_node_rpm src/joystick_node_rpm.cpp)
add_executable(joystick_node_tq src/joystick_node_tq.cpp)
add_executable(joystick_node_pcc src/joystick_node_pcc.cpp)
add_executable(joystick_node_aq src/joystick_node_aq.cpp)
add_executable(joystick_node_9c src/joystick_node_9c.cpp)
add_executable(joystick_node_apc src/joystick_node_apc.cpp)

#의존성 설정
ament_target_dependencies(joystick_node_rpm rclcpp sensor_msgs)
ament_target_dependencies(joystick_node_tq rclcpp sensor_msgs)
ament_target_dependencies(joystick_node_pcc rclcpp sensor_msgs)
ament_target_dependencies(joystick_node_aq rclcpp sensor_msgs)
ament_target_dependencies(joystick_node_9c rclcpp sensor_msgs)
ament_target_dependencies(joystick_node_apc rclcpp sensor_msgs)

#실행 파일 설치 경로 설정
install(TARGETS
joystick_node_rpm
joystick_node_tq
joystick_node_pcc
joystick_node_aq
joystick_node_9c
joystick_node_apc
DESTINATION lib/${PROJECT_NAME}
)

#패키지 설정
ament_package()

