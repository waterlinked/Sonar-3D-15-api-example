cmake_minimum_required(VERSION 3.8)
project(waterlinked_3d_sonar_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Add your source files here
# add_executable(${PROJECT_NAME}_node src/your_node.cpp)
# ament_target_dependencies(${PROJECT_NAME}_node rclcpp sensor_msgs std_msgs)

install(DIRECTORY proto/
  DESTINATION share/${PROJECT_NAME}/proto
)

install(PROGRAMS
  scripts/sonar_3d_15_node.py
  scripts/inspect_sonar_data.py
  scripts/interface_sonar_api.py
  scripts/save_sonar_data.py
  scripts/sonar_3d_15_protocol_pb2.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
