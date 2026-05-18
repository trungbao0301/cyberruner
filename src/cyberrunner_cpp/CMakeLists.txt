cmake_minimum_required(VERSION 3.8)
project(cyberrunner_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── ROS2 dependencies ────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)

# ── OpenCV ───────────────────────────────────────────────────────────────────
find_package(OpenCV REQUIRED)

# ── hidapi ───────────────────────────────────────────────────────────────────
find_package(PkgConfig REQUIRED)
pkg_check_modules(HIDAPI REQUIRED hidapi-hidraw)

# ── nlohmann JSON ────────────────────────────────────────────────────────────
find_package(nlohmann_json REQUIRED)

# ── Threads ──────────────────────────────────────────────────────────────────
find_package(Threads REQUIRED)

# ── Common ROS dependencies list ─────────────────────────────────────────────
set(ROS_DEPS
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  rcl_interfaces
  cv_bridge
)

include_directories(include)

# ── camera_node ───────────────────────────────────────────────────────────────
add_executable(camera_node src/camera_node.cpp)
ament_target_dependencies(camera_node ${ROS_DEPS})
target_link_libraries(camera_node ${OpenCV_LIBS})

# ── estimator_node ────────────────────────────────────────────────────────────
add_executable(estimator_node src/estimator_node.cpp)
ament_target_dependencies(estimator_node ${ROS_DEPS})
target_link_libraries(estimator_node
  ${OpenCV_LIBS}
  nlohmann_json::nlohmann_json)

# ── marble_node ───────────────────────────────────────────────────────────────
add_executable(marble_node src/marble_node.cpp)
ament_target_dependencies(marble_node ${ROS_DEPS})
target_link_libraries(marble_node ${OpenCV_LIBS})

# ── path_node ─────────────────────────────────────────────────────────────────
add_executable(path_node src/path_node.cpp)
ament_target_dependencies(path_node ${ROS_DEPS})
target_link_libraries(path_node
  ${OpenCV_LIBS}
  nlohmann_json::nlohmann_json)

# ── path_progress_node ────────────────────────────────────────────────────────
add_executable(path_progress_node src/path_progress_node.cpp)
ament_target_dependencies(path_progress_node ${ROS_DEPS})

# ── ri_node ───────────────────────────────────────────────────────────────────
add_executable(ri_node src/ri_node.cpp)
ament_target_dependencies(ri_node ${ROS_DEPS})

# ── controller_node ───────────────────────────────────────────────────────────
add_executable(controller_node src/controller_node.cpp)
ament_target_dependencies(controller_node ${ROS_DEPS})
target_link_libraries(controller_node ${OpenCV_LIBS} Threads::Threads)

# ── hiwonder_node ─────────────────────────────────────────────────────────────
add_executable(hiwonder_node src/hiwonder_node.cpp)
ament_target_dependencies(hiwonder_node rclcpp std_msgs std_srvs)
target_include_directories(hiwonder_node PRIVATE ${HIDAPI_INCLUDE_DIRS})
target_link_libraries(hiwonder_node ${HIDAPI_LIBRARIES})

# ── tuner_node ────────────────────────────────────────────────────────────────
add_executable(tuner_node src/tuner_node.cpp)
ament_target_dependencies(tuner_node ${ROS_DEPS})
target_link_libraries(tuner_node
  ${OpenCV_LIBS}
  nlohmann_json::nlohmann_json)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS
  camera_node
  estimator_node
  marble_node
  path_node
  path_progress_node
  ri_node
  controller_node
  hiwonder_node
  tuner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
