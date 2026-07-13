cmake_minimum_required(VERSION 3.5)          # 3.5 or later is required to recognize $ENV{ROS_VERSION}
project(dm_imu)

# ========== 1. Detect ROS version ==========
if($ENV{ROS_VERSION} EQUAL 1)
  set(USE_ROS1 TRUE)
  add_compile_definitions(USE_ROS1)
  # set(PACKAGE_XML_SOURCE "package.ros1.xml")
elseif($ENV{ROS_VERSION} EQUAL 2)
  set(USE_ROS2 TRUE)
  add_compile_definitions(USE_ROS2)
  # set(PACKAGE_XML_SOURCE "package.ros2.xml")
else()
  message(STATUS ">>> Building dm_imu in pure-CMake mode")
endif()

# ========== 2. Common compile flags ==========
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(Threads REQUIRED)

# ========== 3. ROS 1 branch ==========
if(USE_ROS1)
#   configure_file(
#   ${CMAKE_CURRENT_SOURCE_DIR}/${PACKAGE_XML_SOURCE}
#   ${CMAKE_CURRENT_BINARY_DIR}/package.xml
#   COPYONLY
# )
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    serial
  )

  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES dm_imu
  )

  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
  )

  # Build library (optional, if some code is later split into a library)
  add_library(${PROJECT_NAME}
    src/bsp_crc.cpp
    src/imu_driver.cpp
  )
  target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

  # Build executable
  add_executable(${PROJECT_NAME}_node
    src/bsp_crc.cpp
    src/imu_driver.cpp
    src/test_imu.cpp
  )
  target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
  )

  install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/package.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
  install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
  install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
# ========== 4. ROS 2 branch ==========
elseif(USE_ROS2)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(serial REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)
  include_directories(include)

  # Library
  add_library(${PROJECT_NAME}
    src/bsp_crc.cpp
    src/imu_driver.cpp
  )
  ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs serial sensor_msgs tf2 tf2_ros tf2_geometry_msgs)
  target_link_libraries(${PROJECT_NAME} Threads::Threads)

  # Executable
  add_executable(${PROJECT_NAME}_node
    src/test_imu.cpp
  )
  ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs serial sensor_msgs tf2 tf2_ros tf2_geometry_msgs)
  target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

  install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
  install(DIRECTORY include/ DESTINATION include)
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
  install(FILES package.xml DESTINATION share/${PROJECT_NAME})
  ament_package()

# ========== 5. Pure CMake branch (no ROS) ==========
else()
  include_directories(include)

  # Library
  add_library(${PROJECT_NAME}
    src/bsp_crc.cpp
    src/imu_driver.cpp
  )
  target_link_libraries(${PROJECT_NAME} Threads::Threads)

  # Executable
  add_executable(${PROJECT_NAME}_node
    src/test_imu.cpp
  )
  target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

  install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)
  install(DIRECTORY include/ DESTINATION include)
endif()