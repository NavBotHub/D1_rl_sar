cmake_minimum_required(VERSION 3.5)
project(dmbot_serial)

# ========== 1. 检测 ROS 版本 ==========
if($ENV{ROS_VERSION} EQUAL 1)
  set(USE_ROS1 TRUE)
  message(STATUS ">>> Building for ROS 1 (Noetic)")
elseif($ENV{ROS_VERSION} EQUAL 2)
  set(USE_ROS2 TRUE)
  message(STATUS ">>> Building for ROS 2 (Foxy/Humble)")
else()
  message(STATUS ">>> Building in pure-CMake mode")
endif()

# ========== 2. 通用编译 flags ==========
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(Threads REQUIRED)

# ========== 3. ROS 1 分支 ==========
if(USE_ROS1)
  configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/package.ros1.xml
    ${CMAKE_CURRENT_BINARY_DIR}/package.xml
    COPYONLY
  )
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
  )

  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES motor
    CATKIN_DEPENDS roscpp
  )

  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
  )

  # 生成库
  add_library(motor
    src/protocol/damiao.cpp
    src/protocol/socketcan.cpp
  )
  add_library(dmbot_serial::motor ALIAS motor) 
  target_link_libraries(motor ${catkin_LIBRARIES} Threads::Threads)

  # 生成可执行文件
  add_executable(test_motor src/test.cpp)
  target_link_libraries(test_motor motor ${catkin_LIBRARIES})

  add_executable(set_zero_all src/set_zero_all.cpp)
  target_link_libraries(set_zero_all motor ${catkin_LIBRARIES})

  add_executable(set_zero_one src/set_zero_one.cpp)
  target_link_libraries(set_zero_one motor ${catkin_LIBRARIES})

  install(TARGETS motor test_motor set_zero_all set_zero_one
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# ========== 4. ROS 2 分支 ==========
elseif(USE_ROS2)
  configure_file(
      ${CMAKE_CURRENT_SOURCE_DIR}/package.ros2.xml
      ${CMAKE_CURRENT_BINARY_DIR}/package.xml
      COPYONLY
  )
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)

  include_directories(include)

  # 生成库
  add_library(motor
    src/protocol/damiao.cpp
    src/protocol/socketcan.cpp
  )
  add_library(dmbot_serial::motor ALIAS motor)
  ament_target_dependencies(motor rclcpp std_msgs)
  target_link_libraries(motor Threads::Threads)

  # 生成可执行文件
  add_executable(test_motor src/test.cpp)
  ament_target_dependencies(test_motor rclcpp std_msgs)
  target_link_libraries(test_motor motor)

  add_executable(set_zero_all src/set_zero_all.cpp)
  target_link_libraries(set_zero_all motor)

  add_executable(set_zero_one src/set_zero_one.cpp)
  target_link_libraries(set_zero_one motor)


  install(TARGETS motor
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  install(TARGETS test_motor set_zero_all set_zero_one
    DESTINATION lib/${PROJECT_NAME}
  )
  
  install(DIRECTORY include/
    DESTINATION include/
  )
  ament_export_targets(export_${PROJECT_NAME})
  ament_export_dependencies(rclcpp std_msgs)
  ament_package()

# ========== 5. 纯 CMake 分支 ==========
else()
  include_directories(include)

  add_library(motor
    src/protocol/damiao.cpp
    src/protocol/socketcan.cpp
  )
  add_library(dmbot_serial::motor ALIAS motor)
  target_link_libraries(motor Threads::Threads)

  add_executable(test_motor src/test.cpp)
  target_link_libraries(test_motor motor)

  add_executable(set_zero_all src/set_zero_all.cpp)
  target_link_libraries(set_zero_all motor)

  add_executable(set_zero_one src/set_zero_one.cpp)
  target_link_libraries(set_zero_one motor)

  install(TARGETS motor test_motor set_zero_all set_zero_one
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib)
  install(DIRECTORY include/ DESTINATION include)
endif()

if(USE_ROS1 OR USE_ROS2)
  if(USE_ROS1)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/package.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
  else()
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/package.xml
    DESTINATION share/${PROJECT_NAME}
  )
  endif()
endif()