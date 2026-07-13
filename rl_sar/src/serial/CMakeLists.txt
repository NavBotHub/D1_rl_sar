cmake_minimum_required(VERSION 3.5)
project(serial)

# ========== 1. Detect ROS version ==========
if($ENV{ROS_VERSION} EQUAL 1)
  set(USE_ROS1 TRUE)
  message(STATUS ">>> Building dm_imu for ROS 1 (Noetic)")
elseif($ENV{ROS_VERSION} EQUAL 2)
  set(USE_ROS2 TRUE)
  message(STATUS ">>> Building dm_imu for ROS 2 (Foxy/Humble)")
else()
  message(STATUS ">>> Building dm_imu in pure-CMake mode")
endif()

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

if(USE_ROS2)
    find_package(rosidl_default_generators REQUIRED)
    find_package(ament_cmake REQUIRED)
    ## Sources
    set(serial_SRCS
        src/serial.cc
        include/serial/serial.h
        include/serial/v8stdint.h
    )

    # If unix
    list(APPEND serial_SRCS src/impl/unix.cc)
    list(APPEND serial_SRCS src/impl/list_ports/list_ports_linux.cc)


    add_library(${PROJECT_NAME} ${serial_SRCS})

    target_link_libraries(${PROJECT_NAME} rt pthread)

    include_directories(include)

    install(TARGETS
            ${PROJECT_NAME}
            ARCHIVE DESTINATION lib
            LIBRARY DESTINATION lib
    )

    ament_export_include_directories(include)

    ament_export_libraries(${PROJECT_NAME})
    #add_executable(${PROJECT_NAME} src/serial.cc)
    install(
            DIRECTORY include/serial
            DESTINATION include)

    ament_export_dependencies(rosidl_default_runtime)
    ament_package()
endif()
