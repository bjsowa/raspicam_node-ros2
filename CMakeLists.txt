cmake_minimum_required(VERSION 3.5)
project(raspicam2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set (CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-as-needed")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(class_loader REQUIRED)

FIND_LIBRARY(mmal_LIBS REQUIRED NAMES mmal PATHS /opt/vc/lib)
FIND_LIBRARY(mmalcore_LIBS REQUIRED NAMES mmal_core PATHS /opt/vc/lib)
FIND_LIBRARY(mmalutil_LIBS REQUIRED NAMES mmal_util PATHS /opt/vc/lib)
FIND_LIBRARY(mmalvcclient_LIBS REQUIRED NAMES mmal_vc_client PATHS /opt/vc/lib)
FIND_LIBRARY(bcmhost_LIBS REQUIRED NAMES bcm_host PATHS /opt/vc/lib)

include_directories(include /opt/vc/include)

add_library(raspicam STATIC
    src/raspicam_common/RaspiCamControl.c
    src/raspicam_common/RaspiCLI.c
    src/raspicam_common/RaspiPreview.c
    src/raspicam_common/RaspiCommonSettings.c
    src/raspicam_common/RaspiHelpers.c
)
target_link_libraries(raspicam
  ${bcmhost_LIBS} ${mmal_LIBS} ${mmalcore_LIBS} ${mmalutil_LIBS} ${mmalvcclient_LIBS})
target_include_directories(raspicam 
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include/raspicam_common
)

# composable ros node
add_library(raspicam_publisher SHARED
  src/raspicam.cpp
  src/raspicam_publisher.cpp
)
target_link_libraries(raspicam_publisher
  raspicam
)
ament_target_dependencies(raspicam_publisher
  rclcpp
  rclcpp_components
  sensor_msgs
  class_loader
)
rclcpp_components_register_node(raspicam_publisher 
  PLUGIN "RaspicamPublisher"
  EXECUTABLE "raspicam_node"
)

install(TARGETS
  raspicam_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_package()
