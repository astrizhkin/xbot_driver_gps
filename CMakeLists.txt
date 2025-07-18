cmake_minimum_required(VERSION 3.0.2)
include(ExternalProject)
project(xbot_driver_gps)


find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        xbot_msgs
        roscpp
        serial
        robot_localization
        tf2_geometry_msgs
        tf2
        rosgraph_msgs
        message_generation
        )

add_service_files(
        FILES
        SetDatumSrv.srv
        )

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
#        std_msgs
#        geometry_msgs
)

# Geographiclib installs FindGeographicLib.cmake to this non-standard location
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)


set(EP_NEMATODE "NemaTode")
ExternalProject_Add(
        ${EP_NEMATODE}

        PREFIX ${EP_NEMATODE}
        GIT_REPOSITORY https://github.com/astrizhkin/NemaTode
        GIT_TAG ec627bc39607428163216b546f7a607f1fdc0fa5
        GIT_SHALLOW OFF

        BUILD_ALWAYS OFF

        INSTALL_COMMAND cmake -E echo "Skipping install step."

        CMAKE_CACHE_ARGS
        BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config Release
)


find_package(
        Boost
)


catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES mower_comms
        #  CATKIN_DEPENDS mower_msgs roscpp serial
        #  DEPENDS system_lib
        DEPENDS GeographicLib
)

###########
## Build ##
###########

ExternalProject_Get_property(${EP_NEMATODE} SOURCE_DIR)
message("Source dir of ${EP_NEMATODE} = ${SOURCE_DIR}")
include_directories(${SOURCE_DIR}/include)


include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)

add_executable(driver_gps_node
        src/driver_gps_node.cpp
        src/gps_log.h
        src/devices/gps_device.cpp
        src/devices/gps_device.h
        src/devices/serial_gps_device.cpp
        src/devices/serial_gps_device.h
        src/devices/tcp_gps_device.cpp
        src/devices/tcp_gps_device.h
        src/interfaces/gps_interface.cpp
        src/interfaces/gps_interface.h
        src/interfaces/nmea_gps_interface.cpp
        src/interfaces/nmea_gps_interface.h
        src/interfaces/ublox_gps_interface.cpp
        src/interfaces/ublox_gps_interface.h
        src/interfaces/ubx_datatypes.h
)

add_dependencies(driver_gps_node ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(driver_gps_node ${EP_NEMATODE})
ExternalProject_Get_property(${EP_NEMATODE} BINARY_DIR)
target_link_libraries(driver_gps_node ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES} ${BINARY_DIR}/libNemaTode.a)
