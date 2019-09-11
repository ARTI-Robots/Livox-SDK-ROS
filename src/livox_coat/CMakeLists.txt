# Copyright(c) 2019 livoxtech limited.

cmake_minimum_required(VERSION 3.0)


#---------------------------------------------------------------------------------------
# Start livox_coat project
#---------------------------------------------------------------------------------------
include(cmake/version.cmake)
project(livox_coat VERSION ${LIVOX_COAT_VERSION} LANGUAGES CXX)
message(STATUS "livox_coat version: ${LIVOX_COAT_VERSION}")

#---------------------------------------------------------------------------------------
# find package and the dependecy
#---------------------------------------------------------------------------------------
find_package(Boost 1.54 REQUIRED COMPONENTS 
	system
	thread
	chrono
	)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	sensor_msgs
	std_msgs
	message_generation
	rosbag	
	)
		
## Generate messages in the 'msg' folder
add_message_files(FILES
	CustomPoint.msg
	CustomMsg.msg
#   Message2.msg
	)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
	std_msgs
	)

find_package(PkgConfig)
pkg_check_modules(APR apr-1)
if (APR_FOUND)
	message(${APR_INCLUDE_DIRS})
	message(${APR_LIBRARIES})
endif (APR_FOUND)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects als    o need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also n    eed
catkin_package(CATKIN_DEPENDS
	roscpp rospy std_msgs message_runtime
	)

#---------------------------------------------------------------------------------------
# Set default build to release
#---------------------------------------------------------------------------------------
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Release or Debug" FORCE)
endif()

#---------------------------------------------------------------------------------------
# Compiler config
#---------------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


#---------------------------------------------------------------------------------------
# generate excutable and add libraries
#---------------------------------------------------------------------------------------
add_executable(${PROJECT_NAME}_node
    ""
    )

add_dependencies(${PROJECT_HUB}_node ${PROJECT_NAME}_generate_messages_cpp)
#---------------------------------------------------------------------------------------
# precompile macro and compile option
#---------------------------------------------------------------------------------------
target_compile_options(${PROJECT_NAME}_node
    PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall -Werror -Wno-c++11-long-long>
    )

#---------------------------------------------------------------------------------------
# source file
#---------------------------------------------------------------------------------------
target_sources(${PROJECT_NAME}_node
    PRIVATE
    common/delog/delog.cpp
    coat/lvx_file.cpp
	coat/ldq.cpp
	coat/lds.cpp
	coat/lds_lvx.cpp
	coat/lds_lidar.cpp
	coat/lds_hub.cpp
	coat/lddc.cpp
	coat/coat.cpp
    )

#---------------------------------------------------------------------------------------
# include file
#---------------------------------------------------------------------------------------
target_include_directories(${PROJECT_NAME}_node
    PUBLIC
    ${catkin_INCLUDE_DIRS}
	${APR_INCLUDE_DIRS}
	common
    common/spdlog
    common/delog
    coat
    )

#---------------------------------------------------------------------------------------
# link libraries
#---------------------------------------------------------------------------------------
target_link_libraries(${PROJECT_NAME}_node 	
	livox_sdk_static.a
	${Boost_LIBRARY}
	${catkin_LIBRARIES}
	${APR_LIBRARIES}
	)

#---------------------------------------------------------------------------------------
# end of CMakeList.txt
#---------------------------------------------------------------------------------------


