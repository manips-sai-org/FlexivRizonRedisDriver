cmake_minimum_required(VERSION 3.16.3)

project(flexiv_rizon4_redis_driver)

# CMAKE OPTIONS
# set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)
# SET(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_PREFIX_PATH= "~/rdk_install_flexiv")

# Create a library
add_executable(${PROJECT_NAME} driver_robot_only.cpp)
add_executable(${PROJECT_NAME}_with_gripper driver_with_flexiv_gripper.cpp)

# Find flexiv_rdk INTERFACE library
find_package(flexiv_rdk REQUIRED)

# - hiredis
find_library(HIREDIS_LIBRARY hiredis)
find_path(HIREDIS_INCLUDE_DIR hiredis/hiredis.h REQUIRED)

# - jsoncpp
if (CMAKE_SYSTEM_NAME MATCHES Darwin) 
    find_library(JSONCPP_LIBRARY jsoncpp)
elseif (CMAKE_SYSTEM_NAME MATCHES Linux)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(JSONCPP jsoncpp)
    include_directories(${JSONCPP_INCLUDE_DIRS})
    link_directories(${JSONCPP_LIBRARY_DIRS})
    set(JSONCPP_LIBRARY ${JSONCPP_LIBRARIES})
endif ()

# - eigen3
find_package(Eigen3 REQUIRED)

# create headers list
set(flexiv_rizon4_redis_driver_INCLUDE_DIRS 
    ${EIGEN3_INCLUDE_DIR}
    ${JSONCPP_INCLUDE_DIRS}
	${HIREDIS_INCLUDE_DIR}
)

# create libraries list
set(flexiv_rizon4_redis_driver_LIBRARIES 
    flexiv::flexiv_rdk
    ${HIREDIS_LIBRARY}
    ${JSONCPP_LIBRARY}
    -lrt
    -lpthread
)

include_directories( ${flexiv_rizon4_redis_driver_INCLUDE_DIRS} )
target_link_libraries(${PROJECT_NAME} ${flexiv_rizon4_redis_driver_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_with_gripper ${flexiv_rizon4_redis_driver_LIBRARIES})
