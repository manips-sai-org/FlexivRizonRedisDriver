cmake_minimum_required(VERSION 3.16.3)

project(flexiv_rizon4_redis_driver)

# CMAKE OPTIONS
# set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)
# SET(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_PREFIX_PATH "$ENV{HOME}/rdk_install")
# Create a library
add_executable(${PROJECT_NAME}_robot_only driver_robot_only.cpp)
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

# tinyxml2 (NEEDED for V1.9.4.1 due to new third party package handling in Flexiv RDK)
find_package(tinyxml2 REQUIRED)

# create headers list
set(flexiv_rizon4_redis_driver_INCLUDE_DIRS 
    ${EIGEN3_INCLUDE_DIR}
    ${JSONCPP_INCLUDE_DIRS}
	${HIREDIS_INCLUDE_DIR}
    ${tinyxml2_INCLUDE_DIRS}
)

# create libraries list
set(flexiv_rizon4_redis_driver_LIBRARIES 
    flexiv::flexiv_rdk
    ${HIREDIS_LIBRARY}
    ${JSONCPP_LIBRARY}
    tinyxml2::tinyxml2
    -lrt
    -lpthread
)

include_directories( ${flexiv_rizon4_redis_driver_INCLUDE_DIRS} )
target_link_libraries(${PROJECT_NAME}_robot_only ${flexiv_rizon4_redis_driver_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_with_gripper ${flexiv_rizon4_redis_driver_LIBRARIES})

target_compile_options(${PROJECT_NAME}_robot_only PRIVATE -O3)
target_compile_options(${PROJECT_NAME}_with_gripper PRIVATE -O3)

set(CONFIG_FOLDER "${CMAKE_CURRENT_SOURCE_DIR}/config_folder")
add_definitions(-DCONFIG_FOLDER="${CONFIG_FOLDER}")

include(CTest)
if(BUILD_TESTING)
    add_executable(flexiv_startup_tests tests/startup_tests.cpp)
    target_include_directories(flexiv_startup_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(flexiv_startup_tests PRIVATE Eigen3::Eigen)
    add_test(NAME flexiv_startup_sequence COMMAND flexiv_startup_tests)

    add_executable(flexiv_idle_tests tests/idle_tests.cpp)
    target_include_directories(flexiv_idle_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(flexiv_idle_tests PRIVATE Eigen3::Eigen)
    add_test(NAME flexiv_idle_handoff COMMAND flexiv_idle_tests)
endif()
