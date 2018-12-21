project(panda_dual_arm_control_vrep)
cmake_minimum_required(VERSION 2.8)
aux_source_directory(. SRC_LIST)

LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" )
MESSAGE( STATUS "CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH} )


find_package(PkgConfig QUIET)
find_package(Eigen3 REQUIRED)
find_package(RBDL REQUIRED)

set (VREP_ROOT
    $ENV{HOME}/vrep/
    CACHE "VRep root dir" STRING)

include_directories(
  ${EIGEN3_INCLUDE_DIR} 
  ${RBDL_INCLUDE_DIR}
  ${VREP_ROOT}/programming/include 
  ${VREP_ROOT}/programming/remoteApi
)


include_directories (${VREP_PATH}/remoteApi ${VREP_PATH}/include include)

add_definitions (-DNON_MATLAB_PARSING)
add_definitions (-DMAX_EXT_API_CONNECTIONS=255)
#add_definitions (-DDO_NOT_USE_SHARED_MEMORY)

set (SOURCES
	src/controller.cpp
	src/main.cpp
  src/vrep_bridge.cpp
  src/panda_rbdl_model.cpp

	${VREP_ROOT}/programming/remoteApi/extApi.c
	${VREP_ROOT}/programming/remoteApi/extApiPlatform.c
	${VREP_ROOT}/programming/common/shared_memory.c
)

add_executable(panda_dual_arm_control_vrep ${SOURCES})

target_link_libraries(panda_dual_arm_control_vrep
	${RBDL_LIBRARY}
	pthread
	rt
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(CMAKE_BUILD_TYPE RELEASE)

