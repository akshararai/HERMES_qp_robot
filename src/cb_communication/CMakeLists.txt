cmake_minimum_required(VERSION 2.6)

#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_VERBOSE_MAKEFILE ON)
PROJECT(hermes_communication_tools)

#execute_process(COMMAND /usr/xenomai/bin/xeno-config --skin=native --cflags OUTPUT_VARIABLE XENO_NATIVE_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
#execute_process(COMMAND /usr/xenomai/bin/xeno-config --skin=native --ldflags OUTPUT_VARIABLE XENO_NATIVE_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)

find_package(catkin REQUIRED COMPONENTS
  sl_config
  sl
  xenomai_ros
)

#SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -Wall -O3 ${XENO_NATIVE_CFLAGS}")
#SET(CMAKE_VERBOSE_MAKEFILE on )

find_package(Boost COMPONENTS system filesystem thread REQUIRED)
find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS xenomai_ros sl
  DEPENDS Boost
)
sl_package()

INCLUDE_DIRECTORIES(
  include 
  /usr/local/rtnet/include
  /usr/src/rtnet/stack/include
  ${Eigen_INCLUDE_DIRS} 
  ${catkin_INCLUDE_DIRS}
  ${Xenomai_INCLUDE_DIR})
LINK_DIRECTORIES(#${PROJECT_NAME_BIN}/lib 
${Boost_LIBRARY_DIR})

if(Xenomai_FOUND)
  add_xenomai_flags()
endif()

#SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
#SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

add_library(${PROJECT_NAME}
		src/gdc_common/FootSensorMsg.cpp
		src/gdc_common/FootSensorState.cpp
		src/gdc_common/GDCMsg.cpp
		src/gdc_common/GDCState.cpp
		src/gdc_common/GDCNetwork.cpp
		src/gdc_common/gdc_utils.cpp
		src/imu_common/3dmgx2.cpp
    src/imu_common/microstrain_3dmgx3_25.cpp
		src/imu_common/ImuInterfaceNonRT.cpp
		src/imu_common/ImuInterfaceNonRTStream.cpp
    src/imu_common/ImuInterface.cpp)
target_link_libraries(${PROJECT_NAME}
	  ${Xenomai_LIBRARIES_NATIVE}
    ${Xenomai_LIBRARY_RTDM}
)
ADD_EXECUTABLE(test_communication_loop
		src/programs/test_communication_loop.cpp)
target_link_libraries(test_communication_loop
  ${PROJECT_NAME}
  ${Xenomai_LIBRARIES_NATIVE}
  ${Xenomai_LIBRARY_RTDM} 
  ${Xenomai_LIBRARY_RTDK}
  ${Boost_LIBRARIES}
  ${utilities_LIBRARIES}
)

ADD_EXECUTABLE(save_gdc_card_state
		src/programs/save_gdc_card_state.cpp)
target_link_libraries(save_gdc_card_state
  ${PROJECT_NAME}
  ${Xenomai_LIBRARIES_NATIVE}
  ${Xenomai_LIBRARY_RTDM} 
  ${Xenomai_LIBRARY_RTDK}
  ${Boost_LIBRARIES}
  ${utilities_LIBRARIES}
)

ADD_EXECUTABLE(record_min_max
		src/programs/record_min_max.cpp)
target_link_libraries(record_min_max
  ${PROJECT_NAME}
  ${Xenomai_LIBRARIES_NATIVE}
  ${Xenomai_LIBRARY_RTDM} 
  ${Xenomai_LIBRARY_RTDK}
  ${Boost_LIBRARIES}
  ${utilities_LIBRARIES}
)

ADD_EXECUTABLE(imu_test_loop
		src/programs/imu_test_loop.cpp)
target_link_libraries(imu_test_loop
  ${PROJECT_NAME}
  ${Xenomai_LIBRARIES_NATIVE}
  ${Xenomai_LIBRARY_RTDM} 
  ${Xenomai_LIBRARY_RTDK}
  ${Boost_LIBRARIES}
  ${utilities_LIBRARIES}
)

ADD_EXECUTABLE(periodic_loop
		src/programs/test_periodic_loop.cpp)
target_link_libraries(periodic_loop
  ${PROJECT_NAME}
  ${Xenomai_LIBRARIES_NATIVE}
  ${Xenomai_LIBRARY_RTDM} 
  ${Xenomai_LIBRARY_RTDK}
  ${Boost_LIBRARIES}
  ${utilities_LIBRARIES}
)

ADD_EXECUTABLE(imu_non_rt_test
		src/programs/imu_non_rt_test.cpp)
target_link_libraries(imu_non_rt_test
		${PROJECT_NAME}
		${Xenomai_LIBRARIES_NATIVE}
		${Xenomai_LIBRARY_RTDM} 
		${Xenomai_LIBRARY_RTDK}
		${Boost_LIBRARIES}
		${utilities_LIBRARIES}
		)


# add a target to generate API documentation with Doxygen
find_package(Doxygen)
if(DOXYGEN_FOUND)
add_custom_target(${PROJECT_NAME}_doc
${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/doc/Doxyfile
WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
COMMENT "Generating API documentation with Doxygen" VERBATIM
)
endif(DOXYGEN_FOUND)
