cmake_minimum_required(VERSION 2.6)

project(pf_imu)

find_package(catkin REQUIRED COMPONENTS xenomai_ros)

#execute_process(COMMAND /usr/xenomai/bin/xeno-config --skin=native --cflags OUTPUT_VARIABLE XENO_NATIVE_CFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)
#execute_process(COMMAND /usr/xenomai/bin/xeno-config --skin=native --ldflags OUTPUT_VARIABLE XENO_NATIVE_LDFLAGS OUTPUT_STRIP_TRAILING_WHITESPACE)


#SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -Wall -O3 ${XENO_NATIVE_CFLAGS}")
#SET(CMAKE_VERBOSE_MAKEFILE on )

find_package(Boost COMPONENTS system filesystem thread REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS xenomai_ros
)

INCLUDE_DIRECTORIES(include  
  /usr/local/rtnet/include 
  /usr/src/rtnet/stack/include
  ${catkin_INCLUDE_DIRS} 
  ${Xenomai_INCLUDE_DIR})
LINK_DIRECTORIES(#${PROJECT_NAME_BIN}/lib 
${Boost_LIBRARY_DIR})

if(Xenomai_FOUND)
message(STATUS "Xenomai_INCLUDE_DIR: ${Xenomai_INCLUDE_DIR}")
  add_xenomai_flags()
endif()


#SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
#SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

ADD_LIBRARY(${PROJECT_NAME}
		src/pf_imu.cpp)
target_link_libraries(${PROJECT_NAME}
  ${Xenomai_LIBRARIES_NATIVE}
  ${Xenomai_LIBRARY_RTDM}
  ${Boost_LIBRARIES})

ADD_EXECUTABLE(test_pf_imu
		tests/test_pf_imu.cpp)
target_link_libraries(test_pf_imu
  ${PROJECT_NAME}
  ${Xenomai_LIBRARIES_NATIVE}
  ${Xenomai_LIBRARY_RTDM}
  ${Boost_LIBRARIES}
)

ADD_EXECUTABLE(test_rt_serial
    src/rt_serial_uprog.c)
target_link_libraries(test_rt_serial
#  ${PROJECT_NAME}
  ${Xenomai_LIBRARIES_NATIVE}
  ${Xenomai_LIBRARY_RTDM}
#  ${Boost_LIBRARIES}
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
