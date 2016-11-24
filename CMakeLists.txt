cmake_minimum_required(VERSION 2.8.3)
project(asr_mild_base_laserscanner)


find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
	tf
	nav_msgs
)


catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp std_msgs tf nav_msgs
)


include_directories(
	include 
	${catkin_INCLUDE_DIRS}
)

add_executable(asr_mild_base_laserscanner 
	src/SickPLS.cc 
	src/SickPLSMessage.cc
	src/SickPLSBufferMonitor.cc
	src/sick.cpp

)

target_link_libraries(asr_mild_base_laserscanner   ${catkin_LIBRARIES}
)
