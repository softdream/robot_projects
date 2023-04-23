CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT(lidar_test)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

ADD_DEFINITIONS(-std=c++17)

# include directories
INCLUDE_DIRECTORIES(
	${CMAKE_SOURCE_DIR}
	${CMAKE_SOURCE_DIR}/include
	${CMAKE_SOURCE_DIR}/include/lidar_include
	${CMAKE_SOURCE_DIR}/src
)

# link directories
LINK_DIRECTORIES(${CMAKE_SOURCE_DIR}/lib)

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})

ADD_EXECUTABLE(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/src/main.cpp)

# link 
TARGET_LINK_LIBRARIES(${PROJECT_NAME} libcspclidar_driver.a pthread)