CMAKE_MINIMUM_REQUIRED(VERSION 3.16)

# Set a default build type if none was specified
IF(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
        MESSAGE("Setting build type to 'RelWithDebInfo' as none was specified.")
        SET(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Choose the type of build." FORCE)
        # Set the possible values of build type for cmake-gui
        SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
ENDIF()


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

# eigen
FIND_PACKAGE( Eigen3 )

# directory of eigen
INCLUDE_DIRECTORIES( ${EIGEN3_INCLUDE_DIR} )

# opencv
FIND_PACKAGE( OpenCV REQUIRED )

# directory of opencv
INCLUDE_DIRECTORIES( ${OpenCV_INCLUDE_DIRS} )

# opencv link directories
LINK_DIRECTORIES( ${OpenCV_LIBRARIES_DIRS} )

# link directories
LINK_DIRECTORIES(${CMAKE_SOURCE_DIR}/lib)

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})

ADD_EXECUTABLE(${PROJECT_NAME} ${CMAKE_SOURCE_DIR}/src/main.cpp)

# link 
TARGET_LINK_LIBRARIES(${PROJECT_NAME} libcspclidar_driver.a pthread)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OpenCV_LIBS})
