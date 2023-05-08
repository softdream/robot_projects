CMAKE_MINIMUM_REQUIRED( VERSION 3.16 )

# Set a default build type if none was specified
IF(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
        MESSAGE("Setting build type to 'RelWithDebInfo' as none was specified.")
        SET(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Choose the type of build." FORCE)
        # Set the possible values of build type for cmake-gui
        SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
ENDIF()

# project name
PROJECT( robot_slam )

#
ADD_DEFINITIONS( -std=c++17 )

# find required eigen
FIND_PACKAGE( Eigen3 )

# directory of eigen headers
INCLUDE_DIRECTORIES( ${EIGEN3_INCLUDE_DIR} )

# find required opencv
FIND_PACKAGE( OpenCV REQUIRED )

# directory of opencv headers
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

# directory of opencv link libs
LINK_DIRECTORIES(${OpenCV_LIBRARY_DIRS})

# headers of this project
INCLUDE_DIRECTORIES(
	${CMAKE_SOURCE_DIR}/include	
	${CMAKE_SOURCE_DIR}/include/common
	${CMAKE_SOURCE_DIR}/include/lidar/lidar_include
	${CMAKE_SOURCE_DIR}/include/lidar
	${CMAKE_SOURCE_DIR}/include/odom
	${CMAKE_SOURCE_DIR}/include/slam
	${CMAKE_SOURCE_DIR}/include/device
	${CMAKE_SOURCE_DIR}/src
)

# link directory of this project
LINK_DIRECTORIES(${CMAKE_SOURCE_DIR}/lib)

# add source files
AUX_SOURCE_DIRECTORY(${CMAKE_SOURCE_DIR}/src src_files)
AUX_SOURCE_DIRECTORY(${CMAKE_SOURCE_DIR}/src/odom src_files)
AUX_SOURCE_DIRECTORY(${CMAKE_SOURCE_DIR} src_files)

# generate executable file
ADD_EXECUTABLE( ${PROJECT_NAME} ${src_files} )

# link 
TARGET_LINK_LIBRARIES(${PROJECT_NAME} libcspclidar_driver.a pthread)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OpenCV_LIBS})

