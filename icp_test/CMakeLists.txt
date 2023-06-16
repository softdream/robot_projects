CMAKE_MINIMUM_REQUIRED( VERSION 3.16 )

# Set a default build type if none was specified
IF(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
        MESSAGE("Setting build type to 'RelWithDebInfo' as none was specified.")
        SET(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Choose the type of build." FORCE)
        # Set the possible values of build type for cmake-gui
        SET_PROPERTY(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
ENDIF()


# project name
PROJECT( icp_test )

ADD_DEFINITIONS( -std=c++14 )

#find required eigen
FIND_PACKAGE( Eigen3 )

#directory of eigen headers
INCLUDE_DIRECTORIES( ${EIGEN3_INCLUDE_DIR} )

#find required opencv
FIND_PACKAGE( OpenCV REQUIRED )

#directory of opencv headers
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

AUX_SOURCE_DIRECTORY( ${PROJECT_SOURCE_DIR}/src src_file )
AUX_SOURCE_DIRECTORY( ${PROJECT_SOURCE_DIR} src_file )

INCLUDE_DIRECTORIES( "${PROJECT_SOURCE_DIR}/include" )

#directory of opencv link libs
LINK_DIRECTORIES(${OpenCV_LIBRARY_DIRS})

ADD_EXECUTABLE( icp_test ${src_file} )

TARGET_LINK_LIBRARIES( icp_test ${OpenCV_LIBS} pthread )

