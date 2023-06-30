CMAKE_MINIMUM_REQUIRED(VERSION 3.16.1)

PROJECT(body_detect)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
        SET(CMAKE_CXX_STANDARD 14)
endif()

ADD_COMPILE_OPTIONS(-Wall -Wextra -Wpedantic)

INCLUDE_DIRECTORIES(include include/util/)

INCLUDE_DIRECTORIES(include ${PROJECT_SOURCE_DIR})

SET(SYS_ROOT ${CMAKE_SYSROOT})
MESSAGE("SYS_ROOT : " ${SYS_ROOT})

LINK_DIRECTORIES(${SYS_ROOT}/usr/lib/hbbpu/)

#find required opencv
FIND_PACKAGE( OpenCV REQUIRED )

#directory of opencv headers
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src/ src_file)

#directory of opencv link libs
LINK_DIRECTORIES(${OpenCV_LIBRARY_DIRS})

LINK_DIRECTORIES(${PROJECT_SOURCE_DIR}/lib/)

ADD_EXECUTABLE( body_detect ${src_file} )

TARGET_LINK_LIBRARIES( body_detect ${OpenCV_LIBS} )

TARGET_LINK_LIBRARIES( body_detect libdnn_node.so )

TARGET_LINK_LIBRARIES( body_detect 
	alog
        hlog
        easy_dnn
        dnn
        cnn_intf
        ion
        hbrt_bernoulli_aarch64 )
