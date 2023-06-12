CMAKE_MINIMUM_REQUIRED( VERSION 3.16 )

# project name
PROJECT(rtmp_test)

#
ADD_DEFINITIONS( -std=c++17 )

# find required opencv
FIND_PACKAGE( OpenCV REQUIRED )

# directory of opencv headers
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

# directory of opencv link libs
LINK_DIRECTORIES(${OpenCV_LIBRARY_DIRS})

INCLUDE_DIRECTORIES(
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_SOURCE_DIR}
)

AUX_SOURCE_DIRECTORY(${CMAKE_SOURCE_DIR} src_files)

AUX_SOURCE_DIRECTORY(${CMAKE_SOURCE_DIR}/src src_files)

ADD_EXECUTABLE( ${PROJECT_NAME} ${src_files} )

TARGET_LINK_LIBRARIES(${PROJECT_NAME} rtmp gmp ssl crypto z dap nettle gnutls )
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OpenCV_LIBS})
