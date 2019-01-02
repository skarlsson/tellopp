cmake_minimum_required(VERSION 2.8.11)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

option(BUILD_STATIC_LIBS "enable static linking support" ON)
option(BUILD_SHARED_LIBS "enable static linking support" OFF)
option(BUILD_TESTS "build tests" ON)
option(LINK_SHARED "link shared" OFF)

set(KSPP_VERSION_MAJOR 1)
set(KSPP_VERSION_MINOR 0)

set(CMAKE_CXX_STANDARD 14)

#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wpedantic -O2")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wpedantic -g -O3")
#/set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")

find_package(Boost COMPONENTS program_options filesystem system REQUIRED)
set(BOOST_LIBS
        ${Boost_PROGRAM_OPTIONS_LIBRARY}
        ${Boost_FILESYSTEM_LIBRARY}
        ${Boost_SYSTEM_LIBRARY}
        pthread
        rt
        c
        )

SET(CSI_INCLUDE_PATH ${Boost_INCLUDE_DIR})

SET(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)
SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)
add_definitions(-g -D_FILE_OFFSET_BITS=64 -D_REENTRANT -DEXTERNAL_LOCKS -DMULTITHREAD)
add_definitions(-fPIC)
add_definitions(-Wno-deprecated)

SET(DLIB_LIBS dlib cblas lapack X11)

SET(OPENCV_LIBS opencv_core opencv_highgui opencv_imgproc opencv_videoio opencv_imgcodecs)

SET(EXT_LIBS_SHARED ${DLIB_LIBS} ${OPENCV_LIBS} ${BOOST_LIBS} glog crypto ssl)
SET(EXT_LIBS_STATIC ${DLIB_LIBS} ${OPENCV_LIBS} ${BOOST_LIBS} glog crypto ssl)

SET(CSI_LIBS_SHARED tellopp ${EXT_LIBS_SHARED})
SET(CSI_LIBS_STATIC tellopp_s ${EXT_LIBS_STATIC})

SET(MKL_INCLUDE_PATH /opt/intel/mkl/include)
SET(MKL_LIB_PATH /opt/intel/lib)

include_directories(${CSI_INCLUDE_PATH} ${MKL_INCLUDE_PATH} ${CMAKE_SOURCE_DIR}/include)
link_directories(${CSI_LIBRARY_PATH} ${MKL_LIB_PATH})

#TARGETS

file(GLOB LIB_SRCS
        src/*.cpp
    )

file(GLOB LIB_HEADERS
        include/tellopp/*.h
       )

if (BUILD_STATIC_LIBS)
    ADD_LIBRARY(tellopp_s STATIC ${LIB_SRCS} ${LIB_HEADERS})
    INSTALL(TARGETS tellopp_s
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib
            )
endif ()

if (BUILD_SHARED_LIBS)
    ADD_LIBRARY(tellopp SHARED ${LIB_SRCS} ${LIB_HEADERS})
    INSTALL(TARGETS tellopp
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib
            )
endif ()

INSTALL(DIRECTORY ${CMAKE_SOURCE_DIR}/include/ DESTINATION include)

add_subdirectory(examples)



