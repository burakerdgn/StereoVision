cmake_minimum_required( VERSION 3.6 )

# Require C++11 (or later)
set( CMAKE_CXX_STANDARD 11 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )
set( CMAKE_CXX_EXTENSIONS OFF )

# Create Project
project( Sample )
add_executable( demo_realsense realsense.h realsense.cpp vdisp.h vdisp.cpp realss.cpp )
add_executable( demo_basic vdisp.h vdisp.cpp imageread.cpp )
add_executable( demo_zed vdisp.h vdisp.cpp zed.cpp )
#add_executable( demo1 realsense.h realsense.cpp vdisp.h vdisp.cpp main.cpp )

# Set StartUp Project
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "demo" )

# Find Package
# librealsense2
set( realsense2_DIR "C:/Program Files/librealsense2/lib/cmake/realsense2" CACHE PATH "Path to librealsense2 config directory." )
find_package( realsense2 REQUIRED )

#doxygen
option(BUILD_DOC "Build documentation" ON)
find_package(Doxygen)
if (DOXYGEN_FOUND)
    # set input and output files
    set(DOXYGEN_IN /media/nvidia/32B270B0B2707A65/burak/stereo/project-library/docs/Doxyfile.in)
    #set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/../../docs/doxy_demo)
    set(DOXYGEN_OUT /media/nvidia/32B270B0B2707A65/burak/stereo/project-library/docs/doxy_demo)

    # request to configure the file
    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
    message("Doxygen build started")

    # note the option ALL which allows to build the docs together with the application
    add_custom_target( doc_doxygen ALL
        COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM )
else (DOXYGEN_FOUND)
  message("Doxygen need to be installed to generate the doxygen documentation")
endif (DOXYGEN_FOUND)
#

#added for zed
find_package(ZED 2 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})
TARGET_LINK_LIBRARIES(demo_zed
                       ${ZED_LIBRARIES}
                       ${SPECIAL_OS_LIBS}
                        ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_NPP_LIBRARIES_ZED} )
#

# For RealSense SDK v2.16.4 and previous
if(NOT realsense2_INCLUDE_DIR)
  set(realsense2_INCLUDE_DIR ${realsense_INCLUDE_DIR})
endif()

# OpenCV
set( OpenCV_DIR "C:/Program Files/opencv/build" CACHE PATH "Path to OpenCV config directory." )
find_package( OpenCV REQUIRED )

if( realsense2_FOUND AND OpenCV_FOUND )
  # Additional Include Directories
  include_directories( ${realsense2_INCLUDE_DIR} )
  include_directories( ${OpenCV_INCLUDE_DIRS} )

  # Additional Dependencies
  target_link_libraries( demo_realsense ${realsense2_LIBRARY} )
  target_link_libraries( demo_realsense ${OpenCV_LIBS} )
  target_link_libraries( demo_zed ${OpenCV_LIBS} )
  target_link_libraries( demo_basic ${OpenCV_LIBS} )
  #target_link_libraries( demo_zed ${OpenCV_LIBS} )
endif()
