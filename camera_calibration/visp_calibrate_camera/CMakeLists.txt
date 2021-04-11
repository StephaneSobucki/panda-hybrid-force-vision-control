cmake_minimum_required(VERSION 2.8)
project(franka-visual-servoing)

find_package(VISP REQUIRED)

if(UNIX AND NOT APPLE)
    include_directories(${VISP_INCLUDE_DIRS}) 
endif()
if(WIN32)
    include_directories(${VISP_INCLUDE_DIRS} ${MSG_PACK_DIR} ${BINDINGS_DIR} ${BOOST_DIR})
endif()


add_executable(calibrate-camera src/calibrate-camera.cpp) 
target_link_libraries(calibrate-camera ${VISP_LIBRARIES})

