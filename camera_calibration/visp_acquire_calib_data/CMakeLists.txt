cmake_minimum_required(VERSION 2.8)
project(franka-visual-servoing)

find_package(VISP REQUIRED)

if(UNIX AND NOT APPLE)
    include_directories(${VISP_INCLUDE_DIRS}) 
endif()
if(WIN32)
    include_directories(${VISP_INCLUDE_DIRS} ${MSG_PACK_DIR} ${BINDINGS_DIR} ${BOOST_DIR})
endif()


add_executable(acquire_calib_data src/acquire_calib_data.cpp) 
target_link_libraries(acquire_calib_data ${VISP_LIBRARIES})

