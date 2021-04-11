cmake_minimum_required(VERSION 2.8)
project(franka-visual-servoing)

find_package(VISP REQUIRED)

if(UNIX AND NOT APPLE) 
    set(COPPELIA_DIR "/home/stephane/Downloads/CoppeliaSim_Edu_V4_0_0_Ubuntu18_04")
    set(BZERO_LIB "${COPPELIA_DIR}/libb0.so")
endif()
if(WIN32)
    set(BOOST_DIR "C:/Users/Stephane/Downloads/boost_1_72_0/boost_1_72_0")
    set(COPPELIA_DIR "C:/Program Files/CoppeliaRobotics/CoppeliaSimEdu")
    set(BZERO_LIB "${COPPELIA_DIR}/b0.lib")
endif()

set(MSG_PACK_DIR "${COPPELIA_DIR}/programming/b0RemoteApiBindings/cpp/msgpack-c/include/")
set(BINDINGS_DIR "${COPPELIA_DIR}/programming/bluezero/include/b0/bindings/")

if(UNIX AND NOT APPLE)
    include_directories(${VISP_INCLUDE_DIRS} ${MSG_PACK_DIR} ${BINDINGS_DIR})
endif()
if(WIN32)
    include_directories(${VISP_INCLUDE_DIRS} ${MSG_PACK_DIR} ${BINDINGS_DIR} ${BOOST_DIR})
endif()


add_executable(panda_hybrid_servo src/panda_hybrid_servo.cpp src/b0RemoteApi.cpp)
target_link_libraries(panda_hybrid_servo ${VISP_LIBRARIES} ${BZERO_LIB})

