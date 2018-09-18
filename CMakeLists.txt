cmake_minimum_required(VERSION 3.5)

find_package(Boost 1.45.0 COMPONENTS thread) 

if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS}) 
    add_executable(arduino_ifce src/arduino_interface.cpp) 
    target_link_libraries(arduino_ifce ${Boost_LIBRARIES})
endif()
