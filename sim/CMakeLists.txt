cmake_minimum_required(VERSION 3.16)
project(flightsim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenMP QUIET)

set(SIM_DIR ${CMAKE_SOURCE_DIR})
set(QUICKPID_DIR ${CMAKE_SOURCE_DIR}/../.pio/libdeps/esp32-s3-devkitc-1/QuickPID/src)

add_library(flightsim SHARED
    ${SIM_DIR}/sim_flight_control.cpp
    ${SIM_DIR}/arduino_stubs.cpp
    ${QUICKPID_DIR}/QuickPID.cpp
)

target_include_directories(flightsim PUBLIC
    ${SIM_DIR}
    ${QUICKPID_DIR}
)

target_compile_definitions(flightsim PRIVATE
    ARDUINO=100
    SIM_BUILD
)

target_compile_options(flightsim PRIVATE
    -O2 -Wall -Wno-unused-parameter -Wno-unused-variable
    $<$<BOOL:${OpenMP_CXX_FOUND}>:-fopenmp>
)

if(OpenMP_CXX_FOUND)
    target_link_libraries(flightsim PUBLIC OpenMP::OpenMP_CXX)
endif()

install(TARGETS flightsim DESTINATION ${SIM_DIR})
