cmake_minimum_required(VERSION 3.0)
PROJECT(indi_flirptu CXX C)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake_modules/")

include(GNUInstallDirs)

set(INDI_FLIRPTU_VERSION_MAJOR 0)
set(INDI_FLIRPTU_VERSION_MINOR 1)

find_package(INDI REQUIRED)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/config.h )
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/indi_flirptu.xml.cmake ${CMAKE_CURRENT_BINARY_DIR}/indi_flirptu.xml )

include_directories( ${CMAKE_CURRENT_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_SOURCE_DIR})
include_directories( ${INDI_INCLUDE_DIR})

include(CMakeCommon)

########### FLIR PTU ###########
set(indi_flirptu_SRCS
   ${CMAKE_CURRENT_SOURCE_DIR}/flirptu.cpp
   )

add_executable(indi_flirptu ${indi_flirptu_SRCS})

target_link_libraries(indi_flirptu ${INDI_LIBRARIES} )

install(TARGETS indi_flirptu RUNTIME DESTINATION bin )

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/indi_flirptu.xml DESTINATION ${INDI_DATA_DIR})
