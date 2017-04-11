cmake_minimum_required(VERSION 2.8.8)

#set(FX3_CORE_ASM_SRC ${CMAKE_SOURCE_DIR}/fx3/cyfx_gcc_startup.S)
#set(FX3_CORE_CXX_SRC ${CMAKE_SOURCE_DIR}/fx3/cyfxtx.cpp ${CMAKE_SOURCE_DIR}/fx3/cyfxcppsyscall.cpp)
#set(FX3_LD_SCRIPT ${CMAKE_SOURCE_DIR}/fx3/fx3cpp.ld)
include(${CMAKE_SOURCE_DIR}/cmake/fx3.cmake)

project(FreeSRP)

fx3_enable_cxx()
fx3_enable_stdcxx_libs()

set(COMMON_FLAGS "-fmessage-length=0 -ffunction-sections -fdata-sections -fshort-wchar -Wall -Wextra -Wno-unused-parameter")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-exceptions -fno-rtti -fno-unwind-tables -std=c++11 ${COMMON_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${COMMON_FLAGS} -Wwrite-strings -std=c99")

file(GLOB_RECURSE FREESRP_SRC_FILES
    ${PROJECT_SOURCE_DIR}/src/*.h
    ${PROJECT_SOURCE_DIR}/src/*.c
    ${PROJECT_SOURCE_DIR}/src/*.cpp
    ${PROJECT_SOURCE_DIR}/src/ad9364/*.h
    ${PROJECT_SOURCE_DIR}/src/ad9364/*.c
    ${PROJECT_SOURCE_DIR}/src/ad9364/*.cpp
    ${PROJECT_SOURCE_DIR}/src/ad9364/ad9364/*.h
    ${PROJECT_SOURCE_DIR}/src/ad9364/ad9364/*.c
    ${PROJECT_SOURCE_DIR}/src/ad9364/ad9364/*.cpp
)

fx3_add_target(FreeSRP
    ${FREESRP_SRC_FILES}
)
