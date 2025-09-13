cmake_minimum_required(VERSION 3.10)
project(ArenaCameraTest LANGUAGES CXX)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Arena SDK 路径
set(ARENA_LIBRARY_DIR "${CMAKE_SOURCE_DIR}/3rdparty/camera_arena/lucid/lib64")
set(ARENA_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/3rdparty/camera_arena/lucid/include")

# OpenCV SDK 路径
set(OpenCV_DIR "${CMAKE_SOURCE_DIR}/3rdparty/opencv/lib/cmake/opencv4")
find_package(OpenCV REQUIRED)
#message(STATUS "OpenCV_DIR: ${OpenCV_DIR}")
#message(STATUS "OpenCV_LIBRARIES: ${OpenCV_LIBRARIES}")

# VTK SDK 路径
set(VTK_DIR "${CMAKE_SOURCE_DIR}/3rdparty/vtk/lib/cmake/vtk-8.2")
find_package(VTK REQUIRED)
#message(STATUS "VTK_DIR: ${VTK_DIR}")
#message(STATUS "VTK_LIBRARIES: ${VTK_LIBRARIES}")

# PCL SDK 路径
set(PCL_DIR "${CMAKE_SOURCE_DIR}/3rdparty/pcl/share/pcl-1.12")
find_package(PCL 1.12 REQUIRED COMPONENTS common io visualization)## 添加visualization
#message(STATUS "PCL_LIBRARIES: ${PCL_LIBRARIES}")

# GenICam 路径
set(GENICAM_LIBRARY_DIR "${CMAKE_SOURCE_DIR}/3rdparty/camera_arena/GenICam/lib")
set(GENICAM_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/3rdparty/camera_arena/GenICam/include")

# 查找头文件路径
include_directories(
    ${ARENA_INCLUDE_DIR}/Arena
    ${ARENA_INCLUDE_DIR}/ArenaC
    ${ARENA_INCLUDE_DIR}/GenTL
    ${ARENA_INCLUDE_DIR}/Save
    ${ARENA_INCLUDE_DIR}/SaveC
    ${GENICAM_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${VTK_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

# 查找库文件（Arena）
find_library(ARENA_LIB arena PATHS ${ARENA_LIBRARY_DIR})
find_library(ARENAC_LIB arenac PATHS ${ARENA_LIBRARY_DIR})
find_library(ARENACD_LIB arenacd PATHS ${ARENA_LIBRARY_DIR})
find_library(ARENAD_LIB arenad PATHS ${ARENA_LIBRARY_DIR})
find_library(GENTL_LIB gentl PATHS ${ARENA_LIBRARY_DIR})
find_library(GENTLD_LIB gentld PATHS ${ARENA_LIBRARY_DIR})
find_library(LUCIDLOG_LIB lucidlog PATHS ${ARENA_LIBRARY_DIR})
find_library(LUCIDLOGD_LIB lucidlogd PATHS ${ARENA_LIBRARY_DIR})
find_library(SAVE_LIB save PATHS ${ARENA_LIBRARY_DIR})
find_library(SAVEC_LIB savec PATHS ${ARENA_LIBRARY_DIR})
find_library(SAVECD_LIB savecd PATHS ${ARENA_LIBRARY_DIR})
find_library(SAVED_LIB saved PATHS ${ARENA_LIBRARY_DIR})

# 查找库文件（GenICam）
find_library(GCBASE_LIB GCBase_gcc54_v3_3_LUCID PATHS ${GENICAM_LIBRARY_DIR})
find_library(GENAPI_LIB GenApi_gcc54_v3_3_LUCID PATHS ${GENICAM_LIBRARY_DIR})
find_library(LOG4CPP_LIB log4cpp_gcc54_v3_3_LUCID PATHS ${GENICAM_LIBRARY_DIR})
find_library(LOG_LIB Log_gcc54_v3_3_LUCID PATHS ${GENICAM_LIBRARY_DIR})
find_library(MATHPARSER_LIB MathParser_gcc54_v3_3_LUCID PATHS ${GENICAM_LIBRARY_DIR})
find_library(NODEMAPDATA_LIB NodeMapData_gcc54_v3_3_LUCID PATHS ${GENICAM_LIBRARY_DIR})
find_library(XMLPARSER_LIB XmlParser_gcc54_v3_3_LUCID PATHS ${GENICAM_LIBRARY_DIR})

# 添加可执行文件
add_executable(${PROJECT_NAME} main.cpp)

# 链接库
target_link_libraries(${PROJECT_NAME} PRIVATE
        ${ARENA_LIB}
        ${ARENAC_LIB}
        ${ARENACD_LIB}
        ${ARENAD_LIB}
        ${GENTL_LIB}
        ${GENTLD_LIB}
        ${LUCIDLOG_LIB}
        ${LUCIDLOGD_LIB}
        ${SAVE_LIB}
        ${SAVEC_LIB}
        ${SAVECD_LIB}
        ${SAVED_LIB}
        ${GCBASE_LIB}
        ${GENAPI_LIB}
        ${LOG4CPP_LIB}
        ${LOG_LIB}
        ${MATHPARSER_LIB}
        ${NODEMAPDATA_LIB}
        ${XMLPARSER_LIB}
        ${OpenCV_LIBRARIES}
        ${VTK_LIBRARIES}
        ${PCL_LIBRARIES}
)

# 设置编译选项
target_compile_options(${PROJECT_NAME} PRIVATE -Wall)

# 设置运行时库路径（Linux）
set(CMAKE_INSTALL_RPATH "${ARENA_LIBRARY_DIR}")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# 安装目标（可选）
install(TARGETS ${PROJECT_NAME} DESTINATION bin)
