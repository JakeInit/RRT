cmake_minimum_required(VERSION 3.0)
set(RRT_VERSION "0.0.1")
project(RRT)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
message(STATUS "cmake module path = " ${CMAKE_MODULE_PATH})

# Create the folders for the output files and set paths
file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Include all source files
file(GLOB PROJECT_SRC   "src/*.cpp" "system/src/*.cpp")

# Set the cmake build type and print the type
set(CMAKE_BUILD_TYPE Release)
message(STATUS "CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message(STATUS "Architecture found as " ${ARCHITECTURE})

message(STATUS "Project Source Directory = " ${PROJECT_SOURCE_DIR})

#----------------------------------------------------------------------
#-----------------Find Required packages-------------------------------
#----------------------------------------------------------------------
find_package (Eigen3 REQUIRED NO_MODULE)
find_package(Threads)
find_package(Boost REQUIRED system filesystem thread locale date_time)
find_package(fcl REQUIRED)

#----------------------------------------------------------------------
#----Include Directories-----------------------------------------------
#----------------------------------------------------------------------
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/system/include)
include_directories(/usr/include/glib-2.0/)
include_directories(/usr/lib/${ARCHITECTURE}-linux-gnu/glib-2.0/include/)
include_directories (/usr/local/include)
include_directories(${EIGEN3_INCLUDE_DIR})

message(STATUS "eigen3 directory = " ${EIGEN3_INCLUDE_DIR})
message(STATUS "Boost directory = " ${Boost_INCLUDE_DIRS})
message(STATUS "FCL directory = " ${FCL_INCLUDE_DIR})

#----------------------------------------------------------------------
#------Link Libraries--------------------------------------------------
#----------------------------------------------------------------------
link_directories(${PROJECT_SOURCE_DIR}/lib)

#----------------------------------------------------------------------
#----add and link libraries + definitions------------------------------
#----------------------------------------------------------------------
add_library(localRRT STATIC ${PROJECT_SRC})
target_compile_definitions(localRRT PRIVATE _TURN_OFF_PLATFORM_STRING)
target_compile_definitions(localRRT PRIVATE PROJECTDIR="${PROJECT_SOURCE_DIR}/")
target_link_libraries(localRRT -lm boost_system boost_filesystem boost_thread boost_locale boost_date_time jsoncpp ccd fcl)
set(LINK_LIBRARIES localRRT pthread)

#----------------------------------------------------------------------
#----Check if running in windows environment---------------------------
#----------------------------------------------------------------------
if(WIN32)
    add_definitions( "/wd4100 /wd4800 /wd4244 /wd4251 /wd4127 /wd4250 /wd4482 /MP /D_CRT_SECURE_NO_WARNINGS /DWIN32" )
else(WIN32)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -Dlinux")
    if(COVERAGE)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage")
        message(STATUS "Building with Code Coverage Metrics")
    endif(COVERAGE)
endif(WIN32)

#----------------------------------------------------------------------
#-----Add main executable file for runtime-----------------------------
#----------------------------------------------------------------------
add_executable(rrtAplication src/apps/main.cpp)
target_link_libraries(rrtAplication PRIVATE ${LINK_LIBRARIES})
