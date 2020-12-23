cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set the project name
project(SLAMSim)

include_directories(
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/src
        ${PROJECT_SOURCE_DIR}/lib

)

# IF WINDOWS LINK TO SFML THROUGH STATIC LIBRARY
if (WIN32)
        include_directories(${PROJECT_SOURCE_DIR}/SFML/include)
        link_directories(${PROJECT_SOURCE_DIR}/SFML/lib)
endif (WIN32)

file(GLOB all_SRCS
        "${PROJECT_SOURCE_DIR}/include/*.h"
        "${PROJECT_SOURCE_DIR}/include/*.hpp"
        "${PROJECT_SOURCE_DIR}/src/*.cpp"
        "${PROJECT_SOURCE_DIR}/src/*.c"
        )

add_executable(SLAMSim ${all_SRCS})

if (UNIX)
        if (APPLE)
                file(GLOB LIBRARIES "${PROJECT_SOURCE_DIR}/lib/SFML/*.dylib")
                target_link_libraries(SLAMSim ${LIBRARIES})
        else ()
                file(GLOB LIBRARIES "${PROJECT_SOURCE_DIR}/SFML/lib/*.lib")
                target_link_libraries(SLAMSim ${LIBRARIES})
        endif ()
endif (UNIX)