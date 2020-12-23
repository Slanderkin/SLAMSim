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

# IF UNIX LINK TO SFML THROUGH DYNAMIC LIBRARY
if (UNIX)
	find_package(SFML 2 REQUIRED network audio graphics window system)
	if (SFML_FOUND)
	 include_directories(${SFML_INCLUDE_DIR})
	  target_link_libraries(SLAMSim sfml-graphics sfml-window sfml-system)
	endif (SFML_FOUND)
endif (UNIX)