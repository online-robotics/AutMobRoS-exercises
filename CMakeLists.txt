cmake_minimum_required(VERSION 3.5.1)
project(template_project)   # MODIFY Project name

set(CMAKE_CXX_STANDARD 14)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})

find_package(EEROS REQUIRED)

include_directories(${EEROS_INCLUDE_DIR})
link_directories(${EEROS_LIB_DIR})

include_directories(
    ${PROJECT_SOURCE_DIR}/inc
    ${PROJECT_SOURCE_DIR}/src
)

file(GLOB all_SRCS
        "${PROJECT_SOURCE_DIR}/inc/*.h"
		"${PROJECT_SOURCE_DIR}/inc/*.hpp"
        "${PROJECT_SOURCE_DIR}/inc/*/*.h"
		"${PROJECT_SOURCE_DIR}/inc/*/*.hpp"
        "${PROJECT_SOURCE_DIR}/src/*.cpp"
		"${PROJECT_SOURCE_DIR}/src/*.c"
        )

add_custom_command(OUTPUT copyHalConfig POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy "${PROJECT_SOURCE_DIR}/config/*.json" "${CMAKE_BINARY_DIR}")

add_executable(${PROJECT_NAME} ${all_SRCS} copyHalConfig)
target_link_libraries(${PROJECT_NAME} eeros ucl ${CMAKE_DL_LIBS})
