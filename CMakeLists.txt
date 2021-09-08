cmake_minimum_required(VERSION 3.0.0)
project(SlimeVR-Feeder-App VERSION 0.1.0)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

include(CTest)
enable_testing()

# OpenVR
set(OPENVR_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/libraries/openvr/headers")

set(SIZEOF_VOIDP ${CMAKE_SIZEOF_VOID_P})
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(PROCESSOR_ARCH "64")
else()
    set(PROCESSOR_ARCH "32")
endif()

if(WIN32)
    set(PLATFORM_NAME "win")
elseif(UNIX AND NOT APPLE)
    set(PLATFORM_NAME "linux")
elseif(APPLE)
    set(PLATFORM_NAME "osx")
endif()

find_library(OPENVR_LIB openvr_api HINTS "${CMAKE_CURRENT_SOURCE_DIR}/libraries/openvr/lib/${PLATFORM_NAME}${PROCESSOR_ARCH}/" NO_DEFAULT_PATH )
find_file(OPENVR_DLL openvr_api.dll HINTS "${CMAKE_CURRENT_SOURCE_DIR}/libraries/openvr/bin/${PLATFORM_NAME}${PROCESSOR_ARCH}/" NO_DEFAULT_PATH )

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/libraries/fmt")
# Project
add_executable("${PROJECT_NAME}" "src/main.cpp" "src/pathtools_excerpt.cpp" "src/pathtools_excerpt.h" "src/matrix_utils.cpp" "src/matrix_utils.h")
target_include_directories("${PROJECT_NAME}" PUBLIC "${OPENVR_INCLUDE_DIR}")
target_include_directories("${PROJECT_NAME}" PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src/")
target_link_libraries("${PROJECT_NAME}" PUBLIC "${OPENVR_LIB}" fmt::fmt)
set_property(TARGET "${PROJECT_NAME}" PROPERTY CXX_STANDARD 17)

# IDE Config
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/src" PREFIX "Header Files" FILES ${HEADERS})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/src" PREFIX "Source Files" FILES ${SOURCES})

add_custom_command(TARGET "${PROJECT_NAME}" POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OPENVR_DLL}" $<TARGET_FILE_DIR:${PROJECT_NAME}>)

install(DIRECTORY "${PROJECT_SOURCE_DIR}/bindings"
    DESTINATION "."
)
install(FILES "${OPENVR_DLL}" "${PROJECT_SOURCE_DIR}/manifest.vrmanifest"
    DESTINATION "."
)

install(TARGETS "${PROJECT_NAME}"
    RUNTIME
    DESTINATION .
)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
set(CPACK_GENERATOR "ZIP")
set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY ON)
set(CPACK_VERBATIM_VARIABLES YES)
include(CPack)
