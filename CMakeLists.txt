cmake_minimum_required(VERSION 3.15)
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/vcpkg/scripts/buildsystems/vcpkg.cmake")
project(SlimeVR-Feeder-App LANGUAGES CXX VERSION 0.1.3)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

find_package(fmt CONFIG REQUIRED)
find_library(OPENVR_LIB openvr)
# TODO: find_file DLL on windows, or fallback to LIB/

message("openvr_root: ${OPENVR_LIB}")

# Project
add_executable("${PROJECT_NAME}" "src/main.cpp" "src/pathtools_excerpt.cpp" "src/pathtools_excerpt.h" "src/matrix_utils.cpp" "src/matrix_utils.h")
#target_include_directories("${PROJECT_NAME}" PUBLIC "${OPENVR_INCLUDE_DIR}")
#target_include_directories("${PROJECT_NAME}" PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src/")
target_link_libraries("${PROJECT_NAME}" PRIVATE "${OPENVR_LIB}" fmt::fmt)
target_compile_features("${PROJECT_NAME}" PRIVATE cxx_std_17)

# IDE Config
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/src" PREFIX "Header Files" FILES ${HEADERS})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/src" PREFIX "Source Files" FILES ${SOURCES})

#happens post-build
#add_custom_command(TARGET "${PROJECT_NAME}" POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different "${OPENVR_DLL}" $<TARGET_FILE_DIR:${PROJECT_NAME}>)
# happens during confiuration
#file(COPY bindings manifest.vrmanifest "${OPENVR_DLL}" DESTINATION .)

install(DIRECTORY "${PROJECT_SOURCE_DIR}/bindings"
    DESTINATION "."
)
#install(FILES "${OPENVR_DLL}" "${PROJECT_SOURCE_DIR}/manifest.vrmanifest"
#    DESTINATION "."
#)

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
