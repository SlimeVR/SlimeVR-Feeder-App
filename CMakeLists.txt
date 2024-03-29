cmake_minimum_required(VERSION 3.15)
#set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/vcpkg/scripts/buildsystems/vcpkg.cmake")

project(SlimeVR-Feeder-App LANGUAGES CXX)

include("${CMAKE_CURRENT_SOURCE_DIR}/vcpkg/scripts/buildsystems/vcpkg.cmake")

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

find_package(fmt CONFIG REQUIRED)
find_library(OPENVR_LIB openvr_api)
find_package(Protobuf CONFIG REQUIRED)
find_package(simdjson CONFIG REQUIRED)

set(protos_OUTPUT_DIR ${CMAKE_CURRENT_BINARY_DIR}/protos)
file(MAKE_DIRECTORY "${protos_OUTPUT_DIR}")

include_directories(include)

set(CMAKE_SKIP_BUILD_RPATH  FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH $ORIGIN)

# Project
add_executable("${PROJECT_NAME}" "src/main.cpp" "src/pathtools_excerpt.cpp" "src/pathtools_excerpt.h" "src/matrix_utils.cpp" "src/matrix_utils.h" "src/bridge.cpp" "src/bridge.hpp" "src/setup.cpp" "src/setup.hpp" "ProtobufMessages.proto")
target_link_libraries("${PROJECT_NAME}" PRIVATE "${OPENVR_LIB}" fmt::fmt protobuf::libprotobuf simdjson::simdjson)
protobuf_generate(TARGET "${PROJECT_NAME}" LANGUAGE cpp PROTOC_OUT_DIR ${protos_OUTPUT_DIR})
target_include_directories("${PROJECT_NAME}" PUBLIC ${protos_OUTPUT_DIR} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_compile_features("${PROJECT_NAME}" PRIVATE cxx_std_17)

# IDE Config
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/src" PREFIX "Header Files" FILES ${HEADERS})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/src" PREFIX "Source Files" FILES ${SOURCES})

add_custom_target(
    version
    COMMAND ${CMAKE_COMMAND}
        -Dlocal_dir="${CMAKE_CURRENT_SOURCE_DIR}"
        -Doutput_dir="${CMAKE_CURRENT_BINARY_DIR}"
        -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/gitversion.cmake"
)
add_dependencies("${PROJECT_NAME}" version)

install(DIRECTORY "${PROJECT_SOURCE_DIR}/bindings"
    DESTINATION "."
)
install(FILES "${PROJECT_SOURCE_DIR}/manifest.vrmanifest"
    DESTINATION "."
)

if (WIN32)
    set(VCPKG_BINARIES "${CMAKE_BINARY_DIR}/vcpkg_installed/x64-windows/bin")
    install(FILES "${VCPKG_BINARIES}/openvr_api.dll" "${VCPKG_BINARIES}/fmt.dll" "${VCPKG_BINARIES}/libprotobuf.dll" "${VCPKG_BINARIES}/simdjson.dll" DESTINATION ".")
elseif(UNIX)
    # TODO: MacOS
    set(VCPKG_BINARIES "${CMAKE_BINARY_DIR}/vcpkg_installed/x64-linux/bin")
    install(FILES "${VCPKG_BINARIES}/libopenvr_api.so" DESTINATION ".")
endif()

install(TARGETS "${PROJECT_NAME}"
    RUNTIME
    DESTINATION .
)

set(CPACK_SYSTEM_NAME ${CMAKE_SYSTEM_NAME})
if(${CPACK_SYSTEM_NAME} MATCHES Windows)
  if(CMAKE_CL_64)
    set(CPACK_SYSTEM_NAME win64)
    set(CPACK_IFW_TARGET_DIRECTORY "@RootDir@/Program Files/${CMAKE_PROJECT_NAME}")
  else()
    set(CPACK_SYSTEM_NAME win32)
  endif()
endif()

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PACKAGE_FILE_NAME ${CPACK_PROJECT_NAME}-${CPACK_SYSTEM_NAME})
set(CPACK_GENERATOR "ZIP")
set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY ON)
set(CPACK_VERBATIM_VARIABLES YES)
include(CPack)
