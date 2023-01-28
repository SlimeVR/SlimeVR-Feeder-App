cmake_minimum_required(VERSION 3.15)

set(_build_version "unknown")

find_package(Git)
if (GIT_FOUND)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} describe --tags
        WORKING_DIRECTORY "${local_dir}"
        OUTPUT_VARIABLE _build_version
        ERROR_QUIET
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    message(STATUS "version in git: ${_build_version}")
else()
    message(STATUS "git not found")
endif()

message(STATUS "copying ${local_dir}/cmake/version.h.in to ${output_dir}/version.h")
configure_file("${local_dir}/cmake/version.h.in" "${output_dir}/version.h" @ONLY)