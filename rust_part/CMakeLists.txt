
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CARGO_CMD cargo build --verbose)
    set(TARGET_DIR "debug")
else ()
    set(CARGO_CMD cargo build --release --verbose)
    set(TARGET_DIR "release")
endif ()

# TODO: this would be really nice to enable, but is not really setup everywhere else
# if(ENABLE_LTO)
#     set(RUST_FLAGS "-Clinker-plugin-lto" "-Clinker=clang-13" "-Clink-arg=-fuse-ld=lld-13")
# endif()

if (MSVC)
    set(RUST_PART_LIB "${CMAKE_CURRENT_BINARY_DIR}/${TARGET_DIR}/rust_part.lib")
elseif (UNIX)
    set(RUST_PART_LIB "${CMAKE_CURRENT_BINARY_DIR}/${TARGET_DIR}/librust_part.a")
else ()
    message(FATAL_ERROR "It's not unix and not windows, ¯\\_(ツ)_/¯")
endif ()

set(RUST_PART_CXX "${CMAKE_CURRENT_BINARY_DIR}/rust_part.cpp")
add_library(rust_part STATIC ${RUST_PART_CXX})
add_custom_command(
    OUTPUT ${RUST_PART_CXX} ${RUST_PART_LIB}
    COMMAND ${CMAKE_COMMAND} -E env CARGO_TARGET_DIR=${CMAKE_CURRENT_BINARY_DIR} RUSTFLAGS="${RUST_FLAGS}" ${CARGO_CMD}
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/cxxbridge/rust_part/src/lib.rs.cc ${RUST_PART_CXX}
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/cxxbridge/rust_part/src/lib.rs.h ${CMAKE_CURRENT_BINARY_DIR}/rust_part.h
    DEPENDS src/lib.rs
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(rust_part ${RUST_PART_LIB})
# if (MSVC)
#   target_link_libraries(rust_part ws2_32 userenv advapi32 shell32)
#   if (CMAKE_BUILD_TYPE STREQUAL "Release")
#     target_link_libraries(rust_part msvcrt)
#   else ()
#     target_link_libraries(rust_part msvcrtd)
#   endif()
# else
if (UNIX)
  target_link_libraries(rust_part pthread m dl)
endif ()

add_test(NAME rust_part_test 
    COMMAND cargo test
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})