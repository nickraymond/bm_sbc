cmake_minimum_required(VERSION 3.18)
project(bm_sbc LANGUAGES C CXX)

# Export compile_commands.json for IDE / clangd integration. Must be set
# before any targets are defined so it covers bm_sbc_core, the apps, and
# tests (not just bm_core, which sets it in its own subdirectory scope).
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# Options
# ---------------------------------------------------------------------------
set(APP "" CACHE STRING "Name of a single app under apps/ to build")
option(BUILD_ALL_APPS "Build every app found under apps/" OFF)

# ---------------------------------------------------------------------------
# bm_core integration
# ---------------------------------------------------------------------------
# The submodule lives at lib/bm_core.  Its CMake helpers expose
# setup_bm_ip_stack() and setup_bm_os() after we include the entry-point
# file.  Until the submodule is populated these calls are stubbed out so the
# skeleton still configures cleanly.

set(BM_CORE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/bm_core)

include(${BM_CORE_DIR}/cmake/bm_core.cmake)

# Provide include paths that the LINUX / POSIX backends need.
# bm_core expects the integrator to supply bm_config.h (lives in src/core/).
set(BM_SBC_PLATFORM_INCLUDES
  ${CMAKE_CURRENT_SOURCE_DIR}/src/core
  ${CMAKE_CURRENT_SOURCE_DIR}/src/platform/linux
)

setup_bm_ip_stack(LINUX "${BM_SBC_PLATFORM_INCLUDES}")
setup_bm_os(POSIX "${BM_SBC_PLATFORM_INCLUDES}")

# Build bm_core in hosted (non-embedded) mode — skips ADIN2111 driver,
# guards ELF-specific attributes, etc.
set(BM_HOSTED ON CACHE BOOL "Build bm_core for hosted environment (Linux/macOS)")

add_subdirectory(${BM_CORE_DIR})

# ---------------------------------------------------------------------------
# tomlc17 – TOML v1.1 parser (submodule, MIT license, tag R260323)
# ---------------------------------------------------------------------------
add_library(tomlc17 STATIC ${CMAKE_CURRENT_SOURCE_DIR}/lib/tomlc17/src/tomlc17.c)
target_include_directories(tomlc17 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/lib/tomlc17/src)

# ---------------------------------------------------------------------------
# Git SHA – regenerated on every build via a custom command.
# The helper script only overwrites git_sha.h when the SHA changes,
# so runtime.cpp is not recompiled needlessly.
# ---------------------------------------------------------------------------
set(GIT_SHA_HEADER ${CMAKE_CURRENT_BINARY_DIR}/generated/git_sha.h)

add_custom_target(git_sha_header ALL
  COMMAND ${CMAKE_COMMAND}
    -DSRC_DIR=${CMAKE_CURRENT_SOURCE_DIR}
    -DOUT=${GIT_SHA_HEADER}
    -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/gen_git_sha.cmake
)

# ---------------------------------------------------------------------------
# bm_sbc_core – shared stack runtime library (section 3.1)
# ---------------------------------------------------------------------------
add_library(bm_sbc_core STATIC
  src/core/bm_log.c
  src/core/runtime.cpp
  src/core/app_runner.cpp
  src/core/pcap_file_sink.cpp
  src/core/critical_op.c
  src/platform/linux/platform_linux.cpp
  src/net/virtual_port_device.cpp
  src/net/gateway_device.cpp
  src/net/gateway_ipc.cpp
  src/transports/uart_l2/cobs.c
  src/transports/uart_l2/crc32c.c
  src/transports/uart_l2/frame_codec.c
  src/transports/uart_l2/uart_l2_transport.cpp
)

target_include_directories(bm_sbc_core
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/core
    ${CMAKE_CURRENT_SOURCE_DIR}/src/platform/linux
    ${CMAKE_CURRENT_SOURCE_DIR}/src/net
    ${CMAKE_CURRENT_SOURCE_DIR}/src/transports/uart_l2
)

target_include_directories(bm_sbc_core PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/generated)
add_dependencies(bm_sbc_core git_sha_header)

target_link_libraries(bm_sbc_core PUBLIC bmcore tomlc17)

# ---------------------------------------------------------------------------
# App selection helpers (section 5)
# ---------------------------------------------------------------------------

# build_app(<app_name>)
#   Adds an executable target `bm_sbc_<app_name>` from apps/<app_name>/.
function(build_app APP_NAME)
  set(APP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/apps/${APP_NAME})
  if(NOT IS_DIRECTORY ${APP_DIR})
    message(FATAL_ERROR "App directory not found: ${APP_DIR}")
  endif()

  # Collect all .cpp sources in the app directory.
  file(GLOB APP_SOURCES ${APP_DIR}/*.cpp)
  if(NOT APP_SOURCES)
    message(FATAL_ERROR "No .cpp sources found in ${APP_DIR}")
  endif()

  set(APP_TARGET bm_sbc_${APP_NAME})
  add_executable(${APP_TARGET}
    src/core/main.cpp
    src/core/version_info.cpp
    ${APP_SOURCES}
  )

  # bm_sbc_core contains platform stubs (platform_linux.cpp)
  # that bmcore/bcmp references.
  # GNU ld (Linux) is single-pass, so those object files are
  # silently dropped unless something already references them.
  # Force the whole archive on Linux to break the circular dependency.
  # macOS's linker is multi-pass and handles this automatically.
  target_link_libraries(${APP_TARGET} PRIVATE
    "$<$<PLATFORM_ID:Linux>:-Wl,--push-state,--whole-archive>"
    bm_sbc_core
    "$<$<PLATFORM_ID:Linux>:-Wl,--pop-state>"
  )
  target_compile_definitions(${APP_TARGET} PRIVATE BM_SBC_APP_NAME="${APP_NAME}")
  target_include_directories(${APP_TARGET} PRIVATE
    ${APP_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/generated
  )
  add_dependencies(${APP_TARGET} git_sha_header)
endfunction()

# --- Single-app mode ---
if(APP)
  build_app(${APP})

  # Convenience alias: the default node executable name.
  add_executable(bm_sbc_node ALIAS bm_sbc_${APP})

  install(TARGETS bm_sbc_${APP}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

  # Install DFU support files alongside the gateway binary.
  if(APP STREQUAL "gateway")
    install(PROGRAMS deploy/dfu_prestart.sh
      DESTINATION ${CMAKE_INSTALL_BINDIR})
  endif()

  set(_BM_APP_SUMMARY "${APP}")

# --- Build-all mode ---
elseif(BUILD_ALL_APPS)
  file(GLOB APP_DIRS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/apps
       ${CMAKE_CURRENT_SOURCE_DIR}/apps/*)
  set(_BM_APP_LIST "")
  foreach(APP_ENTRY ${APP_DIRS})
    if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/apps/${APP_ENTRY})
      build_app(${APP_ENTRY})
      list(APPEND _BM_APP_LIST ${APP_ENTRY})
    endif()
  endforeach()

  # Default node target points to the first app alphabetically.
  list(SORT _BM_APP_LIST)
  list(GET _BM_APP_LIST 0 FIRST_APP)
  add_executable(bm_sbc_node ALIAS bm_sbc_${FIRST_APP})

  list(JOIN _BM_APP_LIST ", " _BM_APP_SUMMARY)

# --- No app selected – build the example by default ---
else()
  if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/apps/example)
    build_app(example)
    add_executable(bm_sbc_node ALIAS bm_sbc_example)
    set(_BM_APP_SUMMARY "example (default)")
  else()
    message(STATUS
      "No app selected and no apps/example/ found. "
      "Use -DAPP=<name> or -DBUILD_ALL_APPS=ON.")
    set(_BM_APP_SUMMARY "NONE")
  endif()
endif()

# ---------------------------------------------------------------------------
# Build summary
# ---------------------------------------------------------------------------
message(STATUS "")
message(STATUS "======================================")
message(STATUS "  bm_sbc app(s): ${_BM_APP_SUMMARY}")
if(NOT APP AND NOT BUILD_ALL_APPS)
  message(STATUS "  Tip: use -DAPP=<name> or -DBUILD_ALL_APPS=ON")
endif()
message(STATUS "======================================")

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
enable_testing()

add_executable(test_frame_codec
  tests/test_frame_codec.c
  src/transports/uart_l2/cobs.c
  src/transports/uart_l2/crc32c.c
  src/transports/uart_l2/frame_codec.c
)
target_include_directories(test_frame_codec PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/transports/uart_l2
)
add_test(NAME frame_codec COMMAND test_frame_codec)
