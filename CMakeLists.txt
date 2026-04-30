#
# Copyright (c) 2023 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
cmake_minimum_required(VERSION 3.20.0)

# Configure VERSION from repository branch and tag
find_package(Git)

if(GIT_EXECUTABLE)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} branch --show-current
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_BRANCH
    )
  execute_process(
    COMMAND ${GIT_EXECUTABLE} describe --dirty --tags --long --match [0-9]*.[0-9]*.[0-9]*
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_VERSION
    )
  # Prefer a https URL for display (falls back to ssh/origin.url)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} config --get remote.origin.url
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_REPO_URL_SSH
    )
  execute_process(
    COMMAND ${GIT_EXECUTABLE} config --get remote.origin.https.url
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_REPO_URL_HTTPS
    )
  execute_process(
    COMMAND ${GIT_EXECUTABLE} log -1 --pretty=%an
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_AUTHOR
    )
endif()

if(GIT_BRANCH STREQUAL "")
  set(GIT_BRANCH unknown)
  message(WARNING "Unknown Git Branch")
endif()

if(GIT_VERSION STREQUAL "")
  set(GIT_VERSION 0.0.0-0)
  message(WARNING "Unknown Tag/Version")
endif()

# Choose URL: origin.https.url > origin.url
if(NOT GIT_REPO_URL_HTTPS STREQUAL "")
  set(GIT_REPO_URL ${GIT_REPO_URL_HTTPS})
else()
  set(GIT_REPO_URL ${GIT_REPO_URL_SSH})
endif()

# Normalize GitHub SSH URLs to HTTPS for display in firmware info
if(GIT_REPO_URL MATCHES "^git@github\\.com:(.+)$")
  set(GIT_REPO_URL "https://github.com/${CMAKE_MATCH_1}")
elseif(GIT_REPO_URL MATCHES "^ssh://git@github\\.com/(.+)$")
  set(GIT_REPO_URL "https://github.com/${CMAKE_MATCH_1}")
endif()
string(REGEX REPLACE "\\.git$" "" GIT_REPO_URL "${GIT_REPO_URL}")

if(GIT_REPO_URL STREQUAL "")
  set(GIT_REPO_URL unknown)
  message(WARNING "Unknown Git Repo URL")
endif()

if(GIT_AUTHOR STREQUAL "")
  set(GIT_AUTHOR unknown)
  message(WARNING "Unknown Git Author")
endif()

string(STRIP ${GIT_BRANCH} GIT_BRANCH)
string(STRIP ${GIT_VERSION} GIT_VERSION)
string(STRIP ${GIT_REPO_URL} GIT_REPO_URL)
string(STRIP ${GIT_AUTHOR} GIT_AUTHOR)

string(REPLACE "-" ";" GIT_VERSION ${GIT_VERSION})
list(GET GIT_VERSION 0 GIT_RELEASE)
list(GET GIT_VERSION 1 GIT_TWEAK)
string(REPLACE "v" "" GIT_RELEASE ${GIT_RELEASE})
string(REPLACE "." ";" GIT_RELEASE ${GIT_RELEASE})
list(GET GIT_RELEASE 0 GIT_MAJOR)
list(GET GIT_RELEASE 1 GIT_MINOR)
list(GET GIT_RELEASE 2 GIT_PATCH)

configure_file(${CMAKE_SOURCE_DIR}/VERSION_SRC ${CMAKE_SOURCE_DIR}/VERSION @ONLY)

# Pass metadata to C compilation units
add_compile_definitions(
  FW_GIT_BRANCH="${GIT_BRANCH}"
  FW_GIT_REPO_URL="${GIT_REPO_URL}"
  FW_GIT_AUTHOR="${GIT_AUTHOR}"
)

# Zephyr
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(NONE)

find_package(Python3 QUIET COMPONENTS Interpreter)

if(NOT Python3_Interpreter_FOUND)
  find_program(PYTHON_EXECUTABLE NAMES python3 python REQUIRED)
  set(Python3_EXECUTABLE ${PYTHON_EXECUTABLE})
endif()

set(SOFTDEVICE_HEX_52840 ${CMAKE_SOURCE_DIR}/softdevice/s140_nrf52_7.3.0_softdevice.hex)
set(UF2_FAMILY_ID_52840 0xADA52840)
set(UF2_FAMILY_ID_52833 0xADA52840) # Adafruit UF2 family is shared across nRF52 SoCs

set(zephyr_app_hex ${CMAKE_BINARY_DIR}/zephyr/zephyr.hex)
set(mergehex_py ${ZEPHYR_BASE}/scripts/build/mergehex.py)
set(uf2conv_py  ${ZEPHYR_BASE}/scripts/build/uf2conv.py)

if(WITH_SOFTDEVICE)
  if(CONFIG_SOC_NRF52840)
    set(sd_softdevice_hex ${SOFTDEVICE_HEX_52840})
    set(sd_uf2_family_id ${UF2_FAMILY_ID_52840})
  else()
    message(FATAL_ERROR "WITH_SOFTDEVICE is currently only supported on nRF52840 targets")
  endif()

  if(DEFINED sd_softdevice_hex)
    if(NOT EXISTS ${sd_softdevice_hex})
      message(FATAL_ERROR "SoftDevice hex not found: ${sd_softdevice_hex}")
    endif()

    set(sd_merged_hex ${CMAKE_BINARY_DIR}/zephyr/zephyr_sd.hex)
    set(sd_merged_uf2 ${CMAKE_BINARY_DIR}/zephyr/zephyr_sd.uf2)

    add_custom_command(
      OUTPUT ${sd_merged_hex} ${sd_merged_uf2}
      COMMAND ${Python3_EXECUTABLE} ${mergehex_py} --overlap=replace -o ${sd_merged_hex} ${sd_softdevice_hex} ${zephyr_app_hex}
      COMMAND ${Python3_EXECUTABLE} ${uf2conv_py} ${sd_merged_hex} -c -f ${sd_uf2_family_id} -o ${sd_merged_uf2}
      DEPENDS ${sd_softdevice_hex} ${zephyr_app_hex}
      VERBATIM
    )

    add_custom_target(sd_merged_uf2 ALL DEPENDS ${sd_merged_hex} ${sd_merged_uf2})
  endif()
endif()

# ============================================================================
# Combined bootloader + firmware hex (zephyr_full.hex)
#
# pm_static reserves the UF2 bootloader region, but nothing is merged into it
# at build time, so the default zephyr.hex / sysbuild merged.hex contain only
# the application. Flashing those to a fresh nRF52 chip via SWD/J-Link leaves
# it without an MBR, without UICR.NRFFW[0], and without the UF2 bootloader,
# so the device never enumerates over USB.
#
# When a UF2 bootloader hex is supplied, this section produces a real combined
# image (zephyr_full.hex) containing everything needed to flash a virgin chip:
#
#     UF2 bootloader (MBR + bootloader code + UICR)
#   + SoftDevice (when WITH_SOFTDEVICE)
#   + application
#
# Resolution order for the bootloader hex:
#   1. -DUF2_BOOTLOADER_HEX=<path> at cmake time
#   2. UF2_BOOTLOADER_HEX environment variable
#   3. bootloader/<BOARD>.hex   (e.g. bootloader/mochi_uf2.hex)
#   4. bootloader/<SOC>.hex     (e.g. bootloader/nrf52833.hex)
#
# If none of those resolve, zephyr_full.hex is simply not generated and the
# rest of the build proceeds as before.
# ============================================================================

if(NOT DEFINED UF2_BOOTLOADER_HEX AND DEFINED ENV{UF2_BOOTLOADER_HEX})
  set(UF2_BOOTLOADER_HEX $ENV{UF2_BOOTLOADER_HEX})
endif()

if(NOT DEFINED UF2_BOOTLOADER_HEX OR UF2_BOOTLOADER_HEX STREQUAL "")
  set(_uf2_bl_candidates
    ${CMAKE_SOURCE_DIR}/bootloader/${BOARD}.hex
  )
  if(CONFIG_SOC_NRF52840)
    list(APPEND _uf2_bl_candidates ${CMAKE_SOURCE_DIR}/bootloader/nrf52840.hex)
  elseif(CONFIG_SOC_NRF52833)
    list(APPEND _uf2_bl_candidates ${CMAKE_SOURCE_DIR}/bootloader/nrf52833.hex)
  endif()
  foreach(_c ${_uf2_bl_candidates})
    if(EXISTS ${_c})
      set(UF2_BOOTLOADER_HEX ${_c})
      break()
    endif()
  endforeach()
endif()

if(DEFINED UF2_BOOTLOADER_HEX AND NOT UF2_BOOTLOADER_HEX STREQUAL "")
  if(NOT EXISTS ${UF2_BOOTLOADER_HEX})
    message(FATAL_ERROR "UF2_BOOTLOADER_HEX not found: ${UF2_BOOTLOADER_HEX}")
  endif()

  set(full_hex ${CMAKE_BINARY_DIR}/zephyr/zephyr_full.hex)

  # Order matters with --overlap=replace: later inputs win on overlap.
  # Bootloader provides MBR + UICR; SoftDevice (if present) replaces MBR
  # with the Nordic-supplied one; application fills the app region.
  set(full_inputs ${UF2_BOOTLOADER_HEX})
  if(WITH_SOFTDEVICE AND DEFINED sd_softdevice_hex)
    list(APPEND full_inputs ${sd_softdevice_hex})
  endif()
  list(APPEND full_inputs ${zephyr_app_hex})

  add_custom_command(
    OUTPUT ${full_hex}
    COMMAND ${Python3_EXECUTABLE} ${mergehex_py} --overlap=replace -o ${full_hex} ${full_inputs}
    DEPENDS ${full_inputs}
    VERBATIM
  )
  add_custom_target(zephyr_full ALL DEPENDS ${full_hex})

  message(STATUS "Combined hex: ${full_hex}")
  message(STATUS "  bootloader: ${UF2_BOOTLOADER_HEX}")
  if(WITH_SOFTDEVICE AND DEFINED sd_softdevice_hex)
    message(STATUS "  softdevice: ${sd_softdevice_hex}")
  endif()
endif()

# NORDIC SDK APP START
FILE(GLOB_RECURSE app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

# NORDIC SDK APP END

FILE(GLOB_RECURSE files Fusion/Fusion/*.c)
target_sources(app PRIVATE ${files})

FILE(GLOB_RECURSE files vqf-c/src/*.c)
target_sources(app PRIVATE ${files})

target_include_directories(app PRIVATE src)
target_include_directories(app PRIVATE Fusion/Fusion)
target_include_directories(app PRIVATE vqf-c/src)

# Force recompile for updated build timestamp
add_custom_target(touch_util_h ALL
    COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_SOURCE_DIR}/src/build_defines.h
)
