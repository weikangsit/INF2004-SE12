# Arm GNU toolchain (arm-none-eabi) for the RP2040.
# Finds the compiler on PATH, or in the Raspberry Pi Pico VS Code
# extension's install (~/.pico-sdk/toolchain/<ver>/bin).
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR cortex-m0plus)

file(GLOB _pico_tc LIST_DIRECTORIES true "$ENV{USERPROFILE}/.pico-sdk/toolchain/*/bin"
                                        "$ENV{HOME}/.pico-sdk/toolchain/*/bin")
find_program(ARM_GCC arm-none-eabi-gcc HINTS ${_pico_tc} REQUIRED)
get_filename_component(_tc_bin "${ARM_GCC}" DIRECTORY)
if(CMAKE_HOST_WIN32)
    set(_exe ".exe")
else()
    set(_exe "")
endif()

set(CMAKE_C_COMPILER   "${_tc_bin}/arm-none-eabi-gcc${_exe}")
set(CMAKE_ASM_COMPILER "${_tc_bin}/arm-none-eabi-gcc${_exe}")
set(CMAKE_OBJCOPY      "${_tc_bin}/arm-none-eabi-objcopy${_exe}" CACHE FILEPATH "")
set(CMAKE_SIZE         "${_tc_bin}/arm-none-eabi-size${_exe}" CACHE FILEPATH "")

# Bare metal: CMake's link test would fail without a startup file.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
