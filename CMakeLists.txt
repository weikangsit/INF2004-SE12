# Group 12 robot car - CMake build for the Raspberry Pi Pico W.
#
# Builds the car firmware (src/) on top of the INF2004 micro T-Kernel 3.0
# template (github.com/sirfonzie/INF2004_ProjectTemplate) and the Pico C SDK,
# with exactly the compiler flags and kernel sources of the template's
# `make NET=lwip MQTT=1` build (dual-core, USB console, WiFi + lwIP + MQTT).
# The template's demo app_program/ and demo mqtt_utk.c are replaced by src/.
#
#   cmake -S . -B build -G Ninja -DWIFI_SSID="ssid" -DWIFI_PASSWORD="pw"
#   cmake --build build            ->  build/robocar_g12.uf2
#
# Defaults find the tools installed by the Raspberry Pi Pico VS Code
# extension (~/.pico-sdk) and the template cloned next to this folder.
cmake_minimum_required(VERSION 3.20)

if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_LIST_DIR}/cmake/arm-none-eabi.cmake")
endif()

project(robocar_g12 C ASM)

# ---------------------------------------------------------------- settings
if(DEFINED ENV{USERPROFILE})
    set(_home "$ENV{USERPROFILE}")
else()
    set(_home "$ENV{HOME}")
endif()

# Major version of a Pico SDK folder (0 if it is not one)
function(_sdk_major dir out)
    set(_v 0)
    if(EXISTS "${dir}/pico_sdk_version.cmake")
        file(STRINGS "${dir}/pico_sdk_version.cmake" _l REGEX "set\\(PICO_SDK_VERSION_MAJOR")
        string(REGEX MATCH "[0-9]+" _v "${_l}")
    endif()
    set(${out} ${_v} PARENT_SCOPE)
endfunction()

# The template needs SDK 2.x (its header layout differs from 1.x). Newest
# ~/.pico-sdk/sdk/<ver> from the Pico VS Code extension is preferred; an
# environment PICO_SDK_PATH is used only if it is 2.x as well.
file(GLOB _sdk_guess LIST_DIRECTORIES true "${_home}/.pico-sdk/sdk/*")
list(SORT _sdk_guess COMPARE NATURAL ORDER DESCENDING)
set(_sdk_default "")
if(DEFINED ENV{PICO_SDK_PATH})
    _sdk_major("$ENV{PICO_SDK_PATH}" _env_major)
    if(_env_major GREATER_EQUAL 2)
        set(_sdk_default "$ENV{PICO_SDK_PATH}")
    else()
        message(STATUS "Ignoring PICO_SDK_PATH=$ENV{PICO_SDK_PATH} (SDK ${_env_major}.x; need 2.x)")
    endif()
endif()
if(_sdk_default STREQUAL "" AND _sdk_guess)
    list(GET _sdk_guess 0 _sdk_default)
endif()

set(PICO_SDK_PATH "${_sdk_default}" CACHE PATH "Pico C SDK 2.x (with lib/cyw43-driver, lib/lwip, lib/tinyusb)")

# A build folder configured earlier may have cached an old 1.x SDK: fix it.
_sdk_major("${PICO_SDK_PATH}" _major)
if(_major LESS 2)
    if(NOT _sdk_default STREQUAL "" AND NOT _sdk_default STREQUAL PICO_SDK_PATH)
        message(WARNING "PICO_SDK_PATH=${PICO_SDK_PATH} is not a Pico SDK 2.x - using ${_sdk_default}")
        set(PICO_SDK_PATH "${_sdk_default}" CACHE PATH "Pico C SDK 2.x" FORCE)
    else()
        message(FATAL_ERROR "PICO_SDK_PATH=${PICO_SDK_PATH} is not a Pico SDK 2.x.\n"
            "Install the Raspberry Pi Pico VS Code extension, or pass -DPICO_SDK_PATH=<pico-sdk 2.2/2.3>")
    endif()
endif()
message(STATUS "Pico SDK: ${PICO_SDK_PATH}")

set(MTK3_DIR "${CMAKE_CURRENT_LIST_DIR}/../mtk3_template" CACHE PATH "INF2004 micro T-Kernel template checkout")
set(WIFI_SSID "Kuro" CACHE STRING "WiFi network the car joins (personal hotspot)")
set(WIFI_PASSWORD "change-me" CACHE STRING "WiFi password (pass on the command line, never commit)")
set(WIFI_SECURITY "WPA3_WPA2" CACHE STRING "Hotspot security: WPA2, WPA3_WPA2 or WPA3")
set_property(CACHE WIFI_SECURITY PROPERTY STRINGS WPA2 WPA3_WPA2 WPA3)
set(MQTT_BROKER_IP "172.20.10.6" CACHE STRING "IP of the laptop running web/server.py (hotspot Kuro)")
set(MQTT_BROKER_PORT "1883" CACHE STRING "MQTT broker TCP port")
set(MQTT_PASSWORD "" CACHE STRING "Broker --password (empty = open broker)")
option(ROBOCAR_SMP "Run micro T-Kernel on both RP2040 cores" ON)

foreach(_p "${PICO_SDK_PATH}/lib/cyw43-driver/src/cyw43_ll.c"
           "${PICO_SDK_PATH}/lib/lwip/src/core/init.c"
           "${PICO_SDK_PATH}/lib/tinyusb/src/tusb.c")
    if(NOT EXISTS "${_p}")
        message(FATAL_ERROR "Pico SDK incomplete: ${_p} missing.\n"
            "Set -DPICO_SDK_PATH=... and run: git submodule update --init lib/tinyusb lib/cyw43-driver lib/lwip")
    endif()
endforeach()
if(NOT EXISTS "${MTK3_DIR}/kernel/tkernel/task.c")
    message(FATAL_ERROR "micro T-Kernel template not found at MTK3_DIR=${MTK3_DIR}\n"
        "git clone https://github.com/sirfonzie/INF2004_ProjectTemplate.git ${MTK3_DIR}")
endif()
if(WIFI_PASSWORD STREQUAL "change-me")
    message(WARNING "WIFI_PASSWORD is still the placeholder - pass -DWIFI_PASSWORD=\"<hotspot password>\"")
endif()

# CYW43 join mode (cyw43_ll.h). WPA3_WPA2 joins a WPA3 hotspot, and also
# a WPA2 one or a hotspot in transition mode.
if(WIFI_SECURITY STREQUAL "WPA2")
    set(WIFI_AUTH_MODE_VALUE "CYW43_AUTH_WPA2_MIXED_PSK")
elseif(WIFI_SECURITY STREQUAL "WPA3")
    set(WIFI_AUTH_MODE_VALUE "CYW43_AUTH_WPA3_SAE_AES_PSK")
elseif(WIFI_SECURITY STREQUAL "WPA3_WPA2")
    set(WIFI_AUTH_MODE_VALUE "CYW43_AUTH_WPA3_WPA2_AES_PSK")
else()
    message(FATAL_ERROR "WIFI_SECURITY must be WPA2, WPA3_WPA2 or WPA3")
endif()

set(MTK3 "${MTK3_DIR}")
set(SDK  "${PICO_SDK_PATH}")

# Generated credentials header (build dir only, never committed)
if(MQTT_PASSWORD STREQUAL "")
    set(MQTT_PASSWORD_LINE "/* MQTT_PASSWORD not set: open broker */")
else()
    set(MQTT_PASSWORD_LINE "#define MQTT_PASSWORD    \"${MQTT_PASSWORD}\"")
endif()
configure_file(cmake/wifi_secrets.h.in "${CMAKE_BINARY_DIR}/generated/wifi_secrets.h" @ONLY)

# ---------------------------------------------------------------- sources
# Kernel / drivers / SDK pieces: the object list of the template's
# `make NET=lwip MQTT=1` build, minus app_program/ and the demo mqtt_utk.c.
set(KERNEL_SOURCES
    ${MTK3}/device/adc/adc.c
    ${MTK3}/device/adc/sysdepend/rp2040/adc_rp2040.c
    ${MTK3}/device/common/drvif/msdrvif.c
    ${MTK3}/device/i2c/i2c.c
    ${MTK3}/device/i2c/sysdepend/rp2040/i2c_rp2040.c
    ${MTK3}/device/ser/ser.c
    ${MTK3}/device/ser/sysdepend/rp2040/ser_rp2040.c
    ${MTK3}/kernel/inittask/inittask.c
    ${MTK3}/kernel/sysdepend/cpu/core/armv6m/cpu_cntl.c
    ${MTK3}/kernel/sysdepend/cpu/core/armv6m/dispatch.S
    ${MTK3}/kernel/sysdepend/cpu/core/armv6m/exc_hdr.c
    ${MTK3}/kernel/sysdepend/cpu/core/armv6m/interrupt.c
    ${MTK3}/kernel/sysdepend/cpu/core/armv6m/reset_hdl.c
    ${MTK3}/kernel/sysdepend/cpu/rp2040/boot_2nd.c
    ${MTK3}/kernel/sysdepend/cpu/rp2040/smp_atomic.c
    ${MTK3}/kernel/sysdepend/cpu/rp2040/smp_rp2040.c
    ${MTK3}/kernel/sysdepend/cpu/rp2040/vector_tbl.c
    ${MTK3}/kernel/sysdepend/pico_rp2040/cpu_clock.c
    ${MTK3}/kernel/sysdepend/pico_rp2040/devinit.c
    ${MTK3}/kernel/sysdepend/pico_rp2040/hw_setting.c
    ${MTK3}/kernel/sysdepend/pico_rp2040/power_save.c
    ${MTK3}/kernel/sysinit/sysinit.c
    ${MTK3}/kernel/tkernel/cpuctl.c
    ${MTK3}/kernel/tkernel/device.c
    ${MTK3}/kernel/tkernel/deviceio.c
    ${MTK3}/kernel/tkernel/eventflag.c
    ${MTK3}/kernel/tkernel/int.c
    ${MTK3}/kernel/tkernel/klock.c
    ${MTK3}/kernel/tkernel/mailbox.c
    ${MTK3}/kernel/tkernel/memory.c
    ${MTK3}/kernel/tkernel/mempfix.c
    ${MTK3}/kernel/tkernel/mempool.c
    ${MTK3}/kernel/tkernel/messagebuf.c
    ${MTK3}/kernel/tkernel/misc_calls.c
    ${MTK3}/kernel/tkernel/mutex.c
    ${MTK3}/kernel/tkernel/objname.c
    ${MTK3}/kernel/tkernel/power.c
    ${MTK3}/kernel/tkernel/ready_queue_smp.c
    ${MTK3}/kernel/tkernel/rendezvous.c
    ${MTK3}/kernel/tkernel/semaphore.c
    ${MTK3}/kernel/tkernel/smp_dispatch.c
    ${MTK3}/kernel/tkernel/smp_lock.c
    ${MTK3}/kernel/tkernel/task.c
    ${MTK3}/kernel/tkernel/task_manage.c
    ${MTK3}/kernel/tkernel/task_sync.c
    ${MTK3}/kernel/tkernel/time_calls.c
    ${MTK3}/kernel/tkernel/timer.c
    ${MTK3}/kernel/tkernel/tkinit.c
    ${MTK3}/kernel/tkernel/wait.c
    ${MTK3}/kernel/tstdlib/bitop.c
    ${MTK3}/kernel/tstdlib/string.c
    ${MTK3}/kernel/usermain/usermain.c
    ${MTK3}/lib/libbsp/sysdepend/cpu/rp2040/libbsp_pwm.c
    ${MTK3}/lib/liblwip/sysdepend/pico_rp2040/sys_arch.c
    ${MTK3}/lib/libtk/fastlock.c
    ${MTK3}/lib/libtk/fastmlock.c
    ${MTK3}/lib/libtk/kmalloc.c
    ${MTK3}/lib/libtk/sysdepend/cpu/core/armv6m/int_armv6m.c
    ${MTK3}/lib/libtk/sysdepend/cpu/core/armv6m/wusec_armv6m.c
    ${MTK3}/lib/libtk/sysdepend/cpu/rp2040/int_rp2040.c
    ${MTK3}/lib/libtk/sysdepend/cpu/rp2040/ptimer_rp2040.c
    ${MTK3}/lib/libtm/libtm.c
    ${MTK3}/lib/libtm/libtm_printf.c
    ${MTK3}/lib/libtm/sysdepend/no_device/tm_com.c
    ${MTK3}/lib/libtm/sysdepend/pico_rp2040/tm_com.c
    ${MTK3}/lib/libtm/sysdepend/pico_rp2040/usb/usb_console.c
    ${MTK3}/lib/libtm/sysdepend/pico_rp2040/usb/usb_descriptors.c
    ${MTK3}/lib/libtm/sysdepend/pico_rp2040/usb/usb_sdk_compat.c
    ${MTK3}/lib/libtm/sysdepend/pico_rp2040/usb/usb_tinyusb_glue.c
    ${MTK3}/lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c
    ${MTK3}/lib/libwifi/sysdepend/pico_rp2040/cyw43_utk_compat.c
    ${MTK3}/lib/libwifi/sysdepend/pico_rp2040/cyw43_utk_kernel.c
    ${SDK}/lib/cyw43-driver/src/cyw43_ctrl.c
    ${SDK}/lib/cyw43-driver/src/cyw43_ll.c
    ${SDK}/lib/cyw43-driver/src/cyw43_lwip.c
    ${SDK}/lib/cyw43-driver/src/cyw43_stats.c
    ${SDK}/lib/lwip/src/apps/mqtt/mqtt.c
    ${SDK}/lib/lwip/src/core/altcp.c
    ${SDK}/lib/lwip/src/core/altcp_alloc.c
    ${SDK}/lib/lwip/src/core/altcp_tcp.c
    ${SDK}/lib/lwip/src/core/def.c
    ${SDK}/lib/lwip/src/core/dns.c
    ${SDK}/lib/lwip/src/core/inet_chksum.c
    ${SDK}/lib/lwip/src/core/init.c
    ${SDK}/lib/lwip/src/core/ip.c
    ${SDK}/lib/lwip/src/core/ipv4/acd.c
    ${SDK}/lib/lwip/src/core/ipv4/autoip.c
    ${SDK}/lib/lwip/src/core/ipv4/dhcp.c
    ${SDK}/lib/lwip/src/core/ipv4/etharp.c
    ${SDK}/lib/lwip/src/core/ipv4/icmp.c
    ${SDK}/lib/lwip/src/core/ipv4/igmp.c
    ${SDK}/lib/lwip/src/core/ipv4/ip4.c
    ${SDK}/lib/lwip/src/core/ipv4/ip4_addr.c
    ${SDK}/lib/lwip/src/core/ipv4/ip4_frag.c
    ${SDK}/lib/lwip/src/core/mem.c
    ${SDK}/lib/lwip/src/core/memp.c
    ${SDK}/lib/lwip/src/core/netif.c
    ${SDK}/lib/lwip/src/core/pbuf.c
    ${SDK}/lib/lwip/src/core/raw.c
    ${SDK}/lib/lwip/src/core/stats.c
    ${SDK}/lib/lwip/src/core/sys.c
    ${SDK}/lib/lwip/src/core/tcp.c
    ${SDK}/lib/lwip/src/core/tcp_in.c
    ${SDK}/lib/lwip/src/core/tcp_out.c
    ${SDK}/lib/lwip/src/core/timeouts.c
    ${SDK}/lib/lwip/src/core/udp.c
    ${SDK}/lib/lwip/src/netif/ethernet.c
    ${SDK}/lib/tinyusb/src/class/cdc/cdc_device.c
    ${SDK}/lib/tinyusb/src/common/tusb_fifo.c
    ${SDK}/lib/tinyusb/src/device/usbd.c
    ${SDK}/lib/tinyusb/src/device/usbd_control.c
    ${SDK}/lib/tinyusb/src/portable/raspberrypi/rp2040/dcd_rp2040.c
    ${SDK}/lib/tinyusb/src/portable/raspberrypi/rp2040/rp2040_usb.c
    ${SDK}/lib/tinyusb/src/tusb.c
    ${SDK}/src/rp2_common/hardware_dma/dma.c
    ${SDK}/src/rp2_common/hardware_gpio/gpio.c
    ${SDK}/src/rp2_common/hardware_pio/pio.c
    ${SDK}/src/rp2_common/pico_cyw43_driver/cyw43_bus_pio_spi.c
)

set(APP_SOURCES
    src/main.c
    src/hal.c            # SDK side (hardware/... headers, no kernel)
    src/hal_irq.c        # kernel side of the GPIO interrupt
    src/i2c_drv.c        # SDK side
    src/line_barcode.c
    src/barcode_map.c
    src/motion.c
    src/imu.c
    src/ultrasonic.c
    src/nav_fsm.c
    src/comms.c          # replaces the template's demo mqtt_utk.c
    src/proto.c          # packed MQTT payloads (Change Requirement 1)
    src/fault.c          # shared fault bitmask
    src/telemetry.c
    src/safety.c
)

add_executable(robocar_g12 ${KERNEL_SOURCES} ${APP_SOURCES})
set_target_properties(robocar_g12 PROPERTIES SUFFIX ".elf")

# ---------------------------------------------------------------- flags
# Same order as the template makefile: generated secrets first so they win
# over config/wifi_secrets.h if one exists in the template checkout.
target_include_directories(robocar_g12 PRIVATE
    ${CMAKE_BINARY_DIR}/generated
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${MTK3}/lib/libtm/sysdepend/pico_rp2040/usb
    ${SDK}/lib/tinyusb/src
    ${SDK}/src/common/pico_base_headers/include
    ${SDK}/src/common/pico_binary_info/include
    ${SDK}/src/rp2040/pico_platform/include
    ${SDK}/src/rp2_common/pico_platform_compiler/include
    ${SDK}/src/rp2_common/pico_platform_sections/include
    ${SDK}/src/rp2_common/pico_platform_panic/include
    ${SDK}/src/rp2_common/pico_platform_common/include
    ${SDK}/src/rp2_common/hardware_base/include
    ${SDK}/src/rp2_common/hardware_irq/include
    ${SDK}/src/rp2_common/hardware_resets/include
    ${SDK}/src/rp2_common/hardware_sync/include
    ${SDK}/src/rp2_common/hardware_sync_spin_lock/include
    ${SDK}/src/rp2_common/hardware_timer/include
    ${SDK}/src/rp2040/hardware_regs/include
    ${SDK}/src/rp2040/hardware_structs/include
    ${MTK3}/lib/libwifi/sysdepend/pico_rp2040
    ${SDK}/lib/cyw43-driver/src
    ${SDK}/lib/cyw43-driver/firmware
    ${SDK}/src/rp2_common/pico_cyw43_driver/include
    ${SDK}/src/boards/include
    ${SDK}/src/common/hardware_claim/include
    ${SDK}/src/common/pico_util/include
    ${SDK}/src/rp2_common/hardware_pio/include
    ${SDK}/src/rp2_common/hardware_gpio/include
    ${SDK}/src/rp2_common/hardware_dma/include
    ${SDK}/src/rp2_common/hardware_clocks/include
    ${SDK}/src/rp2_common/hardware_pwm/include        # car: motor / servo PWM
    ${MTK3}/lib/liblwip/sysdepend/pico_rp2040/include
    ${MTK3}/lib/liblwip/sysdepend/pico_rp2040
    ${SDK}/lib/lwip/src/include
    ${MTK3}/include
    ${MTK3}/config
    ${MTK3}/kernel/knlinc
)

set(_cpu -mcpu=cortex-m0plus -mthumb -ffreestanding)
target_compile_options(robocar_g12 PRIVATE
    ${_cpu} -O2 -g3
    $<$<COMPILE_LANGUAGE:C>:-std=gnu11 -mfloat-abi=soft -ffunction-sections -fdata-sections>
    $<$<COMPILE_LANGUAGE:ASM>:-x assembler-with-cpp>
)

target_compile_definitions(robocar_g12 PRIVATE
    _PICO_RP2040_
    $<$<BOOL:${ROBOCAR_SMP}>:CNF_SMP=1>
    # C-only defines of the template build (its .S files get none of these)
    $<$<COMPILE_LANGUAGE:C>:APP_TEST=0;TM_CONSOLE_UART=0;BOARD_UART0_FREE=1;TM_CONSOLE_USB_CDC=1;NDEBUG>
    $<$<COMPILE_LANGUAGE:C>:TM_WIFI_CYW43=1;CYW43_LWIP=1;CYW43_ENABLE_BLUETOOTH=0;CYW43_USE_OTP_MAC=1>
    $<$<COMPILE_LANGUAGE:C>:PICO_CYW43_LOGGING_ENABLED=0;PICO_RP2040=1;PICO_32BIT=1;PICO_ON_DEVICE=1>
    $<$<COMPILE_LANGUAGE:C>:PICO_BUILD=1;PICO_NO_HARDWARE=0;RASPBERRYPI_PICO_W=1>
    $<$<COMPILE_LANGUAGE:C>:CYW43_DEFAULT_PIN_WL_REG_ON=23u;CYW43_DEFAULT_PIN_WL_DATA_OUT=24u>
    $<$<COMPILE_LANGUAGE:C>:CYW43_DEFAULT_PIN_WL_DATA_IN=24u;CYW43_DEFAULT_PIN_WL_HOST_WAKE=24u>
    $<$<COMPILE_LANGUAGE:C>:CYW43_DEFAULT_PIN_WL_CLOCK=29u;CYW43_DEFAULT_PIN_WL_CS=25u>
    $<$<COMPILE_LANGUAGE:C>:TM_NET_LWIP=1;TM_MQTT=1>
    # lwIP's MQTT output ring defaults to 256 bytes: too small for the
    # burst at connect (2 subscribes + offline backlog), frames got dropped
    $<$<COMPILE_LANGUAGE:C>:MQTT_OUTPUT_RINGBUF_SIZE=2048>
)

# Stricter warnings for our own files only (the kernel is built as-is).
# PROHIBIT_DEF_SIZE_T: our files use <string.h>, whose size_t would clash
# with the kernel's own typedef in <tk/syslib.h>.
set_source_files_properties(${APP_SOURCES} PROPERTIES
    COMPILE_OPTIONS "-Wall;-Wextra"
    COMPILE_DEFINITIONS "PROHIBIT_DEF_SIZE_T=1")

target_link_options(robocar_g12 PRIVATE
    ${_cpu} -nostartfiles -O2 -g3 -mfloat-abi=soft
    -Wl,--gc-sections
    -T ${MTK3}/etc/linker/pico_rp2040/tkernel_map.ld
    -Wl,-Map=$<TARGET_FILE_DIR:robocar_g12>/robocar_g12.map
)
target_link_libraries(robocar_g12 PRIVATE m)     # sinf / atan2f / sqrtf ...

# ---------------------------------------------------------------- .uf2
file(GLOB _picotool_guess "${_home}/.pico-sdk/picotool/*/picotool")
find_program(PICOTOOL picotool HINTS ${_picotool_guess})
if(PICOTOOL)
    add_custom_command(TARGET robocar_g12 POST_BUILD
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:robocar_g12>
        COMMAND ${PICOTOOL} uf2 convert $<TARGET_FILE:robocar_g12>
                ${CMAKE_BINARY_DIR}/robocar_g12.uf2 --family rp2040
        COMMENT "Creating robocar_g12.uf2 (drag onto the RPI-RP2 drive)")
else()
    message(WARNING "picotool not found: only robocar_g12.elf will be built. "
                    "Install the Pico VS Code extension or put picotool on PATH.")
endif()
