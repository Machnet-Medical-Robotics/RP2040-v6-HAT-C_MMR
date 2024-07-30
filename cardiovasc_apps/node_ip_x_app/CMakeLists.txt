# ================================================================================

# *** Target for the valves app (node IP = 192.168.11.2) ***

set(TARGET_NAME_2 node_ip_2_valves)

add_executable(${TARGET_NAME_2}
        node_ip_x_app.c
        )

target_compile_definitions(${TARGET_NAME_2} PUBLIC
        PICO_VALVES
        )

target_link_libraries(${TARGET_NAME_2} PRIVATE
        pico_stdlib
        hardware_spi
        hardware_dma
        ETHERNET_FILES
        IOLIBRARY_FILES
        TIMER_FILES
        )

# printfs go out on the uart
pico_enable_stdio_usb(${TARGET_NAME_2} 0)
pico_enable_stdio_uart(${TARGET_NAME_2} 1)

pico_add_extra_outputs(${TARGET_NAME_2})

# ================================================================================

# *** Target for the sensors app (node IP = 192.168.11.3) ***

set(TARGET_NAME_3 node_ip_3_sensors)

add_executable(${TARGET_NAME_3}
        node_ip_x_app.c
        ${PORT_DIR}/../libraries/pico-sdk/src/rp2_common/hardware_adc/adc.c
        )

target_compile_definitions(${TARGET_NAME_3} PUBLIC
        PICO_SENSORS
        )

target_include_directories(${TARGET_NAME_3} PUBLIC
        ${PORT_DIR}/../libraries/pico-sdk/src/rp2_common/hardware_adc/include
        )

target_link_libraries(${TARGET_NAME_3} PRIVATE
        pico_stdlib
        hardware_spi
        hardware_dma
        ETHERNET_FILES
        IOLIBRARY_FILES
        TIMER_FILES
        )

# printfs go out on the uart
pico_enable_stdio_usb(${TARGET_NAME_3} 0)
pico_enable_stdio_uart(${TARGET_NAME_3} 1)

pico_add_extra_outputs(${TARGET_NAME_3})

