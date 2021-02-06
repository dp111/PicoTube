cmake_minimum_required(VERSION 3.12)
include(pico_sdk_import.cmake)

project(PicoTube)
pico_sdk_init()

add_definitions(-DPICO=1)
add_definitions(-DUSE_MEMORY_POINTER=1)

add_executable(PicoTube
    tube-client.c
    tube-defs.h
    tube.S
    tube-ula.c
    programs.c
    programs.h
    logging.c
    logging.h
    utils.c
    utils.h
    copro-65tube.c
    copro-65tube.h
    copro-65tubeasmM0.S
    copro-65tubeasm.h
    copro-null.c
    copro-null.h
    tuberom_6502_turbo.c
    tuberom_6502.c
    tuberom_6502.h
)

pico_generate_pio_header(PicoTube ${CMAKE_CURRENT_LIST_DIR}/bus6502.pio)

target_link_libraries( PicoTube pico_stdlib pico_multicore hardware_pio)
pico_add_extra_outputs(PicoTube)
#pico_set_binary_type(PicoTube copy_to_ram)

target_link_options(PicoTube PRIVATE LINKER:--sort-section=alignment)
