cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)
include(pico_extras_import.cmake)

project(pi_pico_MAX31856 C CXX)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# add a compilation subdirectory
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/MAX31856)

# add a header directory 
include_directories(${CMAKE_CURRENT_LIST_DIR}/MAX31856)


# generate an executable file
add_executable(pico_MAX31856  
main.cpp
)

#target_include_directories(pcpp PUBLIC ${CMAKE_CURRENT_LIST_DIR}/SSD1306)
target_link_libraries(pico_MAX31856 MAX31856 pico_stdlib hardware_spi pico_runtime pico_multicore)

# enable usb output, disable uart output
pico_enable_stdio_usb(pico_MAX31856 1)
pico_enable_stdio_uart(pico_MAX31856 0)

# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(pico_MAX31856)
