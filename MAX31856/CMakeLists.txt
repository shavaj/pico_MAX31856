aux_source_directory(. DIR_MAX31856_SRCS)
include_directories(.)


add_library(MAX31856 ${DIR_MAX31856_SRCS})
target_link_libraries(MAX31856 PUBLIC pico_stdlib hardware_spi)