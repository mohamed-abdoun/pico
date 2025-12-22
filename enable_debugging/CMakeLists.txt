cmake_minimum_required(VERSION 3.13)

# Include the Pico SDK
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(myproject C CXX ASM)

# Set C/C++ standards
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK
pico_sdk_init()

# Create executable
add_executable(myproject
    main.c
)

# Link libraries
target_link_libraries(myproject pico_stdlib)

# Enable USB serial output, disable UART
pico_enable_stdio_usb(myproject 1)
pico_enable_stdio_uart(myproject 0)

# Create UF2 file
pico_add_extra_outputs(myproject)