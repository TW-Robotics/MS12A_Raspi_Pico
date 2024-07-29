cmake_minimum_required(VERSION 3.13)

# Define the project name and version



# Include the Pico SDK
include(pico_sdk_import.cmake)
include(pico_extras_import_optional.cmake)
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(ms12apico CXX C ASM)
# Initialize the Pico SDK
pico_sdk_init()


# Define the executable target
add_executable(${PROJECT_NAME}
    src/main.cpp
    src/MeSerial.cpp
    src/MeSmartServo.cpp
)

# Add the header files as sources (optional, for IDE support)
target_sources(${PROJECT_NAME} PRIVATE
    src/MeSerial.h
    src/MeSmartServo.h
)

# Link the necessary libraries
target_link_libraries(${PROJECT_NAME} pico_stdlib hardware_uart)

# Enable USB and UART output
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 1)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${PROJECT_NAME})


