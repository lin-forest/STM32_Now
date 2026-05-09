cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME 3_MCLM_t2)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
    
    # ===== App core =====
    App/config/app_task.c

    # ===== Drivers (no RTOS dependency) =====
    # App/drivers/motor_BLDC.c
    App/drivers/motor_DC_tb6612.c
    # App/drivers/motor_DC_ibt4.c
    # App/drivers/encoder.c
    App/drivers/motor_DC_at8236.c
    

    # ===== FreeRTOS tasks =====
    App/tasks/command_task.c
    App/tasks/encoder_task.c
    App/tasks/logger_task.c
    App/tasks/Ack_task.c
    App/tasks/heartbeat_task.c

    # App/tasks/BLDC_task.c
    # App/tasks/ibt4_DC_task.c
    App/tasks/at8236_DC_task.c
    App/tasks/tb6612_DC_task.c



    # ===== Modules =====
    App/modules/pid.c
    App/modules/speed_map.c
    App/modules/filter.c

    # ===== Services =====
    App/services/command.c
    App/services/logger.c
    App/services/can_filter.c
    # App/services/can_service.c # Add this line
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
    # ===== CubeMX / HAL =====
    ${CMAKE_SOURCE_DIR}/Core/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/STM32F1xx_HAL_Driver/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F1xx/Include
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include

    # ===== FreeRTOS =====
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/include
    ${CMAKE_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2

    # ===== App layer =====
    ${CMAKE_SOURCE_DIR}/App
    ${CMAKE_SOURCE_DIR}/App/drivers
    ${CMAKE_SOURCE_DIR}/App/tasks
    ${CMAKE_SOURCE_DIR}/App/modules
    ${CMAKE_SOURCE_DIR}/App/services
    ${CMAKE_SOURCE_DIR}/App/config
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
