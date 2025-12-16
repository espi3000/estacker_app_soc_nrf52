cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(estacker_app_soc_nrf52 C CXX)



# Set -fpermissive for both C and C++ files
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive")
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fpermissive")

target_sources(app PRIVATE src/main.c)

#target_sources(app PRIVATE src/ai.c)
target_sources(app PRIVATE src/tof.c)
target_sources(app PRIVATE src/bluetooth.c)
target_sources(app PRIVATE src/gpios.c)
target_sources(app PRIVATE src/uart.c)
target_sources(app PRIVATE src/vl53l1_platform.c) # ToF sensor
target_sources(app PRIVATE src/VL53L1X_api.c) # ToF sensor

target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/inc)

# Does not work:
#add_compile_options(-EI_CLASSIFIER_ALLOCATION_STATIC)