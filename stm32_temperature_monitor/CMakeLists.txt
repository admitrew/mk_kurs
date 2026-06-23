cmake_minimum_required(VERSION 3.16)

project(stm32_temperature_monitor VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets SerialPort)

add_executable(stm32_temperature_monitor
    main.cpp
    mainwindow.h
    mainwindow.cpp
)

target_link_libraries(stm32_temperature_monitor
    Qt6::Widgets
    Qt6::SerialPort
)