# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(EXTRA_COMPONENT_DIRS "${EXTRA_COMPONENT_DIRS} 
    $ENV{IOT_SOLUTION_PATH}/components/motor/servo 
    C:/Users/FA.PROJECTOR-MSK/YandexDisk/Software/cpp/lib")


include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESP32_ESP-IDF)