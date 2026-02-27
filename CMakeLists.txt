cmake_minimum_required(VERSION 3.5)
project(CANopen_Cia402)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_executable(xml_to_json_bin xml_to_json.cpp pugixml.cpp)

add_executable(canopen_control_bin canopen_control.cpp canopen.cpp pugixml.cpp)
target_link_libraries(canopen_control_bin pthread)

message(STATUS "Targets: xml_to_json, canopen_control")
cmake_minimum_required(VERSION 3.10)
project(CANopen_Cia402 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Sources: xml_to_json tool uses bundled pugixml and single-header nlohmann::json
add_executable(xml_to_json
  xml_to_json.cpp
  pugixml.cpp
)

target_include_directories(xml_to_json PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

if (MSVC)
  target_compile_options(xml_to_json PRIVATE /W4 /permissive-)
else()
  target_compile_options(xml_to_json PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS xml_to_json RUNTIME DESTINATION bin)
