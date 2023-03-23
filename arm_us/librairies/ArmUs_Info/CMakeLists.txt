add_library(
    ArmUs_Info
    ArmUs_Info.hpp
    ArmUs_Info.cpp
)

target_include_directories(
    ArmUs_Info 
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)