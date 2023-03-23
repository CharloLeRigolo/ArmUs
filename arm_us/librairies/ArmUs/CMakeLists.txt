add_library(
    ArmUs 
    ArmUs.hpp
    ArmUs.cpp
)

target_include_directories(
    ArmUs 
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

add_dependencies(
    ArmUs
    ArmUs_Info
    ArmUs_Controller
)