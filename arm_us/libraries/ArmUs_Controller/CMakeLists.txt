add_library(
    ArmUs_Controller 
    ArmUs_Controller.hpp
    ArmUs_Controller.cpp
)

target_include_directories(
    ArmUs_Controller 
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)