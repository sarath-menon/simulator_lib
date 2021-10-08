

add_library(simulator_lib STATIC
src/simulator.cpp
)

target_include_directories(simulator_lib PUBLIC include)

target_link_libraries(simulator_lib PRIVATE
yaml-cpp
)


