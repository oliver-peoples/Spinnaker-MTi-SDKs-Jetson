cmake_minimum_required (VERSION 3.19)

project (MTSDK)

include_directories(/usr/local/xsens/include)
link_directories(/usr/local/xsens/lib)

# Add source to this project's executable.
add_executable (MTSDK source.cpp)

target_link_libraries(MTSDK xstypes xsensdeviceapi pthread)

# TODO: Add tests and install targets if needed.