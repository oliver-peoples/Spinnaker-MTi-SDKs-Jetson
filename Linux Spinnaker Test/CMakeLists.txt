# CMakeList.txt : CMake project for FLIR Chameleon Camera Calibration, include source and define
# project specific logic here.

cmake_minimum_required (VERSION 3.19)

project (FLIR_Stream)

find_package( OpenCV REQUIRED )
include_directories(/opt/spinnaker/include)
link_directories(/opt/spinnaker/lib)

# Add source to this project's executable.
add_executable (FLIR_Stream source.cpp FLIR.h)

target_link_libraries(FLIR_Stream ${OpenCV_LIBS} Spinnaker)

# TODO: Add tests and install targets if needed.