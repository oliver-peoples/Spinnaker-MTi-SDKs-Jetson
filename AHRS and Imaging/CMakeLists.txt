# CMakeList.txt : CMake project for FLIR Chameleon Camera Calibration, include source and define
# project specific logic here.

cmake_minimum_required (VERSION 3.19)

project (Orient)

find_package( OpenCV REQUIRED )
include_directories(/opt/spinnaker/include)
link_directories(/opt/spinnaker/lib)
include_directories(/usr/local/xsens/include)
link_directories(/usr/local/xsens/lib)

# Add source to this project's executable.
add_executable (Orient source2.cpp hmath.h calibration_includes.h FLIR.h calibration.h)

target_link_libraries(Orient ${OpenCV_LIBS} Spinnaker xstypes xsensdeviceapi pthread)

# TODO: Add tests and install targets if needed.