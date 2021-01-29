#include <vector>
#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <ostream>
#include <istream>
#include <fstream>
#include <filesystem>
#include <map>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco.hpp>
#include <Spinnaker.h>
#include <SpinnakerDefs.h>
#include <SpinGenApi/SpinnakerGenApi.h>
#include <xsensdeviceapi.h>
#include <xstypes/xstime.h>
#include <xscommon/xsens_mutex.h>
#include <list>
#include "hmath.h"
#include "mv_utilities.h"
#include "FLIR.h"
#include "calibration.h"