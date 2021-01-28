#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <filesystem>
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
#include <Spinnaker.h>
#include <SpinnakerDefs.h>
#include <SpinGenApi/SpinnakerGenApi.h>
#include "flir.h"

int asymmetricCircleGridCalibration(bool imagery, Spinnaker::CameraPtr pCam, int n_samples)
{
    std::cout << "Running asymmetric circle grid calibration...\n\n";

    cv::Size grid;
    std::string width, height;

    std::cout << "Please enter grid width: ";
    std::cin >> width;
    
    try
    {
        grid.width = std::stoi(width);
    }
    catch(const std::exception& e)
    {
        exit(EXIT_FAILURE);
    }

    std::cout << "Please enter grid height: ";
    std::cin >> height;
    
    try
    {
        grid.height = std::stoi(height);
    }
    catch(const std::exception& e)
    {
        exit(EXIT_FAILURE);
    }

    cv::SimpleBlobDetector::Params parameters;

    parameters.minThreshold = 8;
    parameters.maxThreshold = 255;
    parameters.filterByArea = true;
    parameters.minArea = 64;
    parameters.maxArea = 2500;
    parameters.filterByCircularity = true;
    parameters.minCircularity = 0.1;
    parameters.filterByInertia = true;
    parameters.minInertiaRatio = 0.01;

    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(parameters);

    std::vector<cv::Point3f> object_point_set;
    for (int i = 0; i < grid.height; i++)
    {
        for (int j = 0; j < grid.width; j++)
        {
            object_point_set.push_back(cv::Point3f((2 * j + i % 2), i, 0));
        }
    }

    std::vector<std::vector<cv::Point2f>> image_points;
    std::vector<std::vector<cv::Point3f>> world_points;

    int samples = 0;
    char save_points = 's';

    if (imagery == true)
    {
        while (samples < n_samples)

        {
            std::vector<cv::Point2f> centers;

            cv::Mat raw_img = flir::getImage(pCam);
            cv::Mat img = raw_img.clone();

            bool found = cv::findCirclesGrid(img, grid, centers, cv::CALIB_CB_ASYMMETRIC_GRID, detector);

			if (found == true)
			{
				cv::drawChessboardCorners(img, grid, centers, found);
			}

            cv::Mat display;
            cv::resize(img, display, {0,0}, 0.5, 0.5);
            cv::imshow("image", display);
            
            if (cv::waitKey(40) == save_points && found == true)
            {
                image_points.push_back(centers);
                world_points.push_back(object_point_set);
                samples++;
                std::cout << "Saved cluster " << samples << "!" << std::endl;
            }
        }
    }

    if (imagery == false)
    {
        while (samples < n_samples)

        {
            std::vector<cv::Point2f> centers;

            cv::Mat img = flir::getImage(pCam);

            bool found = cv::findCirclesGrid(img, grid, centers, cv::CALIB_CB_ASYMMETRIC_GRID, detector);

			if (found == true)
			{
                std::cout << "\r Points found!";
				cv::drawChessboardCorners(img, grid, centers, found);
			}
            else
            {
                std::cout << "\r No points found!";
            }
        }
    }

    cv::Mat cameraMatrix, distCoeffs, rvecs, tvecs;

    cv::calibrateCamera(world_points, image_points, {2048,1536}, cameraMatrix, distCoeffs, rvecs, tvecs);

    std::cout << "cameraMatrix : " << cameraMatrix << std::endl;
    std::cout << "distCoeffs : " << distCoeffs << std::endl;

    std::ofstream cameraMatrix_file;
    cameraMatrix_file.open("/home/finn/camera_calibrations/chameleon_cameraMatrix.csv");
    cameraMatrix_file << std::to_string(cameraMatrix.at<double>(0, 0)) + "," + std::to_string(cameraMatrix.at<double>(0, 1)) + "," + std::to_string(cameraMatrix.at<double>(0, 2)) + ",\n";
    cameraMatrix_file << std::to_string(cameraMatrix.at<double>(1, 0)) + "," + std::to_string(cameraMatrix.at<double>(1, 1)) + "," + std::to_string(cameraMatrix.at<double>(1, 2)) + ",\n";
    cameraMatrix_file << std::to_string(cameraMatrix.at<double>(2, 0)) + "," + std::to_string(cameraMatrix.at<double>(2, 1)) + "," + std::to_string(cameraMatrix.at<double>(2, 2)) + ",\n";
    cameraMatrix_file.close();

    std::ofstream distortion_file;
    distortion_file.open("/home/finn/camera_calibrations/chameleon_distCoeffs.csv");
    distortion_file << std::to_string(distCoeffs.at<double>(0, 0)) + "," + std::to_string(distCoeffs.at<double>(0, 1)) + "," + std::to_string(distCoeffs.at<double>(0, 2)) + "," + std::to_string(distCoeffs.at<double>(0, 3)) + "," + std::to_string(distCoeffs.at<double>(0, 4)) + ",\n";
    distortion_file.close();

    char quit = 'q';

    if (imagery == true)
    {
        while (true)
        {
            cv::Mat raw_img = flir::getImage(pCam);
            cv::Mat img = raw_img.clone();
            cv::Mat display;
            cv::undistort(img, display, cameraMatrix, distCoeffs);

            cv::resize(display, display, {0,0}, 0.5, 0.5);
            cv::imshow("image", display);
            
            if (cv::waitKey(40) == quit)
            {
                break;
            }
        }
    }

    cv::destroyAllWindows();

    return 0;
}

int chessboardCalibration(bool imagery, Spinnaker::CameraPtr pCam, int n_samples)
{
    std::cout << "Running chessboard calibration..." << std::endl;

    cv::Size grid;
    std::string width, height;

    std::cout << "Please enter grid width: ";
    std::cin >> width;
    
    try
    {
        grid.width = std::stoi(width);
    }
    catch(const std::exception& e)
    {
        exit(EXIT_FAILURE);
    }

    std::cout << "Please enter grid height: ";
    std::cin >> height;
    
    try
    {
        grid.height = std::stoi(height);
    }
    catch(const std::exception& e)
    {
        exit(EXIT_FAILURE);
    }

    if (imagery == true)
    {
        int flags = cv::CALIB_CB_FAST_CHECK;
		cv::TermCriteria criteria(cv::TermCriteria::MAX_ITER, 30, 0.001);

        while (true)
        {
            std::vector<cv::Point2f> corners;

            cv::Mat img;
            flir::getImage(img, pCam);

            bool found = cv::findChessboardCorners(img, grid, corners, flags);

            cv::Mat display;
            cv::resize(img, display, {0,0}, 0.5, 0.5);
            cv::imshow("image", display);
            cv::waitKey(10);
        }
    }

    if (imagery == false)
    {
        /* code */
    }
    
    
    return 0;
}

int main(int argc, char **argv) 
{
    cv::Size grid_size;
    int n_samples = 0;
    std::string calibration_type;
    int index;
    bool imagery = false;

    if (argc == 2)
    {
        if (std::string(argv[1]) == "y")
        {
            imagery = true;
        }
    }

    if (argc == 3)
    {
        if (std::string(argv[1]) == "y")
        {
            imagery = true;
        }

        try
        {
            n_samples = std::stoi(std::string(argv[2]));
        }
        catch(const std::exception& e)
        {
            exit(EXIT_FAILURE);
        }
    }

    std::cout << n_samples << std::endl;

    std::vector<int (*)(bool, Spinnaker::CameraPtr, int)> func_ptr;

    func_ptr.push_back(asymmetricCircleGridCalibration);
    func_ptr.push_back(chessboardCalibration);

    std::cout << "TRN Camera Calibration Utility";
    std::cout << "\n\n\n";
    std::cout << "I'm going to look for cameras...\n\n";

    Spinnaker::SystemPtr system = Spinnaker::System::GetInstance();
    Spinnaker::CameraList camList = system->GetCameras();

    const unsigned int numCameras = camList.GetSize();

    if (numCameras == 0)
    {
        std::cout << "Darn! I couldn't find a single one!\n";

        camList.Clear();
        system->ReleaseInstance();

        exit(EXIT_FAILURE);
    }
    else if (numCameras > 1)
    {
        std::cout << "I found too many! There should only be one camera for this TRN system!\n";

        camList.Clear();
        system->ReleaseInstance();

        exit(EXIT_FAILURE);
    }

    std::cout << "Found one!\n";
    
    Spinnaker::CameraPtr pCam = camList.GetByIndex(0);
    Spinnaker::GenApi::INodeMap& nodeMapTLDevice = pCam->GetTLDeviceNodeMap();

    flir::PrintDeviceInfo(nodeMapTLDevice);

    pCam->Init();
    
    Spinnaker::GenApi::INodeMap& nodeMap = pCam->GetNodeMap();

    Spinnaker::GenApi::CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
    if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
    {
        std::cout << "Unable to set acquisition mode to continuous (enum retrieval). Aborting..." << std::endl << std::endl;
        exit(EXIT_FAILURE);
    }

    Spinnaker::GenApi::CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
    if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
    {
        std::cout << "Unable to set acquisition mode to continuousd. Aborting..." << std::endl << std::endl;
        exit(EXIT_FAILURE);
    }

    const int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();

    ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);

    Spinnaker::GenApi::CEnumerationPtr ptrTriggerMode = nodeMap.GetNode("TriggerMode");
    if (!Spinnaker::GenApi::IsAvailable(ptrTriggerMode) || !Spinnaker::GenApi::IsReadable(ptrTriggerMode))
    {
        std::cout << "Unable to set acquisition mode to continuousd. Aborting..." << std::endl << std::endl;
        exit(EXIT_FAILURE);
    }

    Spinnaker::GenApi::CEnumEntryPtr ptrTriggerModeOff = ptrTriggerMode->GetEntryByName("Off");
    if (!Spinnaker::GenApi::IsAvailable(ptrTriggerModeOff) || !Spinnaker::GenApi::IsReadable(ptrTriggerModeOff))
    {
        std::cout << "Unable to set acquisition mode to continuousd. Aborting..." << std::endl << std::endl;
        exit(EXIT_FAILURE);
    }

    ptrTriggerMode->SetIntValue(ptrTriggerModeOff->GetValue());

    Spinnaker::GenApi::INodeMap& sNodeMap = pCam->GetTLStreamNodeMap();

    Spinnaker::GenApi::CEnumerationPtr ptrHandlingMode = sNodeMap.GetNode("StreamBufferHandlingMode");
    if (!Spinnaker::GenApi::IsAvailable(ptrHandlingMode) || !Spinnaker::GenApi::IsWritable(ptrHandlingMode))
    {
        std::cout << "Unable to get stream buffer handling mode. Aborting..." << std::endl << std::endl;
        exit(EXIT_FAILURE);
    }

    Spinnaker::GenApi::CEnumEntryPtr ptrHandlingModeEntry = ptrHandlingMode->GetCurrentEntry();
    if (!Spinnaker::GenApi::IsAvailable(ptrHandlingModeEntry) || !Spinnaker::GenApi::IsReadable(ptrHandlingModeEntry))
    {
        std::cout << "Unable to get current stream buffer handling mode. Aborting..." << std::endl << std::endl;
        exit(EXIT_FAILURE);
    }

    // Set stream buffer Count Mode to manual
    Spinnaker::GenApi::CEnumerationPtr ptrStreamBufferCountMode = sNodeMap.GetNode("StreamBufferCountMode");
    if (!Spinnaker::GenApi::IsAvailable(ptrStreamBufferCountMode) || !Spinnaker::GenApi::IsWritable(ptrStreamBufferCountMode))
    {
        std::cout << "Unable to set Buffer Count Mode (node retrieval). Aborting..." << std::endl << std::endl;
        exit(EXIT_FAILURE);
    }

    Spinnaker::GenApi::CEnumEntryPtr ptrStreamBufferCountModeManual = ptrStreamBufferCountMode->GetEntryByName("Manual");
    if (!Spinnaker::GenApi::IsAvailable(ptrStreamBufferCountModeManual) || !Spinnaker::GenApi::IsReadable(ptrStreamBufferCountModeManual))
    {
        std::cout << "Unable to set Buffer Count Mode entry (Entry retrieval). Aborting..." << std::endl << std::endl;
        exit(EXIT_FAILURE);
    }

    ptrStreamBufferCountMode->SetIntValue(ptrStreamBufferCountModeManual->GetValue());

    // Retrieve and modify Stream Buffer Count
    Spinnaker::GenApi::CIntegerPtr ptrBufferCount = sNodeMap.GetNode("StreamBufferCountManual");
    if (!Spinnaker::GenApi::IsAvailable(ptrBufferCount) || !Spinnaker::GenApi::IsWritable(ptrBufferCount))
    {
        std::cout << "Unable to set Buffer Count (Integer node retrieval). Aborting..." << std::endl << std::endl;
        exit(EXIT_FAILURE);
    }

    ptrBufferCount->SetValue(5);

    ptrHandlingModeEntry = ptrHandlingMode->GetEntryByName("NewestOnly");
    ptrHandlingMode->SetIntValue(ptrHandlingModeEntry->GetValue());

    pCam->BeginAcquisition();
    
    std::cout << "\n\nPlease enter number associated with desired calibration type:\n" << std::endl;
    std::cout << "1. Asymmetric Circle Grid\n";
    std::cout << "2. Chessboard";
    std::cout << "\n\n";
    std::cout << "Desired calibration technique: ";
    std::cin >> calibration_type;

    try
    {
        index = std::stoi(calibration_type);
    }
    catch(const std::exception& e)
    {
        std::cout << "\n\nYou entered something wrong. Maybe text instead of a number? Or did you put a decimal somehwere, hmm? Adios!\n\n";
        exit(EXIT_FAILURE);
    }

    if (index <= func_ptr.size() && index != 0)
    {
        std::cout << "\nA valid option! Bravo.\n\n";

        if (index == 1)
        {
            asymmetricCircleGridCalibration(imagery, pCam, n_samples);
        }

        if (index == 2)
        {
            chessboardCalibration(imagery, pCam, n_samples);
        }
    }
    else
    {
        std::cout << "\n\nAn invalid option was entered! Exiting...adios!\n\n";
        exit(EXIT_FAILURE);
    }
  
    return 0; 
} 
