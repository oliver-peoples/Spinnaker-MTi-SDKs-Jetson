#include "calibration_includes.h"

int main(int argc, char **argv) 
{
    cv::Vec3d rvec = {1.391895819721201, 1.079665068873341, 1.391895819721201};
    hmath::Quaternion quaternion = mv::math::quaternionFromRVEC(rvec);

    std::cout << rvec << ", " << quaternion << std::endl;

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
    
    mv::spinnaker::standardSetup(pCam);

    pCam->BeginAcquisition();

    mv::calibration::calibration cal = mv::calibration::loadCalibration();

    std::cout << cal.cameraMatrix << std::endl;
    std::cout << cal.distCoeffs << std::endl;

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    parameters->cornerRefinementMaxIterations = 30;

    cv::Vec3d origin_tvec = {0,0,0};
    cv::Vec3d origin_rvec = {0,0,0};
    cv::Point3f origin(0,0,0);

    // hmath::Quaternion previous(1,0,0,0);

    while (true)
    {
        std::cout << "New image!\n\n\n";
        cv::Mat img = flir::getImage(pCam);
        cv::Mat output_image;
        cv::undistort(img, output_image, cal.cameraMatrix, cal.distCoeffs);
        cv::cvtColor(output_image, output_image, cv::COLOR_GRAY2BGR);
        cv::aruco::detectMarkers(output_image, dictionary, markerCorners, markerIds, parameters);
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cal.cameraMatrix, mv::calibration::idealDistCoeffs, rvecs, tvecs);

        for (int i = 0; i < markerIds.size(); i++)
        {
            std::vector<cv::Point3f> axis_projection;
            axis_projection.push_back(origin);

            // std::cout << "Marker " << markerIds[i] << "!" << std::endl;
            auto rvec = rvecs[i];
            // std::cout << rvec << ", " << sqrt(rvec[0]*rvec[0] + rvec[1]*rvec[1] + rvec[2]*rvec[2]) <<std::endl;
            auto tvec = tvecs[i];
            // std::cout << tvec << std::endl;

            hmath::Quaternion rvec_quaternion = mv::math::quaternionFromRVEC(rvec);
            hmath::Vector3 rotation_axis(rvec_quaternion.i, rvec_quaternion.j, rvec_quaternion.k);
            rotation_axis.normalize();
            rotation_axis *= 0.025;

            cv::Point3f axis(rotation_axis.i, rotation_axis.j, rotation_axis.k);

            axis_projection.push_back(axis);

            std::cout << markerIds[i] << " ";


            // hmath::Quaternion mapping = mv::math::getTTPrimeMappingQuaternion(previous, rvec_quaternion);

            // std::cout << "Previous: " << previous << " Current: " << rvec_quaternion << " TTprim: " << mapping << " RVEC Norm: " << mv::math::cvVec3dNorm(rvec) << std::endl;

            if (mv::math::cvVec3dNorm(rvec) > hmath::H_PI)
            {
                std::cout << "Holllyyyy shit lieutenant, its a value greater than pi! Magnitude is " << mv::math::cvVec3dNorm(rvec) << std::endl;
            }

            // previous = rvec_quaternion;
            
            // cv::aruco::drawAxis(output_image, cal.cameraMatrix, mv::calibration::idealDistCoeffs, rvec, tvec, 0.025);
            // cv::aruco::drawAxis(output_image, cal.cameraMatrix, mv::calibration::idealDistCoeffs, origin_rvec, tvec, 0.025);

            // std::vector<cv::Point2f> image_points;

            // cv::projectPoints(axis_projection, rvec, tvec, cal.cameraMatrix, mv::calibration::idealDistCoeffs, image_points);
            // cv::line(output_image, image_points[0], image_points[1], cv::Scalar(255,0,255), 3);
        }

        std::cout << std::endl;

        // cv::aruco::drawDetectedMarkers(output_image, markerCorners, markerIds);

        cv::resize(output_image, output_image, {0,0}, 0.5, 0.5);

        cv::imshow("test", output_image);
        cv::waitKey(0);
    }   
  
    return 0; 
} 