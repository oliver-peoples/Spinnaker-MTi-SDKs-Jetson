#include "calibration_includes.h"

// Avast ye mateys. I've had a fair bit of wine, and I'm now barreling throughs my gins faster than a rail gun shot through
// a half foot thick steel plate, and am watching pirates of the caribbean and coding. As ye sit high, upon
// that tall, sober, horse of yours, and gaze upon my finely written shitshow code, I suggest you keep in mind an adapted quote, taken
// from the fine theatrical experience I have sitting to the left of me on my iPad:

// As a rather smolderingly attractive fictional british 1800's navy commander said based off first impressions:

// "That might just be the worst pirate [mathematician] I've ever seen."

// Then, in a few seconds, as he saw the pirate's [mathematician's] work, he later said:

// "That might just be the best pirate [mathematician] I've ever seen."

// And as a rather drunk pirate [mathematician] of the same era once said

// "...but you have heard of me."

// *drum roll* and now, a Dasher and Fisher gin and tonic presents, a fine coordinate system mapping quaternion calculator...* 

struct AAP
{
    hmath::Quaternion aq, aq_prime;
    hmath::Quaternion gq, gq_prime;
    hmath::Quaternion a_map, g_map;
};

class CallbackHandler : public XsCallback
{
public:
	CallbackHandler(size_t maxBufferSize = 5)
		: m_maxNumberOfPacketsInBuffer(maxBufferSize)
		, m_numberOfPacketsInBuffer(0)
	{
	}

	virtual ~CallbackHandler() throw()
	{
	}

	bool packetAvailable() const
	{
		xsens::Lock locky(&m_mutex);
		return m_numberOfPacketsInBuffer > 0;
	}

	XsDataPacket getNextPacket()
	{
		assert(packetAvailable());
		xsens::Lock locky(&m_mutex);
		XsDataPacket oldestPacket(m_packetBuffer.front());
		m_packetBuffer.pop_front();
		--m_numberOfPacketsInBuffer;
		return oldestPacket;
	}

protected:
	virtual void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet)
	{
		xsens::Lock locky(&m_mutex);
		assert(packet != nullptr);
		while (m_numberOfPacketsInBuffer >= m_maxNumberOfPacketsInBuffer)
			(void)getNextPacket();

		m_packetBuffer.push_back(*packet);
		++m_numberOfPacketsInBuffer;
		assert(m_numberOfPacketsInBuffer <= m_maxNumberOfPacketsInBuffer);
	}
private:
	mutable xsens::Mutex m_mutex;

	size_t m_maxNumberOfPacketsInBuffer;
	size_t m_numberOfPacketsInBuffer;
	std::list<XsDataPacket> m_packetBuffer;
};

auto handleError(std::string errorString, XsControl* control)
{
	control->destruct();
	std::cout << errorString << std::endl;
	std::cout << "Press [ENTER] to continue." << std::endl;
	std::cin.get();
	return -1;
}

int main()
{
    //---------------------------------------------------------------------------------------------
    // XSENS AHRS CONFIGURATION
    //---------------------------------------------------------------------------------------------

	XsControl* control = XsControl::construct();
	assert(control != nullptr);

	XsPortInfoArray portInfoArray = XsScanner::scanPorts();

	XsPortInfo mtPort;
	for (auto const &portInfo : portInfoArray)
	{
		if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig())
		{
			mtPort = portInfo;
			break;
		}
	}

	if (mtPort.empty())
    {
		return handleError("No MTi device found. Aborting.", control);
    }

	std::cout << "Found a device with ID: " << mtPort.deviceId().toString().toStdString() << " @ port: " << mtPort.portName().toStdString() << ", baudrate: " << mtPort.baudrate() << std::endl;

	std::cout << "Opening port..." << std::endl;
	if (!control->openPort(mtPort.portName().toStdString(), mtPort.baudrate()))
    {
		return handleError("Could not open port. Aborting.", control);
    }

	XsDevice* device = control->device(mtPort.deviceId());
	assert(device != nullptr);

	CallbackHandler callback;
	device->addCallbackHandler(&callback);

	std::cout << "Putting device into configuration mode..." << std::endl;
	if (!device->gotoConfig())
    {
		return handleError("Could not put device into configuration mode. Aborting.", control);
    }

	std::cout << "Configuring the device..." << std::endl;
	XsOutputConfigurationArray configArray;
	configArray.push_back(XsOutputConfiguration(XDI_PacketCounter, 0));
	configArray.push_back(XsOutputConfiguration(XDI_SampleTimeFine, 0));
	configArray.push_back(XsOutputConfiguration(XDI_Quaternion, 0));

	if (!device->setOutputConfiguration(configArray))
    {
		return handleError("Could not configure MTi device. Aborting.", control);
    }

	std::cout << "Putting device into measurement mode..." << std::endl;
	if (!device->gotoMeasurement())
    {
		return handleError("Could not put device into measurement mode. Aborting.", control);
    }

    //---------------------------------------------------------------------------------------------
    // FLIR CHAMELEON CAMERA CONNECTION
    //---------------------------------------------------------------------------------------------

    // Get camera

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

    //---------------------------------------------------------------------------------------------
    // MAIN PROCEDURES
    //---------------------------------------------------------------------------------------------

    // Aruco detector setup

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    parameters->cornerRefinementMaxIterations = 30;

    mv::calibration::calibration cal = mv::calibration::loadCalibration();

    AAP aap_array[250];

    for (int i = 0; i < 250; i++)
    {
        AAP* aap = &aap_array[i];

        aap->aq_prime.w = 1;
        aap->aq_prime.i = 0;
        aap->aq_prime.j = 0;
        aap->aq_prime.k = 0;

        aap->gq_prime.w = 1;
        aap->gq_prime.i = 0;
        aap->gq_prime.j = 0;
        aap->gq_prime.k = 0;
    }
    
    // Begin alignment process

    std::cout << "\n" << std::string(80, '~') << "\n" << "Beginning Data Receipt..." << "\n" << std::string(80, '~');
	std::cout << std::endl;

	while (true)
	{
		if (callback.packetAvailable())
		{
            // Getchoseffs a fine camara imuj bby

            cv::Mat img = flir::getImage(pCam);

            // Now getchoseffs a fine quaternion hun

            XsDataPacket packet = callback.getNextPacket();
            XsQuaternion xs_qt = packet.orientationQuaternion();

            // Mmmm damn, look at that data ungghhh. Now find yo selfs some aruco markers shugah

            cv::Mat output_image;
            cv::undistort(img, output_image, cal.cameraMatrix, cal.distCoeffs);
            cv::cvtColor(output_image, output_image, cv::COLOR_GRAY2BGR);

            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f>> markerCorners;
            
            cv::aruco::detectMarkers(output_image, dictionary, markerCorners, markerIds, parameters);

            std::vector<cv::Vec3d> rvecs, tvecs;

            cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cal.cameraMatrix, mv::calibration::idealDistCoeffs, rvecs, tvecs);

            for (int i = 0; i < markerIds.size(); i++)
            {
                hmath::Quaternion quaternion = mv::math::quaternionFromRVEC(rvecs[i]);
                cv::aruco::drawAxis(output_image, cal.cameraMatrix, mv::calibration::idealDistCoeffs, rvecs[i], tvecs[0], 0.025);

                AAP* aap = &aap_array[markerIds[i]];

                if (aap->aq_prime.isIdentity() && aap->gq_prime.isIdentity())
                {
                    aap->aq_prime = mv::math::quaternionFromRVEC(rvecs[i]);
                    aap->gq_prime = hmath::Quaternion(xs_qt.w(), xs_qt.x(), xs_qt.y(), xs_qt.z());
                }
                
                else if (!(aap->aq_prime.isIdentity() && aap->gq_prime.isIdentity()))
                {
                    aap->aq = aap->aq_prime;
                    aap->gq = aap->gq_prime;
                }
            }

            mv::cvTools::naiveShow(output_image, 1);
		}
	}

	std::cout << "\n" << std::string(80, '~') << "\n" << "End Data Receipt..." << "\n" << std::string(80, '~');
	std::cout << std::endl;

	// Nothing in here 268 - 289

	std::cout << "Stopping recording..." << std::endl;
	if (!device->stopRecording())
		return handleError("Failed to stop recording. Aborting.", control);

	std::cout << "Closing log file..." << std::endl;
	if (!device->closeLogFile())
		return handleError("Failed to close log file. Aborting.", control);

	std::cout << "Closing port..." << std::endl;
	control->closePort(mtPort.portName().toStdString());

	std::cout << "Freeing XsControl object..." << std::endl;
	control->destruct();

	std::cout << "Successful exit." << std::endl;

	std::cout << "Press [ENTER] to continue." << std::endl;
	std::cin.get();

	// Nothing in here 268 - 289

	return 0;

    // // Aruco Stuff

    // while (true)
    // {
    //     cv::Mat img = flir::getImage(pCam);
    //     XsDataPacket packet = callback.getNextPacket();
    //     XsQuaternion quaternion = packet.orientationQuaternion();

    //     cv::Mat output_image;
    //     cv::undistort(img, output_image, cal.cameraMatrix, cal.distCoeffs);
    //     cv::cvtColor(output_image, output_image, cv::COLOR_GRAY2BGR);

    //     std::vector<int> markerIds;
    //     std::vector<std::vector<cv::Point2f>> markerCorners;
        
    //     cv::aruco::detectMarkers(output_image, dictionary, markerCorners, markerIds, parameters);

    //     std::vector<cv::Vec3d> rvecs, tvecs;

    //     cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cal.cameraMatrix, mv::calibration::idealDistCoeffs, rvecs, tvecs);

    //     for (int idx = 0; idx < markerIds.size(); idx++)
    //     {
    //         AAP* aap = &aap_array[markerIds[idx]];

    //         if (aap->aq.isIdentity() && aap_array->gq.isIdentity())
    //         {
    //             std::cout << "We ain't seen this guy before!" << std::endl;

    //             aap->aq = mv::math::quaternionFromRVEC(rvecs[idx]);
    //         }
    //     }
    // }
}

// * This only works if you have an Xsens MTi-28 (about 7k aud) and a flir chameleon something or other (2k aud)