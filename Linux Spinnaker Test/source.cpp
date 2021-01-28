#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <Spinnaker.h>
#include <SpinnakerDefs.h>
#include <SpinGenApi/SpinnakerGenApi.h>

#ifdef _DEBUG
// Disables heartbeat on GEV cameras so debugging does not incur timeout errors
int DisableHeartbeat(INodeMap& nodeMap, INodeMap& nodeMapTLDevice)
{
    std::cout << "Checking device type to see if we need to disable the camera's heartbeat..." << std::endl << std::endl;

    //
    // Write to boolean node controlling the camera's heartbeat
    //
    // *** NOTES ***
    // This applies only to GEV cameras and only applies when in DEBUG mode.
    // GEV cameras have a heartbeat built in, but when debugging applications the
    // camera may time out due to its heartbeat. Disabling the heartbeat prevents
    // this timeout from occurring, enabling us to continue with any necessary debugging.
    // This procedure does not affect other types of cameras and will prematurely exit
    // if it determines the device in question is not a GEV camera.
    //
    // *** LATER ***
    // Since we only disable the heartbeat on GEV cameras during debug mode, it is better
    // to power cycle the camera after debugging. A power cycle will reset the camera
    // to its default settings.
    //
    CEnumerationPtr ptrDeviceType = nodeMapTLDevice.GetNode("DeviceType");
    if (!IsAvailable(ptrDeviceType) || !IsReadable(ptrDeviceType))
    {
        std::cout << "Error with reading the device's type. Aborting..." << std::endl << std::endl;
        return -1;
    }
    else
    {
        if (ptrDeviceType->GetIntValue() == DeviceType_GigEVision)
        {
            std::cout << "Working with a GigE camera. Attempting to disable heartbeat before continuing..." << std::endl << std::endl;
            CBooleanPtr ptrDeviceHeartbeat = nodeMap.GetNode("GevGVCPHeartbeatDisable");
            if (!IsAvailable(ptrDeviceHeartbeat) || !IsWritable(ptrDeviceHeartbeat))
            {
                std::cout << "Unable to disable heartbeat on camera. Continuing with execution as this may be non-fatal..."
                     << std::endl
                     << std::endl;
            }
            else
            {
                ptrDeviceHeartbeat->SetValue(true);
                std::cout << "WARNING: Heartbeat on GigE camera disabled for the rest of Debug Mode." << std::endl;
                std::cout << "         Power cycle camera when done debugging to re-enable the heartbeat..." << std::endl << std::endl;
            }
        }
        else
        {
            std::cout << "Camera does not use GigE interface. Resuming normal execution..." << std::endl << std::endl;
        }
    }
    return 0;
}
#endif

// This function acquires and saves 10 images from a device.
int AcquireImages(Spinnaker::CameraPtr pCam, Spinnaker::GenApi::INodeMap& nodeMap, Spinnaker::GenApi::INodeMap& nodeMapTLDevice)
{
    int result = 0;

    std::cout << std::endl << std::endl << "*** IMAGE ACQUISITION ***" << std::endl << std::endl;

    try
    {
        //
        // Set acquisition mode to continuous
        //
        // *** NOTES ***
        // Because the example acquires and saves 10 images, setting acquisition
        // mode to continuous lets the example finish. If set to single frame
        // or multiframe (at a lower number of images), the example would just
        // hang. This would happen because the example has been written to
        // acquire 10 images while the camera would have been programmed to
        // retrieve less than that.
        //
        // Setting the value of an enumeration node is slightly more complicated
        // than other node types. Two nodes must be retrieved: first, the
        // enumeration node is retrieved from the nodemap; and second, the entry
        // node is retrieved from the enumeration node. The integer value of the
        // entry node is then set as the new value of the enumeration node.
        //
        // Notice that both the enumeration and the entry nodes are checked for
        // availability and readability/writability. Enumeration nodes are
        // generally readable and writable whereas their entry nodes are only
        // ever readable.
        //
        // Retrieve enumeration node from nodemap
        Spinnaker::GenApi::CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
        if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
        {
            std::cout << "Unable to set acquisition mode to continuous (enum retrieval). Aborting..." << std::endl << std::endl;
            return -1;
        }

        // Retrieve entry node from enumeration node
        Spinnaker::GenApi::CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
        if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
        {
            std::cout << "Unable to set acquisition mode to continuous (entry retrieval). Aborting..." << std::endl << std::endl;
            return -1;
        }

        // Retrieve integer value from entry node
        const int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();

        // Set integer value from entry node as new value of enumeration node
        ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);

        std::cout << "Acquisition mode set to continuous..." << std::endl;

#ifdef _DEBUG
        std::cout << std::endl << std::endl << "*** DEBUG ***" << std::endl << std::endl;

        // If using a GEV camera and debugging, should disable heartbeat first to prevent further issues
        if (DisableHeartbeat(nodeMap, nodeMapTLDevice) != 0)
        {
            return -1;
        }

        std::cout << std::endl << std::endl << "*** END OF DEBUG ***" << std::endl << std::endl;
#endif

        //
        // Begin acquiring images
        //
        // *** NOTES ***
        // What happens when the camera begins acquiring images depends on the
        // acquisition mode. Single frame captures only a single image, multi
        // frame captures a set number of images, and continuous captures a
        // continuous stream of images. Because the example calls for the
        // retrieval of 10 images, continuous mode has been set.
        //
        // *** LATER ***
        // Image acquisition must be ended when no more images are needed.
        //
        pCam->BeginAcquisition();

        std::cout << "Acquiring images..." << std::endl;

        //
        // Retrieve device serial number for filename
        //
        // *** NOTES ***
        // The device serial number is retrieved in order to keep cameras from
        // overwriting one another. Grabbing image IDs could also accomplish
        // this.
        //
        Spinnaker::GenICam::gcstring deviceSerialNumber("");
        Spinnaker::GenApi::CStringPtr ptrStringSerial = nodeMapTLDevice.GetNode("DeviceSerialNumber");
        if (IsAvailable(ptrStringSerial) && IsReadable(ptrStringSerial))
        {
            deviceSerialNumber = ptrStringSerial->GetValue();

            std::cout << "Device serial number retrieved as " << deviceSerialNumber << "..." << std::endl;
        }
        std::cout << std::endl;

        // Retrieve, convert, and save images
        const unsigned int k_numImages = 10;

        for (unsigned int imageCnt = 0; imageCnt < k_numImages; imageCnt++)
        {
            try
            {
                //
                // Retrieve next received image
                //
                // *** NOTES ***
                // Capturing an image houses images on the camera buffer. Trying
                // to capture an image that does not exist will hang the camera.
                //
                // *** LATER ***
                // Once an image from the buffer is saved and/or no longer
                // needed, the image must be released in order to keep the
                // buffer from filling up.
                //
                Spinnaker::ImagePtr pResultImage = pCam->GetNextImage(1000);

                //
                // Ensure image completion
                //
                // *** NOTES ***
                // Images can easily be checked for completion. This should be
                // done whenever a complete image is expected or required.
                // Further, check image status for a little more insight into
                // why an image is incomplete.
                //
                if (pResultImage->IsIncomplete())
                {
                    // Retrieve and print the image status description
                    std::cout << "Image incomplete: " << Spinnaker::Image::GetImageStatusDescription(pResultImage->GetImageStatus())
                         << "..." << std::endl
                         << std::endl;
                }
                else
                {
                    //
                    // Print image information; height and width recorded in pixels
                    //
                    // *** NOTES ***
                    // Images have quite a bit of available metadata including
                    // things such as CRC, image status, and offset values, to
                    // name a few.
                    //
                    const size_t width = pResultImage->GetWidth();

                    const size_t height = pResultImage->GetHeight();

                    std::cout << "Grabbed image " << imageCnt << ", width = " << width << ", height = " << height << std::endl;

                    //
                    // Convert image to mono 8
                    //
                    // *** NOTES ***
                    // Images can be converted between pixel formats by using
                    // the appropriate enumeration value. Unlike the original
                    // image, the converted one does not need to be released as
                    // it does not affect the camera buffer.
                    //
                    // When converting images, color processing algorithm is an
                    // optional parameter.
                    //
                    Spinnaker::ImagePtr convertedImage = pResultImage->Convert(Spinnaker::PixelFormat_Mono8, Spinnaker::HQ_LINEAR);

                    // Create a unique filename
                    std::ostringstream filename;

                    filename << "Acquisition-";
                    if (!deviceSerialNumber.empty())
                    {
                        filename << deviceSerialNumber.c_str() << "-";
                    }
                    filename << imageCnt << ".jpg";

                    //
                    // Save image
                    //
                    // *** NOTES ***
                    // The standard practice of the examples is to use device
                    // serial numbers to keep images of one device from
                    // overwriting those of another.
                    //
                    convertedImage->Save(filename.str().c_str());

                    std::cout << "Image saved at " << filename.str() << std::endl;
                }

                //
                // Release image
                //
                // *** NOTES ***
                // Images retrieved directly from the camera (i.e. non-converted
                // images) need to be released in order to keep from filling the
                // buffer.
                //
                pResultImage->Release();

                std::cout << std::endl;
            }
            catch (Spinnaker::Exception& e)
            {
                std::cout << "Error: " << e.what() << std::endl;
                result = -1;
            }
        }

        //
        // End acquisition
        //
        // *** NOTES ***
        // Ending acquisition appropriately helps ensure that devices clean up
        // properly and do not need to be power-cycled to maintain integrity.
        //

        pCam->EndAcquisition();
    }
    
    catch (Spinnaker::Exception& e)
    {
        std::cout << "Error: " << e.what() << std::endl;
        return -1;
    }

    return result;
}

// This function prints the device information of the camera from the transport
// layer; please see NodeMapInfo example for more in-depth comments on printing
// device information from the nodemap.
int PrintDeviceInfo(Spinnaker::GenApi::INodeMap& nodeMap)
{
    int result = 0;
    std::cout << std::endl << "*** DEVICE INFORMATION ***" << std::endl << std::endl;

    try
    {
        Spinnaker::GenApi::FeatureList_t features;
        const Spinnaker::GenApi::CCategoryPtr category = nodeMap.GetNode("DeviceInformation");
        if (IsAvailable(category) && IsReadable(category))
        {
            category->GetFeatures(features);

            for (auto it = features.begin(); it != features.end(); ++it)
            {
                const Spinnaker::GenApi::CNodePtr pfeatureNode = *it;
                std::cout << pfeatureNode->GetName() << " : ";
                Spinnaker::GenApi::CValuePtr pValue = static_cast<Spinnaker::GenApi::CValuePtr>(pfeatureNode);
                std::cout << (IsReadable(pValue) ? pValue->ToString() : "Node not readable");
                std::cout << std::endl;
            }
        }
        else
        {
            std::cout << "Device control information not available." << std::endl;
        }
    }
    catch (Spinnaker::Exception& e)
    {
        std::cout << "Error: " << e.what() << std::endl;
        result = -1;
    }

    return result;
}

// This function acts as the body of the example; please see NodeMapInfo example
// for more in-depth comments on setting up cameras.
int RunSingleCamera(Spinnaker::CameraPtr pCam)
{
    int result;

    try
    {
        // Retrieve TL device nodemap and print device information
        Spinnaker::GenApi::INodeMap& nodeMapTLDevice = pCam->GetTLDeviceNodeMap();

        result = PrintDeviceInfo(nodeMapTLDevice);

        // Initialize camera
        pCam->Init();

        // Retrieve GenICam nodemap
        Spinnaker::GenApi::INodeMap& nodeMap = pCam->GetNodeMap();

        // Acquire images
        result = result | AcquireImages(pCam, nodeMap, nodeMapTLDevice);

        // Deinitialize camera
        pCam->DeInit();
    }
    catch (Spinnaker::Exception& e)
    {
        std::cout << "Error: " << e.what() << std::endl;
        result = -1;
    }

    return result;
}

// Example entry point; please see Enumeration example for more in-depth
// comments on preparing and cleaning up the system.
int main(int /*argc*/, char** /*argv*/)
{
    // Retrieve singleton reference to system object
    Spinnaker::SystemPtr system = Spinnaker::System::GetInstance();

    // Retrieve list of cameras from the system
    Spinnaker::CameraList camList = system->GetCameras();

    const unsigned int numCameras = camList.GetSize();

    std::cout << "Number of cameras detected: " << numCameras << std::endl << std::endl;

    // Finish if there are no cameras
    if (numCameras == 0)
    {
        // Clear camera list before releasing system
        camList.Clear();

        // Release system
        system->ReleaseInstance();

        std::cout << "Not enough cameras!" << std::endl;
        std::cout << "Done! Press Enter to exit..." << std::endl;
        getchar();

        return -1;
    }

    //
    // Create shared pointer to camera
    //
    // *** NOTES ***
    // The CameraPtr object is a shared pointer, and will generally clean itself
    // up upon exiting its scope. However, if a shared pointer is created in the
    // same scope that a system object is explicitly released (i.e. this scope),
    // the reference to the shared point must be broken manually.
    //
    // *** LATER ***
    // Shared pointers can be terminated manually by assigning them to nullptr.
    // This keeps releasing the system from throwing an exception.
    //
    Spinnaker::CameraPtr pCam = nullptr;

    int result = 0;

    // Run example on each camera
    for (unsigned int i = 0; i < numCameras; i++)
    {
        // Select camera
        pCam = camList.GetByIndex(i);

        std::cout << std::endl << "Running example for camera " << i << "..." << std::endl;

        // Run example
        result = result | RunSingleCamera(pCam);

        std::cout << "Camera " << i << " example complete..." << std::endl << std::endl;
    }

    //
    // Release reference to the camera
    //
    // *** NOTES ***
    // Had the CameraPtr object been created within the for-loop, it would not
    // be necessary to manually break the reference because the shared pointer
    // would have automatically cleaned itself up upon exiting the loop.
    //
    pCam = nullptr;

    // Clear camera list before releasing system
    camList.Clear();

    // Release system
    system->ReleaseInstance();

    std::cout << std::endl << "Done! Press Enter to exit..." << std::endl;
    getchar();

    return result;
}