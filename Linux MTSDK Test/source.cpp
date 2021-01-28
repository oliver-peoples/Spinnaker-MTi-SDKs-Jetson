//  Copyright (c) 2003-2020 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

//--------------------------------------------------------------------------------
// Xsens device API C++ example MTi receive data.
//--------------------------------------------------------------------------------
#include <xsensdeviceapi.h>
#include <xstypes/xstime.h>
#include <xscommon/xsens_mutex.h>

#include <iostream>
#include <iomanip>
#include <list>
#include <string>

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

//--------------------------------------------------------------------------------
int main()
{
	std::cout << "Creating XsControl object..." << std::endl;
	XsControl* control = XsControl::construct();
	assert(control != nullptr);

	std::cout << "Scanning for devices..." << std::endl;
	XsPortInfoArray portInfoArray = XsScanner::scanPorts();

	// Find an MTi device
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
		return handleError("No MTi device found. Aborting.", control);

	std::cout << "Found a device with ID: " << mtPort.deviceId().toString().toStdString() << " @ port: " << mtPort.portName().toStdString() << ", baudrate: " << mtPort.baudrate() << std::endl;

	std::cout << "Opening port..." << std::endl;
	if (!control->openPort(mtPort.portName().toStdString(), mtPort.baudrate()))
		return handleError("Could not open port. Aborting.", control);

	// Get the device object
	XsDevice* device = control->device(mtPort.deviceId());
	assert(device != nullptr);

	// Create and attach callback handler to device
	CallbackHandler callback;
	device->addCallbackHandler(&callback);

	// Put the device into configuration mode before configuring the device
	std::cout << "Putting device into configuration mode..." << std::endl;
	if (!device->gotoConfig())
		return handleError("Could not put device into configuration mode. Aborting.", control);

	std::cout << "Configuring the device..." << std::endl;
	XsOutputConfigurationArray configArray;
	configArray.push_back(XsOutputConfiguration(XDI_PacketCounter, 0));
	configArray.push_back(XsOutputConfiguration(XDI_SampleTimeFine, 0));
	configArray.push_back(XsOutputConfiguration(XDI_Quaternion, 0));

	if (!device->setOutputConfiguration(configArray))
		return handleError("Could not configure MTi device. Aborting.", control);

	std::cout << "Putting device into measurement mode..." << std::endl;
	if (!device->gotoMeasurement())
		return handleError("Could not put device into measurement mode. Aborting.", control);

	while (true)
	{
		if (callback.packetAvailable())
		{
			std::cout << std::setw(5) << std::fixed << std::setprecision(2);

			// Retrieve a packet
			XsDataPacket packet = callback.getNextPacket();

			if (packet.containsCalibratedData())
			{
				XsVector acc = packet.calibratedAcceleration();
				std::cout << "\r"
					<< "Acc X:" << acc[0]
					<< ", Acc Y:" << acc[1]
					<< ", Acc Z:" << acc[2];

				XsVector gyr = packet.calibratedGyroscopeData();
				std::cout << " |Gyr X:" << gyr[0]
					<< ", Gyr Y:" << gyr[1]
					<< ", Gyr Z:" << gyr[2];

				XsVector mag = packet.calibratedMagneticField();
				std::cout << " |Mag X:" << mag[0]
					<< ", Mag Y:" << mag[1]
					<< ", Mag Z:" << mag[2];
			}

			if (packet.containsOrientation())
			{
				XsQuaternion quaternion = packet.orientationQuaternion();
				std::cout << "\r"
					<< "q0:" << quaternion.w()
					<< ", q1:" << quaternion.x()
					<< ", q2:" << quaternion.y()
					<< ", q3:" << quaternion.z();

				XsEuler euler = packet.orientationEuler();
				std::cout << " |Roll:" << euler.roll()
					<< ", Pitch:" << euler.pitch()
					<< ", Yaw:" << euler.yaw();
			}

			if (packet.containsLatitudeLongitude())
			{
				XsVector latLon = packet.latitudeLongitude();
				std::cout << " |Lat:" << latLon[0]
					<< ", Lon:" << latLon[1];
			}

			if (packet.containsAltitude())
				std::cout << " |Alt:" << packet.altitude();

			if (packet.containsVelocity())
			{
				XsVector vel = packet.velocity(XDI_CoordSysEnu);
				std::cout << " |E:" << vel[0]
					<< ", N:" << vel[1]
					<< ", U:" << vel[2];
			}

			std::cout << std::flush;
		}
		XsTime::msleep(0);
	}
	std::cout << "\n" << std::string(79, '-') << "\n";
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
}