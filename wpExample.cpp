#include <iostream>
#include <iomanip>
#include <string>
#include <cstring>
#include <math.h>
#include <limits>

#include "DJIHardDriverManifold.h"
#include "conboardsdktask.h"
#include "APIThread.h"
#include "User_Config.h"
#include "LinuxWaypoint.h"
#include "unistd.h"

#include <DJI_Version.h>

using namespace std;
using namespace DJI;
using namespace DJI::onboardSDK;

int attemptControl(CoreAPI* api);
void loadWaypoints(CoreAPI* api, WayPoint* waypointObj, int numWaypoints,
		int timeout);
float64_t radsToDegrees(float64_t rads) {
	return rads * 360 / (2 * M_PI);
}
float64_t degreesToRads(float64_t degrees) {
	return degrees * 2 * M_PI / 360;
}

//! Main function for the command line sample. Makes heavy use of DJI_Script.
int main(int argc, char *argv[]) {
	std::string droneVerStr, SdkVerStr;
	DJI::onboardSDK::ActivateData adata;

	string userKey ="";
	char userKeyCstr[65];
	strcpy(userKeyCstr, userKey.c_str());

	adata.ID = 0;
	adata.encKey = userKeyCstr;

	//! Introductory messages
	int initialConfirmation;

	switch (targetVersion) {
	case DJI::onboardSDK::versionA3_31:
		droneVerStr = "A3";
		SdkVerStr = "3.1";
		break;
	case DJI::onboardSDK::versionM100_31:
		droneVerStr = "M100";
		SdkVerStr = "3.1";
		break;
	case DJI::onboardSDK::versionM100_23:
		droneVerStr = "M100";
		SdkVerStr = "2.3";
		break;
	default:
		droneVerStr = "Error. Please check your User_Config file.";
		SdkVerStr = "Error. Please check your User_Config file.";
		break;
	}

	//! @note replace these two lines below to change to another serial device driver.
	HardDriverManifold driver(deviceName, baudRate);
	driver.init();

	//! @note Instantiate a CoreAPI with this hardware driver and a Script with this CoreAPI.
	CoreAPI api(&driver);
	Flight* flight = new Flight(&api);
	WayPoint* waypointObj = new WayPoint(&api);

	//! @note Tell the SDK our hardware and software versions
	api.setVersion(targetVersion);

	//! @note replace these four lines below to change to another serial device driver. Currently, the manifold driver works for all linux setups.
	APIThread send(&api, 1);
	APIThread read(&api, 2);
	send.createThread();
	read.createThread();

	//! @note Sequence for automatic activation. Reuse the --SS load fuctionality
	std::cout << "\nAttempting automatic activation..\n";

	uint8_t activationStatus = 99;
	activationStatus = api.activate(&adata, 1);
	//! Give the callback time to return
	usleep(500000);

	//! See what value the callback set
	switch (activationStatus) {
	case ACK_ACTIVE_SUCCESS:
		std::cout << "Automatic activation successful." << std::endl;
		usleep(3000000);
		break;
	case ACK_ACTIVE_NEW_DEVICE:
		std::cout
				<< "Your activation did not go through. \nThis is a new device. \nMake sure DJI GO is turned on and connected to the internet \nso you can contact the server for activation. \nActivate with '--SS load ../key.txt' followed by '--CA ac'."
				<< std::endl;
		usleep(3000000);
		break;
	case ACK_ACTIVE_PARAMETER_ERROR:
		std::cout
				<< "Your activation did not go through. \nThere was a parameter error. \nPlease check your setup and retry."
				<< std::endl;
		usleep(3000000);
		break;
	case ACK_ACTIVE_ENCODE_ERROR:
		std::cout
				<< "Your activation did not go through. \nThere was an encoding error. \nPlease check your setup and retry."
				<< std::endl;
		usleep(3000000);
		break;
	case ACK_ACTIVE_APP_NOT_CONNECTED:
		std::cout
				<< "Your activation did not go through. \nDJI Go does not seem to be connected. \nPlease check your mobile device and activate with '--SS load ../key.txt' followed by '--CA ac'."
				<< std::endl;
		usleep(3000000);
		break;
	case ACK_ACTIVE_NO_INTERNET:
		std::cout
				<< "Your activation did not go through. \nYour mobile device doesn't seem to be connected to the internet. \nPlease check your connection and activate with '--SS load ../key.txt' followed by '--CA ac'."
				<< std::endl;
		usleep(3000000);
		break;
	case ACK_ACTIVE_SERVER_REFUSED:
		std::cout
				<< "Your activation did not go through. \nThe server refused your credentials. \nPlease check your DJI developer account details in ../key.txt."
				<< std::endl;
		usleep(3000000);
		break;
	case ACK_ACTIVE_ACCESS_LEVEL_ERROR:
		std::cout
				<< "Your activation did not go through. \nYou don't seem to have the right DJI SDK permissions. \nPlease check your DJI developer account details."
				<< std::endl;
		usleep(3000000);
		break;
	case ACK_ACTIVE_VERSION_ERROR:
		std::cout
				<< "Your activation did not go through. \nYour SDK version in User_config.h does not match the one reported by the drone. \nPlease correct that, rebuild and activate."
				<< std::endl;
		usleep(3000000);
		break;
	default:
		std::cout
				<< "There was an error with the activation command. Perhaps your drone is not powered on? \nPlease check your setup and retry."
				<< std::endl;
		usleep(3000000);
		break;
	}

	//std::cout << "\nAttempting to take control\n";

	int controlAck = 0;
	ackReturnData wpAck;
	int numWaypoints = 4;
	int blockingTime = 1;
	controlAck = attemptControl(&api);
	DJI::onboardSDK::BroadcastData data;
	float64_t latitude, longitude;
	streamsize oldss = std::cout.precision();
	std::cout << setprecision(numeric_limits<long double>::digits10 + 1);
	while (1) {

		data = api.getBroadcastData();
		latitude = radsToDegrees(data.pos.latitude);
		longitude = radsToDegrees(data.pos.longitude);
		std::cout << "Time: " << data.timeStamp.time << endl;
		std::cout << "Velocity x (m/s): " << data.v.x << endl;
		std::cout << "Velocity y (m/s): " << data.v.y << endl;
		std::cout << "Velocity z (m/s): " << data.v.z << endl;
		std::cout << "Latitude (degrees): " << latitude << endl;
		std::cout << "Longitude (degrees): " << longitude << endl;
		std::cout << "Altitude (m): " << data.pos.altitude << endl;
		std::cout << "Height (m): " << data.pos.height << endl;
		std::cout << "GPS Health: " << (int) data.pos.health << endl;
		std::cout << "Magnetometer x: " << data.mag.x << endl;
		std::cout << "Magnetometer y: " << data.mag.y << endl;
		std::cout << "Magnetometer z: " << data.mag.z << endl;
		std::cout << "Accelerometer x: " << data.a.x << endl;
		std::cout << "Accelerometer y: " << data.a.y << endl;
		std::cout << "Accelerometer z: " << data.a.z << endl;
		std::cout << "World x: " << data.w.x << endl;
		std::cout << "World y: " << data.w.y << endl;
		std::cout << "World z: " << data.w.z << endl;
		std::cout << "Battery (%): " << (int) data.battery << endl;
		std::cout << "Device Status: " << (int) data.ctrlInfo.deviceStatus
				<< endl;
		std::cout << "Radio Mode: " << data.rc.mode << endl; //8000 = F, 0 = A, -8000 = P
		std::cout << "Control Mode: " << (int) data.ctrlInfo.mode << endl;
		std::cout << endl;

		if (controlAck == 1) {

			//std::cout << "Flight Status: " << status << endl;

			wpAck = initWaypointMission(&api, waypointObj, numWaypoints,
					blockingTime);
			std::cout << "Mission Status: " << (int) wpAck.ack << endl;
			if ((int) wpAck.ack == 0) {
				loadWaypoints(&api, waypointObj, numWaypoints, blockingTime);
				std::cout << "Starting Waypoint mission\n";
				wpAck = startWaypointMission(&api, waypointObj, blockingTime);
			}
			//usleep(5000000);
		}
		if ((int) data.ctrlInfo.deviceStatus != 2 && data.rc.mode == 8000) {
			controlAck = attemptControl(&api);
		}
		usleep(500000);
	}

}

int attemptControl(CoreAPI* api) {
	uint8_t controlStatus = 99;
	controlStatus = api->setControl(true, 1);
	//! Give the callback time to return
	//usleep(500000);
	switch (controlStatus) {
	case ACK_SETCONTROL_NEED_MODE_F:
		std::cout
				<< "Failed to obtain control.\nYour RC mode switch is not in mode F. (Is the RC connected and paired?)"
				<< std::endl;
		return -1;
		break;
	case ACK_SETCONTROL_OBTAIN_SUCCESS:
		std::cout << "Obtained control successfully." << std::endl;
		return 1;
		break;
	case ACK_SETCONTROL_OBTAIN_RUNNING:
		std::cout << "Obtain control running.." << std::endl;
		return -1;
		break;
	case ACK_SETCONTROL_IOC:
		std::cout
				<< "The aircraft is in IOC mode. Cannot obtain control.\nGo to DJI GO and stop all intelligent flight modes before trying this."
				<< std::endl;
		return -1;
		break;
	default:
		std::cout << "Error in setControl API function." << std::endl;
		return -1;
		break;
	}
}

void loadWaypoints(CoreAPI* api, WayPoint* waypointObj, int numWaypoints,
		int timeout) {
	// example: just creates a box of waypoints from original position
	//! Get current GPS location
	PositionData curPosition = api->getBroadcastData().pos;
	//! Create an offset in radians for the waypoint creation loop to access and permute.
	float64_t offsetInRadians = 0.000003;
	//! Waypoint creation and upload
	for (int index = 0; index < numWaypoints; index++) {
		PositionData wpPosition;
		switch (index) {
		case 0:
			wpPosition = curPosition;
			wpPosition.latitude += offsetInRadians;
			wpPosition.altitude = 20;
			break;
		case 1:
			wpPosition = curPosition;
			wpPosition.latitude += offsetInRadians;
			wpPosition.longitude += offsetInRadians;
			wpPosition.altitude = 20;
			break;
		case 2:
			wpPosition = curPosition;
			wpPosition.longitude += offsetInRadians;
			wpPosition.altitude = 20;
			break;
		case 3:
			wpPosition = curPosition;
			wpPosition.altitude = 20;
			break;
		}
		//std::cout << "Adding waypoint " << index << "\n";
		ackReturnData wpAck = addWaypoint(api, waypointObj, &wpPosition,
				(uint8_t) index, timeout);
	}
}

