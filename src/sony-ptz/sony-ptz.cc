/*
 *  SonyPTZ v1.2
 *
 *  A Player/Stage Plugin Driver for Sony PTZ Cameras
 *
 *  Copyright (C) 2015 Oliver Saunders (o.saunders at me.com)
 *  Based on previous work by Brian Gerkey & Brad Tonkes, et al.
 *  This program presents a refactored sonyevid30 driver with 
 *  significant additional documentation, bug fixes, design changes, new features and an original calibration function
 *  The original driver has been modified and used under the terms of the GNU Public License 
 *  and full attribution and gratitude is hereby given to the original developers for the their work 
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 */

/*
 * $Id: sony-ptz.cc 2015-08-18 20:08 OMCS $
 *
 * Alternative driver for initializing, sending commands to and receiving data from Sony PTZ cameras. Based on sonyevid30 and tested with the EVI-D30, EVI-D70 and EVI-D100 cameras
 * 
 */

/** @ingroup drivers */
/** @{ */
/** @defgroup driver_sony-ptz sony-ptz
 * @brief Sony EVI-D30, EVI-D70 and EVI-D100 pan-tilt-zoom cameras

The sony-ptz driver is an improved version of sonyevid30 and connects to a Sony PTZ camera 
via a serial port to allow control of pan and tilt position / speed and zoom position.
Control of zoom speed is implemented although this requires a modified version of Player.

@par Compile-time dependencies

- none

@par Provides

- @ref interface_ptz

@par Requires

- None

@par Configuration requests

- PLAYER_PTZ_REQ_GENERIC

@par Configuration file options

- port (string)
  - Default: "/dev/ttyUSB0"
  - The serial port the PTZ camera is connected to.

- demo (integer)
    - Default: 0
    - If set to 1, the demo function will run after the driver initialises

@par Example

@verbatim
driver
(
  name "sony-ptz"
  provides ["ptz:0"]
  port "/dev/ttyUSB0"
  alwayson 1
  demo 0
)
@endverbatim

@author Oliver Saunders

*/
/** @} */

#include <iostream> /* Print statements */
#include <sstream> /* Used for integer to string conversion */
#include <fcntl.h> /* File IO and flags */
#include <unistd.h> /* Close() function */
#include <termios.h> /* Defines terminal control structure and POSIX control functions for serial port access */
#include <cmath> /* Mathematical functions for use with conversions to / from camera units and radians */
#include <poll.h> /* Provides the poll() function, pollfd struct and assorted data types */
#include <libplayercore/playercore.h> /* Access to the Player API */

/* Function to convert int to string using a std::ostringstream */
#define TOSTR(intValue, suffix) dynamic_cast <std::ostringstream&> (std::ostringstream() << std::dec << intValue).str() + suffix;

/* If no value is found in the config file, set a sensible default value */
#define DEFAULT_SERIAL_PORT "/dev/ttyUSB0"

/* Compile-time options */
//#define DEMO_MODE // If set, the camera will perform a demo routine after calibration and then exit
//#define CAMERA_SHUTDOWN // If set, the camera will attempt to power down when a client unsubscribes
//#define DEBUG_POWER // If set, the VISCA packet for the reply to a power inquiry will be printed to stdout to assist with debugging, the EVI-D70P does not seem to return accurate data from a CAM_PowerInq

/* Define the time to sleep between reading for data in microseconds */
#define SLEEP_TIME_MS 100000

/* Mathematical constants */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* VISCA Packet constants */
#define MAX_PTZ_PACKET_LENGTH 16 
#define MAX_PTZ_MESSAGE_LENGTH 14
#define MAX_PTZ_REPLY_LENGTH 11 // Used for polling, for the size of packets that are replies to inquiries, use MAX_PTZ_PACKET_LENGTH
#define MIN_PTZ_REPLY_LENGTH 4
#define MIN_PTZ_INQUIRY_LENGTH 3

#define VISCA_COMMAND_BYTE 0x01 // '0x81 0x01' is a command
#define VISCA_INQUIRY_BYTE 0x09 // '0x81 0x09' is an inquiry

#define COMMAND_COMPLETE_CODE 0xFF // Arbitrary value returned by the ReceiveVISCAPacket() function if a command completion is received

#define CENTER 0x0000 // Simple definition for the centre position for both pan and tilt in all supported cameras, makes movement commands more self-explanatory

#define PTZ_PROXY_GET_STATUS_SUBTYPE 5 // This constant represents the subtype for a GetStatus() message in PtzProxy

/* PTZ Camera model identifiers */
#define  D30_MODEL_HEX 0x0402
#define  D70_MODEL_HEX 0x040E
#define D100_MODEL_HEX 0x040D

/* Define minimum and maximum pan and tilt in degrees for supported cameras, these will be used during calibration to set the correct conversion factor
 * Values are taken from official Sony documentation 
 */
#define D30_MIN_PAN_DEGREES  -100.0
#define D30_MAX_PAN_DEGREES   100.0
#define D30_MIN_TILT_DEGREES -25.0
#define D30_MAX_TILT_DEGREES  25.0

#define D70_MIN_PAN_DEGREES -170.0
#define D70_MAX_PAN_DEGREES  170.0
#define D70_MIN_TILT_DEGREES -30.0
#define D70_MAX_TILT_DEGREES  90.0

#define D100_MIN_PAN_DEGREES -100.0
#define D100_MAX_PAN_DEGREES  100.0
#define D100_MIN_TILT_DEGREES -25.0
#define D100_MAX_TILT_DEGREES  25.0

/* Zoom Field of View and Speed Definitions */

/* Field of view values in degrees, assuming unmodified lenses. Values used were found in the technical specifications - e.g. pg. 7 of the EVI-D70P manual */
#define D30_MIN_ZOOM_FOV 4.4
#define D30_MAX_ZOOM_FOV 48.8

#define D70_MIN_ZOOM_FOV 2.7
#define D70_MAX_ZOOM_FOV 48.0

#define D100_MIN_ZOOM_FOV 6.6
#define D100_MAX_ZOOM_FOV 65.0

#define D30_MIN_ZOOM_SPEED  0x02
#define D70_MIN_ZOOM_SPEED  0x01
#define D100_MIN_ZOOM_SPEED 0x01

#define MAX_ZOOM_SPEED 0x07

/* VISCA Protocol Information
 * VISCA is a serial protocol used for communicating with Sony PTZ cameras. Transmitted information is either a 'command' or an 'inquiry'
 * The basic unit of data is a packet, a packet can be from 3 to 16 bytes in length and comprises a header, message of between 1 and 14 bytes
 * and a terminator.
 *
 * Headers comprise the sender and receivers' addresses, the controller (computer sending commands) always has address 0 and cameras range from 1 to 7
 * 0x81 would be the header for a packet sent from the controller to camera 1, 0x90 would be the reply sent from camera 1 to the controller because the address in the reply packet is the address of the camera replying + 8
 * Packets can be broadcast from the controller to all connected cameras using address 0x88 in the header 
 * 
 * The message in a packet being sent from the controller will specify whether it is a command or inquiry packet, which category it has and the specific command or inquiry to send
 * The message in a reply packet being sent from a camera to the controller will be 0xX0 (X = address of camera + 8) 0x4Y for an ACK or 0x5Y for a command / inquiry reply 0xFF
 * Y is the socket number (see the VISCA specification for more details on this)
 *
 * The terminator is always 0xFF and signifies the end of a packet
 *
 */

/* The following section defines arrays which store speed data for pan and tilt motions 
 * This data is used for converting speed values in camera units (the D70 ranges from 0x00 to 0x18) to values in degrees per second, which we then convert to radians
 * These values are available in the manuals for EVI cameras - e.g. pg. 57 in the D70-P manual lists values for both the D70 and D30
 * Because only the D30 has a linear speed gradient, it is impractical to use calibration to obtain these values
 * Therefore the values provided by Sony are included here. In future versions it would be sensible to handle this during calibration
 */

/* All possible pan speeds (in radians per second) */

const static float D30_PAN_SPEEDS[] =
{ 
    DTOR(0.0),  DTOR(3.3),  DTOR(6.7),  DTOR(10.0), DTOR(13.3), DTOR(16.7),
    DTOR(20.0), DTOR(23.3), DTOR(26.7), DTOR(30.0), DTOR(33.3), DTOR(36.7),
    DTOR(40.0), DTOR(43.3), DTOR(46.7), DTOR(50.0), DTOR(53.3), DTOR(56.7),
    DTOR(60.0), DTOR(63.3), DTOR(66.7), DTOR(70.0), DTOR(73.3), DTOR(76.7),
    DTOR(80.0)
};

const static float D70_PAN_SPEEDS[] =
{ 
    DTOR(0.0),  DTOR(1.7),  DTOR(4.3),  DTOR(7.4),  DTOR(9.8),  DTOR(13.2),
    DTOR(18.0), DTOR(21.8), DTOR(31.5), DTOR(34.5), DTOR(39.3), DTOR(47.0),
    DTOR(49.0), DTOR(54.1), DTOR(56.6), DTOR(61.8), DTOR(64.7), DTOR(69.3),
    DTOR(72.2), DTOR(79.5), DTOR(84.0), DTOR(90.9), DTOR(100.0)
};

const static float D100_PAN_SPEEDS[] =
{ 
    DTOR(0.0), DTOR(2.0), DTOR(2.4), DTOR(3.0), DTOR(3.7), DTOR(4.7),
    DTOR(6.1), DTOR(7.4), DTOR(9.1), DTOR(11),  DTOR(14),  DTOR(18),
    DTOR(22),  DTOR(27),  DTOR(34),  DTOR(42),  DTOR(52),  DTOR(65), 
    DTOR(81),  DTOR(100), DTOR(125), DTOR(155), DTOR(190), DTOR(240), 
    DTOR(300)
};

/* All possible tilt speeds (in radians per second) */

const static float D30_TILT_SPEEDS[] =
{ 
    DTOR(0.0),  DTOR(2.5),  DTOR(5.0),  DTOR(7.5),  DTOR(10.0), DTOR(12.5),
    DTOR(15.0), DTOR(17.5), DTOR(20.0), DTOR(22.5), DTOR(25.0), DTOR(27.5),
    DTOR(30.0), DTOR(32.5), DTOR(35.0), DTOR(37.5), DTOR(40.0), DTOR(42.5),
    DTOR(45.0), DTOR(47.5), DTOR(50.0),
};

const static float D70_TILT_SPEEDS[] =
{ 
    DTOR(0.0),  DTOR(1.7),  DTOR(4.3),  DTOR(7.4),  DTOR(9.8),  DTOR(13.2),
    DTOR(18.0), DTOR(21.8), DTOR(25.0), DTOR(28.8), DTOR(31.5), DTOR(34.5),
    DTOR(39.3), DTOR(47.0), DTOR(49.0), DTOR(54.1), DTOR(56.6), DTOR(61.8),
    DTOR(64.7), DTOR(69.3), DTOR(72.2)
};

const static float D100_TILT_SPEEDS[] =
{ 
    DTOR(0.0), DTOR(2.0), DTOR(2.4), DTOR(3.0), DTOR(3.7), DTOR(4.7),
    DTOR(6.1), DTOR(7.4), DTOR(9.1), DTOR(11),  DTOR(14),  DTOR(18),
    DTOR(22),  DTOR(27),  DTOR(34),  DTOR(42),  DTOR(52),  DTOR(65), 
    DTOR(81),  DTOR(100), DTOR(125),
};

/* Boolean flag tracks whether the camera will zoom in or out when given a positive or negative speed command */
bool zoomIn; 

/* Struct which contains all the parameters that define a Sony PTZ camera */
typedef struct 
{
    char modelName[10]; // Camera Model Name
    int modelID; // Camera Model ID (hexadecimal)

    /* Pan parameters in camera units */
    int minPanAngleCU;
    int maxPanAngleCU; 
    int maxPanSpeed;
    const float* possiblePanSpeeds; 

    /* Tilt parameters in camera units */
    int minTiltAngleCU;
    int maxTiltAngleCU;
    int maxTiltSpeed;
    const float* possibleTiltSpeeds; 

    /* Zoom parameters */ 

    /* Field of view values (in degrees) differ depending on camera and are set after camera model has been detected */
    double minZoomFOVRad; 
    double maxZoomFOVRad; 
    int maxZoomCU;

    int minZoomSpeed;
    int maxZoomSpeed;

     /* Camera specific limits in degrees */ 
    double minPanAngleDegrees;
    double minTiltAngleDegrees;
    double maxPanAngleDegrees;
    double maxTiltAngleDegrees;

    /* Conversion factors for converting camera units to degrees, used for setting and receiving absolute position */
    double panConversionFactor;
    double tiltConversionFactor;

    /* Boolean flags for calibration */
    bool maxSpeedIsKnown;
    bool cameraIsCalibrated;
    
} sony_ptz_cam_t;

/* Initialise structs for supported camera types
 * Provide data for accepted speeds and set calibration flags to false 
 * All other values will be read or discovered during calibration so are set to zero
 */

const static sony_ptz_cam_t EVI_D30 =
{
    "EVI-D30", D30_MODEL_HEX, // modelName, modelID
    0, 0, 0, D30_PAN_SPEEDS,  // minPanAngleCU, maxPanAngleCU, maxPanSpeed, possiblePanSpeeds
    0, 0, 0, D30_TILT_SPEEDS, // minTiltAngleCU, maxTiltAngleCU, maxTiltSpeed, possibleTiltSpeeds
    D30_MIN_ZOOM_FOV, D30_MAX_ZOOM_FOV, 0, D30_MIN_ZOOM_SPEED, MAX_ZOOM_SPEED, // minZoomFOVRad, maxZoomFOVRad, maxZoomCU, minZoomSpeed, maxZoomSpeed
    D30_MIN_PAN_DEGREES, D30_MIN_TILT_DEGREES, D30_MAX_PAN_DEGREES, D30_MAX_TILT_DEGREES, // Minimum and maximum values in degrees
    0, 0, false, false, // panConversionFactor and tiltConversionFactor (set in GetCameraModel()), maxSpeedIsKnown, cameraIsCalibrated
}; 

const static sony_ptz_cam_t EVI_D70 = 
{
    "EVI-D70P", D70_MODEL_HEX, 
    0, 0, 0,  D70_PAN_SPEEDS,
    0, 0, 0,  D70_TILT_SPEEDS,
    D70_MIN_ZOOM_FOV, D70_MAX_ZOOM_FOV, 0, D70_MIN_ZOOM_SPEED, MAX_ZOOM_SPEED,
    D70_MIN_PAN_DEGREES, D70_MIN_TILT_DEGREES, D70_MAX_PAN_DEGREES, D70_MAX_TILT_DEGREES,
    0, 0, false, false
}; 

const static sony_ptz_cam_t EVI_D100 = 
{
    "EVI-D100", D100_MODEL_HEX, 
    0, 0, 0, D100_PAN_SPEEDS, 
    0, 0, 0, D100_TILT_SPEEDS, 
    D100_MIN_ZOOM_FOV, D100_MAX_ZOOM_FOV, 0, D100_MIN_ZOOM_SPEED, MAX_ZOOM_SPEED,
    D100_MIN_PAN_DEGREES, D100_MIN_TILT_DEGREES, D100_MAX_PAN_DEGREES, D100_MAX_TILT_DEGREES, 
    0,  0, false, false
}; 

/* Constants for the supported camera models */
const static int NUM_SUPPORTED_MODELS = 3; // This driver currently supports Sony EVI-D30, D70 and D100 models 
const static sony_ptz_cam_t SONY_PTZ_CAMERAS[NUM_SUPPORTED_MODELS] = {EVI_D30, EVI_D70, EVI_D100}; /* Struct array representing supported camera models */

/* Conversion functions used for going between camera units and radians */

float panPositionToRadians(short currentPanCU)
{
    return DTOR(-currentPanCU); // Negative in camera units will be positive in radians
}

float tiltPositionToRadians(sony_ptz_cam_t ptzCam, short currentTiltCU)
{
    return DTOR(currentTiltCU);
}

float zoomPositionToRadians(sony_ptz_cam_t ptzCam, short currentZoomCU)
{
    return DTOR( RTOD(ptzCam.maxZoomFOVRad) + (static_cast<double>(currentZoomCU) * (RTOD(ptzCam.minZoomFOVRad) - RTOD(ptzCam.maxZoomFOVRad))) / 1024.0);
}

/* Convert pan speed in radians per second into the closest match in camera units */
float panSpeedToCU(sony_ptz_cam_t ptzCam, float panSpeedRadians)
{
    int panDirection;

    if (panSpeedRadians < 0)
    {
        panDirection = 1;
    }

    else
    {
        panDirection = -1;
    }

    panSpeedRadians = fabsf(panSpeedRadians); // Use the absolute value gained from the fabsf function, this is similar to abs() but specifically for floating point numbers

    int possibleSpeed = 0;

    /* Iterate through possible speeds and see if any of the speeds in camera units match the speed in radians */
    while (possibleSpeed < ptzCam.maxPanSpeed && ptzCam.possiblePanSpeeds[possibleSpeed] < panSpeedRadians)
    {
        possibleSpeed++;
    }

    /* If no matches were found or if there is an exact match... */
    if (possibleSpeed == 0)
    {
        return 0;
    }

    else
    {
        /* Store the absolute values for the difference between the pan speed in radians and the two closest matches in camera units */
        float firstDifference =  fabs(ptzCam.possiblePanSpeeds[possibleSpeed] - panSpeedRadians);
        float secondDifference = fabs(ptzCam.possiblePanSpeeds[possibleSpeed - 1] - panSpeedRadians);

        /* Return whichever value is closer */
        if (firstDifference < secondDifference)
        {
            return (short) (possibleSpeed * panDirection);
        }

        else
        {
            return (short) ( (possibleSpeed - 1) * panDirection);
        }
    }
}

/* Convert tilt speed in radians per second into the closest match in camera units */
float tiltSpeedToCU(sony_ptz_cam_t ptzCam, float tiltSpeedRadians)
{
    int tiltDirection;

    if (tiltSpeedRadians < 0)
    {
        tiltDirection = -1;
    }

    else
    {
        tiltDirection = 1;
    }

    tiltSpeedRadians = fabsf(tiltSpeedRadians); 

    int possibleSpeed = 0;

    /* Iterate through possible speeds and see if any of the speeds in camera units match the speed in radians */
    while (possibleSpeed < ptzCam.maxTiltSpeed && ptzCam.possibleTiltSpeeds[possibleSpeed] < tiltSpeedRadians)
    {
        possibleSpeed++;
    }

    /* If no matches were found or if there is an exact match... */
    if (possibleSpeed == 0)
    {
        return 0;
    }

    else
    {
        /* Store the absolute values for the difference between the tilt speed in radians and the two closest matches in camera units */
        float firstDifference = fabs(ptzCam.possibleTiltSpeeds[possibleSpeed] - tiltSpeedRadians);
        float secondDifference = fabs(ptzCam.possibleTiltSpeeds[possibleSpeed - 1] - tiltSpeedRadians);

        /* Return whichever value is closer */
        if (firstDifference < secondDifference)
        {
            return (short) (possibleSpeed * tiltDirection);
        }

        else
        {
            return (short) ( (possibleSpeed - 1) * tiltDirection);
        }
    }
}

float zoomSpeedToCU(sony_ptz_cam_t ptzCam, float zoomSpeed)
{
    /* Threshold invalid values */
    if (zoomSpeed > ptzCam.maxZoomSpeed)
    {
        std::cout << "Thresholding zoom speed to maximum supported value (" << ptzCam.maxZoomSpeed << ")" << std::endl;
        zoomSpeed = ptzCam.maxZoomSpeed;
    }

    else if (zoomSpeed < -ptzCam.maxZoomSpeed)
    {
        std::cout << "Thresholding zoom speed to maximum supported value (" << -ptzCam.maxZoomSpeed << ")" << std::endl;
        zoomSpeed = -ptzCam.maxZoomSpeed;
    }

    // If zoomSpeed is less than zero
    if (zoomSpeed < 0)
    {
        zoomIn = false;
    }

    else
    {
        zoomIn = true;
    }

    return zoomSpeed;
}

/* Convert pan position in radians into camera units */
float panPositionToCU(sony_ptz_cam_t ptzCam, float panPositionRadians)
{
    return static_cast<short> (-(RTOD(panPositionRadians)));
}

/* Convert tilt position in radians into camera units */
float tiltPositionToCU(sony_ptz_cam_t ptzCam, float tiltPositionRadians)
{
    return static_cast<short>(RTOD(tiltPositionRadians));
}

/* Convert zoom position in radians into camera units, assumes the relationship is linear but this is not actually the case */
float zoomPositionToCU(sony_ptz_cam_t ptzCam, float zoomPositionRadians)
{
    if (fabs(zoomPositionRadians) < 0.0000001)
    {
        zoomPositionRadians = ptzCam.maxZoomFOVRad;
    }

    /* Use rounding instead of truncation as in sonyevid30, credit for this improvement goes to Fred Labrosse */
    return static_cast<short> ((1024.0 * (zoomPositionRadians - ptzCam.maxZoomFOVRad) / (ptzCam.minZoomFOVRad - ptzCam.maxZoomFOVRad)) + 0.5);
}

/* Convert pan speed in camera units to a value in radians */
float panSpeedToRadians(sony_ptz_cam_t ptzCam, short panSpeedCU)
{
    int panDirection;

    if (panSpeedCU < 0)
    {
        panDirection = 1;
    }

    else
    {
        panDirection = -1;
    }

    panSpeedCU = abs(panSpeedCU);

    if (panSpeedCU < ptzCam.maxPanSpeed)
    {
        return ptzCam.possiblePanSpeeds[panSpeedCU] * panDirection; // Will result in a negative value if panSpeed is less than zero
    }

    else
    {
        return ptzCam.possiblePanSpeeds[ptzCam.maxPanSpeed] * panDirection;
    }
}

/* Convert tilt speed in camera units to value in radians */
float tiltSpeedToRadians(sony_ptz_cam_t ptzCam, short tiltSpeedCU)
{
    int tiltDirection;

    if (tiltSpeedCU < 0)
    {
        tiltDirection = -1;
    }

    else
    {
        tiltDirection = 1;
    }

    tiltSpeedCU = abs(tiltSpeedCU);

    if (tiltSpeedCU < ptzCam.maxTiltSpeed)
    {
        return ptzCam.possibleTiltSpeeds[tiltSpeedCU] * tiltDirection; // Will result in a negative value if tiltSpeed is less than zero
    }

    else
    {
        return ptzCam.possibleTiltSpeeds[ptzCam.maxTiltSpeed] * tiltDirection;
    }
}

/* Class declaration for the driver, inherits from the ThreadedDriver class */
class SonyPTZ : public ThreadedDriver
{
    public:
        SonyPTZ(ConfigFile* cf, int section); // Constructor prototype

        /* Alternate zoom functions provided for direct use */
        int SetTeleZoom(uint32_t zoomSpeed = MAX_ZOOM_SPEED);
        int SetWideZoom(uint32_t zoomSpeed = MAX_ZOOM_SPEED);
        int StopZoom();

    protected:
        /* Stores all the parameters for representing a Sony EVI pan-tilt-zoom camera */
        sony_ptz_cam_t ptzCamConfig; 

        /* Required Main functions for a ThreadedDriver
         * A constructor, registration function 
         * and both C++ and C init functions are also required
         */
        virtual int  MainSetup();
        virtual void MainQuit();
        virtual void Main();

        /* Low level VISCA functions */
        void PrintVISCAPacket(const char* infoStr, unsigned char* cmdStr, int cmdLen);
        int  ReceiveVISCAPacket(unsigned char* cmdReply);
        int  SendVISCAPacket(unsigned char* msgStr, int msgLen, unsigned char* msgReply, uint8_t camID = 1); // Default camera ID is 1 (first camera attached)
        int  SendVISCAInquiry(unsigned char* inqStr, int inqLen, unsigned char* inqReply, uint8_t camID = 1);
        int  SendVISCACommand(unsigned char* cmdStr, int cmdLen, uint8_t camID = 1);
        int  CancelVISCACommand(char socketId); // Cancels command in progress

        /* Message processing functions */
        int ProcessGenericRequest(QueuePointer &resp_queue, player_msghdr* hdr, player_ptz_req_generic_t *req);
        int ProcessMessage(QueuePointer &resp_queue, player_msghdr* hdr, void* data); // This function is called on every incoming message
        int ProcessPTZRequest(player_ptz_cmd_t* ptzCmd);

        /* Higher level commands */
        int CalibrateCamera(); // Function to set correct minimum and maximum values for pan, tilt and zoom depending on hardware in use
        int GetPanTiltStatus(); // Gets status of previously executed pan and tilt commands, used for calibration
        int GetCameraModel(); // Function to return information on the camera model; including socket number, vendor ID, etc.
        int GetMaximumSpeeds();
        int GetAbsolutePanTiltPosition(short* panPositionDegrees, short* tiltPositionDegrees);
        int GetAbsoluteZoomPosition(short* zoomPosition); // Zoom 'position' refers to the zoom level, it is named as such for consistency with pan and tilt position functions 
        int SetCameraPower(bool powerUp); // Function to switch the camera on or off, also returns current power status
        int SetAbsolutePanTiltPosition(short panPositionRequestDegrees, short tiltPositionRequestDegrees, bool waitForCompletion = false); // The boolean specifies whether to accept additional commands during execution (default) or wait until movement is completed
        int SetAbsoluteZoomPosition(short zoomPositionRequestCU);
        int SetPanTiltSpeed(short panSpeedRequestCU, short tiltSpeedRequestCU);
        int SetZoomSpeed(short zoomSpeedRequestCU);
        int SetAbsolutePanTiltSpeed(short panSpeedRequestCU, short tiltSpeedRequestCU); /* Used in conjunction with the SetAbsolutePanTiltSpeed() function */

        /* Function to update internal data representation based on new data from the camera */
        int UpdatePTZData(player_ptz_data_t &ptzData);

        /* Demo function, may be useful for diagnostics */
        void PanTiltDemo(); 

        /* Serial port variables */
        std::string serialPort; // Variable to hold the path of the serial port the camera is connected to
        int serialPortFileDescriptor; // File descriptor for the serial port, used to determine if reading was successful 

        /* Variables used in the ReceiveVISCAPacket() function */
        unsigned char cmdBuffer[MAX_PTZ_PACKET_LENGTH]; // Buffer to store a PTZ packet
        int bytesReceived; // Counts bytes received from the PTZ camera
        struct pollfd readData; // Struct of type pollfd used for polling the camera for new data

        int PTZControlMode; // Stores the current PTZ control mode, position / velocity
        bool waitForCompletion; // Defines whether the camera waits for the completion of a movement command before processing any new commands 

        /* Variables for reading from and writing to the config file outside of the constructor */
        ConfigFile* configFile;
        int configFileSection;

        /* Variables for Velocity Mode */

        /* Velocity values are measured in radians per second */
        float lastExecutedPanSpeedRadians; 
        float lastRequestedPanSpeedRadians; 

        float lastExecutedTiltSpeedRadians;
        float lastRequestedTiltSpeedRadians;

        float lastExecutedZoomSpeed;
        float lastRequestedZoomSpeed;

        /* Variables for Position Mode */
        float lastRequestedPanPositionRadians;
        float lastRequestedTiltPositionRadians;
        float lastRequestedZoomPositionRadians;

        /* Variable to store the pan / tilt status reply, used for calibration */
        unsigned char panTiltStatus[MAX_PTZ_PACKET_LENGTH];

        short currentPanSpeed;
        short currentTiltSpeed;
};

/* Constructor definition 
 * PLAYER_MSGQUEUE_DEFAULT_MAXLEN is defined in libplayerinterface/player.h
 * PLAYER_PTZ_CODE represents the PTZ interface provided by Player
 * Arguments are a pointer to the config file, the section within the config file to read (typically 0), whether new commands overwrite old ones
 * The maximum length of the incoming queue and the interface the driver supports - in this case PTZ
 */
SonyPTZ::SonyPTZ(ConfigFile* cf, int section) : 
ThreadedDriver(cf, section, true, PLAYER_MSGQUEUE_DEFAULT_MAXLEN, PLAYER_PTZ_CODE), 
serialPort (cf->ReadString(section, "port", DEFAULT_SERIAL_PORT))
{
    std::cerr << "SonyPTZ Driver Initialising..." << std::endl;

    /* Set some default values for variables */
    serialPortFileDescriptor = -1;
    readData.events = POLLIN; // Set poll type

    PTZControlMode = PLAYER_PTZ_POSITION_CONTROL; // Control mode set to Position instead of Velocity by default
    this->waitForCompletion = false;
    
    /* Allow the config file and section variables to be used outside of the constructor */
    this->configFile = cf;
    this->configFileSection = section;

    /* Set sensible initial values for speed, these will be updated by either the GetMaximumSpeeds() function during calibration or an explicit call to SetPanTiltSpeed() */
    this->currentPanSpeed = 0x10;
    this->currentTiltSpeed = 0x10;

    /* Set position and speed variables to zero to begin */
    this->lastRequestedPanSpeedRadians = 0.0;
    this->lastExecutedPanSpeedRadians  = 0.0;

    this->lastRequestedTiltSpeedRadians = 0.0;
    this->lastExecutedTiltSpeedRadians  = 0.0;

    this->lastRequestedZoomSpeed = 0.0;
    this->lastExecutedZoomSpeed = 0.0;

    this->lastRequestedPanPositionRadians  = 0.0;
    this->lastRequestedTiltPositionRadians = 0.0;
    this->lastRequestedZoomPositionRadians = 0.0;

    /* Read the serial port value from the configuration file, if present */
    //this->serialPort = cf->ReadString(section, "port", DEFAULT_SERIAL_PORT); // The third argument is the default value if no value is found in the config file

    /* Set up buffer and counter variables for receiving data */
    this->bytesReceived = 0;
    memset(this->cmdBuffer, 0, sizeof this->cmdBuffer);

    /* All messages sent to the driver appear in InQueue which is then manipulated by ProcessMessage(). SetReplace means new incoming commands overwrite old ones instead of queuing up */
    InQueue->SetReplace(true); // See message.h for more information
}

/* This method is used for necessary setup such as opening a serial port, detecting camera models and calibrating or reading data */
int SonyPTZ::MainSetup()
{
    struct termios serialConf; /* Struct to store serial port configuration */
    int serialFlags; /* Variable to store bitmask of serial port flags */

    std::cerr << "MainSetup()..." << std::endl;

    std::cout << "Attempting to initialise serial port " << this->serialPort << "..." << std::endl;

    /* Try to read the serial port connected to the PTZ camera, if this doesn't work then return -1 to indicate an error
     * O_RDWR means open in read / write mode, O_SYNC means writing will be done synchronously 
     * O_NONBLOCK returns an error if no data is available rather than blocking - we use this because a camera may not actually be connected
     * S_IRUSR and S_IWUSR are file permission bits allowing read and write access permissions 
     */
    if ( ( serialPortFileDescriptor = open(this->serialPort.c_str(), O_RDWR | O_SYNC | O_NONBLOCK, S_IRUSR | S_IWUSR) )  < 0) 
    {
        perror("Failed to open serial port");
        return -1;
    }

    std::cout << "Successfully opened " << this->serialPort << std::endl;

    /* Required for correct polling */
    readData.fd = serialPortFileDescriptor;

    /* Attempt to flush serial port of any data received but not read */
    if (tcflush(serialPortFileDescriptor, TCIFLUSH) < 0)  
    {
        perror("Call to tcflush() in SonyPTZ::MainSetup() failed");
        close(serialPortFileDescriptor);
        serialPortFileDescriptor = -1; // Set to -1 to indicate port has been closed
        return -1; // Return indicating an error
    }

    /* Attempt to read current serial port config */
    if (tcgetattr(serialPortFileDescriptor, &serialConf) < 0) // Read config into serialConf struct
    {
        perror("Call to tcgetattr() in SonyPTZ::MainSetup() failed, could not read current serial port config");
        close(serialPortFileDescriptor);
        serialPortFileDescriptor = - 1;
        return -1;
    }

    /* If the serial port has been successfully flushed and attributes can be read, set up port for communicating with PTZ camera */
    cfmakeraw(&serialConf); // Enable 'raw' mode as this is a serial port and not an actual tty / terminal
    cfsetspeed(&serialConf, B9600); // Set speed to 9600 baud for input and output, B9600 is a macro defined in <termios.h>. 9600 is the standard VISCA baud rate

    /* Attempt to configure the serial port with the new settings, 'raw' and 9600 baud */
    if (tcsetattr(serialPortFileDescriptor, TCSAFLUSH, &serialConf) < 0)
    {
        perror("Call to tcsetattr() in SonyPTZ::MainSetup() failed, could not set new serial port config");
        close(serialPortFileDescriptor);
        serialPortFileDescriptor = -1;
        return -1;
    }

    /* Attempt to read the flags on the serial port file descriptor, if it succeeded it will return the value of the flags */
    if ((serialFlags = fcntl(serialPortFileDescriptor, F_GETFL)) < 0) 
    {
        perror("Could not read serial port flags in SonyPTZ::MainSetup(), fcntl() failed");
        close(serialPortFileDescriptor);
        serialPortFileDescriptor = -1;
        return -1;
    }

    /* Attempt to read the model of the camera to make absolutely sure we are connected to a PTZ camera and not something else, also sets the correct parameters for the specific model */
    if ((GetCameraModel()) != 0)
    {
        perror("Could not connect to PTZ camera in SonyPTZ::MainSetup::GetCameraModel(). Either nothing is connected to the serial port, the power is off, or no PTZ camera was detected");
        close(serialPortFileDescriptor);
        serialPortFileDescriptor = -1;
        return -1;
    }

    /* Finally, attempt to reopen the port in blocking mode now we are sure it is connected to the camera we want to communicate with
     * The third argument  to fcntl() defines what operation is performed on the flag bitmask
     * In this case XOR which removes the O_NONBLOCK flag. 
     */
    if (fcntl(serialPortFileDescriptor, F_SETFL, serialFlags ^ O_NONBLOCK) < 0) 
    {
        perror("Could not set serial port flags in SonyPTZ::MainSetup(), fcntl() failed");
        close(serialPortFileDescriptor);
        serialPortFileDescriptor = -1;
        return -1;
    }

    /* Power up the camera in case it is in standby mode */
    if ((SetCameraPower(true)) < 0)
    {
        std::cerr << "Could not set camera power, communications error on " << serialPort << std::endl;
        return -1;
    }

    /* Check the configuration file to see if values for the connected camera are present, the driver calibrates new cameras and then saves the results in the config file for future access */
    int cameraConfig[6]; // Array to store calibration data, 6 is the expected number of configuration parameters 
    
    /* Figure out if previously calibrated by checking for calibration data in the config file */
    int calibrationData = configFile->ReadTupleInt(configFileSection, ptzCamConfig.modelName, 0, -1);

    if (calibrationData != -1)
    {
        ptzCamConfig.cameraIsCalibrated = true;
    }

    /* If the driver has previously been calibrated with a model that is the same as the connected camera, attempt to read these values from the config file */
    if (ptzCamConfig.cameraIsCalibrated) // If this is set to false the camera will run the calibration routine, otherwise it will attempt to read previously gathered data from the config file 
    {
        for (int currentValue = 0; currentValue < (sizeof(cameraConfig) / sizeof(cameraConfig[0])); currentValue++)
        {
            /* Mark any values that are not present or could not be parsed with a value of -1 */
            cameraConfig[currentValue] = configFile->ReadTupleInt(configFileSection, ptzCamConfig.modelName, currentValue, -1);
            
            if (cameraConfig[currentValue] == -1) // Error reading config file
            {
                perror("SonyPTZ::MainSetup(): Config file could not be parsed, possibly corrupt - recommend reverting to example config file.");
                exit(-1);
            }
        }

        /* Set the values according to the calibration data */
        ptzCamConfig.minPanAngleCU  = cameraConfig[0];
        ptzCamConfig.maxPanAngleCU  = cameraConfig[1];
        ptzCamConfig.minTiltAngleCU = cameraConfig[2];
        ptzCamConfig.maxTiltAngleCU = cameraConfig[3];
        ptzCamConfig.maxPanSpeed    = cameraConfig[4];
        ptzCamConfig.maxTiltSpeed   = cameraConfig[5];
    }

    /* If the camera has not been previously calibrated, run the calibration routine which will find the correct values and print them for future use */
    else
    {
        if ((CalibrateCamera() != 0)) // Calibration routine to acquire minimum and maximum values for position and maximum speeds
        {
            std::cerr << "Failed calibration" << std::endl;
            return -1;
        } 
    }

    /* Set the conversion factor - this relates the maximum pan angle in camera units to the maximum pan angle in degrees supported by the hardware */
    ptzCamConfig.panConversionFactor = ptzCamConfig.maxPanAngleCU / (double) ptzCamConfig.maxPanAngleDegrees;
    ptzCamConfig.tiltConversionFactor = ptzCamConfig.maxTiltAngleCU / (double) ptzCamConfig.maxTiltAngleDegrees;

    std::cout << "MainSetup() completed" << std::endl;

    if (configFile->ReadInt(configFileSection, "demo", 0) != 0)
    {
        PanTiltDemo(); 
    }

    return 0; // Return 0 to indicate setup completed successfully 
}

void SonyPTZ::MainQuit() 
{
    std::cout << "MainQuit()..." << std::endl;

    if (serialPortFileDescriptor == -1) // If the serial port was closed before sending any commands
    {
        return; // No need to move camera back to home position
    }

    /* Move camera back to home position and set zoom to minimum */
    SetAbsolutePanTiltPosition(CENTER, CENTER, true); 
    SetWideZoom();

    /* Power down camera - optional */
    #ifdef CAMERA_SHUTDOWN
    SetCameraPower(false);
    #endif
    
    if (close(serialPortFileDescriptor) < 0) // Close the device file for the serial port if still open
    {
        perror("SonyPTZ::MainQuit:close():");
    }

    std::cout << "Closed serial port " << this->serialPort << std::endl; 

    exit(0);
}

int SonyPTZ::GetMaximumSpeeds()
{
    unsigned char maxSpeedInquiry[] = {0x09, 0x06, 0x11}; // 'Pan-tiltMaxSpeedInq'
    unsigned char maxSpeedInquiryReply[MAX_PTZ_PACKET_LENGTH];

    int maxSpeedInquiryReplyLength = SendVISCAInquiry(maxSpeedInquiry, 3, maxSpeedInquiryReply);

    if (maxSpeedInquiryReplyLength < 4)
    {
        perror("SonyPTZ::GetMaximumSpeeds(): Error retrieving maximum speed values");
        return -1;
    }

    /* Set the maximum speeds according to the parameters reported by the camera */
    ptzCamConfig.maxPanSpeed  = maxSpeedInquiryReply[2];
    ptzCamConfig.maxTiltSpeed = maxSpeedInquiryReply[3];

    /* Set the current speed to the fastest possible, the client can then change this using a call to SetPanTiltSpeed() */
    this->currentPanSpeed  = ptzCamConfig.maxPanSpeed;
    this->currentTiltSpeed = ptzCamConfig.maxTiltSpeed;

    ptzCamConfig.maxSpeedIsKnown = true;

    return 0;
}

/* This function tests various values for pan, tilt and zoom in order to acquire accurate minimum and maximum values that properly reflect the connected hardware */
int SonyPTZ::CalibrateCamera()
{
    std::cout << "Calibrating..." << std::endl;

    /* Query camera for maximum speed settings, this sets maxPanSpeed and maxTiltSpeed in the sony_ptz_cam_t struct accordingly */
    GetMaximumSpeeds();

    /* Increment varies by model for the quickest and most accurate calibration */
    int testIncrement;

    if (ptzCamConfig.modelID == D70_MODEL_HEX)
    {
        testIncrement = 1;
    }

    else
    {
        testIncrement = 10;
    }

    /* Set initial test positions for pan and tilt */
    int testPositionPan = 0x0600; // Value discovered through experimentation, this should ideally be a value close to an upper or lower limit but still within range for all supported cameras
    int testPositionTilt = 0x3E8; // Set test position for tilt to a value close to the expected maximum value but still within range for every supported camera 

    /* Move to the start positions for testing pan and tilt */ 
    SetAbsolutePanTiltPosition(testPositionPan, testPositionTilt, true); // Passing a value of true means the hardware will wait until the command has completed before processing any new movement commands

    bool stopPanning = false;
    bool stopTilting = false;

    bool incrementPan = true;
    bool incrementTilt = true;

    while (true) 
    {
        SetAbsolutePanTiltPosition(testPositionPan, testPositionTilt, true);
        GetPanTiltStatus(); // This updates the state of the panTiltStatus variable

        /* The hexadecimal values here relate to the pan/tilter status codes provided in the technical manuals */
        /* VISCA is most-significant-bit first so 0x02 is equivalent to an S value of --1-, see pg. 51 EVI-D70P manual */
        if ( (panTiltStatus[3] == 0x02 || panTiltStatus[3] == 0x08) && incrementPan)
        {
            ptzCamConfig.maxPanAngleCU = testPositionPan;
            printf("Maximum Pan Angle: 0x%2X\n", ptzCamConfig.maxPanAngleCU); 
            incrementPan = false;

            if (ptzCamConfig.modelID == D70_MODEL_HEX)
            {
                testPositionPan = 0xF800;
            }
        }

        if (panTiltStatus[3] == 0x04 || panTiltStatus[3] == 0x06)
        {
            ptzCamConfig.maxTiltAngleCU = testPositionTilt;
            printf("Maximum Tilt Angle: 0x%2X\n", ptzCamConfig.maxTiltAngleCU); 
            incrementTilt = false;

            /* Optimisation for the D70 */
            if (ptzCamConfig.modelID == D70_MODEL_HEX)
            {
                testPositionTilt = 0xFEDD;
            }
        }

        if ( (panTiltStatus[3] == 0x01 || panTiltStatus[3] == 0x09) && !stopPanning)
        {
            ptzCamConfig.minPanAngleCU = testPositionPan;
            printf("Minimum Pan Angle: 0x%2X\n", ptzCamConfig.minPanAngleCU); 
            stopPanning = true;
        }

        if ( (panTiltStatus[3] == 0x08 || panTiltStatus[3] == 0x09) && !stopTilting) 
        {
            ptzCamConfig.minTiltAngleCU = testPositionTilt;
            printf("Minimum Tilt Angle: 0x%2X\n", ptzCamConfig.minTiltAngleCU); 
            stopTilting = true;
        }  

        if (stopPanning && stopTilting)
        {
            break;
        }

        if (incrementPan && !stopPanning)
        {
            testPositionPan+= testIncrement;
        }

        else if (!incrementPan && !stopPanning)
        {
            testPositionPan-= testIncrement;
        }

        if (incrementTilt && !stopTilting)
        {
            testPositionTilt+= testIncrement;
        }

        else if (!incrementTilt && !stopTilting)
        {
            testPositionTilt-= testIncrement;
        }
    } 
    
    /* Print the values discovered by calibration so the user can copy these into the config file for future use */
    std::cout << "Calibrated values are as follows (you may copy these into the config file): ";

    /* Variables to store configuration data */
    std::string calibrationData = std::string(ptzCamConfig.modelName) + " ";

    /* Build a tuple for minPanAngleCU, maxPanAngleCU, minTiltAngleCU, maxTiltAngleCU, maxPanSpeed and maxTiltSpeed for potential insertion into the file */
    calibrationData +=  "["; 
    calibrationData += TOSTR(ptzCamConfig.minPanAngleCU,  " "); // Use the macro function defined at the top of this file
    calibrationData += TOSTR(ptzCamConfig.maxPanAngleCU,  " ");
    calibrationData += TOSTR(ptzCamConfig.minTiltAngleCU, " ");
    calibrationData += TOSTR(ptzCamConfig.maxTiltAngleCU, " ");
    calibrationData += TOSTR(ptzCamConfig.maxPanSpeed,    " ");
    calibrationData += TOSTR(ptzCamConfig.maxTiltSpeed,    "");
    calibrationData += "]";

    std::cout << calibrationData << std::endl;

    ptzCamConfig.cameraIsCalibrated = true;

    SetAbsolutePanTiltPosition(CENTER, CENTER, true); // Move camera back to the centre 

    return 0;
}

/* Utility function to print a VISCA packet as a hexadecimal string for debugging purposes */
void SonyPTZ::PrintVISCAPacket(const char* infoStr, unsigned char* cmdStr, int cmdLen)
{
    std::cout << infoStr << ": "; // Print the information string passed into the function

    for (int i = 0; i < cmdLen; i++) // Iterate through the packet and print the hexadecimal pairs 
    {
        printf(" %.2X", cmdStr[i]);
    }

    std::cout << std::endl;
}

int SonyPTZ::ReceiveVISCAPacket(unsigned char* cmdReply)
{
    unsigned char storedReply[MAX_PTZ_PACKET_LENGTH]; // Define a temporary char array to store the reply
    int currentByte; // Iterator for use with reply
    int bufferPtr = -1; // Value points to a specific part of the buffer

    memset(storedReply, 0, MAX_PTZ_PACKET_LENGTH); // Set the temporary array to a string containing all-zeroes of the maximum packet length
    memset(cmdReply, 0, MAX_PTZ_MESSAGE_LENGTH); // Do the same for the reply that was passed in

    if (this->bytesReceived > 0)
    {
        memcpy(storedReply, this->cmdBuffer, MAX_PTZ_PACKET_LENGTH); // Copy what is currently in the buffer to the storedReply variable

        // Find the terminating byte of the VISCA packet (0xFF)
        for (currentByte = 0; currentByte < this->bytesReceived; currentByte++)
        {
            if (storedReply[currentByte] == 0xFF)
            {
                bufferPtr = currentByte;
                break;
            }
        }
    }

    while (bufferPtr < 0)
    {
        int dataReceived = poll(&readData, 1, 1000); // This uses the poll function for synchronous IO multiplexing, requires that poll.h or replace.h be included and a struct of type pollfd be instantiated

        if (dataReceived == 0 && !waitForCompletion) /* No data received, this could be caused by a timeout or simply a lack of data, print error message */
        {
            std::cerr << "PTZ poll timeout, error" << std::endl;
        }

        else if (dataReceived < 0)
        {
            std::cerr << "PTZ poll failure, error" << std::endl;
        }

        int newBytesReceived = read(serialPortFileDescriptor, storedReply + this->bytesReceived, MAX_PTZ_REPLY_LENGTH - (this->bytesReceived));

        if ((this->bytesReceived += newBytesReceived) < 0) // If no bytes were received there was an error with communications
        {
            perror("SonyPTZ::ReceiveVISCAPacket()::read()");
            return -1;
        }

        else if (!newBytesReceived) // If no new bytes were received, flush input and return zero - potential glitch
        {
            if (tcflush(serialPortFileDescriptor, TCIFLUSH) < 0)
            {
                perror("SonyPTZ::ReceiveVISCAPacket()::tcflush()");
                return -1;
            }

            this->bytesReceived = 0; // Reset counter
            return 0;
        }

        // Find the VISCA packet terminator
        for (currentByte = 0; currentByte < this->bytesReceived; currentByte++)
        {
            if (storedReply[currentByte] == 0xFF)
            {
                bufferPtr = currentByte;
                break;
            }
        }
    }

    // Keep extra bytes received 
    if (bufferPtr == (this->bytesReceived - 1))
    {
        this->bytesReceived = 0;
    }

    else
    {
        // Store the bytes
        memcpy(this->cmdBuffer, storedReply + (bufferPtr + 1), this->bytesReceived - (bufferPtr + 1) );
        this->bytesReceived = this->bytesReceived - (bufferPtr + 1); 
    }

    // Ignore anything in the packet before the start character, which is 0x90
    for (currentByte = 0; currentByte < bufferPtr; currentByte++)
    {
        if (storedReply[currentByte] == 0x90 && storedReply[currentByte + 1] != 0x90)
        {
            break;
        }
    }

    if (currentByte == bufferPtr)
    {
        return 0;
    }

    memcpy(cmdReply, storedReply + currentByte, (bufferPtr + 1) - currentByte);

    /* Detect command completion and either wait or discard this information depending on if waitForCompletion is set */
    if (this->waitForCompletion) // If we care about detecting command completions...
    {
        if ((cmdReply[0] == 0x90) && ((cmdReply[1] >> 4) == 0x05) && (cmdReply[2] == 0xFF))
        {
            return COMMAND_COMPLETE_CODE; // Indicates that the command being waited for has completed
        }
    }

    return ( (bufferPtr + 1) - currentByte);
}

/* Low level function to send a packet, constructs the packet using the header, command or inquiry passed in and the terminator, then attempts to send it to the camera */
int SonyPTZ::SendVISCAPacket(unsigned char* msgStr, int msgLen, unsigned char* msgReply, uint8_t camID)
{
    unsigned char packetToSend[MAX_PTZ_PACKET_LENGTH];
    int currentByte; // VISCA packets may be from 3 to 16 bytes in size

    if (msgLen > MAX_PTZ_MESSAGE_LENGTH)
    {
        std::cerr << "SonyPTZ::SendVISCAPacket(): Message is too large at " << msgLen << "bytes in size" << std::endl;
        return -1;
    }

    packetToSend[0] = 0x80 | camID; // Address of the controller will always be 0, first camera will be 1, etc.

    for (currentByte = 0; currentByte < msgLen; currentByte++)
    {
        packetToSend[currentByte + 1] = msgStr[currentByte];
    }

    packetToSend[currentByte + 1] = 0xFF; // VISCA packet terminator

    /* Packet has been constructed, now we attempt to send it via the serial port */
    if (write(serialPortFileDescriptor, packetToSend, currentByte + 2) < 0)
    {
        perror("Error sending VISCA packet in SonyPTZ::SendVISCAPacket()");
        return -1;
    } 

    return (ReceiveVISCAPacket(msgReply));
}

/* Function that uses the SendVISCAPacket() and ReceiveVISCAPacket() functions to send an inquiry to a camera */
int SonyPTZ::SendVISCAInquiry(unsigned char* inqStr, int inqLen, unsigned char* inqReply, uint8_t camID)
{
    int inqReplyLen;

    if ((inqReplyLen = SendVISCAPacket(inqStr, inqLen, inqReply, camID)) <= 0)
    {
        return inqReplyLen;
    }

    while ((inqReply[0] != 0x90) || (inqReply[1] != 0x50))
    {
        if ((inqReply[0] != 0x90) || ((inqReply[1] >> 4) != 0x05) || (inqReplyLen != 3)) /* Inquiry replies should always be 3 bytes */
        {
            PrintVISCAPacket("SonyPTZ::SendVISCAInquiry(): expected information return but received", inqReply, inqReplyLen);
        }

        /* Call receive function for reply */
        if ((inqReplyLen = ReceiveVISCAPacket(inqReply)) <= 0)
        {
            return inqReplyLen;
        }
    }

    return inqReplyLen;
}

/* Function to send a VISCA command using lower level functions */
int SonyPTZ::SendVISCACommand(unsigned char* cmdStr, int cmdLen, uint8_t camID)
{
    /* Variables to store the command reply and its length */
    unsigned char cmdReply[MAX_PTZ_PACKET_LENGTH];
    int cmdReplyLen;

    /* Cancel any previous commands, this driver is aimed at real-time applications, both buffers will be cleared unless the user has specifically requested blocking until completion */
    if (!this->waitForCompletion)
    {
        CancelVISCACommand(1);
        CancelVISCACommand(2);
    }

    if ((cmdReplyLen = SendVISCAPacket(cmdStr, cmdLen, cmdReply, camID)) <= 0) /* This uses a lower level function to send the actual serial packet */
    {
        return cmdReplyLen;
    }

    /* Wait for the ACK which signals the command was received */
    while ( (cmdReply[0] != 0x90) || ( (cmdReply[1] >> 4) != 0x04 ) || (cmdReplyLen != 3) ) // ACK messages are 3 bytes long and for camera 1 should begin with 0x90
    {
        if ( (cmdReply[0] != 0x90) || ( (cmdReply[1] >> 4) != 0x05))
        {
            PrintVISCAPacket("SonyPTZ::SendVISCACommand(): Expected ACK, but received", cmdReply, cmdReplyLen); // Print whatever was received instead of an ACK for debugging purposes
        }
        
        if ((cmdReplyLen = ReceiveVISCAPacket(cmdReply)) <= 0) // Receive the reply, if the return value is 0 or less then return the length 
        {
            return cmdReplyLen;
        }
    }

     /* If the flag to wait for command completion is set, wait for the ReceiveVISCAPacket() function to return a command completion before continuing execution */
    if (this->waitForCompletion)
    {
        while (true)
        {
            if ((ReceiveVISCAPacket(cmdReply) == COMMAND_COMPLETE_CODE))
            {
                break;
            }
        }

        this->waitForCompletion = false; // Command has completed so unset the flag
    }

    return 0;
}

/* Function to cancel all commands currently being processed by the camera */ 
int SonyPTZ::CancelVISCACommand(char socketId)
{
    unsigned char cancelCommand[MAX_PTZ_MESSAGE_LENGTH];
    unsigned char cancelCommandReply[MAX_PTZ_MESSAGE_LENGTH];

    int cancelCommandReplyLength;

    cancelCommand[0] = socketId;
    cancelCommand[0] |= 0x20;

    if ((cancelCommandReplyLength = SendVISCAPacket(cancelCommand, 1, cancelCommandReply)) <= 0)
    {
        return(cancelCommandReplyLength);
    }

    /* Wait to receive response indicating successful cancellation */
    while ((cancelCommandReply[0] != 0x90) || ((cancelCommandReply[1] >> 4) != 0x06) ||
          !((cancelCommandReply[2] == 0x04) || (cancelCommandReply[2] == 0x05)) || (cancelCommandReplyLength != 4))
    {
        if ((cancelCommandReply[0] != 0x90) || ((cancelCommandReply[1] >> 4) != 0x05) || (cancelCommandReply[2] != 0xFF))
        {
            PrintVISCAPacket("SonyPTZ::CancelVISCACommand(): Received Unexpected Response: ", cancelCommandReply, cancelCommandReplyLength);
        }

        if ((cancelCommandReplyLength = ReceiveVISCAPacket(cancelCommandReply)) <= 0)
        {
            return(cancelCommandReplyLength);
        }
    }

    return 0;
}

/* Function to set camera power status, will not send a power on command if already powered on */
int SonyPTZ::SetCameraPower(bool powerUp)
{
    unsigned char powerInquiry[] = {0x09, 0x04, 0x00}; // Actual command that is sent would be '0x8X 0x09 0x04 0x00 0xFF' where X is the address of the camera (0x81 for the first camera connected)
    unsigned char powerInquiryReply[MAX_PTZ_PACKET_LENGTH]; // Example reply: Y0 50 03 FF - the Y0 and FF are just the start and terminating packets, the 50 indicates an inquiry reply and 03 indicates the camera is currently powered down

    /* Send a VISCA inquiry: Arguments are the command to send, the length of the command in bytes (ignoring the header and terminator), the variable to store the reply in and the camera ID (1 is the first camera connected, maximum is theoretically 7) */
    int powerInquiryReplyLength = SendVISCAInquiry(powerInquiry, MIN_PTZ_INQUIRY_LENGTH, powerInquiryReply);

    if (powerInquiryReplyLength < MIN_PTZ_REPLY_LENGTH) // Malformed or incomplete reply packet, problem with connection. Reply must be at least 4 bytes including header and terminator
    {
        std::cerr << "Did not receive enough bytes from inquiry, problem with connection" << std::endl;
        return -1;
    }

    else if (powerInquiryReply[2] == 0x02 && powerUp == true) // If the third byte of the reply is set to 0x02 the power is already on, do nothing
    {
        #ifdef DEBUG_POWER
        PrintVISCAPacket("Power reply:", powerInquiryReply, powerInquiryReplyLength);
        std::cerr << "Power is already on, command ignored" << std::endl;
        #endif
        return 0; 
    }

    else if (powerInquiryReply[2] == 0x02 && powerUp == false) // If the power is on and we have specified the camera should power down 
    {
        /* The power on and power off commands are defined under CAM_POWER in the VISCA standard - not to be confused with the power inquiry command */
        unsigned char powerOffCommand[] = {0x01, 0x04, 0x00, 0x03}; // Notice that the address and terminator are left out, this is handled by SendVISCACommand()

        std::cerr << "Attempting to power down the camera..." << std::endl;

        return SendVISCACommand(powerOffCommand, 4); // Arguments are the command to send, the length of the command in bytes and the camera ID of the receiver (defaults to 1)
    }

    else if (powerInquiryReply[2] == 0x03 && powerUp == true) // If the third byte of the reply is set to 0x03 the camera is on standby, power up
    {
        unsigned char powerOnCommand[] = {0x01, 0x04, 0x00, 0x02}; 

        std::cerr << "Attempting to power up the camera..." << std::endl;

        return SendVISCACommand(powerOnCommand, 4); 
    }

    else
    {
        perror("Error communicating with camera in SonyPTZ::SetCameraPower()");
        return -1; // Failed to communicate with camera, return a failure
    }
}

/* Returns various information about the type of camera connected such as vendor, model ID, etc. */
int SonyPTZ::GetCameraModel()
{
    unsigned char modelInquiry[] = {0x09, 0x00, 0x02}; // Defined as CAM_VersionInq in the VISA specification
    unsigned char modelInquiryReply[MAX_PTZ_PACKET_LENGTH];
    int modelInquiryReplyLength;

    if ((modelInquiryReplyLength = SendVISCAInquiry(modelInquiry, MIN_PTZ_INQUIRY_LENGTH, modelInquiryReply)) <= 0) // If there is an error sending the inquiry...
    {
        return modelInquiryReplyLength; // Return the length of the reply
    }

    else
    {
        /* The reply from a CAM_VersionInq inquiry will be hexadecimal 'Y0 50 00 01 MN PQ RS TU VW FF' where MNPQ is the model code (04xx), RSTU the ROM version and VW the socket number */
        int modelID = (modelInquiryReply[4] << 8) + modelInquiryReply[5]; // We use a left bitshift to isolate just the model number from the reply - 'MN PQ'

        int supportedModel; // Iterator for supported camera models

        for (supportedModel = 0; supportedModel < NUM_SUPPORTED_MODELS; supportedModel++)
        {
            /* Iterate through the supported models, see the SONY_PTZ_CAMERAS definition near the top of the file for more information */
            if (SONY_PTZ_CAMERAS[supportedModel].modelID == modelID) // If the model of the connected camera was recognised as one of the supported models...
            {
                ptzCamConfig = SONY_PTZ_CAMERAS[supportedModel]; // Set the value of the ptzCamConfig struct (of type sony_ptz_cam_t, defined near the top of this file) accordingly, represents a camera 
                std::cout <<  "Recognised an " << ptzCamConfig.modelName << " Camera..." << std::endl;
                break; // Found the camera, no need to continue iterating
            }
        }

        if (supportedModel >= NUM_SUPPORTED_MODELS)
        {
            std::cerr << "Unable to identify connected camera... setting to D30. THIS MAY CAUSE UNDEFINED BEHAVIOUR" << std::endl;
            ptzCamConfig = SONY_PTZ_CAMERAS[0]; // The first item in the struct array is the D30
        }
    }

    /* Experimental feature for returning zoom level as a percentage, not guaranteed to be accurate */
    short currentZoom;

    std::cout << "Calibrating Zoom..." << std::endl;

    SetTeleZoom();
    usleep(SLEEP_TIME_MS * 40);

    GetAbsoluteZoomPosition(&currentZoom);
    
    ptzCamConfig.maxZoomCU = currentZoom;

    SetWideZoom();
    /* End of experimental feature */

    return 0;
}

/* Function to return current pan-tilt status, uses the (poorly documented) Pan-tiltModeInq VISCA command */
int SonyPTZ::GetPanTiltStatus()
{
    unsigned char panTiltModeInquiry[MAX_PTZ_MESSAGE_LENGTH] = {0x09, 0x06, 0x10};

    int panTiltModeReplyLength = SendVISCAInquiry(panTiltModeInquiry, MIN_PTZ_INQUIRY_LENGTH, panTiltStatus); // Set as a global variable because we need access to this

    if (panTiltModeReplyLength < MIN_PTZ_REPLY_LENGTH) // Malformed or incomplete reply packet, problem with connection. Reply must be at least 4 bytes including header and terminator
    {
        std::cerr << "Did not receive enough bytes from inquiry, problem with connection" << std::endl;
        return -1;
    }

    return 0; 
}

/* Updates the absolute pan and tilt position (in degrees) variables using pointers */
int SonyPTZ::GetAbsolutePanTiltPosition(short* panPositionDegrees, short* tiltPositionDegrees)
{
    unsigned char panTiltPositionInquiry[] = {0x09, 0x06, 0x12}; // 'Pan-tiltPosInq'
    unsigned char panTiltPositionReply[MAX_PTZ_PACKET_LENGTH];
    int panTiltPositionReplyLength;

    short absPanPosition, absTiltPosition;

    if ((panTiltPositionReplyLength = SendVISCAInquiry(panTiltPositionInquiry, 3, panTiltPositionReply)) <= 0)
    {
        return panTiltPositionReplyLength;
    }

    /* Get the 4 bytes that represent the current pan position */
    absPanPosition = panTiltPositionReply[5];
    absPanPosition |= (panTiltPositionReply[4] <<  4);
    absPanPosition |= (panTiltPositionReply[3] <<  8);
    absPanPosition |= (panTiltPositionReply[2] << 12);

    /* Set the current pan position with respect to the conversion factor */
    *panPositionDegrees = (short) round(absPanPosition / ptzCamConfig.panConversionFactor);

    /* Next 4 bytes are the current tilt position */
    absTiltPosition = panTiltPositionReply[9];
    absTiltPosition |= (panTiltPositionReply[8] <<  4);
    absTiltPosition |= (panTiltPositionReply[7] <<  8);
    absTiltPosition |= (panTiltPositionReply[6] << 12);

    /* Set the current tilt position */
    *tiltPositionDegrees = (short) round(absTiltPosition / ptzCamConfig.tiltConversionFactor);

    return 0;
}

/* Returns the absolute zoom position, takes a pointer just like the function to return the pan / tilt position */
int SonyPTZ::GetAbsoluteZoomPosition(short* zoomPosition)
{
    unsigned char zoomPositionInquiry[] = {0x09, 0x04, 0x47}; // 'CAM_ZoomPosInq'  
    unsigned char zoomPositionInquiryReply[MAX_PTZ_PACKET_LENGTH];
    int zoomPositionReplyLength;

    if ((zoomPositionReplyLength = SendVISCAInquiry(zoomPositionInquiry, 3, zoomPositionInquiryReply)) <= 0)
    {
        return zoomPositionReplyLength;
    }

    *zoomPosition = zoomPositionInquiryReply[5]; 

    /* Skip the first two header bytes */
    *zoomPosition |= (zoomPositionInquiryReply[4] <<  4);
    *zoomPosition |= (zoomPositionInquiryReply[3] <<  8);
    *zoomPosition |= (zoomPositionInquiryReply[2] << 12);

    return 0;
}

/* Function to set the absolute pan and tilt position given a position in camera units */
int SonyPTZ::SetAbsolutePanTiltPosition(short panPositionRequestDegrees, short tiltPositionRequestDegrees, bool waitForCompletion)
{
    unsigned char setAbsPanTiltCommand[MAX_PTZ_MESSAGE_LENGTH];
    short convertedPan, convertedTilt;

    this->waitForCompletion = waitForCompletion; // Set whether the camera will block any new position requests until this request has completed 

    /* Threshold values if outside the limits, only takes effect if camera has been calibrated and we know maximum limits */
    if (ptzCamConfig.cameraIsCalibrated)
    {
        /* Threshold if position requested is outside the maximum or minimum values */
        if (panPositionRequestDegrees >  ptzCamConfig.maxPanAngleDegrees)
        {
            panPositionRequestDegrees = ptzCamConfig.maxPanAngleDegrees;
            std::cout << "Camera Pan Angle Thresholded to " << ptzCamConfig.maxPanAngleDegrees << std::endl;
        }

        else if (panPositionRequestDegrees < ptzCamConfig.minPanAngleDegrees)
        {
            panPositionRequestDegrees = ptzCamConfig.minPanAngleDegrees;
            std::cout << "Camera Pan Angle Thresholded to " << ptzCamConfig.minPanAngleDegrees << " degrees" << std::endl;
        }

        if (tiltPositionRequestDegrees > ptzCamConfig.maxTiltAngleDegrees)
        {
            tiltPositionRequestDegrees = ptzCamConfig.maxTiltAngleDegrees;
            std::cout << "Camera Tilt Angle Thresholded to " << ptzCamConfig.maxTiltAngleDegrees << " degrees" << std::endl;
        }

        else if (tiltPositionRequestDegrees < ptzCamConfig.minTiltAngleDegrees)
        {
            tiltPositionRequestDegrees = ptzCamConfig.minTiltAngleDegrees;
            std::cout << "Camera Tilt Angle Thresholded to " << ptzCamConfig.minTiltAngleDegrees << " degrees" << std::endl;
        }
    }

    /* If the camera is calibrated we have the conversion values */
    if (ptzCamConfig.cameraIsCalibrated)
    {
        convertedPan  = (short) panPositionRequestDegrees * ptzCamConfig.panConversionFactor;
        convertedTilt = (short) tiltPositionRequestDegrees * ptzCamConfig.tiltConversionFactor;
    }

    else
    {
        convertedPan =  (short) panPositionRequestDegrees;
        convertedTilt = (short) tiltPositionRequestDegrees;
    }

    /* Build the VISCA command, 01 06 02 is the prefix for 'Pan-tiltDrive' commands */
    setAbsPanTiltCommand[0] = 0x01;
    setAbsPanTiltCommand[1] = 0x06;
    setAbsPanTiltCommand[2] = 0x02;

    /* Set using SetAbsolutePanTiltSpeed(), pre-calibration speed is defined in the constructor, speed after will default to the maximum possible speed */
    setAbsPanTiltCommand[3] = this->currentPanSpeed;
    setAbsPanTiltCommand[4] = this->currentTiltSpeed;

    /* Set the pan position */
    setAbsPanTiltCommand[5] = (unsigned char) ((convertedPan & 0xF000) >> 12);
    setAbsPanTiltCommand[6] = (unsigned char) ((convertedPan & 0xF00)  >>  8);
    setAbsPanTiltCommand[7] = (unsigned char) ((convertedPan & 0x00F0) >>  4);
    setAbsPanTiltCommand[8] = (unsigned char) (convertedPan  & 0x00F);

    /* Now set the tilt position */
    setAbsPanTiltCommand[9] =  (unsigned char) ((convertedTilt & 0xF000) >> 12);
    setAbsPanTiltCommand[10] = (unsigned char) ((convertedTilt & 0xF00)  >>  8);
    setAbsPanTiltCommand[11] = (unsigned char) ((convertedTilt & 0x00F0) >>  4);
    setAbsPanTiltCommand[12] = (unsigned char) (convertedTilt  & 0x000F);

    return (SendVISCACommand(setAbsPanTiltCommand, 13));
}

/* This function sets the speed for pan and tilt movement using the VISCA Pan-tiltDrive command, use this if you want to set speed without a specific position */
int SonyPTZ::SetPanTiltSpeed(short panSpeedRequestCU, short tiltSpeedRequestCU)
{
    unsigned char setPanTiltSpeedCommmand[MAX_PTZ_MESSAGE_LENGTH];

    unsigned char panDirection;
    unsigned char tiltDirection;

    if (panSpeedRequestCU == 0)
    {
        panDirection = 0x03; // Either up, down or stop depending on what tilt direction is set to
    }

    else if (panSpeedRequestCU < 0)
    {
        panDirection = 0x01; // If 0x01 is set for the pan direction it relates to a movement to the left (either Left, UpLeft or DownLeft)
    }

    else
    {
        panDirection = 0x02; // Relates to a movement to the right (Right, UpRight, DownRight)
    }

    if (tiltSpeedRequestCU == 0)
    {
        tiltDirection = 0x03; // If 0x03 is set for both pan and tilt direction, the camera will stop 
    }

    else if (tiltSpeedRequestCU < 0)
    {
        tiltDirection = 0x02; 
    }

    else
    {
        tiltDirection = 0x01;
    }

    /* Build the command packet, see 'Pan-tiltDrive' in the VISCA specification */
    setPanTiltSpeedCommmand[0] = 0x01;
    setPanTiltSpeedCommmand[1] = 0x06;
    setPanTiltSpeedCommmand[2] = 0x01;

    setPanTiltSpeedCommmand[3] = (unsigned char) abs(panSpeedRequestCU);
    setPanTiltSpeedCommmand[4] = (unsigned char) abs(tiltSpeedRequestCU);
    setPanTiltSpeedCommmand[5] = panDirection;
    setPanTiltSpeedCommmand[6] = tiltDirection;

    /* Issuing a command to change speed whilst a position command is current fails with an error and doesn't set the speed.
     * The solution used here is the same as what the sonyevid30 driver does which is cancel the current commands
     */
    CancelVISCACommand(1);
    CancelVISCACommand(2);

    return (SendVISCACommand(setPanTiltSpeedCommmand, 7));
}

int SonyPTZ::SetZoomSpeed(short zoomSpeedRequestCU)
{
    zoomSpeedRequestCU = abs(zoomSpeedRequestCU);

    /* Stop zoom if speed received is 0 */
    if (zoomSpeedRequestCU == 0)
    {
        std::cout << "Received zoomspeed request of 0, stopping zoom" << std::endl; 
        return StopZoom();
    }
    
    unsigned char setZoomSpeedCommand[MAX_PTZ_MESSAGE_LENGTH] = {0x01, 0x04, 0x07};

    if (zoomIn)
    {
        /* If the speed value initially passed in is positive then we want to zoom in so we use the VISCA command for a telescopic zoom */
        setZoomSpeedCommand[3] = 0x20 | (zoomSpeedRequestCU & 0x7);
    }

    else
    {
        /* If the speed value initially passed in is negative then we want to zoom out so we use the VISCA command for a wide zoom */
        setZoomSpeedCommand[3] = 0x30 | (zoomSpeedRequestCU & 0x7);
    }

    //printf("zoomSpeedRequestCU: %hi\n", zoomSpeedRequestCU);
    //PrintVISCAPacket("Sending Zoom VISCA Packet", setZoomSpeedCommand, MAX_PTZ_MESSAGE_LENGTH);
    return (SendVISCACommand(setZoomSpeedCommand, 4));
}

/* This function sets the speed for SetAbsolutePanTiltPosition() calls only by setting variables which are passed in
 * This is NOT the function to use if you are controlling the camera using speed only i.e. with SetPanTiltSpeed()
 */
int SonyPTZ::SetAbsolutePanTiltSpeed(short panSpeedRequestCU, short tiltSpeedRequestCU)
{
    if (panSpeedRequestCU > ptzCamConfig.maxPanSpeed)
    {
        std::cerr << "Camera Pan Speed Thresholded to Maximum Value 0x" << std::hex << std::uppercase << ptzCamConfig.maxPanSpeed << std::endl;
        currentPanSpeed  = ptzCamConfig.maxPanSpeed;

        return 0;
    }

    if (tiltSpeedRequestCU > ptzCamConfig.maxTiltSpeed)
    {
        std::cerr << "Camera Tilt Speed Thresholded to Maximum Value 0x" << std::hex << std::uppercase << ptzCamConfig.maxTiltSpeed << std::endl;
        currentTiltSpeed = ptzCamConfig.maxTiltSpeed;

        return 0;
    }

    std::cout << "Pan  Speed: " << std::dec << panSpeedRequestCU << std::endl;
    this->currentPanSpeed  = panSpeedRequestCU;
    std::cout << "Tilt Speed: " << std::dec << tiltSpeedRequestCU << std::endl;
    this->currentTiltSpeed = tiltSpeedRequestCU;

    return 0;
}

/* Set the zoom position using the Direct mode of the 'CAM_Zoom' VISCA command */
int SonyPTZ::SetAbsoluteZoomPosition(short zoomPositionRequestCU)
{
    unsigned char setAbsZoomCommand[MAX_PTZ_MESSAGE_LENGTH] = {0x01, 0x04, 0x47};

    if (zoomPositionRequestCU < 0) 
    {
        zoomPositionRequestCU = 0;
        std::cerr << "Zoom thresholded to minimum value" << std::endl;
    }

    else if (zoomPositionRequestCU > 1023)
    {
        zoomPositionRequestCU = 1023;
        std::cerr << "Zoom thresholded to maximum value" << std::endl;
    }

    // Set the Zoom Position
    setAbsZoomCommand[3] = (unsigned char) ((zoomPositionRequestCU & 0xF000) >> 12);
    setAbsZoomCommand[4] = (unsigned char) ((zoomPositionRequestCU & 0x0F00) >>  8);
    setAbsZoomCommand[5] = (unsigned char) ((zoomPositionRequestCU & 0x00F0) >>  4);
    setAbsZoomCommand[6] = (unsigned char)  (zoomPositionRequestCU & 0x000F);

    return (SendVISCACommand(setAbsZoomCommand, 7));
}

/* Stop the zoom command in progress */
int SonyPTZ::StopZoom()
{
    unsigned char stopZoomCommand[MAX_PTZ_MESSAGE_LENGTH] = {0x01, 0x04, 0x07, 0x00};

    return (SendVISCACommand(stopZoomCommand, 4));
}

/* Set the camera zoom to a wide angle with a given speed (default is maximum speed), see CAM_ZOOM in the specification */
int SonyPTZ::SetWideZoom(uint32_t zoomSpeed)
{
    unsigned char setWideZoomCommand[MAX_PTZ_MESSAGE_LENGTH] = {0x01, 0x04, 0x07};

    if (zoomSpeed > ptzCamConfig.maxZoomSpeed)
    {
        std::cerr << "Zoom Speed Thresholded to " << std::hex << std::uppercase << ptzCamConfig.maxZoomSpeed << std::endl;
        zoomSpeed = ptzCamConfig.maxZoomSpeed;
    }

    else if (zoomSpeed < ptzCamConfig.minZoomSpeed)
    {
        std::cerr << "Zoom Speed Thresholded to " << std::hex << std::uppercase << ptzCamConfig.minZoomSpeed << std::endl;
        zoomSpeed = ptzCamConfig.minZoomSpeed;
    }

    /* 0x30 is the VISCA byte for a wide zoom */
    setWideZoomCommand[3] = 0x30 | (zoomSpeed & 0x7);

    return (SendVISCACommand(setWideZoomCommand, 4));
}

/* Set the camera zoom to a telescopic angle with a given speed */
int SonyPTZ::SetTeleZoom(uint32_t zoomSpeed)
{
    unsigned char setTeleZoomCommand[MAX_PTZ_MESSAGE_LENGTH] = {0x01, 0x04, 0x07};

    if (zoomSpeed > ptzCamConfig.maxZoomSpeed)
    {
        std::cerr << "Zoom Speed Thresholded to " << std::hex << std::uppercase << ptzCamConfig.maxZoomSpeed << std::endl;
        zoomSpeed = ptzCamConfig.maxZoomSpeed;
    }

    else if (zoomSpeed < ptzCamConfig.minZoomSpeed)
    {
        std::cerr << "Zoom Speed Thresholded to " << std::hex << std::uppercase << ptzCamConfig.minZoomSpeed << std::endl;
        zoomSpeed = ptzCamConfig.minZoomSpeed;
    }

    setTeleZoomCommand[3] = 0x20 | (zoomSpeed & 0x7);

    return (SendVISCACommand(setTeleZoomCommand, 4));
}

/* Updates the internal representation of the current pan, tilt and zoom position and velocity */
int SonyPTZ::UpdatePTZData(player_ptz_data_t &ptzData)
{
    short currentPanCU, currentTiltCU, currentZoomCU; // Variables to hold current PTZ values, CU means 'camera unit' (internal representation) not radians or degrees

    /* Check that we can get the current pan, tilt and zoom values */
    if ((GetAbsolutePanTiltPosition(&currentPanCU, &currentTiltCU)) != 0)
    {
        perror("SonyPTZ::UpdatePTZData(): Error acquiring current pan and tilt position. Aborting.");
        return -1;
    }

    if ((GetAbsoluteZoomPosition(&currentZoomCU)) != 0)
    {
        perror("SonyPTZ::UpdatePTZData(): Error acquiring current zoom position. Terminating.");
        return -1;
    }             

    /* The player_ptz_data_t struct expects member values for PTZ to be in radians; */
    ptzData.pan  = panPositionToRadians(currentPanCU); 
    ptzData.tilt = tiltPositionToRadians(ptzCamConfig, currentTiltCU);
    ptzData.zoom = zoomPositionToRadians(ptzCamConfig, currentZoomCU); 

    /* We cannot poll the camera for the current pan or tilt speed.
     * In position mode the speed isn't important and in velocity mode it is reasonable to assume the camera moves at the maximum possible speed 
     * until hitting the pan / tilt / zoom limit when the speed becomes zero 
     */

    /* Check to see if we have hit the pan limit. A negative pan value in camera units will be converted to a positive value in radians */
    if ((currentPanCU <= -ptzCamConfig.maxPanAngleCU && lastExecutedPanSpeedRadians > 0) ||  // If the pan limit HAS been reached or exceeded...
       (currentPanCU  >=  ptzCamConfig.maxPanAngleCU && lastExecutedPanSpeedRadians < 0))
    {
        lastExecutedPanSpeedRadians = 0; // Set the speed to zero
    }

    ptzData.panspeed = lastExecutedPanSpeedRadians; // Update the received command to reflect the last executed speed value

    /* Check to see if we have hit the tilt limit */
    if ((currentTiltCU <= -ptzCamConfig.maxTiltAngleCU  && lastExecutedTiltSpeedRadians < 0) || 
       (currentTiltCU  >=  ptzCamConfig.maxTiltAngleCU  && lastExecutedTiltSpeedRadians > 0)) 
    {
		lastExecutedTiltSpeedRadians = 0;
	}

    ptzData.tiltspeed = lastExecutedTiltSpeedRadians;

    return 0;
}

/* Function to process generic requests sent to the driver, used in the specific ProcessPTZRequest() function */
int SonyPTZ::ProcessGenericRequest(QueuePointer &resp_queue, player_msghdr* hdr, player_ptz_req_generic_t* request) // player_ptz_req_generic_t is defined in player_interfaces.h
{
    /* VISCA Inquiries return immediately with the requested data whereas VISCA Commands return a completion acknowledgement (ACK) */
    if (request->config[0] == VISCA_COMMAND_BYTE) // request->config will contain the reply or command, 0x01 at the beginning of the message indicates a command (as opposed to an inquiry)
    {
        if(SendVISCACommand( (uint8_t*) request->config, request->config_count) < 0) // Cast request->config to uint8_t* from uint32_t*, if the command could not be sent then return failure
        {
            return -1;
        }

        else
        {
            return 0;
        }
    }

    else // If the request was an inquiry, the reply will be put back into the request->config buffer
    {
        request->config_count = SendVISCAInquiry( (uint8_t*) request->config, request->config_count, (uint8_t*) request->config);

        /* Publish the response */
        Publish (device_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK, hdr->subtype);
        
        return -1;
    }
}

/* Specific function to handle PTZ requests, called by the ProcessMessage() function where required */
int SonyPTZ::ProcessPTZRequest(player_ptz_cmd_t* ptzCmd)
{
    switch (PTZControlMode) /* Requests will be processed differently for position and for speed */
    {
        case PLAYER_PTZ_VELOCITY_CONTROL: // Changing speed...
            if (ptzCmd->panspeed != lastRequestedPanSpeedRadians || ptzCmd->tiltspeed != lastRequestedTiltSpeedRadians || ptzCmd->zoomspeed != lastRequestedZoomSpeed) // If the speeds in the command do not match the previous requests...
            {
                /* Update them */
                lastRequestedPanSpeedRadians = ptzCmd->panspeed;
                lastRequestedTiltSpeedRadians = ptzCmd->tiltspeed;
                lastRequestedZoomSpeed = ptzCmd->zoomspeed;
            
                /* Translate from radians to camera units */
                short panSpeedRequestCU = panSpeedToCU(ptzCamConfig, lastRequestedPanSpeedRadians); 
                short tiltSpeedRequestCU = tiltSpeedToCU(ptzCamConfig, lastRequestedTiltSpeedRadians);
                double zoomSpeedRequestCU = zoomSpeedToCU(ptzCamConfig, lastRequestedZoomSpeed);

                /* Assume the camera is moving at the requested speed, if it hits the pan or tilt limit it will be reset in the UpdatePTZData() function */
                lastExecutedPanSpeedRadians = panSpeedToRadians(ptzCamConfig, panSpeedRequestCU);
                lastExecutedTiltSpeedRadians = tiltSpeedToRadians(ptzCamConfig, tiltSpeedRequestCU);
                lastExecutedZoomSpeed = zoomSpeedRequestCU;
            
                /* Attempt to change the pan / tilt speed to match the requested values */
                if (SetPanTiltSpeed(panSpeedRequestCU, tiltSpeedRequestCU))
                {
                    perror("SonyPTZ::ProcessPTZRequest:SetPanTiltSpeed() Error");
                    return -1;
                }   
                
                /* Handle controlling zoom using SetSpeed() in a PtzProxy */
                if (SetZoomSpeed(zoomSpeedRequestCU))
                {
                    perror("SonyPTZ::ProcessPTZRequest:SetZoomSpeed() Error");
                    return -1;
                }
            }

            break;
    
        case PLAYER_PTZ_POSITION_CONTROL: // Changing of position or zoom...
            if (ptzCmd->pan != lastRequestedPanPositionRadians || ptzCmd->tilt != lastRequestedTiltPositionRadians)
            {
                lastRequestedPanPositionRadians = ptzCmd->pan;
                lastRequestedTiltPositionRadians = ptzCmd->tilt;

                /* Translate these to camera units */
                short panPositionRequestCU = panPositionToCU(ptzCamConfig, lastRequestedPanPositionRadians);
                short tiltPositionRequestCU = tiltPositionToCU(ptzCamConfig, lastRequestedTiltPositionRadians);

                /* Attempt to send the command to change position */
                if (SetAbsolutePanTiltPosition(panPositionRequestCU, tiltPositionRequestCU, false))
                {
                    perror("SonyPTZ::ProcessPTZRequest::SetAbsolutePanTiltPosition(): Error setting new pan / tilt positions");
                    return -1;
                }
            }

            /* Changing zoom... */
            if (ptzCmd->zoom != 0 && ptzCmd->zoom != lastRequestedZoomPositionRadians)
            {
                lastRequestedZoomPositionRadians = ptzCmd->zoom;

                /* Translate */
                short zoomPositionRequestCU = zoomPositionToCU(ptzCamConfig, lastRequestedZoomPositionRadians);

                /* Attempt to send the command to change the zoom level */
                if ((SetAbsoluteZoomPosition(zoomPositionRequestCU)))
                {
                    perror("SonyPTZ::ProcessPTZRequest::SetAbsoluteZoomPosition(): Error setting new zoom position");
                    return -1;
                }
            }

            /* The sonyevid30 driver uses this function to set the pan and tilt speed back to zero when in position mode, this is replicated here for compatibility purposes  
             * In a future version it would be sensible to set them to the real values 
             */
            lastExecutedPanSpeedRadians = 0;
            lastExecutedTiltSpeedRadians = 0;
            break;

        default:
            perror("SonyPTZ::ProcessPTZRequest(): Control mode not recognised, should be either PLAYER_PTZ_VELOCITY_CONTROL or PLAYER_PTZ_POSITION_CONTROL");
            return -1;
    }

    return 0;
}

/* Function to process messages, uses the ProcessGenericRequest() and ProcessPTZRequest() functions depending on message type */
int SonyPTZ::ProcessMessage(QueuePointer &resp_queue, player_msghdr* msgHeader, void* data)
{
    /* Process messages here.  Send a response if necessary, using Publish().
    * If the message  is handled successfully, return 0.  Otherwise,
    * return -1 and a NACK will be sent if a response is required.
    */

    assert(msgHeader); /* Ensure the Player message header (defined in player.h) is not null, every Player message starts with this header */

    if (Message::MatchMessage(msgHeader, PLAYER_MSGTYPE_REQ, PLAYER_PTZ_REQ_GENERIC, device_addr)) // Uses the Message class defined in player.h, if the message received is a generic PTZ request...
    {
        assert(msgHeader->size == sizeof(player_ptz_req_generic_t)); // Ensure the header is the correct size for a generic request
        
        return ProcessGenericRequest(resp_queue, msgHeader, (player_ptz_req_generic_t *) data); // Cast the void data pointer to have a type of player_ptz_req_generic_t* and process the generic request
    }

    /* Place for future implementation of the GetStatus() method in the C++ PtzProxy, this returns information on the connected camera */
    else if (Message::MatchMessage(msgHeader, PLAYER_MSGTYPE_REQ, PTZ_PROXY_GET_STATUS_SUBTYPE, device_addr)) 
    {
        Publish(device_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK, msgHeader->subtype);
        std::cout << "PTZ Proxy GetStatus() received. Not yet implemented in this driver, recommend using std::cout on the PtzProxy object instead" << std::endl;
        exit(-1);
        
        return 0;
    }

    else if (Message::MatchMessage(msgHeader, PLAYER_MSGTYPE_CMD, PLAYER_PTZ_CMD_STATE, device_addr)) // If the message received is a PTZ command...
    {
        /* Ensure the header is the expected size for a player PTZ command (player_ptz_cmd_t), also accounting for the additional zoom speed parameter in the modified Player server */
        assert(msgHeader->size == sizeof(player_ptz_cmd_t) || msgHeader->size == (sizeof(player_ptz_cmd_t) - sizeof (float)) ); 

        player_ptz_cmd_t* ptzCmd = reinterpret_cast<player_ptz_cmd_t*> (data); /* Reinterpret the data passed in as a void pointer as being of type 'player_ptz_cmd_t*' */

        if (ProcessPTZRequest(ptzCmd))
        {
            return -1;
        }

        else
        {
            return 0;
        }
    }

    else if (Message::MatchMessage(msgHeader, PLAYER_MSGTYPE_REQ, PLAYER_PTZ_REQ_CONTROL_MODE, device_addr)) // If the message is a PTZ control mode request...
    {
        /* Allow for switching between velocity and position modes, the two supported PTZ modes in Player */
        player_ptz_req_control_mode_t* ctrlMode = reinterpret_cast<player_ptz_req_control_mode_t*> (data); // Reinterpret to correct player_ptz datatype

        if (ctrlMode->mode != PLAYER_PTZ_VELOCITY_CONTROL && ctrlMode->mode != PLAYER_PTZ_POSITION_CONTROL) // If the requested mode is neither velocity nor speed...
        {
            std::cerr << "Undefined PTZ Control Mode Requested: " << ctrlMode->mode << std::endl;
            return -1;
        }

        else
        {
            PTZControlMode = ctrlMode->mode;
            Publish(device_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK, msgHeader->subtype); /* Publish an ACK response to subscribers */
            return 0; // Success
        }
    }

    else
    {
        return -1;
    }
}

/* Demo function used to test basic pan and tilt functionality */
void SonyPTZ::PanTiltDemo()
{
    std::cout << std::endl << "Hit enter to start the hardware demo function..." << std::endl << std::endl;
    std::cin.ignore();

    /* Demonstrate speed control functionality */
    std::cout << "Setting speed..." << std::endl;
    SetAbsolutePanTiltSpeed(0x12, 0x12); 

    /* Basic movement */
    std::cout << std::endl << "Minimum Pan..." << std::endl;
    SetAbsolutePanTiltPosition(ptzCamConfig.minPanAngleDegrees, CENTER, true);
    std::cout << "Maximum Pan..." << std::endl;
    SetAbsolutePanTiltPosition(ptzCamConfig.maxPanAngleDegrees, CENTER, true);
    std::cout << "Minimum Tilt..." << std::endl;
    SetAbsolutePanTiltPosition(CENTER, ptzCamConfig.minTiltAngleDegrees, true);
    std::cout << "Maximum Tilt..." << std::endl;
    SetAbsolutePanTiltPosition(CENTER, ptzCamConfig.maxTiltAngleDegrees, true);

    std::cin.ignore();

    /* Combined movement */
    std::cout << "Maximum Pan, Minimum Tilt..." << std::endl;
    SetAbsolutePanTiltPosition(ptzCamConfig.maxPanAngleDegrees, ptzCamConfig.minTiltAngleDegrees, true);
    std::cout << "Minimum Pan, Minimum Tilt..." << std::endl;
    SetAbsolutePanTiltPosition(ptzCamConfig.minPanAngleDegrees, ptzCamConfig.minTiltAngleDegrees, true);
    std::cout << "Maximum Pan, Maximum Tilt..." << std::endl;
    SetAbsolutePanTiltPosition(ptzCamConfig.maxPanAngleDegrees, ptzCamConfig.maxTiltAngleDegrees, true);
    std::cout << std::endl;

    /* Demonstrate how the camera can accurately report current positions */
    short currentPanPos, currentTiltPos;

    if (GetAbsolutePanTiltPosition(&currentPanPos, &currentTiltPos) != 0)
    {
        perror("SonyPTZ::PanTiltDemo: Error getting current pan / tilt position, terminating");
        exit(1);
    }

    std::cout << "Requesting position data from camera..." << std::endl;
    std::cout << "Current Pan Position: "  << std::dec << currentPanPos << " degrees" << std::endl;
    std::cout << "Current Tilt Position: " << std::dec << currentTiltPos << " degrees" << std::endl;

    std::cin.ignore();

    /* The camera will threshold values above or below the minimum supported by the hardware */
    std::cout << "Thresholding Invalid Values..." << std::endl;
    SetAbsolutePanTiltPosition(ptzCamConfig.maxPanAngleDegrees + 100, ptzCamConfig.minTiltAngleDegrees - 100, true);
    std::cin.ignore(); 

    SetAbsolutePanTiltPosition(CENTER, CENTER, true);

    /* Demonstrate Zoom Functionality */
    short currentZoomPos;

    std::cout << "Telescopic Zoom..." << std::endl;
    SetTeleZoom();
    std::cin.ignore();

    GetAbsoluteZoomPosition(&currentZoomPos);
    std::cout << "Zoom: " << (currentZoomPos / ptzCamConfig.maxZoomCU) * 100 << "%" << std::endl;
    std::cin.ignore();

    std::cout << "Wide Angle Zoom..." << std::endl;
    SetWideZoom();
    std::cin.ignore();    
    GetAbsoluteZoomPosition(&currentZoomPos);
    std::cout << "Zoom: " << (currentZoomPos / ptzCamConfig.maxZoomCU) * 100 << "%" << std::endl;
    std::cin.ignore();
    
    std::cout << "Zoom to arbitrary level..." << std::endl;
    SetAbsoluteZoomPosition(ptzCamConfig.maxZoomCU * 0.45);
    std::cin.ignore();
    GetAbsoluteZoomPosition(&currentZoomPos);
    std::cout << "Zoom Position: " << round( (currentZoomPos / (float) ptzCamConfig.maxZoomCU) * 100) << "%" << std::endl;

    std::cout << std::endl << "Wide Zoom with speed..." << std::endl;
    SetWideZoom(1);
    GetAbsoluteZoomPosition(&currentZoomPos);
    std::cin.ignore();

    SetAbsolutePanTiltSpeed(ptzCamConfig.maxPanSpeed, ptzCamConfig.maxTiltSpeed); 
    std::cin.ignore();

    std::cout << "Returning to Home Position..." << std::endl << std::endl;
    SetAbsolutePanTiltPosition(CENTER, CENTER, true);

    MainQuit();
}

/* The main function, this runs in a separate thread to other functions because the driver is multi-threaded */
void SonyPTZ::Main()
{
    std::cout << "Driver waiting for clients..." << std::endl;

    /* Main loop, runs as long as device has subscribers */
    while (true)
    {
        player_ptz_data_t ptzData; // Instantiate a new struct to hold data returned by the PTZ camera, this is defined in libplayerinterface/interfaces/008_ptz.def
        struct timespec ts; // Stores time for sleep, requires a suitable time.h header

        /* Check if the driver needs to be shutdown, process incoming requests and then check again */
        pthread_testcancel();
        ProcessMessages(); // Inherited function from ThreadedDriver, calls the implemented ProcessMessage() function within this driver for each incoming message
        pthread_testcancel();

        /* Update the ptzData variable */
        if (UpdatePTZData(ptzData)) // If there are errors during the update...
        {
            pthread_exit(NULL); // Terminate the calling thread
        }

        /* Another test to see if we need to cancel */
        pthread_testcancel();

        /* Publish data from communicating with the camera */
        Publish(device_addr, PLAYER_MSGTYPE_DATA, PLAYER_PTZ_DATA_STATE, &ptzData, sizeof(player_ptz_data_t), NULL);
        
        // Suspend thread until the next set of data is available
        ts.tv_sec = 0;
        ts.tv_nsec = SLEEP_TIME_MS * 1000; // Delay is currently set to 100000 microseconds or 0.1 seconds
        nanosleep(&ts, NULL);
    }
}

/* Factory method, declared outside the class definition so it can be used without an existing object. Returns a pointer to a new instance of the PTZ driver */
Driver* SonyPTZ_Init(ConfigFile* cf, int section)
{
    /* Create a new driver instance and return it from this method */
    return( (Driver*) (new SonyPTZ(cf, section)) );
}

/* Driver registration function; required for initialisation, declared outside class definition for the same reasons as the factory method */
void SonyPTZ_Register(DriverTable * table)
{
    table->AddDriver("sony-ptz", SonyPTZ_Init); // Add the driver to the driver table indicating supported interfaces (i.e. PTZ) and the factory function to create a new driver instance
}

/* The following function is the entry point for the driver - it is required for building the shared object (.so, .dylib) containing the plugin driver code
 * The 'extern' is used to prevent C++ compiler name mangling of the player_driver_init function
 * It is crucial the function be named exactly as it is and that it call the register function defined above
 */
extern "C" 
{
  int player_driver_init(DriverTable* table)
  {
    SonyPTZ_Register(table);
    return 0;
  }
}

