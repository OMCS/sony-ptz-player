#include <iostream>
#include <libplayerc++/playerc++.h>
#include <libplayerc++/playerclient.h>
#include <pthread.h>
#include <unistd.h>

using namespace PlayerCc;

int main()
{
    PlayerClient robot("localhost");
    PtzProxy     ptz(&robot, 0);

    short minPanAngleDegrees = 0, minTiltAngleDegrees = 0, maxPanAngleDegrees = 0, maxTiltAngleDegrees = 0;
    bool foundLimits = false;

    while (true)
    {
        ptz.SelectControlMode(PLAYER_PTZ_POSITION_CONTROL);
        ptz.SetCam(DTOR(0), DTOR(0), DTOR(0));
        std::cout << "Hit enter to start the sony-ptz client test program..." << std::endl;
        std::cin.get();

        /* We cannot query the mininum and maximum values directly from the driver unless we read from its config file, so we trust the calibration data is correct */
        if (!foundLimits)
        {
            std::cout << "Testing hardware limits reported by the driver...\n" << std::endl;
            ptz.SetCam(DTOR(-500), DTOR(-500), 0);
            sleep(8);
            robot.Read();

            minPanAngleDegrees  = RTOD(ptz.GetPan());
            minTiltAngleDegrees = RTOD(ptz.GetTilt());
            
            /* Occasionally there is an issue with incorrect readings, this causes the proxy to take another reading if the pan value is unusually high */
            if (minPanAngleDegrees > -30 || minTiltAngleDegrees > -15)
            {
                robot.Read();
                minPanAngleDegrees = RTOD(ptz.GetPan());
                minTiltAngleDegrees = RTOD(ptz.GetTilt());
            }

            if (abs(minPanAngleDegrees) % 2 != 0)
            {
                minPanAngleDegrees--;
            }

            if (abs(minTiltAngleDegrees) %2 != 0)
            {
                minTiltAngleDegrees--;
            }

            std::cout << "Minimum Pan  is "  << minPanAngleDegrees  << " degrees" << std::endl;
            std::cout << "Minimum Tilt is "  << minTiltAngleDegrees << " degrees" << std::endl;

            ptz.SetCam(DTOR(1000), DTOR(1000), 0);
            sleep(8);
            robot.Read();

            maxPanAngleDegrees  = RTOD(ptz.GetPan());
            maxTiltAngleDegrees = RTOD(ptz.GetTilt());

            if (maxPanAngleDegrees % 2 != 0)
            {
                maxPanAngleDegrees++;
            }

            if (maxTiltAngleDegrees % 2 != 0)
            {
                maxTiltAngleDegrees++;
            }

            if (maxPanAngleDegrees > 180)
            {
                robot.Read();
                maxPanAngleDegrees = RTOD(ptz.GetPan());
            }

            std::cout << "\nMaximum Pan  is "  << maxPanAngleDegrees  << " degrees" << std::endl;
            std::cout << "Maximum Tilt is "  << maxTiltAngleDegrees << " degrees" << std::endl;

            foundLimits = true;
        }

        std::cout << "\nHardware limits and camera communications have been verified, moving on to movement and zoom commands\n" << std::endl;
        ptz.SetCam(DTOR(0), DTOR(0), DTOR(0));
        sleep(2);

        ptz.SelectControlMode(PLAYER_PTZ_VELOCITY_CONTROL);
        std::cout << "Demonstrating zoom speed functionality using SetSpeed()..." << std::endl;
        std::cin.ignore();
        ptz.SetSpeed(0, 0, 5.0);
        sleep(2);
        ptz.SetSpeed(0, 0, -7.0);
        sleep(2);

        ptz.SelectControlMode(PLAYER_PTZ_POSITION_CONTROL);
        std::cout << "Moving to one quarter of the maximum pan angle and half the maximum tilt angle using SetCam()..." << std::endl;
        std::cin.ignore();
        ptz.SetCam(DTOR(maxPanAngleDegrees / 4), DTOR( (minTiltAngleDegrees + maxTiltAngleDegrees) / 2), 0);
        sleep(4);
        robot.Read();

        ptz.SelectControlMode(PLAYER_PTZ_VELOCITY_CONTROL);
        std::cout << "Moving towards maximum pan angle and minimum tilt angle using SetSpeed()..." << std::endl;
        std::cin.ignore();
        robot.Read();
        ptz.SetSpeed(DTOR(50), DTOR(-50), -7);
        sleep(1);

        std::cout << "Stopping by setting speeds to zero" << std::endl;
        ptz.SetSpeed(0, 0, 0);
        sleep(2);

        std::cout << "Move towards the centre and stop when approaching zero on x and y axes...\n" << std::endl;
        while (true)
        {
            robot.Read();

            if (RTOD(ptz.GetPan()) > 0)
            {
                ptz.SetSpeed(DTOR(-20), DTOR(0), 0);
            }

            else
            {
                ptz.SetSpeed(DTOR(0), DTOR(0), 0);
                break;
            }
        }

        while (true)
        {
            robot.Read();

            if (RTOD(ptz.GetTilt()) < 0)
            {
                ptz.SetSpeed(DTOR(0), DTOR(20), 0);
            }

            else
            {
                ptz.SetSpeed(DTOR(0), DTOR(0), 0);
                break;
            }
        }

        std::cout << "Pan: " << RTOD(ptz.GetPan()) << std::endl;
        std::cout << "Tilt: " << RTOD(ptz.GetTilt()) << std::endl;

        sleep(2);
        std::cout << "\nDemonstrating stopping in progress zoom by setting speed to zero...\n" << std::endl;
        ptz.SetSpeed(0, 0, 7);
        sleep(2);
        ptz.SetSpeed(0, 0, 0);

        std::cout << "Hit enter to repeat the demo." << std::endl;
        std::cin.ignore();
    }

    return 0;
}

