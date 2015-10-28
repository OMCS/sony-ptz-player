# CS39440 Major Project by Oliver Saunders #
This repository contains resources for my third year major project as part of my BSc degree at Aberystwyth University.

## Sony PTZ Camera Driver for Player ##
This project aims to provide a refactored and improved version of the open source *sonyevid30* driver for using Sony pan-tilt-zoom cameras (models EVI-D30, D70, D100) with the *Player* robotics framework. It aims to provide bug fixes, new features and a significant amount of documentation.

This improved driver (known as Sony-PTZ) offers a unique calibration function and a number of bug-fixes, it provides a restructured codebase and a number of useful additions to the original driver.

There are also minor changes to the *Player* codebase to allow for controlling camera zoom speed via a C++ *ptzProxy*, provided in a patch file. 

## Downloading *Player* and Applying the Patch ##

The patch is designed for the latest (3.1.0-svn as of summer 2015) SVN trunk release of *Player*. To download this install subversion and run the following command:

    svn checkout svn://svn.code.sf.net/p/playerstage/svn/code/player/trunk player

After this, find the .patch file included in the ./src/ directory, change directory to the directory containing the *Player* source code and apply the changes using the UNIX *patch* command:

    patch -p0 < ../Add_Zoom_Speed_Support.patch 

## Next Steps ##

Once the previous steps have been carried out, compile *Player* using *CMake* (create a "build" directory, enter it and then run *cmake ../* followed by the usual *make* and *make install*) and then use the provided "run.sh" script to compile and start the driver. 

Ensure the options in the default configuration file(s) (there is one for Linux and one for other compatible systems) are set the way you want them. You may need to change the serial port address or other configuration data. Note that there is a built in hardware test and demo function provided in the driver which is controlled by use of the *demo* configuration flag.

## Testing ##

There are more details provided in the report (see the "doc" directory) but if you wish to quickly test the driver, you can use the provided test program in the "test" directory - compile this using *cmake*, start the *Player* server using the provided configuration file and run the "sony-ptz-test" binary to see it in action. After each message is printed to the screen hit the enter key to proceed to the next test.

## Licensing ##

All provided code is hereby released under the terms of the GNU GPL license, version 2. The *sony-ptz* driver is made possible by the *sonyevid30* driver developed by Brian Gerkey, Brad Tonkes et al. and significant portions of the codebase were used both directly and indirectly in its creation, with full attribution and gratitude. 

