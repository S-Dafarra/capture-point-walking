/**
 * @file main.cpp
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <ZMPWalkingCoordinator.h>


int main(int argc, char * argv[])
{
    // YARP setting
    yarp::os::Network yarp;
    if (!yarp::os::Network::checkNetwork(5.0))
    {
        yError() << " YARP server not available!";
        return EXIT_FAILURE;
    }

    // Configure ResourceFinder
    yarp::os::ResourceFinder &rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    rf.setDefaultConfigFile("zmpWalking.ini");
    rf.configure(argc, argv);

    // Configure the module
    WalkingControllers::ZMPWalkingCoordinator module;

    if (rf.check("help")) {
        std::cout << module.helpDescription();
        return 0;
    }

    return module.runModule(rf);
}


