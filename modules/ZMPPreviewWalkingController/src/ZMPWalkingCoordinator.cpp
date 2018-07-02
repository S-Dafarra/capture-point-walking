/**
 * @file ZMPWalkingCoordinator.cpp
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#include <ZMPWalkingCoordinator.h>
#include <yarp/os/LogStream.h>

using namespace WalkingControllers;



bool ZMPWalkingCoordinator::prepareWalkingPrivate()
{

}

bool ZMPWalkingCoordinator::stopWalkingPrivate()
{

}

bool ZMPWalkingCoordinator::pauseWalkingPrivate()
{

}

bool ZMPWalkingCoordinator::startWalkingPrivate()
{

}

bool ZMPWalkingCoordinator::onTheFlyStartWalkingPrivate(double smoothingTime)
{

}

bool ZMPWalkingCoordinator::configurePrivate(yarp::os::ResourceFinder &rf)
{
    m_verbose = false;
    if (rf.check("verbose")) {
        m_verbose = true;
    }

    std::string name = rf.check("name", yarp::os::Value("walking-coordinator")).asString();
    setName(name.c_str());

    m_robotComponent = std::make_shared<RobotComponent>();

    if (!(m_robotComponent->configure(rf.findGroup("ROBOT")))) {
        yError() << "[ZMPWalkingCoordinator::configurePrivate] Failed to configure robot.";
        return false;
    }

    return true;
}

bool ZMPWalkingCoordinator::updatePrivate()
{

}

ZMPWalkingCoordinator::ZMPWalkingCoordinator()
{
}

ZMPWalkingCoordinator::~ZMPWalkingCoordinator()
{

}

std::string ZMPWalkingCoordinator::helpDescription() const
{
    std::stringstream ss;
    ss << "ZMP Preview Walking Coordinator module" << std::endl
    << "" << std::endl << std::endl
    << "Usage:" << std::endl
    << "zmpPreviewWalkingController [options]" << std::endl << std::endl

    << "Recognised options:" << std::endl
    << " --help\tprints this description" << std::endl;

    return ss.str();
}

bool ZMPWalkingCoordinator::close()
{

}
