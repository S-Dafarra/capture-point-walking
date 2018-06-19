/**
 * @file JointsSinks.h
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef JOINTSSINKS_H
#define JOINTSSINKS_H

#include <iDynTree/Core/VectorDynSize.h>

#include <yarp/dev/IPidControl.h>

#include <vector>

namespace WalkingControllers {
    class JointsSinks;

    enum class ControlModes;
}

enum class WalkingControllers::ControlModes {
    Position,
    PositionDirect,
    Velocity
};

class WalkingControllers::JointsSinks {
public:
    JointsSinks();

    virtual ~JointsSinks();

    virtual bool setControlMode(const WalkingControllers::ControlModes &controlMode) = 0;

    virtual bool setPositionReference(const iDynTree::VectorDynSize& jointsPositionsInRad, double positioningTimeInSec = 5.0) = 0;

    virtual bool setDirectPositionReference(const iDynTree::VectorDynSize& jointsPositionsInRad) = 0;

    virtual bool setVelocityReference(const iDynTree::VectorDynSize& jointsVelInRadPerSec) = 0;

    virtual bool setPositionPIDs(const std::vector<yarp::dev::Pid>& positionPIDs) = 0;

    virtual bool setPositionPIDsSmoothingTimes(const iDynTree::VectorDynSize& smoothingTimesInSec) = 0;

};


#endif // JOINTSSINKS_H
