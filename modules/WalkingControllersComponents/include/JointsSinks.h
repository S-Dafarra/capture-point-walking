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
}

class WalkingControllers::JointsSinks {
public:
    JointsSinks();

    virtual ~JointsSinks();

    virtual bool setControlMode(int controlMode) = 0;

    virtual bool setDesiredPositions(const iDynTree::VectorDynSize& jointsPositionsInRad) = 0;

    virtual bool setDesiredVelocities(const iDynTree::VectorDynSize& jointsVelInRadPerSec) = 0;

    virtual bool setPositionPIDs(const std::vector<yarp::dev::Pid>& positionPIDs) = 0;

    virtual bool setPositionPIDsSmoothingTimes(const iDynTree::VectorDynSize& smoothingTimesInSec) = 0;

};


#endif // JOINTSSINKS_H
