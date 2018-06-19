/**
 * @file JointsSources.h
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef JOINTSSOURCES_H
#define JOINTSSOURCES_H

#include <yarp/dev/IPidControl.h>

#include <iDynTree/Core/VectorDynSize.h>

#include <vector>
#include <string>

namespace WalkingControllers {
    class JointsSources;
}

class WalkingControllers::JointsSources {
public:
    JointsSources();

    virtual ~JointsSources();

    virtual bool getJointsName(std::vector<std::string>& jointNames) = 0;

    virtual bool getPositions(iDynTree::VectorDynSize& jointsPositionsInRad) = 0;

    virtual bool getVelocities(iDynTree::VectorDynSize& jointsVelInRadPerSec) = 0;

    virtual bool getPositionLimits(std::vector<std::pair<double, double>>& jointsLimitsInRad) = 0;

    virtual bool getVelocityLimits(std::vector<std::pair<double, double>>& jointsLimitsInRad) = 0;

    virtual bool getPositionPIDs(std::vector<yarp::dev::Pid>& positionPIDs) = 0;

    virtual bool getPositionPIDsSmoothingTimes(iDynTree::VectorDynSize& smoothingTimesInSec) = 0;

};

#endif // JOINTSSOURCES_H
