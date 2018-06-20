/**
 * @file RobotComponent.h
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef ROBOTCOMPONENT_H
#define ROBOTCOMPONENT_H

#include <yarp/os/Value.h>
#include <iDynTree/Model/Model.h>
#include <vector>
#include <string>

namespace WalkingControllers {
    class JointsSources;

    class JointsSinks;

    class RobotComponent;
}

class WalkingControllers::RobotComponent {

    class RobotComponentImplementation;
    RobotComponentImplementation *m_pimpl;

public:

    RobotComponent();

    ~RobotComponent();

    bool configure(const std::string& robotName, const yarp::os::Value &remoteControlBoards,
                   iDynTree::Model& URDFmodel, const std::vector<std::string> &controlledJoints);

    JointsSources &allJointsSources(); //order equal to the one in the defined model

    JointsSources &controlledJointsSources(); //order equal to the one in the defined model

    JointsSinks &allJointsSinks();

    JointsSinks &controlledJointsSinks();
};

#endif // ROBOTCOMPONENT_H
