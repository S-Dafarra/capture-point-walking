/**
 * @file RobotComponent.h
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef ROBOTCOMPONENT_H
#define ROBOTCOMPONENT_H

#include <yarp/os/Searchable.h>
#include <iDynTree/Model/Model.h>
#include <JointsSources.h>
#include <JointsSinks.h>
#include <vector>
#include <string>

namespace WalkingControllers {

    class RobotComponent;
}

class WalkingControllers::RobotComponent {

    class RobotComponentImplementation;
    RobotComponentImplementation *m_pimpl;

public:

    RobotComponent();

    ~RobotComponent();

    bool configure(const yarp::os::Searchable& robotComponentSettings);

    JointsSources &allJointsSources(); //order depending on the controlBoards order (only for the joints available in the URDFmodel)

    JointsSources &controlledJointsSources(); //order equal to the one in controlledJoints

    JointsSinks &allJointsSinks();

    JointsSinks &controlledJointsSinks();

    const iDynTree::Model &robotModel() const;
};

#endif // ROBOTCOMPONENT_H
