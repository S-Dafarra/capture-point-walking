/**
 * @file PIDHandlerComponent.h
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef NONLINEARIKCOMPONENT_H
#define NONLINEARIKCOMPONENT_H

#include <iDynTree/InverseKinematics.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/Position.h>

#include <yarp/os/Searchable.h>
#include <yarp/os/Value.h>

#include <string>

//DEBUG
#include "iDynTree/KinDynComputations.h"

namespace WalkingControllers {

    class NonLinearIKComponent;

}


/*@ Computes the inverse kinematics for a walking problem.
 * The optimisation problem this class solves is the following:
 * \f[
 *
 * \f]
 */
class WalkingControllers::NonLinearIKComponent {

    bool m_verbose;
    iDynTree::InverseKinematics m_ik;

    std::string m_lFootFrame;
    std::string m_rFootFrame;
    std::string m_additionalFrame;

    iDynTree::Transform m_baseTransform;
    iDynTree::Rotation  m_additionalRotation;
    iDynTree::Rotation  m_gravityFrame_R_world;

    iDynTree::VectorDynSize m_jointRegularization, m_guess, m_feedback, m_qResult;

    //DEBUG
    iDynTree::KinDynComputations m_lchecker;
    iDynTree::VectorDynSize m_dummyVel;
    iDynTree::Twist m_dummyBaseVel;
    iDynTree::Vector3 m_dummygrav;

    unsigned int m_solverVerbosity;
    double m_maxCpuTime;

    bool m_prepared, m_configured;

    double m_additionalRotationWeight;
    iDynTree::VectorDynSize m_jointsRegularizationWeights;

    bool prepareIK();

    bool setModel(const iDynTree::Model& model,
                          const std::vector< std::string > & consideredJoints = std::vector<std::string>());

    bool setFootFrame(const std::string& foot,
                      const std::string& footFrame);

public:

    /*@ Constructor
     */
    NonLinearIKComponent();

    /*@ Destructor
     */
    ~NonLinearIKComponent();

    /*@ Set the verbosity level
     *
     * @param verboseMode the verbosity mode
     */
    void setVerboseMode(bool verboseMode);

    /*@ Initialize the IK
     * @param ikOption the options for the IK
     * @param jointList list of joints to be considered for the inverse kinematics
     * @return true on success, false otherwise
     */
    bool configure(yarp::os::Searchable& ikOption, const iDynTree::Model& model, const std::vector<std::string>& jointList);

    bool setJointLimits(std::vector<std::pair<double, double> >& jointLimits);

    bool updateGravityFrameToWorldFrameRotation(const iDynTree::Rotation& gravityFrame_R_world);

    bool updateAdditionalRotation(const iDynTree::Rotation& additionalRotation); //Defined in a "gravity frame", with the z pointing upwards

    bool setFullModelFeedBack(const iDynTree::VectorDynSize& feedback);

    bool setInitialGuess(const iDynTree::VectorDynSize& guess);

    bool setDesiredJointConfiguration(const iDynTree::VectorDynSize& desiredJointConfiguration);

    /*@ Compute the inverse kinematics
     *
     * @param startingFoot the starting stance foot
     * @param guess the initial guess for the solution
     * @return true on success, false otherwise
     */
    bool computeIK(const iDynTree::Transform& leftTransform,
                   const iDynTree::Transform& rightTransform,
                   const iDynTree::Position& comPosition,
                   iDynTree::VectorDynSize& result);


    const std::string getLeftFootFrame() const;

    const std::string getRightFootFrame() const;

    bool usingAdditionalRotationTarget();

    const iDynTree::VectorDynSize& desiredJointConfiguration() const;

    bool setAdditionalRotationWeight(double weight);

    double additionalRotationWeight();

    bool setDesiredJointsWeights(const iDynTree::VectorDynSize& desiredJointsWeights);

    const iDynTree::VectorDynSize& desiredJointsWeights();
};

#endif // NONLINEARIKCOMPONENT_H
