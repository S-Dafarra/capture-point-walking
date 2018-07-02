/**
 * @file NonLinearIKComponent.cpp
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#include <NonLinearIKComponent.h>

#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <Eigen/Core>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <iDynTree/yarp/YARPConfigurationsLoader.h>

#include <sstream>

//DEBUG
#include "iDynTree/KinDynComputations.h"

using namespace WalkingControllers;

NonLinearIKComponent::NonLinearIKComponent()
: m_verbose(false)
, m_lFootFrame("l_sole")
, m_rFootFrame("r_sole")
, m_gravityFrame_R_world(iDynTree::Rotation::Identity())
, m_prepared(false)
, m_configured(false)
, m_additionalRotationWeight(1.0)
{}

NonLinearIKComponent::~NonLinearIKComponent()
{
}

void NonLinearIKComponent::setVerboseMode(bool verboseMode) { m_verbose = verboseMode; }

bool NonLinearIKComponent::configure(yarp::os::Searchable& ikOption, const iDynTree::Model& model, const std::vector<std::string>& jointList)
{
    if (m_configured) {
        yError() << "[NonLinearIKComponent::configure] Cannot configure twice.";
        return false;
    }

    m_solverVerbosity = static_cast<unsigned int>(ikOption.check("solver-verbosity",yarp::os::Value(0)).asInt());
    m_maxCpuTime = ikOption.check("max-cpu-time",yarp::os::Value(0.2)).asDouble();
    std::string lFootFrame = ikOption.check("left_foot_frame", yarp::os::Value("l_sole")).asString();
    std::string rFootFrame = ikOption.check("right_foot_frame", yarp::os::Value("r_sole")).asString();
    std::string solverName = ikOption.check("solver_name", yarp::os::Value("mumps")).asString();
    m_additionalFrame = ikOption.check("additional_frame", yarp::os::Value("")).asString();
    if(m_additionalFrame.size()!=0){
        if(!iDynTree::parseRotationMatrix(ikOption, "additional_rotation", m_additionalRotation)){
            m_additionalRotation = iDynTree::Rotation::Identity();
            yInfo() << "[NonLinearIKComponent::configure] Using the identity as desired rotation for the additional frame";
        }
        if(m_verbose)
            yInfo() << "[NonLinearIKComponent::configure] Desired rotation for the additional frame:\n" << m_additionalRotation.toString();
    }
    yarp::os::Value jointRegularization = ikOption.find("jointRegularization");

    if(!setModel(model, jointList)){
        yError()<<"[NonLinearIKComponent::configure] Error while loading the model.";
        return false;
    }

    if(m_verbose){
        double joint_min;
        double joint_max;

        //for each joint, ask the limits
        yInfo() << "[NonLinearIKComponent::configure] Joint Limits:";
        for (iDynTree::JointIndex jointIdx = 0; jointIdx < static_cast<int>(m_ik.reducedModel().getNrOfJoints()); ++jointIdx) {
            iDynTree::IJointConstPtr joint = m_ik.reducedModel().getJoint(jointIdx);
            //if the joint does not have limits skip it
            if (!joint->hasPosLimits())
                continue;
            //for each DoF modelled by the joint get the limits
            for (unsigned dof = 0; dof < joint->getNrOfDOFs(); ++dof) {
                if (!joint->getPosLimits(dof,
                                        joint_min,
                                        joint_max))
                    continue;
                yInfo() << m_ik.reducedModel().getJointName(jointIdx) << ": " <<  iDynTree::rad2deg(joint_min) << " to " << iDynTree::rad2deg(joint_max);
            }
        }

        m_baseTransform = m_ik.fullModel().getFrameTransform( m_ik.fullModel().getFrameIndex(m_lFootFrame) ).inverse();

        iDynTree::LinkIndex baseDebug = m_ik.reducedModel().getFrameLink(m_ik.reducedModel().getFrameIndex(m_lFootFrame));
        m_lchecker.loadRobotModel(m_ik.reducedModel());
        if(!m_lchecker.setFloatingBase(m_ik.reducedModel().getLinkName(baseDebug)))
            return false;

        m_dummyVel.resize(static_cast<unsigned int>(m_ik.reducedModel().getNrOfDOFs()));
        m_dummyVel.zero();
        m_dummyBaseVel.zero();
        m_dummygrav.zero();
        m_lchecker.setRobotState(m_baseTransform, m_dummyVel, m_dummyBaseVel, m_dummyVel, m_dummygrav);
        if(m_additionalFrame.size()!=0){

            yInfo() << "[NonLinearIKComponent::configure] Relative rotation between" << m_lFootFrame << " and " << m_additionalFrame << ":\n" << m_lchecker.getRelativeTransform(m_lFootFrame, m_additionalFrame).getRotation().toString();
        }
        yInfo() << "[NonLinearIKComponent::configure]  Position of the COM in zero position for all the joints: " << m_lchecker.getCenterOfMassPosition().toString();
    }

    if(!(jointRegularization.isNull())){
        if (!jointRegularization.isList() || !jointRegularization.asList()){
            yError()<<"[NonLinearIKComponent::configure] Unable to read the jointRegularization list.";
            return false;
        }
        yarp::os::Bottle *guessValue = jointRegularization.asList();

        if(guessValue->size() != m_ik.reducedModel().getNrOfDOFs()){
            std::ostringstream errorMsg;
            errorMsg << "[NonLinearIKComponent::configure] The jointRegularization list should have the same dimension of the number of DoFs of the provided model.";
            errorMsg << "Model = " << m_ik.reducedModel().getNrOfDOFs();
            errorMsg << "Guess = " << guessValue->size();

            yError() << errorMsg.str();
            return false;
        }

        for(size_t i = 0; i < guessValue->size(); ++i){
            if(!guessValue->get(i).isDouble() && !guessValue->get(i).isInt()){
                yError("[NonLinearIKComponent::configure] The jointRegularization value is expected to be a double");
            }
            m_jointRegularization(static_cast<unsigned int>(i)) = guessValue->get(i).asDouble()*iDynTree::deg2rad(1);
        }
        m_guess = m_jointRegularization;
    }

    m_jointsRegularizationWeights.resize(m_jointRegularization.size());
    yarp::os::Value regularizationWeights = ikOption.find("regularizationWeight");

    if (!(regularizationWeights.isNull())) {
        if (regularizationWeights.isList()) {
            yarp::os::Bottle *weightsValues = regularizationWeights.asList();

            if (weightsValues->size() == 1){
                if (weightsValues->get(0).isDouble() || weightsValues->get(0).isInt()){
                    iDynTree::toEigen(m_jointsRegularizationWeights).setConstant(regularizationWeights.asDouble());
                } else {
                    yError("[NonLinearIKComponent::configure] Error while reading the regularizationWeight list.");
                    return false;
                }
            } else {

                if (weightsValues->size() != m_ik.reducedModel().getNrOfDOFs()){
                    yError("[NonLinearIKComponent::configure] The regularization list should have the same dimension of the number of DoFs of the provided model. Model = %lu, Guess = %lu.", m_ik.reducedModel().getNrOfDOFs(), weightsValues->size());
                    return false;
                }

                for(size_t i = 0; i < weightsValues->size(); ++i){
                    if(!weightsValues->get(i).isDouble() && !weightsValues->get(i).isInt()){
                        yError("[NonLinearIKComponent::configure] The regularizationWeight values are expected to be double");
                    }
                    m_jointsRegularizationWeights(static_cast<unsigned int>(i)) = weightsValues->get(i).asDouble();
                }
            }

        } else if (regularizationWeights.isDouble() || regularizationWeights.isInt()){
            iDynTree::toEigen(m_jointsRegularizationWeights).setConstant(regularizationWeights.asDouble());
        } else {
            yError("Error while reading the regularizationWeight field.");
            return false;
        }
    } else {
        iDynTree::toEigen(m_jointsRegularizationWeights).setConstant(0.5);
    }

    if(!this->setFootFrame("left",lFootFrame)){
        yError() << "[NonLinearIKComponent::configure] Unable to select frame:" << lFootFrame;
        return false;
    }

    if(!this->setFootFrame("right",rFootFrame)){
        yError() << "[NonLinearIKComponent::configure] Unable to select frame:" << rFootFrame;
        return false;
    }

    m_ik.setLinearSolverName(solverName);

    if (m_verbose) {
        yInfo() << "[NonLinearIKComponent::configure] Solver verbosity: " << m_solverVerbosity;
        yInfo() << "[NonLinearIKComponent::configure] Max CPU time: " << m_maxCpuTime;
        yInfo() << "[NonLinearIKComponent::configure] Joint Regularization (RAD): " << m_jointRegularization.toString();
    }

    if (!prepareIK())
        return false;

    m_configured = true;

    return true;
}

bool NonLinearIKComponent::setJointLimits(std::vector<std::pair<double, double> > &jointLimits)
{
    if (!m_configured) {
        yError() << "[NonLinearIKComponent::setJointLimits] First you have to call the configured method.";
        return false;
    }

    if (jointLimits.size() != m_ik.fullModel().getNrOfDOFs()) {
        yError() << "[NonLinearIKComponent::setJointLimits] The input joint limits needs to have dimension equal to the total number of DoFs defined in the provided model.";
        return false;
    }

    if (!m_ik.setJointLimits(jointLimits)) {
        yError() << "[NonLinearIKComponent::setJointLimits] Failed to set joint limits to the IK.";
        return false;
    }

    return true;
}


bool NonLinearIKComponent::setModel(const iDynTree::Model& model, const std::vector< std::string >& consideredJoints)
{
    if(!(m_ik.setModel(model,consideredJoints)))
        return false;

    m_feedback.resize(static_cast<unsigned int>(m_ik.fullModel().getNrOfDOFs()));
    m_feedback.zero();

    m_jointRegularization.resize(static_cast<unsigned int>(m_ik.reducedModel().getNrOfDOFs()));
    m_jointRegularization.zero();

    m_guess.resize(static_cast<unsigned int>(m_ik.reducedModel().getNrOfDOFs()));
    m_guess.zero();

    m_qResult.resize(static_cast<unsigned int>(m_ik.reducedModel().getNrOfDOFs()));
    m_qResult.zero();

    m_prepared = false;

    return true;
}

bool NonLinearIKComponent::setFootFrame(const std::string& foot, const std::string& footFrame)
{
    if (foot == "left"){
        m_lFootFrame = footFrame;
    }
    else if (foot == "right"){
        m_rFootFrame = footFrame;
    }
    else{
        yError() << "[NonLinearIKComponent::setFootFrame] Allowed foot parameters: \"left\", \"right\".";
        return false;
    }

    m_prepared = false;

    return true;
}

bool NonLinearIKComponent::updateGravityFrameToWorldFrameRotation(const iDynTree::Rotation &gravityFrame_R_world)
{
    if (!m_configured) {
        yError() << "[NonLinearIKComponent::updateGravityFrameToWorldFrameRotation] First you have to call the configured method.";
        return false;
    }

    m_gravityFrame_R_world = gravityFrame_R_world;
    return true;
}

bool NonLinearIKComponent::setFullModelFeedBack(const iDynTree::VectorDynSize& feedback)
{
    if (!m_configured) {
        yError() << "[NonLinearIKComponent::setFullModelFeedBack] First you have to call the configured method.";
        return false;
    }

    if(feedback.size() != m_ik.fullModel().getNrOfDOFs()){
        yError("[NonLinearIKComponent::setFullModelFeedBack] The feedback is expected to have the same dimension of the total number of degrees of freedom of the model. Input -> %d != %lu <- Model.",feedback.size(), m_ik.fullModel().getNrOfDOFs());
        return false;
    }
    m_feedback = feedback;
    return true;
}


bool NonLinearIKComponent::prepareIK()
{
    if(m_ik.reducedModel().getNrOfDOFs() == 0){
        yError() << "[NonLinearIKComponent::prepareIK] First you have to load a model.";
        return false;
    }

    m_ik.clearProblem();

    m_ik.setMaxCPUTime(m_maxCpuTime);
    m_ik.setVerbosity(m_solverVerbosity);

    m_ik.setRotationParametrization(iDynTree::InverseKinematicsRotationParametrizationRollPitchYaw);
    m_ik.setDefaultTargetResolutionMode(iDynTree::InverseKinematicsTreatTargetAsConstraintFull);

    iDynTree::LinkIndex base = m_ik.fullModel().getFrameLink(m_ik.fullModel().getFrameIndex(m_lFootFrame));

    m_baseTransform = m_ik.fullModel().getFrameTransform( m_ik.fullModel().getFrameIndex(m_lFootFrame) ).inverse();

    if(!m_ik.setFloatingBaseOnFrameNamed(m_ik.fullModel().getLinkName(base))){
        yError() << "[NonLinearIKComponent::prepareIK] Invalid frame selected for the left foot: "<< m_lFootFrame;
        return false;
    }

    m_ik.addFrameConstraint(m_lFootFrame,iDynTree::Transform::Identity());

    m_ik.setCOMAsConstraint(true);

    bool ok;
    ok = m_ik.addTarget(m_rFootFrame, iDynTree::Transform::Identity());
    if(!ok){
        yError() << "[NonLinearIKComponent::prepareIK] Unable to add a constraint for "<< m_rFootFrame << ".";
        return false;
    }
    m_ik.setTargetResolutionMode(m_rFootFrame, iDynTree::InverseKinematicsTreatTargetAsConstraintFull);

    if(m_additionalFrame.size() != 0){
        ok = m_ik.addRotationTarget(m_additionalFrame, m_additionalRotation, m_additionalRotationWeight);
        if(!ok){
            yError() << "[NonLinearIKComponent::prepareIK] Unable to add a rotation target on "<< m_additionalFrame << ".";
            return false;
        }
        m_ik.setTargetResolutionMode(m_additionalFrame, iDynTree::InverseKinematicsTreatTargetAsConstraintNone);
    }

    m_ik.setCostTolerance(1e-4);
    m_ik.setConstraintsTolerance(1e-4);
    m_ik.setCOMAsConstraintTolerance(1e-4);

    ///DEBUG
    iDynTree::LinkIndex baseDebug = m_ik.reducedModel().getFrameLink(m_ik.reducedModel().getFrameIndex(m_lFootFrame));
    m_lchecker.loadRobotModel(m_ik.reducedModel());
    if(!m_lchecker.setFloatingBase(m_ik.reducedModel().getLinkName(baseDebug)))
        return false;

    m_dummyVel.resize(static_cast<unsigned int>(m_ik.reducedModel().getNrOfDOFs()));
    m_dummyVel.zero();
    m_dummyBaseVel.zero();
    m_dummygrav.zero();

    m_prepared = true;

    return true;
}

bool NonLinearIKComponent::updateAdditionalRotation(const iDynTree::Rotation& additionalRotation)
{
    if (!m_configured) {
        yError() << "[NonLinearIKComponent::updateAdditionalRotation] First you have to call the configured method.";
        return false;
    }

    if(m_additionalFrame.size() == 0){
        yError() << "[NonLinearIKComponent::updateAdditionalRotation] Cannot update the additional rotation if no frame is provided.";
        return false;
    }
    m_additionalRotation = additionalRotation;
    return true;
}

bool NonLinearIKComponent::setInitialGuess(const iDynTree::VectorDynSize& guess)
{
    if (!m_configured) {
        yError() << "[NonLinearIKComponent::setInitialGuess] First you have to call the configured method.";
        return false;
    }

    if(guess.size() != m_ik.reducedModel().getNrOfDOFs()){
        yError("[nonlinearikcomponent::setInitialGuess] The guess is expected to have the same dimension of the reduced number of degrees of freedom. Input -> %d != %lu <- Model.",guess.size(), m_ik.reducedModel().getNrOfDOFs());
        return false;
    }

    m_guess = guess;

    return true;
}

bool NonLinearIKComponent::setDesiredJointConfiguration(const iDynTree::VectorDynSize& desiredJointConfiguration)
{
    if (!m_configured) {
        yError() << "[NonLinearIKComponent::setDesiredJointConfiguration] First you have to call the configured method.";
        return false;
    }

    if(desiredJointConfiguration.size() != m_ik.reducedModel().getNrOfDOFs()){
        yError("[nonlinearikcomponent::setDesiredJointConfiguration] The desiredJointConfiguration is expected to have the same dimension of the reduced number of degrees of freedom. Input -> %d != %lu <- Model.",desiredJointConfiguration.size(), m_ik.reducedModel().getNrOfDOFs());
        return false;
    }

    m_jointRegularization = desiredJointConfiguration;

    return true;
}



bool NonLinearIKComponent::computeIK(const iDynTree::Transform& leftTransform, const iDynTree::Transform& rightTransform, const iDynTree::Position& comPosition, iDynTree::VectorDynSize& result)
{
    if (!m_configured) {
        yError() << "[NonLinearIKComponent::computeIK] First you have to call the configured method.";
        return false;
    }

    if(!m_prepared){
        if(!prepareIK()){
            yError()<<"[NonLinearIKComponent::computeIK] Error in the preparation phase.";
            return false;
        }
    }

    iDynTree::Transform desiredRightTransform;
    iDynTree::Position desiredCoMPosition;
    iDynTree::Transform baseTransform;
    bool ok = true;

    desiredRightTransform = leftTransform.inverse()*rightTransform;

    if (m_verbose) {
        yInfo() << "[NonLinearIKComponent::computeIK] Desired right Transform: ";
        yInfo() << desiredRightTransform.toString();
    }

    m_ik.updateTarget(m_rFootFrame, desiredRightTransform);

    if(m_additionalFrame.size() != 0){
        m_ik.updateRotationTarget(m_additionalFrame, leftTransform.getRotation().inverse() * m_gravityFrame_R_world.inverse() * m_additionalRotation, m_additionalRotationWeight);
    }

    desiredCoMPosition = leftTransform.inverse() * comPosition;
    if (m_verbose) {
        yInfo() << "[NonLinearIKComponent::computeIK] Input CoM position: ";
        yInfo() << comPosition.toString();
        yInfo() << "[NonLinearIKComponent::computeIK] Desired CoM position: ";
        yInfo() << desiredCoMPosition.toString();
    }

    m_ik.setCOMTarget(desiredCoMPosition, 100.0);

    ok = m_ik.setCurrentRobotConfiguration(m_baseTransform,m_feedback);
    if(!ok){
        yError() << "[NonLinearIKComponent::computeIK] Error while setting the feedback.";
        return false;
    }

    ok = m_ik.setReducedInitialCondition(&m_baseTransform, &m_guess);
    if(!ok){
        yError() << "[NonLinearIKComponent::computeIK] Error while setting the guess.";
        return false;
    }

    ok = m_ik.setDesiredReducedJointConfiguration(m_jointRegularization, m_jointsRegularizationWeights);
    if(!ok){
        yError() << "[NonLinearIKComponent::computeIK] Error while setting the desired joint configuration.";
        return false;
    }

    ok = m_ik.solve();

    if(!ok){
        yError() << "[NonLinearIKComponent::computeIK] Failed in finding a solution.";
        return false;
    }

    m_ik.getReducedSolution(baseTransform, m_qResult);

    m_lchecker.setRobotState(m_baseTransform, m_qResult, m_dummyBaseVel, m_dummyVel,m_dummygrav);

    iDynTree::Position comError = desiredCoMPosition - m_lchecker.getCenterOfMassPosition();
    iDynTree::Position footError = desiredRightTransform.getPosition() - m_lchecker.getRelativeTransform(m_lFootFrame, m_rFootFrame).getPosition();

    if (m_verbose) {
        yInfo() << "[NonLinearIKComponent::computeIK] CoM error position: "<< comError.toString();
        yInfo() << "[NonLinearIKComponent::computeIK] Foot position error: "<<footError.toString();
    }

    result = m_qResult;
    m_guess = m_qResult;

    return true;
}

const std::string NonLinearIKComponent::getLeftFootFrame() const
{
    return m_lFootFrame;
}

const std::string NonLinearIKComponent::getRightFootFrame() const
{
    return m_rFootFrame;
}

bool NonLinearIKComponent::usingAdditionalRotationTarget()
{
    return (m_additionalFrame.size() != 0);
}

const iDynTree::VectorDynSize &NonLinearIKComponent::desiredJointConfiguration() const
{
    return m_jointRegularization;
}

bool NonLinearIKComponent::setAdditionalRotationWeight(double weight)
{
    if (weight < 0){
        yError() << "[NonLinearIKComponent::setAdditionalRotationWeight] The additionalRotationWeight is supposed to be non-negative.";
        return false;
    }
    m_additionalRotationWeight = weight;
    return true;
}

double NonLinearIKComponent::additionalRotationWeight()
{
    return m_additionalRotationWeight;
}

bool NonLinearIKComponent::setDesiredJointsWeights(const iDynTree::VectorDynSize &desiredJointsWeights)
{
    if (!m_configured) {
        yError() << "[NonLinearIKComponent::setDesiredJointsWeights] First you have to call the configured method.";
        return false;
    }

    if (desiredJointsWeights.size() != m_jointRegularization.size()){
        yError() << "[NonLinearIKComponent::setDesiredJointsWeight] The desiredJointsWeights are supposed to have the same dimension of the joint regularizations.";
        return false;
    }
    m_jointsRegularizationWeights = desiredJointsWeights;
    return true;
}

const iDynTree::VectorDynSize& NonLinearIKComponent::desiredJointsWeights()
{
    return m_jointsRegularizationWeights;
}


