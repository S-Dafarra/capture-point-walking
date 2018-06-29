/**
 * @file UnicycleGeneratorComponent.h
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#include <UnicycleGeneratorComponent.h>

#include <yarp/os/LogStream.h>

#include "iDynTree/Core/Utils.h"
#include <Utils.h>
#include <chrono>

using namespace WalkingControllers;

bool UnicycleGeneratorComponent::readPlannerConfigurationFile(yarp::os::Searchable &config)
{
    if (config.isNull()){
        yError() << "[UnicycleGeneratorComponent::readPlannerConfigurationFile] Empty configuration for unicycle planner.";
        return false;
    }

    yarp::os::Bottle generalSection = config.findGroup("GENERAL");

    if (generalSection.isNull()) {
        yError() << "[UnicycleGeneratorComponent::readPlannerConfigurationFile] The [GENERAL] section is necessary.";
        return false;
    }

    yarp::os::Value tempValue;
    bool ok = true;

    ok = ok && YarpHelper::getDoubleFromSearchable(generalSection, "period", m_dT);
    ok = ok && YarpHelper::getDoubleFromSearchable(generalSection, "plannerHorizon", m_plannerHorizon);
    ok = ok && YarpHelper::getDoubleFromSearchable(generalSection, "unicycleGain", m_unicycleGain);
    tempValue = config.find("referencePosition");
    if (!YarpHelper::yarpListToiDynTreeVectorFixSize(tempValue, m_referencePointDistance)){
        yError() << "[UnicycleGeneratorComponent::readPlannerConfigurationFile] Initialization failed while reading referencePosition vector.";
        return false;
    }
    ok = ok && YarpHelper::getDoubleFromSearchable(generalSection, "timeWeight", m_timeWeight);
    ok = ok && YarpHelper::getDoubleFromSearchable(generalSection, "positionWeight", m_positionWeight);
    ok = ok && YarpHelper::getDoubleFromSearchable(generalSection, "slowWhenTurningGain", m_slowWhenTurningGain);
    ok = ok && YarpHelper::getDoubleFromSearchable(generalSection, "maxStepLength",  m_maxStepLength);
    ok = ok && YarpHelper::getDoubleFromSearchable(generalSection, "minStepLength", m_minStepLength);
    ok = ok && YarpHelper::getDoubleFromSearchable(generalSection, "nominalWidth", m_nominalWidth);
    ok = ok && YarpHelper::getDoubleFromSearchable(generalSection, "minWidth", m_minWidth);
    ok = ok && YarpHelper::getDoubleFromSearchable(generalSection, "maxAngleVariation", m_maxAngleVariation);
    ok = ok && YarpHelper::getDoubleFromSearchable(generalSection, "minAngleVariation", m_minAngleVariation);
    ok = ok && YarpHelper::getDoubleFromSearchable(generalSection, "maxStepDuration", m_maxStepDuration);
    ok = ok && YarpHelper::getDoubleFromSearchable(generalSection, "minStepDuration", m_minStepDuration);
    ok = ok && YarpHelper::getDoubleFromSearchable(generalSection, "nominalDuration", m_nominalDuration);
    ok = ok && YarpHelper::getDoubleFromSearchable(generalSection, "lastStepSwitchTime", m_lastStepSwitchTime);
    ok = ok && YarpHelper::getDoubleFromSearchable(generalSection, "switchOverSwingRatio", m_switchOverSwingRatio);
    ok = ok && YarpHelper::getBoolFromSearchable(generalSection, "swingLeft", m_swingLeft);
    ok = ok && YarpHelper::getBoolFromSearchable(generalSection, "startAlwaysSameFoot", m_forceStart);

    if (!ok) {
        return false;
    }

    yarp::os::Bottle feetSection = config.findGroup("FEET_GENERATION");

    if (feetSection.isNull()) {
        m_useFeetGeneration = false;
    } else {
        m_useFeetGeneration = true;
    }

    if (m_useFeetGeneration) {
        ok = ok && YarpHelper::getDoubleFromSearchable(feetSection, "stepHeight", m_stepHeight);
        ok = ok && YarpHelper::getDoubleFromSearchable(feetSection, "stepLandingVelocity", m_landingVelocity);
        ok = ok && YarpHelper::getDoubleFromSearchable(feetSection, "footApexTime", m_apexTime);

        if (!ok){
            return false;
        }
    }

    yarp::os::Bottle heightSection = config.findGroup("COM_HEIGHT_GENERATION");

    if (heightSection.isNull()) {
        m_useHeightGeneration = false;
    } else {
        m_useHeightGeneration = true;
    }

    if (m_useHeightGeneration) {
        ok = ok && YarpHelper::getDoubleFromSearchable(heightSection, "comHeight", m_comHeight);
        ok = ok && YarpHelper::getDoubleFromSearchable(heightSection, "comHeightDelta", m_comHeightDelta);

        if (!ok){
            return false;
        }
    }

    yarp::os::Bottle zmpSection = config.findGroup("ZMP_GENERATION");

    if (zmpSection.isNull()) {
        m_useZMPGeneration = false;
    } else {
        m_useZMPGeneration = true;
    }

    if (m_useZMPGeneration) {
        tempValue = zmpSection.find("rStancePosition");
        if (!YarpHelper::yarpListToiDynTreeVectorFixSize(tempValue, m_rZMPStancePosition)){
            yError() << "[UnicycleGeneratorComponent::readPlannerConfigurationFile] Initialization failed while reading rStancePosition vector.";
            return false;
        }

        tempValue = zmpSection.find("rSwitchInitPosition");
        if (!YarpHelper::yarpListToiDynTreeVectorFixSize(tempValue, m_rZMPInitialSwitchPosition)){
            yError() << "[UnicycleGeneratorComponent::readPlannerConfigurationFile] Initialization failed while reading rSwitchInitPosition vector.";
            return false;
        }

        tempValue = zmpSection.find("lStancePosition");
        if (!YarpHelper::yarpListToiDynTreeVectorFixSize(tempValue, m_lZMPStancePosition)){
            yError() << "[UnicycleGeneratorComponent::readPlannerConfigurationFile] Initialization failed while reading lStancePosition vector.";
            return false;
        }

        tempValue = zmpSection.find("lSwitchInitPosition");
        if (!YarpHelper::yarpListToiDynTreeVectorFixSize(tempValue, m_lZMPInitialSwitchPosition)){
            yError() << "[UnicycleGeneratorComponent::readPlannerConfigurationFile] Initialization failed while reading lSwitchInitPosition vector.";
            return false;
        }
    }

    return true;
}

bool UnicycleGeneratorComponent::configurePlanner()
{
    bool ok = true;

    ok = ok && m_generator.unicyclePlanner()->setDesiredPersonDistance(m_referencePointDistance(0), m_referencePointDistance(1));
    ok = ok && m_generator.unicyclePlanner()->setControllerGain(m_unicycleGain);
    ok = ok && m_generator.unicyclePlanner()->setMaximumIntegratorStepSize(m_dT);
    ok = ok && m_generator.unicyclePlanner()->setMaxStepLength(m_maxStepLength);
    ok = ok && m_generator.unicyclePlanner()->setWidthSetting(m_minWidth, m_nominalWidth);
    ok = ok && m_generator.unicyclePlanner()->setMaxAngleVariation(m_maxAngleVariation);
    ok = ok && m_generator.unicyclePlanner()->setCostWeights(m_positionWeight, m_timeWeight);
    ok = ok && m_generator.unicyclePlanner()->setStepTimings(m_minStepDuration, m_maxStepDuration, m_nominalDuration);
    ok = ok && m_generator.unicyclePlanner()->setPlannerPeriod(m_dT);
    ok = ok && m_generator.unicyclePlanner()->setMinimumAngleForNewSteps(m_minAngleVariation);
    ok = ok && m_generator.unicyclePlanner()->setMinimumStepLength(m_minStepLength);
    ok = ok && m_generator.unicyclePlanner()->setSlowWhenTurnGain(m_slowWhenTurningGain);
    m_generator.unicyclePlanner()->addTerminalStep(true);
    m_generator.unicyclePlanner()->startWithLeft(m_swingLeft);
    m_generator.unicyclePlanner()->resetStartingFootIfStill(m_forceStart);

    ok = ok && m_generator.setSwitchOverSwingRatio(m_switchOverSwingRatio);
    ok = ok && m_generator.setTerminalHalfSwitchTime(m_lastStepSwitchTime);
    ok = ok && m_generator.setPauseConditions(m_maxStepDuration, m_nominalDuration);

    if (m_useFeetGeneration) {
        m_feetGenerator = m_generator.addFeetCubicSplineGenerator();
        ok = ok && m_feetGenerator->setStepHeight(m_stepHeight);
        ok = ok && m_feetGenerator->setFootLandingVelocity(m_landingVelocity);
        ok = ok && m_feetGenerator->setFootApexTime(m_apexTime);
    }

    if (m_useHeightGeneration) {
        m_heightGenerator = m_generator.addCoMHeightTrajectoryGenerator();
        ok = ok && m_heightGenerator->setCoMHeightSettings(m_comHeight, m_comHeightDelta);
    }

    if (m_useZMPGeneration) {
        m_zmpGenerator = m_generator.addZMPTrajectoryGenerator();
        ok = ok && m_zmpGenerator->setStanceZMPDelta(m_lZMPStancePosition, m_rZMPStancePosition);
        ok = ok && m_zmpGenerator->setInitialSwitchZMPDelta(m_lZMPInitialSwitchPosition, m_rZMPInitialSwitchPosition);
    }

    if (ok){
        m_generatorState = GeneratorState::Configured;
        m_generatorThread = std::thread(&UnicycleGeneratorComponent::computeThread, this);
    }

    return ok;
}

void UnicycleGeneratorComponent::computeThread()
{
    while (true) {
        bool start = false;
        double initTime;
        double endTime;
        double dT;
        iDynTree::Vector2 desiredPoint;
        bool correctLeft, correctRight;
        iDynTree::Vector2 measuredPositionLeft, measuredPositionRight;
        double measuredAngleLeft, measuredAngleRight;
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_conditionVariable.wait(lock, [&]{return ((m_generatorState == GeneratorState::Called) ||
                                                       (m_generatorState == GeneratorState::Closing));});
//             yInfo() << "After updating trajectories, desired point is: " << m_desiredPoint.toString();

            if (m_generatorState == GeneratorState::Closing){
                break;
            }

            start = true;
            dT = m_dT;
            initTime = m_initTime;
            endTime = initTime + m_plannerHorizon;
            desiredPoint = m_desiredPoint;
            correctLeft = m_correctLeft;
            correctRight = m_correctRight;
            measuredPositionLeft(0) = m_measuredTransformLeft.getPosition()(0);
            measuredPositionLeft(1) = m_measuredTransformLeft.getPosition()(1);
            measuredPositionRight(0) = m_measuredTransformRight.getPosition()(0);
            measuredPositionRight(1) = m_measuredTransformRight.getPosition()(1);
            measuredAngleLeft = m_measuredTransformLeft.getRotation().asRPY()(2);
            measuredAngleRight = m_measuredTransformRight.getRotation().asRPY()(2);
        }

        m_generator.unicyclePlanner()->clearDesiredTrajectory();

        if (!m_generator.unicyclePlanner()->addDesiredTrajectoryPoint(endTime, desiredPoint)){
            std::lock_guard<std::mutex> guard(m_mutex);
            m_generatorState = GeneratorState::Configured;
            yError() << "Error while setting the new reference.";
            start = false;
            break;
        }

        if (start){
            if (!correctLeft || !correctRight){
                iDynTree::Vector2 measuredPosition;
                double measuredAngle;
                measuredPosition = correctLeft ? measuredPositionLeft : measuredPositionRight;
                measuredAngle = correctLeft ? measuredAngleLeft : measuredAngleRight;

                if (m_generator.reGenerate(initTime, dT, endTime, correctLeft, measuredPosition, measuredAngle)){
                    std::lock_guard<std::mutex> guard(m_mutex);
                    m_generatorState = GeneratorState::Returned;
                } else {

                    std::lock_guard<std::mutex> guard(m_mutex);
                    m_generatorState = GeneratorState::Configured;
                    yError() << "Failed in computing new trajectory.";

                }
            } else {

                if (m_generator.reGenerate(initTime, dT, endTime, measuredPositionLeft, measuredAngleLeft, measuredPositionRight, measuredAngleRight)){
                    std::lock_guard<std::mutex> guard(m_mutex);
                    m_generatorState = GeneratorState::Returned;
                } else {

                    std::lock_guard<std::mutex> guard(m_mutex);
                    m_generatorState = GeneratorState::Configured;
                    yError() << "Failed in computing new trajectory.";

                }
            }
        }
    }
}

bool UnicycleGeneratorComponent::privateUpdateTrajectories(double initTime, bool correctLeft, bool correctRight, bool newStanceFootIsLeft, const iDynTree::Transform& measuredLeft, const iDynTree::Transform& measuredRight)
{

    iDynTree::Vector2 unicyclePosition; //with respect the measured step
    unicyclePosition(0) = 0.0;
    unicyclePosition(1) = newStanceFootIsLeft ? -m_nominalWidth/2 : m_nominalWidth/2;
    iDynTree::Vector2 newLocalReferencePosition, newReferencePosition;

    newLocalReferencePosition(0) = unicyclePosition(0) + m_referencePointDistance(0);
    newLocalReferencePosition(1) = unicyclePosition(1) + m_referencePointDistance(1);

    double theta = newStanceFootIsLeft ? measuredLeft.getRotation().asRPY()(2) : measuredRight.getRotation().asRPY()(2);
    double s_theta = std::sin(theta);
    double c_theta = std::cos(theta);

    iDynTree::Position stancePosition = newStanceFootIsLeft ? measuredLeft.getPosition() : measuredRight.getPosition();

    newReferencePosition(0) = c_theta * newLocalReferencePosition(0) - s_theta * newLocalReferencePosition(1) + stancePosition(0);
    newReferencePosition(1) = s_theta * newLocalReferencePosition(0) + c_theta * newLocalReferencePosition(1) + stancePosition(1);



    if (!m_inputReference->getNewReference(newReferencePosition, theta, m_plannerHorizon, m_desiredPoint)) {
        yError() << "[UnicycleGeneratorComponent::updateTrajectories] Failed to get a new input reference.";
        return false;
    }

    m_initTime = initTime;
    m_correctLeft = correctLeft;
    m_correctRight = correctRight;
    m_measuredTransformLeft = measuredLeft;
    m_measuredTransformRight = measuredRight;
    m_generatorState = GeneratorState::Called;
//     yInfo() << ">> GeneratorState::Called with initTime " << m_initTime;
//    yInfo() << "After updating trajectories, desired point is: " << m_desiredPoint.toString();
    m_conditionVariable.notify_one();

    return true;
}

UnicycleGeneratorComponent::UnicycleGeneratorComponent()
    : m_initTime(0.0)
    , m_generatorState(GeneratorState::NotConfigured)
{
    m_desiredPoint.zero();
}

UnicycleGeneratorComponent::~UnicycleGeneratorComponent()
{
    {
        std::lock_guard<std::mutex> guard(m_mutex);
        m_generatorState = GeneratorState::Closing;
        m_conditionVariable.notify_one();
    }

    if (m_generatorThread.joinable()) {
        m_generatorThread.join();
        //Empty thread now
        m_generatorThread = std::thread();
    }
}

bool UnicycleGeneratorComponent::configure(yarp::os::Searchable &config, std::shared_ptr<UnicycleReferenceSource> referenceSource)
{
    if (m_generatorState == GeneratorState::Configured) {
        yError() << "[UnicycleGeneratorComponent::configure] Cannot configure twice.";
        return false;
    }

    if (!referenceSource) {
        yError() << "[UnicycleGeneratorComponent::configure] Empty referenceSource pointer.";
        return false;
    }

    m_inputReference = referenceSource;

    if (!readPlannerConfigurationFile(config)){
        yError() << "[UnicycleGeneratorComponent::configure] Failed while reading the unicycle planner configuration file.";
        return false;
    }
    yInfo("[UnicycleGeneratorComponent::configure] Planner configuration files read successfully!");

    if (!configurePlanner()){
        yError() << "[UnicycleGeneratorComponent::configure] Failed to configure the unicycle trajectory generator.";
        return false;
    }

    return true;
}

bool UnicycleGeneratorComponent::generateFirstTrajectories()
{

    std::lock_guard<std::mutex> guard(m_mutex);

    if (m_generatorState == GeneratorState::NotConfigured){
        yError() <<"[UnicycleGeneratorComponent::generateFirstTrajectories] The trajectory generator has not been configured yet.";
        return false;
    }

    if (m_generatorState == GeneratorState::Called){
        yError() <<"[UnicycleGeneratorComponent::generateFirstTrajectories] Cannot launch the generator twice.";
        return false;
    }

    //reset generator
    m_generator.unicyclePlanner()->clearDesiredTrajectory();

    if (!m_generator.unicyclePlanner()->addDesiredTrajectoryPoint(0.0, m_referencePointDistance)){
        yError() << "[UnicycleGeneratorComponent::generateFirstTrajectories] Error while setting the first reference.";
        return false;
    }

    if (!m_inputReference->getNewReference(m_referencePointDistance, 0.0, m_plannerHorizon, m_desiredPoint)) {
        yError() << "[UnicycleGeneratorComponent::generateFirstTrajectories] Failed to get a new input reference.";
        return false;
    }

    if (!m_generator.unicyclePlanner()->addDesiredTrajectoryPoint(m_plannerHorizon, m_desiredPoint)){
        yError() << "[UnicycleGeneratorComponent::generateFirstTrajectories] Error while setting the new reference.";
        return false;
    }

    if (!m_generator.generate(0.0, m_dT, m_plannerHorizon)){
        yError() << "[UnicycleGeneratorComponent::generateFirstTrajectories] Error while computing the first trajectories.";
        return false;
    }

    m_generatorState = GeneratorState::Returned;

    return true;
}

bool UnicycleGeneratorComponent::generateFirstTrajectories(const iDynTree::Transform &leftToRightTransform, const iDynTree::Position &initialCOMPositionInLeft)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if (m_generatorState == GeneratorState::NotConfigured){
        yError() <<"[UnicycleGeneratorComponent::generateFirstTrajectories] The trajectory generator has not been configured yet.";
        return false;
    }

    if (m_generatorState == GeneratorState::Called){
        yError() <<"[UnicycleGeneratorComponent::generateFirstTrajectories] Cannot launch the generator twice.";
        return false;
    }

    //reset generator
    m_generator.unicyclePlanner()->clearDesiredTrajectory();

    if (!m_inputReference->getNewReference(m_referencePointDistance, 0.0, m_plannerHorizon, m_desiredPoint)) {
        yError() << "[UnicycleGeneratorComponent::generateFirstTrajectories] Failed to get a new input reference.";
        return false;
    }

    if (!m_generator.unicyclePlanner()->addDesiredTrajectoryPoint(m_plannerHorizon, m_desiredPoint)){
        yError() << "[UnicycleGeneratorComponent::generateFirstTrajectories] Error while setting the new reference.";
        return false;
    }

    std::shared_ptr<FootPrint> left, right;

    left = m_generator.getLeftFootPrint();
    right = m_generator.getRightFootPrint();

    iDynTree::Vector2 leftPosition, rightPosition;
    double leftAngle, rightAngle;

    if (m_swingLeft){ // The standing foot is the right -> the unicycle starts parallel to the right foot

        rightPosition(0) = 0.0;
        rightPosition(1) = -leftToRightTransform.inverse().getPosition()(1)/2;
        rightAngle = 0;
        right->addStep(rightPosition, rightAngle, 0.0);

        leftPosition(0) = leftToRightTransform.inverse().getPosition()(0);
        leftPosition(1) = leftToRightTransform.inverse().getPosition()(1)/2;
        leftAngle = leftToRightTransform.inverse().getRotation().asRPY()(2);
        left->addStep(leftPosition, leftAngle, 0.0);

    } else {

        leftPosition(0) = 0.0;
        leftPosition(1) = -leftToRightTransform.getPosition()(1)/2;
        leftAngle = 0;
        left->addStep(leftPosition, leftAngle, 0.0);

        rightPosition(0) = leftToRightTransform.getPosition()(0);
        rightPosition(1) = leftToRightTransform.getPosition()(1)/2;
        rightAngle = leftToRightTransform.getRotation().asRPY()(2);
        right->addStep(rightPosition, rightAngle, 0.0);

    }

    InitialState initWeight;
    initWeight.initialVelocity = 0;
    initWeight.initialAcceleration = 0;
    initWeight.initialPosition = (leftToRightTransform.getPosition()(1) - initialCOMPositionInLeft(1))/leftToRightTransform.getPosition()(1);

    if (initWeight.initialPosition < 0){
        yWarning() << "[UnicycleGeneratorComponent::generateFirstTrajectories] The CoM seems to be on the edge of the left foot. This may cause problems.";
        initWeight.initialPosition = 0;
    }

    if (initWeight.initialPosition > 1){
        yWarning() << "[UnicycleGeneratorComponent::generateFirstTrajectories] The CoM seems to be on the edge of the right foot. This may cause problems.";
        initWeight.initialPosition = 1;
    }

    if (m_useZMPGeneration && (!m_zmpGenerator->setWeightInitialState(initWeight))) {
        yError() << "[UnicycleGeneratorComponent::generateFirstTrajectories] Failed in setting intialWeightState.";
        return false;
    }

    if (!m_generator.generate(0.0, m_dT, m_plannerHorizon)){
        yError() << "[UnicycleGeneratorComponent::generateFirstTrajectories] Error while computing the first trajectories.";
        return false;
    }

    m_generatorState = GeneratorState::Returned;

    return true;

}

bool UnicycleGeneratorComponent::updateTrajectories(double initTime, const InitialState& initialWeightInLeftFoot, bool correctLeft,
                                                         const iDynTree::Transform &measured)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if (m_generatorState == GeneratorState::Called){
        yError() <<"[UnicycleGeneratorComponent::updateTrajectories] Cannot launch the generator twice.";
        return false;
    }

    if (m_generatorState != GeneratorState::Returned){
        yError() <<"[UnicycleGeneratorComponent::updateTrajectories] The trajectory generator has not computed any trajectory yet. Be sure that the trajectory generator is correctly configured and that the method generateFirstTrajectories has been called once.";
        return false;
    }

    if (!m_useZMPGeneration) {
        yWarning() << "[UnicycleGeneratorComponent::updateTrajectories] The initialWeightInLeftFoot is ignored.";
    } else {
        if (!m_zmpGenerator->setWeightInitialState(initialWeightInLeftFoot)) {
            m_generatorState = GeneratorState::Configured;
            yError() << "[UnicycleGeneratorComponent::updateTrajectories] Failed in setting intialWeightState.";
            return false;
        }
    }

    return privateUpdateTrajectories(initTime, correctLeft, !correctLeft, correctLeft, measured, measured); //when correctLeft and correctRight are different, only one measured transform is used.
}

bool UnicycleGeneratorComponent::updateTrajectories(double initTime, bool correctLeft, const iDynTree::Transform &measured)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if (m_generatorState == GeneratorState::Called){
        yError() <<"[UnicycleGeneratorComponent::updateTrajectories] Cannot launch the generator twice.";
        return false;
    }

    if (m_generatorState != GeneratorState::Returned){
        yError() <<"[UnicycleGeneratorComponent::updateTrajectories] The trajectory generator has not computed any trajectory yet. Be sure that the trajectory generator is correctly configured and that the method generateFirstTrajectories has been called once.";
        return false;
    }

    if (m_useZMPGeneration) {
        yWarning() << "[UnicycleGeneratorComponent::updateTrajectories] The initialWeightInLeftFoot is not set.";
    }

    return privateUpdateTrajectories(initTime, correctLeft, !correctLeft, correctLeft, measured, measured); //when correctLeft and correctRight are different, only one measured transform is used.
}

bool UnicycleGeneratorComponent::updateTrajectories(double initTime, const InitialState &initialWeightInLeftFoot, bool newStanceFootIsLeft, const iDynTree::Transform &measuredLeft, const iDynTree::Transform &measuredRight)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if (m_generatorState == GeneratorState::Called){
        yError() <<"[UnicycleGeneratorComponent::updateTrajectories] Cannot launch the generator twice.";
        return false;
    }

    if (m_generatorState != GeneratorState::Returned){
        yError() <<"[UnicycleGeneratorComponent::updateTrajectories] The trajectory generator has not computed any trajectory yet. Be sure that the trajectory generator is correctly configured and that the method generateFirstTrajectories has been called once.";
        return false;
    }

    if (!m_useZMPGeneration) {
        yWarning() << "[UnicycleGeneratorComponent::updateTrajectories] The initialWeightInLeftFoot is ignored.";
    } else {
        if (!m_zmpGenerator->setWeightInitialState(initialWeightInLeftFoot)) {
            m_generatorState = GeneratorState::Configured;
            yError() << "[UnicycleGeneratorComponent::updateTrajectories] Failed in setting intialWeightState.";
            return false;
        }
    }

    return privateUpdateTrajectories(initTime, true, true, newStanceFootIsLeft, measuredLeft, measuredRight);
}

bool UnicycleGeneratorComponent::updateTrajectories(double initTime, bool newStanceFootIsLeft, const iDynTree::Transform &measuredLeft, const iDynTree::Transform &measuredRight)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if (m_generatorState == GeneratorState::Called){
        yError() <<"[UnicycleGeneratorComponent::updateTrajectories] Cannot launch the generator twice.";
        return false;
    }

    if (m_generatorState != GeneratorState::Returned){
        yError() <<"[UnicycleGeneratorComponent::updateTrajectories] The trajectory generator has not computed any trajectory yet. Be sure that the trajectory generator is correctly configured and that the method generateFirstTrajectories has been called once.";
        return false;
    }

    if (m_useZMPGeneration) {
        yWarning() << "[UnicycleGeneratorComponent::updateTrajectories] The initialWeightInLeftFoot is not set.";
    }

    return privateUpdateTrajectories(initTime, true, true, newStanceFootIsLeft, measuredLeft, measuredRight);
}

bool UnicycleGeneratorComponent::trajectoryComputed() const
{
    return (m_generatorState == GeneratorState::Returned);
}

bool UnicycleGeneratorComponent::getFeetTrajectories(std::vector<iDynTree::Transform> &lFootTrajectory, std::vector<iDynTree::Transform> &rFootTrajectory) const
{
    if (!trajectoryComputed()){
        yError() << "[UnicycleGeneratorComponent::getFeetTrajectories] No trajectories are available.";
        return false;
    }

    if (!m_useFeetGeneration) {
        yError() << "[UnicycleGeneratorComponent::getFeetTrajectories] Feet generation is not active.";
        return false;
    }

    m_feetGenerator->getFeetTrajectories(lFootTrajectory, rFootTrajectory);

    return true;
}

bool UnicycleGeneratorComponent::getWeightPercentage(std::vector<double> &weightInLeft, std::vector<double> &weightInRight) const
{
    if (!trajectoryComputed()){
        yError() << "[UnicycleGeneratorComponent::getWeightPercentage] No trajectories are available.";
        return false;
    }

    if (!m_useZMPGeneration) {
        yError() << "[UnicycleGeneratorComponent::getWeightPercentage] ZMP generation is not active.";
        return false;
    }

    m_zmpGenerator->getWeightPercentage(weightInLeft, weightInRight);

    return true;
}

bool UnicycleGeneratorComponent::getZMPTrajectory(std::vector<iDynTree::Vector2> &ZMPTrajectory) const
{
    if (!trajectoryComputed()){
        yError() << "[UnicycleGeneratorComponent::getZMPTrajectory] No trajectories are available.";
        return false;
    }

    if (!m_useZMPGeneration) {
        yError() << "[UnicycleGeneratorComponent::getZMPTrajectory] ZMP generation is not active.";
        return false;
    }

   m_zmpGenerator->getZMPTrajectory(ZMPTrajectory);

    return true;
}

bool UnicycleGeneratorComponent::getZMPTrajectory(std::vector<iDynTree::Vector2> &ZMPTrajectory, std::vector<iDynTree::Vector2> &ZMPVelocity, std::vector<iDynTree::Vector2> &ZMPAcceleration) const
{
    if (!trajectoryComputed()){
        yError() << "[UnicycleGeneratorComponent::getZMPTrajectory] No trajectories are available.";
        return false;
    }

    if (!m_useZMPGeneration) {
        yError() << "[UnicycleGeneratorComponent::getZMPTrajectory] ZMP generation is not active.";
        return false;
    }

    m_zmpGenerator->getZMPTrajectory(ZMPTrajectory, ZMPVelocity, ZMPAcceleration);

    return true;
}

bool UnicycleGeneratorComponent::getLocalZMPTrajectories(std::vector<iDynTree::Vector2> &leftZMPTrajectory,
                                                              std::vector<iDynTree::Vector2> &rightZMPTrajectory) const
{
    if (!trajectoryComputed()){
        yError() << "[UnicycleGeneratorComponent::getLocalZMPTrajectories] No trajectories are available.";
        return false;
    }

    if (!m_useZMPGeneration) {
        yError() << "[UnicycleGeneratorComponent::getLocalZMPTrajectories] ZMP generation is not active.";
        return false;
    }

    m_zmpGenerator->getLocalZMPTrajectories(leftZMPTrajectory, rightZMPTrajectory);

    return true;
}

bool UnicycleGeneratorComponent::getCoMHeightTrajectory(std::vector<double> &CoMHeightTrajectory) const
{
    if (!trajectoryComputed()){
        yError() << "[UnicycleGeneratorComponent::getCoMHeightTrajectory] No trajectories are available.";
        return false;
    }

    if (!m_useHeightGeneration) {
        yError() << "[UnicycleGeneratorComponent::getCoMHeightTrajectory] Height generation is not active";
    }

    m_heightGenerator->getCoMHeightTrajectory(CoMHeightTrajectory);

    return true;
}

bool UnicycleGeneratorComponent::getCoMHeightVelocity(std::vector<double> &CoMHeightVelocity) const
{
    if (!trajectoryComputed()){
        yError() << "[UnicycleGeneratorComponent::getCoMHeightVelocity] No trajectories are available.";
        return false;
    }

    if (!m_useHeightGeneration) {
        yError() << "[UnicycleGeneratorComponent::getCoMHeightVelocity] Height generation is not active";
    }

    m_heightGenerator->getCoMHeightVelocity(CoMHeightVelocity);

    return true;
}

bool UnicycleGeneratorComponent::getCoMHeightAccelerationProfile(std::vector<double> &CoMHeightAccelerationProfile) const
{
    if (!trajectoryComputed()){
        yError() << "[UnicycleGeneratorComponent::getCoMHeightAccelerationProfile] No trajectories are available.";
        return false;
    }

    if (!m_useHeightGeneration) {
        yError() << "[UnicycleGeneratorComponent::getCoMHeightAccelerationProfile] Height generation is not active";
    }

    m_heightGenerator->getCoMHeightAccelerationProfile(CoMHeightAccelerationProfile);

    return true;
}

bool UnicycleGeneratorComponent::getFeetStandingPeriods(std::vector<bool> &lFootContacts, std::vector<bool> &rFootContacts) const
{
    if (!trajectoryComputed()){
        yError() << "[UnicycleGeneratorComponent::getFeetStandingPeriods] No trajectories are available";
        return false;
    }

    m_generator.getFeetStandingPeriods(lFootContacts, rFootContacts);

    return true;
}

bool UnicycleGeneratorComponent::getWhenUseLeftAsFixed(std::vector<bool> &leftIsFixed) const
{
    if (!trajectoryComputed()){
        yError() << "[UnicycleGeneratorComponent::getWhenUseLeftAsFixed] No trajectories are available";
        return false;
    }

    m_generator.getWhenUseLeftAsFixed(leftIsFixed);

    return true;
}

bool UnicycleGeneratorComponent::getInitialStatesAtMergePoints(std::vector<InitialState> &initialStates) const
{
    if (!trajectoryComputed()){
        yError() << "[UnicycleGeneratorComponent::getInitialStatesAtMergePoints] No trajectories are available.";
        return false;
    }

    if (!m_useZMPGeneration) {
        yError() << "[UnicycleGeneratorComponent::getInitialStatesAtMergePoints] ZMP generation is not active.";
        return false;
    }

    m_zmpGenerator->getInitialStatesAtMergePoints(initialStates);

    return true;
}

bool UnicycleGeneratorComponent::getMergePoints(std::vector<size_t> &mergePoints) const
{
    if (!trajectoryComputed()){
        yError() << "[UnicycleGeneratorComponent::getMergePoints] No trajectories are available";
        return false;
    }

    m_generator.getMergePoints(mergePoints);

    return true;
}
