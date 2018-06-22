/**
 * @file PIDHandlerComponent.cpp
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#include <PIDHandlerComponent.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IPidControl.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IRemoteVariables.h>
#include <yarp/os/Value.h>
#include <yarp/os/LogStream.h>

#include <sstream>
#include <cmath>

WalkingPIDHandler::WalkingPIDHandler()
    : m_useGainScheduling(false)
    , m_phaseInitTime(0.0)
    , m_previousPhase(PIDPhase::Default)
    , m_currentPIDIndex(-1) //DEFAULT
    , m_desiredPIDIndex(-1)
    , m_firmwareDelay(0.0)
    , m_maximumContactDelay(0.1)
    , m_dT(0.01)
    , m_configured(false)
{
}

WalkingPIDHandler::~WalkingPIDHandler()
{
    {
        std::lock_guard<std::mutex> guard(m_mutex);
        m_useGainScheduling = false;
        m_conditionVariable.notify_one();
    }

    if (m_handlerThread.joinable()){
        m_handlerThread.join();
        m_handlerThread = std::thread();
    }
}

bool WalkingPIDHandler::parsePIDGroup(const yarp::os::Bottle *group, PIDmap& pidMap)
{
    if (!group){
        yError() << "Empty group pointer.";
        return false;
    }

    pidMap.clear();
    for (size_t i = 1; i < group->size(); ++i){
        yarp::os::Value subgroup = group->get(i);
        if (isPIDElement(subgroup)){
            std::string jointName;
            yarp::dev::Pid desiredPID;
            if (!parsePIDElement(subgroup, jointName, desiredPID))
                return false;

            PIDmap::const_iterator original = m_originalPID.find(jointName);

            yarp::dev::Pid safePID;
            if(original != m_originalPID.end()){
                safePID = original->second;
                safePID.setKp(desiredPID.kp);
                safePID.setKi(desiredPID.ki);
                safePID.setKd(desiredPID.kd);
            } else {
                yWarning() << "No joint found with name " << jointName;
                safePID = desiredPID;
            }

            std::pair<PIDmap::iterator, bool> result = pidMap.insert(PIDmap::value_type(jointName, safePID));
            if (!(result.second)) {
                yError() << "Error while inserting item "<< jointName <<" in the map";
                return false;
            }
        }
    }
    return true;
}

bool WalkingPIDHandler::parseDefaultPIDGroup(const yarp::os::Bottle *defaultGroup)
{
    if (!defaultGroup){
        yError() << "Empty default group pointer.";
        return false;
    }

    m_defaultPID = m_originalPID;
    for (size_t i = 1; i < defaultGroup->size(); ++i){
        yarp::os::Value subgroup = defaultGroup->get(i);
        if (isPIDElement(subgroup)){
            std::string jointName;
            yarp::dev::Pid desiredPID;
            if (!parsePIDElement(subgroup, jointName, desiredPID))
                return false;

            PIDmap::iterator result = m_defaultPID.find(jointName);
            if (result != m_originalPID.end()) {
                result->second.setKp(desiredPID.kp);
                result->second.setKi(desiredPID.ki);
                result->second.setKd(desiredPID.kd);
            } else {
                yWarning() << "No joint found with the name " << jointName <<". Skipping the insertion of the default PID.";
            }
        }
    }

    m_defaultPIDsVector.resize(m_axisMap.size());

    for (AxisMap::iterator joint; joint != m_axisMap.end(); ++joint) {
        m_defaultPIDsVector[joint->second] = m_defaultPID[joint->first];
    }

    return true;

}

bool WalkingPIDHandler::parsePIDElement(const yarp::os::Value &groupElement, std::string &jointName, yarp::dev::Pid &pid)
{
    if (groupElement.isList()){
        yarp::os::Bottle* element;
        element = groupElement.asList();
        if ((element->get(0).isString()) && (element->size() == 4)) {
            jointName = element->get(0).toString();
        } else {
            yError() << "Invalid element in the PID file: " << element->toString();
            return false;
        }

        for (size_t g = 1; g < 4; ++g) {
            yarp::os::Value gain = element->get(g);
            if(!(gain.isDouble())&& !(gain.isInt())){
                yError() << "The gains are supposed to be numeric. " << jointName << ": " <<  element->get(g).toString();
                return false;
            }
        }
        pid.setKp(element->get(1).asDouble());
        pid.setKi(element->get(2).asDouble());
        pid.setKd(element->get(3).asDouble());
    } else {
        return false;
    }
    return true;
}

bool WalkingPIDHandler::parsePIDConfigurationFile(const yarp::os::Bottle &PIDSettings)
{
    m_useGainScheduling = PIDSettings.check("useGainScheduling", yarp::os::Value(false)).asBool();
    m_firmwareDelay = PIDSettings.check("firmwareDelay", yarp::os::Value(0.0)).asDouble();
    double smoothingTime = PIDSettings.check("smoothingTime", yarp::os::Value(1.0)).asDouble();

    if (smoothingTime <= 0.0) {
        yError() << "The smoothing time is supposed to be positive.";
        return false;
    }
    m_desiredSmoothingTimes.resize(static_cast<unsigned int>(m_axisMap.size()));

    for (unsigned int j = 0; j < m_desiredSmoothingTimes.size(); ++j) {
        m_desiredSmoothingTimes(j) = smoothingTime;
    }


    if (m_firmwareDelay < 0){
        yError() << "The firmwareDelay field is expected to be non-negative.";
        return false;
    }

    bool defaultFound = false;
    for (size_t g = 1; g < PIDSettings.size(); ++g){
        if (PIDSettings.get(g).isList()){
            yarp::os::Bottle *group = PIDSettings.get(g).asList();
            if (group->get(0).toString() == "DEFAULT"){
                yInfo() << "Parsing DEFAULT PID group.";

                if(!parseDefaultPIDGroup(group))
                    return false;

                defaultFound = true;
            } else if (group->check("activationPhase")){
                std::string name = group->get(0).toString();
                yInfo() << "Parsing " << name << " PID group.";

                yarp::os::Value phaseInput = group->find("activationPhase");
                PIDPhase phase;
                if (!fromStringToPIDPhase(phaseInput.asString(), phase))
                    return false;

                double activationOffset = group->check("activationOffset", yarp::os::Value(0.0)).asDouble();
//                double smoothingTime = group->check("smoothingTime", yarp::os::Value(1.0)).asDouble(); //For the time being we use a common smoothingTime

                if ((phase == PIDPhase::Switch) && (activationOffset < (m_maximumContactDelay - m_firmwareDelay))) {
                    yWarning() << "If the activationOffset of "<< name << " is lower than (maximumContactDelay - firmwareDelay). The group may be triggered too in advance in case of late activation of the contact.";
                }

                PIDmap groupMap;
                if (!parsePIDGroup(group, groupMap))
                    return false;

                m_PIDs.insert(m_PIDs.end(), PIDSchedulingObject(name, phase, activationOffset, groupMap));

                if (!(m_PIDs.back().setPeriod(m_dT))) {
                    yError() << "Failed while setting the period for group " << name << ".";
                    m_PIDs.pop_back();
                    return false;
                }

            }
        }
    }
    if (!defaultFound){
        yError() << "DEFAULT PID group not found.";
        return false;
    }
    return true;
}

bool WalkingPIDHandler::fromStringToPIDPhase(const std::string &input, PIDPhase &output)
{
    if (input == "SWING_LEFT"){
        output = PIDPhase::SwingLeft;
    } else if (input == "SWING_RIGHT"){
        output = PIDPhase::SwingRight;
    } else if (input == "SWITCH"){
        output = PIDPhase::Switch;
    } else {
        yError() << "Unrecognized PID Phase " << input;
        return false;
    }
    return true;
}

bool WalkingPIDHandler::guessPhases(const std::deque<bool> &leftIsFixed, const std::deque<bool> &rightIsFixed)
{
    if (leftIsFixed.size() != rightIsFixed.size()){
        yError() << "Incongruous dimension of the leftIsFixed and rightIsFixed vectors.";
        return false;
    }

    if (m_phases.size() != leftIsFixed.size())
        m_phases.resize(leftIsFixed.size());

    for (size_t instant = 0; instant < leftIsFixed.size(); ++instant){
        if (leftIsFixed[instant] && rightIsFixed[instant]){
            m_phases[instant] = PIDPhase::Switch;
        } else if (leftIsFixed[instant]) {
            m_phases[instant] = PIDPhase::SwingRight;
        } else if (rightIsFixed[instant]) {
            m_phases[instant] = PIDPhase::SwingLeft;
        } else {
            yError() << "Unrecognized phase.";
            return false;
        }
    }
    return true;
}

void WalkingPIDHandler::setPIDThread()
{
    std::string name;
    PIDmap desiredPIDs;

    while (m_useGainScheduling){
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_conditionVariable.wait(lock, [&]{return (((m_desiredPIDIndex != -1) && (m_desiredPIDIndex != m_currentPIDIndex)) || !m_useGainScheduling);});
            if (m_useGainScheduling){

                m_currentPIDIndex = m_desiredPIDIndex;
                desiredPIDs = m_PIDs[static_cast<size_t>(m_desiredPIDIndex)].getDesiredGains();
                name = m_PIDs[static_cast<size_t>(m_desiredPIDIndex)].name();

                yInfo() << "Inserting " << name << " PID group.";
            }
        }
        if (!m_useGainScheduling)
            break;

        if (!setPID(desiredPIDs)){
            yError() << "Unable to set the PIDs for group " << name;
        }

    }
}


bool WalkingPIDHandler::modifyFixedLists(std::deque<bool> &leftIsFixed, std::deque<bool> &rightIsFixed, bool leftIsActuallyFixed, bool rightIsActuallyFixed)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    unsigned int delayInstants = static_cast<unsigned int>(std::round(m_maximumContactDelay/m_dT));

    if ((leftIsFixed.front() == leftIsActuallyFixed) && (rightIsFixed.front() == rightIsActuallyFixed))
        return true;

    if (leftIsFixed.front() && rightIsFixed.front()) { //possible late activation
        if (leftIsActuallyFixed && !rightIsActuallyFixed) {
            size_t i = 0;
            while ((i <= delayInstants) && ((i+1) < leftIsFixed.size()) && (leftIsFixed[i+1])) { //avoid that both feet are not in contact
                rightIsFixed[i] = false;
                ++i;
            }
        } else if (rightIsActuallyFixed && !leftIsActuallyFixed) {
            size_t i = 0;
            while ((i <= delayInstants) && ((i+1) < rightIsFixed.size()) && (rightIsFixed[i+1])) { //avoid that both feet are not in contact
                leftIsFixed[i] = false;
                ++i;
            }
        } else {
            yError() << "Is the robot jumping alredy?";
            return false;
        }
    } else if (leftIsActuallyFixed && rightIsActuallyFixed && (m_previousPhase != PIDPhase::Switch)) { //possible early activation (avoiding late deactivation by checking the previous phase)
        if (leftIsFixed.front() && !rightIsFixed.front()) {
            size_t i = 0;
            while (((i+1) < rightIsFixed.size()) && !(rightIsFixed[i+1])) { //keep rightIsFixed equal to true for at least one instant
                rightIsFixed[i] = true;
                ++i;
            }
        } else if (rightIsFixed.front() && !leftIsFixed.front()) {
            size_t i = 0;
            while (((i+1) < leftIsFixed.size()) && !(leftIsFixed[i+1])) { //keep leftIsFixed equal to true for at least one instant
                leftIsFixed[i] = true;
                ++i;
            }
        } else {
            yError() << "Is the robot jumping alredy?";
            return false;
        }
    }

    return true;
}

bool WalkingPIDHandler::getAxisMap()
{
    m_axisMap.clear();

    std::vector<std::string> allJoints;
    if (!(m_robotComponent->allJointsSources().getJointsName(allJoints))) {
        yError() << "[WalkingPIDHandler::getAxisMap] Failed to retrieve joint list from RobotComponent.";
        return false;
    }

    for (size_t j = 0; j < allJoints.size(); ++j) {
        m_axisMap[allJoints[j]] = j;
    }

    return true;
}

bool WalkingPIDHandler::getPID(PIDmap& output)
{
    std::vector<yarp::dev::Pid> allPIDs;

    if (!(m_robotComponent->allJointsSources().getPositionPIDs(allPIDs))) {
        yError() << "[WalkingPIDHandler::getPID] Failed to retrieve PIDs from RobotComponent.";
        return false;
    }

    output.clear();

    for (AxisMap::iterator joint; joint != m_axisMap.end(); ++joint) {
        output[joint->first] = allPIDs[joint->second];
    }

    return true;
}

bool WalkingPIDHandler::setPID(const PIDmap &pidMap)
{
    if (pidMap.empty()) {
        yInfo("Empty PID list");
        return true;
    }

    if (m_axisMap.empty()) {
        yError("Empty axis map. Cannot setPIDs.");
        return false;
    }

    m_outputPIDs = m_defaultPIDsVector;

    for (PIDmap::const_iterator pid = pidMap.cbegin(); pid != pidMap.cend(); ++pid){
        m_outputPIDs[m_axisMap[pid->first]] = pid->second;
    }

    if (!(m_robotComponent->allJointsSinks().setPositionPIDs(m_outputPIDs))) {
        yError() << "[WalkingPIDHandler::setPID] Failed to set desired PIDs to the RobotComponent object.";
        return false;
    }

    return true;
}


bool WalkingPIDHandler::isPIDElement(const yarp::os::Value &groupElement)
{
    bool yes = false;
    yes = groupElement.isList();
    yes = yes && groupElement.asList()->get(0).isString();
    yes = yes && (groupElement.asList()->get(0).asString() != "activationPhase");
    yes = yes && (groupElement.asList()->get(0).asString() != "activationOffset");
//    yes = yes && (groupElement.asList()->get(0).asString() != "smoothingTime"); //For the time being we use a common smoothingTime
    return yes;
}

bool WalkingPIDHandler::configure(const yarp::os::Bottle &PIDSettings, std::shared_ptr<WalkingControllers::RobotComponent> &robotComponent, double dT)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if (!robotComponent) {
        yError() << "[WalkingPIDHandler::configure] Empty RobotComponent pointer.";
        return false;
    }

    m_robotComponent = robotComponent;

    m_originalPID.clear();
    m_defaultPID.clear();

    if (dT <= 0){
        yError() << "[WalkingPIDHandler::configure] The parameter dT is supposed to be positive.";
        return false;
    }

    m_dT = dT;

    if (!PIDSettings.isNull()) {
        yInfo("[WalkingPIDHandler::configure] Loading custom PIDs");

        if (!getPID(m_originalPID)){
            yError("[WalkingPIDHandler::configure] Error while reading the original PIDs.");
            return false;
        }

        if (!getAxisMap()){
            yError("[WalkingPIDHandler::configure] Failed in populating the axis map.");
            return false;
        }

        if (!parsePIDConfigurationFile(PIDSettings)){
            yError() << "[WalkingPIDHandler::configure] Failed in parsing the PID configuration file.";
            return false;
        }

        if (!(setPID(m_defaultPID))){
            yError("[WalkingPIDHandler::configure] Error while setting the desired PIDs.");
        } else{
            yInfo("[WalkingPIDHandler::configure] DEFAULT PID successfully loaded");
        }

        if (m_useGainScheduling) {
            if (!(m_robotComponent->allJointsSources().getPositionPIDsSmoothingTimes(m_originalSmoothingTimes))) {
              yError() << "[WalkingPIDHandler::configure] Error while retrieving the original smoothing times. Deactivating gain scheduling.";
              m_useGainScheduling = false;
              } else if (!(m_robotComponent->allJointsSinks().setPositionPIDsSmoothingTimes(m_desiredSmoothingTimes))) {
                yError() << "[WalkingPIDHandler::configure] Error while setting the default smoothing time. Deactivating gain scheduling.";
                m_useGainScheduling = false;
            } else {
                m_handlerThread = std::thread(&WalkingPIDHandler::setPIDThread, this);
            }
        }
    }

    m_configured = true;
    return true;
}

bool WalkingPIDHandler::setMaximumContactDelay(double maxContactDelay)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if (maxContactDelay < 0) {
        yError() << " The maxContactDelay parameter is supposed to be non negative.";
        return false;
    }

    m_maximumContactDelay = maxContactDelay;
    return true;
}

bool WalkingPIDHandler::restorePIDs()
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if (!m_configured) {
        yError() << "WalkingPIDHandler::updatePhases] The configure method has not been called yet.";
        return false;
    }

    if (!m_originalPID.empty()) {
        if (!(setPID(m_originalPID))) {
            yError("[WalkingPIDHandler::restorePIDs] Error while restoring the original PIDs.");
            return false;
        }
        m_originalPID.clear();
    }

    if (m_useGainScheduling) {
        if (!(m_robotComponent->allJointsSinks().setPositionPIDsSmoothingTimes(m_originalSmoothingTimes))) {
            yError("[WalkingPIDHandler::restorePIDs] Error while restoring the original smoothing times.");
            return false;
        }
    }

    return true;
}

bool WalkingPIDHandler::usingGainScheduling()
{
    std::lock_guard<std::mutex> guard(m_mutex);

    return m_useGainScheduling;
}

bool WalkingPIDHandler::updatePhases(const std::deque<bool> &leftIsFixed, const std::deque<bool> &rightIsFixed, double time)
{
    std::lock_guard<std::mutex> guard(m_mutex);

    if (!m_configured) {
        yError() << "WalkingPIDHandler::updatePhases] The configure method has not been called yet.";
        return false;
    }

    if (!guessPhases(leftIsFixed, rightIsFixed))
        return false;

    if (m_phases.front() != m_previousPhase){
        m_phaseInitTime = time;
        m_previousPhase = m_phases.front();
    }

    std::vector<size_t> desiredPIDs;
    for (size_t pid = 0; pid < m_PIDs.size(); ++pid){
        if (!m_PIDs[pid].computeInitTime(time, m_phases, m_phaseInitTime))
            return false;

        if (m_PIDs[pid].initTime() <= (time + m_firmwareDelay)){
            desiredPIDs.push_back(pid);
        }
    }

    if (desiredPIDs.size() > 1){
        std::ostringstream message;
        message << "The following PID groups would be activated at the same time: ";
        for (size_t pid = 0; pid < desiredPIDs.size()-1; ++pid)
            message << m_PIDs[desiredPIDs[pid]].name() << ", ";
        message << m_PIDs[desiredPIDs.back()].name() << ".";
        message << "Only the first will be set to the robot.";
        yWarning("%s", message.str().c_str());
    }

    if (desiredPIDs.size() > 0)
        m_desiredPIDIndex = static_cast<int>(desiredPIDs[0]);

    m_conditionVariable.notify_one();

    return true;
}

bool WalkingPIDHandler::updatePhases(const std::deque<bool> &leftIsFixed, const std::deque<bool> &rightIsFixed, bool leftIsActuallyFixed, bool rightIsActuallyFixed, double time)
{
    {
        std::lock_guard<std::mutex> guard(m_mutex);

        if ((leftIsFixed.size() == 0) || (rightIsFixed.size() == 0)) {
            yError() << "Empty list of contacts.";
            return false;
        }
        if (leftIsFixed.size() != rightIsFixed.size()){
            yError() << "Incongruous dimension of the leftIsFixed and rightIsFixed vectors.";
            return false;
        }
        m_leftIsFixedModified = leftIsFixed;
        m_rightIsFixedModified = rightIsFixed;
    }

    if (!modifyFixedLists(m_leftIsFixedModified, m_rightIsFixedModified, leftIsActuallyFixed, rightIsActuallyFixed)){
        yError() << "Error while modying the list of contacts.";
        return false;
    }

    return updatePhases(m_leftIsFixedModified, m_rightIsFixedModified, time);
}

bool WalkingPIDHandler::reset()
{

    if (!m_configured) {
        yError() << "WalkingPIDHandler::reset] The configure method has not been called yet.";
        return false;
    }

    if (m_useGainScheduling && (m_desiredPIDIndex != -1)) {
        if (!(setPID(m_defaultPID))) {
            yError("Error while setting the original PIDs during reset.");
            return false;
        }
    }

    m_phaseInitTime = 0.0;
    m_previousPhase = PIDPhase::Default;
    m_currentPIDIndex = -1; //DEFAULT
    m_desiredPIDIndex = -1;

    return true;
}

PIDSchedulingObject::PIDSchedulingObject(const std::string &name, const PIDPhase &activationPhase, double activationOffset, const PIDmap &desiredPIDs)
    :m_name(name)
    ,m_desiredPIDs(desiredPIDs)
    ,m_activationPhase(activationPhase)
    ,m_activationOffset(activationOffset)
    ,m_computedInitTime(0.0)
    ,m_dT(0.01)
{

}

bool PIDSchedulingObject::setPeriod(double dT)
{
    if (m_dT <= 0){
        yError() << "The dT is supposed to be positive";
        return false;
    }
    m_dT = dT;
    return true;
}

const PIDmap& PIDSchedulingObject::getDesiredGains()
{
    return m_desiredPIDs;
}

bool PIDSchedulingObject::computeInitTime(double time, const std::vector<PIDPhase> &phases, double currentPhaseInitTime)
{
    if (currentPhaseInitTime > time){
        yError() << "The initial time of the current phase cannot be greater than the current time.";
        return false;
    }

    if (phases.size() == 0){
        yError() << "Empty phase vector.";
        return false;
    }

    if (phases.front() == m_activationPhase){
        m_computedInitTime = currentPhaseInitTime + m_activationOffset;
        return true;
    }

    size_t k = 0;
    while ((k < phases.size()) && (phases[k] != m_activationPhase)){
        ++k;
    }
    m_computedInitTime = time + k*m_dT + m_activationOffset;
    return true;
}

double PIDSchedulingObject::initTime()
{
    return m_computedInitTime;
}

std::string PIDSchedulingObject::name()
{
    return m_name;
}

