/**
 * @file RobotComponent.cpp
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#include <RobotComponent.h>
#include <JointsSources.h>
#include <JointsSinks.h>
#include <Utils.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IVelocityControl.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IPidControl.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/IRemoteVariables.h>

#include <iDynTree/yarp/YARPConversions.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <Eigen/Dense>

#include <memory>
#include <cassert>
#include <cstddef>
#include <map>
#include <algorithm>

using namespace WalkingControllers;

class AllJointsSources : public JointsSources {

    yarp::dev::PolyDriver* m_robotDriver;
    yarp::dev::IEncodersTimed* m_encodersInterface;
    yarp::dev::IPidControl* m_pidInterface;
    yarp::dev::IRemoteVariables* m_remoteVariablesInterface;
    yarp::dev::IAxisInfo* m_axisInfo;
    yarp::dev::IControlLimits* m_limitsInfo;
    yarp::sig::Vector m_positionFeedbackInDegrees;
    yarp::sig::Vector m_velocityFeedbackInDegrees;
    std::vector<std::string> m_allJoints;
    bool m_configured;

    friend class WalkingControllers::RobotComponent;
    AllJointsSources()
        : m_robotDriver(nullptr)
        , m_configured(false)
    {}

    bool configure(yarp::dev::PolyDriver* robotDriver) {
        if (m_configured) {
            yError() << "[AllJointsSources::configure] Cannot configure twice.";
            return false;
        }

        if (!robotDriver) {
            yError() << "[AllJointsSources::configure] Empty robotDriver pointer.";
            return false;
        }

        m_robotDriver = robotDriver;

        if (!m_robotDriver->view(m_encodersInterface) || !m_encodersInterface)
        {
            yError() << "[AllJointsSources::configure]Cannot obtain IEncoders interface";
            return false;
        }

        if (!m_robotDriver->view(m_limitsInfo) || !m_limitsInfo)
        {
            yError() << "[AllJointsSources::configure]Cannot obtain IControlLimits interface";
            return false;
        }

        if (!m_robotDriver->view(m_pidInterface) || !m_pidInterface)
        {
            yError() << "[AllJointsSources::configure]Cannot obtain IPidControl interface";
            return false;
        }

        if (!m_robotDriver->view(m_remoteVariablesInterface) || !m_remoteVariablesInterface)
        {
            yError() << "[AllJointsSources::configure]Cannot obtain IRemoteVariables interface";
            return false;
        }

        if (!m_robotDriver->view(m_axisInfo) || !m_axisInfo){
            yError() << "[AllJointsSources::configure]Cannot obtain IAxisInfo interface";
            return false;
        }

        int axes = 0;

        if (!m_encodersInterface->getAxes(&axes) || axes <= 0){
            yError() << "[AllJointsSources::configure]No axes found in controlboard";
            return false;
        }

        m_allJoints.clear();
        m_allJoints.reserve(static_cast<size_t>(axes));
        // read all the joint names
        for (int i = 0; i < axes; ++i) {
            std::string axisName;
            m_axisInfo->getAxisName(i, axisName);
            m_allJoints.push_back(axisName);
        }

        m_positionFeedbackInDegrees.resize(static_cast<unsigned int>(m_allJoints.size()));
        m_positionFeedbackInDegrees.zero();

        m_configured = true;
        return true;
    }

public:

    AllJointsSources(const AllJointsSources& rhs) = delete;

    ~AllJointsSources() override;

    virtual bool getJointsName(std::vector<std::string>& jointNames) override {
        if (!m_configured) {
            yError() << "[AllJointsSources::getJointsName] Not configured yet.";
            return false;
        }

        jointNames = m_allJoints;

        return true;
    }

    virtual bool getPositions(iDynTree::VectorDynSize& jointsPositionsInRad) override {
        if (!m_configured) {
            yError() << "[AllJointsSources::getPositions] Not configured yet.";
            return false;
        }

        if (!(m_encodersInterface->getEncoders(m_positionFeedbackInDegrees.data()))) {
            yError() << "[AllJointsSources::getPositions] Failed to read joints position.";
            return false;
        }

        unsigned int size = static_cast<unsigned int>(m_positionFeedbackInDegrees.size());
        jointsPositionsInRad.resize(size);
        for(unsigned j = 0; j < m_positionFeedbackInDegrees.size(); ++j) {
            jointsPositionsInRad(j) = iDynTree::deg2rad(m_positionFeedbackInDegrees(j));
        }
        return true;
    }

    virtual bool getVelocities(iDynTree::VectorDynSize& jointsVelInRadPerSec) override {
        if (!m_configured) {
            yError() << "[AllJointsSources::getVelocities] Not configred yet.";
            return false;
        }

        if (!(m_encodersInterface->getEncoderSpeeds(m_velocityFeedbackInDegrees.data()))) {
            yError() << "[AllJointsSources::getVelocities] Failed to read joint velocities.";
            return false;
        }

        unsigned int size = static_cast<unsigned int>(m_velocityFeedbackInDegrees.size());
        jointsVelInRadPerSec.resize(size);
        for(unsigned j = 0; j < m_velocityFeedbackInDegrees.size(); ++j) {
            jointsVelInRadPerSec(j) = iDynTree::deg2rad(m_velocityFeedbackInDegrees(j));
        }
        return true;
    }

    virtual bool getPositionLimits(std::vector<std::pair<double, double>>& jointsLimitsInRad) override {
        if (!m_configured) {
            yError() << "[AllJointsSources::getPositionLimits] Not configured yet.";
            return false;
        }

        jointsLimitsInRad.resize(m_allJoints.size());
        double min, max;
        int index = 0;

        for (size_t j = 0; j < m_allJoints.size(); ++j) {
            index = static_cast<int>(j);
            if (!(m_limitsInfo->getLimits(index, &min, &max))) {
                yError() << "[AllJointsSources::getPositionLimits] Failed to get limits for joint " << m_allJoints[j] << ".";
                return false;
            }

            jointsLimitsInRad[j].first = iDynTree::deg2rad(min);
            jointsLimitsInRad[j].second = iDynTree::deg2rad(max);
        }

        return true;
    }

    virtual bool getVelocityLimits(std::vector<std::pair<double, double>>& jointsLimitsInRad) override {
        if (!m_configured) {
            yError() << "[AllJointsSources::getVelocityLimits] Not configured yet.";
            return false;
        }

        jointsLimitsInRad.resize(m_allJoints.size());
        double min, max;
        int index = 0;

        for (size_t j = 0; j < m_allJoints.size(); ++j) {
            index = static_cast<int>(j);
            if (!(m_limitsInfo->getVelLimits(index, &min, &max))) {
                yError() << "[AllJointsSources::getVelocityLimits] Failed to get velocity limits for joint " << m_allJoints[j] << ".";
                return false;
            }

            jointsLimitsInRad[j].first = iDynTree::deg2rad(min);
            jointsLimitsInRad[j].second = iDynTree::deg2rad(max);
        }

        return true;
    }

    virtual bool getPositionPIDs(std::vector<yarp::dev::Pid>& positionPIDs) override {
        if (!m_configured) {
            yError() << "[AllJointsSources::getPositionPIDs] Not configured yet.";
            return false;
        }

        positionPIDs.resize(m_allJoints.size());

        if (!(m_pidInterface->getPids(yarp::dev::VOCAB_PIDTYPE_POSITION, positionPIDs.data()))) {
            yError() << "[AllJointsSources::getPositionPIDs] Failed to get position PIDs.";
            return false;
        }

        return true;
    }

    virtual bool getPositionPIDsSmoothingTimes(iDynTree::VectorDynSize& smoothingTimesInSec) override {
        if (!m_configured) {
            yError() << "[AllJointsSources::getPositionPIDsSmoothingTimes] Not configured yet.";
            return false;
        }

        yarp::os::Bottle input;

        if (!(m_remoteVariablesInterface->getRemoteVariable("posPidSlopeTime", input))) {
            yError() << "[AllJointsSources::getPositionPIDsSmoothingTimes] Failed to get posPidSlope remote variable.";
            return false;
        }

        if (input.size() != m_allJoints.size()) {
            yError() << "[AllJointsSources::getPositionPIDsSmoothingTimes] The bottle size is different from the joints number.";
            return false;
        }

        unsigned int size = static_cast<unsigned int>(m_allJoints.size());

        smoothingTimesInSec.resize(size);

        unsigned int index;
        for (size_t j = 0; j < m_allJoints.size(); ++j) {
            index = static_cast<unsigned int>(j);
            smoothingTimesInSec(index) = input.get(j).asDouble();
        }

        return true;
    }

};
AllJointsSources::~AllJointsSources(){}

class ControlledJointsSources : public JointsSources {

    std::shared_ptr<AllJointsSources> m_allJointInterface;
    std::map<unsigned int, unsigned int> m_controlledToAllJointsMap;
    std::vector<std::string> m_controlledJointsList;
    iDynTree::VectorDynSize m_allJointsPosition, m_allJointsVelocity, m_allJointsSmoothingTimes;
    std::vector<std::pair<double, double>> m_allJointsPositionLimits, m_allJointsVelocityLimits;
    std::vector<yarp::dev::Pid> m_allJointsPIDs;
    unsigned int m_size;
    bool m_configured;

    friend class WalkingControllers::RobotComponent;
    ControlledJointsSources()
        :m_configured(false)
    { }

    bool configure(std::shared_ptr<AllJointsSources> allJointsSources, const std::vector<std::string> &controlledJoints) {
        m_allJointInterface = allJointsSources;


        std::vector<std::string> allJoints, controlledJointsInput;

        if (!(allJointsSources->getJointsName(allJoints))) {
            yError() << "[ControlledJointsSources::configure] Cannot retrieve the full list of joints.";
            return false;
        }

        size_t size = allJoints.size();

        m_allJointsPosition.resize(static_cast<unsigned int>(size));
        m_allJointsVelocity.resize(static_cast<unsigned int>(size));
        m_allJointsSmoothingTimes.resize(static_cast<unsigned int>(size));

        m_allJointsPositionLimits.resize(size);
        m_allJointsVelocityLimits.resize(size);
        m_allJointsPIDs.resize(size);

        controlledJointsInput = (controlledJoints.size() > 0) ? controlledJoints : allJoints;

        unsigned int allJointsIndex, controlledJointsIndex;
        std::vector<std::string>::const_iterator allJointsIterator;

        for (size_t j = 0; j < controlledJointsInput.size(); ++j) {

            allJointsIterator = std::find(allJoints.begin(), allJoints.end(), controlledJointsInput[j]);

            if (allJointsIterator == allJoints.end()) {
                yError() << "[ControlledJointsSources::configure] Unable to find a joint named " << controlledJointsInput[j] << ".";
                return false;
            }

            controlledJointsIndex = static_cast<unsigned int>(j);
            allJointsIndex = static_cast<unsigned int>(allJointsIterator - allJoints.begin());

            m_controlledToAllJointsMap[controlledJointsIndex] = allJointsIndex;
        }

        m_size = static_cast<unsigned int>(m_controlledToAllJointsMap.size());
        m_configured = true;

        return true;
    }

public:

    ControlledJointsSources(const ControlledJointsSources& rhs) = delete;

    virtual ~ControlledJointsSources() override;


    virtual bool getJointsName(std::vector<std::string>& jointNames) override {
        if (!m_configured) {
            yError() << "[ControlledJointsSources::getJointsName] Not configured yet.";
            return false;
        }

        jointNames = m_controlledJointsList;
        return true;
    }

    virtual bool getPositions(iDynTree::VectorDynSize& jointsPositionsInRad) override {
        if (!m_configured) {
            yError() << "[ControlledJointsSources::getPositions] Not configured yet.";
            return false;
        }

        if (!(m_allJointInterface->getPositions(m_allJointsPosition))) {
            yError() << "[ControlledJointsSources::getPositions] Failed to retrieve positions from all joints.";
            return false;
        }

        jointsPositionsInRad.resize(m_size);

        for (unsigned int j = 0; j < m_size; ++j) {
            jointsPositionsInRad(j) = m_allJointsPosition(m_controlledToAllJointsMap[j]);
        }

        return true;
    }

    virtual bool getVelocities(iDynTree::VectorDynSize& jointsVelInRadPerSec) override {
        if (!m_configured) {
            yError() << "[ControlledJointsSources::getVelocities] Not configured yet.";
            return false;
        }

        if (!(m_allJointInterface->getVelocities(m_allJointsVelocity))) {
            yError() << "[ControlledJointsSources::getVelocities] Failed to retrieve velocities from all joints.";
            return false;
        }

        jointsVelInRadPerSec.resize(m_size);

        for (unsigned int j = 0; j < m_size; ++j) {
            jointsVelInRadPerSec(j) = m_allJointsVelocity(m_controlledToAllJointsMap[j]);
        }

        return true;
    }

    virtual bool getPositionLimits(std::vector<std::pair<double, double>>& jointsLimitsInRad) override {
        if (!m_configured) {
            yError() << "[ControlledJointsSources::getPositionLimits] Not configured yet.";
            return false;
        }

        if (!(m_allJointInterface->getPositionLimits(m_allJointsPositionLimits))) {
            yError() << "[ControlledJointsSources::getPositionLimits] Failed to retrieve position limits from all joints.";
            return false;
        }

        jointsLimitsInRad.resize(m_size);

        for (unsigned int j = 0; j < m_size; ++j) {
            jointsLimitsInRad[j] = m_allJointsPositionLimits[m_controlledToAllJointsMap[j]];
        }

        return true;
    }

    virtual bool getVelocityLimits(std::vector<std::pair<double, double>>& jointsLimitsInRad) override {
        if (!m_configured) {
            yError() << "[ControlledJointsSources::getVelocityLimits] Not configured yet.";
            return false;
        }

        if (!(m_allJointInterface->getVelocityLimits(m_allJointsVelocityLimits))) {
            yError() << "[ControlledJointsSources::getVelocityLimits] Failed to retrieve velocity limits from all joints.";
            return false;
        }

        jointsLimitsInRad.resize(m_size);

        for (unsigned int j = 0; j < m_size; ++j) {
            jointsLimitsInRad[j] = m_allJointsVelocityLimits[m_controlledToAllJointsMap[j]];
        }

        return true;
    }

    virtual bool getPositionPIDs(std::vector<yarp::dev::Pid>& positionPIDs) override {
        if (!m_configured) {
            yError() << "[ControlledJointsSources::getPositionPIDs] Not configured yet.";
            return false;
        }

        if (!(m_allJointInterface->getPositionPIDs(m_allJointsPIDs))) {
            yError() << "[ControlledJointsSources::getPositionPIDs] Failed to retrieve PIDs from all joints.";
            return false;
        }

        positionPIDs.resize(m_size);

        for (unsigned int j = 0; j < m_size; ++j) {
            positionPIDs[j] = m_allJointsPIDs[m_controlledToAllJointsMap[j]];
        }

        return true;
    }

    virtual bool getPositionPIDsSmoothingTimes(iDynTree::VectorDynSize& smoothingTimesInSec) override {
        if (!m_configured) {
            yError() << "[ControlledJointsSources::getPositionPIDsSmoothingTimes] Not configured yet.";
            return false;
        }

        if (!(m_allJointInterface->getPositionPIDsSmoothingTimes(m_allJointsSmoothingTimes))) {
            yError() << "[ControlledJointsSources::getPositionPIDsSmoothingTimes] Failed to retrieve the position PID smoothing times from all joints.";
            return false;
        }

        smoothingTimesInSec.resize(m_size);

        for (unsigned int j = 0; j < m_size; ++j) {
            smoothingTimesInSec(j) = m_allJointsSmoothingTimes(m_controlledToAllJointsMap[j]);
        }

        return true;
    }
};
ControlledJointsSources::~ControlledJointsSources(){}

class SelectedJointsSinks : public JointsSinks {

    yarp::dev::PolyDriver* m_robotDriver;
    yarp::dev::IEncodersTimed* m_encodersInterface;
    yarp::dev::IPidControl* m_pidInterface;
    yarp::dev::IRemoteVariables* m_remoteVariablesInterface;
    yarp::dev::IPositionDirect* m_positionDirectInterface;
    yarp::dev::IPositionControl* m_positionInterface;
    yarp::dev::IVelocityControl* m_velocityInterface;
    yarp::dev::IControlMode* m_controlModeInterface;

    std::vector<std::string> m_jointList;
    std::vector<int> m_selectedAxis;
    yarp::os::Bottle m_jointStructure;

    yarp::sig::Vector m_encodersBuffer;
    iDynTree::VectorDynSize m_toDegBuffer, m_allJointsSmoothingTime;

    bool m_configured;

    friend class WalkingControllers::RobotComponent;
    SelectedJointsSinks()
        : m_robotDriver(nullptr)
        , m_configured(false)
    {}

    bool configure(yarp::dev::PolyDriver* robotDriver,
                   const yarp::os::Bottle &jointsStructure,
                   const std::vector<std::string> &jointsList,
                   const std::vector<std::string> &controlledJoints = std::vector<std::string>()) {

        if (m_configured) {
            yError() << "[SelectedJointsSinks::configure] Cannot configure twice.";
            return false;
        }

        if (!robotDriver) {
            yError() << "[SelectedJointsSinks::configure] Empty robotDriver pointer.";
            return false;
        }

        m_robotDriver = robotDriver;

        m_jointStructure = jointsStructure;

        if (!m_robotDriver->view(m_positionInterface) || !m_positionInterface)
        {
            yError() << "[SelectedJointsSinks::configure] Cannot obtain IPositionControl interface";
            return false;
        }

        if (!m_robotDriver->view(m_encodersInterface) || !m_encodersInterface)
        {
            yError() << "[SelectedJointsSinks::configure]Cannot obtain IEncoders interface";
            return false;
        }

        if (!m_robotDriver->view(m_positionDirectInterface) || !m_positionDirectInterface)
        {
            yError() << "[SelectedJointsSinks::configure] Cannot obtain IPositionDirect interface";
            return false;
        }

        if (!m_robotDriver->view(m_velocityInterface) || !m_velocityInterface) {
            yError() << "[SelectedJointsSinks::configure] Cannot obtain IVelocity interface";
            return false;
        }

        if (!m_robotDriver->view(m_controlModeInterface) || !m_controlModeInterface)
        {
            yError() << "[SelectedJointsSinks::configure] Cannot obtain IControlMode interface";
            return false;
        }

        if (!m_robotDriver->view(m_pidInterface) || !m_pidInterface)
        {
            yError() << "[SelectedJointsSinks::configure] Cannot obtain IPidControl interface";
            return false;
        }

        if (!m_robotDriver->view(m_remoteVariablesInterface) || !m_remoteVariablesInterface)
        {
            yError() << "[SelectedJointsSinks::configure] Cannot obtain IRemoteVariables interface";
            return false;
        }

        m_selectedAxis.clear();

        m_jointList = jointsList;

        if (controlledJoints.size() > 0) {

            std::vector<std::string>::const_iterator jointsListIterator;

            for (size_t j = 0; j < controlledJoints.size(); ++j) {

                jointsListIterator = std::find(m_jointList.begin(), m_jointList.end(), controlledJoints[j]);

                if (jointsListIterator == m_jointList.end()) {
                    yError() << "[SelectedJointsSinks::configure] Unable to find a joint named " << controlledJoints[j] << ".";
                    return false;
                }

                m_selectedAxis.push_back(static_cast<int>(jointsListIterator - m_jointList.begin()));
            }
        } else {
            for (size_t i = 0; i < m_jointList.size(); ++i) {
                m_selectedAxis.push_back(static_cast<int>(i));
            }
        }

        //here I need to resize and initialize some buffers
        m_encodersBuffer.resize(m_jointList.size());
        m_allJointsSmoothingTime.resize(static_cast<unsigned int>(m_jointList.size()));
        m_toDegBuffer.resize(static_cast<unsigned int>(m_selectedAxis.size()));

        m_configured = true;

        return true;
    }

    bool verifyReachedPosition(const iDynTree::VectorDynSize& desiredPositionsRAD, std::pair<int, double>& worst) {

        if(!m_encodersInterface->getEncoders(m_encodersBuffer.data())){
            yError() << "[SelectedJointsSinks::verifyReachedPosition] Error reading encoders.";
            return false;
        }
        worst.first = -1;
        worst.second = 0.0;

        double positionDeg, currentPosDeg, positionErr;
        for (unsigned j = 0; j < m_selectedAxis.size(); ++j) {
            positionDeg = iDynTree::rad2deg(desiredPositionsRAD(j));
            currentPosDeg = m_encodersBuffer[static_cast<size_t>(m_selectedAxis[j])];
            positionErr = std::abs(positionDeg - currentPosDeg);
            if (positionErr >= worst.second) {
                worst.first = m_selectedAxis[j];
                worst.second = positionErr;
            }
        }

        return true;
    }


public:

    ~SelectedJointsSinks() override;

    virtual bool setControlMode(const WalkingControllers::ControlModes &controlMode) override {
        if (!m_configured) {
            yError() << "[SelectedJointsSinks::setControlMode] Not configured yet.";
            return false;
        }

        if (controlMode == WalkingControllers::ControlModes::Position) {
            std::vector<int> controlModes(m_selectedAxis.size(), VOCAB_CM_POSITION);
            if (!m_controlModeInterface->setControlModes(static_cast<int>(m_selectedAxis.size()),m_selectedAxis.data(), controlModes.data())) {
                yError() << " [SelectedJointsSinks::setControlMode] Error while setting position mode.";
                return false;
            }
        } else if (controlMode == WalkingControllers::ControlModes::PositionDirect) {
            std::vector<int> controlModes(m_selectedAxis.size(), VOCAB_CM_POSITION_DIRECT);
            if (!m_controlModeInterface->setControlModes(static_cast<int>(m_selectedAxis.size()),m_selectedAxis.data(), controlModes.data())) {
                yError() << " [SelectedJointsSinks::setControlMode] Error while setting position direct mode.";
                return false;
            }
        } else if (controlMode == WalkingControllers::ControlModes::Velocity) {
            std::vector<int> controlModes(m_selectedAxis.size(), VOCAB_CM_VELOCITY);
            if (!m_controlModeInterface->setControlModes(static_cast<int>(m_selectedAxis.size()),m_selectedAxis.data(), controlModes.data())) {
                yError() << " [SelectedJointsSinks::setControlMode] Error while setting velocity mode.";
                return false;
            }
        }

        return true;
    }

    virtual bool setPositionReference(const iDynTree::VectorDynSize& jointsPositionsInRad, double positioningTimeInSec = 5.0) override {
        if (!m_configured) {
            yError() << "[SelectedJointsSinks::setPositionReference] Not configured yet.";
            return false;
        }

        if (jointsPositionsInRad.size() != m_selectedAxis.size()) {
            yError() << "[SelectedJointsSinks::setPositionReference] Dimension mismatch between jointsPositionsInRad and the selected axis.";
            return false;
        }

        std::pair<int, double> worst(-1, 0.0);

        if (!verifyReachedPosition(jointsPositionsInRad, worst))
            return false;

        if (worst.second < 2.0) {
            return true;
        }

        if (positioningTimeInSec < 0.01) {
            yError() << "[SelectedJointsSinks::setPositionReference] The positioning time is too short.";
            return false;
        }

//        if (!setControlMode(WalkingControllers::ControlModes::Position)) {
//            yError()<<"[SelectedJointsSinks::setPositionReference] Failed in setting POSITION mode.";
//            return false;
//        }

        double positionDeg, positionErr;

        if (!m_encodersInterface->getEncoders(m_encodersBuffer.data())) {
            yError() << "[SelectedJointsSinks::setPositionReference] Error while reading encoders.";
            return false;
        }
        worst.first = -1;
        worst.second = 0.0;
        std::vector<double> refSpeeds(m_selectedAxis.size());

        for (unsigned j = 0; j < m_selectedAxis.size(); ++j) {
            positionDeg = iDynTree::rad2deg(jointsPositionsInRad(j));

            positionErr = std::abs(positionDeg - m_encodersBuffer[static_cast<size_t>(m_selectedAxis[j])]);
            refSpeeds[j] = std::max(3.0, positionErr/positioningTimeInSec);
        }

        if (!m_positionInterface->setRefSpeeds(static_cast<int>(m_selectedAxis.size()), m_selectedAxis.data(), refSpeeds.data())) {
            yError() << "[SelectedJointsSinks::setPositionReference] Error while setting the desired speed of joints.";
            return false;
        }

        iDynTree::toEigen(m_toDegBuffer) = iDynTree::toEigen(jointsPositionsInRad) * iDynTree::rad2deg(1);

        if (!m_positionInterface->positionMove(static_cast<int>(m_selectedAxis.size()), m_selectedAxis.data(), m_toDegBuffer.data())) {
            yError() << "Error while setting the desired positions.";
            return false;
        }

        refSpeeds.assign(m_selectedAxis.size(), 5.0);
        if (!m_positionInterface->setRefSpeeds(static_cast<int>(m_selectedAxis.size()), m_selectedAxis.data(), refSpeeds.data())) {
            yError() << "Error while setting the desired speed of joints.";
            return false;
        }

        bool terminated = false;
        int attempt = 0;
        do {
            if (!terminated) {
                m_positionInterface->checkMotionDone(&terminated);
            }

            if (terminated){
                if (!verifyReachedPosition(jointsPositionsInRad, worst))
                    return false;

                if (worst.second < 2.0) {
                    return true;
                }
            }

            yarp::os::Time::delay(positioningTimeInSec * 0.5);
            attempt++;
        } while (attempt < 4);

        if (terminated) {
            yError() << "The joint " << m_jointList[static_cast<size_t>(worst.first)] << " was the worst in positioning with an error of " << worst.second << "[deg].";
        } else {
            yError() << "Unable to complete the motion.";
        }

        return false;
    }

    virtual bool setDirectPositionReference(const iDynTree::VectorDynSize& jointsPositionsInRad) override {

        if (!m_configured) {
            yError() << "[SelectedJointsSinks::setDirectPositionReference] Not configured yet.";
            return false;
        }

        if (jointsPositionsInRad.size() != m_selectedAxis.size()) {
            yError() << "[SelectedJointsSinks::setDirectPositionReference] Dimension mismatch between desired position vector and the number of controlled joints.";
            return false;
        }

        if (!m_encodersInterface->getEncoders(m_encodersBuffer.data())){
            yError() << "[SelectedJointsSinks::setDirectPositionReference] Error reading encoders.";
            return false;
        }

        iDynTree::toEigen(m_toDegBuffer) = iDynTree::toEigen(jointsPositionsInRad) * iDynTree::rad2deg(1);

        for (unsigned j = 0; j < m_selectedAxis.size(); ++j) {

            if (std::abs(m_toDegBuffer(j) - m_encodersBuffer[static_cast<size_t>(m_selectedAxis[j])]) > 15.0){
                yError() << "[SelectedJointsSinks::setDirectPositionReference] The distance between the current and the desired position of joint "
                         << m_jointList[static_cast<size_t>(m_selectedAxis[j])] << " is greater than 15.0 degrees.";
                return false;
            }
        }

        if (!m_positionDirectInterface->setPositions(static_cast<int>(m_selectedAxis.size()), m_selectedAxis.data(), m_toDegBuffer.data())){
            yError() << "[SelectedJointsSinks::setDirectPositionReference] Error while setting the desired position.";
            return false;
        }

        return true;
    }

    virtual bool setVelocityReference(const iDynTree::VectorDynSize& jointsVelInRadPerSec) override {

        if (!m_configured) {
            yError() << "[SelectedJointsSinks::setVelocityReference] Not configured yet.";
            return false;
        }

        if (jointsVelInRadPerSec.size() != m_selectedAxis.size()) {
            yError() << "[SelectedJointsSinks::setVelocityReference] Dimension mismatch between desired velocity "
                     << "vector and the number of selected joints.";
            return false;
        }

        if ((iDynTree::toEigen(jointsVelInRadPerSec).minCoeff() < -iDynTree::deg2rad(30)) ||
           (iDynTree::toEigen(jointsVelInRadPerSec).maxCoeff() > iDynTree::deg2rad(30))) {
            yError() << "[SelectedJointsSinks::setVelocityReference] The absolute value of the desired velocity is higher than 30 deg/s.";
            return false;
        }

        iDynTree::toEigen(m_toDegBuffer) = iDynTree::toEigen(jointsVelInRadPerSec) * iDynTree::rad2deg(1);

        if (!m_velocityInterface->velocityMove(static_cast<int>(m_selectedAxis.size()), m_selectedAxis.data(), m_toDegBuffer.data()))
        {
            yError() << "[SelectedJointsSinks::setVelocityReference] Error while setting the desired velocity.";
            return false;
        }

        return true;
    }

    virtual bool setPositionPIDs(const std::vector<yarp::dev::Pid>& positionPIDs) override {

        if (!m_configured) {
            yError() << "[SelectedJointsSinks::setPositionPIDs] Not configured yet.";
            return false;
        }

        if (positionPIDs.size() != m_selectedAxis.size()) {
            yError() << "[SelectedJointsSinks::setPositionPIDs] The number of desired PIDs should match the number of selected joints.";
            return false;
        }

        for (size_t j = 0; j < m_selectedAxis.size(); ++j) {
            if (!m_pidInterface->setPid(yarp::dev::VOCAB_PIDTYPE_POSITION, m_selectedAxis[j], positionPIDs[j])) {
                yError() << "[SelectedJointsSinks::setPositionPIDs] Error while setting the desired position PID on "
                         << m_jointList[static_cast<size_t>(m_selectedAxis[j])] << " axis.";
                return false;
            }
        }
        return true;
    }

    virtual bool setPositionPIDsSmoothingTimes(const iDynTree::VectorDynSize& smoothingTimesInSec) override {

        if (!m_configured) {
            yError() << "[SelectedJointsSinks::setPositionPIDsSmoothingTimes] Not configured yet.";
            return false;
        }

        if (smoothingTimesInSec.size() != m_selectedAxis.size()) {
            yError() << "[SelectedJointsSinks::setPositionPIDsSmoothingTimes] The number of desired smoothing times should match the number of selected joints.";
            return false;
        }

        yarp::os::Bottle input, output;

        if (!(m_remoteVariablesInterface->getRemoteVariable("posPidSlopeTime", input))) {
            yError() << "[SelectedJointsSinks::setPositionPIDsSmoothingTimes] Failed to get the current posPidSlope remote variable.";
            return false;
        }

        if (input.size() != m_allJointsSmoothingTime.size()) {
            yError() << "[SelectedJointsSinks::setPositionPIDsSmoothingTimes] The bottle size is different from the joints number.";
            return false;
        }

        for (unsigned int j = 0; j < m_allJointsSmoothingTime.size(); ++j) {
            m_allJointsSmoothingTime(j) = input.get(j).asDouble();
        }

        for (size_t j = 0; j < m_selectedAxis.size(); ++j) {
            m_allJointsSmoothingTime(static_cast<unsigned int>(m_selectedAxis[j])) = smoothingTimesInSec(static_cast<unsigned int>(j));
        }

        unsigned int jointIndex = 0;
        for (size_t b = 0; b < m_jointStructure.size(); ++b) {
            yarp::os::Bottle &innerList = output.addList();

            for (size_t j = 0; j < m_jointStructure.get(b).asList()->size(); ++j) {
                assert(jointIndex < m_allJointsSmoothingTime.size());
                innerList.addDouble(m_allJointsSmoothingTime(jointIndex));
                ++jointIndex;
            }
        }

        assert(jointIndex == m_allJointsSmoothingTime.size());

        if (!(m_remoteVariablesInterface->setRemoteVariable("posPidSlopeTime", output))) {
            yError() << "[SelectedJointsSinks::setPositionPIDsSmoothingTimes] Failed to set the desired posPidSlope remote variable.";
            return false;
        }

        return true;
    }

};
SelectedJointsSinks::~SelectedJointsSinks(){}

class RobotComponent::RobotComponentImplementation {
public:
    yarp::dev::PolyDriver* robotDriver;
    yarp::os::Bottle remoteControlBoards;
    std::string robotName;
    std::shared_ptr<AllJointsSources> allJointsSources_ptr;
    std::shared_ptr<ControlledJointsSources> controlledJointsSources_ptr;
    std::shared_ptr<SelectedJointsSinks> allJointsSinks_ptr, controlledJointsSinks_ptr;
};

RobotComponent::RobotComponent()
    :m_pimpl(new RobotComponentImplementation())
{
    assert(m_pimpl);

    m_pimpl->allJointsSources_ptr.reset(new AllJointsSources());
    m_pimpl->controlledJointsSources_ptr.reset(new ControlledJointsSources());
    m_pimpl->allJointsSinks_ptr.reset(new SelectedJointsSinks());
    m_pimpl->controlledJointsSinks_ptr.reset(new SelectedJointsSinks());
}

RobotComponent::~RobotComponent()
{
    if (m_pimpl->robotDriver) {
        if (!(m_pimpl->controlledJointsSinks_ptr->setControlMode(WalkingControllers::ControlModes::Position))) {
            yError() << "[RobotComponent::~RobotComponent] Failed in setting POSITION mode.";
        }

        m_pimpl->robotDriver->close();
        delete m_pimpl->robotDriver;
        m_pimpl->robotDriver = nullptr;
    }

    if (m_pimpl) {
        delete m_pimpl;
        m_pimpl = nullptr;
    }
}

bool RobotComponent::configure(const std::string& robotName, const yarp::os::Value &remoteControlBoards,
                               iDynTree::Model& URDFmodel, const std::vector<std::string> &controlledJoints)
{
    m_pimpl->robotName = robotName;

    std::string errorSign = "[RobotComponent::configure]";
    //yarp::os::Bottle remoteControlBoards;
    m_pimpl->remoteControlBoards.clear();
    yarp::os::Bottle &remoteControlBoardsList = m_pimpl->remoteControlBoards.addList();

    yarp::os::Bottle *devicesList = remoteControlBoards.asList();
    for (size_t dev = 0; dev < devicesList->size(); ++dev) {
        if(devicesList->get(dev).isString()){
            remoteControlBoardsList.addString("/" + m_pimpl->robotName + "/" + devicesList->get(dev).asString());
        } else {
            yError() << errorSign << "Unrecognized element in the remoteControlBoards list: " << devicesList->get(dev).toString() << ".";
            return false;
        }
    }

    yarp::os::Bottle allAxis;
    yarp::os::Bottle &allAxisList = allAxis.addList();

    yarp::dev::PolyDriver tempDriver;
    yarp::dev::IEncoders* tempEncoders;
    yarp::dev::IAxisInfo* tempAxis;

    yarp::os::Property tempOptions;
    tempOptions.put("local", "/RobotComponent/temp");
    tempOptions.put("device", "remote_controlboard");

    int tempAxesNum = 0;
    std::string tempString;

    yarp::os::Bottle jointsStructure;
    std::vector<std::string> jointList;

    for (size_t rcb = 0; rcb < remoteControlBoardsList.size(); ++rcb) {
        tempOptions.put("remote", remoteControlBoardsList.get(rcb));
        if (!tempDriver.open(tempOptions)) {
            yError() << errorSign << "Error while opening " << remoteControlBoardsList.get(rcb).asString() << " control board.";
            return false;
        }

        if (!tempDriver.view(tempEncoders) || !tempEncoders) {
           yError() << errorSign << "Cannot obtain IEncoders interface in control board "<< remoteControlBoardsList.get(rcb).asString();
            return false;
        }

        if (!tempDriver.view(tempAxis) || !tempAxis) {
            yError() << errorSign << "Cannot obtain IAxis interface in control board " << remoteControlBoardsList.get(rcb).asString();
            return false;
        }

        yarp::os::Bottle &subList = jointsStructure.addList();

        if (!tempEncoders->getAxes(&tempAxesNum) || tempAxesNum <= 0) {
            yError() << errorSign << "No axes found in controlboard "<< remoteControlBoardsList.get(rcb).asString();

        } else{

            for(int ax = 0; ax < tempAxesNum; ++ax){
                if (!(tempAxis->getAxisName(ax, tempString))){
                    yError() << errorSign <<
                                "Failed in getting the name of axis "<< ax << " in control board " << remoteControlBoardsList.get(rcb).asString();
                    return false;
                }
                if (URDFmodel.getJointIndex(tempString) != iDynTree::JOINT_INVALID_INDEX){
                    allAxisList.addString(tempString);
                    jointList.push_back(tempString);
                    subList.addDouble(0.0);
                } else {
                    yWarning() << errorSign << "Joint " << tempString << " not found in the URDF model.";
                }
            }
        }
        tempDriver.close();
        tempEncoders = nullptr;
        tempAxis = nullptr;
    }

    yarp::os::Property deviceOptions;
    deviceOptions.put("device", "remotecontrolboardremapper");
    deviceOptions.put("remoteControlBoards", m_pimpl->remoteControlBoards.get(0));
    deviceOptions.put("localPortPrefix", "/RobotComponent/player");
    deviceOptions.put("axesNames", allAxis.get(0));
    yarp::os::Property & remoteControlBoardsOpts = deviceOptions.addGroup("REMOTE_CONTROLBOARD_OPTIONS");
    remoteControlBoardsOpts.put("writeStrict","on");

    if (m_pimpl->robotDriver) {
        m_pimpl->robotDriver->close();
    }

    m_pimpl->robotDriver = new yarp::dev::PolyDriver();
    if (!(m_pimpl->robotDriver) || !(m_pimpl->robotDriver)->open(const_cast<yarp::os::Property&>(deviceOptions)))
    {
        yError() << errorSign << "Problem in opening the robot controlboard";
        return false;
    }

    if (!(m_pimpl->allJointsSources_ptr->configure(m_pimpl->robotDriver))) {
        yError() << errorSign << "Error while configuring the allJointsSources object.";
        return false;
    }

    if (!(m_pimpl->controlledJointsSources_ptr->configure(m_pimpl->allJointsSources_ptr, controlledJoints))) {
        yError() << errorSign << "Error while configuring the controlledJointsSources object.";
        return false;
    }

    if (!(m_pimpl->allJointsSinks_ptr->configure(m_pimpl->robotDriver, jointsStructure, jointList))) {
        yError() << errorSign << "Error while configuring the allJointsSinks object.";
        return false;
    }

    if (!(m_pimpl->controlledJointsSinks_ptr->configure(m_pimpl->robotDriver, jointsStructure, jointList, controlledJoints))) {
        yError() << errorSign << "Error while configuring the controlledJointsSinks object.";
        return false;
    }

    return true;
}

JointsSources &RobotComponent::allJointsSources()
{
    return *(m_pimpl->allJointsSources_ptr);
}

JointsSources &RobotComponent::controlledJointsSources()
{
    return *(m_pimpl->controlledJointsSources_ptr);
}

JointsSinks &RobotComponent::allJointsSinks()
{
    return *(m_pimpl->allJointsSinks_ptr);
}

JointsSinks &RobotComponent::controlledJointsSinks()
{
    return *(m_pimpl->controlledJointsSinks_ptr);
}
