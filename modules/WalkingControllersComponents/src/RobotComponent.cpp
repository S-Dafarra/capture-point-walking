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
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IPidControl.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/IRemoteVariables.h>

#include <iDynTree/yarp/YARPConversions.h>

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
//    yarp::dev::IPositionDirect* m_positionDirectInterface;
//    yarp::dev::IPositionControl* m_positionInterface;
//    yarp::dev::IControlMode* m_controlModeInterface;
    yarp::dev::IAxisInfo* m_axisInfo;
    yarp::dev::IControlLimits* m_limitsInfo;
    yarp::sig::Vector m_positionFeedbackInDegrees;
    yarp::sig::Vector m_velocityFeedbackInDegrees;
    std::vector<std::string> m_allJoints;
    bool m_configured;

protected:
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

//        if (!m_robotDriver->view(m_positionInterface) || !m_positionInterface)
//        {
//            yError() << "[AllJointsSources::configure]Cannot obtain IPositionControl interface";
//            return false;
//        }

//        if (!m_robotDriver->view(m_positionDirectInterface) || !m_positionDirectInterface)
//        {
//            yError() << "[AllJointsSources::configure]Cannot obtain IPositionDirect interface";
//            return false;
//        }

//        if (!m_robotDriver->view(m_controlModeInterface) || !m_controlModeInterface)
//        {
//            yError() << "[AllJointsSources::configure]Cannot obtain IControlMode interface";
//            return false;
//        }

        if (!m_robotDriver->view(m_axisInfo) || !m_axisInfo){
            yError() << "[AllJointsSources::configure]Cannot obtain IAxisInfo interface";
            return false;
        }

        int axes = 0;

        if (!m_encodersInterface->getAxes(&axes) || axes <= 0){
            yError() << "[AllJointsSources::configure]No axes found in controlboard";
            return false;
        }

        m_allJoints.reserve(static_cast<size_t>(axes));
        // read all the joint names
        for (int i = 0; i < axes; ++i) {
            std::string axisName;
            m_axisInfo->getAxisName(i, axisName);
            m_allJoints.push_back(axisName);
        }

//        if(!getJointList(rf)){
//            return false;
//        }

//        if (m_controlledJoints.empty()) {
//            //control all joints
//            yWarning() << "No joint list provided. Using the full robot.";
//            m_controlledJoints = m_allJoints;
//            for (int i = 0; i < axes; ++i) {
//                m_controlledJointsIndices.push_back(i);
//            }
//        } else {
//            //control only the chosen one
//            for (auto& controlledJoint : m_controlledJoints) {
//                std::vector<std::string>::iterator controlBoardIndex = std::find(m_allJoints.begin(), m_allJoints.end(), controlledJoint);
//                if (controlBoardIndex == m_allJoints.end()) {
//                    yError("Axes %s not found in control board", controlledJoint.c_str());
//                    close();
//                    return false;
//                }
//                m_controlledJointsIndices.push_back(static_cast<int>(std::distance(m_allJoints.begin(), controlBoardIndex)));
//            }
//        }

        m_positionFeedbackInDegrees.resize(static_cast<unsigned int>(m_allJoints.size()));
        m_positionFeedbackInDegrees.zero();

//        m_encoderBuffer.resize(static_cast<unsigned int>(m_allJoints.size()));
//        m_encoderBuffer.zero();

//        m_positionFeedbackInRadians.resize(static_cast<unsigned int>(m_allJoints.size()));
//        m_positionFeedbackInRadians.zero();

//        m_previousPositionFeedbackInRadians.resize(static_cast<unsigned int>(m_allJoints.size()));
//        m_previousPositionFeedbackInRadians.zero();

//        m_toDegBuffer.resize(static_cast<unsigned int>(m_controlledJoints.size()));
//        m_toDegBuffer.zero();

//        m_velocityFeedbackInDegrees.resize(static_cast<unsigned int>(m_allJoints.size()));
//        m_velocityFeedbackInDegrees.zero();
//        m_velocityFeedbackInRadians.resize(static_cast<unsigned int>(m_allJoints.size()));
//        m_velocityFeedbackInRadians.zero();

//        m_positionFeedbackInDegreesFiltered.resize(static_cast<unsigned int>(m_allJoints.size()));
//        m_positionFeedbackInDegreesFiltered.zero();

//        m_velocityFeedbackInDegreesFiltered.resize(static_cast<unsigned int>(m_allJoints.size()));
//        m_velocityFeedbackInDegreesFiltered.zero();

//        m_qResult.resize(static_cast<unsigned int>(m_controlledJoints.size()));
//        m_qResult.zero();

//        m_initialJointValuesInRadYarp.resize(static_cast<unsigned int>(m_controlledJoints.size()));
//        m_initialJointValuesInRadYarp.zero();
//        m_desiredJointInRadYarp.resize(static_cast<unsigned int>(m_controlledJoints.size()));
//        m_desiredJointInRadYarp.zero();
//        m_initialJointValuesInRad.resize(static_cast<unsigned int>(m_controlledJoints.size()));
//        m_initialJointValuesInRad.zero();
//        m_desiredJointInRad.resize(static_cast<unsigned int>(m_controlledJoints.size()));
//        m_desiredJointInRad.zero();

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

protected:

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

class RobotComponent::RobotComponentImplementation {
public:
    yarp::dev::PolyDriver* robotDriver;
    yarp::os::Bottle remoteControlBoards;
    std::string robotName;
    yarp::os::Bottle jointsStructure;
    std::shared_ptr<AllJointsSources> allJointsSources_ptr;
    std::shared_ptr<ControlledJointsSources> controlledJointsSources_ptr;
};

RobotComponent::RobotComponent()
    :m_pimpl(new RobotComponentImplementation())
{
    assert(m_pimpl);

    m_pimpl->allJointsSources_ptr.reset(new AllJointsSources());
    m_pimpl->controlledJointsSources_ptr.reset(new ControlledJointsSources());
}

RobotComponent::~RobotComponent()
{
    if (m_pimpl) {
        delete m_pimpl;
        m_pimpl = nullptr;
    }
}

bool RobotComponent::configure(const yarp::os::Searchable &rf, const iDynTree::Model URDFmodel, const std::vector<std::string> &controlledJoints)
{
    std::string errorSign = "[RobotComponent::configure]";
    //yarp::os::Bottle remoteControlBoards;
    m_pimpl->remoteControlBoards.clear();
    yarp::os::Bottle &remoteControlBoardsList = m_pimpl->remoteControlBoards.addList();
    yarp::os::Value* inputControlBoards;

    m_pimpl->robotName = rf.check("robot", yarp::os::Value("icubSim")).asString();

    if (!YarpHelper::getStringFromSearchable(rf, "robot", m_pimpl->robotName)) {
        return false;
    }


    if (rf.check("remoteControlBoards", inputControlBoards)){
        yarp::os::Bottle *devicesList = inputControlBoards->asList();
        for (size_t dev = 0; dev < devicesList->size(); ++dev) {
            if(devicesList->get(dev).isString()){
                remoteControlBoardsList.addString("/" + m_pimpl->robotName + "/" + devicesList->get(dev).asString());
            }
        }
    } else {
        remoteControlBoardsList.addString("/" + m_pimpl->robotName + "/all_joints");
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

    m_pimpl->jointsStructure.clear();

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

        yarp::os::Bottle subList = m_pimpl->jointsStructure.addList();

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
