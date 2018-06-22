/**
 * @file PIDHandlerComponent.h
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef PIDHANDLERCOMPONENT_H
#define PIDHANDLERCOMPONENT_H

#include <RobotComponent.h>

#include <memory>
#include <map>
#include <string>
#include <vector>
#include <deque>
#include <yarp/os/Bottle.h>
#include <yarp/os/Value.h>
#include <mutex>
#include <thread>
#include <condition_variable>

namespace WalkingControllers {

    typedef std::map<std::string, size_t> AxisMap;

    typedef std::map<std::string, yarp::dev::Pid> PIDmap;

    enum class PIDPhase {
        Default,
        SwingLeft,
        SwingRight,
        Switch
    };

    class PIDSchedulingObject;

    class WalkingPIDHandler;

}



class WalkingControllers::PIDSchedulingObject {

    std::string m_name;
    PIDmap m_desiredPIDs;
    PIDPhase m_activationPhase;
    double m_activationOffset;
    double m_computedInitTime;
    double m_dT;

public:
    PIDSchedulingObject(const std::string &name, const PIDPhase &activationPhase, double activationOffset, const PIDmap &desiredPIDs);

    bool setSmoothingTime(double smoothingTime);

    bool setPeriod(double dT);

    const PIDmap& getDesiredGains();

    bool computeInitTime(double time, const std::vector<PIDPhase> &phases, double currentPhaseInitTime);

    double initTime();

    std::string name();

};

class WalkingControllers::WalkingPIDHandler {

    std::shared_ptr<WalkingControllers::RobotComponent> m_robotComponent;
    bool m_useGainScheduling;
    AxisMap m_axisMap;
    PIDmap m_originalPID;
    PIDmap m_defaultPID;
    std::vector<yarp::dev::Pid> m_defaultPIDsVector;
    std::vector<yarp::dev::Pid> m_outputPIDs;
    std::vector<PIDSchedulingObject> m_PIDs;
    std::vector<PIDPhase> m_phases;
    double m_phaseInitTime;
    PIDPhase m_previousPhase;
    int m_currentPIDIndex;
    int m_desiredPIDIndex;
    double m_firmwareDelay, m_maximumContactDelay;
    std::deque<bool> m_leftIsFixedModified, m_rightIsFixedModified;
    double m_dT;
    iDynTree::VectorDynSize m_originalSmoothingTimes, m_desiredSmoothingTimes;

    std::mutex m_mutex;
    std::condition_variable m_conditionVariable;
    std::thread m_handlerThread;

    bool m_configured;

    bool getAxisMap();

    bool getPID(PIDmap& output);

    bool setPID(const PIDmap& pidMap);

    bool isPIDElement(const yarp::os::Value &groupElement);

    bool parsePIDGroup(const yarp::os::Bottle* group, PIDmap& pidMap);

    bool parseDefaultPIDGroup(const yarp::os::Bottle* defaultGroup);

    bool parsePIDElement(const yarp::os::Value &groupElement, std::string &jointName, yarp::dev::Pid &pid);

    bool parsePIDConfigurationFile(const yarp::os::Bottle& PIDSettings);

    bool fromStringToPIDPhase(const std::string &input, PIDPhase &output);

    bool guessPhases(const std::deque<bool> &leftIsFixed, const std::deque<bool> &rightIsFixed);

    void setPIDThread();

    bool modifyFixedLists(std::deque<bool> &leftIsFixed, std::deque<bool> &rightIsFixed, bool leftIsActuallyFixed, bool rightIsActuallyFixed);

public:
    WalkingPIDHandler();

    ~WalkingPIDHandler();

    bool configure(const yarp::os::Bottle& PIDSettings, std::shared_ptr<WalkingControllers::RobotComponent>& robotComponent, double dT);

    bool setMaximumContactDelay(double maxContactDelay);

    bool restorePIDs();

    bool usingGainScheduling();

    bool updatePhases(const std::deque<bool> &leftIsFixed, const std::deque<bool> &rightIsFixed, double time);

    bool updatePhases(const std::deque<bool> &leftIsFixed, const std::deque<bool> &rightIsFixed, bool leftIsActuallyFixed, bool rightIsActuallyFixed, double time);

    bool reset();

};

#endif // PIDHANDLERCOMPONENT_H
