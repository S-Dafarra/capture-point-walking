/**
 * @file ZMPWalkingCoordinator.cpp
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#include <ZMPWalkingCoordinator.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>

using namespace WalkingControllers;



bool ZMPWalkingCoordinator::configureSinkPort(const std::string &name, YarpSigVectorPortSink &port)
{
    if (!port.configure("/"+getName()+"/"+name)) {
        yError() << "Failed to open port named " << "/"+getName()+"/"+name;
        return false;
    }
    return true;
}

bool ZMPWalkingCoordinator::configureOutputPorts(const yarp::os::Searchable &options)
{
    if (options.isNull()) {
        yError() << "Empy configuration for output ports.";
        return false;
    }

    yarp::os::Value* result;
    if(!options.check("controlRobot", result)){
        std::cerr << "Missing field controlRobot in OUTPUT_PORTS section." <<std::endl;
        return false;
    }
    m_outputPorts.controlRobot = result->asBool();

    if (!configureSinkPort(options.check("com_port", yarp::os::Value("com:o")).asString(), m_outputPorts.com) ||
        !configureSinkPort(options.check("leftFoot_port", yarp::os::Value("leftFoot:o")).asString(), m_outputPorts.leftFoot) ||
        !configureSinkPort(options.check("rightFoot_port", yarp::os::Value("rightFoot:o")).asString(), m_outputPorts.rightFoot) ||
        !configureSinkPort(options.check("joints_port", yarp::os::Value("joints:o")).asString(), m_outputPorts.joints) ||
        !configureSinkPort(options.check("contacts_port", yarp::os::Value("contact:o")).asString(), m_outputPorts.contacts) ||
        !configureSinkPort(options.check("leftInContact_port", yarp::os::Value("leftStanding:o")).asString(), m_outputPorts.leftInContact) ||
        !configureSinkPort(options.check("weights_port", yarp::os::Value("weights:o")).asString(),m_outputPorts.weights)){

        m_outputPorts.useOutputPorts = false;
        m_outputPorts.controlRobot = true;
        return false;
    }

    if (options.check("ack_port", result)){

        if(!m_outputPorts.ack.configure("/"+getName()+"/"+result->asString())){
            yError() << "[ZMPWalkingCoordinator::configureOutputPorts] Failed to open the ack_port with the name "<< result->asString();
            return false;
        }
        yInfo()<<"Waiting for ack at port " <<"/"+getName()+"/"+result->asString();
        m_outputPorts.waitResponse = true;
    }

    m_outputPorts.useOutputPorts = true;
    return true;
}

bool ZMPWalkingCoordinator::configureWorldToBaseFromPort(const yarp::os::Searchable &options)
{
    m_useExternalWHB = false;
    if (options.isNull()) {
        return true;
    }

    std::string externalWHBLink;

    if (!YarpHelper::getStringFromSearchable(options, "base_link", externalWHBLink)) {
        return false;
    }

    if (!(m_kinDyn.setFloatingBase(externalWHBLink))){
        yError() << "[ZMPWalkingCoordinator::configureWorldToBaseFromPort] Unable to place the floating base on " << externalWHBLink << ".";
        return false;
    }

    std::string portName;
    if (!YarpHelper::getStringFromSearchable(options, "wHb_port", portName)) {
        return false;
    }

    if(!m_externalWHB.configure("/"+getName()+"/"+portName)){
        yError() << "[ZMPWalkingCoordinator::configureWorldToBaseFromPort] Failed to open the wHb_port with the name "<< portName;
        return false;
    }

    yInfo()<<"Using external wHb. Waiting at port " << "/"+getName()+"/"+portName;
    m_useExternalWHB = true;

    return true;
}

bool ZMPWalkingCoordinator::prepareWalkingPrivate()
{

}

bool ZMPWalkingCoordinator::stopWalkingPrivate()
{

}

bool ZMPWalkingCoordinator::pauseWalkingPrivate()
{

}

bool ZMPWalkingCoordinator::startWalkingPrivate()
{

}

bool ZMPWalkingCoordinator::onTheFlyStartWalkingPrivate(double smoothingTime)
{

}

bool ZMPWalkingCoordinator::configurePrivate(yarp::os::ResourceFinder &rf)
{
    m_verbose = false;
    if (rf.check("verbose")) {
        m_verbose = true;
    }

    std::string name = rf.check("name", yarp::os::Value("walking-coordinator")).asString();
    setName(name.c_str());

    m_periodInSeconds = rf.check("period", yarp::os::Value(0.01)).asDouble();

    m_robotComponent = std::make_shared<RobotComponent>();

    if (!(m_robotComponent->configure(rf.findGroup("ROBOT")))) {
        yError() << "[ZMPWalkingCoordinator::configurePrivate] Failed to configure robot.";
        return false;
    }

    if (!(m_robotComponent->controlledJointsSources().getJointsName(m_controlledJoints))) {
        yError() << "[ZMPWalkingCoordinator::configurePrivate] Failed to retrieve controlled joints list.";
        return false;
    }

    m_IK = std::make_shared<NonLinearIKComponent>();

    if (!(m_IK->configure(rf.findGroup("IK"), m_robotComponent->robotModel(), m_controlledJoints))) {
        yError() << "[ZMPWalkingCoordinator::configurePrivate] Failed to configure IK.";
        return false;
    }

    m_PIDHandler = std::make_shared<PIDHandlerComponent>();

    if (!(m_PIDHandler->configure(rf.findGroup("PID"), m_robotComponent, m_periodInSeconds))) {
        yError() << "[ZMPWalkingCoordinator::configurePrivate] Failed to configure PID handler.";
        return false;
    }

    m_bypassMPC = rf.check("bypassMPC", yarp::os::Value(false)).asBool();

    if (!m_bypassMPC) {
        m_previewController = std::make_shared<PreviewController>();

        if (!(m_previewController->configure(rf.findGroup("PREVIEW")))) {
            yError() << "[ZMPWalkingCoordinator::configurePrivate] Failed to configure Preview controller.";
            return false;
        }
    } else {
        yInfo() << "[ZMPWalkingCoordinator::configurePrivate] Bypassing MPC.";
    }

    yarp::os::Bottle& joypadSettings = rf.findGroup("UNICYCLE").findGroup("JOYPAD");

    if (joypadSettings.isNull()) {
        yInfo() << "[ZMPWalkingCoordinator::configurePrivate] No joypad settings provided.  Using the port /unicyclePlanner/referencePosition:i instead.";
        m_unicycleSource = std::make_shared<UnicyclePortSource>();
    } else {
        std::shared_ptr<UnicycleJoypadSource> unicycleJoypad = std::make_shared<UnicycleJoypadSource>();

        if (!(unicycleJoypad->configure(joypadSettings))) {
            yError() << "[ZMPWalkingCoordinator::configurePrivate] Failed to configure joypad.";
            return false;
        }
        m_unicycleSource = unicycleJoypad;
    }

    m_unicycleGenerator = std::make_shared<UnicycleGeneratorComponent>();

    if (!(m_unicycleGenerator->configure(rf.findGroup("UNICYCLE"), m_unicycleSource))) {
        yError() << "[ZMPWalkingCoordinator::configurePrivate] Failed to configure unicycle generator.";
        return false;
    }

    m_correctTrajectories = rf.check("correctTrajectories", yarp::os::Value(false)).asBool();

    if (m_correctTrajectories)
        yInfo() << "Correcting the trajectories with the measured position of the feet.";

    yarp::os::Bottle &portsSection = rf.findGroup("OUTPUT_PORTS");
    if (!portsSection.isNull()) {
        if (!configureOutputPorts(portsSection)) {
            yError("[ZMPWalkingCoordinator::configurePrivate] Failed to configure the output ports.");
            return false;
        }
        yInfo() <<"[ZMPWalkingCoordinator::configurePrivate] Output ports opened.";
    }

    if (!configureWorldToBaseFromPort(rf.findGroup("WHB"))) {
        yError("[ZMPWalkingCoordinator::configurePrivate] Failed to configure the external world to base.");
        return false;
    }

    yarp::os::Value clockValue = rf.find("gazeboClock");

    if (!clockValue.isNull()) {
        yInfo() << "[ZMPWalkingCoordinator::configurePrivate] Connecting to Gazebo clock";
        m_clockServerName = clockValue.asString();
        m_useGazeboClock = true;
    }

    if (m_useGazeboClock && !m_clockClient.open("/"+getName()+"/clock:o")) {
        yWarning() << "[ZMPWalkingCoordinator::configurePrivate] Error opening the clock client.";
        m_useGazeboClock = false;
    }

    if (m_useGazeboClock && !yarp::os::Network::connect(m_clockClient.getName(), m_clockServerName)) {
        yWarning() << "[ZMPWalkingCoordinator::configurePrivate] Unable to connect to the clock port named " << m_clockServerName;
        m_clockClient.close();
        m_useGazeboClock = false;
    }

    if(m_useGazeboClock){
        m_clockServer.yarp().attachAsClient(m_clockClient);
        m_numberOfSteps = static_cast<int>(m_periodInSeconds / m_clockServer.getStepSize());
    }

    // open RPC port for external commands
    std::string rpcPortName = "/" + getName() + "/rpc";
    this->yarp().attachAsServer(this->m_rpcPort);
    if (!m_rpcPort.open(rpcPortName)) {
        yError("[ZMPWalkingCoordinator::configurePrivate] Could not open %s RPC port", rpcPortName.c_str());
        close();
        return false;
    }

    if (!m_kinDyn.loadRobotModel(m_robotComponent->robotModel())) {
        yError() << "[ZMPWalkingCoordinator::configurePrivate] Failed to load robot model to KinDynComputations.";
        return false;
    }

    m_kinDyn.setFrameVelocityRepresentation(iDynTree::MIXED_REPRESENTATION);

    yInfo("[ZMPWalkingCoordinator::configurePrivate] Walking coordinator ready");

    m_state = WalkingState::Still;

    return true;
}

bool ZMPWalkingCoordinator::updatePrivate()
{

}

ZMPWalkingCoordinator::ZMPWalkingCoordinator()
{
}

ZMPWalkingCoordinator::~ZMPWalkingCoordinator()
{

}

std::string ZMPWalkingCoordinator::helpDescription() const
{
    std::stringstream ss;
    ss << "ZMP Preview Walking Coordinator module" << std::endl
    << "" << std::endl << std::endl
    << "Usage:" << std::endl
    << "zmpPreviewWalkingController [options]" << std::endl << std::endl

    << "Recognised options:" << std::endl
    << " --help\tprints this description" << std::endl;

    return ss.str();
}

bool ZMPWalkingCoordinator::close()
{

}
