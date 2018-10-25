/**
 * @file ZMPWalkingCoordinator.h
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef ZMPWALKINGCOORDINATOR_H
#define ZMPWALKINGCOORDINATOR_H

#include <WalkingControllersComponents.h>
#include <PreviewController.h>
#include <iDynTree/KinDynComputations.h>

#include <ClockServer.h>
#include <memory>

namespace WalkingControllers {
    class ZMPWalkingCoordinator;
}

class WalkingControllers::ZMPWalkingCoordinator : public WalkingControllers::WalkingCoordinatorComponent {

    enum class WalkingState {
        Still,
        Prepared,
        Walking,
        Paused,
        Restoring,
        OnTheFly
    };

    WalkingState m_state;

    std::shared_ptr<RobotComponent> m_robotComponent;
    iDynTree::KinDynComputations m_kinDyn;
    std::shared_ptr<NonLinearIKComponent> m_IK;
    std::vector<std::string> m_controlledJoints;
    std::shared_ptr<PIDHandlerComponent> m_PIDHandler;
    bool m_bypassMPC;
    std::shared_ptr<PreviewController> m_previewController;
    std::shared_ptr<UnicycleGeneratorComponent> m_unicycleGenerator;
    std::shared_ptr<UnicycleReferenceSource> m_unicycleSource;

    bool m_correctTrajectories;

    bool m_useGazeboClock;
    yarp::os::Port m_clockClient;
    GazeboYarpPlugins::ClockServer m_clockServer;
    std::string m_clockServerName;
    int m_numberOfSteps;

    yarp::os::Port m_rpcPort;

    typedef struct {
        YarpSigVectorPortSink com;
        YarpSigVectorPortSink leftFoot;
        YarpSigVectorPortSink rightFoot;
        YarpSigVectorPortSink joints;
        YarpSigVectorPortSink contacts;
        YarpSigVectorPortSink leftInContact;
        YarpSigVectorPortSink weights;
        YarpSigVectorPortSource ack;
        bool controlRobot = true;
        bool waitResponse = false;
        bool useOutputPorts = false;
    } OutputPorts;

    OutputPorts m_outputPorts;

    bool m_useExternalWHB;

    YarpSigVectorPortSource m_externalWHB;


    bool m_verbose;

    bool configureOutputPorts(const yarp::os::Searchable& options);

    bool configureSinkPort(const std::string &name, YarpSigVectorPortSink &port);

    bool configureWorldToBaseFromPort(const yarp::os::Searchable &options);

    virtual bool prepareWalkingPrivate() override;

    virtual bool stopWalkingPrivate() override;

    virtual bool pauseWalkingPrivate() override;

    virtual bool startWalkingPrivate() override;

    virtual bool onTheFlyStartWalkingPrivate(double smoothingTime=2.0) override;

    virtual bool configurePrivate(yarp::os::ResourceFinder &rf) override;

    virtual bool updatePrivate() override;

public:

    ZMPWalkingCoordinator();

    virtual ~ZMPWalkingCoordinator() override;

    std::string helpDescription() const;

    virtual bool close() override;
};

#endif // ZMPWALKINGCOORDINATOR_H
