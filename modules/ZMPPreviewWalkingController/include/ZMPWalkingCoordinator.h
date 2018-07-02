/**
 * @file ZMPWalkingCoordinator.h
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef ZMPWALKINGCOORDINATOR_H
#define ZMPWALKINGCOORDINATOR_H

#include <WalkingCoordinatorComponent.h>
#include <RobotComponent.h>
#include <NonLinearIKComponent.h>
#include <memory>

namespace WalkingControllers {
    class ZMPWalkingCoordinator;
}

class WalkingControllers::ZMPWalkingCoordinator : public WalkingControllers::WalkingCoordinatorComponent {

    std::shared_ptr<RobotComponent> m_robotComponent;

    bool m_verbose;

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
