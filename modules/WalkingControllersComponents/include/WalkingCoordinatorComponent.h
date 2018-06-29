/**
 * @file UnicyclePortSource.h
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef WALKINGCOORDINATORCOMPONENT_H
#define WALKINGCOORDINATORCOMPONENT_H

namespace WalkingControllers {
    class WalkingCoordinatorComponent;
}

#include <thrifts/WalkingCommands.h>
#include <yarp/os/RFModule.h>
#include <mutex>

class WalkingControllers::WalkingCoordinatorComponent : public WalkingCommands, public yarp::os::RFModule {
    std::mutex m_mutex;
protected:
    double m_periodInSeconds;

    virtual bool prepareWalkingPrivate() = 0;

    virtual bool stopWalkingPrivate() = 0;

    virtual bool pauseWalkingPrivate() = 0;

    virtual bool startWalkingPrivate() = 0;

    virtual bool onTheFlyStartWalkingPrivate(double smoothingTime=2.0) = 0;

    virtual bool configurePrivate(yarp::os::ResourceFinder &rf) = 0;

    virtual bool updatePrivate() = 0;

public:

    WalkingCoordinatorComponent();

    virtual ~WalkingCoordinatorComponent() override;

    // RFModule

    virtual bool configure(yarp::os::ResourceFinder &rf) override {
        std::lock_guard<std::mutex> guard(m_mutex);
        return configurePrivate(rf);
    }

    virtual double getPeriod() override {
        std::lock_guard<std::mutex> guard(m_mutex);
        return m_periodInSeconds;
    }

    virtual bool updateModule() override {
        std::lock_guard<std::mutex> guard(m_mutex);
        return updatePrivate();
    }

    // Cleanup method automatically called when stopping the module (like when calling quit)
    virtual bool close() override {
        return true;
    }

    //RPC methods

    /**
     * Method called from RPC
     */
    virtual void quit() override {
        std::lock_guard<std::mutex> guard(m_mutex);
        stopModule();
    }

    /** Reset trajectories and move to first position
     *
     * @note this function returns after the joints trajectory has been set and the robot has moved to the initial position.
     * @return true if the robot has reached the initial position, false otherwise
     */
    virtual bool prepareWalking() override {
        std::lock_guard<std::mutex> guard(m_mutex);
        return prepareWalkingPrivate();
    }

    /**
     * Immediately stop the joints trajectory playing
     * @return false in case of some problem. True otherwise
     */
    virtual bool stopWalking() override {
        std::lock_guard<std::mutex> guard(m_mutex);
        return stopWalkingPrivate();
    }

    /**
     * Pauses the robot motion in the current position.
     * @return alse in case of some problem. True otherwise.
    */
    virtual bool pauseWalking() override {
        std::lock_guard<std::mutex> guard(m_mutex);
        return pauseWalkingPrivate();
    }

    /**
     * Start the robot motion.
     * It can be called to start walking after having prepared or after pauseWalking()
     * @return false in case of some problem. True otherwise
    */
    virtual bool startWalking() override {
        std::lock_guard<std::mutex> guard(m_mutex);
        return startWalkingPrivate();
    }

    /**
     * Start the robot motion from a generic two-feet-standing position.
     * @return false in case of some problem. True otherwise
    */
    virtual bool onTheFlyStartWalking(double smoothingTime = 2.0) override {
        std::lock_guard<std::mutex> guard(m_mutex);
        return onTheFlyStartWalkingPrivate(smoothingTime);
    }

};



#endif // WALKINGCOORDINATORCOMPONENT_H
