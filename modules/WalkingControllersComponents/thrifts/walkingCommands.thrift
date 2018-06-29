/******************************************************************************
 *                                                                            *
 * Copyright (C) 2018 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @file walkingCommands.h
 * @authors: Stefano Dafarra <stefano.dafarra@iit.it>
 */

service WalkingCommands {

    /**
     * Quits the module.
     */
    oneway void quit();

    /** Reset trajectories and move to first position
     *
     * @note this function returns after the joints trajectory has been set and the robot has moved to the initial position.
     * @return true if the robot has reached the initial position, false otherwise
     */
    bool prepareWalking();

    /**
     * Immediately stop the joints trajectory playing
     * @return false in case of some problem. True otherwise
     */
    bool stopWalking();

    /**
     * Pauses the robot motion in the current position.
     * @return alse in case of some problem. True otherwise.
    */
    bool pauseWalking();

    /**
     * Start the robot motion.
     * It can be called to start walking after having prepared or after pauseWalking()
     * @return false in case of some problem. True otherwise
    */
    bool startWalking();

    /**
     * Start the robot motion from a generic two-feet-standing position.
     * @return false in case of some problem. True otherwise
    */
    bool onTheFlyStartWalking(1:double smoothingTime=2.0);
}
