/**
 * @file UnicycleGeneratorComponent.h
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef UNICYCLEGENERATORCOMPONENT_H
#define UNICYCLEGENERATORCOMPONENT_H


#include <UnicycleGenerator.h>
#include <UnicycleReferenceSource.h>
#include "iDynTree/Core/VectorFixSize.h"
#include "yarp/os/BufferedPort.h"
#include "yarp/os/Bottle.h"
#include <memory>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <atomic>

namespace WalkingControllers {

    class UnicycleGeneratorComponent;
}

class WalkingControllers::UnicycleGeneratorComponent
{
    UnicycleGenerator m_generator; //this object should not be called if the planner is computing
    std::shared_ptr<FeetCubicSplineGenerator> m_feetGenerator;
    std::shared_ptr<CoMHeightTrajectoryGenerator> m_heightGenerator;
    std::shared_ptr<ZMPTrajectoryGenerator> m_zmpGenerator;

    //Planner Configuration variables
    iDynTree::Vector2 m_referencePointDistance;
    iDynTree::Vector2 m_rZMPStancePosition, m_lZMPStancePosition, m_rZMPInitialSwitchPosition, m_lZMPInitialSwitchPosition;
    double m_dT, m_unicycleGain, m_slowWhenTurningGain;
    double m_maxStepLength, m_minStepLength, m_minWidth, m_maxAngleVariation, m_minAngleVariation;
    double m_nominalWidth, m_maxStepDuration, m_minStepDuration, m_nominalDuration, m_timeWeight, m_positionWeight;
    double m_lastStepSwitchTime, m_switchOverSwingRatio;
    double m_stepHeight, m_landingVelocity, m_apexTime, m_comHeight, m_comHeightDelta;
    bool m_swingLeft, m_forceStart;
    double m_plannerHorizon;

    bool m_correctLeft;
    iDynTree::Transform m_measuredTransformLeft;

    bool m_correctRight;
    iDynTree::Transform m_measuredTransformRight;

    double m_initTime;

    iDynTree::Vector2 m_desiredPoint; //to be obtained by the joystick

    std::mutex m_mutex;
    std::condition_variable m_conditionVariable;
    std::thread m_generatorThread;

    bool m_useFeetGeneration, m_useZMPGeneration, m_useHeightGeneration;

    std::shared_ptr<WalkingControllers::UnicycleReferenceSource> m_inputReference;

    enum class GeneratorState {
        NotConfigured,
        Configured,
        Called,
        Returned,
        Closing
    };

    GeneratorState m_generatorState;

    bool readPlannerConfigurationFile(yarp::os::Searchable& config);
    bool configurePlanner();
    void computeThread();
    bool privateUpdateTrajectories(double initTime, bool correctLeft, bool correctRight, bool newStanceFootIsLeft, const iDynTree::Transform &measuredLeft, const iDynTree::Transform &measuredRight);


public:
    UnicycleGeneratorComponent();

    ~UnicycleGeneratorComponent();

    /*@ Initialize the pattern generator
     * @param config the options for the pattern generator
     * @return true on success, false otherwise
     */
    bool configure(yarp::os::Searchable& config, std::shared_ptr<WalkingControllers::UnicycleReferenceSource> referenceSource);

    bool generateFirstTrajectories(); //this call is not in a separate thread

    bool generateFirstTrajectories(const iDynTree::Transform &leftToRightTransform, const iDynTree::Position &initialCOMPositionInLeft); //this call is not in a separate thread

    bool updateTrajectories(double initTime, const InitialState &initialWeightInLeftFoot, bool correctLeft, const iDynTree::Transform &measured);

    bool updateTrajectories(double initTime, bool correctLeft, const iDynTree::Transform &measured);

    bool updateTrajectories(double initTime, const InitialState &initialWeightInLeftFoot, bool newStanceFootIsLeft, const iDynTree::Transform &measuredLeft, const iDynTree::Transform &measuredRight);

    bool updateTrajectories(double initTime, bool newStanceFootIsLeft, const iDynTree::Transform &measuredLeft, const iDynTree::Transform &measuredRight);

    bool trajectoryComputed() const;

    //Getters

    bool getFeetTrajectories(std::vector<iDynTree::Transform>& lFootTrajectory, std::vector<iDynTree::Transform>& rFootTrajectory) const;

    bool getWeightPercentage(std::vector<double>& weightInLeft, std::vector<double>& weightInRight) const;

    bool getZMPTrajectory(std::vector<iDynTree::Vector2>& ZMPTrajectory) const;

    bool getZMPTrajectory(std::vector<iDynTree::Vector2> &ZMPTrajectory, std::vector<iDynTree::Vector2> &ZMPVelocity,
                          std::vector<iDynTree::Vector2> &ZMPAcceleration) const;

    bool getLocalZMPTrajectories(std::vector<iDynTree::Vector2 >& leftZMPTrajectory, std::vector< iDynTree::Vector2 >& rightZMPTrajectory) const;

    bool getCoMHeightTrajectory(std::vector<double>& CoMHeightTrajectory) const;

    bool getCoMHeightVelocity(std::vector<double> &CoMHeightVelocity) const;

    bool getCoMHeightAccelerationProfile(std::vector<double >& CoMHeightAccelerationProfile) const;

    bool getFeetStandingPeriods(std::vector <bool>& lFootContacts, std::vector < bool >& rFootContacts) const;

    bool getWhenUseLeftAsFixed(std::vector <bool>& leftIsFixed) const;

    bool getInitialStatesAtMergePoints(std::vector<InitialState>& initialStates) const;

    bool getMergePoints(std::vector<size_t>& mergePoints) const; //indexes in which is suitable to perform a merge of trajectories

};

#endif // UNICYCLEGENERATORCOMPONENT_H
