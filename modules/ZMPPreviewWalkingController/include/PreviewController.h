/**
 * @file PreviewController.h
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef PREVIEWCONTROLLER_H
#define PREVIEWCONTROLLER_H

#include <iDynTree/OCSolvers/MultipleShootingSolver.h>
#include <iDynTree/Integrators/ImplicitTrapezoidal.h>
#include <iDynTree/OptimalControlProblem.h>
#include <iDynTree/ConstraintsGroup.h>
#include <iDynTree/LinearConstraint.h>
#include <iDynTree/LinearSystem.h>
#include <iDynTree/L2NormCost.h>
#include <iDynTree/TimeVaryingObject.h>
#include <iDynTree/TimeRange.h>
#include <iDynTree/Optimizers/OsqpInterface.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/ConvexHullHelpers.h>
#include <iDynTree/BoundingBoxHelpers.h>

#include <yarp/os/Searchable.h>
#include <memory>
#include <deque>

using namespace iDynTree::optimalcontrol;
using namespace iDynTree::optimalcontrol::integrators;
using namespace iDynTree::optimization;

namespace WalkingControllers {
    class PreviewController;

    enum class ConstraintMode{
        ConvexHull,
        BoundingBox,
        Hexagon,
        None
    };

    class ZMPReference;
}

typedef std::vector<std::pair<std::shared_ptr<LinearConstraint>, TimeRange>> ConvexHullConstraintsList;

class WalkingControllers::PreviewController {

    class ZMPReference;

    bool m_configured;
    double m_dT, m_horizon, m_CoMz, m_CoMzAcc;

    ConstraintMode m_constraintMode;
    iDynTree::Polygon m_hexagonalFoot;
    std::vector< iDynTree::Polygon > m_feetPolygons;
    bool m_updateMatrices;

    std::vector< iDynTree::Transform > m_feetTransforms;


    iDynTree::MatrixDynSize m_outputMatrix;

    std::shared_ptr<LinearSystem> m_model;
    std::shared_ptr<L2NormCost> m_cost;
    std::shared_ptr<OptimalControlProblem> m_problem;
    std::shared_ptr<ImplicitTrapezoidal> m_integrator;
    std::shared_ptr<MultipleShootingSolver> m_solver;

    std::vector<iDynTree::VectorDynSize> m_stateSolution, m_controlSolution;

    std::shared_ptr<ConstraintsGroup> m_group;
    ConvexHullConstraintsList m_convexHullDouble, m_convexHullLeft, m_convexHullRight;
    iDynTree::ConvexHullProjectionConstraint m_convexHullComputerDouble, m_convexHullComputerLeft, m_convexHullComputerRight;
    iDynTree::BoundingBoxProjectionConstraint m_boundingBoxComputerDouble, m_boundingBoxComputerLeft, m_boundingBoxComputerRight;
    iDynTree::MatrixDynSize m_convexHullDoubleMatrix, m_convexHullSingleMatrix;

    iDynTree::VectorDynSize m_feedback;

    std::shared_ptr<OsqpInterface> m_optimizer;
    std::shared_ptr<ZMPReference> m_zmpReference;

    bool configureConvexHullConstraints(const yarp::os::Searchable& mpcSettings);

    bool configureOptimizer(const yarp::os::Searchable& solverSettings);

    bool setConvexHullConstraintsDoubleSupport(size_t indexDouble, double initTime, double endTime,
                                               const std::vector<iDynTree::Transform> &feetTransforms);

    bool setConvexHullConstraintsSingleSupportLeft(size_t indexLeft, double initTime, double endTime, const iDynTree::Transform &leftTransform);

    bool setConvexHullConstraintsSingleSupportRight(size_t indexRight, double initTime, double endTime, const iDynTree::Transform &rightTransform);

    bool updateOutputMatrix();


public:

    PreviewController();

    ~PreviewController();

    bool configure(const yarp::os::Searchable& mpcSettings);

    bool setDesiredCoMHeight(double desiredCoMZ);

    bool setDesiredCoMVerticalAcceleration(double verticalAcceleration);

    bool useConvexHullConstraints();

    bool setConvexHullConstraint(const std::deque<iDynTree::Transform>& leftFoot,
                                 const std::deque<iDynTree::Transform>& rightFoot,
                                 const std::deque<bool>& leftInContact,
                                 const std::deque<bool>& rightInContact);

    bool resetProblem();

    bool solve(const std::deque<iDynTree::Vector2>& desiredZMP,
               const iDynTree::Position& CoM_feedBack,
               const iDynTree::Vector3& CoMVelocity_feedback,
               const iDynTree::Vector3& CoMAcceleration_feedback,
               iDynTree::Position& newCoMJerk);

};

#endif // PREVIEWCONTROLLER_H
