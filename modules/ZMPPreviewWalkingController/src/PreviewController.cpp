/**
 * @file PreviewController.h
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#include <PreviewController.h>
#include <yarp/os/LogStream.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/Direction.h>
#include <Utils.h>
#include <cmath>

using namespace WalkingControllers;

class PreviewController::ZMPReference : public TimeVaryingVector {

    const std::deque<iDynTree::Vector2>& m_desiredZMP;
    double m_dT;
    iDynTree::VectorDynSize output;
public:
    ZMPReference(const std::deque<iDynTree::Vector2>& desiredZMP, double dT)
        : m_desiredZMP(desiredZMP)
        , m_dT(dT)
        , output(2)
    { }

    virtual ~ZMPReference() override;

    virtual const iDynTree::VectorDynSize& get(double time, bool &isValid) override {
        size_t zero = 0;
        size_t maxIndex = std::max(zero, static_cast<size_t>(std::round(time/m_dT)));
        size_t index = std::min(m_desiredZMP.size(), maxIndex);

        output(0) = m_desiredZMP[index](0);
        output(1) = m_desiredZMP[index](1);

        isValid = true;

        return output;
    }
};
PreviewController::ZMPReference::~ZMPReference(){}


bool PreviewController::configureConvexHullConstraints(const yarp::os::Searchable &mpcSettings)
{
    yarp::os::Value feetDimensions = mpcSettings.find("foot_size");
    if (feetDimensions.isNull() || !feetDimensions.isList()) {
        std::cerr << "Unable to find feet dimensions in the configuration file. No convex hull constraint will be inserted." <<std::endl;
        m_constraintMode = ConstraintMode::None;
    } else {
        yarp::os::Bottle *feetDimensionsPointer = feetDimensions.asList();
        if (!feetDimensionsPointer || feetDimensionsPointer->size() != 2)
        {
            std::cerr << "Error while reading the feet dimensions. Wrong number of elements." <<std::endl;
            return false;
        }

        yarp::os::Value& xLimits = feetDimensionsPointer->get(0);
        if (xLimits.isNull() || !xLimits.isList())
        {
            std::cerr << "Error while reading the X limits." << std::endl;
            return false;
        }

        yarp::os::Bottle *xLimitsPtr = xLimits.asList();
        if (!xLimitsPtr || xLimitsPtr->size() != 2)
        {
            std::cerr << "Error while reading the X limits. Wrong dimensions." << std::endl;
            return false;
        }
        double xlimit1 = xLimitsPtr->get(0).asDouble();
        double xlimit2 = xLimitsPtr->get(1).asDouble();

        yarp::os::Value& yLimits = feetDimensionsPointer->get(1);
        if (yLimits.isNull() || !yLimits.isList())
        {
            std::cerr << "Error while reading the Y limits." << std::endl;

            return false;
        }

        yarp::os::Bottle *yLimitsPtr = yLimits.asList();
        if (!yLimitsPtr || yLimitsPtr->size() != 2)
        {
            std::cerr << "Error while reading the Y limits. Wrong dimensions." << std::endl;
            return false;
        }
        double ylimit1 = yLimitsPtr->get(0).asDouble();
        double ylimit2 = yLimitsPtr->get(1).asDouble();

        iDynTree::Polygon foot;

        foot = iDynTree::Polygon::XYRectangleFromOffsets(std::abs(std::max(xlimit1, xlimit2)), std::abs(std::min(xlimit1, xlimit2)),
                                                         std::abs(std::max(ylimit1, ylimit2)), std::abs(std::min(ylimit1, ylimit2)));

        m_feetPolygons.resize(2);
        m_feetPolygons[0] = foot;
        m_feetPolygons[1] = foot;

        std::string constraintMode = mpcSettings.check("constraints_mode", yarp::os::Value("convexHull")).asString();

        size_t numberOfConstraint;

        if (constraintMode == "convexHull"){
            m_constraintMode = ConstraintMode::ConvexHull;
            std::cerr << "[INFO] Using convex hull constraints." <<std::endl;
            numberOfConstraint = 6;
        } else if (constraintMode == "boundingBox"){
            m_constraintMode = ConstraintMode::BoundingBox;
            std::cerr << "[INFO] Using bounding box constraints." <<std::endl;
            numberOfConstraint = 4;
        } else if (constraintMode == "hexagon"){
            m_constraintMode = ConstraintMode::Hexagon;
            std::cerr << "[INFO] Using hexagon constraints for the single support phase." <<std::endl;
            numberOfConstraint = 6;
        } else {
            std::cerr << "Unrecognized constraint_mode. Available options: convexHull (default), boundingBox, hexagon." <<std::endl;
            return false;
        }

        if (m_constraintMode == ConstraintMode::Hexagon){
            m_hexagonalFoot.setNrOfVertices(6);
            double xM = std::max(xlimit1, xlimit2);
            double xm = std::min(xlimit1, xlimit2);
            double yM = std::max(ylimit1, ylimit2);
            double ym = std::min(ylimit1, ylimit2);
            m_hexagonalFoot.m_vertices[0] = iDynTree::Position(xM, yM, 0.0);
            m_hexagonalFoot.m_vertices[1] = iDynTree::Position(xm, yM, 0.0);
            m_hexagonalFoot.m_vertices[2] = iDynTree::Position(xm - 0.2 * (xM - xm), (ylimit1 + ylimit2)/2, 0.0);
            m_hexagonalFoot.m_vertices[3] = iDynTree::Position(xm, ym, 0.0);
            m_hexagonalFoot.m_vertices[4] = iDynTree::Position(xM, yM, 0.0);
            m_hexagonalFoot.m_vertices[5] = iDynTree::Position(xM + 0.2 * (xM - xm), (ylimit1 + ylimit2)/2, 0.0);
        }

        if (!m_group){
            m_group = std::make_shared<ConstraintsGroup>("ConvexHull", numberOfConstraint);
        }
    }
    return true;
}

bool PreviewController::configureOptimizer(const yarp::os::Searchable &solverSettings)
{
    m_optimizer = std::make_shared<OsqpInterface>();

    if (!solverSettings.isNull()) {
        OsqpSettings& settings = m_optimizer->settings();

        settings.rho                     = solverSettings.check("rho",                    yarp::os::Value(0.1)).asDouble();
        settings.sigma                   = solverSettings.check("sigma",                  yarp::os::Value(1e-06)).asDouble();
        settings.max_iter = static_cast<unsigned int>(solverSettings.check("max_iter",yarp::os::Value(4000)).asInt());
        settings.eps_abs                 = solverSettings.check("eps_abs",                yarp::os::Value(1e-03)).asDouble();
        settings.eps_rel                 = solverSettings.check("eps_rel",                yarp::os::Value(1e-03)).asDouble();
        settings.eps_prim_inf            = solverSettings.check("eps_prim_inf",           yarp::os::Value(1e-04)).asDouble();
        settings.eps_dual_inf            = solverSettings.check("eps_dual_inf",           yarp::os::Value(1e-04)).asDouble();
        settings.alpha                   = solverSettings.check("alpha",                  yarp::os::Value(1.6)).asDouble();
        settings.linsys_solver = static_cast<unsigned int>(solverSettings.check("linsys_solver", yarp::os::Value(0)).asInt());
        settings.delta                   = solverSettings.check("delta",                  yarp::os::Value(1e-06)).asDouble();
        settings.polish                  = solverSettings.check("polish",                 yarp::os::Value(false)).asBool();
        settings.polish_refine_iter = static_cast<unsigned int>(solverSettings.check("polish_refine_iter", yarp::os::Value(3)).asInt());
        settings.verbose                 = solverSettings.check("verbose",                yarp::os::Value(true)).asBool();
        settings.scaled_termination      = solverSettings.check("scaled_termination",     yarp::os::Value(false)).asBool();
        settings.check_termination       = static_cast<unsigned int>(solverSettings.check("check_termination", yarp::os::Value(25)).asInt());
        settings.warm_start              = solverSettings.check("warm_start",             yarp::os::Value(true)).asBool();
        settings.scaling = static_cast<unsigned int>(solverSettings.check("scaling",      yarp::os::Value(10)).asInt());
        settings.adaptive_rho            = solverSettings.check("adaptive_rho",           yarp::os::Value(true)).asBool();
        settings.adaptive_rho_interval = static_cast<unsigned int>(solverSettings.check("adaptive_rho_interval", yarp::os::Value(0)).asInt());
        settings.adaptive_rho_tolerance  = solverSettings.check("adaptive_rho_tolerance", yarp::os::Value(5)).asDouble();
        settings.adaptive_rho_fraction   = solverSettings.check("adaptive_rho_fraction",  yarp::os::Value(0.4)).asDouble();
        settings.time_limit              = solverSettings.check("time_limit",             yarp::os::Value(0)).asDouble();

    }

    return true;
}

bool PreviewController::setConvexHullConstraintsDoubleSupport(size_t indexDouble, double initTime, double endTime,
                                                              const std::vector<iDynTree::Transform>& feetTransforms)
{
    iDynTree::Direction xAxis, yAxis;
    xAxis.zero();
    xAxis(0) = 1;

    yAxis.zero();
    yAxis(1) = 1;

    iDynTree::Position planeOrigin;
    planeOrigin.zero();

    size_t numberOfDoubleConstraints;
    if (m_constraintMode == ConstraintMode::ConvexHull) {
        numberOfDoubleConstraints = 6;
    } else if (m_constraintMode == ConstraintMode::BoundingBox) {
        numberOfDoubleConstraints = 4;
    } else { //if (m_constraintMode == ConstraintMode::Hexagon){
        numberOfDoubleConstraints = 6;
    }

    bool toBeAdded = false;
    if(indexDouble >= m_convexHullDouble.size()){
        m_convexHullDouble.resize(indexDouble+1);
        toBeAdded = true;
    }

    m_convexHullDouble[indexDouble].first = std::make_shared<LinearConstraint>(numberOfDoubleConstraints,
                                                                               "Double" + std::to_string(indexDouble));
    m_convexHullDouble[indexDouble].second.setTimeInterval(initTime, endTime);

    if ((m_constraintMode == ConstraintMode::ConvexHull) || (m_constraintMode == ConstraintMode::Hexagon)){

        m_convexHullComputerDouble.buildConvexHull(xAxis, yAxis, planeOrigin, m_feetPolygons, feetTransforms);

        m_convexHullDoubleMatrix.resize(m_convexHullComputerDouble.A.rows(), m_outputMatrix.cols());
        iDynTree::toEigen(m_convexHullDoubleMatrix) = iDynTree::toEigen(m_convexHullComputerDouble.A)*iDynTree::toEigen(m_outputMatrix);

        if (!m_convexHullDouble[indexDouble].first->setStateConstraintMatrix(m_convexHullDoubleMatrix)) {
            yError() << "[PreviewController::setConvexHullConstraint] Failed to set constraint matrix (double).";
            return false;
        }

        if (!m_convexHullDouble[indexDouble].first->setUpperBound(m_convexHullComputerDouble.b)) {
            yError() << "[PreviewController::setConvexHullConstraint] Failed to set upper bounds (double).";
            return false;
        }
    } else if (m_constraintMode == ConstraintMode::BoundingBox){

        m_boundingBoxComputerDouble.buildBoundingBox(xAxis, yAxis, planeOrigin, m_feetPolygons, feetTransforms);

        m_convexHullDoubleMatrix.resize(m_boundingBoxComputerDouble.A.rows(), m_outputMatrix.cols());
        iDynTree::toEigen(m_convexHullDoubleMatrix) = iDynTree::toEigen(m_boundingBoxComputerDouble.A)*iDynTree::toEigen(m_outputMatrix);

        if (!m_convexHullDouble[indexDouble].first->setStateConstraintMatrix(m_convexHullDoubleMatrix)) {
            yError() << "[PreviewController::setConvexHullConstraint] Failed to set constraint matrix (double).";
            return false;
        }

        if (!m_convexHullDouble[indexDouble].first->setUpperBound(m_boundingBoxComputerDouble.b)) {
            yError() << "[PreviewController::setConvexHullConstraint] Failed to set upper bounds (double).";
            return false;
        }
    }

    if (toBeAdded) {
        if (!(m_group->addConstraint(m_convexHullDouble[indexDouble].first, m_convexHullDouble[indexDouble].second))) {
            yError() << "[PreviewController::setConvexHullConstraint] Failed to add constraint (double).";
            return false;
        }
    } else {
        if (!(m_group->updateTimeRange(m_convexHullDouble[indexDouble].first->name(), m_convexHullDouble[indexDouble].second))) {
            yError() << "[PreviewController::setConvexHullConstraint] Failed to update constraint (double).";
            return false;
        }
    }

    return true;
}

bool PreviewController::setConvexHullConstraintsSingleSupportLeft(size_t indexLeft, double initTime, double endTime,
                                                                  const iDynTree::Transform& leftTransform)
{
    iDynTree::Direction xAxis, yAxis;
    xAxis.zero();
    xAxis(0) = 1;

    yAxis.zero();
    yAxis(1) = 1;

    iDynTree::Position planeOrigin;
    planeOrigin.zero();

    size_t numberOfSingleConstraints;
    if (m_constraintMode == ConstraintMode::ConvexHull) {
        numberOfSingleConstraints = 4;
    } else if (m_constraintMode == ConstraintMode::BoundingBox) {
        numberOfSingleConstraints = 4;
    } else { //if (m_constraintMode == ConstraintMode::Hexagon){
        numberOfSingleConstraints = 6;
    }

    bool toBeAdded = false;
    if(indexLeft >= m_convexHullLeft.size()){
        m_convexHullLeft.resize(indexLeft+1);
        toBeAdded = true;
    }

    m_convexHullLeft[indexLeft].first = std::make_shared<LinearConstraint>(numberOfSingleConstraints,
                                                                           "Left" + std::to_string(indexLeft));
    m_convexHullLeft[indexLeft].second.setTimeInterval(initTime, endTime);


    if ((m_constraintMode == ConstraintMode::ConvexHull) || (m_constraintMode == ConstraintMode::Hexagon)){
        if (m_constraintMode == ConstraintMode::Hexagon){
            m_convexHullComputerLeft.buildConvexHull(xAxis, yAxis, planeOrigin,
                                                 std::vector<iDynTree::Polygon>(1, m_hexagonalFoot),
                                                 std::vector<iDynTree::Transform>(1, leftTransform));
        } else {
            m_convexHullComputerLeft.buildConvexHull(xAxis, yAxis, planeOrigin,
                                                     std::vector<iDynTree::Polygon>(1, m_feetPolygons[0]),
                                                     std::vector<iDynTree::Transform>(1, leftTransform));
        }

        m_convexHullSingleMatrix.resize(m_convexHullComputerLeft.A.rows(), m_outputMatrix.cols());
        iDynTree::toEigen(m_convexHullSingleMatrix) = iDynTree::toEigen(m_convexHullComputerLeft.A)*iDynTree::toEigen(m_outputMatrix);

        if (!m_convexHullLeft[indexLeft].first->setStateConstraintMatrix(m_convexHullSingleMatrix)) {
            yError() << "[PreviewController::setConvexHullConstraint] Failed to set constraint matrix (left).";
            return false;
        }

        if (!m_convexHullLeft[indexLeft].first->setUpperBound(m_convexHullComputerLeft.b)) {
            yError() << "[PreviewController::setConvexHullConstraint] Failed to set upper bounds (left).";
            return false;
        }
    }
    if (m_constraintMode == ConstraintMode::BoundingBox){

        m_boundingBoxComputerLeft.buildBoundingBox(xAxis, yAxis, planeOrigin,
                                                   std::vector<iDynTree::Polygon>(1, m_feetPolygons[0]),
                std::vector<iDynTree::Transform>(1, leftTransform));

        m_convexHullSingleMatrix.resize(m_boundingBoxComputerLeft.A.rows(), m_outputMatrix.cols());
        iDynTree::toEigen(m_convexHullSingleMatrix) = iDynTree::toEigen(m_boundingBoxComputerLeft.A)*iDynTree::toEigen(m_outputMatrix);

        if (!m_convexHullLeft[indexLeft].first->setStateConstraintMatrix(m_convexHullSingleMatrix)) {
            yError() << "[PreviewController::setConvexHullConstraint] Failed to set constraint matrix (left).";
            return false;
        }

        if (!m_convexHullLeft[indexLeft].first->setUpperBound(m_boundingBoxComputerLeft.b)) {
            yError() << "[PreviewController::setConvexHullConstraint] Failed to set upper bounds (left).";
            return false;
        }
    }

    if (toBeAdded) {
        if (!(m_group->addConstraint(m_convexHullLeft[indexLeft].first, m_convexHullLeft[indexLeft].second))) {
            yError() << "[PreviewController::setConvexHullConstraint] Failed to add constraint (left).";
            return false;
        }
    } else {
        if (!(m_group->updateTimeRange(m_convexHullLeft[indexLeft].first->name(), m_convexHullLeft[indexLeft].second))) {
            yError() << "[PreviewController::setConvexHullConstraint] Failed to update constraint (left).";
            return false;
        }
    }

    return true;
}

bool PreviewController::setConvexHullConstraintsSingleSupportRight(size_t indexRight, double initTime, double endTime,
                                                                   const iDynTree::Transform &rightTransform)
{
    iDynTree::Direction xAxis, yAxis;
    xAxis.zero();
    xAxis(0) = 1;

    yAxis.zero();
    yAxis(1) = 1;

    iDynTree::Position planeOrigin;
    planeOrigin.zero();

    size_t numberOfSingleConstraints;
    if (m_constraintMode == ConstraintMode::ConvexHull) {
        numberOfSingleConstraints = 4;
    } else if (m_constraintMode == ConstraintMode::BoundingBox) {
        numberOfSingleConstraints = 4;
    } else { //if (m_constraintMode == ConstraintMode::Hexagon){
        numberOfSingleConstraints = 6;
    }

    bool toBeAdded = false;
    if(indexRight >= m_convexHullRight.size()){
        m_convexHullRight.resize(indexRight+1);
        toBeAdded = true;
    }

    m_convexHullRight[indexRight].first = std::make_shared<LinearConstraint>(numberOfSingleConstraints,
                                                                             "Right" + std::to_string(indexRight));
    m_convexHullRight[indexRight].second.setTimeInterval(initTime, endTime);


    if ((m_constraintMode == ConstraintMode::ConvexHull) || (m_constraintMode == ConstraintMode::Hexagon)){
        if (m_constraintMode == ConstraintMode::Hexagon){
            m_convexHullComputerRight.buildConvexHull(xAxis, yAxis, planeOrigin,
                                                 std::vector<iDynTree::Polygon>(1, m_hexagonalFoot),
                                                 std::vector<iDynTree::Transform>(1, rightTransform));
        } else {
            m_convexHullComputerRight.buildConvexHull(xAxis, yAxis, planeOrigin,
                                                     std::vector<iDynTree::Polygon>(1, m_feetPolygons[1]),
                                                     std::vector<iDynTree::Transform>(1, rightTransform));
        }

        m_convexHullSingleMatrix.resize(m_convexHullComputerRight.A.rows(), m_outputMatrix.cols());
        iDynTree::toEigen(m_convexHullSingleMatrix) = iDynTree::toEigen(m_convexHullComputerRight.A)*iDynTree::toEigen(m_outputMatrix);

        if (!m_convexHullRight[indexRight].first->setStateConstraintMatrix(m_convexHullSingleMatrix)) {
            yError() << "[PreviewController::setConvexHullConstraint] Failed to set constraint matrix (right).";
            return false;
        }

        if (!m_convexHullRight[indexRight].first->setUpperBound(m_convexHullComputerRight.b)) {
            yError() << "[PreviewController::setConvexHullConstraint] Failed to set upper bounds (right).";
            return false;
        }
    }
    if (m_constraintMode == ConstraintMode::BoundingBox){

        m_boundingBoxComputerRight.buildBoundingBox(xAxis, yAxis, planeOrigin,
                                                    std::vector<iDynTree::Polygon>(1, m_feetPolygons[1]),
                                                    std::vector<iDynTree::Transform>(1, rightTransform));

        m_convexHullSingleMatrix.resize(m_convexHullComputerRight.A.rows(), m_outputMatrix.cols());
        iDynTree::toEigen(m_convexHullSingleMatrix) = iDynTree::toEigen(m_convexHullComputerRight.A)*iDynTree::toEigen(m_outputMatrix);

        if (!m_convexHullRight[indexRight].first->setStateConstraintMatrix(m_convexHullSingleMatrix)) {
            yError() << "[PreviewController::setConvexHullConstraint] Failed to set constraint matrix (right).";
            return false;
        }

        if (!m_convexHullRight[indexRight].first->setUpperBound(m_boundingBoxComputerRight.b)) {
            yError() << "[PreviewController::setConvexHullConstraint] Failed to set upper bounds (right).";
            return false;
        }
    }

    if (toBeAdded) {
        if (!(m_group->addConstraint(m_convexHullRight[indexRight].first, m_convexHullRight[indexRight].second))) {
            yError() << "[PreviewController::setConvexHullConstraint] Failed to add constraint (right).";
            return false;
        }
    } else {
        if (!(m_group->updateTimeRange(m_convexHullRight[indexRight].first->name(), m_convexHullRight[indexRight].second))) {
            yError() << "[PreviewController::setConvexHullConstraint] Failed to update constraint (right).";
            return false;
        }
    }

    return true;
}

bool PreviewController::updateOutputMatrix()
{
    double omegaSquaredInv = m_CoMz/m_CoMzAcc;

    m_outputMatrix(0, 4) =  -omegaSquaredInv;
    m_outputMatrix(1, 5) =  -omegaSquaredInv;

    if (!(m_cost->updatStateSelector(m_outputMatrix))) {
        yError() << "[PreviewController::updateOutputMatrix] Failed to update selector matrix.";
        return false;
    }
    return true;
}

PreviewController::PreviewController()
    : m_configured(false)
    , m_feedback(6)
{

}

PreviewController::~PreviewController()
{

}

bool PreviewController::configure(const yarp::os::Searchable &mpcSettings)
{
    if (mpcSettings.isNull()) {
        yError() << "[PreviewController::configure] Empty configuration.";
        return false;
    }

    m_CoMz = 0.5; //initial value
    m_CoMzAcc = 9.81; //initial value

    double omegaSquaredInv = m_CoMz/m_CoMzAcc;

    m_model = std::make_shared<LinearSystem>(6, 2);

    iDynTree::MatrixDynSize stateMatrix, controlMatrix;

    stateMatrix.zero();
    stateMatrix(0,2) = 1.0;
    stateMatrix(1,3) = 1.0;
    stateMatrix(2,4) = 1.0;
    stateMatrix(3,5) = 1.0;

    if (!m_model->setStateMatrix(stateMatrix)) {
        yError() << "[PreviewController::configure] Failed to set state matrix.";
        return false;
    }

    controlMatrix.zero();
    controlMatrix(4,0) = 1.0;
    controlMatrix(5,1) = 1.0;

    if (!m_model->setControlMatrix(controlMatrix)) {
        yError() << "[PreviewController::configure] Failed to set control matrix.";
        return false;
    }

    iDynTree::MatrixDynSize controlSelector;
    controlSelector.resize(2,2);
    iDynTree::toEigen(controlSelector).setIdentity();

    m_outputMatrix.resize(2,6);
    m_outputMatrix.zero();
    m_outputMatrix(0, 0) = 1.0;
    m_outputMatrix(1, 1) = 1.0;
    m_outputMatrix(0, 4) = -omegaSquaredInv;
    m_outputMatrix(1, 5) = -omegaSquaredInv;

    m_cost = std::make_shared<L2NormCost>("zmpCost", m_outputMatrix, controlSelector);

    iDynTree::VectorDynSize jerkWeightVector, zmpWeightVector;

    yarp::os::Value tempValue = mpcSettings.find("CoMJerk_Weight");
    if (!YarpHelper::yarpListToiDynTreeVectorDynSize(tempValue, jerkWeightVector)) {
        yError() << "[PreviewController::configure] Initialization failed while reading CoMJerk_Weight vector.";
        return false;
    }

    if (jerkWeightVector.size() != 2) {
        yError() << "[PreviewController::configure] The CoMJerk_Weight vector is expected to have dimension 2.";
        return false;
    }

    tempValue = mpcSettings.find("ZMP_Weight");
    if (!YarpHelper::yarpListToiDynTreeVectorDynSize(tempValue, zmpWeightVector)) {
        std::cerr << "Initialization failed while reading ZMP_Weight vector." << std::endl;
        return false;
    }

    if (zmpWeightVector.size() != 2) {
        yError() << "[PreviewController::configure] The ZMP_Weight vector is expected to have dimension 2.";
        return false;
    }

    iDynTree::MatrixDynSize jerkWeight, zmpWeight;

    jerkWeight.resize(2,2);
    jerkWeight(0,0) = jerkWeightVector(0);
    jerkWeight(1,1) = jerkWeightVector(1);

    zmpWeight.resize(2,2);
    zmpWeight(0,0) = zmpWeightVector(0);
    zmpWeight(1,1) = zmpWeightVector(1);

    if (!m_cost->setStateWeight(zmpWeight)) {
        yError() << "[PreviewController::configure] Failed to set the zmpWeight to the cost.";
        return false;
    }

    if (!m_cost->setControlWeight(jerkWeight)) {
        yError() << "[PreviewController::configure] Failed to set the jerkWeight to the cost.";
        return false;
    }

    if (!YarpHelper::getDoubleFromSearchable(mpcSettings, "dT", m_dT))
        return false;

    if (!YarpHelper::getDoubleFromSearchable(mpcSettings, "horizon", m_horizon))
        return false;

    m_problem = std::make_shared<OptimalControlProblem>();

    if (!m_problem->setDynamicalSystemConstraint(m_model)) {
        yError() << "[PreviewController::configure] Unable to set dynamical system to the problem.";
        return false;
    }

    if (!m_problem->addLagrangeTerm(1.0, m_cost)) {
        yError() << "[PreviewController::configure] Unable to set cost to the problem.";
        return false;
    }

    if (!m_problem->setTimeHorizon(0.0, m_horizon)) {
        yError() << "[PreviewController::configure] Unable to set timings to the problem.";
        return false;
    }

    if (!configureConvexHullConstraints(mpcSettings)) {
        return false;
    }

    if (!configureOptimizer(mpcSettings.findGroup("OSQP"))) {
        return false;
    }

    m_integrator = std::make_shared<ImplicitTrapezoidal>();

    bool ok = true;
    m_solver = std::make_shared<MultipleShootingSolver>(m_problem);
    ok = ok && m_solver->setControlPeriod(m_dT);
    ok = ok && m_solver->setStepSizeBounds(m_dT, 3*m_dT);
    ok = ok && m_solver->setIntegrator(m_integrator);
    ok = ok && m_solver->setOptimizer(m_optimizer);

    if (!ok) {
        yError() << "[PreviewController::configure] Failed while preparing MultipleShootingSolver.";
        return false;
    }

    m_updateMatrices = false;

    m_configured = true;

    return true;
}

bool PreviewController::setDesiredCoMHeight(double desiredCoMZ)
{
    if (desiredCoMZ < 0) {
        yError() << "[PreviewController::setDesiredCoMHeight] The desired CoM height should be positive.";
        return false;
    }
    m_CoMz = desiredCoMZ;
    m_updateMatrices = true;

    return true;
}

bool PreviewController::setDesiredCoMVerticalAcceleration(double verticalAcceleration)
{
    if(std::abs(verticalAcceleration) < 0.01){
        yError() << "[PreviewController::setDesiredCoMVerticalAcceleration] The modulus of the vertical acceleration is too little.";
        return false;
    }

    m_CoMzAcc = verticalAcceleration;
    m_updateMatrices = true;

    return true;
}

bool PreviewController::useConvexHullConstraints()
{
    return (m_group != nullptr);

}

bool PreviewController::setConvexHullConstraint(const std::deque<iDynTree::Transform> &leftFoot,
                                                const std::deque<iDynTree::Transform> &rightFoot,
                                                const std::deque<bool> &leftInContact,
                                                const std::deque<bool> &rightInContact)
{
    if((m_horizon <= 0)||(m_dT <= 0)){
        yError() << "[PreviewController::setConvexHullConstraint] No timing settings loaded.";
        return false;
    }

    if(m_feetPolygons.size() != 2){
        yError() << "[PreviewController::setConvexHullConstraint] No feet limits were loaded.";
        return false;
    }

    if((leftFoot.size() == 0)||(rightFoot.size() == 0)||(leftInContact.size() == 0)||(rightInContact.size() == 0)){
        yError() << "[PreviewController::setConvexHullConstraint] Empty input.";
        return false;
    }

    if(leftFoot.size() != rightFoot.size()){
        yError() << "[PreviewController::setConvexHullConstraint] Inconsinstent dimensions of the feet trajectories.";
        return false;
    }

    if(leftInContact.size() != rightInContact.size()){
        yError() << "[PreviewController::setConvexHullConstraint] Inconsinstent dimensions of the feet contact sequence.";
        return false;
    }

    if(leftInContact.size() != leftFoot.size()){
        yError() << "[PreviewController::setConvexHullConstraint] Inconsinstent dimensions between the feet contact sequence and the feet trajectories.";
        return false;
    }

    if(m_outputMatrix.rows() == 0){
        yError() << "[PreviewController::setConvexHullConstraint] Model not computed yet.";
        return false;
    }

    size_t indexDouble = 0, indexLeft = 0, indexRight = 0;
    double t = 0.0;

    unsigned int step = 0;
    do{
        step = static_cast<unsigned int>(std::round(t/m_dT));

        if(leftInContact[step] && rightInContact[step]){ //double support
            double initTime = t;
            double endTime = m_horizon + m_dT;
            while((t < m_horizon) && (step < leftInContact.size()) && leftInContact[step] && rightInContact[step]){
                t += m_dT;
                step++;
            }

            if(step >= leftInContact.size()){
                endTime = m_horizon + m_dT;
                m_feetTransforms[0] = leftFoot.back();
                m_feetTransforms[1] = rightFoot.back();
                t = m_horizon;
            }
            else {
                endTime = t;
                m_feetTransforms[0] = leftFoot[step];
                m_feetTransforms[1] = rightFoot[step];
            }

            if (!setConvexHullConstraintsDoubleSupport(indexDouble, initTime, endTime, m_feetTransforms))
                return false;

            indexDouble++;
        }
        else if(leftInContact[step]){
            double initTime = t;
            double endTime = m_horizon + m_dT;
            while((t < m_horizon) && (step < leftInContact.size()) && leftInContact[step] && !rightInContact[step]){
                t += m_dT;
                step++;
            }
            if(step >= leftInContact.size()){
                endTime = m_horizon + m_dT;
                m_feetTransforms[0] = leftFoot.back();
                t = m_horizon;
            }
            else {
                endTime = t;
                m_feetTransforms[0] = leftFoot[step];
            }

            if (!setConvexHullConstraintsSingleSupportLeft(indexLeft, initTime, endTime, m_feetTransforms[0])) {
                return false;
            }

            indexLeft++;
        }
        else if(rightInContact[step]){
            double initTime = t;
            double endTime = m_horizon + m_dT;
            while((t < m_horizon) && (step < rightInContact.size()) && !leftInContact[step] && rightInContact[step]){
                t += m_dT;
                step++;
            }
            if(step >= rightInContact.size()){
                endTime = m_horizon + m_dT;
                m_feetTransforms[1] = rightFoot.back();
                t = m_horizon;
            }
            else {
                endTime = t;
                m_feetTransforms[1] = rightFoot[step];
            }

            if (!setConvexHullConstraintsSingleSupportRight(indexRight, initTime, endTime, m_feetTransforms[1]))
                return false;

            indexRight++;
        }
        else{
            yError() << "[PreviewController::setConvexHullConstraint] Is the robot jumping already?";
            return false;
        }
    }while(t < std::min(m_horizon, leftInContact.size() * m_dT));

    //disabling leftover constraints

    for(size_t i = indexDouble; i < m_convexHullDouble.size(); ++i){
        if (!(m_convexHullDouble[i].second.setTimeInterval(m_horizon + m_dT, m_horizon + 2*m_dT))) {
            yError() << "[PreviewController::setConvexHullConstraint] Failed to set fake time-range (double).";
            return false;
        }

        if (!(m_group->updateTimeRange(m_convexHullDouble[i].first->name(), m_convexHullDouble[i].second))) {
            yError() << "[PreviewController::setConvexHullConstraint] Failed to deactivate constraint (double).";
            return false;
        }
    }
    for(size_t i = indexLeft; i < m_convexHullLeft.size(); ++i){

        if (!(m_convexHullLeft[i].second.setTimeInterval(m_horizon + m_dT, m_horizon + 2*m_dT))) {
            yError() << "[PreviewController::setConvexHullConstraint] Failed to set fake time-range (left).";
            return false;
        }

        if (!(m_group->updateTimeRange(m_convexHullLeft[i].first->name(), m_convexHullLeft[i].second))) {
            yError() << "[PreviewController::setConvexHullConstraint] Failed to deactivate constraint (left).";
            return false;
        }

    }
    for(size_t i = indexRight; i < m_convexHullRight.size(); ++i){
        if (!(m_convexHullRight[i].second.setTimeInterval(m_horizon + m_dT, m_horizon + 2*m_dT))) {
            yError() << "[PreviewController::setConvexHullConstraint] Failed to set fake time-range (right).";
            return false;
        }

        if (!(m_group->updateTimeRange(m_convexHullRight[i].first->name(), m_convexHullRight[i].second))) {
            yError() << "[PreviewController::setConvexHullConstraint] Failed to deactivate constraint (right).";
            return false;
        }
    }

    return true;
}

bool PreviewController::resetProblem()
{
    for (auto constraint : m_convexHullDouble) {
        m_group->removeConstraint(constraint.first->name());
    }
    m_convexHullDouble.clear();

    for (auto constraint : m_convexHullLeft) {
        m_group->removeConstraint(constraint.first->name());
    }
    m_convexHullLeft.clear();

    for (auto constraint : m_convexHullRight) {
        m_group->removeConstraint(constraint.first->name());
    }
    m_convexHullRight.clear();
    return true;
}

bool PreviewController::solve(const std::deque<iDynTree::Vector2> &desiredZMP,
                              const iDynTree::Position &CoM_feedBack,
                              const iDynTree::Vector3 &CoMVelocity_feedback,
                              const iDynTree::Vector3 &CoMAcceleration_feedback,
                              iDynTree::Position &newCoMJerk)
{
    if (!m_configured) {
        yError() << "[PreviewController::solve] First you have to call the configure method.";
        return false;
    }

    if (m_updateMatrices) {
        if (!updateOutputMatrix()) {
            return false;
        }
        m_updateMatrices = false;
    }

    if (desiredZMP.size() == 0) {
        yError() << "[PreviewController::solve] The ZMP trajectory is null.";
        return false;
    }

    m_zmpReference = std::make_shared<ZMPReference>(desiredZMP, m_dT);

    if (!(m_cost->setStateDesiredTrajectory(m_zmpReference))) {
        yError() << "[PreviewController::solve] Error while setting the desired reference trajectory.";
        return false;
    }

    m_feedback(0) = CoM_feedBack(0);
    m_feedback(1) = CoM_feedBack(1);
    m_feedback(2) = CoMVelocity_feedback(0);
    m_feedback(3) = CoMVelocity_feedback(1);
    m_feedback(4) = CoMAcceleration_feedback(0);
    m_feedback(5) = CoMAcceleration_feedback(1);

    if (!m_solver->setInitialState(m_feedback)) {
        yError() << "[PreviewController::solve] Error while setting initial state.";
        return false;
    }

    if (!m_solver->solve()) {
        yError() << "[PreviewController::solve] Optimization problem failed.";
        return false;
    }

    if (!m_solver->getSolution(m_stateSolution, m_controlSolution)) {
        yError() << "[PreviewController::solve] Failed to retrieve solution.";
        return false;
    }

    newCoMJerk(0) = m_controlSolution.front()(0);
    newCoMJerk(1) = m_controlSolution.front()(1);
    newCoMJerk(2) = 0.0;

    return true;
}

