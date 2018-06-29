/**
 * @file UnicyclePortSource.cpp
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#include <UnicyclePortSource.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Bottle.h>
#include <cmath>
#include <atomic>
#include <thread>
#include <mutex>


using namespace WalkingControllers;

class UnicyclePortSource::UnicyclePortSourceImplementation {
public:
    std::string portName;
    UnicyclePortMode mode;

    yarp::os::BufferedPort<yarp::os::Bottle> port;

    std::atomic<bool> closing;
    bool neverSet = true;
    std::mutex mutex;
    std::thread portThread;

    iDynTree::Vector2 inputPoint;

    void readPortReference() {
        while (!closing){
            yarp::os::Bottle *input = port.read(false);

            if (input != nullptr){
                if (input->size() == 2){
                    std::lock_guard<std::mutex> guard(mutex);
                    if (!(input->get(0).isDouble()) && !(input->get(0).isInt())){
                        yError() << "[UnicyclePortSource] The the first input specified in the " << portName << " port is not numeric.";
                    } else if (!(input->get(1).isDouble()) && !(input->get(1).isInt())){
                        yError() << "[UnicyclePortSource] The the second input specified in the " << portName << " port is not numeric.";
                    } else {
                        inputPoint(0) = input->get(0).asDouble();
                        inputPoint(1) = input->get(1).asDouble();
                        neverSet = false;
                    }
                } else {
                    std::lock_guard<std::mutex> guard(mutex);
                    yError() << "[UnicyclePortSource] The input specified in the " << portName << " is supposed to have dimension 2.";
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    void startThread() {
        portThread = std::thread(&UnicyclePortSource::UnicyclePortSourceImplementation::readPortReference, this);
    }
};



UnicyclePortSource::UnicyclePortSource(const std::string &portName, const UnicyclePortMode &mode)
    : m_pimpl(new UnicyclePortSourceImplementation)
{
    m_pimpl->portName = portName;
    m_pimpl->mode = mode;
    m_pimpl->closing = false;
    m_pimpl->inputPoint.zero();

    if (!m_pimpl->port.open(m_pimpl->portName)){
        yError() << "[UnicyclePortSource::UnicyclePortSource] Failed to open the manual reference port.";
        m_pimpl->closing = true;
    } else {
        m_pimpl->startThread();
    }

}

UnicyclePortSource::~UnicyclePortSource()
{
    if (!(m_pimpl->closing)){
        m_pimpl->closing = true;
        if (m_pimpl->portThread.joinable()) {
            m_pimpl->portThread.join();
            m_pimpl->portThread = std::thread();
        }
        if (!m_pimpl->port.isClosed()){
            m_pimpl->port.interrupt();
            m_pimpl->port.close();
        }
    }
}

bool UnicyclePortSource::getNewReference(const iDynTree::Vector2 &currentReferencePosition, double unicycleAngle, double timeHorizon, iDynTree::Vector2 newReferencePosition)
{
    if (m_pimpl->closing) {
        yError() << "[UnicyclePortSource::getNewReference] Something went wrong when opening the input port.";
        return false;
    }

    std::lock_guard<std::mutex> guard(m_pimpl->mutex);

    if (m_pimpl->neverSet) {
        newReferencePosition = currentReferencePosition;
        return true;
    }

    if (m_pimpl->mode == UnicyclePortMode::AbsolutePosition) {
        newReferencePosition = m_pimpl->inputPoint;
    } else if (m_pimpl->mode == UnicyclePortMode::RelativePosition) {
        double s_theta = std::sin(unicycleAngle);
        double c_theta = std::cos(unicycleAngle);
        newReferencePosition(0) = c_theta * m_pimpl->inputPoint(0) - s_theta * m_pimpl->inputPoint(1) + currentReferencePosition(0);
        newReferencePosition(1) = s_theta * m_pimpl->inputPoint(0) + c_theta * m_pimpl->inputPoint(1) + currentReferencePosition(1);
    } else if (m_pimpl->mode == UnicyclePortMode::AbsoluteVelocity) {
        newReferencePosition(0) = m_pimpl->inputPoint(0) * timeHorizon + currentReferencePosition(0);
        newReferencePosition(1) = m_pimpl->inputPoint(1) * timeHorizon + currentReferencePosition(1);
    } else {//if (m_pimpl->mode == UnicyclePortMode::RelativeVelocity)
        double s_theta = std::sin(unicycleAngle);
        double c_theta = std::cos(unicycleAngle);
        newReferencePosition(0) = c_theta * m_pimpl->inputPoint(0) * timeHorizon - s_theta * m_pimpl->inputPoint(1) * timeHorizon + currentReferencePosition(0);
        newReferencePosition(1) = s_theta * m_pimpl->inputPoint(0) * timeHorizon + c_theta * m_pimpl->inputPoint(1) * timeHorizon + currentReferencePosition(1);
    }

    return true;
}

