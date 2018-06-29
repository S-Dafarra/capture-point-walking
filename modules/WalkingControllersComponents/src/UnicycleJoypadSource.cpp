/**
 * @file UnicycleJoypadSource.cpp
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#include <UnicycleJoypadSource.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IJoypadController.h>
#include <Utils.h>
#include <string>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <cmath>
#include <iDynTree/Core/VectorFixSize.h>

using namespace WalkingControllers;

class UnicycleJoypadSource::UnicycleJoypadSourceImplementation {

    double deadzoneFunction(double input, double deadZone, double fullScale)
    {
        if ((deadZone < 0)||(deadZone >= std::abs(fullScale)))
            return input;

        if (input >= 0){
            if (input > deadZone)
                return (input - deadZone)/(std::abs(fullScale) - deadZone);
            else return 0.0;
        } else {
            if (input < -deadZone)
                return (input + deadZone)/(std::abs(fullScale) - deadZone);
            else return 0.0;
        }
    }

public:
    double period = 0.001;
    std::string joystick, device, local, remote;
    bool referenceInLocalFrame;
    double maxVelX, maxVelY, deadzone;
    iDynTree::Vector2 xRemapping, yRemapping;
    std::atomic<bool> configured;
    yarp::dev::PolyDriver joypadDriver;
    yarp::dev::IJoypadController* joypadInterface;

    std::mutex mutex;
    std::thread joypadThread;

    iDynTree::Vector2 referenceVelocity;

    void readJoypadReference() {
        double x_tmpIn = 0.0, y_tmpIn = 0.0, deadzoneIn, velXTemp, velYTemp;
        iDynTree::Vector2 xRemappingTemp, yRemappingTemp, newDesired;
        bool ok = true;

        while (configured) {
            {
                std::lock_guard<std::mutex>(this->mutex);

                ok = ok && joypadInterface->getAxis(0, x_tmpIn);
                ok = ok && joypadInterface->getAxis(1, y_tmpIn);

                if (!ok) {
                    std::lock_guard<std::mutex>(this->mutex);
                    yWarning() << "[UnicycleJoypadSource] Failed to read from joypad.";
                }

                deadzoneIn = deadzone;
                xRemappingTemp = xRemapping;
                yRemappingTemp = yRemapping;
                velXTemp = maxVelX;
                velYTemp = maxVelY;
            }

            double x_tmp = deadzoneFunction(x_tmpIn, deadzoneIn, 1.0);
            double y_tmp = deadzoneFunction(y_tmpIn, deadzoneIn, 1.0);

            newDesired(0) =(xRemappingTemp(0)*x_tmp + xRemappingTemp(1)*y_tmp) * velXTemp;
            newDesired(1) =(yRemappingTemp(0)*x_tmp + yRemappingTemp(1)*y_tmp) * velYTemp;

            {
                std::lock_guard<std::mutex>(this->mutex);
                referenceVelocity = newDesired;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<unsigned long>(period * 1000)));
        }
    }

    void startThread() {
        joypadThread = std::thread(&UnicycleJoypadSourceImplementation::readJoypadReference, this);
    }

};

UnicycleJoypadSource::UnicycleJoypadSource()
    : m_pimpl(new UnicycleJoypadSourceImplementation)
{
    m_pimpl->configured = true;
}

UnicycleJoypadSource::~UnicycleJoypadSource()
{
    if (m_pimpl->configured) {
        m_pimpl->configured = true;
        if (m_pimpl->joypadThread.joinable()) {
            m_pimpl->joypadThread.join();
            m_pimpl->joypadThread = std::thread();
        }
    }

    if (m_pimpl->joypadDriver.isValid()){
        m_pimpl->joypadDriver.close();
    }
}

bool UnicycleJoypadSource::configure(const yarp::os::Searchable &joypadSettings)
{
    {
        std::lock_guard<std::mutex>(m_pimpl->mutex);

        if (m_pimpl->configured) {
            yError() << "[UnicycleJoypadSource::configure] Cannot configure twice.";
            return false;
        }

        if (joypadSettings.isNull()) {
            yError() << "[UnicycleJoypadSource::configure] Empty configuration.";
            return false;
        }

        bool ok = true;

        ok = ok && YarpHelper::getDoubleFromSearchable(joypadSettings, "period", m_pimpl->period);

        if (m_pimpl->period <= 0) {
            yError() << "[UnicycleJoypadSource::configure] The period is supposed to be a strictly positive number.";
            return false;
        }

        ok = ok && YarpHelper::getStringFromSearchable(joypadSettings, "joystick", m_pimpl->joystick);
        ok = ok && YarpHelper::getStringFromSearchable(joypadSettings, "device", m_pimpl->device);
        ok = ok && YarpHelper::getStringFromSearchable(joypadSettings, "local", m_pimpl->local);
        ok = ok && YarpHelper::getStringFromSearchable(joypadSettings, "remote", m_pimpl->remote);
        ok = ok && YarpHelper::getDoubleFromSearchable(joypadSettings, "maxVelx", m_pimpl->maxVelX);
        ok = ok && YarpHelper::getDoubleFromSearchable(joypadSettings, "maxVely", m_pimpl->maxVelY);
        ok = ok && YarpHelper::getDoubleFromSearchable(joypadSettings, "deadzone", m_pimpl->deadzone);
        ok = ok && YarpHelper::getBoolFromSearchable(joypadSettings, "useUnicycleFrame", m_pimpl->referenceInLocalFrame);

        if (!ok) {
            return false;
        }

        yarp::os::Value xRemapping = joypadSettings.find("xRemapping");

        if (!YarpHelper::yarpListToiDynTreeVectorFixSize(xRemapping, m_pimpl->xRemapping)) {
            yError() << "[UnicycleJoypadSource::configure] Fail to read xRemapping from configuration file.";
            return false;
        }

        yarp::os::Value yRemapping = joypadSettings.find("yRemapping");

        if (!YarpHelper::yarpListToiDynTreeVectorFixSize(xRemapping, m_pimpl->yRemapping)) {
            yError() << "[UnicycleJoypadSource::configure] Fail to read yRemapping from configuration file.";
            return false;
        }

        yarp::os::Property configuration;
        configuration.put("device", m_pimpl->device);
        configuration.put("local", m_pimpl->local);
        std::string remote = m_pimpl->remote + "/" + m_pimpl->joystick;
        configuration.put("remote", remote);

        if (!(m_pimpl->joypadDriver.open(configuration))) {
            yError() << "[UnicycleJoypadSource::configure] Failed to open joypad driver.";
            return false;
        }

        if (!(m_pimpl->joypadDriver.view(m_pimpl->joypadInterface))) {
            yError() << "[UnicycleJoypadSource::configure] Unable to attach JoypadController interface to the PolyDriver object";
            return false;
        }

        m_pimpl->referenceVelocity.zero();

        m_pimpl->configured = true;
    }

    m_pimpl->startThread();

    return true;
}

bool UnicycleJoypadSource::getNewReference(const iDynTree::Vector2 &currentReferencePosition, double unicycleAngle, double timeHorizon, iDynTree::Vector2 newReferencePosition)
{
    if (!(m_pimpl->configured)) {
        std::lock_guard<std::mutex>(m_pimpl->mutex);
        yError() << "[UnicycleJoypadSource::getNewReference] Not configured.";
        return false;
    }

    std::lock_guard<std::mutex>(m_pimpl->mutex);

    if (m_pimpl->referenceInLocalFrame) {
        double s_theta = std::sin(unicycleAngle);
        double c_theta = std::cos(unicycleAngle);
        newReferencePosition(0) = c_theta * m_pimpl->referenceVelocity(0) * timeHorizon - s_theta * m_pimpl->referenceVelocity(1) * timeHorizon + currentReferencePosition(0);
        newReferencePosition(1) = s_theta * m_pimpl->referenceVelocity(0) * timeHorizon + c_theta * m_pimpl->referenceVelocity(1) * timeHorizon + currentReferencePosition(1);
    } else {
        newReferencePosition(0) = m_pimpl->referenceVelocity(0) * timeHorizon + currentReferencePosition(0);
        newReferencePosition(1) = m_pimpl->referenceVelocity(1) * timeHorizon + currentReferencePosition(1);
    }

    return true;
}
