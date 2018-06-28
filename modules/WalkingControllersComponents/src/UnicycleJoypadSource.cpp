/**
 * @file UnicycleJoypadSource.cpp
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#include <UnicycleJoypadSource.h>
#include <yarp/os/LogStream.h>
#include <Utils.h>
#include <string>

using namespace WalkingControllers;

class UnicycleJoypadSource::UnicycleJoypadSourceImplementation {
public:
    double period = 0.001;
    std::string joystick, device, local, remote;
    int stickId = 0;
    double maxVelX, maxVelY, deadzone;
};

UnicycleJoypadSource::UnicycleJoypadSource()
    : m_pimpl(new UnicycleJoypadSourceImplementation)
{ }

UnicycleJoypadSource::~UnicycleJoypadSource()
{ }

bool UnicycleJoypadSource::configure(const yarp::os::Searchable &joypadSettings)
{
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
    ok = ok && YarpHelper::getIntFromSearchable(joypadSettings, "stickid", m_pimpl->stickId);
    ok = ok && YarpHelper::getStringFromSearchable(joypadSettings, "device", m_pimpl->device);
    ok = ok && YarpHelper::getStringFromSearchable(joypadSettings, "local", m_pimpl->local);
    ok = ok && YarpHelper::getStringFromSearchable(joypadSettings, "remote", m_pimpl->remote);
    ok = ok && YarpHelper::getDoubleFromSearchable(joypadSettings, "maxVelx", m_pimpl->maxVelX);
    ok = ok && YarpHelper::getDoubleFromSearchable(joypadSettings, "maxVely", m_pimpl->maxVelY);
    ok = ok && YarpHelper::getDoubleFromSearchable(joypadSettings, "deadzone", m_pimpl->deadzone);

    //I should read also the remapping


    return true;
}
