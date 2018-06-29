/**
 * @file UnicyclePortSource.cpp
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#include <WalkingCoordinatorComponent.h>

using namespace WalkingControllers;



WalkingCoordinatorComponent::WalkingCoordinatorComponent()
    : m_periodInSeconds(0.01)
{ }

WalkingCoordinatorComponent::~WalkingCoordinatorComponent()
{
    std::lock_guard<std::mutex> guard(m_mutex);
    stopModule();
}
