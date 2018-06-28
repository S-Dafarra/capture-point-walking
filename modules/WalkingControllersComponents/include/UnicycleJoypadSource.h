/**
 * @file UnicycleJoypadSource.h
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef UNICYCLEJOYPADSOURCE_H
#define UNICYCLEJOYPADSOURCE_H

#include <UnicycleReferenceSource.h>
#include <yarp/os/Searchable.h>
#include <memory>

namespace WalkingControllers {
    class UnicycleJoypadSource;
}

class WalkingControllers::UnicycleJoypadSource : public WalkingControllers::UnicycleReferenceSource {

    class UnicycleJoypadSourceImplementation;
    std::unique_ptr<UnicycleJoypadSourceImplementation> m_pimpl;

public:

    UnicycleJoypadSource();

    virtual ~UnicycleJoypadSource() override;

    bool configure(const yarp::os::Searchable& joypadSettings);

    virtual bool getNewReference(const iDynTree::Vector2& currentReferencePosition, double unicycleAngle, iDynTree::Vector2 newReferencePosition) override;
};


#endif // UNICYCLEJOYPADSOURCE_H
