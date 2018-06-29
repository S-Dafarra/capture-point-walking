/**
 * @file UnicyclePortSource.h
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef UNICYCLEPORTSOURCE_H
#define UNICYCLEPORTSOURCE_H

#include <UnicycleReferenceSource.h>
#include <memory>

namespace WalkingControllers {

    class UnicyclePortSource;

    enum class UnicyclePortMode {
        RelativePosition,
        RelativeVelocity,
        AbsolutePosition,
        AbsoluteVelocity
    };
}

class WalkingControllers::UnicyclePortSource : public WalkingControllers::UnicycleReferenceSource {

    class UnicyclePortSourceImplementation;
    std::unique_ptr<UnicyclePortSourceImplementation> m_pimpl;

public:

    UnicyclePortSource(const std::string& portName = "/unicyclePlanner/referencePosition:i", const WalkingControllers::UnicyclePortMode &mode = WalkingControllers::UnicyclePortMode::AbsolutePosition);

    virtual ~UnicyclePortSource() override;

    virtual bool getNewReference(const iDynTree::Vector2& currentReferencePosition, double unicycleAngle, double timeHorizon, iDynTree::Vector2 newReferencePosition) override;
};

#endif // UNICYCLEPORTSOURCE_H
