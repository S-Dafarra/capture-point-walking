/**
 * @file UnicycleReferenceSource.h
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef UNICYCLEREFERENCESOURCE_H
#define UNICYCLEREFERENCESOURCE_H

#include <iDynTree/Core/VectorFixSize.h>

namespace WalkingControllers {
    class UnicycleReferenceSource;
}

class WalkingControllers::UnicycleReferenceSource {

public:

    UnicycleReferenceSource();

    virtual ~UnicycleReferenceSource();

    virtual bool getNewReference(const iDynTree::Vector2& currentReferencePosition, double unicycleAngle, iDynTree::Vector2 newReferencePosition) = 0;
};

#endif // UNICYCLEREFERENCESOURCE_H
