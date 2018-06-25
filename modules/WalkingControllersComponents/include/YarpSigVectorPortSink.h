/**
 * @file YarpSigVectorPortSink.h
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef YARPSIGVECTORPORTSINK_H
#define YARPSIGVECTORPORTSINK_H

#include <yarp/sig/Vector.h>
#include <yarp/os/BufferedPort.h>
#include <iDynTree/Core/VectorDynSize.h>

#include <string>

namespace WalkingControllers {
    class YarpSigVectorPortSink;
}

class WalkingControllers::YarpSigVectorPortSink {

    yarp::os::BufferedPort<yarp::sig::Vector> m_outputPort;
    bool m_configured;

public:

    YarpSigVectorPortSink();

    ~YarpSigVectorPortSink();

    bool configure(const std::string& portName);

    bool write(const iDynTree::VectorDynSize &output);

};

#endif // YARPSIGVECTORPORTSINK_H
