/**
 * @file YarpSigVectorPortSource.h
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#ifndef YARPSIGVECTORPORTSOURCE_H
#define YARPSIGVECTORPORTSOURCE_H

#include <yarp/sig/Vector.h>
#include <yarp/os/BufferedPort.h>
#include <iDynTree/Core/VectorDynSize.h>

#include <string>

namespace WalkingControllers {
    class YarpSigVectorPortSource;
}

class WalkingControllers::YarpSigVectorPortSource {

    yarp::os::BufferedPort<yarp::sig::Vector> m_localPort;
    std::string m_remotePortName;
    int m_bufferSize;
    bool m_configured;

public:

    YarpSigVectorPortSource();

    ~YarpSigVectorPortSource();

    bool configure(const std::string& portToConnect, const std::string& localPortName = "");

    bool read(iDynTree::VectorDynSize &output);

    bool blockingRead(iDynTree::VectorDynSize &output, double timeOutInSec = 0.0, double attemptsPeriodInSec = 0.001);

};


#endif // YARPSIGVECTORPORTSOURCE_H
