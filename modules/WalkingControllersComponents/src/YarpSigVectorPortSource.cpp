/**
 * @file YarpSigVectorPortSource.cpp
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#include <YarpSigVectorPortSource.h>
#include <yarp/os/LogStream.h>
#include <iDynTree/yarp/YARPConversions.h>

using namespace WalkingControllers;

YarpSigVectorPortSource::YarpSigVectorPortSource()
    : m_bufferSize(0)
    , m_configured(false)
{ }

YarpSigVectorPortSource::~YarpSigVectorPortSource()
{
    if (!(m_localPort.isClosed())) {
        yarp::os::Network::disconnect(m_remotePortName, m_localPort.getName());
        m_localPort.close();
    }
}

bool YarpSigVectorPortSource::configure(const std::string &localPortName, const std::string &portToConnect)
{
    if (m_configured) {
        yError() << "[YarpSigVectorPortSink::configure] Cannot configure twice.";
        return false;
    }

    if (localPortName.size() > 0) {
        if (!m_localPort.open(localPortName)) {
            yError() << "[YarpSigVectorPortSource::configure] Failed to open local port with name " << localPortName << ".";
            return false;
        }
    } else if (portToConnect.size() > 0) {
        if (!m_localPort.open(portToConnect + ":local")) {
            yError() << "[YarpSigVectorPortSource::configure] Failed to open local port with name " << portToConnect << ":local.";
            return false;
        }
    } else {
        yError() << "[YarpSigVectorPortSource::configure] At least one port name must be specified.";
        return false;
    }

    if (portToConnect.size() > 0) {
        if(!yarp::os::Network::connect(portToConnect, localPortName)){
            yError() << "[YarpSigVectorPortSource::configure] Unable to connect " << localPortName << " to " << portToConnect << ".";
            return false;
        }
    }

    m_configured = true;
    return true;
}

bool YarpSigVectorPortSource::read(iDynTree::VectorDynSize &output)
{
    if (!m_configured) {
        yError() << "[YarpSigVectorPortSource::read] Not configured yet.";
        return false;
    }

    yarp::sig::Vector* input = m_localPort.read(false);

    if (input == nullptr)
        return false;

    output.resize(static_cast<unsigned int>(input->size()));

    if (!iDynTree::toiDynTree(*input, output)) {
        yError() << "[YarpSigVectorPortSource::read] Failed to convert yarp vector.";
        return false;
    }

    m_bufferSize = m_localPort.getPendingReads();

    return true;
}

bool YarpSigVectorPortSource::blockingRead(iDynTree::VectorDynSize &output, double timeOutInSec, double attemptsPeriodInSec)
{
    if (!m_configured) {
        yError() << "[YarpSigVectorPortSource::blockingRead] Not configured yet.";
        return false;
    }

    yarp::sig::Vector* input = nullptr;
    bool read = false;

    if (timeOutInSec > 0) {
        // Initialize the time counter for the timeout
        const double t0 = yarp::os::SystemClock::nowSystem();

        // Loop until something has been read or timeout is reached
        while (!read) {
            const int new_bufferSize = m_localPort.getPendingReads();

            if (new_bufferSize > m_bufferSize) {
                input = m_localPort.read(false);
            }

            if (input == nullptr) {
                yarp::os::Time::delay(attemptsPeriodInSec);
                const double now = yarp::os::Time::now();
                if ((now - t0) > timeOutInSec) {
                    yError() << "[YarpSigVectorPortSource::blockingRead] The port didn't receive any data for longer than " << timeOutInSec << " seconds.";
                    return false;
                }
            } else {
                read = true;
            }
        }
    }
    else {
        input = m_localPort.read(true);
    }

    if (input == nullptr)
        return false;

    output.resize(static_cast<unsigned int>(input->size()));

    if (!iDynTree::toiDynTree(*input, output)) {
        yError() << "[YarpSigVectorPortSource::blockingRead] Failed to convert yarp vector.";
        return false;
    }

    m_bufferSize = m_localPort.getPendingReads();

    return true;
}


