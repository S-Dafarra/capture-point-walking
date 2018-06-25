/**
 * @file YarpSigVectorPortSink.cpp
 * @authors Stefano Dafarra <stefano.dafarra@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#include <YarpSigVectorPortSink.h>
#include <yarp/os/LogStream.h>
#include <iDynTree/yarp/YARPConversions.h>

using namespace WalkingControllers;

YarpSigVectorPortSink::YarpSigVectorPortSink()
    : m_configured(false)
{ }

YarpSigVectorPortSink::~YarpSigVectorPortSink()
{
    if (!m_outputPort.isClosed()){
        m_outputPort.interrupt();
        m_outputPort.close();
    }
}

bool YarpSigVectorPortSink::configure(const std::string &portName)
{
    if (m_configured) {
        yError() << "[YarpSigVectorPortSink::configure] Cannot configure twice.";
        return false;
    }

    if(!m_outputPort.open(portName)){
        yError() << "[YarpSigVectorPortSink::configure] Failed to open sink port named "<< portName;
        return false;
    }

    m_outputPort.setOutputMode(true);

    return true;
}

bool YarpSigVectorPortSink::write(const iDynTree::VectorDynSize &output)
{
    if (m_configured) {
        yError() << "[YarpSigVectorPortSink::write] Not configured yet.";
        return false;
    }

    yarp::sig::Vector& outputBuffer = m_outputPort.prepare();

    outputBuffer.resize(output.size());

    iDynTree::toYarp(output, outputBuffer);

    m_outputPort.write();

    return true;
}
