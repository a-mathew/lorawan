/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * LoRaWAN Application Server for ns-3.
 */

#include "lora-application-server.h"

#include "ns3/log.h"
#include "ns3/simulator.h"

namespace ns3
{
namespace lorawan
{

NS_LOG_COMPONENT_DEFINE("LoraApplicationServer");

NS_OBJECT_ENSURE_REGISTERED(LoraApplicationServer);

TypeId
LoraApplicationServer::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::LoraApplicationServer")
            .SetParent<Application>()
            .AddConstructor<LoraApplicationServer>()
            .AddTraceSource(
                "ReceivedUplink",
                "Trace source fired when an uplink application payload arrives at the AS",
                MakeTraceSourceAccessor(&LoraApplicationServer::m_receivedUplink),
                "ns3::Packet::TracedCallback")
            .AddTraceSource(
                "SentDownlink",
                "Trace source fired when a downlink payload is sent from the AS to the NS",
                MakeTraceSourceAccessor(&LoraApplicationServer::m_sentDownlink),
                "ns3::Packet::TracedCallback")
            .SetGroupName("lorawan");
    return tid;
}

LoraApplicationServer::LoraApplicationServer()
    : m_uplinkCount(0),
      m_downlinkCount(0)
{
    NS_LOG_FUNCTION(this);
}

LoraApplicationServer::~LoraApplicationServer()
{
    NS_LOG_FUNCTION(this);
}

void
LoraApplicationServer::StartApplication()
{
    NS_LOG_FUNCTION(this);
    NS_LOG_INFO("LoraApplicationServer started with "
                << m_handlers.size() << " registered handler(s)");
}

void
LoraApplicationServer::StopApplication()
{
    NS_LOG_FUNCTION(this);
    NS_LOG_INFO("LoraApplicationServer stopped. Stats: uplinks="
                << m_uplinkCount << " downlinks=" << m_downlinkCount);
}

void
LoraApplicationServer::SetNetworkServer(Ptr<NetworkServer> ns)
{
    NS_LOG_FUNCTION(this << ns);
    m_networkServer = ns;

    // Register our OnUplink as the NS's uplink forward callback
    ns->SetUplinkForwardCallback(
        MakeCallback(&LoraApplicationServer::OnUplink, this));

    NS_LOG_INFO("LoraApplicationServer connected to NetworkServer");
}

void
LoraApplicationServer::RegisterHandler(const std::string& name, AppUplinkHandler handler)
{
    NS_LOG_FUNCTION(this << name);
    m_handlers.push_back(std::make_pair(name, handler));
    NS_LOG_INFO("Registered application handler: " << name);
}

void
LoraApplicationServer::OnUplink(LoraDeviceAddress deviceAddress, Ptr<Packet> payload)
{
    NS_LOG_FUNCTION(this << deviceAddress << payload->GetSize());

    m_uplinkCount++;

    NS_LOG_INFO("AS received uplink #" << m_uplinkCount
                << " from device " << deviceAddress
                << " (" << payload->GetSize() << " bytes)");

    // Fire trace source
    m_receivedUplink(payload);

    // Dispatch to all registered handlers
    for (auto& entry : m_handlers)
    {
        NS_LOG_DEBUG("Dispatching uplink to handler: " << entry.first);
        entry.second(deviceAddress, payload->Copy());
    }
}

void
LoraApplicationServer::SendDownlink(LoraDeviceAddress deviceAddress, Ptr<Packet> payload)
{
    NS_LOG_FUNCTION(this << deviceAddress << payload->GetSize());

    if (!m_networkServer)
    {
        NS_LOG_ERROR("SendDownlink: no NetworkServer connected!");
        return;
    }

    m_downlinkCount++;

    NS_LOG_INFO("AS sending downlink #" << m_downlinkCount
                << " to device " << deviceAddress
                << " (" << payload->GetSize() << " bytes)");

    // Fire trace source
    m_sentDownlink(payload);

    m_networkServer->EnqueueDownlink(deviceAddress, payload);
}

Ptr<NetworkServer>
LoraApplicationServer::GetNetworkServer() const
{
    return m_networkServer;
}

} // namespace lorawan
} // namespace ns3