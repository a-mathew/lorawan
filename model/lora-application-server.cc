/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * LoRaWAN Application Server for ns-3.
 */

#include "lora-application-server.h"

#include "ns3/inet-socket-address.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/udp-socket-factory.h"

namespace ns3
{
namespace lorawan
{

NS_LOG_COMPONENT_DEFINE("LoraApplicationServer");

NS_OBJECT_ENSURE_REGISTERED(AsTransportHeader);
NS_OBJECT_ENSURE_REGISTERED(LoraApplicationServer);

TypeId
AsTransportHeader::GetTypeId()
{
    static TypeId tid = TypeId("ns3::lorawan::AsTransportHeader")
                            .SetParent<Header>()
                            .SetGroupName("lorawan")
                            .AddConstructor<AsTransportHeader>();
    return tid;
}

TypeId
AsTransportHeader::GetInstanceTypeId() const
{
    return GetTypeId();
}

uint32_t
AsTransportHeader::GetSerializedSize() const
{
    return 5; // 4 bytes device address + 1 byte flags
}

void
AsTransportHeader::Serialize(Buffer::Iterator start) const
{
    start.WriteHtonU32(m_deviceAddress);
    start.WriteU8(m_flags);
}

uint32_t
AsTransportHeader::Deserialize(Buffer::Iterator start)
{
    m_deviceAddress = start.ReadNtohU32();
    m_flags = start.ReadU8();
    return GetSerializedSize();
}

void
AsTransportHeader::Print(std::ostream& os) const
{
    os << "deviceAddress=" << m_deviceAddress << " flags=" << unsigned(m_flags);
}

void
AsTransportHeader::SetDeviceAddress(uint32_t address)
{
    m_deviceAddress = address;
}

uint32_t
AsTransportHeader::GetDeviceAddress() const
{
    return m_deviceAddress;
}

void
AsTransportHeader::SetConfirmed(bool confirmed)
{
    m_flags = confirmed ? (m_flags | 0x01) : (m_flags & ~0x01);
}

bool
AsTransportHeader::IsConfirmed() const
{
    return (m_flags & 0x01) != 0;
}

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

    if (m_useIpTransport)
    {
        // Socket receiving uplink datagrams from the Network Server
        m_uplinkSocket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        m_uplinkSocket->Bind(InetSocketAddress(Ipv4Address::GetAny(), m_uplinkPort));
        m_uplinkSocket->SetRecvCallback(
            MakeCallback(&LoraApplicationServer::HandleNsDatagram, this));

        // Socket towards the Network Server for downlink requests
        m_downlinkSocket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        m_downlinkSocket->Connect(InetSocketAddress(m_nsAddress, m_downlinkPort));

        NS_LOG_INFO("LoraApplicationServer transport: listening for uplinks on port "
                    << m_uplinkPort << ", downlinks to " << m_nsAddress << ":"
                    << m_downlinkPort);
    }

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
LoraApplicationServer::SetIpTransport(Ipv4Address nsAddress,
                                      uint16_t uplinkPort,
                                      uint16_t downlinkPort)
{
    NS_LOG_FUNCTION(this << nsAddress << uplinkPort << downlinkPort);
    m_nsAddress = nsAddress;
    m_uplinkPort = uplinkPort;
    m_downlinkPort = downlinkPort;
    m_useIpTransport = true;
}

void
LoraApplicationServer::HandleNsDatagram(Ptr<Socket> socket)
{
    NS_LOG_FUNCTION(this << socket);

    Ptr<Packet> datagram;
    while ((datagram = socket->Recv()))
    {
        if (datagram->GetSize() < AsTransportHeader().GetSerializedSize())
        {
            NS_LOG_WARN("Ignoring undersized datagram from the Network Server.");
            continue;
        }
        AsTransportHeader header;
        datagram->RemoveHeader(header);
        OnUplink(LoraDeviceAddress(header.GetDeviceAddress()), datagram);
    }
}

void
LoraApplicationServer::SendDownlink(LoraDeviceAddress deviceAddress,
                                    Ptr<Packet> payload,
                                    bool confirmed)
{
    NS_LOG_FUNCTION(this << deviceAddress << payload->GetSize() << confirmed);

    if (!m_useIpTransport && !m_networkServer)
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

    if (m_useIpTransport && m_downlinkSocket)
    {
        Ptr<Packet> datagram = payload->Copy();
        AsTransportHeader header;
        header.SetDeviceAddress(deviceAddress.Get());
        header.SetConfirmed(confirmed);
        datagram->AddHeader(header);
        m_downlinkSocket->Send(datagram);
    }
    else
    {
        m_networkServer->EnqueueDownlink(deviceAddress, payload, confirmed);
    }
}

Ptr<NetworkServer>
LoraApplicationServer::GetNetworkServer() const
{
    return m_networkServer;
}

} // namespace lorawan
} // namespace ns3