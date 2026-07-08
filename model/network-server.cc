/*
 * Copyright (c) 2018 University of Padova
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Authors: Davide Magrin <magrinda@dei.unipd.it>
 *          Martina Capuzzo <capuzzom@dei.unipd.it>
 *
 * Modified: Added Application Server interface (uplink forwarding,
 *           downlink enqueueing) for Class C support.
 */

#include "network-server.h"

#include "class-a-end-device-lorawan-mac.h"
#include "gateway-lorawan-mac.h"
#include "lora-application-server.h"
#include "lora-device-address.h"
#include "lora-frame-header.h"
#include "lora-net-device.h"
#include "lora-tag.h"
#include "lorawan-mac-header.h"
#include "mac-command.h"
#include "network-controller-components.h"
#include "network-controller.h"
#include "network-scheduler.h"
#include "network-status.h"

#include "ns3/inet-socket-address.h"
#include "ns3/net-device.h"
#include "ns3/node-container.h"
#include "ns3/packet.h"
#include "ns3/point-to-point-net-device.h"
#include "ns3/simulator.h"
#include "ns3/udp-socket-factory.h"

namespace ns3
{
namespace lorawan
{

namespace
{
/// Retries before a downlink is dropped when no gateway can transmit.
constexpr uint8_t MAX_DOWNLINK_RETRIES = 10;
/// Delay between no-gateway retries. Sized so that the total retry budget
/// covers the off-time of a full-length SF12 downlink on a 10% duty-cycle
/// sub-band (~16-18 s).
constexpr uint32_t DOWNLINK_RETRY_DELAY_MS = 2000;
/// Quiet period after an uplink during which no spontaneous Class C downlink
/// is started: RECEIVE_DELAY2 (2 s) plus the RX2 window duration and margin.
/// RX1/RX2 preempt an in-progress RXC demodulation (LoRaWAN 1.0.4 §15), so a
/// downlink overlapping this region would be aborted by the device.
constexpr double CLASS_A_QUIET_PERIOD_S = 2.5;
/// Time the server waits for the acknowledgement of a confirmed Class C
/// downlink before releasing the per-device slot (LoRaWAN 1.0.4 §15,
/// CLASS_C_RESP_TIMEOUT default).
constexpr double CLASS_C_RESP_TIMEOUT_S = 8.0;
/// Recheck period for downlinks held back by protocol state (pending
/// RXParamSetupAns or an outstanding confirmed downlink).
constexpr uint32_t DOWNLINK_HOLD_RECHECK_MS = 2000;
} // namespace

NS_LOG_COMPONENT_DEFINE("NetworkServer");

NS_OBJECT_ENSURE_REGISTERED(NetworkServer);

TypeId
NetworkServer::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::NetworkServer")
            .SetParent<Application>()
            .AddConstructor<NetworkServer>()
            .AddTraceSource(
                "ReceivedPacket",
                "Trace source that is fired when a packet arrives at the network server",
                MakeTraceSourceAccessor(&NetworkServer::m_receivedPacket),
                "ns3::Packet::TracedCallback")
            .AddTraceSource(
                "ForwardedToAS",
                "Trace source fired when an uplink payload is forwarded to the Application Server",
                MakeTraceSourceAccessor(&NetworkServer::m_forwardedToAS),
                "ns3::Packet::TracedCallback")
            .AddTraceSource(
                "SentDownlink",
                "Trace source fired when a downlink from EnqueueDownlink is sent to a gateway",
                MakeTraceSourceAccessor(&NetworkServer::m_sentDownlink),
                "ns3::Packet::TracedCallback")
            .SetGroupName("lorawan");
    return tid;
}

NetworkServer::NetworkServer()
    : m_status(CreateObject<NetworkStatus>()),
      m_controller(CreateObject<NetworkController>(m_status)),
      m_scheduler(CreateObject<NetworkScheduler>(m_status, m_controller))
{
    NS_LOG_FUNCTION_NOARGS();
}

NetworkServer::~NetworkServer()
{
    NS_LOG_FUNCTION_NOARGS();
}

void
NetworkServer::StartApplication()
{
    NS_LOG_FUNCTION_NOARGS();

    if (m_useIpTransport)
    {
        // Socket towards the Application Server for uplink datagrams
        m_asUplinkSocket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        m_asUplinkSocket->Connect(InetSocketAddress(m_asAddress, m_asUplinkPort));

        // Socket receiving downlink requests from the Application Server
        m_asDownlinkSocket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        m_asDownlinkSocket->Bind(InetSocketAddress(Ipv4Address::GetAny(), m_asDownlinkPort));
        m_asDownlinkSocket->SetRecvCallback(MakeCallback(&NetworkServer::HandleAsDatagram, this));

        NS_LOG_INFO("NetworkServer AS transport: uplinks to "
                    << m_asAddress << ":" << m_asUplinkPort << ", listening for downlinks on port "
                    << m_asDownlinkPort);
    }
}

void
NetworkServer::StopApplication()
{
    NS_LOG_FUNCTION_NOARGS();
}

void
NetworkServer::AddGateway(Ptr<Node> gateway, Ptr<NetDevice> netDevice)
{
    NS_LOG_FUNCTION(this << gateway);

    // Get the PointToPointNetDevice
    Ptr<PointToPointNetDevice> p2pNetDevice;
    for (uint32_t i = 0; i < gateway->GetNDevices(); i++)
    {
        p2pNetDevice = DynamicCast<PointToPointNetDevice>(gateway->GetDevice(i));
        if (p2pNetDevice)
        {
            // We found a p2pNetDevice on the gateway
            break;
        }
    }

    // Get the gateway's LoRa MAC layer (assumes gateway's MAC is configured as first device)
    Ptr<GatewayLorawanMac> gwMac =
        DynamicCast<GatewayLorawanMac>(DynamicCast<LoraNetDevice>(gateway->GetDevice(0))->GetMac());
    NS_ASSERT(gwMac);

    // Get the Address
    Address gatewayAddress = p2pNetDevice->GetAddress();

    // Create new gatewayStatus
    Ptr<GatewayStatus> gwStatus = CreateObject<GatewayStatus>(gatewayAddress, netDevice, gwMac);

    m_status->AddGateway(gatewayAddress, gwStatus);
}

void
NetworkServer::AddNodes(NodeContainer nodes)
{
    NS_LOG_FUNCTION_NOARGS();

    // For each node in the container, call the function to add that single node
    NodeContainer::Iterator it;
    for (it = nodes.Begin(); it != nodes.End(); it++)
    {
        AddNode(*it);
    }
}

void
NetworkServer::AddNode(Ptr<Node> node)
{
    NS_LOG_FUNCTION(this << node);

    // Get the LoraNetDevice
    Ptr<LoraNetDevice> loraNetDevice;
    for (uint32_t i = 0; i < node->GetNDevices(); i++)
    {
        loraNetDevice = DynamicCast<LoraNetDevice>(node->GetDevice(i));
        if (loraNetDevice)
        {
            // We found a LoraNetDevice on the node
            break;
        }
    }

    // Get the MAC
    Ptr<ClassAEndDeviceLorawanMac> edLorawanMac =
        DynamicCast<ClassAEndDeviceLorawanMac>(loraNetDevice->GetMac());

    // Update the NetworkStatus about the existence of this node
    m_status->AddNode(edLorawanMac);
}

bool
NetworkServer::Receive(Ptr<NetDevice> device,
                       Ptr<const Packet> packet,
                       uint16_t protocol,
                       const Address& address)
{
    NS_LOG_FUNCTION(this << packet << protocol << address);

    // Create a copy of the packet
    Ptr<Packet> myPacket = packet->Copy();

    // Fire the trace source
    m_receivedPacket(packet);

    // Inform the scheduler of the newly arrived packet
    m_scheduler->OnReceivedPacket(packet);

    // Inform the status of the newly arrived packet
    m_status->OnReceivedPacket(packet, address);

    // Inform the controller of the newly arrived packet
    m_controller->OnNewPacket(packet);

    // ---- Application Server interface: bookkeeping and uplink forwarding ----
    {
        // Strip headers to extract the application payload
        Ptr<Packet> payloadCopy = packet->Copy();
        LorawanMacHeader macHdr;
        payloadCopy->RemoveHeader(macHdr);
        LoraFrameHeader frameHdr;
        frameHdr.SetAsUplink();
        payloadCopy->RemoveHeader(frameHdr);

        LoraDeviceAddress deviceAddr = frameHdr.GetAddress();

        // Remember when this device last talked to us: spontaneous Class C
        // downlinks must stay clear of its RX1/RX2 window region.
        m_lastUplinkTime[deviceAddr] = Simulator::Now();

        // Any uplink from the device releases the per-device
        // confirmed-downlink slot (LoRaWAN 1.0.4 Section 15: the server
        // waits "until the response timeout expires or an uplink is
        // received").
        auto pending = m_confirmedDlPending.find(deviceAddr);
        if (pending != m_confirmedDlPending.end())
        {
            NS_LOG_INFO("Confirmed downlink to "
                        << deviceAddr
                        << (frameHdr.GetAck() ? " acknowledged."
                                              : " released by an uplink without ACK."));
            Simulator::Cancel(pending->second);
            m_confirmedDlPending.erase(pending);
        }

        // An RXParamSetupAns lifts the hold on Class C downlinks that was
        // set when the RX2 parameter change was issued.
        if (m_rx2ChangePending.count(deviceAddr) > 0)
        {
            for (const auto& command : frameHdr.GetCommands())
            {
                if (command->GetCommandType() == RX_PARAM_SETUP_ANS)
                {
                    NS_LOG_INFO("RXParamSetupAns received from "
                                << deviceAddr << "; Class C downlinks resumed.");
                    m_rx2ChangePending.erase(deviceAddr);
                    break;
                }
            }
        }

        // Forward only if an AS is connected and there is application payload
        bool haveAsLink = m_useIpTransport || !m_uplinkForwardCb.IsNull();
        if (haveAsLink && payloadCopy->GetSize() > 0)
        {
            // Deduplicate: with N gateways in range the same uplink arrives N
            // times (and confirmed-uplink retransmissions repeat the FCnt);
            // forward each frame counter value only once per device.
            uint32_t fCnt = frameHdr.GetFCnt();
            auto lastFwd = m_lastForwardedFCnt.find(deviceAddr);
            if (lastFwd != m_lastForwardedFCnt.end() && lastFwd->second == fCnt)
            {
                NS_LOG_DEBUG("Uplink FCnt " << fCnt << " from " << deviceAddr
                                            << " already forwarded to AS; dropping duplicate.");
            }
            else
            {
                m_lastForwardedFCnt[deviceAddr] = fCnt;
                NS_LOG_INFO("Forwarding " << payloadCopy->GetSize()
                                          << " bytes uplink payload from device " << deviceAddr
                                          << " to Application Server");
                m_forwardedToAS(payloadCopy);
                if (m_useIpTransport && m_asUplinkSocket)
                {
                    Ptr<Packet> datagram = payloadCopy->Copy();
                    AsTransportHeader header;
                    header.SetDeviceAddress(deviceAddr.Get());
                    datagram->AddHeader(header);
                    m_asUplinkSocket->Send(datagram);
                }
                else
                {
                    m_uplinkForwardCb(deviceAddr, payloadCopy);
                }
            }
        }
    }

    return true;
}

void
NetworkServer::AddComponent(Ptr<NetworkControllerComponent> component)
{
    NS_LOG_FUNCTION(this << component);

    m_controller->Install(component);
}

Ptr<NetworkStatus>
NetworkServer::GetNetworkStatus()
{
    return m_status;
}

// ---- Application Server Interface ----

void
NetworkServer::SetUplinkForwardCallback(UplinkForwardCallback cb)
{
    NS_LOG_FUNCTION(this);
    m_uplinkForwardCb = cb;
}

void
NetworkServer::EnqueueDownlink(LoraDeviceAddress deviceAddress, Ptr<Packet> payload, bool confirmed)
{
    NS_LOG_FUNCTION(this << deviceAddress << payload->GetSize() << confirmed);

    DoEnqueueDownlink(deviceAddress, payload, confirmed, MAX_DOWNLINK_RETRIES);
}

void
NetworkServer::NotifyRx2ParamChangePending(LoraDeviceAddress deviceAddress)
{
    NS_LOG_FUNCTION(this << deviceAddress);
    m_rx2ChangePending[deviceAddress] = true;
}

void
NetworkServer::ConnectToApplicationServer(Ipv4Address asAddress,
                                          uint16_t uplinkPort,
                                          uint16_t downlinkPort)
{
    NS_LOG_FUNCTION(this << asAddress << uplinkPort << downlinkPort);
    m_asAddress = asAddress;
    m_asUplinkPort = uplinkPort;
    m_asDownlinkPort = downlinkPort;
    m_useIpTransport = true;
}

void
NetworkServer::ConfirmedDownlinkTimeout(LoraDeviceAddress deviceAddress)
{
    NS_LOG_FUNCTION(this << deviceAddress);
    NS_LOG_WARN("No acknowledgement for confirmed downlink to "
                << deviceAddress << " within CLASS_C_RESP_TIMEOUT; releasing slot.");
    m_confirmedDlPending.erase(deviceAddress);
}

void
NetworkServer::HandleAsDatagram(Ptr<Socket> socket)
{
    NS_LOG_FUNCTION(this << socket);

    Ptr<Packet> datagram;
    while ((datagram = socket->Recv()))
    {
        if (datagram->GetSize() < AsTransportHeader().GetSerializedSize())
        {
            NS_LOG_WARN("Ignoring undersized datagram from the Application Server.");
            continue;
        }
        AsTransportHeader header;
        datagram->RemoveHeader(header);
        EnqueueDownlink(LoraDeviceAddress(header.GetDeviceAddress()),
                        datagram,
                        header.IsConfirmed());
    }
}

void
NetworkServer::DoEnqueueDownlink(LoraDeviceAddress deviceAddress,
                                 Ptr<Packet> payload,
                                 bool confirmed,
                                 uint8_t retriesLeft)
{
    NS_LOG_FUNCTION(this << deviceAddress << payload->GetSize() << confirmed
                         << unsigned(retriesLeft));

    // Look up the end device
    Ptr<EndDeviceStatus> edStatus = m_status->GetEndDeviceStatus(deviceAddress);
    if (!edStatus)
    {
        NS_LOG_ERROR("EnqueueDownlink: unknown device " << deviceAddress);
        return;
    }

    // Hold downlinks while an RX2 parameter change awaits RXParamSetupAns:
    // the device may still be listening on the old RXC parameters
    // (LoRaWAN 1.0.4 Section 5.4).
    if (m_rx2ChangePending.count(deviceAddress) > 0)
    {
        NS_LOG_WARN("EnqueueDownlink: RX2 parameter change pending for "
                    << deviceAddress << "; holding downlink.");
        Simulator::Schedule(MilliSeconds(DOWNLINK_HOLD_RECHECK_MS),
                            &NetworkServer::DoEnqueueDownlink,
                            this,
                            deviceAddress,
                            payload,
                            confirmed,
                            retriesLeft);
        return;
    }

    // A pending confirmed downlink blocks ALL further downlinks to the
    // device (head-of-line, as production network servers do) until it is
    // acknowledged or times out (LoRaWAN 1.0.4 Section 15 keeps at most one
    // confirmed downlink outstanding per device).
    if (m_confirmedDlPending.count(deviceAddress) > 0)
    {
        NS_LOG_INFO("EnqueueDownlink: confirmed downlink outstanding for "
                    << deviceAddress << "; holding this one.");
        Simulator::Schedule(MilliSeconds(DOWNLINK_HOLD_RECHECK_MS),
                            &NetworkServer::DoEnqueueDownlink,
                            this,
                            deviceAddress,
                            payload,
                            confirmed,
                            retriesLeft);
        return;
    }

    // Keep clear of the device's Class A receive-window region: RX1/RX2
    // preempt an in-progress RXC demodulation, so a downlink started here
    // would be aborted by the device.
    auto lastUplink = m_lastUplinkTime.find(deviceAddress);
    if (lastUplink != m_lastUplinkTime.end())
    {
        Time safeAt = lastUplink->second + Seconds(CLASS_A_QUIET_PERIOD_S);
        if (Simulator::Now() < safeAt)
        {
            NS_LOG_INFO("EnqueueDownlink: device " << deviceAddress
                                                   << " is inside its Class A window region; "
                                                      "deferring downlink to "
                                                   << safeAt.As(Time::S));
            Simulator::Schedule(safeAt - Simulator::Now(),
                                &NetworkServer::DoEnqueueDownlink,
                                this,
                                deviceAddress,
                                payload,
                                confirmed,
                                retriesLeft);
            return;
        }
    }

    // Coordinate with the Class A scheduler: if a reply is about to be sent
    // in this device's RX1/RX2 windows, let it go first.
    if (m_status->NeedsReply(deviceAddress) || edStatus->HasReceiveWindowOpportunityScheduled())
    {
        NS_LOG_INFO("EnqueueDownlink: Class A reply pending for "
                    << deviceAddress << "; deferring spontaneous downlink.");
        Simulator::Schedule(MilliSeconds(DOWNLINK_HOLD_RECHECK_MS),
                            &NetworkServer::DoEnqueueDownlink,
                            this,
                            deviceAddress,
                            payload,
                            confirmed,
                            retriesLeft);
        return;
    }

    // Find the best gateway for this device (use window 2 = RX2 for Class C)
    Address gwAddress = m_status->GetBestGatewayForDevice(deviceAddress, 2);
    if (gwAddress == Address())
    {
        if (retriesLeft > 0)
        {
            NS_LOG_WARN("EnqueueDownlink: no gateway available for device "
                        << deviceAddress << "; retrying in " << DOWNLINK_RETRY_DELAY_MS
                        << " ms (" << unsigned(retriesLeft) << " retries left)");
            Simulator::Schedule(MilliSeconds(DOWNLINK_RETRY_DELAY_MS),
                                &NetworkServer::DoEnqueueDownlink,
                                this,
                                deviceAddress,
                                payload,
                                confirmed,
                                static_cast<uint8_t>(retriesLeft - 1));
        }
        else
        {
            NS_LOG_ERROR("EnqueueDownlink: no gateway available for device "
                         << deviceAddress << " after all retries; dropping downlink.");
        }
        return;
    }

    // Build LoRaWAN downlink frame: MacHeader + FrameHeader + Payload
    Ptr<Packet> pkt = payload->Copy();

    LoraFrameHeader frameHdr;
    frameHdr.SetAsDownlink();
    frameHdr.SetAddress(deviceAddress);
    frameHdr.SetAck(false);
    frameHdr.SetFCnt(m_downlinkFCnt[deviceAddress]++);
    // FPort > 0 marks an application-data downlink; a Class C device SHALL
    // silently discard RXC downlinks carrying MAC commands (FPort 0).
    frameHdr.SetFPort(1);
    pkt->AddHeader(frameHdr);

    LorawanMacHeader macHdr;
    macHdr.SetMType(confirmed ? LorawanMacHeader::CONFIRMED_DATA_DOWN
                              : LorawanMacHeader::UNCONFIRMED_DATA_DOWN);
    pkt->AddHeader(macHdr);

    // Tag with RX2 parameters (same pattern as NetworkStatus::GetReplyForDevice).
    // The payload may still carry the LoraTag of the uplink it was copied from
    // (e.g. an AS echo handler); remove it first, a packet can only hold one.
    LoraTag staleTag;
    pkt->RemovePacketTag(staleTag);
    LoraTag tag;
    tag.SetDataRate(edStatus->GetMac()->GetSecondReceiveWindowDataRate());
    tag.SetFrequency(edStatus->GetSecondReceiveWindowFrequency());
    pkt->AddPacketTag(tag);

    NS_LOG_INFO("EnqueueDownlink: sending " << payload->GetSize()
                << " bytes to device " << deviceAddress
                << " via gateway " << gwAddress);

    // A confirmed downlink occupies the per-device slot until the device
    // acknowledges it or CLASS_C_RESP_TIMEOUT expires.
    if (confirmed)
    {
        m_confirmedDlPending[deviceAddress] =
            Simulator::Schedule(Seconds(CLASS_C_RESP_TIMEOUT_S),
                                &NetworkServer::ConfirmedDownlinkTimeout,
                                this,
                                deviceAddress);
    }

    // Send through the selected gateway
    m_sentDownlink(pkt);
    m_status->SendThroughGateway(pkt, gwAddress);
}

} // namespace lorawan
} // namespace ns3
