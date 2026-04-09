/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Helper for setting up the LoRaWAN Application Server in ns-3.
 */

#include "lora-application-server-helper.h"

#include "ns3/lora-application-server.h"
#include "ns3/network-server.h"

#include "ns3/csma-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4.h"
#include "ns3/log.h"
#include "ns3/node-container.h"
#include "ns3/string.h"

namespace ns3
{
namespace lorawan
{

NS_LOG_COMPONENT_DEFINE("LoraApplicationServerHelper");

LoraApplicationServerHelper::LoraApplicationServerHelper()
    : m_subnetBase("10.2.0.0"),
      m_subnetMask("255.255.255.0"),
      m_dataRate("100Mbps"),
      m_delay("6560ns")  // Realistic Ethernet propagation delay
{
}

LoraApplicationServerHelper::~LoraApplicationServerHelper()
{
}

void
LoraApplicationServerHelper::SetNetworkServer(Ptr<NetworkServer> ns)
{
    m_networkServer = ns;
}

void
LoraApplicationServerHelper::SetSubnet(const std::string& base, const std::string& mask)
{
    m_subnetBase = base;
    m_subnetMask = mask;
}

void
LoraApplicationServerHelper::SetCsmaChannel(const std::string& dataRate,
                                             const std::string& delay)
{
    m_dataRate = dataRate;
    m_delay = delay;
}

Ptr<LoraApplicationServer>
LoraApplicationServerHelper::Install(Ptr<Node> asNode, Ptr<Node> nsNode)
{
    NS_LOG_FUNCTION(this << asNode << nsNode);

    // ---- 1. Create CSMA LAN between NS and AS nodes ----
    NodeContainer lanNodes;
    lanNodes.Add(nsNode);
    lanNodes.Add(asNode);

    CsmaHelper csma;
    csma.SetChannelAttribute("DataRate", StringValue(m_dataRate));
    csma.SetChannelAttribute("Delay", StringValue(m_delay));
    NetDeviceContainer csmaDevices = csma.Install(lanNodes);

    NS_LOG_INFO("CSMA LAN created between NS (node "
                << nsNode->GetId() << ") and AS (node "
                << asNode->GetId() << ")");

    // ---- 2. Install Internet stack (only on nodes that don't have it) ----
    InternetStackHelper internet;

    if (!nsNode->GetObject<Ipv4>())
    {
        internet.Install(nsNode);
        NS_LOG_INFO("Internet stack installed on NS node " << nsNode->GetId());
    }
    else
    {
        NS_LOG_INFO("NS node " << nsNode->GetId() << " already has Internet stack");
    }

    if (!asNode->GetObject<Ipv4>())
    {
        internet.Install(asNode);
        NS_LOG_INFO("Internet stack installed on AS node " << asNode->GetId());
    }
    else
    {
        NS_LOG_INFO("AS node " << asNode->GetId() << " already has Internet stack");
    }

    // ---- 3. Assign IP addresses ----
    Ipv4AddressHelper ipv4;
    ipv4.SetBase(Ipv4Address(m_subnetBase.c_str()),
                 Ipv4Mask(m_subnetMask.c_str()));
    Ipv4InterfaceContainer interfaces = ipv4.Assign(csmaDevices);

    NS_LOG_INFO("NS IP: " << interfaces.GetAddress(0)
                << "  AS IP: " << interfaces.GetAddress(1)
                << "  subnet: " << m_subnetBase << "/" << m_subnetMask);

    // ---- 4. Create and install the LoraApplicationServer application ----
    Ptr<LoraApplicationServer> appServer = CreateObject<LoraApplicationServer>();
    asNode->AddApplication(appServer);

    // ---- 5. Connect AS to Network Server via callback ----
    // In reality this would be gRPC/MQTT over the IP link above.
    // We model the API as a direct C++ callback — the CSMA+IP link
    // provides the correct topology and could carry actual packets
    // in a future extension.
    if (m_networkServer)
    {
        appServer->SetNetworkServer(m_networkServer);
        NS_LOG_INFO("AS connected to NetworkServer via callback "
                     "(models gRPC/MQTT over " << interfaces.GetAddress(0)
                     << " <-> " << interfaces.GetAddress(1) << ")");
    }
    else
    {
        NS_LOG_WARN("No NetworkServer set - AS will not receive uplinks");
    }

    NS_LOG_INFO("LoraApplicationServer installed on node " << asNode->GetId());

    return appServer;
}

} // namespace lorawan
} // namespace ns3
