/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Helper for setting up the LoRaWAN Application Server in ns-3.
 *
 * Creates a CSMA (Ethernet) LAN between the NS and AS nodes with
 * full IP stack, matching real LoRaWAN deployments where NS and AS
 * communicate over IP (gRPC, MQTT, HTTP).
 */

#ifndef LORA_APPLICATION_SERVER_HELPER_H
#define LORA_APPLICATION_SERVER_HELPER_H

#include "ns3/ipv4-address.h"
#include "ns3/node.h"
#include "ns3/ptr.h"

#include <string>

namespace ns3
{
namespace lorawan
{

class NetworkServer;
class LoraApplicationServer;

/**
 * @ingroup lorawan
 *
 * Helper class for creating and configuring a LoraApplicationServer.
 *
 * Creates a CSMA LAN with IP addresses between the NS and AS nodes,
 * matching the real-world architecture where these servers communicate
 * over Ethernet/IP.
 *
 * Usage:
 *   LoraApplicationServerHelper asHelper;
 *   asHelper.SetNetworkServer(nsApp);
 *   Ptr<LoraApplicationServer> appServer = asHelper.Install(asNode, nsNode);
 */
class LoraApplicationServerHelper
{
  public:
    LoraApplicationServerHelper();
    ~LoraApplicationServerHelper();

    /**
     * Set the NetworkServer application that the AS will connect to.
     */
    void SetNetworkServer(Ptr<NetworkServer> ns);

    /**
     * Set the IP subnet for the NS-AS CSMA LAN.
     * Default: 10.2.0.0/24
     *
     * @param base    Network address (e.g., "10.2.0.0").
     * @param mask    Subnet mask (e.g., "255.255.255.0").
     */
    void SetSubnet(const std::string& base, const std::string& mask);

    /**
     * Set CSMA channel parameters.
     *
     * @param dataRate  Data rate string (default "100Mbps").
     * @param delay     Channel delay string (default "2ms").
     */
    void SetCsmaChannel(const std::string& dataRate, const std::string& delay);

    /**
     * Install the LoraApplicationServer on a node and create a CSMA
     * LAN with IP stack between the AS node and the NS node.
     *
     * @param asNode  Node to install the Application Server on.
     * @param nsNode  Node where the NetworkServer is running.
     * @return Pointer to the installed LoraApplicationServer.
     */
    Ptr<LoraApplicationServer> Install(Ptr<Node> asNode, Ptr<Node> nsNode);

  private:
    Ptr<NetworkServer> m_networkServer;

    std::string m_subnetBase;  //!< IP network base (default "10.2.0.0")
    std::string m_subnetMask;  //!< IP subnet mask  (default "255.255.255.0")
    std::string m_dataRate;    //!< CSMA data rate   (default "100Mbps")
    std::string m_delay;       //!< CSMA delay       (default "2ms")
};

} // namespace lorawan
} // namespace ns3

#endif /* LORA_APPLICATION_SERVER_HELPER_H */
