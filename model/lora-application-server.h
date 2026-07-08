/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * LoRaWAN Application Server for ns-3.
 *
 * Models the Application Server (AS) in the LoRaWAN architecture.
 * Receives uplink application payloads from the Network Server,
 * dispatches them to registered application handlers, and allows
 * handlers to enqueue downlink payloads back through the NS.
 *
 * Usage:
 *   Ptr<LoraApplicationServer> appServer = CreateObject<LoraApplicationServer>();
 *   asNode->AddApplication(appServer);
 *   appServer->SetNetworkServer(nsApp);
 *   appServer->RegisterHandler("myApp",
 *       MakeCallback(&MyAppServer::OnUplink, myAppServer));
 */

#ifndef LORA_APPLICATION_SERVER_H
#define LORA_APPLICATION_SERVER_H

#include "lora-device-address.h"
#include "network-server.h"

#include "ns3/application.h"
#include "ns3/callback.h"
#include "ns3/header.h"
#include "ns3/ipv4-address.h"
#include "ns3/packet.h"
#include "ns3/ptr.h"
#include "ns3/socket.h"
#include "ns3/traced-callback.h"

#include <map>
#include <string>
#include <vector>

namespace ns3
{
namespace lorawan
{

class NetworkServer; // Forward declaration

/**
 * @ingroup lorawan
 *
 * Header prepended to payloads exchanged between the NetworkServer and the
 * LoraApplicationServer over the UDP/IP transport. Carries the LoRa device
 * address the payload belongs to and a flags field (bit 0: the downlink is
 * to be sent as CONFIRMED_DATA_DOWN).
 */
class AsTransportHeader : public Header
{
  public:
    static TypeId GetTypeId();
    TypeId GetInstanceTypeId() const override;

    uint32_t GetSerializedSize() const override;
    void Serialize(Buffer::Iterator start) const override;
    uint32_t Deserialize(Buffer::Iterator start) override;
    void Print(std::ostream& os) const override;

    void SetDeviceAddress(uint32_t address);
    uint32_t GetDeviceAddress() const;
    void SetConfirmed(bool confirmed);
    bool IsConfirmed() const;

  private:
    uint32_t m_deviceAddress = 0; //!< LoRa device address of the payload.
    uint8_t m_flags = 0;          //!< Bit 0: confirmed downlink requested.
};

/**
 * @ingroup lorawan
 *
 * Callback type for application handlers that process uplink payloads.
 */
typedef Callback<void, LoraDeviceAddress, Ptr<Packet>> AppUplinkHandler;

/**
 * @ingroup lorawan
 *
 * LoRaWAN Application Server.
 *
 * This ns3::Application models the Application Server in the LoRaWAN
 * architecture.  It receives uplink application payloads forwarded by
 * the NetworkServer and dispatches them to registered handlers
 * (e.g., FUOTA, custom apps).  Handlers can send downlinks
 * via SendDownlink() which routes through the NetworkServer.
 *
 * Trace sources:
 *   - ReceivedUplink: fires when an uplink payload arrives from the NS.
 *   - SentDownlink:   fires when a downlink payload is sent to the NS.
 */
class LoraApplicationServer : public Application
{
  public:
    static TypeId GetTypeId();

    LoraApplicationServer();
    ~LoraApplicationServer() override;

    /**
     * Set the NetworkServer that this AS is connected to.
     * Also registers the uplink forwarding callback on the NS.
     */
    void SetNetworkServer(Ptr<NetworkServer> ns);

    /**
     * Use a UDP/IP transport towards the Network Server instead of the
     * direct callback: uplinks are received as datagrams on uplinkPort and
     * downlink requests are sent to nsAddress:downlinkPort. The datagrams
     * actually traverse the link between the two nodes (e.g. the CSMA LAN
     * set up by LoraApplicationServerHelper).
     *
     * @param nsAddress    IPv4 address of the Network Server.
     * @param uplinkPort   UDP port this server listens on for uplinks.
     * @param downlinkPort UDP port the Network Server listens on.
     */
    void SetIpTransport(Ipv4Address nsAddress, uint16_t uplinkPort, uint16_t downlinkPort);

    /**
     * Register an application handler for uplink payloads.
     * All registered handlers are called for every uplink (broadcast).
     * Handlers filter by device address internally if needed.
     *
     * @param name     Human-readable name (e.g., "myApp").
     * @param handler  Callback invoked on each uplink payload.
     */
    void RegisterHandler(const std::string& name, AppUplinkHandler handler);

    /**
     * Called by the NetworkServer when an uplink payload arrives.
     * Dispatches to all registered handlers.
     */
    void OnUplink(LoraDeviceAddress deviceAddress, Ptr<Packet> payload);

    /**
     * Send a downlink payload to an end device.
     * Called by application handlers. Forwards to NS for transmission.
     *
     * @param deviceAddress The target end device address.
     * @param payload       The application payload to deliver.
     * @param confirmed     Whether to request a CONFIRMED_DATA_DOWN.
     */
    void SendDownlink(LoraDeviceAddress deviceAddress,
                      Ptr<Packet> payload,
                      bool confirmed = false);

    Ptr<NetworkServer> GetNetworkServer() const;

  protected:
    void StartApplication() override;
    void StopApplication() override;

  private:
    /**
     * Receive an uplink datagram from the Network Server (UDP transport).
     *
     * @param socket The socket the datagram arrived on.
     */
    void HandleNsDatagram(Ptr<Socket> socket);

    Ptr<NetworkServer> m_networkServer;

    bool m_useIpTransport = false; //!< Whether the NS link uses UDP/IP.
    Ipv4Address m_nsAddress;       //!< Network Server IPv4 address.
    uint16_t m_uplinkPort = 0;     //!< Local port receiving uplink datagrams.
    uint16_t m_downlinkPort = 0;   //!< NS port for downlink datagrams.
    Ptr<Socket> m_uplinkSocket;    //!< Socket receiving NS -> AS uplinks.
    Ptr<Socket> m_downlinkSocket;  //!< Socket for AS -> NS downlinks.

    /// Registered handlers: (name, callback)
    std::vector<std::pair<std::string, AppUplinkHandler>> m_handlers;

    uint64_t m_uplinkCount;
    uint64_t m_downlinkCount;

    // ---- Trace sources (same pattern as NetworkServer) ----

    /**
     * Trace source fired when an uplink application payload is received
     * from the Network Server and dispatched to handlers.
     */
    TracedCallback<Ptr<const Packet>> m_receivedUplink;

    /**
     * Trace source fired when a downlink application payload is forwarded
     * to the Network Server for transmission.
     */
    TracedCallback<Ptr<const Packet>> m_sentDownlink;
};

} // namespace lorawan
} // namespace ns3

#endif /* LORA_APPLICATION_SERVER_H */