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
#include "ns3/packet.h"
#include "ns3/ptr.h"
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
     */
    void SendDownlink(LoraDeviceAddress deviceAddress, Ptr<Packet> payload);

    Ptr<NetworkServer> GetNetworkServer() const;

  protected:
    void StartApplication() override;
    void StopApplication() override;

  private:
    Ptr<NetworkServer> m_networkServer;

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