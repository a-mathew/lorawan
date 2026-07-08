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

#ifndef NETWORK_SERVER_H
#define NETWORK_SERVER_H

#include "class-a-end-device-lorawan-mac.h"
#include "gateway-status.h"
#include "lora-device-address.h"
#include "network-controller.h"
#include "network-scheduler.h"
#include "network-status.h"

#include "ns3/application.h"
#include "ns3/log.h"
#include "ns3/net-device.h"
#include "ns3/node-container.h"
#include "ns3/object.h"
#include "ns3/packet.h"
#include "ns3/point-to-point-net-device.h"

namespace ns3
{
namespace lorawan
{

/**
 * @ingroup lorawan
 *
 * Callback signature for forwarding uplink application payloads to an
 * Application Server.
 *
 * @param address  The LoRa device address of the sender.
 * @param payload  The application-layer payload (MAC headers stripped).
 */
typedef Callback<void, LoraDeviceAddress, Ptr<Packet>> UplinkForwardCallback;

/**
 * @ingroup lorawan
 *
 * The NetworkServer is an application standing on top of a node equipped with
 * links that connect it with the gateways.
 *
 * This version of the NetworkServer application attempts to closely mimic an actual
 * network server, by providing as much functionality as possible.
 */
class NetworkServer : public Application
{
  public:
    /**
     *  Register this type.
     *  @return The object TypeId.
     */
    static TypeId GetTypeId();

    NetworkServer();           //!< Default constructor
    ~NetworkServer() override; //!< Destructor

    /**
     * Start the network server application.
     */
    void StartApplication() override;

    /**
     * Stop the network server application.
     */
    void StopApplication() override;

    /**
     * Inform the NetworkServer application that these nodes are connected to the network.
     *
     * This method will create a DeviceStatus object for each new node, and add
     * it to the list.
     *
     * @param nodes The end device NodeContainer.
     */
    void AddNodes(NodeContainer nodes);

    /**
     * Inform the NetworkServer application that this node is connected to the network.
     *
     * This method will create a DeviceStatus object for the new node (if it
     * doesn't already exist).
     *
     * @param node The end device Node.
     */
    void AddNode(Ptr<Node> node);

    /**
     * Add the gateway to the list of gateways connected to this network server.
     *
     * Each gateway is identified by its Address in the network connecting it to the network
     * server.
     *
     * @param gateway A pointer to the gateway Node.
     * @param netDevice A pointer to the network server's NetDevice connected to the gateway.
     */
    void AddGateway(Ptr<Node> gateway, Ptr<NetDevice> netDevice);

    /**
     * Add a NetworkControllerComponent to this NetworkServer application.
     *
     * @param component A pointer to the NetworkControllerComponent object.
     */
    void AddComponent(Ptr<NetworkControllerComponent> component);

    /**
     * Receive a packet from a gateway.
     *
     * This function is meant to be provided to NetDevice objects as a ReceiveCallback.
     *
     * @copydoc ns3::NetDevice::ReceiveCallback
     */
    bool Receive(Ptr<NetDevice> device,
                 Ptr<const Packet> packet,
                 uint16_t protocol,
                 const Address& sender);

    /**
     * Get the NetworkStatus object of this NetworkServer application.
     *
     * @return A pointer to the NetworkStatus object.
     */
    Ptr<NetworkStatus> GetNetworkStatus();

    // ---- Application Server Interface ----

    /**
     * Set the callback used to forward uplink application payloads to the
     * Application Server.
     *
     * @param cb The callback to invoke on every uplink with payload.
     */
    void SetUplinkForwardCallback(UplinkForwardCallback cb);

    /**
     * Enqueue a downlink application payload for an end device.
     *
     * The NetworkServer wraps the payload in proper LoRaWAN headers,
     * tags with RX2 parameters, selects the best gateway, and transmits.
     * For Class C devices the RX2 window is always open.
     *
     * @param deviceAddress The target end device address.
     * @param payload       The raw application payload to deliver.
     */
    void EnqueueDownlink(LoraDeviceAddress deviceAddress, Ptr<Packet> payload);

  protected:
    /**
     * Internal worker for EnqueueDownlink.
     *
     * Defers transmission while the device is inside its Class A
     * receive-window region (RX1/RX2 preempt an RXC demodulation,
     * LoRaWAN 1.0.4 Section 15) and retries when no gateway is currently
     * able to transmit (busy or duty-cycle limited).
     *
     * @param deviceAddress The target end device address.
     * @param payload       The raw application payload to deliver.
     * @param retriesLeft   Remaining no-gateway retries before dropping.
     */
    void DoEnqueueDownlink(LoraDeviceAddress deviceAddress,
                           Ptr<Packet> payload,
                           uint8_t retriesLeft);

    Ptr<NetworkStatus> m_status;         //!< Ptr to the NetworkStatus object.
    Ptr<NetworkController> m_controller; //!< Ptr to the NetworkController object.
    Ptr<NetworkScheduler> m_scheduler;   //!< Ptr to the NetworkScheduler object.

    TracedCallback<Ptr<const Packet>> m_receivedPacket; //!< The `ReceivedPacket` trace source.

    /// Trace source fired when an uplink payload is forwarded to the Application Server.
    TracedCallback<Ptr<const Packet>> m_forwardedToAS;

    /// Trace source fired when a downlink built by EnqueueDownlink is sent to a gateway.
    TracedCallback<Ptr<const Packet>> m_sentDownlink;

    UplinkForwardCallback m_uplinkForwardCb; //!< Callback to Application Server.

    /// Last uplink frame counter forwarded to the AS, per device (deduplicates
    /// multi-gateway copies and confirmed-uplink retransmissions).
    std::map<LoraDeviceAddress, uint32_t> m_lastForwardedFCnt;

    /// Downlink frame counter per device for server-initiated (Class C) downlinks.
    std::map<LoraDeviceAddress, uint16_t> m_downlinkFCnt;

    /// Time of the last uplink received from each device, used to keep
    /// spontaneous Class C downlinks clear of the RX1/RX2 window region.
    std::map<LoraDeviceAddress, Time> m_lastUplinkTime;
};

} // namespace lorawan

} // namespace ns3
#endif /* NETWORK_SERVER_H */
