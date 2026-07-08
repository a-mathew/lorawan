/*
 * Copyright (c) 2017 University of Padova
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Davide Magrin <magrinda@dei.unipd.it>
 *         Martina Capuzzo <capuzzom@dei.unipd.it>
 *
 * Modified by: Peggy Anderson <peggy.anderson@usask.ca>
 *
 * Class C implementation compliant with LoRaWAN v1.0.4, Section 15.
 *
 * Class C timing diagram (per LoRaWAN 1.0.4):
 *
 *  |<-TX->|<--RXC-->|<-RX1->|<--RXC-->|<-RX2->|<---RXC (continuous)--->
 *         ^         ^       ^         ^       ^
 *         |         |       |         |       |
 *    end of TX   DELAY1  close    DELAY2  close
 *                         RX1              RX2
 *
 * RXC uses the same frequency and data rate as RX2.
 * RXC opens whenever the device is not in TX, RX1, or RX2.
 * If a Class C downlink is being demodulated when RX1 or RX2 must open,
 * the demodulation MUST be terminated and the Class A window takes priority.
 * (LoRaWAN 1.0.4 Section 15, line 2270-2272)
 */

#ifndef CLASS_C_END_DEVICE_LORAWAN_MAC_H
#define CLASS_C_END_DEVICE_LORAWAN_MAC_H

#include "class-a-end-device-lorawan-mac.h"

namespace ns3
{
namespace lorawan
{

/**
 * @ingroup lorawan
 *
 * Class representing the MAC layer of a Class C LoRaWAN device.
 *
 * Class C extends Class A: it inherits all Class A behavior (RX1/RX2 windows)
 * and adds a continuous receive window (RXC) that is open whenever the device
 * is not transmitting or in a Class A receive window.
 */
class ClassCEndDeviceLorawanMac : public ClassAEndDeviceLorawanMac
{
  public:
    static TypeId GetTypeId();

    ClassCEndDeviceLorawanMac();
    ~ClassCEndDeviceLorawanMac() override;

    // Sending — overrides Class A to close RXC before TX
    void SendToPhy(Ptr<Packet> packet) override;

    // Receiving — overrides Class A to manage RXC lifecycle
    void Receive(Ptr<const Packet> packet) override;
    void FailedReception(Ptr<const Packet> packet) override;
    void TxFinished(Ptr<const Packet> packet) override;

    // Class A receive windows — overridden to insert RXC gaps
    void OpenFirstReceiveWindow();
    void CloseFirstReceiveWindow();
    void OpenSecondReceiveWindow();
    void CloseSecondReceiveWindow();

    // Class C continuous receive window
    void OpenContinuousReceiveWindow();
    void CloseContinuousReceiveWindow();

    // Getters
    Time GetNextClassTransmissionDelay(Time waitTime) override;
    bool IsContinuousReceiveWindowOpen() const;

    // MAC commands — overrides Class A to also update RXC params
    void OnRxParamSetupReq(uint8_t rx1DrOffset, uint8_t rx2DataRate, double frequencyHz) override;

  private:
    /**
     * Send an uplink acknowledging a received confirmed downlink, unless a
     * regular uplink already carried the ACK bit in the meantime
     * (LoRaWAN 1.0.4 Section 15: the ACK uplink must be sent no later than
     * CLASS_C_RESP_TIMEOUT after the confirmed downlink).
     */
    void SendAckUplink();

    bool m_continuousRxOpen;       //!< Whether RXC is currently open
    bool m_downlinkReceivedInRx;   //!< DL received in RX1 → skip RX2
    uint32_t m_firstReceiveWindowFrequencyHz; //!< Last uplink frequency used to restore RX1 tuning
    EventId m_scheduledAckUplink;  //!< Scheduled ACK uplink for a confirmed downlink

}; /* ClassCEndDeviceLorawanMac */
} /* namespace lorawan */
} /* namespace ns3 */
#endif /* CLASS_C_END_DEVICE_LORAWAN_MAC_H */
