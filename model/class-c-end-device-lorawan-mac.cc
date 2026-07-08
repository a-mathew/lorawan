/*
 * Copyright (c) 2017 University of Padova
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Author: Davide Magrin <magrinda@dei.unipd.it>
 *         Martina Capuzzo <capuzzom@dei.unipd.it>
 *
 * Modified by: Peggy Anderson <peggy.anderson@usask.ca>
 *              qiuyukang <b612n@qq.com>
 *
 * Class C implementation compliant with LoRaWAN v1.0.4, Section 15.
 *
 * References:
 *   - LoRaWAN Link Layer Specification v1.0.4 (TS001-1.0.4)
 *   - Semtech "LoRaWAN 1.0.4 Class B and C for End Devices"
 *   - Microchip "LoRaWAN Message and End-Device Types"
 *   - The Things Network "Device Classes" documentation
 */

#include "class-c-end-device-lorawan-mac.h"

#include "end-device-lora-phy.h"
#include "lora-tag.h"

#include "ns3/log.h"
#include "ns3/simulator.h"

namespace ns3
{
namespace lorawan
{

NS_LOG_COMPONENT_DEFINE("ClassCEndDeviceLorawanMac");

NS_OBJECT_ENSURE_REGISTERED(ClassCEndDeviceLorawanMac);

TypeId
ClassCEndDeviceLorawanMac::GetTypeId()
{
    static TypeId tid = TypeId("ns3::ClassCEndDeviceLorawanMac")
                            .SetParent<ClassAEndDeviceLorawanMac>()
                            .SetGroupName("lorawan")
                            .AddConstructor<ClassCEndDeviceLorawanMac>();
    return tid;
}

ClassCEndDeviceLorawanMac::ClassCEndDeviceLorawanMac()
    : m_continuousRxOpen(false),
      m_downlinkReceivedInRx(false),
      m_firstReceiveWindowFrequencyHz(0)
{
    NS_LOG_FUNCTION(this);
    // All other members (m_receiveDelay1/2, m_rx1DrOffset, m_closeFirstWindow,
    // m_secondReceiveWindow, m_closeSecondWindow, m_secondReceiveWindowFrequencyHz,
    // m_secondReceiveWindowDataRate) are initialized by ClassAEndDeviceLorawanMac.
}

ClassCEndDeviceLorawanMac::~ClassCEndDeviceLorawanMac()
{
    NS_LOG_FUNCTION_NOARGS();
}

/////////////////////
// Sending methods //
/////////////////////

void
ClassCEndDeviceLorawanMac::SendToPhy(Ptr<Packet> packetToSend)
{
    NS_LOG_DEBUG("PacketToSend: " << packetToSend);

    // LoRaWAN 1.0.4 Section 15: Close RXC before transmitting
    CloseContinuousReceiveWindow();

    // The radio may still be listening in RX (e.g. inside an RX1/RX2
    // window); it must be in STANDBY before switching to TX
    {
        Ptr<EndDeviceLoraPhy> phy = DynamicCast<EndDeviceLoraPhy>(m_phy);
        if (phy->GetState() == EndDeviceLoraPhy::State::RX)
        {
            phy->SwitchToStandby();
        }
    }

    // Reset the per-TX-cycle flag
    m_downlinkReceivedInRx = false;

    // Craft LoraTxParameters object
    LoraTxParameters params;
    params.sf = GetSfFromDataRate(m_dataRate);
    params.headerDisabled = m_headerDisabled;
    params.codingRate = m_codingRate;
    params.bandwidthHz = GetBandwidthFromDataRate(m_dataRate);
    params.nPreamble = m_nPreambleSymbols;
    params.crcEnabled = true;
    params.lowDataRateOptimizationEnabled = LoraPhy::GetTSym(params) > MilliSeconds(16);

    Ptr<LogicalLoraChannel> txChannel = GetRandomChannelForTx();

    NS_LOG_DEBUG("PacketToSend: " << packetToSend);
    m_phy->Send(packetToSend, params, txChannel->GetFrequency(), m_txPowerDbm);

    // Register for duty cycle
    Time duration = LoraPhy::GetOnAirTime(packetToSend, params);
    m_channelHelper->AddEvent(duration, txChannel);

    // Prepare PHY for RX1 (will be switched to RX1 freq/SF when RX1 opens)
    m_firstReceiveWindowFrequencyHz = txChannel->GetFrequency();
    DynamicCast<EndDeviceLoraPhy>(m_phy)->SetFrequency(txChannel->GetFrequency());

    uint8_t replyDataRate = GetFirstReceiveWindowDataRate();
    NS_LOG_DEBUG("m_dataRate: " << unsigned(m_dataRate)
                                << ", m_rx1DrOffset: " << unsigned(m_rx1DrOffset)
                                << ", replyDataRate: " << unsigned(replyDataRate) << ".");

    DynamicCast<EndDeviceLoraPhy>(m_phy)->SetSpreadingFactor(GetSfFromDataRate(replyDataRate));
}

//////////////////////////
//  Receiving methods   //
//////////////////////////

void
ClassCEndDeviceLorawanMac::Receive(Ptr<const Packet> packet)
{
    NS_LOG_FUNCTION(this << packet);

    if (!packet || packet->GetSize() < 1)
    {
        NS_LOG_WARN("Dropping malformed downlink: empty/undersized packet");
        if (!m_continuousRxOpen)
        {
            Simulator::Schedule(MilliSeconds(1),
                                &ClassCEndDeviceLorawanMac::OpenContinuousReceiveWindow,
                                this);
        }
        return;
    }

    Ptr<Packet> packetCopy = packet->Copy();

    LorawanMacHeader mHdr;
    packetCopy->RemoveHeader(mHdr);

    if (packetCopy->GetSize() < 7)
    {
        NS_LOG_WARN("Dropping malformed downlink: insufficient bytes for frame header ("
                    << packetCopy->GetSize() << "B)");
        if (!m_continuousRxOpen)
        {
            Simulator::Schedule(MilliSeconds(1),
                                &ClassCEndDeviceLorawanMac::OpenContinuousReceiveWindow,
                                this);
        }
        return;
    }

    NS_LOG_DEBUG("Mac Header: " << mHdr);

    if (!mHdr.IsUplink())
    {
        NS_LOG_INFO("Found a downlink packet.");

        LoraFrameHeader fHdr;
        fHdr.SetAsDownlink();
        packetCopy->RemoveHeader(fHdr);

        NS_LOG_DEBUG("Frame Header: " << fHdr);

        bool messageForUs = (m_address == fHdr.GetAddress());

        if (messageForUs)
        {
            NS_LOG_INFO("The message is for us!");

            // LoRaWAN 1.0.4 §15: a Class C (RXC) downlink SHALL NOT transport
            // MAC commands; if it does, the entire frame is silently
            // discarded. MAC commands only arrive on RX1/RX2 downlinks.
            if (m_continuousRxOpen && (!fHdr.GetCommands().empty() || fHdr.GetFPort() == 0))
            {
                NS_LOG_WARN("Discarding Class C (RXC) downlink carrying MAC commands "
                            "(LoRaWAN 1.0.4 Section 15).");
                Simulator::Schedule(MilliSeconds(1),
                                    &ClassCEndDeviceLorawanMac::OpenContinuousReceiveWindow,
                                    this);
                return;
            }

            // Mark that we received a downlink in this RX cycle
            m_downlinkReceivedInRx = true;

            // Cancel pending RX2 window (LoRaWAN 1.0.4: if DL received in RX1,
            // device does not open RX2 and goes directly to RXC)
            Simulator::Cancel(m_secondReceiveWindow);
            Simulator::Cancel(m_closeSecondWindow);

            // Reset the ADR backoff counter only for Class A (RX1/RX2)
            // receptions: reference stacks do not reset it for downlinks
            // received in RXC, so a device served exclusively through RXC
            // still performs ADR backoff on its uplink path.
            if (!m_continuousRxOpen)
            {
                m_adrAckCnt = 0;
            }

            LoraTag tag;
            packet->PeekPacketTag(tag);
            m_lastRxSnr = tag.GetReceivePower() + 174 - 10 * log10(125000) - 6;

            // A confirmed downlink must be acknowledged with an uplink no
            // later than CLASS_C_RESP_TIMEOUT (LoRaWAN 1.0.4 Section 15).
            // Set the ACK flag for the next uplink and schedule one in case
            // no regular uplink is due.
            if (mHdr.GetMType() == LorawanMacHeader::CONFIRMED_DATA_DOWN)
            {
                NS_LOG_INFO("Received a confirmed downlink; scheduling acknowledgement.");
                m_ackDownlinkPending = true;
                if (!m_scheduledAckUplink.IsPending())
                {
                    Time ackDelay = Seconds(m_uniformRV->GetValue(1, 3));
                    m_scheduledAckUplink =
                        Simulator::Schedule(ackDelay,
                                            &ClassCEndDeviceLorawanMac::SendAckUplink,
                                            this);
                }
            }

            // Parse the MAC commands
            ParseCommands(fHdr);

            // TODO Pass the packet up to the NetDevice

            // Call the trace source
            m_receivedPacket(packet);

            // After receiving, reopen RXC
            Simulator::Schedule(MilliSeconds(1),
                                &ClassCEndDeviceLorawanMac::OpenContinuousReceiveWindow,
                                this);
        }
        else
        {
            NS_LOG_DEBUG("The message is intended for another recipient.");

            if (m_retxParams.waitingAck && m_secondReceiveWindow.IsExpired())
            {
                if (m_retxParams.retxLeft == 0)
                {
                    uint8_t txs = m_nbTrans - (m_retxParams.retxLeft);
                    m_requiredTxCallback(txs, false, m_retxParams.firstAttempt,
                                         m_retxParams.packet);
                    NS_LOG_DEBUG("Failure: no more retransmissions left. Used "
                                 << unsigned(txs) << " transmissions.");
                    ResetRetransmissionParameters();
                }
                else
                {
                    this->Send(m_retxParams.packet);
                    NS_LOG_INFO("We have " << unsigned(m_retxParams.retxLeft)
                                           << " retransmissions left: rescheduling transmission.");
                }
            }

            // Reopen RXC if not already open
            if (!m_continuousRxOpen)
            {
                Simulator::Schedule(MilliSeconds(1),
                                    &ClassCEndDeviceLorawanMac::OpenContinuousReceiveWindow,
                                    this);
            }
        }
    }
    else if (m_retxParams.waitingAck && m_secondReceiveWindow.IsExpired())
    {
        NS_LOG_INFO("The packet we are receiving is in uplink.");
        if (m_retxParams.retxLeft > 0)
        {
            this->Send(m_retxParams.packet);
            NS_LOG_INFO("We have " << unsigned(m_retxParams.retxLeft)
                                   << " retransmissions left: rescheduling transmission.");
        }
        else
        {
            uint8_t txs = m_nbTrans - (m_retxParams.retxLeft);
            m_requiredTxCallback(txs, false, m_retxParams.firstAttempt, m_retxParams.packet);
            NS_LOG_DEBUG("Failure: no more retransmissions left. Used " << unsigned(txs)
                                                                        << " transmissions.");
            ResetRetransmissionParameters();
        }
    }

    // A Class C device must always return to RXC when idle
    if (!m_continuousRxOpen)
    {
        Simulator::Schedule(MilliSeconds(1),
                            &ClassCEndDeviceLorawanMac::OpenContinuousReceiveWindow,
                            this);
    }
}

void
ClassCEndDeviceLorawanMac::FailedReception(Ptr<const Packet> packet)
{
    NS_LOG_FUNCTION(this << packet);

    // Same retransmission recovery as Class A: a corrupted reception around
    // RX2 must not stall a pending confirmed-uplink retransmission. Skipped
    // while transmitting (a preempted RXC reception can be reported mid-TX;
    // TxFinished will schedule the receive windows for the ongoing uplink).
    if (DynamicCast<EndDeviceLoraPhy>(m_phy)->GetState() != EndDeviceLoraPhy::State::TX &&
        m_secondReceiveWindow.IsExpired() && m_retxParams.waitingAck)
    {
        if (m_retxParams.retxLeft > 0)
        {
            this->Send(m_retxParams.packet);
            NS_LOG_INFO("We have " << unsigned(m_retxParams.retxLeft)
                                   << " retransmissions left: rescheduling transmission.");
        }
        else
        {
            uint8_t txs = m_nbTrans - (m_retxParams.retxLeft);
            m_requiredTxCallback(txs, false, m_retxParams.firstAttempt, m_retxParams.packet);
            NS_LOG_DEBUG("Failure: no more retransmissions left. Used " << unsigned(txs)
                                                                        << " transmissions.");
            ResetRetransmissionParameters();
        }
    }

    // After failed reception, ensure RXC gets reopened
    if (!m_continuousRxOpen)
    {
        Simulator::Schedule(MilliSeconds(1),
                            &ClassCEndDeviceLorawanMac::OpenContinuousReceiveWindow,
                            this);
    }
}

///////////////////////////////////////////////////////////////////////////
// TX/RX Window Lifecycle
//
// LoRaWAN 1.0.4 Class C timing:
//   TX → RXC → RX1 → RXC → RX2 → RXC (stays open)
//
// 1. TxFinished:     Open RXC immediately; schedule RX1 at DELAY1, RX2 at DELAY2
// 2. OpenFirstRx:    Close RXC; open RX1 on uplink freq/DR
// 3. CloseFirstRx:   Close RX1; open RXC (gap between RX1 and RX2)
// 4. OpenSecondRx:   Close RXC; open RX2 on RX2 freq/DR
// 5. CloseSecondRx:  Close RX2; open RXC continuously
///////////////////////////////////////////////////////////////////////////

void
ClassCEndDeviceLorawanMac::TxFinished(Ptr<const Packet> packet)
{
    NS_LOG_FUNCTION_NOARGS();

    // LoRaWAN 1.0.4 Section 15: Open RXC immediately after TX ends
    // (RXC opens between end-of-TX and start-of-RX1)
    OpenContinuousReceiveWindow();

    // Schedule RX1 at RECEIVE_DELAY1
    Simulator::Schedule(m_receiveDelay1,
                        &ClassCEndDeviceLorawanMac::OpenFirstReceiveWindow,
                        this);

    // Schedule RX2 at RECEIVE_DELAY2
    m_secondReceiveWindow = Simulator::Schedule(
        m_receiveDelay2,
        &ClassCEndDeviceLorawanMac::OpenSecondReceiveWindow,
        this);
}

void
ClassCEndDeviceLorawanMac::OpenFirstReceiveWindow()
{
    NS_LOG_FUNCTION_NOARGS();

    // LoRaWAN 1.0.4 Section 15: Class A windows take priority over RXC.
    // If RXC is demodulating, terminate it and open RX1.
    CloseContinuousReceiveWindow();

    Ptr<EndDeviceLoraPhy> phy = DynamicCast<EndDeviceLoraPhy>(m_phy);

    // Set PHY in Standby mode
    phy->SwitchToStandby();

    // Configure for RX1 explicitly. RXC may have overwritten PHY tuning to RX2
    // after TX finished, so RX1 cannot rely on previously set PHY parameters.
    if (m_firstReceiveWindowFrequencyHz != 0)
    {
        phy->SetFrequency(m_firstReceiveWindowFrequencyHz);
    }
    phy->SetSpreadingFactor(GetSfFromDataRate(GetFirstReceiveWindowDataRate()));

    // Calculate the duration of a single symbol for RX1 data rate
    double tSym = pow(2, GetSfFromDataRate(GetFirstReceiveWindowDataRate())) /
                  GetBandwidthFromDataRate(GetFirstReceiveWindowDataRate());

    // Schedule closing of RX1
    m_closeFirstWindow = Simulator::Schedule(
        Seconds(m_receiveWindowDurationInSymbols * tSym),
        &ClassCEndDeviceLorawanMac::CloseFirstReceiveWindow,
        this);

    // A Class C device listens in RX (drawing RX current) during its
    // Class A windows too
    phy->SwitchToRx();

    NS_LOG_INFO("Class C: RX1 window opened on "
                << m_firstReceiveWindowFrequencyHz << " Hz, DR"
                << unsigned(GetFirstReceiveWindowDataRate()));
}

void
ClassCEndDeviceLorawanMac::CloseFirstReceiveWindow()
{
    NS_LOG_FUNCTION_NOARGS();

    Ptr<EndDeviceLoraPhy> phy = DynamicCast<EndDeviceLoraPhy>(m_phy);

    switch (phy->GetState())
    {
    case EndDeviceLoraPhy::State::TX:
    case EndDeviceLoraPhy::State::SLEEP:
        break;
    case EndDeviceLoraPhy::State::RX:
        if (phy->IsReceivingPacket())
        {
            // PHY is demodulating: let it finish, Receive() handles the result
            NS_LOG_DEBUG("PHY is receiving in RX1: Receive will handle the result.");
            return;
        }
        // Nothing was detected during RX1: leave the listening state
        phy->SwitchToStandby();
        break;
    case EndDeviceLoraPhy::State::STANDBY:
        break;
    }

    NS_LOG_INFO("Class C: RX1 window closed.");

    // LoRaWAN 1.0.4: Open RXC in the gap between RX1 and RX2
    // (unless a downlink was already received, in which case RX2 is skipped)
    if (m_downlinkReceivedInRx)
    {
        NS_LOG_INFO("Downlink already received in RX1; skipping RX2, opening RXC.");
        Simulator::Cancel(m_secondReceiveWindow);
        Simulator::Cancel(m_closeSecondWindow);
        OpenContinuousReceiveWindow();
    }
    else
    {
        // Open RXC for the gap between RX1 close and RX2 open
        OpenContinuousReceiveWindow();
    }
}

void
ClassCEndDeviceLorawanMac::OpenSecondReceiveWindow()
{
    NS_LOG_FUNCTION_NOARGS();

    // If a downlink was received in RX1, skip RX2
    if (m_downlinkReceivedInRx)
    {
        NS_LOG_INFO("Skipping RX2: downlink already received.");
        return;
    }

    // LoRaWAN 1.0.4 Section 15: Close RXC, open Class A RX2
    CloseContinuousReceiveWindow();

    Ptr<EndDeviceLoraPhy> phy = DynamicCast<EndDeviceLoraPhy>(m_phy);

    if (phy->IsReceivingPacket())
    {
        NS_LOG_INFO("Won't open RX2 since a reception is in progress.");
        return;
    }

    phy->SwitchToStandby();

    // Switch to RX2 frequency and data rate
    NS_LOG_INFO("Opening RX2 using parameters: " << m_secondReceiveWindowFrequencyHz
                << " Hz, DR" << unsigned(m_secondReceiveWindowDataRate));

    phy->SetFrequency(m_secondReceiveWindowFrequencyHz);
    phy->SetSpreadingFactor(GetSfFromDataRate(m_secondReceiveWindowDataRate));

    // Calculate symbol duration for RX2 data rate
    double tSym = pow(2, GetSfFromDataRate(GetSecondReceiveWindowDataRate())) /
                  GetBandwidthFromDataRate(GetSecondReceiveWindowDataRate());

    // Schedule closing of RX2
    m_closeSecondWindow = Simulator::Schedule(
        Seconds(m_receiveWindowDurationInSymbols * tSym),
        &ClassCEndDeviceLorawanMac::CloseSecondReceiveWindow,
        this);

    // A Class C device listens in RX (drawing RX current) during its
    // Class A windows too
    phy->SwitchToRx();

    NS_LOG_INFO("Class C: RX2 window opened.");
}

void
ClassCEndDeviceLorawanMac::CloseSecondReceiveWindow()
{
    NS_LOG_FUNCTION_NOARGS();

    Ptr<EndDeviceLoraPhy> phy = DynamicCast<EndDeviceLoraPhy>(m_phy);

    switch (phy->GetState())
    {
    case EndDeviceLoraPhy::State::TX:
    case EndDeviceLoraPhy::State::SLEEP:
        break;
    case EndDeviceLoraPhy::State::RX:
        if (phy->IsReceivingPacket())
        {
            // PHY is demodulating: let it finish
            NS_LOG_DEBUG("PHY is receiving in RX2: Receive will handle the result.");
            return;
        }
        // Nothing was detected during RX2: leave the listening state
        phy->SwitchToStandby();
        break;
    case EndDeviceLoraPhy::State::STANDBY:
        // Nothing was detected in RX2
        break;
    }

    NS_LOG_INFO("Class C: RX2 window closed.");

    // LoRaWAN 1.0.4: After RX2 closes, open the continuous RXC
    OpenContinuousReceiveWindow();

    // Handle retransmission logic (same as Class A after RX2 closes)
    if (m_retxParams.waitingAck)
    {
        NS_LOG_DEBUG("No ACK received in RX1 or RX2. Class C continues listening on RXC.");
        if (m_retxParams.retxLeft == 0)
        {
            uint8_t txs = m_nbTrans - (m_retxParams.retxLeft);
            m_requiredTxCallback(txs, false, m_retxParams.firstAttempt, m_retxParams.packet);
            NS_LOG_DEBUG("Failure: no more retransmissions left. Used " << unsigned(txs)
                                                                        << " transmissions.");
            ResetRetransmissionParameters();
        }
        else
        {
            NS_LOG_INFO("We have " << unsigned(m_retxParams.retxLeft)
                                   << " retransmissions left: rescheduling transmission.");
            this->Send(m_retxParams.packet);
        }
    }
    else
    {
        uint8_t txs = m_nbTrans - (m_retxParams.retxLeft);
        m_requiredTxCallback(txs, true, m_retxParams.firstAttempt, m_retxParams.packet);
        NS_LOG_INFO("We have " << unsigned(m_retxParams.retxLeft)
                                << " transmissions left. Unconfirmed message cycle complete.");
        ResetRetransmissionParameters();
    }
}

///////////////////////////////////////////////////////////////////////////
// Continuous RXC Window Management
///////////////////////////////////////////////////////////////////////////

void
ClassCEndDeviceLorawanMac::OpenContinuousReceiveWindow()
{
    NS_LOG_FUNCTION(this);

    Ptr<EndDeviceLoraPhy> phy = DynamicCast<EndDeviceLoraPhy>(m_phy);

    // Don't open if we're currently transmitting
    if (phy->GetState() == EndDeviceLoraPhy::State::TX)
    {
        NS_LOG_INFO("Won't open RXC since we are in TX mode.");
        return;
    }

    // Don't open if a packet is being demodulated (let it finish; the
    // receive path reopens RXC afterwards)
    if (phy->IsReceivingPacket())
    {
        NS_LOG_INFO("Won't open RXC since a reception is in progress.");
        return;
    }

    // Set PHY to Standby before switching parameters
    phy->SwitchToStandby();

    // RXC uses the same frequency and data rate as RX2
    // (LoRaWAN 1.0.4 Section 15: "RxC parameters are identical by default
    //  to the Rx2 parameters (same channel and same data rate)")
    NS_LOG_INFO("Opening RXC using parameters: "
                << m_secondReceiveWindowFrequencyHz << " Hz, DR"
                << unsigned(m_secondReceiveWindowDataRate));

    phy->SetFrequency(m_secondReceiveWindowFrequencyHz);
    phy->SetSpreadingFactor(GetSfFromDataRate(m_secondReceiveWindowDataRate));

    // Class C continuous listening: the receiver is actually on the whole
    // time, drawing RX current — it does not idle in standby.
    phy->SwitchToRx();

    m_continuousRxOpen = true;

    NS_LOG_INFO("Class C device: RXC window is now open.");
}

void
ClassCEndDeviceLorawanMac::CloseContinuousReceiveWindow()
{
    NS_LOG_FUNCTION(this);

    if (!m_continuousRxOpen)
    {
        NS_LOG_DEBUG("RXC is already closed.");
        return;
    }

    Ptr<EndDeviceLoraPhy> phy = DynamicCast<EndDeviceLoraPhy>(m_phy);

    // LoRaWAN 1.0.4 Section 15: "If the end device is demodulating a
    // Class C downlink when the RX1 or RX2 window needs to be opened,
    // the demodulation should be terminated."
    if (phy->GetState() == EndDeviceLoraPhy::State::RX)
    {
        if (phy->IsReceivingPacket())
        {
            NS_LOG_INFO("Terminating RXC demodulation: Class A window takes priority.");
        }
        phy->SwitchToStandby();
    }

    m_continuousRxOpen = false;

    NS_LOG_INFO("Class C device: RXC window closed.");
}

/////////////////////////
// Getters and Setters //
/////////////////////////

Time
ClassCEndDeviceLorawanMac::GetNextClassTransmissionDelay(Time waitTime)
{
    NS_LOG_FUNCTION_NOARGS();

    if (!m_retxParams.waitingAck)
    {
        // New packet: must wait for any pending Class A windows to close
        if (!m_closeFirstWindow.IsExpired() || !m_closeSecondWindow.IsExpired() ||
            !m_secondReceiveWindow.IsExpired())
        {
            NS_LOG_WARN(
                "Attempting to send when there are receive windows: Transmission postponed.");
            double tSym = pow(2, GetSfFromDataRate(GetSecondReceiveWindowDataRate())) /
                          GetBandwidthFromDataRate(GetSecondReceiveWindowDataRate());
            Time endSecondRxWindow = Time(m_secondReceiveWindow.GetTs()) +
                                     Seconds(m_receiveWindowDurationInSymbols * tSym);

            NS_LOG_DEBUG("Duration until endSecondRxWindow for new transmission: "
                         << (endSecondRxWindow - Now()).As(Time::S));
            waitTime = Max(waitTime, endSecondRxWindow - Now());
        }
    }
    else
    {
        // Retransmission: ACK_TIMEOUT after RX2
        double ack_timeout = m_uniformRV->GetValue(1, 3);
        Time retransmitWaitTime =
            Time(m_secondReceiveWindow.GetTs()) - Now() + Seconds(ack_timeout);

        NS_LOG_DEBUG("ack_timeout: " << ack_timeout
                                     << " retransmitWaitTime: " << retransmitWaitTime.As(Time::S));
        waitTime = Max(waitTime, retransmitWaitTime);
    }

    return waitTime;
}

bool
ClassCEndDeviceLorawanMac::IsContinuousReceiveWindowOpen() const
{
    return m_continuousRxOpen;
}

void
ClassCEndDeviceLorawanMac::SendAckUplink()
{
    NS_LOG_FUNCTION(this);

    if (!m_ackDownlinkPending)
    {
        // A regular uplink already carried the acknowledgement
        return;
    }

    NS_LOG_INFO("Sending an empty uplink to acknowledge a confirmed downlink.");
    Send(Create<Packet>());
}

/////////////////////////
// MAC command methods //
/////////////////////////

void
ClassCEndDeviceLorawanMac::OnRxParamSetupReq(uint8_t rx1DrOffset,
                                              uint8_t rx2DataRate,
                                              double frequencyHz)
{
    NS_LOG_FUNCTION(this << unsigned(rx1DrOffset) << unsigned(rx2DataRate)
                         << uint32_t(frequencyHz));

    bool rx1DrOffsetAck = true;
    bool rx2DataRateAck = true;
    bool channelAck = true;

    if (rx1DrOffset >= m_replyDataRateMatrix.at(m_dataRate).size())
    {
        NS_LOG_WARN("Invalid rx1DrOffset");
        rx1DrOffsetAck = false;
    }

    if (!GetSfFromDataRate(rx2DataRate) || !GetBandwidthFromDataRate(rx2DataRate))
    {
        NS_LOG_WARN("Invalid rx2DataRate");
        rx2DataRateAck = false;
    }

    if (!m_channelHelper->IsFrequencyValid(frequencyHz))
    {
        NS_LOG_WARN("Invalid rx2 frequency");
        channelAck = false;
    }

    if (rx1DrOffsetAck && rx2DataRateAck && channelAck)
    {
        m_rx1DrOffset = rx1DrOffset;
        m_secondReceiveWindowDataRate = rx2DataRate;
        m_secondReceiveWindowFrequencyHz = frequencyHz;

        // LoRaWAN 1.0.4: When RX2 params change, RXC params also change
        // since RXC uses the same parameters as RX2.
        if (m_continuousRxOpen)
        {
            NS_LOG_INFO("Updating RXC parameters to match new RX2 parameters.");
            OpenContinuousReceiveWindow();
        }
    }

    NS_LOG_INFO("Adding RxParamSetupAns reply");
    m_macCommandList.emplace_back(
        Create<RxParamSetupAns>(rx1DrOffsetAck, rx2DataRateAck, channelAck));
}

} /* namespace lorawan */
} /* namespace ns3 */
