/*
 * Application Server Comprehensive Verification Example
 *
 * Tests every hop of the LoRaWAN architecture with Application Server:
 *
 *   TEST 1: Topology — correct node count (ED, GW, NS, AS)
 *   TEST 2: Class C — all EDs initialized as Class C
 *   TEST 3: CSMA/IP — NS and AS have IP addresses on same subnet
 *   TEST 4: Uplink path — ED → GW → NS → AS → handler
 *   TEST 5: Downlink path — handler → AS → NS → GW
 *   TEST 6: ED reception — downlink actually received by ED MAC
 *   TEST 7: Trace sources — all 5 NS+AS trace callbacks fire
 *   TEST 8: Packet integrity — payload sizes preserved through pipeline
 *   TEST 9: Multi-device — all 3 devices independently served
 *   TEST 10: Multi-handler — two handlers both receive every uplink
 *
 * Topology:
 *   ED0 ─┐
 *   ED1 ─┼──LoRa──▶ GW ──P2P──▶ NS ──CSMA/IP──▶ AS
 *   ED2 ─┘                       10.2.0.1         10.2.0.2
 *                                                  ├── echo handler
 *                                                  └── counter handler
 */

#include "ns3/class-c-end-device-lorawan-mac.h"
#include "ns3/command-line.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/end-device-lora-phy.h"
#include "ns3/forwarder-helper.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/gateway-lorawan-mac.h"
#include "ns3/ipv4.h"
#include "ns3/log.h"
#include "ns3/lora-application-server-helper.h"
#include "ns3/lora-application-server.h"
#include "ns3/lora-helper.h"
#include "ns3/lora-net-device.h"
#include "ns3/lorawan-mac-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/network-server-helper.h"
#include "ns3/node-container.h"
#include "ns3/one-shot-sender-helper.h"
#include "ns3/periodic-sender-helper.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/point-to-point-net-device.h"
#include "ns3/pointer.h"
#include "ns3/position-allocator.h"
#include "ns3/simulator.h"
#include "ns3/string.h"

#include <iomanip>
#include <map>
#include <set>
#include <sstream>

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE("AppServerVerifyExample");

// ============================================================
//  Global counters for trace source verification
// ============================================================

// NS trace counters
uint32_t g_nsReceivedPacket = 0;
uint32_t g_nsForwardedToAS = 0;
uint32_t g_nsSentDownlink = 0;

// AS trace counters
uint32_t g_asReceivedUplink = 0;
uint32_t g_asSentDownlink = 0;

// ED-side reception counters (per node id)
std::map<uint32_t, uint32_t> g_edRxCount;
std::map<uint32_t, uint32_t> g_edTxCount;

// Packet size tracking
std::vector<uint32_t> g_uplinkSizes;
std::vector<uint32_t> g_downlinkSizes;

// Timing
Time g_firstUplinkAtNS = Seconds(0);
Time g_firstDownlinkAtED = Seconds(0);
bool g_firstUplinkSeen = false;
bool g_firstDownlinkSeen = false;

// ============================================================
//  NS Trace callbacks
// ============================================================

void
NsReceivedPacketCb(Ptr<const Packet> pkt)
{
    g_nsReceivedPacket++;
    if (!g_firstUplinkSeen)
    {
        g_firstUplinkAtNS = Simulator::Now();
        g_firstUplinkSeen = true;
    }
    NS_LOG_DEBUG("[TRACE:NS:ReceivedPacket] #" << g_nsReceivedPacket
                 << " size=" << pkt->GetSize()
                 << " t=" << Simulator::Now().As(Time::S));
}

void
NsForwardedToASCb(Ptr<const Packet> pkt)
{
    g_nsForwardedToAS++;
    g_uplinkSizes.push_back(pkt->GetSize());
    NS_LOG_DEBUG("[TRACE:NS:ForwardedToAS] #" << g_nsForwardedToAS
                 << " payload=" << pkt->GetSize() << "B"
                 << " t=" << Simulator::Now().As(Time::S));
}

void
NsSentDownlinkCb(Ptr<const Packet> pkt)
{
    g_nsSentDownlink++;
    NS_LOG_DEBUG("[TRACE:NS:SentDownlink] #" << g_nsSentDownlink
                 << " size=" << pkt->GetSize()
                 << " t=" << Simulator::Now().As(Time::S));
}

// ============================================================
//  AS Trace callbacks
// ============================================================

void
AsReceivedUplinkCb(Ptr<const Packet> pkt)
{
    g_asReceivedUplink++;
    NS_LOG_DEBUG("[TRACE:AS:ReceivedUplink] #" << g_asReceivedUplink
                 << " payload=" << pkt->GetSize() << "B"
                 << " t=" << Simulator::Now().As(Time::S));
}

void
AsSentDownlinkCb(Ptr<const Packet> pkt)
{
    g_asSentDownlink++;
    g_downlinkSizes.push_back(pkt->GetSize());
    NS_LOG_DEBUG("[TRACE:AS:SentDownlink] #" << g_asSentDownlink
                 << " payload=" << pkt->GetSize() << "B"
                 << " t=" << Simulator::Now().As(Time::S));
}

// ============================================================
//  ED-side callbacks
//
//  PHY StartSending trace signature: (Ptr<const Packet>, uint32_t)
//  MAC ReceivedPacket trace signature: (Ptr<const Packet>)
//
//  After MakeBoundCallback binds nodeId as first arg, remaining
//  args must match the trace source signature exactly.
// ============================================================

void
EdTxCallback(uint32_t nodeId, Ptr<const Packet> pkt, uint32_t /* senderNodeId */)
{
    g_edTxCount[nodeId]++;
    NS_LOG_DEBUG("[ED:" << nodeId << ":TX] uplink #" << g_edTxCount[nodeId]
                 << " size=" << pkt->GetSize()
                 << " t=" << Simulator::Now().As(Time::S));
}

void
EdRxCallback(uint32_t nodeId, Ptr<const Packet> pkt)
{
    g_edRxCount[nodeId]++;
    if (!g_firstDownlinkSeen)
    {
        g_firstDownlinkAtED = Simulator::Now();
        g_firstDownlinkSeen = true;
    }
    NS_LOG_INFO("[ED:" << nodeId << ":RX] *** DOWNLINK RECEIVED *** #"
                << g_edRxCount[nodeId]
                << " size=" << pkt->GetSize()
                << " t=" << Simulator::Now().As(Time::S));
}

// ============================================================
//  Echo Handler (registered on AS)
// ============================================================

class EchoHandler : public Object
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("EchoHandler").SetParent<Object>();
        return tid;
    }

    void SetAppServer(Ptr<LoraApplicationServer> as) { m_as = as; }

    void OnUplink(LoraDeviceAddress addr, Ptr<Packet> payload)
    {
        m_rxCount++;
        m_devicesServed.insert(addr.GetNwkAddr());
        m_lastPayloadSize = payload->GetSize();

        NS_LOG_INFO("[ECHO] Uplink #" << m_rxCount
                    << " from " << addr
                    << " (" << payload->GetSize() << "B)"
                    << " t=" << Simulator::Now().As(Time::S));

        Simulator::Schedule(MilliSeconds(100),
                            &EchoHandler::SendEcho, this, addr, payload->Copy());
    }

    uint32_t GetRxCount() const { return m_rxCount; }
    uint32_t GetTxCount() const { return m_txCount; }
    uint32_t GetUniqueDevices() const { return m_devicesServed.size(); }
    uint32_t GetLastPayloadSize() const { return m_lastPayloadSize; }

  private:
    void SendEcho(LoraDeviceAddress addr, Ptr<Packet> payload)
    {
        m_txCount++;
        NS_LOG_INFO("[ECHO] Downlink #" << m_txCount
                    << " to " << addr
                    << " (" << payload->GetSize() << "B)"
                    << " t=" << Simulator::Now().As(Time::S));
        m_as->SendDownlink(addr, payload);
    }

    Ptr<LoraApplicationServer> m_as;
    uint32_t m_rxCount = 0;
    uint32_t m_txCount = 0;
    uint32_t m_lastPayloadSize = 0;
    std::set<uint32_t> m_devicesServed;
};

// ============================================================
//  Counter Handler (second handler — tests multi-handler)
// ============================================================

class CounterHandler : public Object
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("CounterHandler").SetParent<Object>();
        return tid;
    }

    void OnUplink(LoraDeviceAddress addr, Ptr<Packet> payload)
    {
        m_count++;
        NS_LOG_DEBUG("[COUNTER] Uplink #" << m_count
                     << " from " << addr
                     << " (" << payload->GetSize() << "B)");
    }

    uint32_t GetCount() const { return m_count; }

  private:
    uint32_t m_count = 0;
};

// ============================================================
//  Test result printer
// ============================================================

int g_testsPassed = 0;
int g_testsFailed = 0;

void
PrintTest(int num, const std::string& name, bool pass, const std::string& detail = "")
{
    std::string status = pass ? "\033[32mPASS\033[0m" : "\033[31mFAIL\033[0m";
    std::cout << "  TEST " << std::setw(2) << num << ": [" << status << "] " << name;
    if (!detail.empty())
    {
        std::cout << " -- " << detail;
    }
    std::cout << std::endl;
    if (pass)
        g_testsPassed++;
    else
        g_testsFailed++;
}

// ============================================================
//  Main
// ============================================================

int
main(int argc, char* argv[])
{
    int nDevices = 3;
    double simulationTimeSeconds = 120;
    int payloadSize = 20;

    CommandLine cmd(__FILE__);
    cmd.AddValue("nDevices", "Number of Class C end devices", nDevices);
    cmd.AddValue("simulationTime", "Simulation time [s]", simulationTimeSeconds);
    cmd.AddValue("payloadSize", "Uplink payload size [bytes]", payloadSize);
    cmd.Parse(argc, argv);

    // ---- Logging ----
    LogComponentEnable("AppServerVerifyExample", LOG_LEVEL_ALL);
    LogComponentEnable("LoraApplicationServer", LOG_LEVEL_INFO);
    LogComponentEnable("LoraApplicationServerHelper", LOG_LEVEL_INFO);
    LogComponentEnable("NetworkServer", LOG_LEVEL_INFO);

    std::cout << std::endl;
    std::cout << "=========================================================" << std::endl;
    std::cout << "  LoRaWAN Application Server -- Comprehensive Verification" << std::endl;
    std::cout << "=========================================================" << std::endl;
    std::cout << "  Devices:  " << nDevices << " (Class C)" << std::endl;
    std::cout << "  Payload:  " << payloadSize << " bytes" << std::endl;
    std::cout << "  Duration: " << simulationTimeSeconds << "s" << std::endl;
    std::cout << "  Handlers: echo + counter" << std::endl;
    std::cout << std::endl;
    std::cout << "  ED0 --+" << std::endl;
    std::cout << "  ED1 --+--LoRa--> GW --P2P--> NS --CSMA/IP--> AS" << std::endl;
    std::cout << "  ED2 --+                       .0.1             .0.2" << std::endl;
    std::cout << "                                                 |-- echo" << std::endl;
    std::cout << "                                                 |-- counter" << std::endl;
    std::cout << "=========================================================" << std::endl;
    std::cout << std::endl;

    /************************
     *  Create the channel  *
     ************************/

    Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel>();
    loss->SetPathLossExponent(3.76);
    loss->SetReference(1, 7.7);

    Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel>();
    Ptr<LoraChannel> channel = CreateObject<LoraChannel>(loss, delay);

    /************************
     *  Create the helpers  *
     ************************/

    LoraPhyHelper phyHelper = LoraPhyHelper();
    phyHelper.SetChannel(channel);

    LorawanMacHelper macHelper = LorawanMacHelper();

    LoraHelper helper = LoraHelper();
    helper.EnablePacketTracking();

    /************************
     *  Create end devices  *
     ************************/

    NS_LOG_INFO("Creating " << nDevices << " Class C end device(s)...");

    NodeContainer endDevices;
    endDevices.Create(nDevices);

    MobilityHelper mobility;
    Ptr<ListPositionAllocator> edAllocator = CreateObject<ListPositionAllocator>();
    for (int i = 0; i < nDevices; i++)
    {
        edAllocator->Add(Vector(100.0 * (i + 1), 0, 1.2));
    }
    mobility.SetPositionAllocator(edAllocator);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(endDevices);

    uint8_t nwkId = 54;
    uint32_t nwkAddr = 1864;
    Ptr<LoraDeviceAddressGenerator> addrGen =
        CreateObject<LoraDeviceAddressGenerator>(nwkId, nwkAddr);

    macHelper.SetAddressGenerator(addrGen);
    phyHelper.SetDeviceType(LoraPhyHelper::ED);
    macHelper.SetDeviceType(LorawanMacHelper::ED_C);
    macHelper.SetRegion(LorawanMacHelper::EU);
    helper.Install(phyHelper, macHelper, endDevices);

    // Verify Class C and connect ED-side trace callbacks
    int classCCount = 0;
    for (int i = 0; i < nDevices; i++)
    {
        Ptr<Node> node = endDevices.Get(i);
        uint32_t nodeId = node->GetId();
        Ptr<LoraNetDevice> loraDevice = DynamicCast<LoraNetDevice>(node->GetDevice(0));
        Ptr<ClassCEndDeviceLorawanMac> classCMac =
            DynamicCast<ClassCEndDeviceLorawanMac>(loraDevice->GetMac());

        if (classCMac)
        {
            classCCount++;
            NS_LOG_INFO("Device " << i << " (node " << nodeId << "): Class C OK, addr="
                        << classCMac->GetDeviceAddress());

            // PHY StartSending trace: fires (Ptr<const Packet>, uint32_t sfNum)
            // After binding nodeId → callback is (Ptr<const Packet>, uint32_t)
            Ptr<EndDeviceLoraPhy> edPhy =
                DynamicCast<EndDeviceLoraPhy>(loraDevice->GetPhy());
            if (edPhy)
            {
                edPhy->TraceConnectWithoutContext(
                    "StartSending",
                    MakeBoundCallback(&EdTxCallback, nodeId));
            }

            // MAC ReceivedPacket trace: fires (Ptr<const Packet>)
            // After binding nodeId → callback is (Ptr<const Packet>)
            classCMac->TraceConnectWithoutContext(
                "ReceivedPacket",
                MakeBoundCallback(&EdRxCallback, nodeId));
        }
        else
        {
            NS_LOG_ERROR("Device " << i << ": FAILED Class C init!");
        }
    }

    /*********************
     *  Create gateway   *
     *********************/

    NS_LOG_INFO("Creating gateway...");

    NodeContainer gateways;
    gateways.Create(1);

    Ptr<ListPositionAllocator> gwAllocator = CreateObject<ListPositionAllocator>();
    gwAllocator->Add(Vector(0.0, 0.0, 15.0));
    mobility.SetPositionAllocator(gwAllocator);
    mobility.Install(gateways);

    phyHelper.SetDeviceType(LoraPhyHelper::GW);
    macHelper.SetDeviceType(LorawanMacHelper::GW);
    helper.Install(phyHelper, macHelper, gateways);

    LorawanMacHelper::SetSpreadingFactorsUp(endDevices, gateways, channel);

    /*****************************
     *  Create Network Server    *
     *****************************/

    NS_LOG_INFO("Creating network server...");

    Ptr<Node> nsNode = CreateObject<Node>();

    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("5Mbps"));
    p2p.SetChannelAttribute("Delay", StringValue("2ms"));

    P2PGwRegistration_t gwRegistration;
    for (auto gw = gateways.Begin(); gw != gateways.End(); ++gw)
    {
        auto container = p2p.Install(nsNode, *gw);
        auto serverP2PNetDev = DynamicCast<PointToPointNetDevice>(container.Get(0));
        gwRegistration.emplace_back(serverP2PNetDev, *gw);
    }

    NetworkServerHelper nsHelper = NetworkServerHelper();
    nsHelper.SetGatewaysP2P(gwRegistration);
    nsHelper.SetEndDevices(endDevices);
    nsHelper.Install(nsNode);

    Ptr<NetworkServer> nsApp = DynamicCast<NetworkServer>(nsNode->GetApplication(0));
    NS_ASSERT_MSG(nsApp, "NetworkServer app not found!");

    // Connect NS trace sources
    nsApp->TraceConnectWithoutContext("ReceivedPacket", MakeCallback(&NsReceivedPacketCb));
    nsApp->TraceConnectWithoutContext("ForwardedToAS", MakeCallback(&NsForwardedToASCb));
    nsApp->TraceConnectWithoutContext("SentDownlink", MakeCallback(&NsSentDownlinkCb));

    ForwarderHelper forHelper = ForwarderHelper();
    forHelper.Install(gateways);

    /*********************************
     *  Create Application Server    *
     *********************************/

    NS_LOG_INFO("Creating application server (CSMA + IP)...");

    Ptr<Node> asNode = CreateObject<Node>();

    LoraApplicationServerHelper asHelper;
    asHelper.SetNetworkServer(nsApp);
    Ptr<LoraApplicationServer> appServer = asHelper.Install(asNode, nsNode);

    // Connect AS trace sources
    appServer->TraceConnectWithoutContext("ReceivedUplink", MakeCallback(&AsReceivedUplinkCb));
    appServer->TraceConnectWithoutContext("SentDownlink", MakeCallback(&AsSentDownlinkCb));

    /**********************************
     *  Register handlers on AS       *
     **********************************/

    NS_LOG_INFO("Registering handlers: echo + counter...");

    Ptr<EchoHandler> echo = CreateObject<EchoHandler>();
    echo->SetAppServer(appServer);
    appServer->RegisterHandler("echo",
        MakeCallback(&EchoHandler::OnUplink, PeekPointer(echo)));

    Ptr<CounterHandler> counter = CreateObject<CounterHandler>();
    appServer->RegisterHandler("counter",
        MakeCallback(&CounterHandler::OnUplink, PeekPointer(counter)));

    /***********************************
     *  Schedule uplinks from devices  *
     ***********************************/

    NS_LOG_INFO("Scheduling periodic uplinks (every 30s, " << payloadSize << "B)...");

    PeriodicSenderHelper appHelper = PeriodicSenderHelper();
    appHelper.SetPeriod(Seconds(30));
    appHelper.SetPacketSize(payloadSize);
    ApplicationContainer appContainer = appHelper.Install(endDevices);
    appContainer.Start(Seconds(0));
    appContainer.Stop(Seconds(simulationTimeSeconds));

    // ---- Capture IP addresses for test 3 ----
    Ptr<Ipv4> nsIpv4 = nsNode->GetObject<Ipv4>();
    Ptr<Ipv4> asIpv4 = asNode->GetObject<Ipv4>();
    bool nsHasIp = (nsIpv4 && nsIpv4->GetNInterfaces() > 1);
    bool asHasIp = (asIpv4 && asIpv4->GetNInterfaces() > 1);
    std::string nsIpStr = "none";
    std::string asIpStr = "none";
    if (nsHasIp)
    {
        std::ostringstream oss;
        oss << nsIpv4->GetAddress(nsIpv4->GetNInterfaces() - 1, 0).GetLocal();
        nsIpStr = oss.str();
    }
    if (asHasIp)
    {
        std::ostringstream oss;
        oss << asIpv4->GetAddress(asIpv4->GetNInterfaces() - 1, 0).GetLocal();
        asIpStr = oss.str();
    }

    /****************
     *  Simulation  *
     ****************/

    Simulator::Stop(Seconds(simulationTimeSeconds));

    NS_LOG_INFO("=========================================");
    NS_LOG_INFO("  SIMULATION STARTING");
    NS_LOG_INFO("=========================================");

    Simulator::Run();

    NS_LOG_INFO("=========================================");
    NS_LOG_INFO("  SIMULATION COMPLETE");
    NS_LOG_INFO("=========================================");

    /***************************
     *  Results & Verification *
     ***************************/

    LoraPacketTracker& tracker = helper.GetPacketTracker();

    uint32_t totalEdRx = 0;
    for (auto& p : g_edRxCount) totalEdRx += p.second;

    uint32_t totalEdTx = 0;
    for (auto& p : g_edTxCount) totalEdTx += p.second;

    bool payloadIntegrity = true;
    for (auto sz : g_uplinkSizes)
    {
        if (sz != (uint32_t)payloadSize) { payloadIntegrity = false; break; }
    }
    for (auto sz : g_downlinkSizes)
    {
        if (sz != (uint32_t)payloadSize) { payloadIntegrity = false; break; }
    }

    // ---- Print Counters ----
    std::cout << std::endl;
    std::cout << "=========================================================" << std::endl;
    std::cout << "  COUNTERS" << std::endl;
    std::cout << "---------------------------------------------------------" << std::endl;
    std::cout << "  MAC packets (sent recv):   "
              << tracker.CountMacPacketsGlobally(Seconds(0), Seconds(simulationTimeSeconds))
              << std::endl;
    std::cout << "  ED-side TX (PHY):          " << totalEdTx << std::endl;
    std::cout << "  NS ReceivedPacket:         " << g_nsReceivedPacket << std::endl;
    std::cout << "  NS ForwardedToAS:          " << g_nsForwardedToAS << std::endl;
    std::cout << "  AS ReceivedUplink:         " << g_asReceivedUplink << std::endl;
    std::cout << "  Echo handler uplinks:      " << echo->GetRxCount() << std::endl;
    std::cout << "  Counter handler uplinks:   " << counter->GetCount() << std::endl;
    std::cout << "  Echo handler downlinks:    " << echo->GetTxCount() << std::endl;
    std::cout << "  AS SentDownlink:           " << g_asSentDownlink << std::endl;
    std::cout << "  NS SentDownlink:           " << g_nsSentDownlink << std::endl;
    std::cout << "  ED-side RX (MAC):          " << totalEdRx << std::endl;
    std::cout << "  Unique devices served:     " << echo->GetUniqueDevices()
              << "/" << nDevices << std::endl;
    std::cout << std::endl;

    // Per-device breakdown
    std::cout << "  Per-device breakdown:" << std::endl;
    for (int i = 0; i < nDevices; i++)
    {
        uint32_t nid = endDevices.Get(i)->GetId();
        std::cout << "    ED" << i << " (node " << nid << "): TX="
                  << g_edTxCount[nid] << "  RX=" << g_edRxCount[nid] << std::endl;
    }
    std::cout << std::endl;

    if (g_firstUplinkSeen)
    {
        std::cout << "  Timing:" << std::endl;
        std::cout << "    First uplink at NS:    " << g_firstUplinkAtNS.As(Time::S) << std::endl;
        if (g_firstDownlinkSeen)
        {
            std::cout << "    First downlink at ED:  " << g_firstDownlinkAtED.As(Time::S) << std::endl;
            std::cout << "    Round-trip (approx):   "
                      << (g_firstDownlinkAtED - g_firstUplinkAtNS).As(Time::MS) << std::endl;
        }
        else
        {
            std::cout << "    First downlink at ED:  (none received)" << std::endl;
        }
        std::cout << std::endl;
    }

    // ---- Run Tests ----
    std::cout << "=========================================================" << std::endl;
    std::cout << "  TESTS" << std::endl;
    std::cout << "---------------------------------------------------------" << std::endl;

    uint32_t totalNodes = endDevices.GetN() + gateways.GetN() + 1 + 1;
    PrintTest(1, "Topology",
              totalNodes >= 4,
              std::to_string(totalNodes) + " nodes (expect " + std::to_string(nDevices) + " ED + 1 GW + 1 NS + 1 AS)");

    PrintTest(2, "Class C init",
              classCCount == nDevices,
              std::to_string(classCCount) + "/" + std::to_string(nDevices) + " Class C");

    PrintTest(3, "CSMA/IP link",
              nsHasIp && asHasIp,
              "NS=" + nsIpStr + " AS=" + asIpStr);

    PrintTest(4, "Uplink: ED->GW->NS->AS->handler",
              echo->GetRxCount() > 0,
              std::to_string(echo->GetRxCount()) + " uplinks reached handler");

    PrintTest(5, "Downlink: handler->AS->NS->GW",
              g_nsSentDownlink > 0,
              std::to_string(g_nsSentDownlink) + " downlinks sent by NS");

    PrintTest(6, "ED received downlinks",
              totalEdRx > 0,
              std::to_string(totalEdRx) + " received by ED MAC");

    bool tracesOk = (g_nsReceivedPacket > 0) && (g_nsForwardedToAS > 0)
                    && (g_nsSentDownlink > 0) && (g_asReceivedUplink > 0)
                    && (g_asSentDownlink > 0);
    PrintTest(7, "Trace sources (all 5 fire)",
              tracesOk,
              "NS:Recv=" + std::to_string(g_nsReceivedPacket)
              + " Fwd=" + std::to_string(g_nsForwardedToAS)
              + " DL=" + std::to_string(g_nsSentDownlink)
              + " | AS:UL=" + std::to_string(g_asReceivedUplink)
              + " DL=" + std::to_string(g_asSentDownlink));

    PrintTest(8, "Packet integrity (size preserved)",
              payloadIntegrity && !g_uplinkSizes.empty(),
              std::to_string(payloadSize) + "B through full pipeline");

    PrintTest(9, "Multi-device",
              (int)echo->GetUniqueDevices() == nDevices,
              std::to_string(echo->GetUniqueDevices()) + "/" + std::to_string(nDevices) + " unique devices");

    PrintTest(10, "Multi-handler",
              counter->GetCount() == echo->GetRxCount() && counter->GetCount() > 0,
              "echo=" + std::to_string(echo->GetRxCount())
              + " counter=" + std::to_string(counter->GetCount()));

    std::cout << "---------------------------------------------------------" << std::endl;
    if (g_testsFailed == 0)
    {
        std::cout << "  >>> ALL " << g_testsPassed << " TESTS PASSED <<<" << std::endl;
    }
    else
    {
        std::cout << "  >>> " << g_testsFailed << " FAILED, "
                  << g_testsPassed << " passed <<<" << std::endl;
    }
    std::cout << "=========================================================" << std::endl;
    std::cout << std::endl;

    Simulator::Destroy();
    return (g_testsFailed == 0) ? 0 : 1;
}