/*
 * Class C LoRaWAN End Device Example with Realistic Channel Model
 *
 * This example demonstrates Class C LoRaWAN end devices under a realistic
 * propagation environment using the 3-layer channel model from the academic
 * literature (Magrin et al., IEEE IoT Journal, 2019):
 *
 *   1. Log-distance path loss  (n=3.76, d0=1m, PL(d0)=7.7 dB)
 *   2. Correlated shadowing    (sigma=4 dB, d_corr=110 m)
 *   3. Building penetration    (3GPP TR 45.820)
 *
 * The realistic channel layers are enabled with --realisticChannel=true.
 * Without it, only log-distance path loss is applied (useful as a baseline).
 *
 * Usage:
 *   ./ns3 run "class-c-realistic-channel-example"
 *   ./ns3 run "class-c-realistic-channel-example --realisticChannel=true"
 *   ./ns3 run "class-c-realistic-channel-example --realisticChannel=true --nDevices=100"
 *
 * References:
 *   - LoRaWAN L2 Specification v1.0.4, Section 15 (Class C)
 *   - D. Magrin, M. Capuzzo, A. Zanella, "A Thorough Study of LoRaWAN
 *     Performance Under Different Parameter Settings," IEEE IoT Journal, 2019
 *   - C. Goursaud, J.-M. Gorce, "Dedicated networks for IoT: PHY/MAC state
 *     of the art and challenges," EAI Endorsed Trans. Internet of Things, 2015
 *   - 3GPP TR 45.820 (building penetration loss model)
 */

#include "ns3/building-allocator.h"
#include "ns3/building-penetration-loss.h"
#include "ns3/buildings-helper.h"
#include "ns3/class-c-end-device-lorawan-mac.h"
#include "ns3/command-line.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/correlated-shadowing-propagation-loss-model.h"
#include "ns3/double.h"
#include "ns3/end-device-lora-phy.h"
#include "ns3/forwarder-helper.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/gateway-lorawan-mac.h"
#include "ns3/log.h"
#include "ns3/lora-helper.h"
#include "ns3/lorawan-mac-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/network-server-helper.h"
#include "ns3/node-container.h"
#include "ns3/periodic-sender-helper.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/point-to-point-net-device.h"
#include "ns3/pointer.h"
#include "ns3/position-allocator.h"
#include "ns3/random-variable-stream.h"
#include "ns3/simulator.h"

#include <algorithm>
#include <ctime>

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE("ClassCRealisticChannelExample");

// Default simulation parameters
int nDevices = 50;
int nGateways = 1;
double radiusMeters = 3000;
double simulationTimeSeconds = 600;
bool realisticChannelModel = false;
int appPeriodSeconds = 60;
int packetSize = 20;
bool printBuildingInfo = false;

int
main(int argc, char* argv[])
{
    CommandLine cmd(__FILE__);
    cmd.AddValue("nDevices", "Number of Class C end devices", nDevices);
    cmd.AddValue("nGateways", "Number of gateways", nGateways);
    cmd.AddValue("radius", "Deployment radius [m]", radiusMeters);
    cmd.AddValue("simulationTime", "Simulation duration [s]", simulationTimeSeconds);
    cmd.AddValue("realisticChannel",
                 "Enable correlated shadowing + building penetration loss",
                 realisticChannelModel);
    cmd.AddValue("appPeriod", "Uplink inter-transmission period [s]", appPeriodSeconds);
    cmd.AddValue("packetSize", "Application payload size [bytes]", packetSize);
    cmd.AddValue("printBuildings", "Write building geometry to buildings.txt", printBuildingInfo);
    cmd.Parse(argc, argv);

    // Logging
    LogComponentEnable("ClassCRealisticChannelExample", LOG_LEVEL_ALL);
    // Uncomment for detailed debugging:
    // LogComponentEnable("ClassCEndDeviceLorawanMac", LOG_LEVEL_INFO);
    // LogComponentEnable("EndDeviceLoraPhy", LOG_LEVEL_ALL);
    // LogComponentEnable("GatewayLoraPhy", LOG_LEVEL_ALL);
    // LogComponentEnable("LoraInterferenceHelper", LOG_LEVEL_ALL);
    // LogComponentEnable("LoraChannel", LOG_LEVEL_INFO);
    // LogComponentEnable("NetworkServer", LOG_LEVEL_ALL);

    /************************
     *  Create the channel  *
     ************************/

    NS_LOG_INFO("Setting up channel model...");

    // Layer 1: Log-distance path loss (Magrin et al. parameters)
    Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel>();
    loss->SetPathLossExponent(3.76);
    loss->SetReference(1, 7.7);

    if (realisticChannelModel)
    {
        NS_LOG_INFO("Realistic channel: adding correlated shadowing + building penetration.");

        // Layer 2: Correlated shadowing (sigma=4 dB, d_corr=110 m)
        Ptr<CorrelatedShadowingPropagationLossModel> shadowing =
            CreateObject<CorrelatedShadowingPropagationLossModel>();
        loss->SetNext(shadowing);

        // Layer 3: Building penetration loss (3GPP TR 45.820)
        Ptr<BuildingPenetrationLoss> buildingLoss = CreateObject<BuildingPenetrationLoss>();
        shadowing->SetNext(buildingLoss);
    }
    else
    {
        NS_LOG_INFO("Basic channel: log-distance path loss only (n=3.76).");
    }

    Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel>();
    Ptr<LoraChannel> channel = CreateObject<LoraChannel>(loss, delay);

    /************************
     *  Create the helpers  *
     ************************/

    NS_LOG_INFO("Creating helpers...");

    LoraPhyHelper phyHelper = LoraPhyHelper();
    phyHelper.SetChannel(channel);

    LorawanMacHelper macHelper = LorawanMacHelper();

    LoraHelper helper = LoraHelper();
    helper.EnablePacketTracking();

    /************************
     *  Create end devices  *
     ************************/

    NS_LOG_INFO("Creating " << nDevices << " Class C end devices...");

    NodeContainer endDevices;
    endDevices.Create(nDevices);

    // Deploy devices uniformly on a disc
    MobilityHelper mobility;
    mobility.SetPositionAllocator("ns3::UniformDiscPositionAllocator",
                                  "rho",
                                  DoubleValue(radiusMeters),
                                  "X",
                                  DoubleValue(0.0),
                                  "Y",
                                  DoubleValue(0.0));
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(endDevices);

    // Set device height to 1.2 m (ground level)
    for (auto j = endDevices.Begin(); j != endDevices.End(); ++j)
    {
        Ptr<MobilityModel> mob = (*j)->GetObject<MobilityModel>();
        Vector position = mob->GetPosition();
        position.z = 1.2;
        mob->SetPosition(position);
    }

    // Configure Class C MAC
    uint8_t nwkId = 54;
    uint32_t nwkAddr = 1864;
    Ptr<LoraDeviceAddressGenerator> addrGen =
        CreateObject<LoraDeviceAddressGenerator>(nwkId, nwkAddr);

    macHelper.SetAddressGenerator(addrGen);
    phyHelper.SetDeviceType(LoraPhyHelper::ED);
    macHelper.SetDeviceType(LorawanMacHelper::ED_C);
    macHelper.SetRegion(LorawanMacHelper::EU);
    helper.Install(phyHelper, macHelper, endDevices);

    NS_LOG_INFO("Class C end devices created.");

    /*********************
     *  Create gateways  *
     *********************/

    NS_LOG_INFO("Creating " << nGateways << " gateway(s)...");

    NodeContainer gateways;
    gateways.Create(nGateways);

    // Gateway at center, 15 m height (tower)
    Ptr<ListPositionAllocator> gwAllocator = CreateObject<ListPositionAllocator>();
    gwAllocator->Add(Vector(0.0, 0.0, 15.0));
    mobility.SetPositionAllocator(gwAllocator);
    mobility.Install(gateways);

    phyHelper.SetDeviceType(LoraPhyHelper::GW);
    macHelper.SetDeviceType(LorawanMacHelper::GW);
    helper.Install(phyHelper, macHelper, gateways);

    /**********************
     *  Handle buildings  *
     **********************/

    double xLength = 130;
    double deltaX = 32;
    double yLength = 64;
    double deltaY = 17;
    int gridWidth = 2 * radiusMeters / (xLength + deltaX);
    int gridHeight = 2 * radiusMeters / (yLength + deltaY);
    if (!realisticChannelModel)
    {
        gridWidth = 0;
        gridHeight = 0;
    }

    Ptr<GridBuildingAllocator> gridBuildingAllocator;
    gridBuildingAllocator = CreateObject<GridBuildingAllocator>();
    gridBuildingAllocator->SetAttribute("GridWidth", UintegerValue(gridWidth));
    gridBuildingAllocator->SetAttribute("LengthX", DoubleValue(xLength));
    gridBuildingAllocator->SetAttribute("LengthY", DoubleValue(yLength));
    gridBuildingAllocator->SetAttribute("DeltaX", DoubleValue(deltaX));
    gridBuildingAllocator->SetAttribute("DeltaY", DoubleValue(deltaY));
    gridBuildingAllocator->SetAttribute("Height", DoubleValue(6));
    gridBuildingAllocator->SetBuildingAttribute("NRoomsX", UintegerValue(2));
    gridBuildingAllocator->SetBuildingAttribute("NRoomsY", UintegerValue(4));
    gridBuildingAllocator->SetBuildingAttribute("NFloors", UintegerValue(2));
    gridBuildingAllocator->SetAttribute(
        "MinX",
        DoubleValue(-gridWidth * (xLength + deltaX) / 2 + deltaX / 2));
    gridBuildingAllocator->SetAttribute(
        "MinY",
        DoubleValue(-gridHeight * (yLength + deltaY) / 2 + deltaY / 2));
    BuildingContainer bContainer = gridBuildingAllocator->Create(gridWidth * gridHeight);

    BuildingsHelper::Install(endDevices);
    BuildingsHelper::Install(gateways);

    // Assign spreading factors AFTER BuildingsHelper so the channel model
    // can access building information when computing received power.
    LorawanMacHelper::SetSpreadingFactorsUp(endDevices, gateways, channel);

    if (printBuildingInfo)
    {
        std::ofstream myfile;
        myfile.open("buildings.txt");
        std::vector<Ptr<Building>>::const_iterator it;
        int j = 1;
        for (it = bContainer.Begin(); it != bContainer.End(); ++it, ++j)
        {
            Box boundaries = (*it)->GetBoundaries();
            myfile << "set object " << j << " rect from " << boundaries.xMin << ","
                   << boundaries.yMin << " to " << boundaries.xMax << "," << boundaries.yMax
                   << std::endl;
        }
        myfile.close();
        NS_LOG_INFO("Building geometry written to buildings.txt");
    }

    /**********************************************
     *  Set up the end device sending application  *
     **********************************************/

    NS_LOG_INFO("Installing periodic sender (period=" << appPeriodSeconds
                << "s, payload=" << packetSize << "B)...");

    PeriodicSenderHelper appHelper = PeriodicSenderHelper();
    appHelper.SetPeriod(Seconds(appPeriodSeconds));
    appHelper.SetPacketSize(packetSize);
    ApplicationContainer appContainer = appHelper.Install(endDevices);

    appContainer.Start(Seconds(0));
    appContainer.Stop(Seconds(simulationTimeSeconds));

    /**************************
     *  Create network server *
     **************************/

    NS_LOG_INFO("Creating network server...");

    Ptr<Node> networkServer = CreateObject<Node>();

    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("5Mbps"));
    p2p.SetChannelAttribute("Delay", StringValue("2ms"));

    P2PGwRegistration_t gwRegistration;
    for (auto gw = gateways.Begin(); gw != gateways.End(); ++gw)
    {
        auto container = p2p.Install(networkServer, *gw);
        auto serverP2PNetDev = DynamicCast<PointToPointNetDevice>(container.Get(0));
        gwRegistration.emplace_back(serverP2PNetDev, *gw);
    }

    NetworkServerHelper nsHelper = NetworkServerHelper();
    nsHelper.SetGatewaysP2P(gwRegistration);
    nsHelper.SetEndDevices(endDevices);
    nsHelper.Install(networkServer);

    ForwarderHelper forHelper = ForwarderHelper();
    forHelper.Install(gateways);

    /****************
     *  Simulation  *
     ****************/

    // Verify Class C initialization
    int classCCount = 0;
    for (int i = 0; i < nDevices; i++)
    {
        Ptr<Node> node = endDevices.Get(i);
        Ptr<LoraNetDevice> loraDevice = DynamicCast<LoraNetDevice>(node->GetDevice(0));
        Ptr<ClassCEndDeviceLorawanMac> classCMac =
            DynamicCast<ClassCEndDeviceLorawanMac>(loraDevice->GetMac());
        if (classCMac)
        {
            classCCount++;
        }
        else
        {
            NS_LOG_ERROR("Device " << i << " failed to initialize as Class C!");
        }
    }
    NS_LOG_INFO(classCCount << "/" << nDevices << " devices confirmed as Class C.");

    Time stopTime = Seconds(simulationTimeSeconds) + Hours(1);
    Simulator::Stop(stopTime);

    NS_LOG_INFO("Running simulation ("
                << nDevices << " devices, "
                << nGateways << " GW, radius=" << radiusMeters << "m, "
                << (realisticChannelModel ? "REALISTIC" : "BASIC") << " channel)...");

    Simulator::Run();

    /***************************
     *  Print results          *
     ***************************/

    NS_LOG_INFO("Computing performance metrics...");

    LoraPacketTracker& tracker = helper.GetPacketTracker();

    std::cout << std::endl;
    std::cout << "=== Class C Realistic Channel Simulation Results ===" << std::endl;
    std::cout << "Channel model: " << (realisticChannelModel ? "Realistic (LogDist + Shadowing + Building)" : "Basic (LogDist only)") << std::endl;
    std::cout << "Devices: " << nDevices
              << "  Gateways: " << nGateways
              << "  Radius: " << radiusMeters << " m"
              << "  Duration: " << simulationTimeSeconds << " s"
              << "  Period: " << appPeriodSeconds << " s"
              << "  Payload: " << packetSize << " B" << std::endl;
    std::cout << std::endl;

    std::cout << "--- MAC-level (global) ---" << std::endl;
    std::cout << "Sent Received: "
              << tracker.CountMacPacketsGlobally(Seconds(0), stopTime) << std::endl;

    std::cout << std::endl;
    std::cout << "--- PHY-level (per gateway) ---" << std::endl;
    std::cout << "TotPktOnGW RecByGW IntfOnGW NoMoreDem UnderSens LostBcTx: "
              << tracker.PrintPhyPacketsPerGw(Seconds(0), stopTime, nGateways) << std::endl;

    Simulator::Destroy();

    return 0;
}
