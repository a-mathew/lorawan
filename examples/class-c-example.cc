/*
 * Class C LoRaWAN End Device Example
 *
 * This example demonstrates the use of Class C LoRaWAN end devices
 * with the lorawan ns-3 module. Class C devices have a continuously
 * open receive window (RX2), enabling low-latency downlink communication.
 *
 * This is particularly useful for scenarios requiring low-latency
 * downlink communication with multiple round-trip exchanges.
 */

#include "ns3/class-c-end-device-lorawan-mac.h"
#include "ns3/command-line.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/correlated-shadowing-propagation-loss-model.h"
#include "ns3/end-device-lora-phy.h"
#include "ns3/forwarder-helper.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/gateway-lorawan-mac.h"
#include "ns3/log.h"
#include "ns3/lora-helper.h"
#include "ns3/lora-net-device.h"
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
#include "ns3/string.h"

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE("ClassCExample");

int
main(int argc, char* argv[])
{
    // Set up logging
    LogComponentEnable("ClassCExample", LOG_LEVEL_ALL);
    LogComponentEnable("ClassCEndDeviceLorawanMac", LOG_LEVEL_INFO);
    // LogComponentEnable("NetworkScheduler", LOG_LEVEL_ALL);
    // LogComponentEnable("NetworkServer", LOG_LEVEL_ALL);

    /***********
     *  Setup  *
     ***********/

    // Network settings
    int nDevices = 5;
    int nGateways = 1;
    double simulationTimeSeconds = 600; // 10 minutes
    double sideLength = 1000;           // meters

    // Parse command line attributes
    CommandLine cmd(__FILE__);
    cmd.AddValue("nDevices", "Number of end devices to include in the simulation", nDevices);
    cmd.AddValue("simulationTime", "Simulation time [s]", simulationTimeSeconds);
    cmd.Parse(argc, argv);

    /************************
     *  Create the channel  *
     ************************/

    // Create the lora channel object
    Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel>();
    loss->SetPathLossExponent(3.76);
    loss->SetReference(1, 7.7);

    Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel>();

    Ptr<LoraChannel> channel = CreateObject<LoraChannel>(loss, delay);

    /************************
     *  Create the helpers  *
     ************************/

    NS_LOG_INFO("Creating helpers...");

    // Create the LoraPhyHelper
    LoraPhyHelper phyHelper = LoraPhyHelper();
    phyHelper.SetChannel(channel);

    // Create the LorawanMacHelper
    LorawanMacHelper macHelper = LorawanMacHelper();

    // Create the LoraHelper
    LoraHelper helper = LoraHelper();
    helper.EnablePacketTracking();

    /************************
     *  Create end devices  *
     ************************/

    NS_LOG_INFO("Creating end devices...");

    // Create a set of nodes
    NodeContainer endDevices;
    endDevices.Create(nDevices);

    // Assign a mobility model to each node
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator>();
    for (int i = 0; i < nDevices; i++)
    {
        double x = (i % 5) * (sideLength / 5) + 100;
        double y = (i / 5) * (sideLength / 5) + 100;
        allocator->Add(Vector(x, y, 1.2));
    }
    mobility.SetPositionAllocator(allocator);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(endDevices);

    // Create the LoraNetDevices of the end devices
    uint8_t nwkId = 54;
    uint32_t nwkAddr = 1864;
    Ptr<LoraDeviceAddressGenerator> addrGen =
        CreateObject<LoraDeviceAddressGenerator>(nwkId, nwkAddr);

    // Create the LoraNetDevices of the end devices — using Class C!
    macHelper.SetAddressGenerator(addrGen);
    phyHelper.SetDeviceType(LoraPhyHelper::ED);
    macHelper.SetDeviceType(LorawanMacHelper::ED_C); // <-- Class C!
    macHelper.SetRegion(LorawanMacHelper::EU);
    helper.Install(phyHelper, macHelper, endDevices);

    NS_LOG_INFO("Class C end devices created successfully.");

    /*********************
     *  Create gateways  *
     *********************/

    NS_LOG_INFO("Creating gateways...");

    NodeContainer gateways;
    gateways.Create(nGateways);

    Ptr<ListPositionAllocator> gwAllocator = CreateObject<ListPositionAllocator>();
    gwAllocator->Add(Vector(sideLength / 2, sideLength / 2, 15.0));
    mobility.SetPositionAllocator(gwAllocator);
    mobility.Install(gateways);

    // Create a netdevice for each gateway
    phyHelper.SetDeviceType(LoraPhyHelper::GW);
    macHelper.SetDeviceType(LorawanMacHelper::GW);
    helper.Install(phyHelper, macHelper, gateways);

    // Set spreading factors (MUST be after gateways are created)
    LorawanMacHelper::SetSpreadingFactorsUp(endDevices, gateways, channel);

    /**********************
     *  Handle buildings  *
     **********************/

    // (No buildings in this simple example)

    /**********************************************
     *  Set up the end device sending application  *
     **********************************************/

    NS_LOG_INFO("Creating applications...");

    // Create periodic senders — Class C devices also do uplinks
    PeriodicSenderHelper appHelper = PeriodicSenderHelper();
    appHelper.SetPeriod(Seconds(60)); // Send an uplink every 60 seconds
    appHelper.SetPacketSize(20);      // 20 bytes payload
    ApplicationContainer appContainer = appHelper.Install(endDevices);

    appContainer.Start(Seconds(0));
    appContainer.Stop(Seconds(simulationTimeSeconds));

    /**************************
     *  Create network server *
     **************************/

    NS_LOG_INFO("Creating network server...");

    // Create the network server node
    Ptr<Node> networkServer = CreateObject<Node>();

    // PointToPoint links between gateways and server
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("5Mbps"));
    p2p.SetChannelAttribute("Delay", StringValue("2ms"));

    // Store the P2P net devices for gateway registration
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

    // Create a forwarder for each gateway
    ForwarderHelper forHelper = ForwarderHelper();
    forHelper.Install(gateways);

    /****************
     *  Simulation  *
     ****************/

    // Verify that Class C continuous RX2 is active on end devices
    for (int i = 0; i < nDevices; i++)
    {
        Ptr<Node> node = endDevices.Get(i);
        Ptr<LoraNetDevice> loraDevice = DynamicCast<LoraNetDevice>(node->GetDevice(0));
        Ptr<ClassCEndDeviceLorawanMac> classCMac =
            DynamicCast<ClassCEndDeviceLorawanMac>(loraDevice->GetMac());
        if (classCMac)
        {
            NS_LOG_INFO("Device " << i << " is Class C. Continuous RX2 will be opened at t=0.");
        }
        else
        {
            NS_LOG_ERROR("Device " << i << " failed to initialize as Class C!");
        }
    }

    Simulator::Stop(Seconds(simulationTimeSeconds));

    NS_LOG_INFO("Running simulation...");
    Simulator::Run();

    NS_LOG_INFO("Computing performance metrics...");

    // Print results
    LoraPacketTracker& tracker = helper.GetPacketTracker();
    NS_LOG_INFO("Printing end devices' MAC-level stats:");
    NS_LOG_INFO("Sent Received");

    std::cout << tracker.CountMacPacketsGlobally(Seconds(0), Seconds(simulationTimeSeconds))
              << std::endl;

    Simulator::Destroy();

    return 0;
}
