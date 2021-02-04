/*
 * This script simulates a complex scenario with multiple gateways and end
 * devices. The metric of interest for this script is the throughput of the
 * network.
 */

#include "ns3/end-device-lora-phy.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/class-a-end-device-lorawan-mac.h"
#include "ns3/gateway-lorawan-mac.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/pointer.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/lora-helper.h"
#include "ns3/node-container.h"
#include "ns3/mobility-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/double.h"
#include "ns3/random-variable-stream.h"
#include "ns3/periodic-sender-helper.h"
#include "ns3/command-line.h"
#include "ns3/network-server-helper.h"
#include "ns3/correlated-shadowing-propagation-loss-model.h"
#include "ns3/building-penetration-loss.h"
#include "ns3/building-allocator.h"
#include "ns3/buildings-helper.h"
#include "ns3/forwarder-helper.h"
#include "ns3/waypoint-mobility-model.h"
#include <algorithm>
#include <ctime>

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE ("ComplexLorawanNetworkExample");

// Network settings
int nDevices = 200;
int nGateways = 1;
double radius = 7500;
double simulationTime = 600;

// Channel model
bool realisticChannelModel = false;

int appPeriodSeconds = 600;

// Output control
bool print = true;

int
main (int argc, char *argv[])
{

  CommandLine cmd;
  cmd.AddValue ("nDevices", "Number of end devices to include in the simulation", nDevices);
  cmd.AddValue ("radius", "The radius of the area to simulate", radius);
  cmd.AddValue ("simulationTime", "The time for which to simulate", simulationTime);
  cmd.AddValue ("appPeriod",
                "The period in seconds to be used by periodically transmitting applications",
                appPeriodSeconds);
  cmd.AddValue ("print", "Whether or not to print various informations", print);
  cmd.Parse (argc, argv);

  // Set up logging
  LogComponentEnable ("ComplexLorawanNetworkExample", LOG_LEVEL_ALL);
  // LogComponentEnable("LoraChannel", LOG_LEVEL_INFO);
  // LogComponentEnable("LoraPhy", LOG_LEVEL_ALL);
  // LogComponentEnable("EndDeviceLoraPhy", LOG_LEVEL_ALL);
  // LogComponentEnable("GatewayLoraPhy", LOG_LEVEL_ALL);
  // LogComponentEnable("LoraInterferenceHelper", LOG_LEVEL_ALL);
  // LogComponentEnable("LorawanMac", LOG_LEVEL_ALL);
  // LogComponentEnable("EndDeviceLorawanMac", LOG_LEVEL_ALL);
  // LogComponentEnable("ClassAEndDeviceLorawanMac", LOG_LEVEL_ALL);
  // LogComponentEnable("GatewayLorawanMac", LOG_LEVEL_ALL);
  // LogComponentEnable("LogicalLoraChannelHelper", LOG_LEVEL_ALL);
  // LogComponentEnable("LogicalLoraChannel", LOG_LEVEL_ALL);
  // LogComponentEnable("LoraHelper", LOG_LEVEL_ALL);
  // LogComponentEnable("LoraPhyHelper", LOG_LEVEL_ALL);
  // LogComponentEnable("LorawanMacHelper", LOG_LEVEL_ALL);
  // LogComponentEnable("PeriodicSenderHelper", LOG_LEVEL_ALL);
  // LogComponentEnable("PeriodicSender", LOG_LEVEL_ALL);
  // LogComponentEnable("LorawanMacHeader", LOG_LEVEL_ALL);
  // LogComponentEnable("LoraFrameHeader", LOG_LEVEL_ALL);
  // LogComponentEnable("NetworkScheduler", LOG_LEVEL_ALL);
  // LogComponentEnable("NetworkServer", LOG_LEVEL_ALL);
  // LogComponentEnable("NetworkStatus", LOG_LEVEL_ALL);
  // LogComponentEnable("NetworkController", LOG_LEVEL_ALL);

  /***********
   *  Setup  *
   ***********/

  NodeContainer nodes;
  nodes.Create (2);

  MobilityHelper mobility;
  // mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
  //                                 "MinX", DoubleValue (0.0),
  //                                 "MinY", DoubleValue (0.0),
  //                                 "DeltaX", DoubleValue (20.0),
  //                                 "DeltaY", DoubleValue (10.0),
  //                                 "GridWidth", UintegerValue (3),
  //                                 "LayoutType", StringValue ("RowFirst"));
 

   
  mobility.SetMobilityModel("ns3::WaypointMobilityModel");

  mobility.Install(nodes);


   Ptr<WaypointMobilityModel> ueWaypointMobility = DynamicCast<WaypointMobilityModel>( nodes.Get(1)->GetObject<MobilityModel>());
   // ueWaypointMobility->SetPosition(Vector (0, 0, 0));
  
   //Первая половина окружности
  ueWaypointMobility->AddWaypoint(Waypoint(Seconds(1.0),Vector(0, 0,0)));
  ueWaypointMobility->AddWaypoint(Waypoint(Seconds(2.0),Vector(-20, -40,0)));
  ueWaypointMobility->AddWaypoint(Waypoint(Seconds(3.0), Vector(-40, -48.99, 0)));
  ueWaypointMobility->AddWaypoint (Waypoint (Seconds (4.0),Vector (-50, -50, 0)));
  ueWaypointMobility->AddWaypoint(Waypoint (Seconds (5.0),Vector (-60, -48.99, 0)));
  ueWaypointMobility->AddWaypoint(Waypoint (Seconds (6.0),Vector (-80, -40, 0)));
  ueWaypointMobility->AddWaypoint(Waypoint (Seconds (7.0),Vector (-100, 0, 0)));
  //Вторая половина окружности
  ueWaypointMobility->AddWaypoint(Waypoint(Seconds(8.0),Vector(-80, 40,0)));
  ueWaypointMobility->AddWaypoint(Waypoint(Seconds(9.0),Vector(-60, 48.99, 0)));
  ueWaypointMobility->AddWaypoint(Waypoint(Seconds(10.0), Vector(-50, 50, 0)));
  ueWaypointMobility->AddWaypoint (Waypoint (Seconds (11.0),Vector (-40, 48.99, 0)));
  ueWaypointMobility->AddWaypoint(Waypoint (Seconds (12.0),Vector (-20, 40, 0)));
  ueWaypointMobility->AddWaypoint(Waypoint (Seconds (13.0),Vector (0, 0, 0)));

   

  ////////////////
  // Simulation //
  ////////////////

  Simulator::Stop (Hours (1));

  NS_LOG_INFO ("Running simulation...");
  Simulator::Run ();

  Simulator::Destroy ();

  ///////////////////////////
  // Print results to file //
  ///////////////////////////
  NS_LOG_INFO ("Computing performance metrics...");



  return 0;
}
