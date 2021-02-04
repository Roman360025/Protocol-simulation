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
#include "ns3/one-shot-sender-helper.h"
#include <ctime>
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/stats-module.h"

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE ("ComplexLorawanNetworkExample");

int
main (int argc, char *argv[])
{



  CommandLine cmd;
  cmd.Parse (argc, argv);

  // Set up logging
   // LogComponentEnable ("SimpleLorawanNetworkExample", LOG_LEVEL_ALL);
  LogComponentEnable ("LoraChannel", LOG_LEVEL_INFO);
  LogComponentEnable ("LoraPhy", LOG_LEVEL_ALL);
  // LogComponentEnable ("EndDeviceLoraPhy", LOG_LEVEL_ALL);
  LogComponentEnable ("GatewayLoraPhy", LOG_LEVEL_ALL);
  LogComponentEnable ("LoraInterferenceHelper", LOG_LEVEL_ALL);
  LogComponentEnable ("LorawanMac", LOG_LEVEL_ALL);
  LogComponentEnable ("EndDeviceLorawanMac", LOG_LEVEL_ALL);
  LogComponentEnable ("ClassAEndDeviceLorawanMac", LOG_LEVEL_ALL);
  LogComponentEnable ("GatewayLorawanMac", LOG_LEVEL_ALL);
  LogComponentEnable ("LogicalLoraChannelHelper", LOG_LEVEL_ALL);
  LogComponentEnable ("LogicalLoraChannel", LOG_LEVEL_ALL);
  // LogComponentEnable ("LoraHelper", LOG_LEVEL_ALL);
  // LogComponentEnable ("LoraPhyHelper", LOG_LEVEL_ALL);
  // LogComponentEnable ("LorawanMacHelper", LOG_LEVEL_ALL);
  // LogComponentEnable ("OneShotSenderHelper", LOG_LEVEL_ALL);
  // LogComponentEnable ("OneShotSender", LOG_LEVEL_ALL);
  // LogComponentEnable ("LorawanMacHeader", LOG_LEVEL_ALL);
  // LogComponentEnable ("LoraFrameHeader", LOG_LEVEL_ALL);
  LogComponentEnableAll (LOG_PREFIX_FUNC);
  LogComponentEnableAll (LOG_PREFIX_NODE);
  LogComponentEnableAll (LOG_PREFIX_TIME);



   /************************
  *  Create End Devices  *
  ************************/
  NodeContainer nodes;
  nodes.Create (2);


    /************************
   *  Create the channel  *
   ************************/

  // Create the lora channel object


  Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel> ();
  loss->SetPathLossExponent (3.76);
  loss->SetReference (1, 7.7);

  Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel> ();

  Ptr<LoraChannel> channel = CreateObject<LoraChannel> (loss, delay);




  

   /************************
  *  Create the helpers  *
  ************************/
  // Create the LoraPhyHelper
        
  LoraPhyHelper phyHelper = LoraPhyHelper ();
  phyHelper.SetChannel (channel);

  // Create the LorawanMacHelper
  LorawanMacHelper macHelper = LorawanMacHelper ();

  

  // Create the LoraHelper
  LoraHelper helper = LoraHelper ();




  phyHelper.SetDeviceType (LoraPhyHelper::ED);
  macHelper.SetDeviceType (LorawanMacHelper::ED_A);

  helper.Install (phyHelper, macHelper, nodes);  



  

  /***********
   *  Setup  *
   ***********/


  OneShotSenderHelper oneShotHelper = OneShotSenderHelper ();
  oneShotHelper.SetSendTime (Seconds (10));
  oneShotHelper.Install (nodes.Get (1));

    /******************
   * Set Data Rates *
   ******************/
  // std::vector<int> sfQuantity (6);
  // sfQuantity = macHelper.SetSpreadingFactorsUp (nodes, channel);

   

   /************************
   *  Mobility  *
   ************************/
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
   NS_LOG_INFO ("IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII");


////////////////
  // Counter //
  ////////////////

  Ptr<PacketCounterCalculator> totalRx =
  CreateObject<PacketCounterCalculator>();
  totalRx->SetKey ("wifi-rx-frames");
  totalRx->SetContext ("node[1]");
  Config::Connect ("/NodeList/1/DeviceList/*/$ns3::LoraNetDevice/Mac/MacRx",
                   MakeCallback (&PacketCounterCalculator::PacketUpdate,
                                 totalRx));

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
