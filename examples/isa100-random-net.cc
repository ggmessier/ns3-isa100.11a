/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 The University Of Calgary- FISHLAB
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author:   Michael Herrmann <mjherrma@ucalgary.ca>
             Rajith Madduma Bandarage <rajith.maddumabandar@ucalgary.ca>
 */

#include "ns3/core-module.h"
#include "ns3/spectrum-module.h"
#include "ns3/isa100-11a-module.h"

#include "ns3/trace-helper.h"

#include <iomanip>
//#include "ns3/ascii-file.h"
#include <iostream>
#include <fstream>

// ************************************************** DEFINES *************************************************
// Defines for simulation
#define SIM_DURATION_S 1e9                  // Duration of simulation in (s) (really long so energy runs out)

// Defines for channel
#define PATH_LOSS_EXP 2.91                  // Path loss exponent from jp measurements
//#define SHADOWING_STD_DEV_DB 0.0           // Shadowing standard deviation from jp measurements (dB)
#define SHADOWING_STD_DEV_DB 4.58           // Shadowing standard deviation from jp measurements (dB)

// Topology
#define SENSOR_DENSITY 0.0093  // Nodes/m^2
#define MIN_NODE_SPACING 3.0               // Node spacing is at least this distance (m)
#define FIELD_SIZE_X 60.0  // Field size in the x direction.
#define FIELD_SIZE_Y 90.0  // Field size in the y direction.


// Defines for node applications
#define SENSOR_SAMPLE_DURATION_S  0.10     // Duration of a sensor sample (s)
#define SENSOR_SAMPLE_POWER_W     0.027    // Power required for performing a sensor sample (W)
#define PACKET_DATA_BYTES         40       // Size of Packet's data payload (bytes)
#define PACKET_OVERHEAD_BYTES 29 // Number of overhead bytes in a packet
#define SENSOR_SAMPLE_PERIOD 2.0 // Sample period (s)
#define TX_EARLIEST_S 2.212e-3  // Transmit dead time at the start of each timeslot (ms)

// DL layer defines
#define SINK_ADDR "00:00"                  // Data sink address

// Phy layer defines
#define DEFAULT_INITIAL_ENERGY_J 3.0       // Originally: 5J Default initial energy each sensor node has available (J)


#define RX_SENSITIVITY -101.0 // Receiver sensitivity (dBm)


// Macros
#define CREATE_STREAM_FILENAME(fnstr){  \
	ss.str( std::string() ); \
	ss.clear(); \
	ss << filePrefix << fnstr; \
	filename = ss.str(); \
	}

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE ("RandomNetworkTdma");

// Global variables for simulation termination.
double networkLifetime;
Time terminateCheckPeriod;
int terminateSim = 0;
int totalFailedNodesCount = 0;
int initialTotalFailedNodesCount = 0;

// ************************ CALLBACK FUNCTIONS ******************************

void BatteryDepletionCallbackEvent(Mac16Address addr)
{
	if(!terminateSim){
		networkLifetime = (Simulator::Now()).GetSeconds();
		NS_LOG_UNCOND(" Node " << addr << " out of energy at " << networkLifetime);
		terminateSim = 1;
	}

//	Simulator::Stop();
}



static void TerminateSimulation()
{
	if(terminateSim){
		NS_LOG_UNCOND(" Simulation terminated!");
		Simulator::Stop();
	}
	else
	  Simulator::Schedule(terminateCheckPeriod,&TerminateSimulation);
}

static void PrintDropPacket ( Ptr<OutputStreamWrapper> stream, Mac16Address addr, Ptr<const Packet> p, std::string message)
{
  *stream->GetStream() << Simulator::Now ().GetNanoSeconds () << "," << addr << ",#" << message << "#," << *p << std::endl;
}

vector<int> reportTxNum;
vector<int> reportRxNum;
vector<Time> reportTxTime;
vector<Time> reportTotalDelay;


static void LogReportTx(Ptr<OutputStreamWrapper> stream, Mac16Address addr )
{
	uint8_t buff[2];
	addr.CopyTo(buff);
	int nodeInd = ( (uint)buff[0] << 8 ) + (uint)buff[1];

	reportTxNum[nodeInd]++;
	reportTxTime[nodeInd] = Simulator::Now();
        //NS_LOG_UNCOND("Log report Tx: "<<reportTxNum[nodeInd]);

//	*stream->GetStream() << "Tx: "<< Simulator::Now().GetMilliSeconds() << ", " << addr << std::endl;
}

static void LogReportRx(Ptr<OutputStreamWrapper> stream, Mac16Address addr )
{
	uint8_t buff[2];
	addr.CopyTo(buff);
	int nodeInd = ( (uint)buff[0] << 8 ) + (uint)buff[1];

	reportRxNum[nodeInd]++;
	reportTotalDelay[nodeInd] += Simulator::Now() - reportTxTime[nodeInd];
        //NS_LOG_UNCOND("Log report Rx: "<<reportRxNum[nodeInd]);

//	*stream->GetStream() << "Rx: "<< Simulator::Now().GetMilliSeconds() << ", " << addr << std::endl;
}

static void LogHops(Ptr<OutputStreamWrapper> stream, vector<int> hops)
{
	double avgHops = 0;
	for(unsigned int iHop=0; iHop < hops.size(); iHop++) //Rajith 0408 Changed int to unsigned int
		avgHops += hops[iHop];

	*stream->GetStream() << "AvgHops," << avgHops/hops.size() << std::endl;
}

/* Rajith 0408 Not used
static void PrintLocations(Ptr<OutputStreamWrapper> stream, int node, double x, double y, double z)
{
	float distToSink = sqrt((FIELD_SIZE_X/2-x)*(FIELD_SIZE_X/2-x) + y*y);
	*stream->GetStream() << "Node " << node << ": (" << x << "," << y << ") " << distToSink << "m from sink." << std::endl;
}
*/ // Rajith 0408 Not used

// Rajith function for the simulation ====================================================================================================================
//========================================================================================================================================================
//========================================================================================================================================================

struct NodeFailedInformation
{
        vector<double> energyOfBatteries;
        Ptr<ListPositionAllocator> positionAlloc;
        vector<bool> allnodesFailedStatus;
};

struct NodeFailedInformation NodeFailureFunction(bool initialCall, uint32_t seed, int iter, unsigned int numSensorNodes, uint16_t optimizerType, std::string optString, Time slotDuration, Ptr<ListPositionAllocator> positionAlloc, vector<bool> allnodesFailedStatus, vector<double> energyOfBatteries) 
{
  terminateSim = 0;
  bool multiplePacketsPerSlot = false;
  unsigned int numSlotsPerFrame;
  NodeFailedInformation nodeFailedInfo; //Rajith Nodefailure struct 0408
  nodeFailedInfo.positionAlloc = positionAlloc; //Rajith position allocation info 0408

  numSlotsPerFrame = ceil(SENSOR_SAMPLE_PERIOD / slotDuration.GetSeconds());

  NS_LOG_UNCOND("Optimization: " << optString << ", Iter: " << iter);
  NS_LOG_UNCOND("Slot Duration: " << slotDuration.GetSeconds() << "s, Slots Per Superframe: " << numSlotsPerFrame);

  terminateCheckPeriod = Seconds(numSlotsPerFrame*slotDuration.GetSeconds());
  Simulator::Schedule(terminateCheckPeriod/2,&TerminateSimulation);

  NS_LOG_UNCOND("Sample update period " << terminateCheckPeriod.GetSeconds() << " s");

  NS_ASSERT(numSensorNodes > 0);
  NS_ASSERT(iter >= 0);

  AsciiTraceHelper asciiTraceHelper;
  std::string filename;
  std::stringstream ss;

  std::string filePath = "/home/rajith/NS-3 Rajith/Results/";
	ss.str( std::string() );
	ss.clear();
	ss << filePath << "N" << numSensorNodes << "_" << optString << "_"<< initialTotalFailedNodesCount << "_"; //Rajith 0408
  std::string filePrefix = ss.str();

  uint16_t numNodes = 1 + numSensorNodes;
  double fieldSizeY = 0;
  
  fieldSizeY = ( (double)numSensorNodes / SENSOR_DENSITY ) / FIELD_SIZE_X;
  
  nodeFailedInfo.allnodesFailedStatus = allnodesFailedStatus; //Rajith Nodefailure status update 0408
  nodeFailedInfo.energyOfBatteries = energyOfBatteries;

  uint16_t newNumNodes = numNodes - initialTotalFailedNodesCount;
  // routing debug
  //  numNodes = 6;

  reportTxNum.assign(newNumNodes,0);
  reportRxNum.assign(newNumNodes,0);
  reportTxTime.assign(newNumNodes,Seconds(0.0));
  reportTotalDelay.assign(newNumNodes,Seconds(0.0));

  NS_LOG_UNCOND("Number of Nodes: " << numNodes);


  if (!newNumNodes){
    NS_FATAL_ERROR("Number of transmit nodes cannot be zero!");
  }

  // Cannot simulate more than 256 nodes
  NS_ASSERT_MSG(newNumNodes <= 256, "Simulation can only support upto 256 nodes total. Num Nodes = " << numNodes);

	// Change the random number seed to alter the random number sequence used by the simulator.
  RngSeedManager::SetSeed (seed);
  NS_LOG_UNCOND("Seed: " << seed);
  
  // ********************************************* HELPER ************************************************

  Ptr<Isa100Helper> isaHelper = CreateObject<Isa100Helper>();
	
  // ******************************************* DL & PHY ATTRIBUTES ******************************************** 

  // These are ISA100 frame size parameters.  Look in the ISA100 standard for more information.
  isaHelper->SetDlAttribute("SuperFramePeriod",UintegerValue(numSlotsPerFrame));
  isaHelper->SetDlAttribute("SuperFrameSlotDuration",TimeValue(slotDuration));

  // Dl attributes
  isaHelper->SetDlAttribute("MaxTxPowerDbm", IntegerValue(4));
  isaHelper->SetDlAttribute("MinTxPowerDbm", IntegerValue(-17));
  isaHelper->SetDlAttribute("DlSleepEnabled", BooleanValue(true));

  // Phy attributes
  isaHelper->SetPhyAttribute ("SupplyVoltage", DoubleValue (3.0));
  isaHelper->SetPhyAttribute ("SensitivityDbm", DoubleValue (RX_SENSITIVITY));

  // Phy power consumption attributes
  isaHelper->SetTrxCurrentAttribute ("TrxOffCurrentA", DoubleValue (0.0003));
  isaHelper->SetTrxCurrentAttribute ("RxOnCurrentA", DoubleValue (0.0118));
  isaHelper->SetTrxCurrentAttribute ("SleepCurrentA", DoubleValue (0.0000002));

  isaHelper->SetTrxCurrentAttribute ("BusyRxCurrentA", DoubleValue (0.0118));
  isaHelper->SetTrxCurrentAttribute ("TxOnCurrentA", DoubleValue (0.0052));
  isaHelper->SetTrxCurrentAttribute ("Slope", DoubleValue (0.0003013));
  isaHelper->SetTrxCurrentAttribute ("Offset", DoubleValue (0.01224));

  // ********************************************* CHANNEL MODEL ************************************************

  NS_LOG_UNCOND("Constructing the channel model...");
  Ptr<SingleModelSpectrumChannel> channel = CreateObject<SingleModelSpectrumChannel> ();
  Ptr<FishLogDistanceLossModel> propLossModel = CreateObject<FishLogDistanceLossModel> ();
  Ptr<ConstantSpeedPropagationDelayModel> propDelayModel = CreateObject<ConstantSpeedPropagationDelayModel> ();

  propLossModel->SetAttribute("PathLossExponent",DoubleValue(PATH_LOSS_EXP));
  propLossModel->SetAttribute("ShadowingStdDev",DoubleValue(SHADOWING_STD_DEV_DB));
  channel->AddPropagationLossModel (propLossModel);
  channel->SetPropagationDelayModel(propDelayModel);

  // Channel hopping, channels 11-26 are available in the hopping pattern (802.15.4 channel page 0, OQPSK)
  // Nodes will operate on channel 11.
  uint8_t hoppingPattern[] = { 11 };

  // ********************************************* FILE STREAMS ************************************************
  Ptr<OutputStreamWrapper> scheduleStream = asciiTraceHelper.CreateFileStream ("/dev/null",std::ios::out);

  CREATE_STREAM_FILENAME("energies.txt");
  Ptr<OutputStreamWrapper> energyStream = asciiTraceHelper.CreateFileStream (filename,std::ios::app);

  CREATE_STREAM_FILENAME("drops.txt");
  Ptr<OutputStreamWrapper> packetDropStream = asciiTraceHelper.CreateFileStream (filename,std::ios::app);

  CREATE_STREAM_FILENAME("reports.txt");
  Ptr<OutputStreamWrapper> reportStream = asciiTraceHelper.CreateFileStream (filename,std::ios::app);

//  CREATE_STREAM_FILENAME("locations.txt");
//  Ptr<OutputStreamWrapper> locationStream = asciiTraceHelper.CreateFileStream (filename,std::ios::app);

/*  CREATE_STREAM_FILENAME("schedule.txt");
  Ptr<OutputStreamWrapper> scheduleStream = asciiTraceHelper.CreateFileStream (filename,std::ios::app);
*/

	*(energyStream->GetStream()) << "Iter," << iter << ",--------------\n";
	*(packetDropStream->GetStream()) << "Iter," << iter << ",--------------\n";
	*(reportStream->GetStream()) << "Iter," << iter << ",--------------\n";
//	*(locationStream->GetStream()) << "#" << iter << "#\n";
//	*(scheduleStream->GetStream()) << "#" << iter << "#\n";


	*(reportStream->GetStream()) << "Seed," << seed << "\n";

	isaHelper->TraceConnectWithoutContext ("HopTrace", MakeBoundCallback (&LogHops, reportStream));

	// ********************************************** NODE LOCATIONS **********************************************
        if(initialCall){ //Rajith 0408
	        NS_LOG_UNCOND(" Creating network...");

        //  isaHelper->TraceConnectWithoutContext ("NodeLocations", MakeBoundCallback (&PrintLocations, locationStream));


	        //Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>(); //Rajith 0407

	        ns3::Vector sinkLoc(FIELD_SIZE_X/2,0.0,0.0);

	        isaHelper->GenerateLocationsFixedNumNodes(positionAlloc,numNodes,FIELD_SIZE_X,fieldSizeY,MIN_NODE_SPACING,sinkLoc);

	        *(reportStream->GetStream()) << "FieldArea," << FIELD_SIZE_X*fieldSizeY << "\n";
	        *(reportStream->GetStream()) << "FieldRatio," << fieldSizeY/FIELD_SIZE_Y << "\n";
                //}
        } //Rajith 0408

        propLossModel->GenerateNewShadowingValues(positionAlloc,newNumNodes,SHADOWING_STD_DEV_DB,nodeFailedInfo.allnodesFailedStatus);
        

	// ********************************************* CREATE NODES **********************************************

        // Nodes 'container' class.
        NodeContainer nc = NodeContainer();
        nc.Create(newNumNodes);

	NetDeviceContainer devContainer = NetDeviceContainer();
	devContainer = isaHelper->Install(nc, channel, 0);

	isaHelper->SetDeviceConstantPosition(devContainer,positionAlloc,nodeFailedInfo.allnodesFailedStatus);

        int j = 1;
	for (int16_t i = 1; i < numNodes; i++)
	{
                if(!nodeFailedInfo.allnodesFailedStatus[i]){
                        Ptr<Isa100Processor> processor = CreateObject<Isa100Processor>();

		        processor->SetAttribute("ActiveCurrent", DoubleValue(0.0078));
		        processor->SetAttribute("SleepCurrent", DoubleValue(0.0000026));
		        processor->SetAttribute("SupplyVoltage", DoubleValue(3.0));

		        isaHelper->InstallProcessor(j,processor);

		        Ptr<Isa100Sensor> sensor = CreateObject<Isa100Sensor>();

		        sensor->SetAttribute("ActiveCurrent", DoubleValue(SENSOR_SAMPLE_POWER_W/3.0));
		        sensor->SetAttribute("IdleCurrent", DoubleValue(0.0));
		        sensor->SetAttribute("SupplyVoltage", DoubleValue(3.0));
		        sensor->SetAttribute("SensingTime", TimeValue( Seconds(SENSOR_SAMPLE_DURATION_S) ) );

		        isaHelper->InstallSensor(j,sensor);

		        Ptr<Isa100Battery> battery = CreateObject<Isa100Battery>();

                        battery->SetInitEnergy(nodeFailedInfo.energyOfBatteries[i]);
		
                        battery->SetBatteryDepletionCallback(MakeCallback(&BatteryDepletionCallbackEvent));

		        isaHelper->InstallBattery(j,battery);
                        j++;
                }
	}

	// Sink application
	Ptr<Isa100BackboneNodeApplication> sinkNodeApp = CreateObject<Isa100BackboneNodeApplication>();

	sinkNodeApp->SetAttribute("SrcAddress",Mac16AddressValue(SINK_ADDR));
	sinkNodeApp->SetAttribute("StartTime",TimeValue(Seconds(0.0)));
        sinkNodeApp->TraceConnectWithoutContext ("ReportRx", MakeBoundCallback (&LogReportRx, reportStream));



	// Install application
	isaHelper->InstallApplication(nc,0,sinkNodeApp);

	// Create the sensor node applications
	Mac16AddressValue address;
	Ptr<Isa100NetDevice> netDevice;

	for (int16_t i = 1; i < newNumNodes; i++)
	{
		Ptr<Isa100FieldNodeApplication> sensorNodeApp = CreateObject<Isa100FieldNodeApplication>();

		// Sensor application attributes
		netDevice = devContainer.Get(i)->GetObject<Isa100NetDevice>();
		netDevice->GetDl()->GetAttribute("Address",address);
		sensorNodeApp->SetAttribute("SrcAddress",address);
		sensorNodeApp->SetAttribute("DestAddress",Mac16AddressValue(SINK_ADDR));
		sensorNodeApp->SetAttribute("PacketSize",UintegerValue(PACKET_DATA_BYTES));
		sensorNodeApp->SetAttribute("StartTime",TimeValue(Seconds(0.0)));
	        sensorNodeApp->TraceConnectWithoutContext ("ReportTx", MakeBoundCallback (&LogReportTx, reportStream));

		// Hook the application and sensor toegether
		sensorNodeApp->SetSensor(netDevice->GetSensor());
		sensorNodeApp->SetProcessor(netDevice->GetProcessor());
		netDevice->GetSensor()->SetSensingCallback(MakeCallback (&Isa100FieldNodeApplication::SensorSampleCallback, sensorNodeApp));

		// Install application
		isaHelper->InstallApplication(nc,i,sensorNodeApp);
	}

	// Traces
  Ptr<NetDevice> baseDevice;
  for (uint16_t i = 0; i < newNumNodes; i++){
    baseDevice = devContainer.Get(i);
    netDevice = baseDevice->GetObject<Isa100NetDevice>();

    netDevice->GetPhy()->TraceConnectWithoutContext ("InfoDropTrace", MakeBoundCallback (&PrintDropPacket, packetDropStream));
    netDevice->GetDl()->TraceConnectWithoutContext ("InfoDropTrace", MakeBoundCallback (&PrintDropPacket, packetDropStream));
  }


  // ******************************************** TDMA OPTIMIZATION *********************************************
  NS_LOG_UNCOND(" Beginning TDMA lifetime optimization...");

  // Optimizer Attributes
  isaHelper->SetTdmaOptAttribute("MultiplePacketsPerSlot", BooleanValue(multiplePacketsPerSlot));
  isaHelper->SetTdmaOptAttribute("NumBytesPkt", UintegerValue (PACKET_DATA_BYTES + PACKET_OVERHEAD_BYTES)); // 29 bytes is the isa100 header size
  isaHelper->SetTdmaOptAttribute("NumPktsNode", UintegerValue (1));
  isaHelper->SetTdmaOptAttribute("SensitivityDbm", DoubleValue (RX_SENSITIVITY));

  // Call the helper
  clock_t begin = clock();
  SchedulingResult schedResult = isaHelper->CreateOptimizedTdmaSchedule(nc,propLossModel,hoppingPattern,1,(OptimizerSelect)optimizerType,scheduleStream);
  clock_t end = clock();

  if(schedResult != SCHEDULE_FOUND){
    *(reportStream->GetStream()) << "Failure," << schedResult << "\n";

    energyStream->GetStream()->flush();
    packetDropStream->GetStream()->flush();
    scheduleStream->GetStream()->flush();
    reportStream->GetStream()->flush();

    return nodeFailedInfo;
  }


  double optTime = ((double)end-(double)begin)/CLOCKS_PER_SEC;
  NS_LOG_UNCOND("  Optimization Time: " << optTime << " s");
  *(reportStream->GetStream()) << "Optimization," << optTime << "\n";


  // ********************************************** RUN SIMULATION **********************************************
  Simulator::Stop (Seconds (SIM_DURATION_S));
  NS_LOG_UNCOND (" Simulation is running ....");
  Simulator::Run ();


  // ************************************************* SIMULATION COMPLETE **************************************************

  int totReportTx = 0, totReportRx = 0;
  Time totDelay = Seconds(0.0);
  bool starvedNode = false;
  double batteryResidualEnergy = 0;       

  int k = 1;
  for (int16_t i = 1; i < numNodes; i++){
        if(!nodeFailedInfo.allnodesFailedStatus[i]){
  	baseDevice = devContainer.Get(k);
  	netDevice = baseDevice->GetObject<Isa100NetDevice>();
        //v = positionAlloc->GetNext();

  	netDevice->GetBattery()->PrintEnergySummary(energyStream);
        batteryResidualEnergy = netDevice->GetBattery()->GetEnergy();

        nodeFailedInfo.energyOfBatteries[i] = batteryResidualEnergy;
        //NS_LOG_UNCOND("batteryResidualEnergy: "<<k<<", "<<batteryResidualEnergy); //Rajith battery residual energy
        if(batteryResidualEnergy==0)  nodeFailedInfo.allnodesFailedStatus[i] = true;

  	totReportTx += reportTxNum[k];
  	totReportRx += reportRxNum[k];
  	totDelay += reportTotalDelay[k];

  	if(reportRxNum[k] == 0){
  		starvedNode = true;
  		NS_LOG_UNCOND("*Starved Node*: " << k);
  	}
        k++;
        }
 // 	NS_LOG_UNCOND(" Node " << i << ", Tx: " << reportTxNum[i] << " Rx: " << reportRxNum[i]);

  }

  if(starvedNode)
    *(reportStream->GetStream()) << "Failure," << STARVED_NODE << "\n";
  else{
    *(reportStream->GetStream()) << "Lifetime," << networkLifetime << "\n";
  	*(reportStream->GetStream()) << "TotalTx," << totReportTx << "\n";
  	*(reportStream->GetStream()) << "TotalRx," << totReportRx << "\n";
  	*(reportStream->GetStream()) << "DropPct," << (1.0-(double)totReportRx/totReportTx) << "\n";
  	*(reportStream->GetStream()) << "AvgDelay," << totDelay.GetSeconds()/totReportRx << "\n";
  }

  // Flush streams
  energyStream->GetStream()->flush();
  packetDropStream->GetStream()->flush();
  scheduleStream->GetStream()->flush();
  reportStream->GetStream()->flush();
 // locationStream->GetStream()->flush();
 return nodeFailedInfo;
}
// end of Rajith function for the simulation =============================================================================================================
//========================================================================================================================================================
//========================================================================================================================================================
// end of 

// ************************************************ MAIN BEGIN ************************************************
int main (int argc, char *argv[])
{

  // *************** Command Line Arguments ***************
  uint32_t seed = 1002;
  std::string optString;
  int numSensorNodes=0;

  int iter = -1;

  CommandLine cmd;
  cmd.AddValue("rndSeed", "Seed for random number generation.", seed);
  cmd.AddValue("iter", "Iteration number.", iter);
  cmd.AddValue("nnodes", "Number of sensor nodes.",numSensorNodes);
//  cmd.AddValue("optType", "0 = min hop, 1 = Goldsmith, 2 = Convex Int", optimizerType);
  cmd.AddValue("optType","Optimization type: MinHop10ms, MinHopPckt, Goldsmith10ms, GoldsmithPckt, ConvInt10ms, ConvIntPckt",optString);

  cmd.Parse (argc, argv);

  uint16_t optimizerType;
  Time slotDuration;


  if(optString == "MinHop10ms"){
  	optimizerType = TDMA_MIN_HOP;
  	slotDuration = MilliSeconds(10);
  }
  else if(optString == "MinHopPckt"){
  	optimizerType = TDMA_MIN_HOP;
  	slotDuration = Seconds((double)(PACKET_OVERHEAD_BYTES+PACKET_DATA_BYTES) * 8 / 250e3 + TX_EARLIEST_S);
  }
  else if(optString == "Goldsmith10ms"){
  	optimizerType = TDMA_GOLDSMITH;
  	slotDuration = MilliSeconds(10);
  }
  else if(optString == "GoldsmithPckt"){
  	optimizerType = TDMA_GOLDSMITH;
  	slotDuration = Seconds((double)(PACKET_OVERHEAD_BYTES+PACKET_DATA_BYTES) * 8 / 250e3 + TX_EARLIEST_S);
  }
  else if(optString == "ConvInt10ms"){
  	optimizerType = TDMA_CONVEX_INT;
  	slotDuration = MilliSeconds(10);
  }
  else if(optString == "ConvIntPckt"){
  	optimizerType = TDMA_CONVEX_INT;
  	slotDuration = Seconds((double)(PACKET_OVERHEAD_BYTES+PACKET_DATA_BYTES) * 8 / 250e3 + TX_EARLIEST_S);
  }
  else
  	NS_FATAL_ERROR("Command line optimization string incorrect.");

  // end of *************** Command Line Arguments ***************

  // *************** Reading Files for Energies and Locations ***************

   std::string filePath = "/home/rajith/NS-3 Rajith/InOut/";
   bool initialCall = true;
   vector<double> energyOfBatteries(numSensorNodes+1,DEFAULT_INITIAL_ENERGY_J*1e6);
   vector<bool> allnodesFailedStatus(numSensorNodes+1);
   std::string line;
   Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
   std::stringstream ss;

   //Reading Energies
   ss.str(std::string());
   ss.clear(); 
   ss << filePath <<"IO_residualEnergies.txt";
   ifstream myfile (ss.str());
   int nNode = 1;
   if (myfile.is_open())
   {
    while (getline (myfile,line))
    {
       initialCall = false;
       istringstream buffer(line);
       buffer >> energyOfBatteries[nNode];
       if(energyOfBatteries[nNode]==0) {
                allnodesFailedStatus[nNode] = true;
                initialTotalFailedNodesCount++;
       }
       nNode++;
     }
    myfile.close();
   }else NS_LOG_UNCOND("Unable to open the file: "<<ss.str()); 
   // end of Reading Energies

   //Reading Locations
   double loc[3];
   ss.str(std::string());
   ss.clear(); 
   ss << filePath <<"IO_positionAlloc.txt";
   ifstream myfile2 (ss.str());
   if (myfile2.is_open())
   {
    while (getline (myfile2,line))
    {
       istringstream buffer(line);
       buffer >> loc[0];
       buffer >> loc[1];
       buffer >> loc[2];

       positionAlloc->Add(Vector(loc[0],loc[1],loc[2]));
     }
    myfile2.close();
   }else NS_LOG_UNCOND("Unable to open the file: "<<ss.str()); 
   // end of Reading Locations    

  // end of *************** Reading Files for Energies and Locations ***************

  // *************** Calling NodeFailureFunction ***************
   NodeFailedInformation Nf = NodeFailureFunction(initialCall, seed, iter, numSensorNodes, optimizerType, optString, slotDuration, positionAlloc, allnodesFailedStatus, energyOfBatteries); //Rajith 0407
   
   for(int i=1; i < numSensorNodes+1; i++){
        if(Nf.allnodesFailedStatus[i]) totalFailedNodesCount++;
   }

   NS_LOG_UNCOND("Total faild node count: "<<totalFailedNodesCount);
   NS_LOG_UNCOND("----------------------------------------------------------------------------------");

  // end of *************** Calling NodeFailureFunction ***************
   
  // *************** Writing Files for Energies and Locations ***************

   AsciiTraceHelper asciiTraceHelper;
   //Writing Energies
   std::string filename = "";
   ss.str( std::string() );
   ss.clear();
   ss << filePath << "N" << numSensorNodes << "_" << optString << "_";
   std::string filePrefix = ss.str();

   Ptr<OutputStreamWrapper> scheduleStream = asciiTraceHelper.CreateFileStream ("/dev/null",std::ios::out);

   CREATE_STREAM_FILENAME("residualEnergies.txt");
   Ptr<OutputStreamWrapper> residualEnergies = asciiTraceHelper.CreateFileStream (filename,std::ios::app);
   for(int i=1; i < numSensorNodes+1; i++){
        *(residualEnergies->GetStream()) << Nf.energyOfBatteries[i] << "\n";
   }
   
   residualEnergies->GetStream()->flush();

   //std::ifstream src(filename, std::ios::binary);
   filename = "";
   ss.str( std::string() );
   ss.clear();
   ss << filePath << "IO" << "_";
   filePrefix = ss.str();

   //Ptr<OutputStreamWrapper> scheduleStream = asciiTraceHelper.CreateFileStream ("/dev/null",std::ios::out);
   CREATE_STREAM_FILENAME("residualEnergies.txt");
   OutputStreamWrapper(filename,std::ios::trunc); //Rajith 0410
   Ptr<OutputStreamWrapper> residualEnergies_cp = asciiTraceHelper.CreateFileStream (filename,std::ios::app);
   for(int i=1; i < numSensorNodes+1; i++){
        *(residualEnergies_cp->GetStream()) << Nf.energyOfBatteries[i] << "\n";
   }
   
   residualEnergies_cp->GetStream()->flush();
   //end of Writing Energies

   //Writing Locations
   filename = "";
   ss.str( std::string() );
   ss.clear();
   ss << filePath << "N" << numSensorNodes << "_" << optString << "_";
   filePrefix = ss.str();

   //Ptr<OutputStreamWrapper> scheduleStream = asciiTraceHelper.CreateFileStream ("/dev/null",std::ios::out);

   CREATE_STREAM_FILENAME("positionAlloc.txt");
   Ptr<OutputStreamWrapper> positionAllocOut = asciiTraceHelper.CreateFileStream (filename,std::ios::app);
   for(int i=0; i < numSensorNodes+1; i++){
        Vector v=positionAlloc->GetNext();
        *(positionAllocOut->GetStream()) << v.x << " " << v.y << " " << v.z <<"\n";
   }
   
   positionAllocOut->GetStream()->flush();

   //src(filename, std::ios::binary);
   filename = "";
   ss.str( std::string() );
   ss.clear();
   ss << filePath << "IO" << "_";
   filePrefix = ss.str();

   //Ptr<OutputStreamWrapper> scheduleStream = asciiTraceHelper.CreateFileStream ("/dev/null",std::ios::out);

   CREATE_STREAM_FILENAME("positionAlloc.txt");
   OutputStreamWrapper(filename,std::ios::trunc); //Rajith 0410
   Ptr<OutputStreamWrapper> positionAllocOut_cp = asciiTraceHelper.CreateFileStream (filename,std::ios::app);
   for(int i=0; i < numSensorNodes+1; i++){
        Vector v=positionAlloc->GetNext();
        *(positionAllocOut_cp->GetStream()) << v.x << " " << v.y << " " << v.z <<"\n";
   }

   positionAllocOut_cp->GetStream()->flush();
   //end of Writing Locations

   //writing failed Node count 0410
   filename = "";
   ss.str( std::string() );
   ss.clear();
   ss << filePath;
   filePrefix = ss.str();

   //Ptr<OutputStreamWrapper> scheduleStream = asciiTraceHelper.CreateFileStream ("/dev/null",std::ios::out);

   CREATE_STREAM_FILENAME("Failed_Node_Count.txt");
   Ptr<OutputStreamWrapper> Failed_Node_Count = asciiTraceHelper.CreateFileStream (filename,std::ios::app);
   *(Failed_Node_Count->GetStream()) << totalFailedNodesCount <<"\n";

   Failed_Node_Count->GetStream()->flush();
   //end of Writing Locations

  // end of *************** Writing Files for Energies and Locations ***************

   /*
   AsciiFile asciifile;
   ss.str( std::string() );
   ss.clear();
   ss << filePath << "IO" << "_"; //Rajith 0408
   filePrefix = ss.str();
   // *y=filename;
   CREATE_STREAM_FILENAME("residualEnergies.txt");
   asciifile.open(&filename,std::ios::in);
   std::string readss;
   asciifile.read(readss 5);
   NS_LOG_UNCOND("Rajith Read file test: "<<readss);
   asciifile.close();
   */
   
	return 0;
}

