/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2017 The University Of Calgary- FISHLAB
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
 * Author: Geoffrey Messier <gmessier@ucalgary.ca>
 *         Hazem Gomaa <gomaa.hazem@gmail.com>
 *         Michael Herrmann <mjherrma@ucalgary.ca>
 */

#ifndef ISA100_HELPER_H
#define ISA100_HELPER_H

#include "ns3/isa100-dl.h"
#include "ns3/isa100-processor.h"
#include "ns3/isa100-sensor.h"
#include "ns3/net-device-container.h"
#include "ns3/tdma-optimizer-base.h"
#include "ns3/isa100-application.h"
#include "ns3/vector.h"

#include "ns3/isa-graph.h"

namespace ns3 {

class Isa100NetDevice;
class SingleModelSpectrumChannel;
class ListPositionAllocator;
//class IsaGraph;

/** Used by the breadth first search that designs a minimum hop network.
 *
 */
typedef struct
{
  int parent;       ///< Parent node in the network tree.
  std::vector<uint16_t> slotSched;        ///< Array of slot numbers where the node is active
  std::vector<DlLinkType> slotType;        ///< What the node is actually doing in those slots (tx,rx,etc)
  int hopCount;       ///< Number of hops to sink
  double pwr;       ///< Transmit power required to reach the next hop.
  int totalPackets;       ///< Number of packets the node has to send.
} ScheduleStruct;

/** Used to contain the SUPERFRAME schedule for a node.
 *
 */
typedef struct
{
  std::vector<uint16_t> slotSched;        ///< Array of slot numbers where the node is active
//	std::vector<uint8_t> channelSched;  ///< channel used for the communication
  std::vector<DlLinkType> slotType;        ///< What the node is actually doing in those slots
} NodeSchedule;

/** Used to contain the node schedule information of a particular slot
 *
 */
typedef struct
{
  uint8_t channelSched;  ///< channel used for the communication
  DlLinkType slotType;  ///< TX/ RX or shared slot
  Mac16Address graphID;  ///< current graph ID
  uint16_t localPathID;  ///< local path ID when multiple paths exist
  uint32_t pathSource;  ///< source node of the path
} NodeInfo;

/** Used to contain the Table for a routing.
 *
 */
typedef struct
{
  uint32_t destID;  ///< destination ID
  Mac16Address nextGraphID;  ///< current graph ID
  std::vector<uint32_t> neighborList;  ///< neighbors for the next transmission
} RoutingTable;

/** Used to contain slot and frequency information
 *
 */
typedef struct
{
  uint32_t timeSlot;  ///< time slot number
  uint8_t channelIndex;  ///< carrier used for the communication
} Resource;

/** Indicates the result of attempting to schedule a routing algorithm solution.
 *
 */
typedef enum
{
  SCHEDULE_FOUND,
  INSUFFICIENT_SLOTS,
  NO_ROUTE,
  STARVED_NODE
} SchedulingResult;


/** Node location callback.
 *
 * @param node number
 * @param x-position.
 * @param y-position.
 * @param z-position.
 */
typedef TracedCallback<int, double, double, double> HelperLocationTracedCallback;

typedef TracedCallback<int, int, int, int> HelperScheduleTracedCallback;

typedef TracedCallback<int, int, double> HelperTxPowerTracedCallback;

typedef TracedCallback<int, double> HelperHopCountTracedCallback;

typedef TracedCallback<int, int> HelperGraphTracedCallback;

/**
 * \defgroup isa100-11a-helper ISA 100.11a Helper Implementation
 *
 * This documents the code that implements the 100.11a helper class.
 *  @{
 */

class Isa100Helper : public Object
{
public:
  static TypeId GetTypeId (void);

  Isa100Helper (void);
  ~Isa100Helper (void);

  /** Creates and installs net devices and installs them in each node in the node container.
   * - This can only be called after the attributes for the phy and DL have been set in the helper object.
   * - This will create a phy and DL object in the net device but nothing else (ie. no sensor, battery, etc.).
   * - The other net device objects (ie. sensor, battery, etc.) can only be installed after this function has been called.
   *
   * @param c Node container.
   * @param channel The channel used to carry transmissions between nodes.
   * @param sinkIndex the index of the sink node
   */
  NetDeviceContainer Install (NodeContainer c, Ptr<SingleModelSpectrumChannel> channel, uint32_t sinkIndex);


  /** Set the source routing table for a specific node.
   * - Must be called after Isa100Helper::Install()
   *
   * @param nodeInd Index of the node being configured.
   * @param numNodes Number of nodes in the network.
   * @param routingTable 2D Mac16Address array where row n contains the multi-hop path to reach node n.
   */
  void SetSourceRoutingTable (uint32_t nodeInd, uint32_t numNodes,  std::string *routingTable);

  /** Install an application object on a specific node.
   * - Must be called after Isa100Helper::Install()
   *
   * @param c Node container.
   * @param nodeIndex Index of the node where the object will be installed.
   * @param app Reference to the application object.
   */
  void InstallApplication (NodeContainer c, uint32_t nodeIndex, Ptr<Isa100Application> app);

  /** Install a battery object on a specific node.
   * - Must be called after Isa100Helper::Install()
   *
   * @param nodeIndex Index of the node where the object will be installed.
   * @param battery Pointer to the battery object.
   */
  void InstallBattery (uint32_t nodeIndex, Ptr<Isa100Battery> battery);

  /** Install a processor object on a specific node.
   * - Must be called after Isa100Helper::Install()
   *
   * @param nodeIndex Index of the node where the object will be installed.
   * @param processor Pointer to the processor object.
   */
  void InstallProcessor (uint32_t nodeIndex, Ptr<Isa100Processor> processor);

  /** Install a sensor object on a specific node.
   * - Must be called after Isa100Helper::Install()
   *
   * @param nodeIndex Index of the node where the object will be installed.
   * @param sensor Pointer to the sensor object.
   */
  void InstallSensor (uint32_t nodeIndex, Ptr<Isa100Sensor> sensor);


  /** Pass a DL attribute to the helper.
   * - Must be called before Isa100Helper::Install().
   * - Attribute is stored and passed to the DL objects during the Isa100Helper::Install().
   *
   * @param n String identifying attribute.
   * @param v Attribute value.
   */
  void SetDlAttribute (std::string n, const AttributeValue &v);

  /** Pass a zigbee trx current attribute to the helper
   * - Must be called before Isa100Helper::Install().
   *  - used to configure attributes for ZigbeeTrxCurrentModel
   *
   *  @param n String identifying attribute.
   *  @param v Attribute value.
   */
  void SetTrxCurrentAttribute (std::string n, const AttributeValue &v);

  /** Pass a zigbee PHY attribute to the helper
   * - Must be called before Isa100Helper::Install().
   *  - used to configure attributes for ZigbeePhy
   *
   *  @param n String identifying attribute.
   *  @param v Attribute value.
   */
  void SetPhyAttribute (std::string n, const AttributeValue &v);

  // ------ Node Location ---------
  /** Set the position of the nodes
   *
   * @param dc node container.
   * @param positionAlloc A list containing the node position vectors.
   */
  void SetDeviceConstantPosition (NetDeviceContainer dc, Ptr<ListPositionAllocator> positionAlloc);

  /** Generate random positions for a fixed number of nodes in a square coverage area.
   * - The result of this function is stored in positionAlloc and not directly in the nodes.
   * - To set the nodes with these random positions, positionAlloc must be passed into
   *   SetDeviceConstantPosition().
   *
   * @param positionAlloc Data structure for holding node positions.
   * @param numNodes Number of nodes.
   * @param xLength Size of field in the x direction (m).
   * @param yLength Size of field in the y direction (m).
   * @param minNodeSpacing Nodes must be separated by at least this much (m).
   * @param sinkLocation Location of the sink node.
   */
  void GenerateLocationsFixedNumNodes (Ptr<ListPositionAllocator> positionAlloc, int numNodes, double xLength, double yLength,
                                       double minNodeSpacing, std::vector<Vector> coreNodeLocations, double factor);

  // ------ Scheduling -----------

  /** Set the DL superframe schedule for a specific node.
   * - Must be called after Isa100Helper::Install()
   *
   * @param nodeInd Index of the node being configured.
   * @param hopPattern Pointer to the array containing hop pattern channel numbers.
   * @param numHop Length of hop pattern array.
   * @param linkSched Pointer to the array indicating which superframe slots this node is active in.
   * @param linkTypes Pointer to array indicating what kind of activity occurs in the active slots (TRANSMIT,RECEIVE,SHARED).
   * @param numLink Length of the linkSched and linkTypes arrays.
   */
  void SetSfSchedule (uint32_t nodeInd, uint8_t *hopPattern, uint32_t numHop, uint16_t *linkSched, DlLinkType *linkTypes, uint32_t numLink);


  /** Pass a TDMA Optimizer attribute to the helper
   *  - used to configure attributes for TDMA Optimizers
   *
   *  @param n String identifying attribute.
   *  @param v Attribute value.
   */
  void SetTdmaOptAttribute (std::string n, const AttributeValue &v);


  /* Uses a tdma-optimizer to determine network routes and schedule.
   *
   * @param c Node container.
   * @param propModel Propagation loss model used for network
   * @param hopPatter The hopping pattern
   * @param numHop The number of hops in the hopping pattern
   * @param optSelect An optional parameter to specify which TDMA optimizer to use
   * @param stream An optional stream to output the schedule to
   * @return Result of scheduling attempt.
   */
  SchedulingResult CreateOptimizedTdmaSchedule (NodeContainer c, Ptr<PropagationLossModel> propModel,
                                                vector<uint8_t> carriers, uint32_t numHop, OptimizerSelect optSelect,
                                                Ptr<OutputStreamWrapper> stream = NULL);

  // ------ Graph Scheduling Exclusive Functions -----------

  /** Construct the data communication schedule of the graph network
     *
     * @param G Pointer for the graph network that need to create the schedule
     */
  SchedulingResult ConstructDataCommunicationSchedule (Ptr<IsaGraph> G, map <uint32_t, Ptr<IsaGraph> > mapOfG);

  /** Schedule link for specific graph type
   *
   * @param u and v are the source and destination of the communication
   * @param Graph Pointer for the graph network that need to create the schedule
   * @param superframe SUPERFRAME size considering the sample rate
   * @param timeSlot earliest slot to be allocated
   * @param option link option whether exclusive or shared
   */
  bool ScheduleLinks (Ptr<Node> u, Ptr<Node> v, Ptr<IsaGraph> Graph, uint32_t superframe, uint32_t timeSlot, DlLinkType option,
                      uint16_t grpahID, uint32_t source);

  /** Get next available channel offset and the time slot
   *
   * @param u and v are the source and destination of the communication
   * @param timeSlot earliest slot to be allocated
   * @param option link option whether exclusive or shared
   * @param repLength for the allocating slot
   */
  Resource GetNextAvailableSlot (uint32_t u, uint32_t v, uint32_t timeSlot, DlLinkType option, uint32_t repLength);

  /** Set created schedule for simulation
   *
   * @return Result of scheduling attempt.
   */
  SchedulingResult ScheduleAndRouteTDMAgraph (OptimizerSelect optSelect);

  /** Resize the main schedule (and repeat length vector) to fit all the slots of the SUPERFRAME
   *
   * @param superframe SUPERFRAME size considering the sample rate
   */
  void ResizeSchedule (uint32_t superframe);

  /** Add Edge Weights to Han's Graph Algorithms (Not Required)
   *
   * @param edge information of the edge. i.e., source and destination node ID pairs
   */
  void AddEdgeWeights (pair<uint32_t,uint32_t> edge);

  /** Iterate over the Graph nodes for "printGraph" TRACE
   *
   * @param Graph Pointer for the graph network that need to create the schedule
   */
  void PrintGraph (Ptr<IsaGraph> Graph);

  /** Iterate over the main schedule for "Schedule" TRACE
   *
   */
  void PrintGraphSchedule ();

  // ------ MIN Load Graph (Wu's) Scheduling Exclusive Functions -----------

  /** Set created schedule for simulation
   *
   * @return Result of scheduling attempt.
   */
  SchedulingResult ConstructDataCommunicationScheduleMinLoad (vector< vector<uint32_t> > UL_Ex, vector<vector< vector<uint32_t> > > UL_Sh,
                                                              vector< vector<uint32_t> > DL_Ex, vector< vector<uint32_t> > DL_Sh, int frameSize);

  bool ScheduleLinksMinLoad (vector< vector<uint32_t> > flows, int frameSize, uint32_t timeSlot, DlLinkType option, bool isBackup);
//  /** Construct the data communication schedule of the graph network
//     *
//     * @param G Pointer for the graph network that need to create the schedule
//     */
//  SchedulingResult ConstructDataCommunicationSchedule (vector< vector<uint32_t>> flows, vector<uint32_t> flowBoundaries);

  /**}@*/

private:
  // -- Flow Matrix Scheduling Functions --

  // ... General Functions ...

  /** Set TDMA Optimizer attributes.
   * @param optimizer Pointer to the Optimizer.
   */
  void SetTdmaOptimizerAttributes (Ptr<TdmaOptimizerBase> optimizer);

  /** Creates an entry in an Isa100Dl compatible superframe schedule.
   *
   * @param src Source node.
   * @param dst Destination node.
   * @param weight Number of slots that need to be scheduled for this link.
   * @param schedules Array of schedules for all nodes.
   * @param nSlot Superframe slot index.
   * @param scheduleSummary Summary of TDMA schedule used by routing algorithm.
   *
   */
  void PopulateNodeSchedule (int src, int dst, int weight, vector<NodeSchedule> &schedules, int &nSlot, vector< vector<int> > &scheduleSummary);

  /** Program source route and TDMA schedules into nodes based on a slot flow matrix.
   *
   * @param slotFlows Slot flow matrix.
   * @param packetsPerSlot Number of packets sent per slot.
   * @return Whether scheduling was possible.
   */
  SchedulingResult ScheduleAndRouteTdma (std::vector< std::vector<int> > slotFlows, int packetsPerSlot);

  /** Calculates transmit powers between nodes.
   *
   * @param c Node container.
   * @param propModel Propagation model.
   */
  void CalculateTxPowers (NodeContainer c, Ptr<PropagationLossModel> propModel);


  // ... TDMA Superframe Generation Functions ...

  /** Creates an array of Isa100Dl superframe schedules based on a packet flow matrix.
   *
   * @param lAll The array of Isa100Dl superframe schedules.
   * @param scheduleSummary Summary of TDMA schedule used by source routing algorithm.
   * @param packetFlows Matrix of packet flows.
   * @return Whether a TDMA schedule could be found.
   */
  SchedulingResult FlowMatrixToTdmaSchedule (vector<NodeSchedule> &lAll, vector< vector<int> > &scheduleSummary, vector< vector<int> > packetFlows);

  // Rajith Revised
  SchedulingResult FlowMatrixToTdmaScheduleRevised (vector<NodeSchedule> &lAll, vector< vector<int> > &scheduleSummary, vector< vector<int> > packetFlows);

  /** Determines if all the outflow links for node have been scheduled.
   *
   * @param node Index for the node being examined.
   * @param packetFlows Matrix of packet flows.
   */
  int AllOutlinksScheduled (int node, vector< vector<int> > &packetFlows);

  /** Adds a node to a vector of node indices except if that node is not already in the vector.
   *
   * @param node Node index.
   * @param q0 Vector of node indices.
   */
  void PushBackNoDuplicates (int node, vector<int> &q0);

  /** Determines if a node is a leaf on the flow graph.
   *
   * @param node Node index.
   * @param flowMatrix Flow matrix.
   */
  bool IsLeaf (int node, vector< vector<int> > packetFlows);

  /** Determines whether a node has a parent in the current Q vector.
   *
   * @param node The node in question.
   * @param q Q vector
   * @param packetFlows Flow matrix
   * @return Whether there is a parent in Q.
   */
  bool NoParentInQ (int node, vector<int> q, vector< vector<int> > &packetFlows);


  // ... Source Routing List Generation ...

  /** Determines source routing strings based on a packet flow matrix.
   *
   * @param routingStrings Vector of routing strings.
   * @param schedule TDMA schedule summary.
   * @return Whether routes could be found for all nodes.
   */
  SchedulingResult CalculateSourceRouteStrings (vector<std::string> &routingStrings, vector< vector<int> > schedule);

  /** Set the DL attributes of a net device.
   * @param device Pointer to the net device object.
   */
  void SetDlAttributes (Ptr<Isa100NetDevice> device);

  /** Set the PHY attributes of a Zigbee PHY.
   * @param device Pointer to the Zigbee PHY.
   */
  void SetPhyAttributes (Ptr<ZigbeePhy> device);

  /** Convert the graphIDs to match the current DROUT routing information
   * @param graphID as 16bit (2 Byte) number
   * @return graphID represent as a MacAddress
   */
  Mac16Address GraphIDConverter (uint16_t graphID);

  /** Trace source for number of hops in scheduled network.
   */
  TracedCallback< vector<int>  > m_hopTrace;



  std::map <std::string, Ptr<AttributeValue> > m_dlAttributes;  ///< Used to store DL attributes before install.
  std::map <std::string, Ptr<AttributeValue> > m_phyAttributes;  ///< Used to store PHY attributes before install.
  std::map <std::string, Ptr<AttributeValue> > m_tdmaOptAttributes;  ///< Used to store tdma optimization attributes
  std::map <std::string, Ptr<AttributeValue> > m_trxCurrentAttributes;  ///< Used to store trx energy attributes
  NetDeviceContainer m_devices;  ///< Contains the devices being set up.

  double **m_txPwrDbm; /// Holds transmit powers between nodes.
  int m_numTimeslots; ///< Number of timeslots in a superframe.

  // Additional private variables used by Rajith
  bool m_graphType;  ///< use to identify the graph
  // slot -> channel -> TX/ RX -> node
  vector<vector<vector<uint32_t> > > m_mainSchedule;  ///< main schedule information (slot, channel index,  TX if 0 and TX if 1)
  // node -> slot -> (channel, TX/ RX/ Shared)
  map<uint32_t, map<uint32_t, NodeInfo> > m_nodeScheduleN;    ///< schedule for each node of the network (map of node to NodeSchedule)
//  map<uint32_t, map<uint32_t, vector<Mac16Address>>> m_tableList;  ///< routing tables of each nodes (Node ID -> destination -> routing table)
  ///< routing tables of each nodes (Node ID -> graphID -> routing table)
  map<uint32_t, map<Mac16Address, RoutingTable> > m_tableList;

  map<uint32_t, map<uint32_t, vector<uint32_t> > > m_avgHopCount;  ///< average hop count of each node
  vector<vector<uint32_t> > m_repLength;   ///<repetition length of the slot and channel index
  vector<uint8_t> m_carriers;    ///< carriers used (channels)
  map<uint32_t, uint32_t> m_nextAvailableSlot;   ///< to track the next available time slot for schedule (current slot -> next)
  map<uint32_t, uint8_t> m_nextAvailableChIndex;   ///< to track the next available carrier for schedule (slot -> next channel index)
  uint16_t m_grpahID;  ///< graph ID for the schedule creation

  map<uint32_t, vector<Ptr<Node> > > m_groupSameSampleRate;         ///< group all the nodes with same sample rate

  HelperLocationTracedCallback m_locationTrace;
  HelperScheduleTracedCallback m_scheduleTrace;
  HelperTxPowerTracedCallback m_txPowerTrace;
  HelperHopCountTracedCallback m_hopCountTrace;
  HelperGraphTracedCallback m_graphTrace;

  vector<pair<uint32_t,uint32_t> > m_edgeWeight;
  bool m_ResourceAvailable;     ///< whether frame size is sufficient to create the scheduling

  uint16_t m_panID;
  ///< routing tables of each nodes (Node ID -> destID -> graphID sequence)
  map<uint32_t, map<uint32_t, vector<vector<Mac16Address> > > > m_tableListMinLoad;
/*  // source -> destination -> Primary path
  map<uint32_t, map<uint32_t, vector<Mac16Address> > > m_tableListPrimary;*/
  // node -> Primary GraphID -> Backup Path
  map<uint32_t, map<Mac16Address, vector<Mac16Address> > > m_tableListBackup;

};


} // namespace ns3

#endif /* ISA100_11A_HELPER_H */
