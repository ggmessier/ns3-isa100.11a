/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2018 The University Of Calgary- FISHLAB
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
 * Author: Rajith Madduma Bandarage <rajith.maddumabandar@ucalgary.ca>
 *
 */

#ifndef ISA100_GRAPH_SCHEDULING_H
#define ISA100_GRAPH_SCHEDULING_H

#include "ns3/core-module.h"
#include "ns3/isa100-11a-module.h"
#include "isa-graph.h"
#include <algorithm>

using namespace std;

namespace ns3 {

/**
 * Construct the data communication schedule based on UTCS Technical Report TR-1994.
 *- ISAGraphScheduling Class is the scheduling model to use for the ISA100.11a network scheduling.
 *
 */

/** Slot information including the time slot and the channel offset value
 *
 */
typedef struct
{
  uint16_t m_timeSlot;            ///< Scheduled slot number of the superframe
  int8_t m_channelOffset;         ///< channel offset when channel hopping is available
} Slot;

/** Link schedule type; whether transmitting or recieving
 *
 */
typedef enum
{
  TX,            ///< if the node transmitting the packet
  RX             ///< if the node receiving the packet
} LinkType;

/** Indicates the result of attempting to schedule a routing algorithm solution.
 *
 */
typedef enum
{
  SLOT_EXCLUSIVE,
  SLOT_SHARED
} LinkOption;         // temporary enumeration; DlLinkType of isa100-dl.h can be used during integration

/** Used by Scheduling algorithm to create the packet schedule.
 *
 */
typedef struct
{
  uint16_t m_nextNode;              ///< Transmit node ID of the packet.
  LinkType m_linkType;              ///< whether packet is transmitting or receiving
//  Slot m_slot;                    ///< Schedule slot information including time slot and the channel offset
  uint16_t m_repLength;             //< packet scheduled repetition length (depends on superframe size)
  /** m_graphID
   * 0x0000 - Uplink graph (0)
   * 0x0001 - Downlink graph of node id = 1 (1)
   * 0xFFFF - Broadcast graph (65535)
   */
  Ptr<IsaGraph> m_graphID;          ///< an ID to identify the graph (pointer to the graph)
  LinkOption m_option;              ///< link option whether share or exclusive
//  double m_pwr;                   ///< Transmit power required to reach the next hop.
} TimeScheduleStruct;

class Isa100GraphScheduling;

/** Class that stores the ISA-100 graph information.
 * - Includes reliable broadcasting, uplink and downlink graph creation.
 */
class Isa100GraphScheduling : public Object
{
public:
  static TypeId GetTypeId (void);

  Isa100GraphScheduling ();

  virtual ~Isa100GraphScheduling ();

  /** Construct the data communication schedule of the graph network
   *
   * @param G Pointer for the graph network that need to create the schedule
   */
  bool ConstructDataCommunicationSchedule (Ptr<IsaGraph> G, Ptr<IsaGraph> GUL, Ptr<IsaGraph> GB, map <uint32_t, Ptr<IsaGraph>> GDL);

  /** Schedule link for specific graph type
   *
   * @param u and v are the source and destination of the communication
   * @param Graph Pointer for the graph network that need to create the schedule
   * @param superframe superframe size considering the sample rate
   * @param timeSlot time slot that required to consider to find the next available slot
   * @param option link option whether exclusive or shared
   */
  bool ScheduleLinks (Ptr<Node> u, Ptr<Node> v, Ptr<IsaGraph> Graph, uint32_t superframe, uint16_t timeSlot, LinkOption option);

  /** Get the total time slots for a given power of sample rate value
   * TS = 10ms; Total time slots = (2^PowerOfrate)*100 // [sample rate in ms *1000/10]
   *
   * @param PowerOfrate power of the sample rate (n of 2^n): range is -2 to 9
   */
  uint16_t GetTimeSlots (int8_t PowerOfrate);

  /** Get next available channel offset and the time slot
   *
   * @param timeSlot time slot that required to consider to find the next available slot
   */
  Slot GetNextAvailableSlot(uint16_t timeSlot, LinkOption option);

  /** print full schedule of the network and each node schedules
   *
   * @param node return the schedule of this node
   */
  void PrintSchedule();
  void PrintNodeSchedule(Ptr<Node> node);

private:
  int8_t m_maximumChannelOffsets;                               ///< maximum available channel offsets for the network. ***this need to be set in helper class***
  map<Ptr<Node>, vector<pair<Slot, TimeScheduleStruct>>> m_nodeSchedule;    ///< schedule for each node of the network
  multimap<Slot, pair<Ptr<Node>,TimeScheduleStruct>> m_schedule;     ///< Total schedule of the network
  map<int8_t, vector<Ptr<Node>>> m_groupSameSampleRate;         ///< group all the nodes with same sample rate

};

} // namespace ns3

#endif /* ISA100_11A_GRAPH_SCHEDULING_H */
