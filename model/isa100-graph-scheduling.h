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
//#include "isa-graph.h"
#include <algorithm>

using namespace std;

namespace ns3 {

/**
 * Construct the data communication schedule based on UTCS Technical Report TR-1994.
 *- ISAGraphScheduling Class is the scheduling model to use for the ISA100.11a network scheduling.
 *
 */

/** Used by Scheduling algorithm to create the packet schedule.
 *
// */
//typedef struct
//{
//  uint16_t m_source;              ///< Source node of the packet.
//  uint16_t m_destination;         ///< Destination node of the packet.
//  vector<uint16_t> m_slotSched;   ///< Array of slot numbers where the node is active
//  vector<DlLinkType> m_slotType;  ///< What the node is actually doing in those slots (tx,rx,etc)
//  uint16_t m_hopCount;                 ///< Number of hops to sink
//  double m_pwr;                   ///< Transmit power required to reach the next hop.
//  uint32_t m_totalPackets;             ///< Number of packets the node has to send.
//} ScheduleStructNew;
//
///** Used to contain the superframe schedule for a node.
// *
// */
//typedef struct
//{
//  Ptr<Node> m_head;               ///< pointer to the node of the network scheduled
//  vector<uint16_t> m_slotSched;   ///< Array of slot numbers where the node is active
//	vector<DlLinkType> m_slotType;  ///< What the node is actually doing in those slots
//} NodeScheduleNew;
//
///** Indicates the result of attempting to schedule a routing algorithm solution.
// *
// */
//typedef enum
//{
//	SCHEDULE_FOUND_N,
//	INSUFFICIENT_SLOTS_N,
//	NO_ROUTE_N,
//	STARVED_NODE_N
//} SchedulingResultNew;

class Isa100GraphScheduling;

/** Class that stores the ISA-100 graph information.
 * - Includes reliable broadcasting, uplink and downlink graph creation.
 */
class Isa100GraphScheduling : public Object
{
public:
  static TypeId GetTypeId (void);

  Isa100GraphScheduling ();

//  Isa100GraphScheduling (Ptr<IsaGraph> G);

  virtual ~Isa100GraphScheduling ();

private:
//  uint32_t m_supperFrame;        ///< Supper Frame for all the nodes in the network

};

} // namespace ns3

#endif /* ISA100_11A_GRAPH_SCHEDULING_H */
