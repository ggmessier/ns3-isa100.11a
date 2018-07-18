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
 */

#include "isa100-graph-scheduling.h"
#include "isa-graph.h"
#include "ns3/log.h"
#include "ns3/type-id.h"
#include <algorithm>

NS_LOG_COMPONENT_DEFINE ("Isa100GraphScheduling");

using namespace std;

namespace ns3 {

TypeId Isa100GraphScheduling::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::Isa100GraphScheduling")
    .SetParent<Object> ()
    .AddConstructor<Isa100GraphScheduling> ()
  ;
  return tid;
}

Isa100GraphScheduling::Isa100GraphScheduling ()
{
  NS_LOG_FUNCTION (this);
  m_maximumChannelOffsets = 4;
}

Isa100GraphScheduling::~Isa100GraphScheduling ()
{
  NS_LOG_FUNCTION (this);
}

bool Isa100GraphScheduling::ConstructDataCommunicationSchedule (Ptr<IsaGraph> G, Ptr<IsaGraph> GUL,
                                                                Ptr<IsaGraph> GB, map <uint32_t, Ptr<IsaGraph>> GDL)
{
  NS_LOG_FUNCTION (this);
  map <uint32_t, GraphNode> graphNodeMap = G->GetGraphNodeMap();
  Ptr<Node> gateway = G->GetGateway();        ///< g node of the algorithm

  for (map<uint32_t, GraphNode>::const_iterator it = graphNodeMap.begin ();
        it != graphNodeMap.end (); ++it)
    {
      // Identify the set of nodes with each sample rate: N 1 , N 2 , . . . , N k .
      int8_t tempPowerofrate = it->second.m_powerOfrate;
      // Sort device sample rates in ascending order: r 1 < r 2 < . . . < r k . (Here Map is storing data in sorted manner.)
      (this)->m_groupSameSampleRate[tempPowerofrate].push_back(it->second.m_head);
    }

  vector<Ptr<Node>> GroupwithSampleRate;

  // for all r i from r 1 to r k do
  for (map<int8_t, vector<Ptr<Node>>>::const_iterator it = (this)->m_groupSameSampleRate.begin ();
        it != (this)->m_groupSameSampleRate.end (); ++it)
    {
      GroupwithSampleRate = it->second;
      // Generate the data Superframe F i
      uint16_t superframe = (this)->GetTimeSlots(it->first);

      // for all node v ∈ N i do
      while(!GroupwithSampleRate.empty())
        {
          Ptr<Node> v = GroupwithSampleRate.back();
          if(v != gateway)
            {
              // Schedule primary and retry links for publishing data
              (this)->ScheduleLinks(v, gateway, GUL , superframe, 0, SLOT_EXCLUSIVE);
              (this)->ScheduleLinks(v, gateway, GUL , superframe, superframe/4, SLOT_SHARED);
              // Schedule primary and retry links for control data
              (this)->ScheduleLinks(gateway, v, GDL[v->GetId()] , superframe, superframe/2, SLOT_EXCLUSIVE);
              (this)->ScheduleLinks(gateway, v, GDL[v->GetId()] , superframe, superframe/4*3, SLOT_SHARED);
            }
          GroupwithSampleRate.pop_back();
        }
    }

  return true;
}

// ScheduleLinks(u, v, G, F , t, o) - additionally channel offset has included.
bool Isa100GraphScheduling::ScheduleLinks (Ptr<Node> u, Ptr<Node> v, Ptr<IsaGraph> Graph, uint32_t superframe,
                                           uint16_t timeSlot, LinkOption option)
{
  NS_LOG_FUNCTION (this);
  //Identify data superframe F ′ with l F ′ = 2l F
  uint32_t superframeF_1 = superframe*2;
  vector <Ptr<Node>> successorsOfU = Graph->GetGraphNode(u->GetId()).m_neighbors;
  TimeScheduleStruct scheduleStructForU;          // schedule information for node u
  TimeScheduleStruct scheduleStructForNext;          // schedule information for node v

  scheduleStructForU.m_graphID = Graph;
  scheduleStructForU.m_linkType = TX;

  Slot slot;

//  multimap<pair<Ptr<Node>, Slot>, TimeScheduleStruct> m_nodeSchedule;    ///< schedule for each node of the network
//  multimap<Slot, pair<Ptr<Node>,TimeScheduleStruct>> m_schedule;     ///< Total schedule of the network

  //  for all node i ∈ Successor(u) do
  for (vector<Ptr<Node>>::const_iterator it = successorsOfU.begin ();
        it != successorsOfU (); ++it)
    {
      Ptr<Node> next = successorsOfU.back();    // node i

      slot = (this)->GetNextAvailableSlot(timeSlot, option);
      uint16_t tempEarliestTime = slot.m_timeSlot;

      // update the total schedule
      pair<Ptr<Node>,TimeScheduleStruct> timeSchedulePair;
      timeSchedulePair.first = u;
      scheduleStructForU.m_nextNode = next;
      scheduleStructForU.m_repLength = superframe;

      pair<Ptr<Node>, Slot> tempNodeAndSlot;
      map<Slot,TimeScheduleStruct> tempNodeSchedule;

      // update the schedule of node Next
      scheduleStructForNext = scheduleStructForU;
      scheduleStructForNext.m_linkType = RX;
      scheduleStructForNext.m_nextNode = u;

      if (successorsOfU.size()==1)              // if i is the only successor of u then
        {
          // update the total schedule
          timeSchedulePair.second = scheduleStructForU;
          (this)->m_schedule[slot] = timeSchedulePair;

          // update the schedule of node u
          tempNodeAndSlot.first = u;
          tempNodeAndSlot.second = slot;
          (this)->m_nodeSchedule[tempNodeAndSlot] = scheduleStructForU;

          // update the schedule of node Next
          tempNodeAndSlot.first = next;
          tempNodeAndSlot.second = slot;
          (this)->m_nodeSchedule[tempNodeAndSlot] = scheduleStructForNext;

          if (next != v)
            {
              (this)->ScheduleLinks(next, v, Graph , superframe, tempEarliestTime, option);
            }
        }
      else
        {
          if(it==successorsOfU.begin ())
            {
              // update the total schedule
              scheduleStructForU.m_repLength = superframeF_1;
              timeSchedulePair.second = scheduleStructForU;
              (this)->m_schedule[slot] = timeSchedulePair;

              // update the schedule of node u
              tempNodeAndSlot.first = u;
              tempNodeAndSlot.second = slot;
              (this)->m_nodeSchedule[tempNodeAndSlot] = scheduleStructForU;

              // update the schedule of node Next
              tempNodeAndSlot.first = next;
              tempNodeAndSlot.second = slot;
              scheduleStructForNext.m_repLength = superframeF_1;
              (this)->m_nodeSchedule[tempNodeAndSlot] = scheduleStructForNext;
            }
          else
            {
              slot.m_timeSlot = tempEarliestTime + superframe;

              // update the total schedule
              scheduleStructForU.m_repLength = superframeF_1;
              timeSchedulePair.second = scheduleStructForU;
              (this)->m_schedule[slot] = timeSchedulePair;

              // update the schedule of node u
              tempNodeAndSlot.first = u;
              tempNodeAndSlot.second = slot;
              (this)->m_nodeSchedule[tempNodeAndSlot] = scheduleStructForU;

              // update the schedule of node Next
              tempNodeAndSlot.first = next;
              tempNodeAndSlot.second = slot;
              scheduleStructForNext.m_repLength = superframeF_1;
              (this)->m_nodeSchedule[tempNodeAndSlot] = scheduleStructForNext;
            }

          if (next != v)
            {
              (this)->ScheduleLinks(next, v, Graph , superframeF_1, tempEarliestTime, option);
            }
        }
    }


  return true;
}

uint16_t Isa100GraphScheduling::GetTimeSlots (int8_t PowerOfrate)
{
  NS_LOG_FUNCTION (this);
  return 2^PowerOfrate*100;
}

Slot Isa100GraphScheduling::GetNextAvailableSlot(uint16_t timeSlot, LinkOption option)
{
  NS_LOG_FUNCTION (this);
  Slot slot;
  slot.m_timeSlot = timeSlot;
  slot.m_channelOffset = 0;
  int8_t tempOffset = 0;

  switch (option)
  {
    case SLOT_EXCLUSIVE:
      while (tempOffset < (this)->m_maximumChannelOffsets)
        {
          if ((this)->m_schedule.count(slot) == 0)
            {
              return slot;
            }
          tempOffset++;
          if (tempOffset == (this)->m_maximumChannelOffsets)
            {
              tempOffset = 0;
              timeSlot++;
            }
          slot.m_channelOffset = tempOffset;
          slot.m_timeSlot = timeSlot;
        }
      break;

    case SLOT_SHARED:
      while (tempOffset < (this)->m_maximumChannelOffsets)
        {
          if ((this)->m_schedule.count(slot) < 5)
            {
              return slot;
            }
          tempOffset++;
          if (tempOffset == (this)->m_maximumChannelOffsets)
            {
              tempOffset = 0;
              timeSlot++;
            }
          slot.m_channelOffset = tempOffset;
          slot.m_timeSlot = timeSlot;
        }
      break;
  }

  return slot;
}

} // namespace ns3

