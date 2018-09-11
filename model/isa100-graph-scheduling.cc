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
//#include "isa-graph.h"
#include "ns3/log.h"
#include "ns3/type-id.h"
#include <algorithm>

NS_LOG_COMPONENT_DEFINE ("Isa100GraphScheduling");

using namespace std;

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (Isa100GraphScheduling);

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
//  m_maximumChannelOffsets = 4;
}

Isa100GraphScheduling::~Isa100GraphScheduling ()
{
  NS_LOG_FUNCTION (this);
}

bool Isa100GraphScheduling::ConstructDataCommunicationSchedule (Ptr<IsaGraph> G, map <uint32_t, Ptr<IsaGraph>> mapOfG)
{
  NS_LOG_FUNCTION (this);

  map <uint32_t, GraphNode> graphNodeMap = G->GetGraphNodeMap();
  Ptr<Node> gateway = G->GetGateway();        ///< g node of the algorithm

  Ptr<IsaGraph> GB = mapOfG[0];
  Ptr<IsaGraph> GUL = mapOfG[G->GetNumofNodes()];
  map <uint32_t, Ptr<IsaGraph>> GDL = mapOfG;

  for (map<uint32_t, GraphNode>::const_iterator it = graphNodeMap.begin ();
        it != graphNodeMap.end (); ++it)
    {
      // Identify the set of nodes with each sample rate: N 1 , N 2 , . . . , N k .
      uint32_t tempTimeSlots = it->second.m_numTimeSlots;
      // Sort device sample rates in ascending order: r 1 < r 2 < . . . < r k . (Here Map is storing data in sorted manner.)
      (this)->m_groupSameSampleRate[tempTimeSlots].push_back(it->second.m_head);
    }

  vector<Ptr<Node>> GroupwithSampleRate;

  // for all r i from r 1 to r k do
  for (map<uint32_t, vector<Ptr<Node>>>::const_iterator it = (this)->m_groupSameSampleRate.begin ();
        it != (this)->m_groupSameSampleRate.end (); ++it)
    {
      GroupwithSampleRate = it->second;
      // Generate the data Superframe F i
      uint32_t superframe = it->first;

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
//  NodeScheduleNew scheduleStructForU;          // schedule information for node u
//  NodeScheduleNew scheduleStructForNext;          // schedule information for node v

//  scheduleStructForU.m_graphID = Graph;
//  scheduleStructForU.slotType = TX;
//  scheduleStructForNext.slotType = RX;

//  Slot slot;
  uint16_t slot;
  slot = (this)->GetNextAvailableSlot(timeSlot, option);

  //  for all node i ∈ Successor(u) do
  for (vector<Ptr<Node>>::const_iterator it = successorsOfU.begin();
        it != successorsOfU.end(); ++it)
    {
      Ptr<Node> next = successorsOfU.back();    // node next
//      slot = (this)->GetNextAvailableSlot(timeSlot, option);
//      uint16_t tempEarliestTime = slot.timeSlot;
//      uint16_t tempEarliestTime = slot;

      // update the total schedule
//      pair<Ptr<Node>,TimeScheduleStruct> timeSchedulePair;
//      timeSchedulePair.first = u;
//      scheduleStructForU.slotSched = tempEarliestTime;
//      scheduleStructForU.m_repLength = superframe;

//      pair<Ptr<Node>, Slot> tempNodeAndSlot;
//      pair<Slot,TimeScheduleStruct> tempNodeSchedule;

      // update the schedule of node Next
//      scheduleStructForNext = scheduleStructForU;
//      scheduleStructForNext.slotType = RX;
//      scheduleStructForNext.m_nextNode = u;

      if (successorsOfU.size()==1)              // if next is the only successor of u then
        {
          (this)->m_mainSchedule[slot][0] = u->GetId();
          (this)->m_mainSchedule[slot][1] = next->GetId();
          (this)->m_nodeScheduleN[u->GetId()][slot] = TX;
          (this)->m_nodeScheduleN[next->GetId()][slot] = RX;
          (this)->m_repLength[slot] = superframe;

          // update the total schedule
//          timeSchedulePair.second = scheduleStructForU;
//          (this)->m_mainSchedule[slot][0] = u->GetId();

          // update the schedule of node u
//          tempNodeSchedule.first = slot;
//          tempNodeSchedule.second = scheduleStructForU;
//          (this)->m_nodeSchedule[u].push_back(tempNodeSchedule);

          // update the schedule of node Next
//          tempNodeSchedule.first = slot;
//          tempNodeSchedule.second = scheduleStructForNext;
//          (this)->m_nodeSchedule[next].push_back(tempNodeSchedule);

          if (next != v)
            {
              (this)->ScheduleLinks(next, v, Graph , superframe, slot, option);
            }
        }
      else
        {
          if(it==successorsOfU.begin ())
            {
              (this)->m_mainSchedule[slot][0] = u->GetId();
              (this)->m_mainSchedule[slot][1] = next->GetId();
              (this)->m_nodeScheduleN[u->GetId()][slot] = TX;
              (this)->m_nodeScheduleN[next->GetId()][slot] = RX;
              (this)->m_repLength[slot] = superframeF_1;
              // update the total schedule
//              scheduleStructForU.m_repLength = superframeF_1;
//              timeSchedulePair.second = scheduleStructForU;
//              (this)->m_schedule[slot] = timeSchedulePair;

              // update the schedule of node u
//              tempNodeSchedule.first = slot;
//              tempNodeSchedule.second = scheduleStructForU;
//              (this)->m_nodeSchedule[u].push_back(tempNodeSchedule);

              // update the schedule of node Next
//              scheduleStructForNext.m_repLength = superframeF_1;
//              tempNodeSchedule.first = slot;
//              tempNodeSchedule.second = scheduleStructForNext;
//              (this)->m_nodeSchedule[next].push_back(tempNodeSchedule);
            }
          else
            {
              slot = (this)->GetNextAvailableSlot(timeSlot + superframe, option);

              (this)->m_mainSchedule[slot][0] = u->GetId();
              (this)->m_mainSchedule[slot][1] = next->GetId();
              (this)->m_nodeScheduleN[u->GetId()][slot] = TX;
              (this)->m_nodeScheduleN[next->GetId()][slot] = RX;
              (this)->m_repLength[slot] = superframeF_1;

              // update the total schedule
//              scheduleStructForU.m_repLength = superframeF_1;
//              timeSchedulePair.second = scheduleStructForU;
//              (this)->m_schedule[slot] = timeSchedulePair;

              // update the schedule of node u
//              tempNodeSchedule.first = slot;
//              tempNodeSchedule.second = scheduleStructForU;
//              (this)->m_nodeSchedule[u].push_back(tempNodeSchedule);

              // update the schedule of node Next
//              scheduleStructForNext.m_repLength = superframeF_1;
//              tempNodeSchedule.first = slot;
//              tempNodeSchedule.second = scheduleStructForNext;
//              (this)->m_nodeSchedule[next].push_back(tempNodeSchedule);
            }

          if (next != v)
            {
              (this)->ResizeSchedule(superframeF_1);
              (this)->ScheduleLinks(next, v, Graph , superframeF_1, slot, option);
            }
        }
    }


  return true;
}

// Identify the earliest slot from t with a channel c to:
uint16_t Isa100GraphScheduling::GetNextAvailableSlot(uint16_t timeSlot, LinkOption option)
{
  NS_LOG_FUNCTION (this);
//  Slot slot;
//  slot.m_timeSlot = timeSlot;
//  slot.m_channelOffset = 0;
//  int8_t tempOffset = 0;
  int nSlot = timeSlot;

  switch (option)
  {
    case SLOT_EXCLUSIVE:
      for(; (this)->m_mainSchedule[nSlot][0] < 0; nSlot++);
//      while (tempOffset < (this)->m_maximumChannelOffsets)
//        {
//          if ((this)->m_schedule.count(slot) == 0)
//            {
//              return slot;
//            }
//          tempOffset++;
//          if (tempOffset == (this)->m_maximumChannelOffsets)
//            {
//              tempOffset = 0;
//              timeSlot++;
//            }
//          slot.m_channelOffset = tempOffset;
//          slot.m_timeSlot = timeSlot;
//        }
      break;

    case SLOT_SHARED:
      for(; (this)->m_mainSchedule[nSlot][0] < 0; nSlot++);
//      while (tempOffset < (this)->m_maximumChannelOffsets)
//        {
//          if ((this)->m_schedule.count(slot) < 5)
//            {
//              return slot;
//            }
//          tempOffset++;
//          if (tempOffset == (this)->m_maximumChannelOffsets)
//            {
//              tempOffset = 0;
//              timeSlot++;
//            }
//          slot.m_channelOffset = tempOffset;
//          slot.m_timeSlot = timeSlot;
//        }
      break;
  }

  return nSlot;
}

void Isa100GraphScheduling::PrintSchedule()
{
  NS_LOG_FUNCTION (this);
  NS_LOG_UNCOND("************ Schedule ************");
//  for (multimap<Slot, pair<Ptr<Node>,TimeScheduleStruct>>::const_iterator it = (this)->m_schedule.begin ();
//        it != (this)->m_schedule.end (); ++it)
//    {
//      NS_LOG_UNCOND("Slot: t-"<<it->first.m_timeSlot<<" c-"<<it->first.m_channelOffset);
//      NS_LOG_UNCOND("node-"<<it->second.first->GetId()<<" next-"<<it->second.second.m_nextNode<<
//                    "gID-"<<it->second.second.m_graphID<<" o-"<<it->second.second.m_option<<" lt-"<<it->second.second.m_linkType);
//    }
  for(int i = 0; i< (this)->m_mainSchedule.size(); i++)
    {
      if((this)->m_mainSchedule[i][0] > 0)
        {
          NS_LOG_UNCOND("Slot: "<<i<<" TX Node: "<<(this)->m_mainSchedule[i][0]<<" RX Node: "<<(this)->m_mainSchedule[i][1]);
        }
    }

}

void Isa100GraphScheduling::PrintNodeSchedule(Ptr<Node> node)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_UNCOND("************ Node Schedule ************ -"<<node->GetId());
//  for (vector<pair<Slot, TimeScheduleStruct>>::const_iterator it = (this)->m_nodeSchedule[node].begin ();
//        it != (this)->m_nodeSchedule[node].end (); ++it)
//    {
//      NS_LOG_UNCOND("Slot: t-"<<it->first.m_timeSlot<<" c-"<<it->first.m_channelOffset);
//      NS_LOG_UNCOND(" next-"<<it->second.m_nextNode<<
//                    "gID-"<<it->second.m_graphID<<" o-"<<it->second.m_option<<" lt-"<<it->second.m_linkType);
//    }
  for (map<uint16_t, LinkType>::const_iterator it = (this)->m_nodeScheduleN[node->GetId()].begin ();
        it != (this)->m_nodeScheduleN[node->GetId()].end (); ++it)
    {
      NS_LOG_UNCOND("Slot: "<<it->first<<" Slot Type: "<<it->second);
    }

}

void Isa100GraphScheduling::ResizeSchedule(uint32_t superframe)
{
  (this)->m_mainSchedule.resize(superframe, vector <int> (2, -1));

  for(int nSlot = 0; nSlot < (this)->m_repLength.size(); nSlot++)
    {
      int repTime = 1;
      while(repTime<(this)->m_mainSchedule.size() && (this)->m_repLength[nSlot] > 0)
        {
          (this)->m_mainSchedule[nSlot+repTime*(this)->m_repLength[nSlot]] = (this)->m_mainSchedule[nSlot];
          (this)->m_nodeScheduleN[(this)->m_mainSchedule[nSlot][0]][nSlot+repTime*(this)->m_repLength[nSlot]] = TX;
          (this)->m_nodeScheduleN[(this)->m_mainSchedule[nSlot][1]][nSlot+repTime*(this)->m_repLength[nSlot]] = RX;
          repTime++;
        }
    }
  (this)->m_repLength.resize(superframe, -1);

}

} // namespace ns3

