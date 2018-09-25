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

#include "ns3/isa100-helper.h"
#include "ns3/isa-graph.h"
#include "ns3/log.h"
#include "ns3/type-id.h"
#include <algorithm>

NS_LOG_COMPONENT_DEFINE ("Isa100GraphScheduling");

using namespace std;

namespace ns3 {

bool Isa100Helper::ConstructDataCommunicationSchedule (Ptr<IsaGraph> G, map <uint32_t, Ptr<IsaGraph>> mapOfG)
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
      m_groupSameSampleRate[tempTimeSlots].push_back(it->second.m_head);
    }

  vector<Ptr<Node>> GroupwithSampleRate;

  // for all r i from r 1 to r k do
  for (map<uint32_t, vector<Ptr<Node>>>::const_iterator it = m_groupSameSampleRate.begin ();
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
              (this)->ScheduleLinks(v, gateway, GUL , superframe, 0, TRANSMIT);
              (this)->ScheduleLinks(v, gateway, GUL , superframe, superframe/4, SHARED);
              // Schedule primary and retry links for control data
              (this)->ScheduleLinks(gateway, v, GDL[v->GetId()] , superframe, superframe/2, TRANSMIT);
              (this)->ScheduleLinks(gateway, v, GDL[v->GetId()] , superframe, superframe/4*3, SHARED);
            }
          GroupwithSampleRate.pop_back();
        }
    }

  return true;
}

// ScheduleLinks(u, v, G, F , t, o) - additionally channel offset has included.
bool Isa100Helper::ScheduleLinks (Ptr<Node> u, Ptr<Node> v, Ptr<IsaGraph> Graph, uint32_t superframe,
                                           uint16_t timeSlot, DlLinkType option)
{
  NS_LOG_FUNCTION (this);
  //Identify data superframe F ′ with l F ′ = 2l F
  uint32_t superframeF_1 = superframe*2;
  vector <Ptr<Node>> successorsOfU = Graph->GetGraphNode(u->GetId()).m_neighbors;

//  Slot slot;
  uint16_t slot;
  slot = (this)->GetNextAvailableSlot(timeSlot, option);

  Ptr<Node> next;
  Ptr<NetDevice> baseDevice_next;
  Ptr<Isa100NetDevice> netDevice_next;

  Mac16AddressValue address_next;

  //  for all node i ∈ Successor(u) do
  for (vector<Ptr<Node>>::const_iterator it = successorsOfU.begin();
        it != successorsOfU.end(); ++it)
    {
      next = successorsOfU.back();    // node next
      baseDevice_next = m_devices.Get(next->GetId());
      netDevice_next = baseDevice_next->GetObject<Isa100NetDevice>();

      netDevice_next->GetDl()->GetAttribute("Address",address_next);

      m_tableList[u->GetId()][v->GetId()].push_back(address_next.Get());   //update the routing tables

      if (successorsOfU.size()==1)              // if next is the only successor of u then
        {
          (this)->m_mainSchedule[slot][0] = u->GetId();
          (this)->m_mainSchedule[slot][1] = next->GetId();
          (this)->m_nodeScheduleN[u->GetId()][slot] = TRANSMIT;
          (this)->m_nodeScheduleN[next->GetId()][slot] = RECEIVE;
          (this)->m_repLength[slot] = superframe;

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
              (this)->m_nodeScheduleN[u->GetId()][slot] = TRANSMIT;
              (this)->m_nodeScheduleN[next->GetId()][slot] = RECEIVE;
              (this)->m_repLength[slot] = superframeF_1;
            }
          else
            {
              slot = (this)->GetNextAvailableSlot(timeSlot + superframe, option);

              (this)->m_mainSchedule[slot][0] = u->GetId();
              (this)->m_mainSchedule[slot][1] = next->GetId();
              (this)->m_nodeScheduleN[u->GetId()][slot] = TRANSMIT;
              (this)->m_nodeScheduleN[next->GetId()][slot] = RECEIVE;
              (this)->m_repLength[slot] = superframeF_1;
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
uint16_t Isa100Helper::GetNextAvailableSlot(uint16_t timeSlot, DlLinkType option)
{
  NS_LOG_FUNCTION (this);
  int nSlot = timeSlot;

  switch (option)
  {
    case TRANSMIT:
    case RECEIVE:
      for(; (this)->m_mainSchedule[nSlot][0] < 0; nSlot++);
      break;

    case SHARED:
      for(; (this)->m_mainSchedule[nSlot][0] < 0; nSlot++);
      break;
  }

  return nSlot;
}

void Isa100Helper::ResizeSchedule(uint32_t superframe)
{
  (this)->m_mainSchedule.resize(superframe, vector <int> (2, -1));

  for(unsigned int nSlot = 0; nSlot < (this)->m_repLength.size(); nSlot++)
    {
      unsigned int repTime = 1;
      while(repTime<(this)->m_mainSchedule.size() && (this)->m_repLength[nSlot] > 0)
        {
          (this)->m_mainSchedule[nSlot+repTime*(this)->m_repLength[nSlot]] = (this)->m_mainSchedule[nSlot];
          (this)->m_nodeScheduleN[(this)->m_mainSchedule[nSlot][0]][nSlot+repTime*(this)->m_repLength[nSlot]] = TRANSMIT;
          (this)->m_nodeScheduleN[(this)->m_mainSchedule[nSlot][1]][nSlot+repTime*(this)->m_repLength[nSlot]] = RECEIVE;
          repTime++;
        }
    }
  (this)->m_repLength.resize(superframe, -1);

}

} // namespace ns3

