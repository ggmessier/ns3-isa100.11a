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

  Ptr<IsaGraph> GB = mapOfG[65535];
  Ptr<IsaGraph> GUL = mapOfG[0];
  map <uint32_t, Ptr<IsaGraph>> GDL = mapOfG;

  for (map<uint32_t, GraphNode>::const_iterator it = graphNodeMap.begin ();
        it != graphNodeMap.end (); ++it)
    {
      // Identify the set of nodes with each sample rate: N 1 , N 2 , . . . , N k .
      uint32_t tempTimeSlots = it->second.m_numTimeSlots;
//      NS_LOG_UNCOND("tempTimeSlots "<<tempTimeSlots);
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
//          NS_LOG_UNCOND("v: "<<v->GetId());
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
                                           uint32_t timeSlot, DlLinkType option)
{
  NS_LOG_FUNCTION (this);

  (this)->ResizeSchedule(superframe);

//  NS_LOG_UNCOND("Graph ID: "<<Graph->GetGraphId());

  //Identify data superframe F ′ with l F ′ = 2l F
  uint32_t superframeF_1 = superframe*2;

  vector <Ptr<Node>> successorsOfU;

  vector <Ptr<Node>> tempSuccessorsOfU = Graph->GetGraphNode(u->GetId()).m_neighbors;
  GraphNode u_graphNode = Graph->GetGraphNode(u->GetId());

  for (uint32_t nNode = 0; nNode < tempSuccessorsOfU.size(); nNode++)
    {
      GraphNode temp_graphNode = Graph->GetGraphNode(tempSuccessorsOfU[nNode]->GetId());
//      NS_LOG_UNCOND("tempSuccessorsOfU[nNode]->GetId(): "<<tempSuccessorsOfU[nNode]->GetId());
//      NS_LOG_UNCOND("u_graphNode.m_avgHopCount "<<u_graphNode.m_avgHopCount);
//      NS_LOG_UNCOND("temp_graphNode.m_avgHopCount "<<temp_graphNode.m_avgHopCount);

      if(Graph->GetGraphId() == 0 && u_graphNode.m_avgHopCount > temp_graphNode.m_avgHopCount)
        {
          successorsOfU.push_back(temp_graphNode.m_head);
        }
      else if(u_graphNode.m_avgHopCount < temp_graphNode.m_avgHopCount)
        {
          successorsOfU.push_back(temp_graphNode.m_head);
        }
    }

//  NS_LOG_UNCOND("size: "<<(this)->m_mainSchedule.size());
//  for(uint32_t j = 0; j<(this)->m_mainSchedule.size();j++)
//    {
//      if ((this)->m_mainSchedule[j][0] != 65535)
//        {
//          NS_LOG_UNCOND("schedule: "<<j<<" "<<(this)->m_mainSchedule[j][0]<<" "<<(this)->m_mainSchedule[j][1]);
//        }
//    }

//  Slot slot;
  uint32_t slot;
  slot = (this)->GetNextAvailableSlot(timeSlot, option);

  Ptr<Node> next;
  Ptr<NetDevice> baseDevice_next;
  Ptr<Isa100NetDevice> netDevice_next;

  Mac16AddressValue address_next;

  //  for all node i ∈ Successor(u) do
  for (uint32_t nNode = 0; nNode < successorsOfU.size(); nNode++)
    {
      next = successorsOfU[nNode];    // node next
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
          if(nNode == 0)
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
uint32_t Isa100Helper::GetNextAvailableSlot(uint32_t timeSlot, DlLinkType option)
{
  NS_LOG_FUNCTION (this);
  uint32_t nSlot = timeSlot;
  uint32_t slotIndex = 0;
  uint32_t frameSize = (this)->m_mainSchedule.size();
//  NS_LOG_UNCOND("frameSize: "<<frameSize);

  switch (option)
  {
    case TRANSMIT:
    case RECEIVE:
//      for(; (this)->m_mainSchedule[nSlot][0] < 65535; nSlot++);
//      NS_LOG_UNCOND("nSlot: "<<nSlot);
//      NS_LOG_UNCOND("slotIndex: "<<slotIndex);
      while((this)->m_mainSchedule[nSlot+slotIndex][0] != 65535)
        {
          slotIndex++;
          if(slotIndex > frameSize)
            {
              frameSize = frameSize*2;
              (this)->ResizeSchedule(frameSize);
            }
        }
      nSlot += slotIndex;
      break;

    case SHARED:
//      for(; (this)->m_mainSchedule[nSlot][0] < 65535; nSlot++);
//      NS_LOG_UNCOND("nSlot: "<<nSlot);
//      NS_LOG_UNCOND("slotIndex: "<<slotIndex);
      while((this)->m_mainSchedule[nSlot+slotIndex][0] != 65535)
        {
          slotIndex++;
          if(slotIndex > frameSize)
            {
              frameSize = frameSize*2;
              (this)->ResizeSchedule(frameSize);
            }
        }
      nSlot += slotIndex;
      break;
  }

  return nSlot;
}

void Isa100Helper::ResizeSchedule(uint32_t superframe)
{
  if (superframe > (this)->m_mainSchedule.size())
    {
      (this)->m_mainSchedule.resize(superframe, vector <uint32_t> (2, 65535));

      for(uint32_t nSlot = 0; nSlot < (this)->m_repLength.size(); nSlot++)
          {
            if((this)->m_repLength[nSlot] > 0)
              {
                uint32_t repTime = 1;
                uint32_t nextSlot = nSlot+repTime*(this)->m_repLength[nSlot];
                while(nextSlot<(this)->m_mainSchedule.size())
                  {
                    (this)->m_mainSchedule[nextSlot] = (this)->m_mainSchedule[nSlot];
                    (this)->m_nodeScheduleN[(this)->m_mainSchedule[nSlot][0]][nextSlot] = TRANSMIT;
                    (this)->m_nodeScheduleN[(this)->m_mainSchedule[nSlot][1]][nextSlot] = RECEIVE;
                    repTime++;
                    nextSlot = nSlot+repTime*(this)->m_repLength[nSlot];
                  }
              }
          }
      (this)->m_repLength.resize(superframe,0);
    }

}

} // namespace ns3

