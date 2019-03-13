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

SchedulingResult Isa100Helper::ConstructDataCommunicationSchedule (Ptr<IsaGraph> G, map <uint32_t, Ptr<IsaGraph>> mapOfG)
{
  NS_LOG_FUNCTION (this);

  (this)->PrintGraph (G);
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
      m_hopCountTrace(it->second.m_head->GetId(),it->second.m_avgHopCount);
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
          bool scheduleFound = true;
          if(v != gateway)
            {
              // Schedule primary and retry links for publishing data
              scheduleFound = (this)->ScheduleLinks(v, gateway, GUL , superframe, 0, TRANSMIT);
              if (!scheduleFound)
                return INSUFFICIENT_SLOTS;
              scheduleFound = (this)->ScheduleLinks(v, gateway, GUL , superframe, superframe/4, SHARED);
              if (!scheduleFound)
                return INSUFFICIENT_SLOTS;
              // Schedule primary and retry links for control data
//              scheduleFound = (this)->ScheduleLinks(gateway, v, GDL[v->GetId()] , superframe, superframe/2, TRANSMIT);
//              if (!scheduleFound)
//                return INSUFFICIENT_SLOTS;
//              scheduleFound = (this)->ScheduleLinks(gateway, v, GDL[v->GetId()] , superframe, superframe/4*3, SHARED);
//              if (!scheduleFound)
//                return INSUFFICIENT_SLOTS;
            }
          GroupwithSampleRate.pop_back();
        }
    }

  return SCHEDULE_FOUND;
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

  Resource resource = (this)->GetNextAvailableSlot(timeSlot, option);
  if(!m_ResourceAvailable)
    return false;
  uint32_t slot = resource.timeSlot;
  uint8_t chIndex = resource.channelIndex;

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

      if(count(m_tableList[u->GetId()][v->GetId()].begin(),m_tableList[u->GetId()][v->GetId()].end(),address_next.Get()) == 0)
        {
          m_tableList[u->GetId()][v->GetId()].push_back(address_next.Get());   //update the routing tables
        }

      // check the slot has not previously used by either TX or RX node.
      while((this)->m_nodeScheduleN[u->GetId()].count(slot) > 0 || (this)->m_nodeScheduleN[next->GetId()].count(slot) > 0)
        {
          resource = GetNextAvailableSlot(++slot, option);
          if(!m_ResourceAvailable)
            return false;
          slot = resource.timeSlot;
          chIndex = resource.channelIndex;
        }

      if (successorsOfU.size()==1)              // if next is the only successor of u then
        {

          (this)->m_mainSchedule[slot][chIndex][0] = u->GetId();
          (this)->m_mainSchedule[slot][chIndex][1] = next->GetId();
          (this)->m_nodeScheduleN[u->GetId()][slot] = pair<uint8_t, DlLinkType> (m_carriers[chIndex],TRANSMIT);
          (this)->m_nodeScheduleN[next->GetId()][slot] = pair<uint8_t, DlLinkType> (m_carriers[chIndex],RECEIVE);
          (this)->m_repLength[slot][chIndex] = superframe;

          if (next != v)
            {
              (this)->ScheduleLinks(next, v, Graph , superframe, slot, option);
            }
        }
      else
        {
          if(nNode == 0)
            {
              (this)->m_mainSchedule[slot][chIndex][0] = u->GetId();
              (this)->m_mainSchedule[slot][chIndex][1] = next->GetId();
              (this)->m_nodeScheduleN[u->GetId()][slot] = pair<uint8_t, DlLinkType> (m_carriers[chIndex],TRANSMIT);
              (this)->m_nodeScheduleN[next->GetId()][slot] = pair<uint8_t, DlLinkType> (m_carriers[chIndex],RECEIVE);
              (this)->m_repLength[slot][chIndex] = superframeF_1;
            }
          else
            {
              resource = (this)->GetNextAvailableSlot(timeSlot + superframe, option);
              if(!m_ResourceAvailable)
                return false;
              slot = resource.timeSlot;
              chIndex = resource.channelIndex;

              // check the slot has not previously used by either TX or RX node.
              while((this)->m_nodeScheduleN[u->GetId()].count(slot) > 0 || (this)->m_nodeScheduleN[next->GetId()].count(slot) > 0)
                {
                  resource = GetNextAvailableSlot(++slot, option);
                  if(!m_ResourceAvailable)
                    return false;
                  slot = resource.timeSlot;
                  chIndex = resource.channelIndex;
                }

              (this)->m_mainSchedule[slot][chIndex][0] = u->GetId();
              (this)->m_mainSchedule[slot][chIndex][1] = next->GetId();
              (this)->m_nodeScheduleN[u->GetId()][slot] = pair<uint8_t, DlLinkType> (m_carriers[chIndex],TRANSMIT);
              (this)->m_nodeScheduleN[next->GetId()][slot] = pair<uint8_t, DlLinkType> (m_carriers[chIndex],RECEIVE);
              (this)->m_repLength[slot][chIndex] = superframeF_1;
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
Resource Isa100Helper::GetNextAvailableSlot(uint32_t timeSlot, DlLinkType option)
{
  Resource resource;
  uint8_t chIndex = 0;

  NS_LOG_FUNCTION (this);
  uint32_t nSlot = timeSlot;
  uint32_t slotIndex = 0;
  uint32_t frameSize = (this)->m_mainSchedule.size();

  switch (option)
  {
    case TRANSMIT:
    case RECEIVE:
      while((this)->m_mainSchedule[nSlot+slotIndex][chIndex][0] != 65535)
        {
          chIndex++;
          if (chIndex >= m_carriers.size())
            {
              chIndex = 0;
              slotIndex++;
              if(slotIndex > frameSize)
                {
                  NS_LOG_UNCOND("INSUFFICIENT SLOTS");
                  m_ResourceAvailable = false;
                }
            }

        }
      nSlot += slotIndex;
      break;

    case SHARED:

      while((this)->m_mainSchedule[nSlot+slotIndex][chIndex][0] != 65535)
        {
          chIndex++;
          if (chIndex >= m_carriers.size())
            {
              chIndex = 0;
              slotIndex++;
              if(slotIndex > frameSize)
                {
                  NS_LOG_UNCOND("INSUFFICIENT SLOTS");
                  m_ResourceAvailable = false;
                }
            }

        }
      nSlot += slotIndex;
      break;
  }

  resource.channelIndex = chIndex;
  resource.timeSlot = nSlot;
  return resource;
}

void Isa100Helper::ResizeSchedule(uint32_t superframe)
{
  uint8_t numChannels = m_carriers.size();
  if (superframe > (this)->m_mainSchedule.size())
    {
      // resize the schedule to support new # of slots (superframe)
      (this)->m_mainSchedule.resize(superframe, vector<vector <uint32_t>> (numChannels, vector <uint32_t> (2, 65535)));

      for(uint32_t nSlot = 0; nSlot < (this)->m_repLength.size(); nSlot++)
          {
            for(uint32_t chIndex = 0; chIndex < (this)->m_repLength[nSlot].size(); chIndex++)
              {
                if((this)->m_repLength[nSlot][chIndex] > 0)
                  {
                    uint32_t repTime = 1;
                    uint32_t nextSlot = nSlot+repTime*(this)->m_repLength[nSlot][chIndex];
                    while(nextSlot<(this)->m_mainSchedule.size())
                      {
                        (this)->m_mainSchedule[nextSlot][chIndex] = (this)->m_mainSchedule[nSlot][chIndex];

                        // populate the repeating schedules through the new superframe
                        uint32_t TxNode = (this)->m_mainSchedule[nSlot][chIndex][0];
                        uint32_t RxNode = (this)->m_mainSchedule[nSlot][chIndex][1];
                        (this)->m_nodeScheduleN[TxNode][nextSlot] = (this)->m_nodeScheduleN[TxNode][nSlot];
                        (this)->m_nodeScheduleN[RxNode][nextSlot] = (this)->m_nodeScheduleN[RxNode][nSlot];

                        repTime++;
                        nextSlot = nSlot+repTime*(this)->m_repLength[nSlot][chIndex];
                      }
                  }
              }

          }
      (this)->m_repLength.resize(superframe,vector<uint32_t> (numChannels));
    }

}

void Isa100Helper::PrintGraph (Ptr<IsaGraph> Graph)
{
  NS_LOG_FUNCTION (this);
  map<uint32_t, GraphNode> graphMap = Graph->GetGraphNodeMap();
  for (map<uint32_t, GraphNode>::const_iterator it = graphMap.begin ();it != graphMap.end (); ++it)
    {
      vector<Ptr<Node> > tempNodeList = it->second.m_neighbors;

      while (!tempNodeList.empty ())
        {
          m_graphTrace(it->second.m_head->GetId (),tempNodeList.back ()->GetId ());
          tempNodeList.pop_back ();
        }

    }
}

} // namespace ns3

