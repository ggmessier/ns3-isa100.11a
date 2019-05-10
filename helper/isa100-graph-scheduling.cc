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
#include <iomanip>
#include <string>
#include <cstring>

NS_LOG_COMPONENT_DEFINE ("Isa100GraphScheduling");

using namespace std;

namespace ns3 {

SchedulingResult Isa100Helper::ConstructDataCommunicationSchedule (Ptr<IsaGraph> G, map <uint32_t, Ptr<IsaGraph>> mapOfG)
{
  NS_LOG_FUNCTION (this);   // ALG 7 - Constructing Data Communication Schedule

  // Trace for initial graph information
  (this)->PrintGraph (G);

  // Get information from the initial Graph
  map <uint32_t, GraphNode> graphNodeMap = G->GetGraphNodeMap();
  Ptr<Node> gateway = G->GetGateway();        ///< g node of the algorithm (ALG 7)

  Ptr<IsaGraph> GB = mapOfG[0xFFFF];   // broadcast graph
  Ptr<IsaGraph> GUL = mapOfG[0];    // UPLINK graph
  map <uint32_t, Ptr<IsaGraph>> GDL = mapOfG;   // DOWNLINK graphs

  for (map<uint32_t, GraphNode>::const_iterator it = graphNodeMap.begin ();
        it != graphNodeMap.end (); ++it)
    {
      // Identify the set of nodes with each sample rate: N 1 , N 2 , . . . , N k .
      uint32_t tempTimeSlots = it->second.m_numTimeSlots;
      // Sort device sample rates in ascending order: r 1 < r 2 < . . . < r k . (Here Map is storing data in sorted manner.)
      m_hopCountTrace(it->second.m_head->GetId(),it->second.m_avgHopCount);
      m_groupSameSampleRate[tempTimeSlots].push_back(it->second.m_head);
    }

  vector<Ptr<Node>> GroupwithSampleRate;

//  m_graphAllocation.resize(0xFFFF, 0);
  m_grpahID = G->GetNumofNodes();
  uint16_t MAX_Uint16 = std::numeric_limits<uint16_t>::max();     // maximum number of graph IDs with two bytes

//  NS_LOG_UNCOND("MAX/2 "<<to_string(MAX_Uint16/2 + 1));
  // Note: first half of the possible graph IDs are UL and DL primary paths (EXCLUSIVE)
  //       Second half of the graph0 IDs are UL and DL backup paths (SHARED)


  // for all r i from r 1 to r k do
  for (map<uint32_t, vector<Ptr<Node>>>::const_iterator it = m_groupSameSampleRate.begin ();
        it != (this)->m_groupSameSampleRate.end (); ++it)
    {
      GroupwithSampleRate = it->second;
      // Generate the data SUPERFRAME F i
      uint32_t superframe = it->first;    // (from above m_numTimeSlots)

      // for all node v ∈ N i do
      while(!GroupwithSampleRate.empty())
        {
          Ptr<Node> v = GroupwithSampleRate.back();
          bool scheduleFound = true;
          if(v != gateway)
            {
              // Schedule primary and retry links for publishing data
              m_paths.clear();
              m_panID = 0;
              if (m_grpahID > MAX_Uint16/2)
                m_grpahID -= MAX_Uint16/2;
//              m_grpahID = G->GetNumofNodes();
//              m_iter = v->GetId() + G->GetNumofNodes();

              scheduleFound = (this)->ScheduleLinks(v, gateway, GUL , superframe, 0, TRANSMIT,
                                                    static_cast<uint16_t>(v->GetId()));
              if (!scheduleFound)
                return INSUFFICIENT_SLOTS;  // schedule FAIL
              for (map<uint16_t, vector<Mac16Address>>::const_iterator it = m_paths.begin(); it != m_paths.end(); ++it)
                {
                  vector<Mac16Address> path = it->second;
                  m_tableList2[v->GetId()][gateway->GetId()].push_back(path);
                }
              m_pathsMap.clear();
//              for (map<uint16_t, vector<Mac16Address>>::const_iterator it = paths.begin ();
//                         it != paths.end (); ++it)
//                      {
//
//                      }
//              for (map<uint16_t, vector<Mac16Address>>::const_iterator it = paths.begin(); it != paths.end(); ++it)
//                {
//                  vector<Mac16Address> path = it->second;
//                  m_tableList2[v->GetId()][gateway->GetId()].push_back(path);
//                }

//              vector<vector<Mac16Address>> paths;
//              m_panID = 0;
//              m_grpahID = G->GetNumofNodes() + MAX_Uint16/2 + 1;
              scheduleFound = (this)->ScheduleLinks(v, gateway, GUL , superframe, superframe/4, SHARED,
                                                    static_cast<uint16_t>(v->GetId()) + MAX_Uint16/2 + 1);
              if (!scheduleFound)
                return INSUFFICIENT_SLOTS; // schedule FAIL
              m_pathsMap.clear();
              // Schedule primary and retry links for control data
//              scheduleFound = (this)->ScheduleLinks(gateway, v, GDL[v->GetId()] , superframe, superframe/2, TRANSMIT,
//                                                    static_cast<uint16_t>(v->GetId()) + MAX_Uint16/4 + 1);
//              if (!scheduleFound)
//                return INSUFFICIENT_SLOTS;  // schedule FAIL
//              scheduleFound = (this)->ScheduleLinks(gateway, v, GDL[v->GetId()] , superframe, superframe/4*3, SHARED,
//                                                    static_cast<uint16_t>(v->GetId()) + MAX_Uint16/4*3 + 1);
//              if (!scheduleFound)
//                return INSUFFICIENT_SLOTS;  // schedule FAIL
            }
          ResizeSchedule((this)->m_mainSchedule.size());
          GroupwithSampleRate.pop_back();
        }
    }

  return SCHEDULE_FOUND;
}

bool Isa100Helper::ScheduleLinks (Ptr<Node> u, Ptr<Node> v, Ptr<IsaGraph> Graph, uint32_t superframe,
                                           uint32_t timeSlot, DlLinkType option, uint16_t grpahID)
{
  NS_LOG_FUNCTION (this);   // ALG 8 - ScheduleLinks(u, v, G, F , t, o) - additionally channel offset has included.

//  (this)->ResizeSchedule(superframe);   // make sure the schedule compromise enough slots to support the SUPERFRAME

  //Identify data SUPERFRAME F ′ with l F ′ = 2l F
  uint32_t superframeF_1 = superframe*2;

  vector <Ptr<Node>> successorsOfU;

  // all successors of node u
  vector <Ptr<Node>> tempSuccessorsOfU = Graph->GetGraphNode(u->GetId()).m_neighbors;
  GraphNode u_graphNode = Graph->GetGraphNode(u->GetId());

  for (uint32_t nNode = 0; nNode < tempSuccessorsOfU.size(); nNode++)
    {
      GraphNode temp_graphNode = Graph->GetGraphNode(tempSuccessorsOfU[nNode]->GetId());

      if(Graph->GetGraphId() == 0 || Graph->GetGraphId() == 0xFFFF)
          successorsOfU.push_back(temp_graphNode.m_head);
      else if (u_graphNode.m_avgHopCount < temp_graphNode.m_avgHopCount)
          successorsOfU.push_back(temp_graphNode.m_head);
    }

  // node i in ALG 8
  Ptr<Node> next;
  Ptr<NetDevice> baseDevice_next;
  Ptr<Isa100NetDevice> netDevice_next;
  Mac16AddressValue address_next;

  //  for all node i ∈ Successor(u) do
  for (uint32_t nNode = 0; nNode < successorsOfU.size(); nNode++)
    {
      // node i in ALG 8
      next = successorsOfU[nNode];
      baseDevice_next = m_devices.Get(next->GetId());
      netDevice_next = baseDevice_next->GetObject<Isa100NetDevice>();
      netDevice_next->GetDl()->GetAttribute("Address",address_next);

      // Schedule S_u and S_i
      //m_nodeScheduleN[u->GetId()]: Schedule S_u for Node u
      //m_nodeScheduleN[next->GetId()]: Schedule S_i for Node i

//      if(count(m_tableList[u->GetId()][v->GetId()].begin(),m_tableList[u->GetId()][v->GetId()].end(),address_next.Get()) == 0)
//        {
//          m_tableList[u->GetId()][v->GetId()].push_back(address_next.Get());   //update the routing tables
//        }

      // Next available resource
      Resource resource;
      uint32_t slot;
      uint8_t chIndex;

      if (successorsOfU.size()==1)              // if next is the only successor of u then
        {
          // identify the earliest slot from t with a channel c
          resource = (this)->GetNextAvailableSlot(u->GetId(), next->GetId(),timeSlot, option, superframe);
          if(!m_ResourceAvailable)
            return false;
          slot = resource.timeSlot;
          chIndex = resource.channelIndex;

          // Allocate entries of M
          (this)->m_mainSchedule[slot][chIndex][0] = u->GetId();
          (this)->m_mainSchedule[slot][chIndex][1] = next->GetId();
          (this)->m_repLength[slot][chIndex] = superframe;

          // Allocate the slots of S_u and S_i
          Mac16Address graphIDMac = GraphIDConverter(grpahID);
          NodeInfo scheduleInfo = {m_carriers[chIndex], TRANSMIT, graphIDMac};
          (this)->m_nodeScheduleN[u->GetId()][slot] = scheduleInfo;
          scheduleInfo.slotType = RECEIVE;
          (this)->m_nodeScheduleN[next->GetId()][slot] = scheduleInfo;
          m_paths[m_panID].push_back(graphIDMac);
          m_pathsMap[next->GetId()] = m_paths[m_panID];

          NS_LOG_UNCOND("1: slot: "<<slot<<" TX: "<<to_string(u->GetId())<<" RX: "<<to_string(next->GetId())<<
                        " Rep Len: "<<superframe<<" graphID "<<graphIDMac);

          // Routing table creation
          RoutingTable rTableRecord;
          rTableRecord.destID = v->GetId();
          rTableRecord.nextGraphID = graphIDMac;
          rTableRecord.neighborList.push_back(address_next.Get());
          m_tableList[u->GetId()][graphIDMac] = rTableRecord;
//          rTableRecord.neighborList.clear();

          if (next != v)
            {
              // ScheduleLink (i, v, G, F, t_i, o)
              (this)->ScheduleLinks(next, v, Graph , superframe, slot, option, grpahID);
            }
        }
      else
        {
          if(nNode == 0)
            {
              // identify the earliest slot from t with a channel c
              (this)->ResizeSchedule(superframeF_1);
              resource = (this)->GetNextAvailableSlot(u->GetId(), next->GetId(),timeSlot, option, superframeF_1);
              if(!m_ResourceAvailable)
                return false;
              slot = resource.timeSlot;
              chIndex = resource.channelIndex;

              // Allocate entries of M
              (this)->m_mainSchedule[slot][chIndex][0] = u->GetId();
              (this)->m_mainSchedule[slot][chIndex][1] = next->GetId();
              (this)->m_repLength[slot][chIndex] = superframeF_1;

              // Allocate the slots of S_u and S_i
              Mac16Address graphIDMac = GraphIDConverter(grpahID);
              NodeInfo scheduleInfo = {m_carriers[chIndex], TRANSMIT, graphIDMac};
              (this)->m_nodeScheduleN[u->GetId()][slot] = scheduleInfo;
              scheduleInfo.slotType = RECEIVE;
              (this)->m_nodeScheduleN[next->GetId()][slot] = scheduleInfo;
              m_paths[m_panID].push_back(graphIDMac);
              m_pathsMap[next->GetId()] = m_paths[m_panID];

              NS_LOG_UNCOND("2: slot: "<<slot<<" TX: "<<to_string(u->GetId())<<" RX: "<<to_string(next->GetId())<<
                            " Rep Len: "<<superframeF_1<<" graphID "<<graphIDMac);

              // Routing table creation
              RoutingTable rTableRecord;
              rTableRecord.destID = v->GetId();
              rTableRecord.nextGraphID = graphIDMac;
              rTableRecord.neighborList.push_back(address_next.Get());
              m_tableList[u->GetId()][graphIDMac] = rTableRecord;
//              rTableRecord.neighborList.clear();

              if (next != v)
                {
                  // ScheduleLink (i, v, G, F', t_i, o)
                  (this)->ScheduleLinks(next, v, Graph , superframeF_1, slot, option, grpahID);
                }

            }
          else
            {
              // identify the earliest slot from t in M with a channel c
              (this)->ResizeSchedule(superframeF_1);
              resource = (this)->GetNextAvailableSlot(u->GetId(), next->GetId(), timeSlot + superframe, option, superframeF_1);
              if(!m_ResourceAvailable)
                return false;
              slot = resource.timeSlot;
              chIndex = resource.channelIndex;

              // Allocate entries of M
              (this)->m_mainSchedule[slot][chIndex][0] = u->GetId();
              (this)->m_mainSchedule[slot][chIndex][1] = next->GetId();
              (this)->m_repLength[slot][chIndex] = superframeF_1;

              // Allocate the slots of S_u and S_i
              if (grpahID > 0xFFFF/2 && m_grpahID < 0xFFFF/2)
                m_grpahID += 0xFFFF/2;
              grpahID = m_grpahID;
              m_grpahID++;
              m_panID++;
              while (!m_paths[m_panID].empty())
                {
                  m_panID++;
                }

              if (m_pathsMap.count(u->GetId()) != 0)
                {
//                  NS_LOG_UNCOND();
                  m_paths[m_panID] = m_pathsMap[u->GetId()];
                }
              Mac16Address graphIDMac = GraphIDConverter(grpahID);
              NodeInfo scheduleInfo = {m_carriers[chIndex], TRANSMIT, graphIDMac};
              (this)->m_nodeScheduleN[u->GetId()][slot] = scheduleInfo;
              scheduleInfo.slotType = RECEIVE;
              (this)->m_nodeScheduleN[next->GetId()][slot] = scheduleInfo;
              m_paths[m_panID].push_back(graphIDMac);
              m_pathsMap[next->GetId()] = m_paths[m_panID];

              NS_LOG_UNCOND("3: slot: "<<slot<<" TX: "<<to_string(u->GetId())<<" RX: "<<to_string(next->GetId())<<
                            " Rep Len: "<<superframeF_1<<" graphID "<<graphIDMac);

              // Routing table creation
              RoutingTable rTableRecord;
              rTableRecord.destID = v->GetId();
              rTableRecord.nextGraphID = graphIDMac;
              rTableRecord.neighborList.push_back(address_next.Get());
              m_tableList[u->GetId()][graphIDMac] = rTableRecord;
//              rTableRecord.neighborList.clear();

              if (next != v)
                {
                  // ScheduleLink (i, v, G, F', t_i, o)
                  (this)->ScheduleLinks(next, v, Graph , superframeF_1, slot, option, grpahID);
                }


//              grpahID++;
            }



//          if (next != v)
//            {
//              // ScheduleLink (i, v, G, F', t_i, o)
//              (this)->ScheduleLinks(next, v, Graph , superframeF_1, slot, option, grpahID);
//            }
        }
    }

  return true;
}

// Identify the earliest slot from t with a channel c to:
Resource Isa100Helper::GetNextAvailableSlot(uint32_t u, uint32_t v, uint32_t timeSlot, DlLinkType option, uint32_t repLength)
{
  Resource resource;
  uint8_t chIndex = 0;

  NS_LOG_FUNCTION (this);
  uint32_t nSlot = timeSlot;
  uint32_t slotIndex = 0;
  uint32_t frameSize = (this)->m_mainSchedule.size();
  bool found = false;
  bool tempFound = false;

  switch (option)
  {
    case TRANSMIT:
    case RECEIVE:
      while(!found)
        {
          uint32_t i = nSlot+slotIndex;
          if ((this)->m_mainSchedule[i][chIndex][0] == 0xFFFF)
            {
              tempFound = true;
              while (i < frameSize)
                {
                  if((this)->m_mainSchedule[i][chIndex][0] == 0xFFFF)
                    {
                      if((this)->m_nodeScheduleN[u].count(i) != 0 || (this)->m_nodeScheduleN[v].count(i) != 0)
                          tempFound = false;
                    }
                  else
                      tempFound = false;

                  if(!tempFound)
                    break;

                  i = i + repLength;
                }
            }
          if (!tempFound)
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
                      found = true;
                    }
                }
            }
          else
            found = true;
        }
      nSlot += slotIndex;
      break;

    case SHARED:
      while(!found)
        {
         uint32_t i = nSlot+slotIndex;
         if ((this)->m_mainSchedule[i][chIndex][0] == 0xFFFF)
           {
             tempFound = true;
             while (i < frameSize)
               {
                 if((this)->m_mainSchedule[i][chIndex][0] == 0xFFFF)
                   {
                     if((this)->m_nodeScheduleN[u].count(i) != 0 || (this)->m_nodeScheduleN[v].count(i) != 0)
                         tempFound = false;
                   }
                 else
                     tempFound = false;

                 if(!tempFound)
                   break;

                 i = i + repLength;
               }
           }
         if (!tempFound)
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
                     found = true;
                   }
               }
           }
         else
           found = true;
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
  NS_LOG_FUNCTION (this);
  NS_LOG_UNCOND("m_panID "<<m_panID);
  if (m_panID > 0)
    {
      for (uint16_t i = 0; i < m_panID; i ++)
        {
          NS_LOG_UNCOND("m_panID "<<m_panID<<" i "<<i);
          m_paths[m_panID + i + 1] = m_paths[i];
        }
    }
  uint8_t numChannels = m_carriers.size();
  uint32_t initRepLength = (this)->m_repLength.size();
  if (superframe > (this)->m_mainSchedule.size())
    {
      // resize the schedule to support new # of slots (SUPERFRAME)
      (this)->m_mainSchedule.resize(superframe, vector<vector <uint32_t>> (numChannels, vector <uint32_t> (2, 0xFFFF)));
      (this)->m_repLength.resize(superframe,vector<uint32_t> (numChannels));

    }

  if (superframe >= (this)->m_mainSchedule.size())
    {
      for(uint32_t nSlot = 0; nSlot < initRepLength; nSlot++)
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

                        // populate the repeating schedules through the new SUPERFRAME
                        uint32_t TxNode = (this)->m_mainSchedule[nSlot][chIndex][0];
                        uint32_t RxNode = (this)->m_mainSchedule[nSlot][chIndex][1];
                        (this)->m_nodeScheduleN[TxNode][nextSlot] = (this)->m_nodeScheduleN[TxNode][nSlot];
                        (this)->m_nodeScheduleN[RxNode][nextSlot] = (this)->m_nodeScheduleN[RxNode][nSlot];
                        (this)->m_repLength[nextSlot][chIndex] = (this)->m_repLength[nSlot][chIndex];

                        NS_LOG_DEBUG("ReSize: slot: "<<nextSlot<<" TX: "<<to_string(TxNode)<<" RX: "<<to_string(RxNode)<<" Rep Len: "<<(this)->m_repLength[nextSlot][chIndex]);

                        repTime++;
                        nextSlot = nSlot+repTime*(this)->m_repLength[nSlot][chIndex];
                      }
                  }
              }

          }
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

void Isa100Helper::PrintGraphSchedule ()
{
  NS_LOG_FUNCTION (this);

  NS_LOG_DEBUG("size: "<<(this)->m_mainSchedule.size());
  for(uint32_t j = 0; j<(this)->m_mainSchedule.size();j++)
    {
      for(uint32_t k = 0; k<(this)->m_mainSchedule[j].size();k++)
        {
          if ((this)->m_mainSchedule[j][k][0] != 0xFFFF)
            {
              m_scheduleTrace(j,(this)->m_mainSchedule[j][k][0],(this)->m_mainSchedule[j][k][1],(this)->m_carriers[k]);
            }
        }
    }

}

SchedulingResult Isa100Helper::ScheduleAndRouteTDMAgraph()
{
  NS_LOG_FUNCTION (this);
  uint32_t numNodes = m_devices.GetN();
  SchedulingResult schedulingResult = SCHEDULE_FOUND;

  (this)->PrintGraphSchedule ();

  (this)->SetDlAttribute("SuperFramePeriod",UintegerValue(m_mainSchedule.size()));

  map<uint8_t, uint8_t> channelIndexMap; // channel -> channel index
  for(uint8_t i; i< m_carriers.size();i++)
    channelIndexMap[m_carriers[i]] = i;

//  map<uint32_t, map<uint32_t, vector<vector<Mac16Address>>>> m_tableList2;

  for(uint32_t nNode=0; nNode < numNodes; nNode++)
  {
    NS_LOG_UNCOND("Here");
    for (map<uint32_t, vector<vector<Mac16Address>>>::const_iterator it = m_tableList2[nNode].begin ();
               it != m_tableList2[nNode].end (); ++it)
        {
          NS_LOG_UNCOND("Here Table 2");
          NS_LOG_UNCOND("Table List 2 ****** node: "<<nNode<<" dst: "<<it->first);
          for(int j = 0; j < it->second.size(); j++)
            {
              vector<Mac16Address> path = it->second[j];
              NS_LOG_UNCOND(" panID: "<<j);
              for(int k = 0; k < path.size(); k++)
                {
                  NS_LOG_UNCOND(path[k]);
                }
            }
        }
  }

  for(uint32_t nNode=0; nNode < numNodes; nNode++)
  {
    NS_LOG_UNCOND("Here");
    for (map<Mac16Address, RoutingTable>::const_iterator it = m_tableList[nNode].begin ();
               it != m_tableList[nNode].end (); ++it)
        {
          RoutingTable rt = it->second;
          for(int j = 0; j < rt.neighborList.size(); j++)
            {
              NS_LOG_UNCOND("Table List 1 ****** node: "<<nNode<<" dst: "<<rt.destID<<" cg: "<<it->first
                               <<" ng: "<<rt.nextGraphID<<" neighbor: "<<rt.neighborList[j]);
            }
        }
  }

  for(uint32_t nNode=0; nNode < numNodes; nNode++)
  {
    NodeSchedule nodeSchedule;
    // this is to match with the routing table used in routing; map <destination ID, graph ID list> m_Table
    map<uint32_t, vector<Mac16Address>> table;
    map<Mac16Address, pair<Mac16Address, vector<Mac16Address>>> graphTable; // graphID -> next graph ID, neighbor

//    NS_LOG_UNCOND("Here");
//    for (map<Mac16Address, RoutingTable>::const_iterator it = m_tableList[nNode].begin ();
//               it != m_tableList[nNode].end (); ++it)
//        {
//          RoutingTable rt = it->second;
//          for(int j = 0; j < rt.neighborList.size(); j++)
//            {
//              NS_LOG_UNCOND("****** node: "<<nNode<<" dst: "<<rt.destID<<" cg: "<<it->first
//                               <<" ng: "<<rt.nextGraphID<<" neighbor: "<<rt.neighborList[j]);
//            }
//        }

//    NS_LOG_UNCOND("ScheduleAndRouteTDMAgraph");
    // Assign schedule to DL
    Ptr<NetDevice> baseDevice = m_devices.Get(nNode);
    Ptr<Isa100NetDevice> netDevice = baseDevice->GetObject<Isa100NetDevice>();

    vector<uint8_t> hoppingPattern;
    NS_LOG_UNCOND("nNode for Node schedule: "<<nNode);
    for (map<uint32_t, NodeInfo>::const_iterator it = m_nodeScheduleN[nNode].begin ();
           it != m_nodeScheduleN[nNode].end (); ++it)
        {
          nodeSchedule.slotSched.push_back(it->first);
          nodeSchedule.slotType.push_back(it->second.slotType);
          hoppingPattern.push_back(it->second.channelSched);

          uint32_t dstID = m_tableList[nNode][it->second.graphID].destID;
          if (count(table[dstID].begin(), table[dstID].end(),it->second.graphID) == 0)
            table[dstID].push_back(it->second.graphID);
//          table = m_tableList2;

//          NS_LOG_UNCOND("nNode: "<<nNode<<" dstID: "<<dstID);
//          NS_LOG_UNCOND("slot: "<<it->first<<" ch: Type "<<to_string(it->second.slotType)<<" Channel: "<<to_string(it->second.channelSched)<<
//                        " cg: "<<it->second.graphID);


          if(m_tableList[nNode][it->second.graphID].neighborList.size() > 0)
            {
              // populate the current graph -> next graph and next node MacAddress mapping
              NS_LOG_UNCOND("graphTable: cg "<<it->second.graphID<<" ng "<<m_tableList[nNode][it->second.graphID].nextGraphID<<" neigh: "
                            <<m_tableList[nNode][it->second.graphID].neighborList[0]);
              graphTable[it->second.graphID].first  = m_tableList[nNode][it->second.graphID].neighborList[0];
              vector<Mac16Address> backUpGraphSequence;
              backUpGraphSequence.push_back(m_tableList[nNode][it->second.graphID].nextGraphID);
              graphTable[it->second.graphID].second = backUpGraphSequence;

            }


//          uint32_t rxNode = m_mainSchedule[it->first][channelIndexMap[it->second.channelSched]][1];
//          Mac16AddressValue address;
//          Ptr<NetDevice> nextDevice = m_devices.Get(rxNode);
//          Ptr<Isa100NetDevice> nextNetDevice = baseDevice->GetObject<Isa100NetDevice>();
//          nextNetDevice->GetDl()->GetAttribute("Address",address);
//          graphTable[it->second.graphID].second = address.Get();

        }

    netDevice->GetDl()->SetAttribute("SuperFramePeriod", UintegerValue(m_mainSchedule.size()));

//    NS_LOG_UNCOND("Here");
//    for(int i = 0; i < m_tableList[nNode].size(); i++)
//        {
//          RoutingTable rt = m_tableList[nNode][i];
//          for(int j = 0; j < rt.neighborList.size(); j++)
//            {
//              NS_LOG_UNCOND("node: "<<nNode<<" dst: "<<rt.destID<<" cg: "<<rt.graphID
//                               <<" ng: "<<rt.nextGraphID<<" neighbor: "<<rt.neighborList[j]<<" flagP: "<<rt.primaryPath);
//            }
//        }

    if(!baseDevice || !netDevice)
      NS_FATAL_ERROR("Installing TDMA schedule on non-existent ISA100 net device.");

//    Ptr<Isa100RoutingAlgorithm> routingAlgorithm = CreateObject<Isa100GraphRoutingAlgorithm>(table);
//    routingAlgorithm->SetGraphTable(graphTable);
//    routingAlgorithm->SetTable(m_tableList2[nNode]);
    Ptr<Isa100RoutingAlgorithm> routingAlgorithm = CreateObject<Isa100GraphRoutingAlgorithm>(m_tableList2[nNode]);
    routingAlgorithm->SetGraphTable(graphTable);
    netDevice->GetDl()->SetRoutingAlgorithm(routingAlgorithm);

    Mac16AddressValue address;
    netDevice->GetDl()->GetAttribute("Address",address);
    netDevice->GetDl()->GetRoutingAlgorithm()->SetAttribute("Address",address);

    // Set the tx power levels in DL
    netDevice->GetDl()->SetTxPowersDbm(m_txPwrDbm[nNode], numNodes);

    // Set the sfSchedule
    Ptr<Isa100DlSfSchedule> schedulePtr = CreateObject<Isa100DlSfSchedule>();

    schedulePtr->SetSchedule(hoppingPattern,nodeSchedule.slotSched,nodeSchedule.slotType);

    netDevice->GetDl()->SetDlSfSchedule(schedulePtr);
  }

  return schedulingResult;
}

Mac16Address Isa100Helper::GraphIDConverter (uint16_t graphID)
{
  NS_LOG_FUNCTION (this);

  // convert the integer graphID to Hex value
  std::stringstream stream;
  stream << std::hex << static_cast<int>(graphID);
  string graphIDHexString = stream.str();

  // 2 Byte Hex value is convertible to a maximum of size 4 array
  char * cstr = new char [4];
  std::strcpy (cstr, graphIDHexString.c_str());

  char gAddr[5] = {'0','0',':','0','0'};

  switch (graphIDHexString.size())
  {
     case 1: gAddr[4] = cstr[0]; break;
     case 2: gAddr[3] = cstr[0]; gAddr[4] = cstr[1]; break;
     case 3: gAddr[1] = cstr[0]; gAddr[3] = cstr[1]; gAddr[4] = cstr[2]; break;
     case 4: gAddr[0] = cstr[0]; gAddr[1] = cstr[1]; gAddr[3] = cstr[2]; gAddr[4] = cstr[3]; break;
  }
  return Mac16Address(gAddr);
}

} // namespace ns3

