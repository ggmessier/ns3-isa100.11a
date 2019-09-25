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

SchedulingResult Isa100Helper::ConstructDataCommunicationSchedule (Ptr<IsaGraph> G, map <uint32_t, Ptr<IsaGraph> > mapOfG)
{
  NS_LOG_FUNCTION (this);   // ALG 7 - Constructing Data Communication Schedule

  // Trace for initial graph information
  (this)->PrintGraph (G);

  // Get information from the initial Graph
  map <uint32_t, GraphNode> graphNodeMap = G->GetGraphNodeMap ();
  Ptr<Node> gateway = G->GetGateway ();        ///< g node of the algorithm (ALG 7)

  Ptr<IsaGraph> GB = mapOfG[0xFFFF];   // broadcast graph
  Ptr<IsaGraph> GUL = mapOfG[0];    // UPLINK graph
  map <uint32_t, Ptr<IsaGraph> > GDL = mapOfG;   // DOWNLINK graphs

  for (map<uint32_t, GraphNode>::const_iterator it = graphNodeMap.begin ();
       it != graphNodeMap.end (); ++it)
    {
      // Identify the set of nodes with each sample rate: N 1 , N 2 , . . . , N k .
      uint32_t tempTimeSlots = it->second.m_numTimeSlots;
      // Sort device sample rates in ascending order: r 1 < r 2 < . . . < r k . (Here Map is storing data in sorted manner.)
      m_hopCountTrace (it->second.m_head->GetId (),it->second.m_avgHopCount);
      m_groupSameSampleRate[tempTimeSlots].push_back (it->second.m_head);
    }

  vector<Ptr<Node> > GroupwithSampleRate;

//  m_graphAllocation.resize(0xFFFF, 0);
  m_grpahID = G->GetNumofNodes ();
  uint16_t MAX_Uint16 = std::numeric_limits<uint16_t>::max ();     // maximum number of graph IDs with two bytes

  // Note: first half of the possible graph IDs are UL and DL primary paths (EXCLUSIVE)
  //       Second half of the graph0 IDs are UL and DL backup paths (SHARED)

  // for all r i from r 1 to r k do
  for (map<uint32_t, vector<Ptr<Node> > >::const_iterator it = m_groupSameSampleRate.begin ();
       it != (this)->m_groupSameSampleRate.end (); ++it)
    {
      GroupwithSampleRate = it->second;
      // Generate the data SUPERFRAME F i
      uint32_t superframe = it->first;    // (from above m_numTimeSlots)

      // for all node v ∈ N i do
      while (!GroupwithSampleRate.empty ())
        {
          Ptr<Node> v = GroupwithSampleRate.back ();
          bool scheduleFound = true;
          if (v != gateway)
            {
              // Schedule primary and retry links for publishing data
              if (m_grpahID > MAX_Uint16 / 2)
                {
                  m_grpahID -= MAX_Uint16 / 2;
                }

              scheduleFound = (this)->ScheduleLinks (v, gateway, GUL, superframe, 0, (DlLinkType)TRANSMIT,
                                                     static_cast<uint16_t> (v->GetId ()), v->GetId ());
              if (!scheduleFound)
                {
                  return INSUFFICIENT_SLOTS; // schedule FAIL
                }

//              scheduleFound = (this)->ScheduleLinks(v, gateway, GUL , superframe, superframe/4, (DlLinkType) SHARED,
//                                                    static_cast<uint16_t>(v->GetId()) + MAX_Uint16/2 + 1, v);

//              scheduleFound = (this)->ScheduleLinks(v, gateway, GUL , superframe, superframe/4, (DlLinkType) SHARED,
//                                                    static_cast<uint16_t>(v->GetId()), v);
//              if (!scheduleFound)
//                return INSUFFICIENT_SLOTS; // schedule FAIL

              // Schedule primary and retry links for control data
//              scheduleFound = (this)->ScheduleLinks(gateway, v, GDL[v->GetId()] , superframe, superframe/2, (DlLinkType) TRANSMIT,
//                                                    static_cast<uint16_t>(v->GetId()) + MAX_Uint16/4 + 1, v);
//              if (!scheduleFound)
//                return INSUFFICIENT_SLOTS;  // schedule FAIL
//              scheduleFound = (this)->ScheduleLinks(gateway, v, GDL[v->GetId()] , superframe, superframe/4*3, (DlLinkType) SHARED,
//                                                    static_cast<uint16_t>(v->GetId()) + MAX_Uint16/4*3 + 1), v;
//              if (!scheduleFound)
//                return INSUFFICIENT_SLOTS;  // schedule FAIL
            }
          ResizeSchedule ((this)->m_mainSchedule.size ());
          GroupwithSampleRate.pop_back ();
        }
    }

  return SCHEDULE_FOUND;
}

bool Isa100Helper::ScheduleLinks (Ptr<Node> u, Ptr<Node> v, Ptr<IsaGraph> Graph, uint32_t superframe,
                                  uint32_t timeSlot, DlLinkType option, uint16_t grpahID, uint32_t source)
{
  NS_LOG_FUNCTION (this);   // ALG 8 - ScheduleLinks(u, v, G, F , t, o) - additionally channel offset has included.

  //Identify data SUPERFRAME F ′ with l F ′ = 2l F
  uint32_t superframeF_1 = superframe * 2;

  vector <Ptr<Node> > successorsOfU;

  // all successors of node u
  vector <Ptr<Node> > tempSuccessorsOfU = Graph->GetGraphNode (u->GetId ()).m_neighbors;
  GraphNode u_graphNode = Graph->GetGraphNode (u->GetId ());

  for (uint32_t nNode = 0; nNode < tempSuccessorsOfU.size (); nNode++)
    {
      GraphNode temp_graphNode = Graph->GetGraphNode (tempSuccessorsOfU[nNode]->GetId ());

      if (Graph->GetGraphId () == 0 || Graph->GetGraphId () == 0xFFFF)
        {
          successorsOfU.push_back (temp_graphNode.m_head);
        }
      else if (u_graphNode.m_avgHopCount < temp_graphNode.m_avgHopCount)
        {
          successorsOfU.push_back (temp_graphNode.m_head);
        }
    }

  // node i in ALG 8
  Ptr<Node> next;

  //  for all node i ∈ Successor(u) do
  for (uint32_t nNode = 0; nNode < successorsOfU.size (); nNode++)
    {
      // node i in ALG 8
      next = successorsOfU[nNode];

      // Next available resource
      Resource resource;
      uint32_t slot;
      uint8_t chIndex;

      if (successorsOfU.size () == 1)              // if next is the only successor of u then
        {
          // identify the earliest slot from t with a channel c
          resource = (this)->GetNextAvailableSlot (u->GetId (), next->GetId (),timeSlot, option, superframe);
          if (!m_ResourceAvailable)
            {
              return false;
            }
          slot = resource.timeSlot;
          chIndex = resource.channelIndex;

          // Allocate entries of M
          (this)->m_mainSchedule[slot][chIndex][0] = u->GetId ();
          (this)->m_mainSchedule[slot][chIndex][1] = next->GetId ();
          (this)->m_repLength[slot][chIndex] = superframe;

          // Allocate the slots of S_u and S_i
          Mac16Address graphIDMac = GraphIDConverter (grpahID);
          NodeInfo scheduleInfo = {m_carriers[chIndex], TRANSMIT, graphIDMac, m_panID, source};
          (this)->m_nodeScheduleN[u->GetId ()][slot] = scheduleInfo;
          scheduleInfo.slotType = RECEIVE;
          (this)->m_nodeScheduleN[next->GetId ()][slot] = scheduleInfo;

//          NS_LOG_UNCOND("1: slot: "<<slot<<" TX: "<<to_string(u->GetId())<<" RX: "<<to_string(next->GetId())<<
//                        " Rep Len: "<<superframe<<" graphID "<<graphIDMac);

          // Routing table creation {destID, nextGraphID,
          RoutingTable rTableRecord = {v->GetId (), graphIDMac, vector<uint32_t> (1,next->GetId ())};
          m_tableList[u->GetId ()][graphIDMac] = rTableRecord;

          if (next != v)
            {
              // ScheduleLink (i, v, G, F, t_i, o)
              (this)->ScheduleLinks (next, v, Graph, superframe, slot, option, grpahID, source);
            }
        }
      else
        {
          if (nNode == 0)    // if i is the first successor then
            {
              // identify the earliest slot from t with a channel c
              (this)->ResizeSchedule (superframeF_1);
              resource = (this)->GetNextAvailableSlot (u->GetId (), next->GetId (),timeSlot, option, superframeF_1);
              if (!m_ResourceAvailable)
                {
                  return false;
                }
              slot = resource.timeSlot;
              chIndex = resource.channelIndex;

              // Allocate entries of M
              (this)->m_mainSchedule[slot][chIndex][0] = u->GetId ();
              (this)->m_mainSchedule[slot][chIndex][1] = next->GetId ();
              (this)->m_repLength[slot][chIndex] = superframeF_1;

              // Allocate the slots of S_u and S_i
              Mac16Address graphIDMac = GraphIDConverter (grpahID);
              NodeInfo scheduleInfo = {m_carriers[chIndex], TRANSMIT, graphIDMac, m_panID, source};
              (this)->m_nodeScheduleN[u->GetId ()][slot] = scheduleInfo;
              scheduleInfo.slotType = RECEIVE;
              (this)->m_nodeScheduleN[next->GetId ()][slot] = scheduleInfo;

//              NS_LOG_UNCOND("2: slot: "<<slot<<" TX: "<<to_string(u->GetId())<<" RX: "<<to_string(next->GetId())<<
//                            " Rep Len: "<<superframeF_1<<" graphID "<<graphIDMac);

              // Routing table creation
              RoutingTable rTableRecord = {v->GetId (), graphIDMac, vector<uint32_t> (1,next->GetId ())};
              m_tableList[u->GetId ()][graphIDMac] = rTableRecord;

              if (next != v)
                {
                  // ScheduleLink (i, v, G, F', t_i, o)
                  (this)->ScheduleLinks (next, v, Graph, superframeF_1, slot, option, grpahID, source);
                }

            }
          else
            {
              // identify the earliest slot from t in M with a channel c
              (this)->ResizeSchedule (superframeF_1);
              resource = (this)->GetNextAvailableSlot (u->GetId (), next->GetId (), timeSlot + superframe, option, superframeF_1);
              if (!m_ResourceAvailable)
                {
                  return false;
                }
              slot = resource.timeSlot;
              chIndex = resource.channelIndex;

              // Allocate entries of M
              (this)->m_mainSchedule[slot][chIndex][0] = u->GetId ();
              (this)->m_mainSchedule[slot][chIndex][1] = next->GetId ();
              (this)->m_repLength[slot][chIndex] = superframeF_1;

              // Allocate the slots of S_u and S_i
              if (grpahID > 0xFFFF / 2 && m_grpahID < 0xFFFF / 2)
                {
                  m_grpahID += 0xFFFF / 2;
                }
              grpahID = m_grpahID;
              m_grpahID++;

              Mac16Address graphIDMac = GraphIDConverter (grpahID);
              NodeInfo scheduleInfo = {m_carriers[chIndex], TRANSMIT, graphIDMac, m_panID, source};
              (this)->m_nodeScheduleN[u->GetId ()][slot] = scheduleInfo;
              scheduleInfo.slotType = RECEIVE;
              (this)->m_nodeScheduleN[next->GetId ()][slot] = scheduleInfo;

//              NS_LOG_UNCOND("3: slot: "<<slot<<" TX: "<<to_string(u->GetId())<<" RX: "<<to_string(next->GetId())<<
//                            " Rep Len: "<<superframeF_1<<" graphID "<<graphIDMac);

              // Routing table creation
              RoutingTable rTableRecord = {v->GetId (), graphIDMac, vector<uint32_t> (1,next->GetId ())};
              m_tableList[u->GetId ()][graphIDMac] = rTableRecord;

              if (next != v)
                {
                  // ScheduleLink (i, v, G, F', t_i, o)
                  (this)->ScheduleLinks (next, v, Graph, superframeF_1, slot, option, grpahID, source);
                }

            }
        }
    }

  return true;
}

// Identify the earliest slot from t with a channel c to:
Resource Isa100Helper::GetNextAvailableSlot (uint32_t u, uint32_t v, uint32_t timeSlot, DlLinkType option, uint32_t repLength)
{
  Resource resource;
  uint8_t chIndex = 0;

  NS_LOG_FUNCTION (this);
  uint32_t nSlot = timeSlot;
  uint32_t slotIndex = 0;
  uint32_t frameSize = (this)->m_mainSchedule.size ();
  bool found = false;
  bool tempFound = false;

  switch (option)
    {
    case TRANSMIT:
    case RECEIVE:
    case LPL:
      while (!found)
        {
          uint32_t i = nSlot + slotIndex;
          if ((this)->m_mainSchedule[i][chIndex][0] == 0xFFFF)
            {
              tempFound = true;
              while (i < frameSize)
                {
                  if ((this)->m_mainSchedule[i][chIndex][0] == 0xFFFF)
                    {
                      if ((this)->m_nodeScheduleN[u].count (i) != 0 || (this)->m_nodeScheduleN[v].count (i) != 0)
                        {
                          tempFound = false;
                        }
                    }
                  else
                    {
                      tempFound = false;
                    }

                  if (!tempFound)
                    {
                      break;
                    }

                  i = i + repLength;
                }
            }
          if (!tempFound)
            {
              chIndex++;
              if (chIndex >= m_carriers.size ())
                {
                  chIndex = 0;
                  slotIndex++;
                  if (slotIndex > frameSize)
                    {
                      NS_LOG_UNCOND ("INSUFFICIENT SLOTS");
                      m_ResourceAvailable = false;
                      found = true;
                    }
                }
            }
          else
            {
              found = true;
            }
        }
      nSlot += slotIndex;
      break;

    case SHARED:
      while (!found)
        {
          uint32_t i = nSlot + slotIndex;
          if ((this)->m_mainSchedule[i][chIndex][0] == 0xFFFF)
            {
              tempFound = true;
              while (i < frameSize)
                {
                  if ((this)->m_mainSchedule[i][chIndex][0] == 0xFFFF)
                    {
                      if ((this)->m_nodeScheduleN[u].count (i) != 0 || (this)->m_nodeScheduleN[v].count (i) != 0)
                        {
                          tempFound = false;
                        }
                    }
                  else
                    {
                      tempFound = false;
                    }

                  if (!tempFound)
                    {
                      break;
                    }

                  i = i + repLength;
                }
            }
          if (!tempFound)
            {
              chIndex++;
              if (chIndex >= m_carriers.size ())
                {
                  chIndex = 0;
                  slotIndex++;
                  if (slotIndex > frameSize)
                    {
                      NS_LOG_UNCOND ("INSUFFICIENT SLOTS");
                      m_ResourceAvailable = false;
                      found = true;
                    }
                }
            }
          else
            {
              found = true;
            }
        }
      nSlot += slotIndex;
      break;
    }

  resource.channelIndex = chIndex;
  resource.timeSlot = nSlot;
  return resource;
}

void Isa100Helper::ResizeSchedule (uint32_t superframe)
{
  NS_LOG_FUNCTION (this);
  uint8_t numChannels = m_carriers.size ();
  uint32_t initRepLength = (this)->m_repLength.size ();
  if (superframe > (this)->m_mainSchedule.size ())
    {
      // resize the schedule to support new # of slots (SUPERFRAME)
      (this)->m_mainSchedule.resize (superframe, vector<vector <uint32_t> > (numChannels, vector <uint32_t> (2, 0xFFFF)));
      (this)->m_repLength.resize (superframe,vector<uint32_t> (numChannels));

    }

  if (superframe >= (this)->m_mainSchedule.size ())
    {
      for (uint32_t nSlot = 0; nSlot < initRepLength; nSlot++)
        {
          for (uint32_t chIndex = 0; chIndex < (this)->m_repLength[nSlot].size (); chIndex++)
            {
              if ((this)->m_repLength[nSlot][chIndex] > 0)
                {
                  uint32_t repTime = 1;
                  uint32_t nextSlot = nSlot + repTime * (this)->m_repLength[nSlot][chIndex];
                  while (nextSlot < (this)->m_mainSchedule.size ())
                    {
//
                      (this)->m_mainSchedule[nextSlot][chIndex] = (this)->m_mainSchedule[nSlot][chIndex];

                      // populate the repeating schedules through the new SUPERFRAME
                      uint32_t TxNode = (this)->m_mainSchedule[nSlot][chIndex][0];
                      uint32_t RxNode = (this)->m_mainSchedule[nSlot][chIndex][1];
                      (this)->m_nodeScheduleN[TxNode][nextSlot] = (this)->m_nodeScheduleN[TxNode][nSlot];
                      (this)->m_nodeScheduleN[RxNode][nextSlot] = (this)->m_nodeScheduleN[RxNode][nSlot];
                      (this)->m_repLength[nextSlot][chIndex] = (this)->m_repLength[nSlot][chIndex];

//                        NS_LOG_UNCOND("ReSize: slot: "<<nextSlot<<" TX: "<<to_string(TxNode)<<" RX: "<<to_string(RxNode)<<" Rep Len: "<<(this)->m_repLength[nextSlot][chIndex]);
//
                      repTime++;
                      nextSlot = nSlot + repTime * (this)->m_repLength[nSlot][chIndex];
                    }
                }
            }

        }
    }

}

void Isa100Helper::PrintGraph (Ptr<IsaGraph> Graph)
{
  NS_LOG_FUNCTION (this);
  map<uint32_t, GraphNode> graphMap = Graph->GetGraphNodeMap ();
  for (map<uint32_t, GraphNode>::const_iterator it = graphMap.begin (); it != graphMap.end (); ++it)
    {
      vector<Ptr<Node> > tempNodeList = it->second.m_neighbors;

      while (!tempNodeList.empty ())
        {
          m_graphTrace (it->second.m_head->GetId (),tempNodeList.back ()->GetId ());
          tempNodeList.pop_back ();
        }

    }
}

void Isa100Helper::PrintGraphSchedule ()
{
  NS_LOG_FUNCTION (this);

  int src, dst, carrier;
  double txPwr;

  NS_LOG_DEBUG ("size: " << (this)->m_mainSchedule.size ());
  for (uint32_t j = 0; j < (this)->m_mainSchedule.size (); j++)
    {
      for (uint32_t k = 0; k < (this)->m_mainSchedule[j].size (); k++)
        {
          if ((this)->m_mainSchedule[j][k][0] != 0xFFFF)
            {
              src = (this)->m_mainSchedule[j][k][0];
              dst = (this)->m_mainSchedule[j][k][1];
              carrier = (this)->m_carriers[k];
              txPwr = m_txPwrDbm[src][dst];
              m_scheduleTrace (j,src,dst,carrier, txPwr);
            }
        }
    }

}

SchedulingResult Isa100Helper::ScheduleAndRouteTDMAgraph (OptimizerSelect optSelect)
{
  NS_LOG_FUNCTION (this);
  uint32_t numNodes = m_devices.GetN ();
  SchedulingResult schedulingResult = SCHEDULE_FOUND;

  // schedule trace
  (this)->PrintGraphSchedule ();

  // resize the superframe size to fit the current schedule
  (this)->SetDlAttribute ("SuperFramePeriod",UintegerValue (m_mainSchedule.size ()));

  // Allocating the channels
  map<uint8_t, uint8_t> channelIndexMap; // channel -> channel index
  for (uint8_t i; i < m_carriers.size (); i++)
    {
      channelIndexMap[m_carriers[i]] = i;
    }

  // Following is to get the MAC addresses of nodes
  Ptr<NetDevice> baseDevice;
  Ptr<Isa100NetDevice> netDevice;
  Ptr<NetDevice> baseDevice_next;
  Ptr<Isa100NetDevice> netDevice_next;
  Mac16AddressValue address_next;

  // "tableForRouting" is only used for graph routing - Han's
  // "tableForRouting" is the direct matching variable of "m_table" of "Isa100RoutingAlgorithm"
  // routing tables of each nodes (Node ID -> destID -> graphID sequence)
  map<uint32_t, map<uint32_t, vector<vector<Mac16Address> > > > tableForRouting;

  for (uint32_t nNode = 0; nNode < numNodes; nNode++)
    {
      NodeSchedule nodeSchedule;
      vector<uint8_t> hoppingPattern;

      // "graphTable" is to match with the m_graphTable used in routing; map <destination ID, graph ID list> m_Table
      map<Mac16Address, Mac16Address > graphTable; // graphID -> next graph ID, neighbor

      // Assign schedule to DL
      baseDevice = m_devices.Get (nNode);
      netDevice = baseDevice->GetObject<Isa100NetDevice>();

      for (map<uint32_t, NodeInfo>::const_iterator it = m_nodeScheduleN[nNode].begin ();
           it != m_nodeScheduleN[nNode].end (); ++it)
        {
          // information extraction from NodeInfo
          uint32_t slot = it->first;
          NodeInfo nInfo = it->second;
          DlLinkType slotType = nInfo.slotType;

          nodeSchedule.slotSched.push_back (slot);
          nodeSchedule.slotType.push_back (slotType);
          hoppingPattern.push_back (nInfo.channelSched);

          // Populating the graph table
          if (m_tableList[nNode][nInfo.graphID].neighborList.size () > 0)
            {
              // populate the current graph -> next graph and next node MacAddress mapping
              baseDevice_next = m_devices.Get (m_tableList[nNode][nInfo.graphID].neighborList[0]);
              netDevice_next = baseDevice_next->GetObject<Isa100NetDevice>();
              netDevice_next->GetDl ()->GetAttribute ("Address",address_next);
              graphTable[nInfo.graphID] = address_next.Get ();
            }

          if (nInfo.pathSource == nNode && optSelect == TDMA_GRAPH)   // only applicable for graph routing
            {
              vector<Mac16Address> Path;
              Mac16Address currentGraph = nInfo.graphID;
              Path.push_back (currentGraph);

              RoutingTable rt = m_tableList[nNode][currentGraph];
              uint32_t dst = rt.destID;
              uint32_t nextNode = rt.neighborList[0];

              while (nextNode != dst)
                {
                  NodeInfo nInfoTemp = m_nodeScheduleN[nextNode][++slot];
                  currentGraph = nInfoTemp.graphID;
                  Path.push_back (currentGraph);
                  rt = m_tableList[nextNode][currentGraph];

                  dst = rt.destID;
                  nextNode = rt.neighborList[0];
                }
              tableForRouting[nNode][dst].push_back (Path);
            }
        }

      netDevice->GetDl ()->SetAttribute ("SuperFramePeriod", UintegerValue (m_mainSchedule.size ()));

      if (!baseDevice || !netDevice)
        {
          NS_FATAL_ERROR ("Installing TDMA schedule on non-existent ISA100 net device.");
        }

      Ptr<Isa100RoutingAlgorithm> routingAlgorithm;
      if (optSelect == TDMA_GRAPH)
        {
          routingAlgorithm = CreateObject<Isa100GraphRoutingAlgorithm> (tableForRouting[nNode], graphTable);
        }
      else if (optSelect == TDMA_MIN_LOAD || optSelect == TDMA_CONVEX_MINLOAD)
        {
          tableForRouting[nNode] = m_tableListMinLoad[nNode];
          routingAlgorithm = CreateObject<Isa100MinLoadRoutingAlgorithm> (tableForRouting[nNode], graphTable, m_tableListBackup[nNode]);
        }

//      NS_LOG_UNCOND("Here");
//      for (map<uint32_t, vector<vector<Mac16Address>>>::const_iterator it = tableForRouting[nNode].begin ();
//                 it != tableForRouting[nNode].end (); ++it)
//          {
//            NS_LOG_UNCOND("Here tableForRouting");
//            NS_LOG_UNCOND("tableForRouting ****** node: "<<nNode<<" dst: "<<it->first);
//            for(unsigned int j = 0; j < it->second.size(); j++)
//              {
//                vector<Mac16Address> path = it->second[j];
//                NS_LOG_UNCOND(" panID: "<<j);
//                for(unsigned int k = 0; k < path.size(); k++)
//                  {
//                    NS_LOG_UNCOND(path[k]);
//                  }
//              }
//          }
//
//      NS_LOG_UNCOND("Here graphTable");
//      for (map<Mac16Address, Mac16Address >::const_iterator it = graphTable.begin ();
//                 it != graphTable.end (); ++it)
//          {
//            NS_LOG_UNCOND("node: "<<nNode<<" graph ID "<<it->first<<" neighbor "<<it->second);
//          }
//
//      NS_LOG_UNCOND("Here Backup graphTable");
//      for (map<Mac16Address, vector<Mac16Address> >::const_iterator it = m_tableListBackup[nNode].begin ();
//                 it != m_tableListBackup[nNode].end (); ++it)
//          {
//            NS_LOG_UNCOND("node: "<<nNode<<" graph ID "<<it->first);
//            vector<Mac16Address> backupPath = it->second;
//            for(unsigned int k = 0; k < backupPath.size(); k++)
//              {
//                NS_LOG_UNCOND(backupPath[k]);
//              }
//          }

//      routingAlgorithm->SetGraphTable (graphTable);
      netDevice->GetDl ()->SetRoutingAlgorithm (routingAlgorithm);
      Mac16AddressValue address;
      netDevice->GetDl ()->GetAttribute ("Address",address);
      netDevice->GetDl ()->GetRoutingAlgorithm ()->SetAttribute ("Address",address);

      // Set the tx power levels in DL
      netDevice->GetDl ()->SetTxPowersDbm (m_txPwrDbm[nNode], numNodes);

      // Set the sfSchedule
      Ptr<Isa100DlSfSchedule> schedulePtr = CreateObject<Isa100DlSfSchedule>();

      schedulePtr->SetSchedule (hoppingPattern,nodeSchedule.slotSched,nodeSchedule.slotType);

      netDevice->GetDl ()->SetDlSfSchedule (schedulePtr);
    }

  return schedulingResult;
}

Mac16Address Isa100Helper::GraphIDConverter (uint16_t graphID)
{
  NS_LOG_FUNCTION (this);

  // convert the integer graphID to Hex value
  std::stringstream stream;
  stream << std::hex << static_cast<int> (graphID);
  string graphIDHexString = stream.str ();

  // 2 Byte Hex value is convertible to a maximum of size 4 array
  char * cstr = new char [4];
  std::strcpy (cstr, graphIDHexString.c_str ());

  char gAddr[5] = {'0','0',':','0','0'};

  switch (graphIDHexString.size ())
    {
    case 1:
      gAddr[4] = cstr[0];
      break;
    case 2:
      gAddr[3] = cstr[0];
      gAddr[4] = cstr[1];
      break;
    case 3:
      gAddr[1] = cstr[0];
      gAddr[3] = cstr[1];
      gAddr[4] = cstr[2];
      break;
    case 4:
      gAddr[0] = cstr[0];
      gAddr[1] = cstr[1];
      gAddr[3] = cstr[2];
      gAddr[4] = cstr[3];
      break;
    }
  return Mac16Address (gAddr);
}

} // namespace ns3

