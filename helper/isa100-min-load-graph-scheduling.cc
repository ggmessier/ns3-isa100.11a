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

NS_LOG_COMPONENT_DEFINE ("Isa100MinLoadGraphScheduling");

using namespace std;

namespace ns3 {

SchedulingResult Isa100Helper::ConstructDataCommunicationScheduleMinLoad(vector< vector<uint32_t>> UL_Ex, vector<vector< vector<uint32_t>>> UL_Sh,
                                                           vector< vector<uint32_t>> DL_Ex, vector< vector<uint32_t>> DL_Sh, int frameSize)
{
  NS_LOG_FUNCTION (this);
//  uint32_t numNodes = m_devices.GetN();
  SchedulingResult schedulingResult = NO_ROUTE;

  // temporary Rajith to print the schedule
//  NS_LOG_DEBUG("size: "<<(this)->m_mainSchedule.size());
  uint32_t txNode;
  uint32_t rxNode;
  int br = -1;

////  cout<< br <<" "<< br<<" "<<br<<" "<<br<<endl;
//  for(uint32_t i = 0; i < UL_Ex.size(); i++)
//    {
//      for(uint32_t j = 0; j < UL_Ex[i].size() -1; j++)
//        {
//          txNode = UL_Ex[i][j];
//          rxNode = UL_Ex[i][j + 1];
////            cout<< i <<" "<< j<<" "<<txNode<<" "<<rxNode<<endl;
//          NS_LOG_UNCOND(j<<" "<<txNode<<" "<<rxNode<<" "<<0);
////          m_scheduleTrace(j,txNode,rxNode,(this)->m_carriers[0]);
//          m_scheduleTrace(j,txNode,rxNode,0);
//        }
//      for(uint32_t k = 0; k < UL_Sh[i].size()-1; k++)
//        {
////            cout<< br <<" "<< br<<" "<<0<<" "<<0<<endl;
//          NS_LOG_UNCOND(br<<" "<<br<<" "<<0<<" "<<k);
//          m_scheduleTrace(br,br,0,0);
//          for(uint32_t m = 0; m < UL_Sh[i][k].size()-1; m++)
//              {
//                  txNode = UL_Sh[i][k][m];
//                  rxNode = UL_Sh[i][k][m + 1];
////                      cout<< i <<" "<< k <<" "<<txNode<<" "<<rxNode<<endl;
//                  NS_LOG_UNCOND(i<<" "<<txNode<<" "<<rxNode<<" "<<0);
//                  m_scheduleTrace(i,txNode,rxNode,0);
//              }
//        }
//      NS_LOG_UNCOND(br<<" "<<br<<" "<<br<<" "<<0);
//      m_scheduleTrace(br,br,br,0);
////      cout<< br <<" "<< br<<" "<<br<<" "<<0<<endl;
//    }
//    // end of temporary schedule

  uint8_t numChannels = m_carriers.size();
  (this)->m_mainSchedule.resize(frameSize, vector<vector <uint32_t>> (numChannels, vector <uint32_t> (2, 65535)));

  bool scheduleFound = true;

  m_grpahID = 3; m_panID = 0;
  scheduleFound = ScheduleLinksMinLoad(UL_Ex, frameSize, 0, TRANSMIT);
  if (!scheduleFound)
    return INSUFFICIENT_SLOTS;  // schedule FAIL
//  m_panID++;
//  scheduleFound = ScheduleLinksMinLoad(UL_Ex, frameSize, 0, TRANSMIT);
//  if (!scheduleFound)
//    return INSUFFICIENT_SLOTS;  // schedule FAIL
//  m_panID++;
//  for(uint32_t i = 0; i < UL_Sh.size(); i++)
//      {
//        scheduleFound = ScheduleLinksMinLoad(UL_Sh[i], frameSize, frameSize/2, SHARED);
//        if (!scheduleFound)
//          return INSUFFICIENT_SLOTS;  // schedule FAIL
//      }


  // scheduling for DOWNLINK need to add here.

  schedulingResult = SCHEDULE_FOUND;

  return schedulingResult;
}

bool Isa100Helper::ScheduleLinksMinLoad(vector< vector<uint32_t>> flows, int frameSize, uint32_t timeSlot, DlLinkType option)
{
  NS_LOG_FUNCTION (this);

  // Next available resource
  Resource resource;
  uint32_t slot;
  uint8_t chIndex;

  uint32_t txNode;
  uint32_t rxNode;
  uint32_t dstNode;

  Ptr<NetDevice> baseDevice_next;
  Ptr<Isa100NetDevice> netDevice_next;
  Mac16AddressValue address_next;

  NS_LOG_UNCOND("ScheduleLinksMinLoad ******************");
//
//  for(uint32_t i = 0; i < flows.size(); i++)
//      {
//        dstNode = flows[i][flows[i].size()-1];
//        //      NS_LOG_UNCOND("dstNode: "<<dstNode);
//        txNode = flows[i][0];
//        rxNode = flows[i][1];
//
//        baseDevice_next = m_devices.Get(rxNode);
//        netDevice_next = baseDevice_next->GetObject<Isa100NetDevice>();
//        netDevice_next->GetDl()->GetAttribute("Address",address_next);
//
//        m_tableList[txNode][dstNode].push_back(address_next.Get());   //update the routing tables
//
//      }

  for(uint32_t i = 0; i < flows.size(); i++)
    {
//      vector<uint32_t> tempFlow = flows[i];
      NS_LOG_UNCOND("flows: "<<i<<" "<<flows.size());
      if (flows[i].empty())
        return true;
      dstNode = flows[i][flows[i].size()-1];
      NS_LOG_UNCOND("dstNode: "<<dstNode);
      for(uint32_t j = 0; j < flows[i].size() - 1; j++)
        {
          txNode = flows[i][j];
          rxNode = flows[i][j + 1];

          NS_LOG_UNCOND("txNode: "<<txNode<<" rxNode: "<<rxNode);
          // identify the earliest slot from t with a channel c
          resource = (this)->GetNextAvailableSlot(txNode, rxNode ,timeSlot, option, frameSize);
          if(!m_ResourceAvailable)
            return false;
          slot = resource.timeSlot;
          chIndex = resource.channelIndex;

          NS_LOG_UNCOND("slot: "<<slot<<" chIndex: "<<to_string(chIndex));

          (this)->m_mainSchedule[slot][chIndex][0] = txNode;
          (this)->m_mainSchedule[slot][chIndex][1] = rxNode;

//          uint16_t grpahID = 0;       // TEMPORARY ***************************************
          Mac16Address macGraphID = GraphIDConverter(m_grpahID);
          NodeInfo scheduleInfo = {m_carriers[chIndex], TRANSMIT, macGraphID};
          (this)->m_nodeScheduleN[txNode][slot] = scheduleInfo;
          scheduleInfo.slotType = RECEIVE;
          (this)->m_nodeScheduleN[rxNode][slot] = scheduleInfo;

          baseDevice_next = m_devices.Get(rxNode);
          netDevice_next = baseDevice_next->GetObject<Isa100NetDevice>();
          netDevice_next->GetDl()->GetAttribute("Address",address_next);

          ///< routing tables of each nodes (Node ID -> destID -> graphID sequence)
          RoutingTable rt;
          if (option == TRANSMIT)
            {
              NS_LOG_UNCOND("TRANSMIT next addr"<<address_next.Get());
              uint32_t initNodePrimaryPath = flows[i][0];
//              if (m_panID < m_tableList2[txNode][dstNode].size() && j == 0)
              if (m_panID < m_tableList2[txNode][dstNode].size() && j == 0)
                m_tableList2[txNode][dstNode][m_panID].push_back(macGraphID);
              else if (j == 0)
                m_tableList2[txNode][dstNode].push_back({macGraphID});
              else
                m_tableList2[initNodePrimaryPath][dstNode][m_panID].push_back(macGraphID);

              if (m_panID > 0)
                {
                  Mac16Address primaryMacGraph = m_tableList2[initNodePrimaryPath][dstNode][0].back();
                  m_tableList[txNode][primaryMacGraph].nextGraphID = macGraphID;
                }

              NS_LOG_UNCOND("m_tableList2 "<<txNode<<" "<<dstNode<<" "<<macGraphID);
              rt.destID = dstNode;
              rt.neighborList.push_back(address_next.Get());
              m_tableList[txNode][macGraphID] = rt;
            }
          else if (option == SHARED)
            {
              uint32_t initNodePrimaryPath = flows[0][0];
              Mac16Address primaryMacGraph = m_tableList2[initNodePrimaryPath][dstNode][m_panID].back();
              m_tableList[txNode][primaryMacGraph].nextGraphID = macGraphID;
              NS_LOG_UNCOND("SHARED next addr"<<address_next.Get());
              NS_LOG_UNCOND("backup graph "<<macGraphID);
              m_tableList[txNode][macGraphID].destID = dstNode;
              m_tableList[txNode][macGraphID].nextGraphID = macGraphID;
              m_tableList[txNode][macGraphID].neighborList.push_back(address_next.Get());
            }

          NS_LOG_UNCOND("Here::");
//          map<uint32_t, map<uint32_t, vector<vector<Mac16Address>>>> m_tableList2;
//          if(count(m_tableList[txNode][dstNode].begin(),m_tableList[txNode][dstNode].end(),address_next.Get()) == 0)
//            {
//              m_tableList[txNode][dstNode].push_back(address_next.Get());   //update the routing tables
////              NS_LOG_UNCOND("m_tableList: "<<txNode<<" dstNode: "<<dstNode<<" address_next"<<address_next.Get());
//            }
        }
      m_grpahID++;
    }

  return true;
}

} // namespace ns3

