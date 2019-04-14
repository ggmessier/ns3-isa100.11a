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

SchedulingResult Isa100Helper::ConstructDataCommunicationScheduleMinLoad(vector< vector<uint32_t>> UL_Ex, vector< vector<uint32_t>> UL_Sh,
                                                           vector< vector<uint32_t>> DL_Ex, vector< vector<uint32_t>> DL_Sh, int frameSize)
{
  NS_LOG_FUNCTION (this);
//  uint32_t numNodes = m_devices.GetN();
  SchedulingResult schedulingResult = NO_ROUTE;

  uint8_t numChannels = m_carriers.size();
  (this)->m_mainSchedule.resize(frameSize, vector<vector <uint32_t>> (numChannels, vector <uint32_t> (2, 65535)));

  bool scheduleFound = true;

  scheduleFound = ScheduleLinksMinLoad(UL_Ex, frameSize, 0, TRANSMIT);
  if (!scheduleFound)
    return INSUFFICIENT_SLOTS;  // schedule FAIL
  scheduleFound = ScheduleLinksMinLoad(UL_Sh, frameSize, frameSize/4, SHARED);
  if (!scheduleFound)
    return INSUFFICIENT_SLOTS;  // schedule FAIL

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

  for(uint32_t i = 0; i < flows.size(); i++)
    {
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

          (this)->m_nodeScheduleN[txNode][slot] = pair<uint8_t, DlLinkType> (m_carriers[chIndex],TRANSMIT);
          (this)->m_nodeScheduleN[rxNode][slot] = pair<uint8_t, DlLinkType> (m_carriers[chIndex],RECEIVE);

          baseDevice_next = m_devices.Get(rxNode);
          netDevice_next = baseDevice_next->GetObject<Isa100NetDevice>();
          netDevice_next->GetDl()->GetAttribute("Address",address_next);

          if(count(m_tableList[txNode][dstNode].begin(),m_tableList[txNode][dstNode].end(),address_next.Get()) == 0)
            {
              m_tableList[txNode][dstNode].push_back(address_next.Get());   //update the routing tables
              NS_LOG_UNCOND("m_tableList: "<<txNode<<" dstNode: "<<dstNode<<" address_next"<<address_next.Get());
            }
        }
    }

  return true;
}

} // namespace ns3

