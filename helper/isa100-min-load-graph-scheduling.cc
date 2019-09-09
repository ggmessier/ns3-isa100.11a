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

SchedulingResult Isa100Helper::ConstructDataCommunicationScheduleMinLoad (vector< vector<uint32_t> > UL_Ex, vector<vector< vector<uint32_t> > > UL_Sh,
                                                                          vector< vector<uint32_t> > DL_Ex, vector< vector<uint32_t> > DL_Sh, int frameSize)
{
  NS_LOG_FUNCTION (this);
  SchedulingResult schedulingResult = NO_ROUTE;

  uint8_t numChannels = m_carriers.size ();
  (this)->m_mainSchedule.resize (frameSize, vector<vector <uint32_t> > (numChannels, vector <uint32_t> (2, 65535)));

  bool scheduleFound = true;

  m_grpahID = 3;  // starting node (1 and 2 considered as the APs)
  m_panID = 0;
  scheduleFound = ScheduleLinksMinLoad (UL_Ex, frameSize, 0, (DlLinkType)TRANSMIT, false);
  if (!scheduleFound)
    {
      return INSUFFICIENT_SLOTS; // schedule FAIL
    }

//  m_panID++;
//  scheduleFound = ScheduleLinksMinLoad(UL_Ex, frameSize, 0, (DlLinkType)TRANSMIT, false);
//  if (!scheduleFound)
//    return INSUFFICIENT_SLOTS;  // schedule FAIL

  m_panID++;
  for(uint32_t i = 0; i < UL_Sh.size(); i++)
      {
        scheduleFound = ScheduleLinksMinLoad(UL_Sh[i], frameSize, frameSize/2, (DlLinkType)SHARED, true);
        if (!scheduleFound)
          return INSUFFICIENT_SLOTS;  // schedule FAIL
      }


  // scheduling for DOWNLINK need to add here.
  schedulingResult = SCHEDULE_FOUND;

  return schedulingResult;
}

bool Isa100Helper::ScheduleLinksMinLoad (vector< vector<uint32_t> > flows, int frameSize, uint32_t timeSlot, DlLinkType option, bool isBackup)
{
  NS_LOG_FUNCTION (this);

  // Next available resource
  Resource resource;
  uint32_t slot;
  uint8_t chIndex;

  uint32_t txNode;  // transmitting node
  uint32_t rxNode;  // receiving node
  uint32_t srcNode; // source node (packet generated node)
  uint32_t dstNode; // destination node (intended destination of the packet)

  Mac16Address primaryMacGraph;

  // For LPL
  Ptr<NetDevice> baseDevice;
  Ptr<Isa100NetDevice> netDevice;
  BooleanValue lplEnabled;

  for (uint32_t i = 0; i < flows.size (); i++)
    {
      NS_LOG_DEBUG ("flows: " << i << " " << flows.size ());
      if (flows[i].empty ())
        {
          return true;
        }

      // For the primary path first node and the last node, respectively, are source and destination.
      // if the the path is backup then the respective primary path source node is the first node of backup path
      // Ex: Backup of 5->4->2->0: (5->1->0, 4->1->0, 2->0) -> srcNode is 5, 4 and 2 and dstNode is 0
      srcNode = flows[i][0];
      dstNode = flows[i][flows[i].size () - 1];

      if (isBackup && i == 0)
        {
          // primary path is the last path (panID = 1 when retransmission is exist
          primaryMacGraph = m_tableListMinLoad[srcNode][dstNode][m_panID-1].back ();
        }

      NS_LOG_DEBUG ("srcNode: "<<srcNode<<" dstNode: " << dstNode);

      // convert the grpahID to MAC address format
      Mac16Address macGraphID = GraphIDConverter (m_grpahID);

      for (uint32_t j = 0; j < flows[i].size () - 1; j++)
        {
          txNode = flows[i][j];
          rxNode = flows[i][j + 1];

          NS_LOG_DEBUG ("txNode: " << txNode << " rxNode: " << rxNode);

          // identify the earliest slot from t with a channel c
          resource = (this)->GetNextAvailableSlot (txNode, rxNode,timeSlot, option, frameSize);
          if (!m_ResourceAvailable)
            {
              return false;
            }
          slot = resource.timeSlot;
          chIndex = resource.channelIndex;

          NS_LOG_DEBUG ("slot: " << slot << " chIndex: " << to_string (chIndex));

          // populate the main schedule
          (this)->m_mainSchedule[slot][chIndex][0] = txNode;
          (this)->m_mainSchedule[slot][chIndex][1] = rxNode;

          // [LPL] low-power listening - this is only for single channel transmission
          // for multi-channel transmission, these LPL scheduling need to be done at the optimizer level scheduling
          baseDevice = m_devices.Get (rxNode);
          netDevice = baseDevice->GetObject<Isa100NetDevice>();
          netDevice->GetDl ()->GetAttribute ("LplEnabled",lplEnabled);

          // populate the node schedules
          NodeInfo scheduleInfo = {m_carriers[chIndex], (DlLinkType)TRANSMIT, macGraphID, m_panID, srcNode};
          (this)->m_nodeScheduleN[txNode][slot] = scheduleInfo;
          if (lplEnabled && m_panID > 0)
            {
              scheduleInfo.slotType = (DlLinkType)LPL;
            }
          else
            {
              scheduleInfo.slotType = (DlLinkType)RECEIVE;
            }
          (this)->m_nodeScheduleN[rxNode][slot] = scheduleInfo;

          if (!isBackup) // transmission and retransmission scheduling   m_panID = 0 or 1
            {
              // initial value for the vector
              if (j == 0)
                {
                  // add a one value vector to the graph sequence
                  m_tableListMinLoad[srcNode][dstNode].push_back({macGraphID});
                }
              else
                {
                  m_tableListMinLoad[srcNode][dstNode][m_panID].push_back (macGraphID); // push a graphID to the graph sequence
                }

              NS_LOG_DEBUG ("m_tableListMinLoad " << txNode << " " << dstNode << " " << macGraphID);

              m_tableList[txNode][macGraphID] = {dstNode, macGraphID, vector<uint32_t> (1,rxNode)};
            }
          else if (isBackup)    // backup path scheduling
            {
              m_tableListBackup[srcNode][primaryMacGraph].push_back(macGraphID); // add graphs to the backup graph sequence

              m_tableList[txNode][macGraphID] = {dstNode, macGraphID, vector<uint32_t> (1,rxNode)};
              NS_LOG_DEBUG ("backup graph " << macGraphID<<" primaryMacGraph "<<primaryMacGraph<<" txNode "<<txNode);
            }
        }
      m_grpahID++;
    }

  return true;
}

} // namespace ns3

