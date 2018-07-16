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

//bool CompareFrameRate ( map <uint32_t, int8_t> e1,  map <uint32_t, int8_t> e2)
//{
//  return (e1. < e2.m_powerOfrate);
//};

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
//  (this)->m_supperFrame = -2;
}
//
//Isa100GraphScheduling::Isa100GraphScheduling (Ptr<IsaGraph> G)
//{
//  NS_LOG_FUNCTION (this);
//  (this)->m_supperFrame = -2;
//  map <uint32_t, GraphNode> graphNodeMap = G->GetGraphNodeMap();
//
//  // powerOfrateMap is a map with power of sample rate (n of 2^n) to nodes (node Ids) with that sample rate
//  map <uint32_t, vector<uint32_t>> powerOfrateMap;
//
//  for (map<uint32_t, GraphNode>::const_iterator it = graphNodeMap.begin ();
//        it != graphNodeMap.end (); ++it)
//    {
//      int8_t tempPowerofrate = it->second.m_powerOfrate;
//      powerOfrateMap[tempPowerofrate].push_back(it->first);
//    }
//}

Isa100GraphScheduling::~Isa100GraphScheduling ()
{
  NS_LOG_FUNCTION (this);
}

} // namespace ns3

