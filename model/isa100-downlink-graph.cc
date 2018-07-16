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

#include "isa100-downlink-graph.h"
#include "ns3/log.h"
#include "ns3/type-id.h"
#include <algorithm>

NS_LOG_COMPONENT_DEFINE ("Isa100DownlinkGraph");

using namespace std;

namespace ns3 {

TypeId Isa100DownlinkGraph::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::Isa100DownlinkGraph")
    .SetParent<Object> ()
    .AddConstructor<Isa100DownlinkGraph> ()
  ;
  return tid;
}

Isa100DownlinkGraph::Isa100DownlinkGraph ()
{
  NS_LOG_FUNCTION (this);
}

Isa100DownlinkGraph::~Isa100DownlinkGraph ()
{
  NS_LOG_FUNCTION (this);
}

} // namespace ns3


