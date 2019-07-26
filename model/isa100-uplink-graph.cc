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

#include "isa-graph.h"
#include "ns3/log.h"
#include "ns3/type-id.h"
#include <algorithm>

NS_LOG_COMPONENT_DEFINE ("Isa100UplinkGraph");

using namespace std;

namespace ns3 {

bool IsaGraph::ReliableUplinkGraph (Ptr<IsaGraph> G)
{
  NS_LOG_FUNCTION (this);

  // Initial graph with gateway and APs edges reversed.
  (this)->FlipEdge ();

  // G graph reversed.
  Ptr<IsaGraph> G_reverse = G->FlipEdge ();

  // Generate reliable broadcast graph from uplink flipped graph
  (this)->ReliableBroadcastGraph (G_reverse);

  (this)->SetGraphId (0);  // destination is the gateway (node 0)

  if (G->GetNumofNodes () == (this)->GetNumofNodes ())
    {
      // Flip edges of created reliable broadcast graph
      (this)->FlipEdge ();
    }
  else
    {
      // Uplink graph is disconnected.
      return false;
    }
  // Reliable uplink graph creation succeeded.
  return true;
}

} // namespace ns3


