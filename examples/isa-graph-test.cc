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

#include "ns3/core-module.h"
#include <iomanip>
#include <iostream>

#include "ns3/isa100-11a-module.h"

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE ("isa-graph-main");

// ************************************************ MAIN BEGIN ************************************************
int main (int argc, char *argv[])
{
	Ptr<Node> gateWay = new Node(0);
	Ptr<Node> acessPoint_1 = new Node(1);
	Ptr<Node> acessPoint_2 = new Node(2);
	NodeContainer nc;
	nc.Add(gateWay);
	nc.Add(acessPoint_1);
	nc.Add(acessPoint_2);

	NodeContainer nc2;
	nc2.Create(8);
	nc.Add(nc2);

  Ptr<IsaGraph> G = CreateObject<IsaGraph>(nc);

  G->AddAccessPoint(1);
  G->AddAccessPoint(2);

	G->AddEdge(nc.Get(0), nc.Get(1));
	G->AddEdge(nc.Get(0), nc.Get(2));
	G->AddEdge(nc.Get(1), nc.Get(3));
	G->AddEdge(nc.Get(1), nc.Get(4));
	G->AddEdge(nc.Get(2), nc.Get(3));
	G->AddEdge(nc.Get(2), nc.Get(4));
	G->AddEdge(nc.Get(2), nc.Get(5));
	G->AddEdge(nc.Get(3), nc.Get(6));
	G->AddEdge(nc.Get(3), nc.Get(4));
	G->AddEdge(nc.Get(4), nc.Get(7));
	G->AddEdge(nc.Get(4), nc.Get(8));
	G->AddEdge(nc.Get(5), nc.Get(4));
	G->AddEdge(nc.Get(5), nc.Get(7));
	G->AddEdge(nc.Get(5), nc.Get(9));
	G->AddEdge(nc.Get(6), nc.Get(8));
	G->AddEdge(nc.Get(7), nc.Get(8));
	G->AddEdge(nc.Get(7), nc.Get(9));
	G->AddEdge(nc.Get(8), nc.Get(10));
	G->AddEdge(nc.Get(9), nc.Get(10));

	// for biderectional connectivity
  G->AddEdge(nc.Get(1), nc.Get(0));
  G->AddEdge(nc.Get(2), nc.Get(0));
  G->AddEdge(nc.Get(3), nc.Get(1));
  G->AddEdge(nc.Get(4), nc.Get(1));
  G->AddEdge(nc.Get(3), nc.Get(2));
  G->AddEdge(nc.Get(4), nc.Get(2));
  G->AddEdge(nc.Get(5), nc.Get(2));
  G->AddEdge(nc.Get(6), nc.Get(3));
  G->AddEdge(nc.Get(4), nc.Get(3));
  G->AddEdge(nc.Get(7), nc.Get(4));
  G->AddEdge(nc.Get(8), nc.Get(4));
  G->AddEdge(nc.Get(4), nc.Get(5));
  G->AddEdge(nc.Get(7), nc.Get(5));
  G->AddEdge(nc.Get(9), nc.Get(5));
  G->AddEdge(nc.Get(8), nc.Get(6));
  G->AddEdge(nc.Get(8), nc.Get(7));
  G->AddEdge(nc.Get(9), nc.Get(7));
  G->AddEdge(nc.Get(10), nc.Get(8));
  G->AddEdge(nc.Get(10), nc.Get(9));

	// print the adjacency list representation of the above graph
	G->PrintGraph();
//	Ptr<IsaGraph> G_R = G->FlipEdge();
//	NS_LOG_UNCOND("Flipped");
//	G_R->PrintGraph();
//	G->GraphFlows();

	//Initialize the reliable broadcast graph
	NodeContainer nc_GB;
	nc_GB.Add(gateWay);
	nc_GB.Add(acessPoint_1);
	nc_GB.Add(acessPoint_2);

	//Initialize the reliable uplink graph
	NodeContainer nc_GU;
	nc_GU.Add(gateWay);
  nc_GU.Add(acessPoint_1);
  nc_GU.Add(acessPoint_2);

	Ptr<IsaGraph> G_B = CreateObject<IsaGraph>(nc_GB);      ///< Broadcast graph creation
	Ptr<IsaGraph> G_U = CreateObject<IsaGraph>(nc_GU);      ///< Uplink graph creation

	G_B->AddEdge(nc_GB.Get(0), nc_GB.Get(1));
	G_B->AddEdge(nc_GB.Get(0), nc_GB.Get(2));
	G_B->SetHopCount(0,0);
	G_B->SetHopCount(1,1);
	G_B->SetHopCount(2,1);

	G_U->AddEdge(nc_GU.Get(1), nc_GU.Get(0));
	G_U->AddEdge(nc_GU.Get(2), nc_GU.Get(0));
	G_U->SetHopCount(0,0);
	G_U->SetHopCount(1,1);
	G_U->SetHopCount(2,1);

	bool reliableGraphB = G_B->ReliableBroadcastGraph(G);
	NS_LOG_UNCOND("**********************************");
	bool reliableGraphU = G_U->ReliableUplinkGraph(G);

	NS_LOG_UNCOND("Reliable Broadcast Graph");
	if(reliableGraphB)
	  {
	    G_B->PrintGraph();
	  }
	else
	  {
	    NS_LOG_UNCOND("Reliable Broadcast Graph cannot be created. Grpah is disconnected.");
	  }
  NS_LOG_UNCOND("Reliable Uplink Graph");
  if(reliableGraphU)
    {
      G_U->PrintGraph();
    }
  else
    {
      NS_LOG_UNCOND("Reliable Uplink Graph cannot be created. Grpah is disconnected.");
    }

	return 0;
}

