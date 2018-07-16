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
//#include "ns3/isa-graph.h"

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

  G->AddGateway(0);
  G->AddAccessPoint(1);
  G->AddAccessPoint(2);

	G->AddEdge(0,1);
	G->AddEdge(0,2);
	G->AddEdge(1,2);
	G->AddEdge(1,3);
	G->AddEdge(1,4);
	G->AddEdge(2,3);
	G->AddEdge(2,4);
	G->AddEdge(2,5);
	G->AddEdge(3,6);
	G->AddEdge(3,4);
	G->AddEdge(4,7);
	G->AddEdge(4,8);
	G->AddEdge(5,4);
	G->AddEdge(5,7);
	G->AddEdge(5,9);
	G->AddEdge(6,8);
	G->AddEdge(7,8);
	G->AddEdge(7,9);
	G->AddEdge(8,10);
	G->AddEdge(9,10);

	// for biderectional connectivity
  G->AddEdge(1,0);
  G->AddEdge(2,0);
  G->AddEdge(2,1);
  G->AddEdge(3,1);
  G->AddEdge(4,1);
  G->AddEdge(3,2);
  G->AddEdge(4,2);
  G->AddEdge(5,2);
  G->AddEdge(6,3);
  G->AddEdge(4,3);
  G->AddEdge(7,4);
  G->AddEdge(8,4);
  G->AddEdge(4,5);
  G->AddEdge(7,5);
  G->AddEdge(9,5);
  G->AddEdge(8,6);
  G->AddEdge(8,7);
  G->AddEdge(9,7);
  G->AddEdge(10,8);
  G->AddEdge(10,9);

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

  G_B->AddGateway(0);
  G_B->AddAccessPoint(1);
  G_B->AddAccessPoint(2);

	Ptr<IsaGraph> G_U = CreateObject<IsaGraph>(nc_GU);      ///< Uplink graph creation

  G_U->AddGateway(0);
  G_U->AddAccessPoint(1);
  G_U->AddAccessPoint(2);

	G_B->AddEdge(0,1);
	G_B->AddEdge(0,2);
	G_B->SetHopCount(0,0);
	G_B->SetHopCount(1,1);
	G_B->SetHopCount(2,1);

	G_U->AddEdge(1, 0);
	G_U->AddEdge(2, 0);
	G_U->SetHopCount(0,0);
	G_U->SetHopCount(1,1);
	G_U->SetHopCount(2,1);

	bool reliableGraphB = G_B->ReliableBroadcastGraph(G);
	NS_LOG_UNCOND("**********************************");
	bool reliableGraphU = G_U->ReliableUplinkGraph(G);
	NS_LOG_UNCOND("**********************************");

	map <uint32_t, Ptr<IsaGraph>> downlinkGraphs = G->ReliableDownlinkGraphs(G);

	NS_LOG_UNCOND("****************** Reliable Broadcast Graph ******************");
	if(reliableGraphB)
	  {
	    G_B->PrintGraph();
	  }
	else
	  {
	    NS_LOG_UNCOND("Reliable Broadcast Graph cannot be created. Graph is disconnected.");
	  }
  NS_LOG_UNCOND("****************** Reliable Uplink Graph ******************");
  if(reliableGraphU)
    {
      G_U->PrintGraph();
    }
  else
    {
      NS_LOG_UNCOND("Reliable Uplink Graph cannot be created. Graph is disconnected.");
    }

  NS_LOG_UNCOND("**********************************===================*************************************");

  for (map<uint32_t, Ptr<IsaGraph>>::const_iterator it = downlinkGraphs.begin ();
         it != downlinkGraphs.end (); ++it)
    {
      NS_LOG_UNCOND(" >>>>>>>>>>>>>>>>>>>>>>>>>> Reliable Downlink Graph of Node " <<it->first);
      it->second->PrintGraph();
    }

	return 0;
}

