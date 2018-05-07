/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 The University Of Calgary- FISHLAB
 *
 * Author:   Rajith Madduma Bandarage <rajith.maddumabandar@ucalgary.ca>
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
	nc2.Create(5);
	nc.Add(nc2);

    Ptr<IsaGraph> G = CreateObject<IsaGraph>(nc);

            G->addEdge(nc.Get(0), nc.Get(1));
            G->addEdge(nc.Get(0), nc.Get(2));
            G->addEdge(nc.Get(1), nc.Get(3));
            G->addEdge(nc.Get(2), nc.Get(4));
            G->addEdge(nc.Get(2), nc.Get(5));
            G->addEdge(nc.Get(5), nc.Get(4));
            G->addEdge(nc.Get(4), nc.Get(7));
            G->addEdge(nc.Get(3), nc.Get(7));
            G->addEdge(nc.Get(3), nc.Get(6));

            // print the adjacency list representation of the above graph

            G->printGraph();
            Ptr<IsaGraph> G_R = G->flipEdge();
            NS_LOG_UNCOND("Flipped");
            G_R->printGraph();

	return 0;
}

