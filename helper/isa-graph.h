/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 The University Of Calgary- FISHLAB
 *
 * Author:   Rajith Madduma Bandarage <rajith.maddumabandar@ucalgary.ca>
 */

#ifndef ISA_GRAPH_H
#define ISA_GRAPH_H

#include "ns3/core-module.h"
#include "ns3/isa100-11a-module.h"

using namespace std;

namespace ns3 {

class IsaGraph;

	class IsaGraph : public Node
	{
		private:

			vector<vector<Ptr<Node>>> Edges;

		public:

		    static TypeId GetTypeId (void);

		    IsaGraph();
			IsaGraph (NodeContainer c);
			virtual ~IsaGraph ();

			vector<Ptr<Node>> GetEdges (Ptr<Node> src);

			Ptr<Node> GetGraphSrcNode (uint32_t src);

			uint32_t getNumofNodes (void);

			void addEdge(Ptr<Node> src, Ptr<Node> dest);

			void addGraphNode(Ptr<Node> src);

			void printGraph();

			Ptr<IsaGraph> flipEdge();

			void GraphFlows (void);

			void BreadthFirstSearchFlows(int parent, int dest, vector<bool> visited, int path[], int &path_index);

	};

} // namespace ns3

#endif /* ISA100_11A_HELPER_H */
