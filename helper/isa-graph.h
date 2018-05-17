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
#include <algorithm>

using namespace std;

namespace ns3 {

typedef struct{

	Ptr<Node> head;
	double avg_hop_count;
	vector <Ptr<Node>> neighbors;
	vector <Ptr<Node>> parents;

}GraphNode;

bool Compare_Average_Hop( const GraphNode & e1, const GraphNode & e2) {
  if( e1.avg_hop_count != e2.avg_hop_count)
    return (e1.avg_hop_count < e2.avg_hop_count);
  return ((e1.head < e2.head) && (e1.neighbors < e2.neighbors) && (e1.parents < e2.parents));
}

class IsaGraph;

	class IsaGraph : public Node
	{
		private:

			map <uint32_t, GraphNode> graphNodeMap;

		public:

		    static TypeId GetTypeId (void);

		    IsaGraph();
			IsaGraph (NodeContainer c);
			virtual ~IsaGraph ();

			vector<Ptr<Node>> GetEdges (Ptr<Node> src);

			Ptr<Node> GetGraphSrcNode (uint32_t id);

			GraphNode GetGraphNode (uint32_t id);

			uint32_t GetNumofNodes (void);

			void AddEdge(Ptr<Node> src, Ptr<Node> dest);

			void AddGraphNode(Ptr<Node> src);

			void PrintGraph();

			Ptr<IsaGraph> FlipEdge();

			void GraphFlows (void);

			void BreadthFirstSearchFlows(int parent, int dest, map <uint32_t, bool> visited, int path[], int &path_index);

			void SetHopCount(uint32_t id, double hopCount);

			bool ReliableBroadcastGraph(Ptr<IsaGraph> G);

			bool BroadcastGraph(Ptr<IsaGraph> G);

			void UpdateSVector(Ptr<IsaGraph> G);

//			void ClearSVector();

		protected:

			map <uint32_t, GraphNode> EdgesForS;

	};

} // namespace ns3

#endif /* ISA100_11A_HELPER_H */
