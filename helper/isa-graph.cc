/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 The University Of Calgary- FISHLAB
 *
 * Author:   Rajith Madduma Bandarage <rajith.maddumabandar@ucalgary.ca>
 */

#include "isa-graph.h"
#include "ns3/log.h"
#include "ns3/type-id.h"
#include <algorithm>

NS_LOG_COMPONENT_DEFINE ("IsaGraph");

using namespace std;

namespace ns3 {

		TypeId IsaGraph::GetTypeId (void)
		{
		  static TypeId tid = TypeId ("ns3::IsaGraph")
			.SetParent<Object> ()
			.AddConstructor<IsaGraph> ()
		  ;
		  return tid;
		}

		IsaGraph::IsaGraph()
		{
			NS_LOG_FUNCTION (this);
		}

		IsaGraph::IsaGraph(NodeContainer c)
		{
			NS_LOG_FUNCTION (this);
	        for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
	        {
			   {
				   uint32_t nodeId = i->operator ->()->GetId();
				   (this)->graphNodeMap[nodeId].head = i->operator ->();
				   (this)->graphNodeMap[nodeId].avg_hop_count = 0;
			   }
	         }
		}

		IsaGraph::~IsaGraph ()
		{
			NS_LOG_FUNCTION (this);
		}

		vector<Ptr<Node>> IsaGraph::GetEdges (Ptr<Node> src)
		{
			NS_LOG_FUNCTION (this);
			return (this)->graphNodeMap[src->GetId()].neighbors;
		}

        //Adding Edge to Graph
        void IsaGraph::AddEdge(Ptr<Node> src, Ptr<Node> dest)
        {
        	NS_LOG_FUNCTION (this);
        	(this)->graphNodeMap[src->GetId()].neighbors.push_back(dest);
        	(this)->graphNodeMap[dest->GetId()].parents.push_back(src);
        }

        //Adding Node to Graph
        void IsaGraph::AddGraphNode(Ptr<Node> src)
        {
        	NS_LOG_FUNCTION (this);
        	uint32_t nodeId = src->GetId();
		    (this)->graphNodeMap[nodeId].head = src;
		    (this)->graphNodeMap[nodeId].avg_hop_count = 0;
        }

        uint32_t IsaGraph::GetNumofNodes (void)
		{
        	NS_LOG_FUNCTION (this);
        	return (this)->graphNodeMap.size();
		}

        Ptr<Node> IsaGraph::GetGraphSrcNode (uint32_t id)
        {
        	NS_LOG_FUNCTION (this);
        	return (this)->graphNodeMap[id].head;
        }

        GraphNode IsaGraph::GetGraphNode (uint32_t id)
        {
        	NS_LOG_FUNCTION (this);
        	return (this)->graphNodeMap[id];
        }

         //Print the graph
        void IsaGraph::PrintGraph()
        {
        	NS_LOG_FUNCTION (this);
        	for(map<uint32_t, GraphNode>::const_iterator it = (this)->graphNodeMap.begin();
        			it != (this)->graphNodeMap.end(); ++it)
        	{
                 NS_LOG_UNCOND("\n Adjacency list of Vertex "<<it->second.head->GetId()<<"\n head ");
                 vector<Ptr<Node>> tempNodeList = it->second.neighbors;

                 while (!tempNodeList.empty())
                 {
					 NS_LOG_UNCOND("-> "<<tempNodeList.back()->GetId());
					 tempNodeList.pop_back();
                 }
                 NS_LOG_UNCOND("\n");
           }
        }

        Ptr<IsaGraph> IsaGraph::FlipEdge()
        {
        	NS_LOG_FUNCTION (this);
        	Ptr<IsaGraph> G = CreateObject<IsaGraph>();
        	G = (this);

        	for(map<uint32_t, GraphNode>::const_iterator it = (this)->graphNodeMap.begin();
        			it != (this)->graphNodeMap.end(); ++it)
        	{
        		 vector<Ptr<Node>> tempEdges = (it->second).parents;
        		 G->graphNodeMap[it->first].parents = it->second.neighbors;
        		 G->graphNodeMap[it->first].neighbors = tempEdges;
            }

        	return G;
         }

        void IsaGraph::GraphFlows (void)
        {
            NS_LOG_FUNCTION (this);
            uint32_t nNodes = (this)->GetNumofNodes();

            map <uint32_t, bool> visited;

            int *path = new int[nNodes];
            int path_index = 0;

        	for(map<uint32_t, GraphNode>::const_iterator it = (this)->graphNodeMap.begin();
        			it != (this)->graphNodeMap.end(); ++it)
        	{
        		visited[it->first] = false;

        	}

        	for(map<uint32_t, GraphNode>::const_iterator it = (this)->graphNodeMap.begin();
        			it != (this)->graphNodeMap.end(); ++it)
        	{
        		NS_LOG_UNCOND("GW to Node "<< it->first <<"\n");
        		BreadthFirstSearchFlows(0, it->first, visited, path, path_index); //here assumed id is 0 for gateway

        	}

        }

        void IsaGraph::BreadthFirstSearchFlows(int parent, int dest, map <uint32_t, bool> visited, int path[], int &path_index)
        {
        	NS_LOG_FUNCTION (this);
            visited[parent] = true;
            path[path_index] = parent;
            path_index++;

            if (parent == dest)
            {
                for (int i = 0; i<path_index-1; i++)
                    NS_LOG_UNCOND("Path "<<path[i] << " ");
                NS_LOG_UNCOND("\n");
            }
            else
            {
                vector<Ptr<Node>> tempNodeList = (this)->graphNodeMap[parent].neighbors;
                while (!tempNodeList.empty())
                {
                	uint32_t nextNode = tempNodeList.back()->GetId();
               	 	BreadthFirstSearchFlows(nextNode, dest, visited, path, path_index);
               	 	tempNodeList.pop_back();
                }
            }

            path_index--;
            visited[parent] = false;
        }

        void IsaGraph::SetHopCount(uint32_t id, double hopCount)
        {
        	NS_LOG_FUNCTION (this);
        	(this)->graphNodeMap[id].avg_hop_count = hopCount;
        }

        bool IsaGraph::ReliableBroadcastGraph(Ptr<IsaGraph> G)
        {
        	NS_LOG_FUNCTION (this);
        	(this)->EdgesForS = (this)->graphNodeMap;
        	return (this)->BroadcastGraph(G);
        }

        bool IsaGraph::BroadcastGraph(Ptr<IsaGraph> G)
		{
			NS_LOG_FUNCTION (this);

			bool S_2 = false; //vector for nodes have at least two edges from VB
			bool S_1 = false; //vector for nodes have one edge from VB

			GraphNode node_min_hop;
			GraphNode nodeMaxoutgoingEdges = (this)->graphNodeMap[0]; //gateway node ID = 0
			uint32_t maxOutgoingEdges = 0;

			(this)->UpdateSVector(G);

			for(map<uint32_t, GraphNode>::const_iterator it = EdgesForS.begin();
					it != EdgesForS.end(); ++it)
			{
				if((this)->graphNodeMap.count(it->first) != 1){ //considering on;y the V - VB nodes
					double hop_count;
					vector<GraphNode> tempParents;

					for(uint32_t i = 0; i< (it->second).parents.size(); ++i){
						uint32_t nextNode = (it->second).parents[i].operator ->()->GetId();
						if((this)->graphNodeMap.count(nextNode)){
							tempParents.push_back((this)->graphNodeMap[nextNode]);
						}
					}

					if(tempParents.size() >= 2){
						sort(tempParents.begin(), tempParents.end(),Compare_Average_Hop);
						hop_count = 0.5*((this)->graphNodeMap[tempParents[0].head->GetId()].avg_hop_count
												+(this)->graphNodeMap[tempParents[1].head->GetId()].avg_hop_count)+1;
						EdgesForS[(it->first)].parents[0] = tempParents[0].head;
						EdgesForS[(it->first)].parents[1] = tempParents[1].head;
						G->graphNodeMap[it->first].avg_hop_count = hop_count;
						if(!S_2 || (node_min_hop.avg_hop_count > hop_count)){
							node_min_hop = G->graphNodeMap[it->first];
							S_2 = true;
						}
					}else if(tempParents.size() == 1 && !S_2){
						uint32_t outgoingEdges = 0;
						hop_count = (this)->graphNodeMap[(it->second).parents[0].operator ->()->GetId()].avg_hop_count+1;
						G->graphNodeMap[it->first].avg_hop_count = hop_count;
						vector<Ptr<Node>> tempNeighbours = G->graphNodeMap[it->first].neighbors;
						for(uint32_t i = 0; i < tempNeighbours.size();i++){
							if((this)->graphNodeMap.count(tempNeighbours[i]->GetId()) != 1){
									outgoingEdges = outgoingEdges+1;
							}
						}
						if(maxOutgoingEdges<=outgoingEdges){
							if(nodeMaxoutgoingEdges.head->GetId() == 0){ //gateway node ID = 0
									maxOutgoingEdges = outgoingEdges;
									nodeMaxoutgoingEdges = G->graphNodeMap[it->first];
							}else if(nodeMaxoutgoingEdges.avg_hop_count<hop_count){
								maxOutgoingEdges = outgoingEdges;
								nodeMaxoutgoingEdges = G->graphNodeMap[it->first];
							}
						}
						S_1 = true;
					}
				}
		    }

			if(S_2){
				NS_LOG_UNCOND("S_2 Node:"<<node_min_hop.head->GetId());
				(this)->AddGraphNode(node_min_hop.head);
				(this)->SetHopCount(node_min_hop.head->GetId(), node_min_hop.avg_hop_count);
				NS_LOG_UNCOND("Avg Hop Count:"<<node_min_hop.avg_hop_count);
				GraphNode tempNode = EdgesForS[node_min_hop.head->GetId()];
				NS_LOG_UNCOND("Edge 1 :"<<tempNode.parents[0].operator ->()->GetId());
				NS_LOG_UNCOND("Edge 2 :"<<tempNode.parents[1].operator ->()->GetId());
				(this)->AddEdge(tempNode.parents[0], node_min_hop.head);
				(this)->AddEdge(tempNode.parents[1], node_min_hop.head);
			}else if(S_1){
				NS_LOG_UNCOND("S_1 Node:"<<nodeMaxoutgoingEdges.head->GetId());
				(this)->AddGraphNode(nodeMaxoutgoingEdges.head);
				(this)->SetHopCount(nodeMaxoutgoingEdges.head->GetId(), nodeMaxoutgoingEdges.avg_hop_count);
				NS_LOG_UNCOND("Avg Hop Count:"<<nodeMaxoutgoingEdges.avg_hop_count);
				GraphNode tempNode = EdgesForS[nodeMaxoutgoingEdges.head->GetId()];
				NS_LOG_UNCOND("Edge only 1 :"<<tempNode.parents[0].operator ->()->GetId());
				(this)->AddEdge(tempNode.parents[0], nodeMaxoutgoingEdges.head);
			}

			if((this)->GetNumofNodes()<G->GetNumofNodes()){
				(this)->BroadcastGraph(G);
			}

			return (S_1 || S_2);
		 }

         void IsaGraph::UpdateSVector(Ptr<IsaGraph> G)
         {
        	 NS_LOG_FUNCTION (this);
        	 map <uint32_t, GraphNode> tempEdgesofS = (this)->EdgesForS;

        	 for(map<uint32_t, GraphNode>::const_iterator it = (this)->EdgesForS.begin();
        				        			it != (this)->EdgesForS.end(); ++it)
			 {
				 bool newNode = false;
				 vector<Ptr<Node>> tempNodeList = G->graphNodeMap[it->first].neighbors;
				 tempEdgesofS[it->first].neighbors = tempNodeList;
				 tempEdgesofS[it->first].parents = G->graphNodeMap[it->first].parents;

				 while (!tempNodeList.empty())
				 {
					 uint32_t nextNode = tempNodeList.back()->GetId();
					 if((this)->graphNodeMap.count(nextNode) != 1){
						 tempEdgesofS[nextNode]=G->graphNodeMap[nextNode];
						 newNode = true;
					 }
					 tempNodeList.pop_back();
				 }

				 if(!newNode)
					 tempEdgesofS.erase(it->first);
			 }

        	 (this)->EdgesForS = tempEdgesofS;

         }

 };
