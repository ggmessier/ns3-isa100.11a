/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 The University Of Calgary- FISHLAB
 *
 * Author:   Rajith Madduma Bandarage <rajith.maddumabandar@ucalgary.ca>
 */

#include "isa-graph.h"
#include "ns3/log.h"
#include "ns3/type-id.h"

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
				   vector<Ptr<Node>> tempNodeList;
				   tempNodeList.push_back(i->operator ->());
				   (this)->Edges.push_back(tempNodeList);

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
			return (this)->Edges[src->GetId()];
		}

        //Adding Edge to Graph
        void IsaGraph::addEdge(Ptr<Node> src, Ptr<Node> dest)
        {
        	NS_LOG_FUNCTION (this);
        	(this)->Edges[src->GetId()].push_back(dest);
        }

        //Adding Node to Graph
        void IsaGraph::addGraphNode(Ptr<Node> src)
        {
        	NS_LOG_FUNCTION (this);
        	vector<Ptr<Node>> tempNodeList;
        	tempNodeList.push_back(src);
        	(this)->Edges.push_back(tempNodeList);
        }

        uint32_t IsaGraph::getNumofNodes (void)
		{
        	NS_LOG_FUNCTION (this);
        	return (this)->Edges.size();
		}

        Ptr<Node> IsaGraph::GetGraphSrcNode (uint32_t src)
        {
        	NS_LOG_FUNCTION (this);
        	return (this)->Edges[src][0];
        }

         //Print the graph
        void IsaGraph::printGraph()
        {
        	NS_LOG_FUNCTION (this);
        	for (uint32_t i = 0; i<(this)->getNumofNodes();++i)
            {
        		 uint32_t src = (Edges[i][0])->GetId();
                 NS_LOG_UNCOND("\n Adjacency list of Vertex "<<src<<"\n head ");
                 vector<Ptr<Node>> tempNodeList = Edges[i];
                 while (!tempNodeList.empty())
                 {
                	 uint32_t dest = tempNodeList.back()->GetId();
                	 if (dest==src) break;
                	 NS_LOG_UNCOND("-> "<<dest);
                	 tempNodeList.pop_back();
                 }
                 NS_LOG_UNCOND("\n");
            }
        }

        Ptr<IsaGraph> IsaGraph::flipEdge()
        {
        	NS_LOG_FUNCTION (this);
        	uint32_t nNodes = (this)->getNumofNodes();
        	vector<vector<Ptr<Node>>> tempEdges = (this)->Edges;
        	Ptr<IsaGraph> G = CreateObject<IsaGraph>();
        	for (uint32_t i = 0; i < nNodes;++i)
            {
       		     vector<Ptr<Node>> tempNodeList;
			     tempNodeList.push_back((this)->GetGraphSrcNode(i));
			     G->Edges.push_back(tempNodeList);
            }

        	for (uint32_t i = 0; i < nNodes;++i)
			{
        		 Ptr<Node> src = (this)->GetGraphSrcNode(i);
				 while (!tempEdges[i].empty())
				 {
					 Ptr<Node> dest = tempEdges[i].back();
					 if (dest==src) break;
					 G->addEdge(dest, src);
					 tempEdges[i].pop_back();
				 }
			}
        	return G;
         }

        void IsaGraph::GraphFlows (void)
        {
            NS_LOG_FUNCTION (this);
            uint32_t nNodes = (this)->getNumofNodes();

            vector<bool> visited(nNodes);

            int *path = new int[nNodes];
            int path_index = 0;

            for (int i = 0; i < nNodes; i++)
                visited.push_back(false);

            for (int i = 0; i < nNodes; i++)
            {
            	NS_LOG_UNCOND("GW to Node"<<i <<"\n");
            	BreadthFirstSearchFlows(0, i, visited, path, path_index);
            }

        }

        void IsaGraph::BreadthFirstSearchFlows(int parent, int dest, vector<bool> visited, int path[], int &path_index)
        {
        	NS_LOG_FUNCTION (this);
            visited[parent] = true;
            path[path_index] = parent;
            path_index++;

            if (parent == dest)
            {
                for (int i = 0; i<path_index; i++)
                    NS_LOG_UNCOND(path[i] << " ");
                NS_LOG_UNCOND("\n");
            }
            else
            {
                vector<Ptr<Node>> tempNodeList = Edges[parent];
                while (!tempNodeList.empty())
                {
                	uint32_t nextNode = tempNodeList.back()->GetId();
               	 	if (nextNode==parent) break;
               	 	BreadthFirstSearchFlows(nextNode, dest, visited, path, path_index);
               	 	tempNodeList.pop_back();
                }
            }

            path_index--;
            visited[parent] = false;
        }

 };
