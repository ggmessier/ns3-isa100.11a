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

NS_LOG_COMPONENT_DEFINE ("IsaGraph");

using namespace std;

namespace ns3 {

bool CompareAverageHop (const GraphNode & e1, const GraphNode & e2)
{
  return (e1.m_avgHopCount < e2.m_avgHopCount);
};

TypeId IsaGraph::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::IsaGraph")
    .SetParent<Object> ()
    .AddConstructor<IsaGraph> ()
  ;
  return tid;
}

IsaGraph::IsaGraph ()
{
  NS_LOG_FUNCTION (this);
  (this)->m_gateway = 0;
}

IsaGraph::IsaGraph (NodeContainer c)
{
  NS_LOG_FUNCTION (this);
  (this)->m_gateway = 0;
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      uint32_t nodeId = i->operator -> ()->GetId ();
      (this)->m_graphNodeMap[nodeId].m_head = i->operator -> ();
      (this)->m_graphNodeMap[nodeId].m_avgHopCount = 0;
      (this)->m_graphNodeMap[nodeId].m_reliability = false;
      (this)->m_graphNodeMap[nodeId].m_powerOfrate = -2;
    }
}

IsaGraph::~IsaGraph ()
{
  NS_LOG_FUNCTION (this);
}

vector<Ptr<Node>> IsaGraph::GetEdges (uint32_t src)
{
  NS_LOG_FUNCTION (this);
  return (this)->m_graphNodeMap[src].m_neighbors;
}

void IsaGraph::AddEdge (uint32_t src, uint32_t dest)
{
  NS_LOG_FUNCTION (this);
  vector<Ptr<Node>> srcNeighbors = (this)->m_graphNodeMap[src].m_neighbors;
  vector<Ptr<Node>> destParents = (this)->m_graphNodeMap[dest].m_parents;

  Ptr<Node> srcNode = (this)->m_graphNodeMap[src].m_head;
  Ptr<Node> destNode = (this)->m_graphNodeMap[dest].m_head;

  if (count(srcNeighbors.begin(),srcNeighbors.end(),destNode) == 0)
    {
      (this)->m_graphNodeMap[src].m_neighbors.push_back (destNode);
    }

  if (count(destParents.begin(),destParents.end(),srcNode) == 0)
    {
      (this)->m_graphNodeMap[dest].m_parents.push_back (srcNode);
    }
}

void IsaGraph::AddNode (Ptr<Node> src)
{
  NS_LOG_FUNCTION (this);
  uint32_t nodeId = src->GetId ();
  (this)->m_graphNodeMap[nodeId].m_head = src;
  (this)->m_graphNodeMap[nodeId].m_avgHopCount = 0;
}

void IsaGraph::AddGraphNode (GraphNode graphNode)
{
  NS_LOG_FUNCTION (this);
  uint32_t nodeId = graphNode.m_head->GetId ();
  uint32_t tempNodeId;

  //adding the graph node to the graph
  (this)->m_graphNodeMap[nodeId] = graphNode;

  // update the parents of newly adding graph node neighbors
  for (vector<Ptr<Node>>::const_iterator it = graphNode.m_neighbors.begin ();
         it != graphNode.m_neighbors.end (); ++it)
      {
        tempNodeId= it->operator ->()->GetId();
        vector<Ptr<Node>> tempParents = (this)->m_graphNodeMap[tempNodeId].m_parents;
        if (count(tempParents.begin(),tempParents.end(), graphNode.m_head) == 0)
          {
            (this)->m_graphNodeMap[tempNodeId].m_parents.push_back(graphNode.m_head);
          }
      }

  // update the neighbors of newly adding graph node parents
  for (vector<Ptr<Node>>::const_iterator it = graphNode.m_parents.begin ();
         it != graphNode.m_parents.end (); ++it)
      {
        tempNodeId= it->operator ->()->GetId();
        vector<Ptr<Node>> tempNeighbors = (this)->m_graphNodeMap[tempNodeId].m_parents;
        if (count(tempNeighbors.begin(),tempNeighbors.end(),graphNode.m_head) == 0)
          {
            (this)->m_graphNodeMap[tempNodeId] .m_neighbors.push_back(graphNode.m_head);
          }
      }
}

void IsaGraph::RemoveGraphNode(uint32_t id)
{
  NS_LOG_FUNCTION (this);
  GraphNode graphNode = (this)->m_graphNodeMap[id];

  // remove graph node from it's neighbor's parents vector
  // Consider link A -> B, and Removing node is A
  // iterate over neighbors of A:
  for (vector<Ptr<Node>>::const_iterator it1 = graphNode.m_neighbors.begin ();
         it1 != graphNode.m_neighbors.end (); ++it1)
    {
      // iterate over parents of B:
      // graphNodeNeighbor is the neighbor of the removing node (B)
      GraphNode graphNodeNeighbor = (this)->m_graphNodeMap[it1->operator ->()->GetId()];
      for (uint32_t i = 0; i < graphNodeNeighbor.m_parents.size(); ++i)
        {
          //find parent record matching to node A and remove it
          if(graphNodeNeighbor.m_parents[i]->GetId() == id)
            {
              graphNodeNeighbor.m_parents.erase(graphNodeNeighbor.m_parents.begin()+i);
            }
        }
    }

  // remove graph node from it's parent's neighbor vector
  // Consider link A <- B, and Removing node is A
  // iterate over parents of A:
  for (vector<Ptr<Node>>::const_iterator it1 = graphNode.m_parents.begin ();
         it1 != graphNode.m_parents.end (); ++it1)
    {
      // iterate over neighbors of B:
      // graphNodeNeighbor is the parent of the removing node (B)
      GraphNode graphNodeNeighbor = (this)->m_graphNodeMap[it1->operator ->()->GetId()];
      for (uint32_t i = 0; i < graphNodeNeighbor.m_neighbors.size(); ++i)
        {
          //find neighbor record matching to node A and remove it
          if(graphNodeNeighbor.m_neighbors[i]->GetId() == id)
            {
              graphNodeNeighbor.m_neighbors.erase(graphNodeNeighbor.m_neighbors.begin()+i);
            }
        }
    }

  // remove the graph node from the graph
  (this)->m_graphNodeMap.erase(id);
}

uint32_t IsaGraph::GetNumofNodes (void)
{
  NS_LOG_FUNCTION (this);
  return (this)->m_graphNodeMap.size ();
}

GraphNode IsaGraph::GetGraphNode (uint32_t id)
{
  NS_LOG_FUNCTION (this);
  return (this)->m_graphNodeMap[id];
}

map <uint32_t, GraphNode> IsaGraph::GetGraphNodeMap (void)
{
  NS_LOG_FUNCTION (this);
  return (this)->m_graphNodeMap;
}

void IsaGraph::AddGateway(uint32_t id)
{
  NS_LOG_FUNCTION (this);
  (this)->m_gateway = id;
}

Ptr<Node> IsaGraph::GetGetway (void)
{
  NS_LOG_FUNCTION (this);
  return (this)->m_graphNodeMap[(this)->m_gateway].m_head;
}

void IsaGraph::AddAccessPoint(uint32_t id)
{
  NS_LOG_FUNCTION (this);
  if(count((this)->m_accessPoints.begin(),(this)->m_accessPoints.end(),id) == 0)
    {
      (this)->m_accessPoints.push_back(id);
    }
}

vector<uint32_t> IsaGraph::GetAccessPoints (void)
{
  NS_LOG_FUNCTION (this);
  return (this)->m_accessPoints;
}

void IsaGraph::SetReliability (uint32_t id)
{
  NS_LOG_FUNCTION (this);
  (this)->m_graphNodeMap[id].m_reliability = true;
}

void IsaGraph::PrintGraph ()
{
  NS_LOG_FUNCTION (this);
  for (map<uint32_t, GraphNode>::const_iterator it = (this)->m_graphNodeMap.begin ();
       it != (this)->m_graphNodeMap.end (); ++it)
    {
      NS_LOG_UNCOND ("\n ****** Adjacency list of Vertex " << it->second.m_head->GetId ());
      vector<Ptr<Node> > tempNodeList = it->second.m_neighbors;
      NS_LOG_UNCOND ("\n Neighbors: ");

      while (!tempNodeList.empty ())
        {
          NS_LOG_UNCOND (" -> " << tempNodeList.back ()->GetId ());
          tempNodeList.pop_back ();
        }
      NS_LOG_UNCOND ("\n Parents: ");

      tempNodeList = it->second.m_parents;

      while (!tempNodeList.empty ())
        {
          NS_LOG_UNCOND (" -> " << tempNodeList.back ()->GetId ());
          tempNodeList.pop_back ();
        }
//      NS_LOG_UNCOND ("\n");
    }
}

Ptr<IsaGraph> IsaGraph::FlipEdge ()
{
  NS_LOG_FUNCTION (this);
  Ptr<IsaGraph> G = CreateObject<IsaGraph> ();
  G = (this);

  for (map<uint32_t, GraphNode>::const_iterator it = (this)->m_graphNodeMap.begin ();
       it != (this)->m_graphNodeMap.end (); ++it)
    {
      vector<Ptr<Node> > tempEdges = (it->second).m_parents;
      G->m_graphNodeMap[it->first].m_parents = it->second.m_neighbors;
      G->m_graphNodeMap[it->first].m_neighbors = tempEdges;
    }

  return G;
}

void IsaGraph::SetHopCount (uint32_t id, double hopCount)
{
  NS_LOG_FUNCTION (this);
  (this)->m_graphNodeMap[id].m_avgHopCount = hopCount;
}

double IsaGraph::GetHopCount (uint32_t id)
{
  NS_LOG_FUNCTION (this);
  return (this)->m_graphNodeMap[id].m_avgHopCount;
}

bool IsaGraph::ReliableBroadcastGraph (Ptr<IsaGraph> G)
{
  NS_LOG_FUNCTION (this);

  // Grpah itself has considered S and it will initially include the getway and the access points
  bool S_2;                             ///< vector for nodes have at least two edges from VB
  bool S_1;                             ///< vector for nodes have one edge from VB
  GraphNode nodeMinHop;                 ///< Graph Node with minimum hops from the gateway
  GraphNode nodeMaxoutgoingEdges;       ///< Graph Node with maximum outgoing edges
  uint32_t maxOutgoingEdges;            ///< Number of outgoing edges of the maximum outgoing edges node

  /*
   *          S
   *       ------
   *        |  |      // edgesForS is to find all the edges coming from S (traversed nodes) to V-S (need to traverse)
   *        v  v
   *       ------
   *        V -S
   *
   */
  map <uint32_t, GraphNode> edgesForS = (this)->m_graphNodeMap;

  while ((this)->GetNumofNodes () < G->GetNumofNodes ())         // S != V
    {
      S_2 = false;
      S_1 = false;
      maxOutgoingEdges = 0;
      nodeMaxoutgoingEdges = (this)->GetGraphNode((this)->GetGetway()->GetId());
      edgesForS = (this)->UpdateSVector (G, edgesForS);

      for (map<uint32_t, GraphNode>::const_iterator it = edgesForS.begin ();
           it != edgesForS.end (); ++it)
        {
          if ((this)->m_graphNodeMap.count (it->first) != 1)
            { //considering only the V - VB nodes
              double hop_count;
              vector<GraphNode> tempParents;

              for (uint32_t i = 0; i < (it->second).m_parents.size (); ++i)
                {
                  uint32_t nextNode = (it->second).m_parents[i].operator -> ()->GetId ();
                  if ((this)->m_graphNodeMap.count (nextNode))
                    {
                      tempParents.push_back ((this)->m_graphNodeMap[nextNode]);
                    }
                }

              if (tempParents.size () >= 2)
                {
                  sort (tempParents.begin (), tempParents.end (),CompareAverageHop);
                  hop_count = 0.5 * ((this)->m_graphNodeMap[tempParents[0].m_head->GetId ()].m_avgHopCount
                                     + (this)->m_graphNodeMap[tempParents[1].m_head->GetId ()].m_avgHopCount) + 1;
                  edgesForS[(it->first)].m_parents[0] = tempParents[0].m_head;
                  edgesForS[(it->first)].m_parents[1] = tempParents[1].m_head;
                  G->m_graphNodeMap[it->first].m_avgHopCount = hop_count;
                  if (!S_2 || (nodeMinHop.m_avgHopCount > hop_count))
                    {
                      nodeMinHop = G->m_graphNodeMap[it->first];
                      S_2 = true;
                    }
                }
              else if (tempParents.size () == 1 && !S_2)
                {
                  uint32_t outgoingEdges = 0;
                  hop_count = (this)->m_graphNodeMap[tempParents[0].m_head->GetId()].m_avgHopCount + 1;
                   G->m_graphNodeMap[it->first].m_avgHopCount = hop_count;
                  vector<Ptr<Node> > tempNeighbours = G->m_graphNodeMap[it->first].m_neighbors;
                  for (uint32_t i = 0; i < tempNeighbours.size (); i++)
                    {
                      if ((this)->m_graphNodeMap.count (tempNeighbours[i]->GetId ()) != 1)
                        {
                          outgoingEdges = outgoingEdges + 1;
                        }
                    }
                  if (maxOutgoingEdges <= outgoingEdges)
                    {
                      if (nodeMaxoutgoingEdges.m_head->GetId () == 0)  //gateway node ID = 0
                        {
                          maxOutgoingEdges = outgoingEdges;
                          nodeMaxoutgoingEdges = G->m_graphNodeMap[it->first];
                        }
                      else if (nodeMaxoutgoingEdges.m_avgHopCount > hop_count)
                        {
                          maxOutgoingEdges = outgoingEdges;
                          nodeMaxoutgoingEdges = G->m_graphNodeMap[it->first];
                        }
                    }
                  S_1 = true;
                }
            }
        }

      if (S_2)
        {
          (this)->AddNode (nodeMinHop.m_head);
          (this)->SetHopCount (nodeMinHop.m_head->GetId (), nodeMinHop.m_avgHopCount);
          GraphNode tempNode = edgesForS[nodeMinHop.m_head->GetId ()];
          (this)->AddEdge (tempNode.m_parents[0]->GetId (), nodeMinHop.m_head->GetId ());
          (this)->AddEdge (tempNode.m_parents[1]->GetId (), nodeMinHop.m_head->GetId ());
          (this)->SetReliability(nodeMinHop.m_head->GetId ());
        }
      else if (S_1)
        {
          (this)->AddNode (nodeMaxoutgoingEdges.m_head);
          (this)->SetHopCount (nodeMaxoutgoingEdges.m_head->GetId (), nodeMaxoutgoingEdges.m_avgHopCount);
          GraphNode tempNode = edgesForS[nodeMaxoutgoingEdges.m_head->GetId ()];
          (this)->AddEdge (tempNode.m_parents[0]->GetId (), nodeMaxoutgoingEdges.m_head->GetId ());
        }
    }

  return (S_1 || S_2);
}

map <uint32_t, GraphNode> IsaGraph::UpdateSVector (Ptr<IsaGraph> G, map <uint32_t, GraphNode> edgesForS)
{
  NS_LOG_FUNCTION (this);
  map <uint32_t, GraphNode> tempEdgesOfS = edgesForS;

  for (map<uint32_t, GraphNode>::const_iterator it = edgesForS.begin ();
       it != edgesForS.end (); ++it)
    {
      bool newNode = false;
      //update neighbors and parents of tempEdgesofS
      vector<Ptr<Node> > tempNodeList = G->m_graphNodeMap[it->first].m_neighbors;
      tempEdgesOfS[it->first].m_neighbors = tempNodeList;
      tempEdgesOfS[it->first].m_parents = G->m_graphNodeMap[it->first].m_parents;

      while (!tempNodeList.empty ())
        {
          uint32_t nextNode = tempNodeList.back ()->GetId ();
          //Node will only consider for selection if it's a having new neighbors
          if ((this)->m_graphNodeMap.count (nextNode) != 1)
            {
              tempEdgesOfS[nextNode] = G->m_graphNodeMap[nextNode];
              newNode = true;
            }
          tempNodeList.pop_back ();
        }

      /* all nodes do not having new neighbors will be removed from the selection vector.
       * This is to reduce the iteration times in edgesforS vector
       */
      if (!newNode)
        {
          tempEdgesOfS.erase (it->first);
        }
    }

  //update the edgesForS vector
  return tempEdgesOfS;
}

bool IsaGraph::ReliableUplinkGraph (Ptr<IsaGraph> G)
{
  NS_LOG_FUNCTION (this);
  // Initial graph with gateway and APs edges reversed.
  (this)->FlipEdge();

  // G graph reversed.
  Ptr<IsaGraph> G_reverse = G->FlipEdge();

  // Generate reliable broadcast graph from uplink flipped graph
  (this)->ReliableBroadcastGraph(G_reverse);

  if(G->GetNumofNodes()==(this)->GetNumofNodes())
    {
      // Flip edges of created reliable broadcast graph
      (this)->FlipEdge();
    }
  else
    {
      // Uplink graph is disconnected.
      return false;
    }
  // Reliable uplink graph creation succeeded.
  return true;
}

map <uint32_t, Ptr<IsaGraph>> IsaGraph::ReliableDownlinkGraphs (Ptr<IsaGraph> G)
{
  NS_LOG_FUNCTION (this);

  Ptr<IsaGraph> S = CreateObject<IsaGraph>();        ///< graph S to count the nodes already explored.

  map <uint32_t, Ptr<IsaGraph>> downlinkGraphs_G;               ///< Downlink graphs of the graph G

  // ***** begin ***** create Downlink graphs for all access points of the graph G
  Ptr<Node> gateWay = G->GetGetway();
  S->AddNode(gateWay);

  vector<uint32_t > tempAccessPoints = G->GetAccessPoints();

  while (!tempAccessPoints.empty ())
   {
     uint32_t nextAccessPointID = tempAccessPoints.back ();

     Ptr<IsaGraph> G_AP = CreateObject<IsaGraph> ();                      ///< Downlink graph creation for access points
     G_AP->AddNode(gateWay);
     G_AP->AddGateway(gateWay->GetId());                                  // Set gate way of the Downlink graph
     Ptr<Node> accessPoint = G->GetGraphNode(nextAccessPointID).m_head;
     S->AddNode(accessPoint);
     G_AP->AddNode(accessPoint);
     G_AP->AddAccessPoint(accessPoint->GetId());                          // Set access point of the Downlink graph
     G_AP->AddEdge(gateWay->GetId(), accessPoint->GetId());               // adding edge e(g,i)
     G_AP->SetHopCount(accessPoint->GetId(), 1);
     downlinkGraphs_G[accessPoint->GetId()] = G_AP;                       // Set Gi (Downlink graph for ith node) - For Access points

     tempAccessPoints.pop_back ();
   }

  // ***** end ***** create Downlink graphs for all access points of the graph G

  map <uint32_t, GraphNode> edgesForS = S->m_graphNodeMap;                ///< vector to determine the number of parents of the respective node

  while (S->GetNumofNodes() < G->GetNumofNodes ())      // while S != V (Nodes of the Graph G)
    {
      GraphNode nodeMinHopPv;           ///< Graph Node with minimum hops from the gateway and satisfying the three conditions
      GraphNode nodeMinHopS_2;           ///< Graph Node with minimum hops from the gateway and satisfying the three conditions
      GraphNode nodeMinHopS_1;          ///< Graph Node with minimum hops from the gateway and which not satisfying the three conditions

      edgesForS = S->UpdateSVector (G, edgesForS);       // Nodes and edges that need to be consider for the Downlink graph creation

      // initially set the hop counts as the maximum possible +1
      uint32_t notPossibleHopCount = G->GetNumofNodes ()+1;
      nodeMinHopS_2.m_avgHopCount = notPossibleHopCount;
      nodeMinHopPv.m_avgHopCount = notPossibleHopCount;
      nodeMinHopS_1.m_avgHopCount = notPossibleHopCount;

      for (map<uint32_t, GraphNode>::const_iterator it = edgesForS.begin ();
             it != edgesForS.end (); ++it)
          {
            if (S->m_graphNodeMap.count (it->first) == 0)   // check whether node is already in the explored node list or not
              {
                GraphNode tempNode;                     // Node which having two parents considering at the moment (temporary Node v)
                vector<GraphNode> tempParents;     // temporary vector for  all the parents of the tempNode in S (explored Nodes)
                tempNode.m_head = it->second.m_head;

                for (uint32_t i = 0; i < (it->second).m_parents.size (); ++i)
                  {
                    uint32_t nextNode = (it->second).m_parents[i].operator -> ()->GetId ();
                    if (S->m_graphNodeMap.count (nextNode))
                      {
                        tempParents.push_back (S->m_graphNodeMap[nextNode]);
                      }
                  }

                if (tempParents.size () >= 2)              //nodes having at least two edges from S
                  {
                    //iterate over all the edges to find the edge pair that need to add to the Downlink graph of the respective node
                    vector<GraphNode> tempNew = tempParents;               ///< copy of temParents vector

                    while (!tempNew.empty())
                      {
                        GraphNode u1;
                        u1.m_head = tempNew.back().m_head;            // get the first parent of the node v
                        uint32_t u1Id = u1.m_head->GetId();
                        tempNew.pop_back();

                        tempNode.m_parents.push_back(u1.m_head);    // add parent u1 to the parents list

                        // consider u1 above with all other parents (u2)
                        for (vector<GraphNode>::const_iterator tempU2 = tempNew.begin ();
                             tempU2 != tempNew.end (); ++tempU2)
                          {
                            GraphNode u2;
                            u2.m_head = tempU2->m_head;
                            uint32_t u2Id = u2.m_head->GetId();
                            tempNode.m_parents.push_back(u2.m_head);
                            tempNode.m_avgHopCount = (downlinkGraphs_G[u1Id]->GetHopCount(u1Id) +
                                downlinkGraphs_G[u2Id]->GetHopCount(u2Id))/2;

                            if(G->C1Condition(u1.m_head, u2.m_head) && (G->C2Condition(u1.m_head, u2.m_head)
                                || G->C3Condition(u1.m_head, u2.m_head, downlinkGraphs_G)))
                              {
                                if(nodeMinHopPv.m_avgHopCount>tempNode.m_avgHopCount)
                                  {
                                    nodeMinHopPv = tempNode;
                                  }
                              }
                            else if (nodeMinHopS_2.m_avgHopCount>tempNode.m_avgHopCount)
                              {
                                nodeMinHopS_2 = tempNode;
                              }
                            tempNode.m_parents.pop_back();    //remove u2 from parents list to consider new parent
                          }
                        tempNode.m_parents.pop_back();      //remove u1 from parents list to consider new parent
                      }
                    // end of iteration over all edge pairs of v
                  }
                else if (tempParents.size () == 1)              //nodes having one edge from S
                  {
                    GraphNode u1;
                    u1.m_head = tempParents.back().m_head;                // get the first parent of the node v
                    uint32_t u1Id = u1.m_head->GetId();
                    tempNode.m_parents.push_back(u1.m_head);    // add parent u1 to the parents list

                    if(nodeMinHopS_1.m_avgHopCount>downlinkGraphs_G[u1Id]->GetHopCount(u1Id))
                      {
                        nodeMinHopS_1 = tempNode;
                        nodeMinHopS_1.m_avgHopCount = downlinkGraphs_G[u1Id]->GetHopCount(u1Id);    // setting the average hop count of parents to S_1 Node
                      }

                    tempNode.m_parents.pop_back();      //remove u1 from parents list to consider new parent
                  }
              }
          }// end of iterator of selected edges (edgesForS)

      if (nodeMinHopPv.m_avgHopCount < notPossibleHopCount)
        {
          nodeMinHopPv.m_reliability = true;            // C1 and (C2 or C3) condition satisfying. Therefore, Node is having reliable downlink graph
          nodeMinHopS_2 = nodeMinHopPv;
        }

      if(nodeMinHopS_2.m_avgHopCount < notPossibleHopCount)
        {
          nodeMinHopS_2.m_avgHopCount++;                  // hop count of u1u2 average considered for above algorithm, but hv is +1 to it.
          downlinkGraphs_G = (this)->ConstructDownlinkGraphs(G,nodeMinHopS_2, downlinkGraphs_G);
          S->AddNode(nodeMinHopS_2.m_head);
          downlinkGraphs_G[nodeMinHopS_2.m_head->GetId()]->PrintGraph();            //Temporary *************************
        }
      else if(nodeMinHopS_1.m_avgHopCount < notPossibleHopCount)
        {
          nodeMinHopS_1.m_avgHopCount++;                  // hop count of u1 considered for above algorithm, but hv is +1 to it.
          uint32_t nodeId = nodeMinHopS_1.m_head->GetId();
          uint32_t nodeParentId = nodeMinHopS_1.m_parents[0]->GetId();

          Ptr<IsaGraph> G_v = CreateObject<IsaGraph> ();                      ///< Downlink graph creation for the selected Node
          G_v.operator *() = downlinkGraphs_G[nodeParentId].operator *();
          G_v->AddGraphNode(nodeMinHopS_1);             // add graph node S_1
          G_v->AddEdge(nodeParentId, nodeId);           // Adding edge of u1 and v
          downlinkGraphs_G[nodeId] = G_v;               // Add Downlink graph to the graph list
          S->AddNode(nodeMinHopS_1.m_head);
          downlinkGraphs_G[nodeId]->PrintGraph();            //Temporary *************************
        }

    } // end of while loop for S != V

  return downlinkGraphs_G;
}

map <uint32_t, Ptr<IsaGraph>> IsaGraph::ConstructDownlinkGraphs (Ptr<IsaGraph> G, GraphNode v, map <uint32_t, Ptr<IsaGraph>> downlinkGraphs)
{
  Ptr<IsaGraph> Gv = CreateObject<IsaGraph> ();                      ///< initial downlink graph (Gv) creation with v

  Ptr<Node> u1 = v.m_parents[0];            // first parent of the node v
  Ptr<Node> u2 = v.m_parents[1];            // second parent of the node v

  uint32_t vId = v.m_head->GetId();              // Id of the Node v
  uint32_t u1Id = u1->GetId();            // Id of the first parent of v
  uint32_t u2Id = u2->GetId();            // Id of the second parent of v

  vector<Ptr<Node>> u2Parents = downlinkGraphs[u2Id]->GetGraphNode(u2Id).m_parents;    // parents of node u1 in it's respective Downlink graph
  vector<Ptr<Node>> u1Parents = downlinkGraphs[u1Id]->GetGraphNode(u1Id).m_parents;    // parents of node u1 in it's respective Downlink graph
  uint32_t u1HopCount = downlinkGraphs[u1Id]->GetGraphNode(u1Id).m_avgHopCount;             // hop count of u1 in it's Downlink graph
  uint32_t u2HopCount = downlinkGraphs[u2Id]->GetGraphNode(u2Id).m_avgHopCount;             // hop count of u2 in it's Downlink graph

  /*  check u1 and u2 satisfy C1 and C2 conditions
   *
   *             u1 <   <--->X (in G)
   *             |   |       |
   *             |    >  u2 <
   *             |       |
   *              >  v  <
   */
  if (G->C1Condition(u1, u2) && G->C2Condition(u1, u2))
    {
      if (count(u2Parents.begin(),u2Parents.end(),u1) >= 1)
        {
          /*  Check u1-> u2 is in Gu2
           *
           *           u1<--->u2 (Gu2)
           *            |     |
           *             > v <
           */
          Gv.operator *() = downlinkGraphs[u2Id].operator *();
          Gv->AddGraphNode(v);                      //Add Node v
          Gv->AddEdge(u1Id, vId);                   //Add edge u1 ---> v
          Gv->AddEdge(u2Id, vId);                   //Add edge u2 ---> v
          Gv->AddEdge(u2Id, u1Id);                  //Add edge u2 ---> u1
          downlinkGraphs[vId] = Gv;
        }
      else
        {
          /*
           *     (Gu1) u1<--->u2
           *            |     |
           *             > v <
           */
          Gv.operator *() = downlinkGraphs[u1Id].operator *();
          Gv->AddGraphNode(v);                      //Add Node v
          Gv->AddNode(u2);                          //Add Node u2
          Gv->AddEdge(u1Id, vId);                   //Add edge u1 ---> v
          Gv->AddEdge(u2Id, vId);                   //Add edge u2 ---> v
          Gv->AddEdge(u2Id, u1Id);                  //Add edge u2 ---> u1
          Gv->AddEdge(u1Id, u2Id);                  //Add edge u1 ---> u2
          downlinkGraphs[vId] = Gv;
        }
    }
  /*  u1 (u2) has at least one parent from the cycle in Gu2 (Gu1)
   *
   *             x1   <--->  x2 (find x1 which is parent of u1 (u2))
   *           /    \        |
   *          |       >  u2 < (u1)
   *          |         | |
   *           > u1 <-->  |   (u2)
   *              |       |
   *               >  v  <
   *
   */
  else if (G->C1Condition(u1, u2) && G->C3Condition(u1, u2, downlinkGraphs))
    {
      for (vector<Ptr<Node>>::const_iterator it1 = u2Parents.begin ();
                it1 != u2Parents.end (); ++it1)
        {
          vector<Ptr<Node>> neighborsOfU2Parent = downlinkGraphs[u2Id]->
              GetGraphNode(it1->operator ->()->GetId()).m_neighbors;
          for (vector<Ptr<Node>>::const_iterator it2 = neighborsOfU2Parent.begin ();
                          it2 != neighborsOfU2Parent.end (); ++it2)
            {
              if (it2->operator ->() == u1)
                {
                  Gv.operator *() = downlinkGraphs[u2Id].operator *();
                  Gv->AddNode(u1);                                    //Add node u1 to Gv
                  Gv->AddEdge(it1->operator ->()->GetId(), u1Id);     //Add edge parent of u2 with u1
                  Gv->AddGraphNode(v);                           //Add Node v
                  Gv->AddEdge(u1Id, vId);                   //Add edge u1 ---> v
                  Gv->AddEdge(u2Id, vId);                   //Add edge u2 ---> v
                  Gv->AddEdge(u2Id, u1Id);                  //Add edge u2 ---> u1
                  Gv->AddEdge(u1Id, u2Id);                  //Add edge u1 ---> u2
                  downlinkGraphs[vId] = Gv;
                }
            }
        }

      if (u2HopCount < u1HopCount)
        {
          for (vector<Ptr<Node>>::const_iterator it1 = u1Parents.begin ();
                                it1 != u1Parents.end (); ++it1)
            {
              vector<Ptr<Node>> neighborsOfU1Parent = downlinkGraphs[u1Id]->
                  GetGraphNode(it1->operator ->()->GetId()).m_neighbors;
              for (vector<Ptr<Node>>::const_iterator it2 = neighborsOfU1Parent.begin ();
                              it2 != neighborsOfU1Parent.end (); ++it2)
                {
                  if (it2->operator ->() == u2)
                    {
                      Gv.operator *() = downlinkGraphs[u1Id].operator *();
                      Gv->AddNode(u2);                                    //Add node u1 to Gv
                      Gv->AddEdge(it1->operator ->()->GetId(), u2Id);     //Add edge parent of u2 with u1
                      Gv->AddGraphNode(v);                           //Add Node v
                      Gv->AddEdge(u1Id, vId);                   //Add edge u1 ---> v
                      Gv->AddEdge(u2Id, vId);                   //Add edge u2 ---> v
                      Gv->AddEdge(u2Id, u1Id);                  //Add edge u2 ---> u1
                      Gv->AddEdge(u1Id, u2Id);                  //Add edge u1 ---> u2
                      downlinkGraphs[vId] = Gv;
                    }
                }
            }
        }
    }
  else
    {
      bool edgeU1toU2 = (count(G->m_graphNodeMap[u1Id].m_neighbors.begin(),
                               G->m_graphNodeMap[u1Id].m_neighbors.end (), u2) >= 1);
      bool edgeU2toU1 = (count(G->m_graphNodeMap[u2Id].m_neighbors.begin(),
                               G->m_graphNodeMap[u2Id].m_neighbors.end (), u1) >= 1);
      if(edgeU1toU2 && edgeU2toU1)
        {
          if(u1HopCount < u2HopCount)
            {
              Gv.operator *() = downlinkGraphs[u1Id].operator *();
              Gv->AddNode(u2);                  //Add node u1 to Gv
            }
          else
            {
              Gv.operator *() = downlinkGraphs[u2Id].operator *();
              Gv->AddNode(u1);                  //Add node u1 to Gv
            }
          Gv->AddGraphNode(v);         //Add Node v
          Gv->AddEdge(u1Id, vId);                   //Add edge u1 ---> v
          Gv->AddEdge(u2Id, vId);                   //Add edge u2 ---> v
          Gv->AddEdge(u2Id, u1Id);                  //Add edge u2 ---> u1
          Gv->AddEdge(u1Id, u2Id);                  //Add edge u1 ---> u2
          downlinkGraphs[vId] = Gv;
        }
      else if(!edgeU1toU2 && !edgeU2toU1)
        {
          if(u1HopCount < u2HopCount)
            {
              Gv.operator *() = downlinkGraphs[u1Id].operator *();
              v.m_parents.erase(v.m_parents.end());
              Gv->AddGraphNode(v);         //Add Node v
              Gv->AddEdge(u1Id, vId);                   //Add edge u1 ---> v
            }
          else
            {
              Gv.operator *() = downlinkGraphs[u2Id].operator *();
              v.m_parents.erase(v.m_parents.begin());
              Gv->AddGraphNode(v);         //Add Node v
              Gv->AddEdge(u2Id, vId);                   //Add edge u2 ---> v
            }
          downlinkGraphs[vId] = Gv;
        }
      else if(edgeU1toU2)
        {
          Gv.operator *() = downlinkGraphs[u1Id].operator *();
          Gv->AddNode(u2);                                    //Add node u1 to Gv
          Gv->AddGraphNode(v);                           //Add Node v
          Gv->AddEdge(u1Id, vId);                   //Add edge u1 ---> v
          Gv->AddEdge(u2Id, vId);                   //Add edge u2 ---> v
          Gv->AddEdge(u1Id, u2Id);                  //Add edge u1 ---> u2
          downlinkGraphs[vId] = Gv;
        }
      else
        {
          Gv.operator *() = downlinkGraphs[u2Id].operator *();
          Gv->AddNode(u1);                                    //Add node u1 to Gv
          Gv->AddGraphNode(v);                           //Add Node v
          Gv->AddEdge(u1Id, vId);                   //Add edge u1 ---> v
          Gv->AddEdge(u2Id, vId);                   //Add edge u2 ---> v
          Gv->AddEdge(u2Id, u1Id);                  //Add edge u2 ---> u1
          downlinkGraphs[vId] = Gv;
        }
    }
  return downlinkGraphs;
}

//*- C1 v has at least two parents u1, u2, and they form a cycle.
bool IsaGraph::C1Condition (Ptr<Node> u1, Ptr<Node> u2)
{
  NS_LOG_FUNCTION (this);

  uint32_t u1Id = u1->GetId();        ///< node Id of the first parent of the respective node v
  uint32_t u2Id = u2->GetId();        ///< node Id of the second parent of the respective node v

  GraphNode u1Gn = (this)->m_graphNodeMap[u1Id];

  // check u1 and u2 not representing the same node
  if (u1Id != u2Id)
    {
      /*  check (u2 is a neighbor of u1) and (u2 is a parent of u1)
       *
       *           u2<---->(u1)
       *
       */
      if (count(u1Gn.m_neighbors.begin(),u1Gn.m_neighbors.end(),u2) >= 1)
        {
          if (count(u1Gn.m_parents.begin(),u1Gn.m_parents.end(),u2) >= 1)
            {
              return true;
            }
        }
  }

  return false;
}

//*- C2 u1 is u2’s parent in u2’s "local" Downlink graph (Node and it's immediate parents)
bool IsaGraph::C2Condition (Ptr<Node> u1, Ptr<Node> u2)
{
  NS_LOG_FUNCTION (this);

  uint32_t u1Id = u1->GetId();        ///< node Id of the first parent of the respective node v
  uint32_t u2Id = u2->GetId();        ///< node Id of the second parent of the respective node v

  // check u1 and u2 not representing the same node
  if (u1Id != u2Id)
    {
      /*  check u1 is a parent of u2's local Downlink graph in G
       *
       *           x1<---->x2 (check if x1 of x2 is u1)
       *            |      |
       *             > u2 <
       */
      vector<Ptr<Node>> u2Parents = (this)->GetGraphNode(u2Id).m_parents;

      if (count(u2Parents.begin(),u2Parents.end(),u1) >= 1)
        {
          return true;
        }
    }

  return false;
}

//*- C3 u2 (u1) has at least one parent from the cycle in Gu1 (Gu2)
bool IsaGraph::C3Condition (Ptr<Node> u1, Ptr<Node> u2, map <uint32_t, Ptr<IsaGraph>> downlinkGraphs)
{
  NS_LOG_FUNCTION (this);

  uint32_t u1Id = u1->GetId();       ///< node Id of the first parent of the respective node v
  uint32_t u2Id = u2->GetId();       ///< node Id of the second parent of the respective node v

  GraphNode u1Gn = (this)->m_graphNodeMap[u1Id];
  GraphNode u2Gn = (this)->m_graphNodeMap[u2Id];

  // check u1 and u2 not representing the same node
  if (u1Id != u2Id)
    {
      /*  u1 has at least one parent from the cycle in Gu2
       *
       *           x1  <--->x2 (check if x1 is parent of u1)
       *          |  |      |
       *          |   > u2 <
       *          |
       *           > u1
       */
      vector<Ptr<Node>> u2Parents = downlinkGraphs[u2Id]->GetGraphNode(u2Id).m_parents;

      for (vector<Ptr<Node>>::const_iterator it = u2Parents.begin ();
                it != u2Parents.end (); ++it)
        {
          if (count(u1Gn.m_parents.begin(),u1Gn.m_parents.end(),it->operator ->()) >= 1)
            {
              return true;
            }
        }

      /*  u2 has at least one parent from the cycle in Gu1
       *
       *           x1  <--->x2 (check if x1 is parent of u2)
       *          |  |      |
       *          |   > u1 <
       *          |
       *           > u2
       */
      vector<Ptr<Node>> u1Parents = downlinkGraphs[u1Id]->GetGraphNode(u1Id).m_parents;

      for (vector<Ptr<Node>>::const_iterator it = u1Parents.begin ();
                it != u1Parents.end (); ++it)
        {
          if (count(u2Gn.m_parents.begin(),u2Gn.m_parents.end(),it->operator ->()) >= 1)
            {
              return true;
            }
        }
    }

  return false;
}

} // namespace ns3
