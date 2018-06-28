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
}

IsaGraph::IsaGraph (NodeContainer c)
{
  NS_LOG_FUNCTION (this);
  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      uint32_t nodeId = i->operator -> ()->GetId ();
      (this)->m_graphNodeMap[nodeId].m_head = i->operator -> ();
      (this)->m_graphNodeMap[nodeId].m_avgHopCount = 0;
      (this)->m_graphNodeMap[nodeId].m_reliability = false;
    }
}

IsaGraph::~IsaGraph ()
{
  NS_LOG_FUNCTION (this);
}

vector<Ptr<GraphNode>> IsaGraph::GetEdges (uint32_t src)
{
  NS_LOG_FUNCTION (this);
  return (this)->m_graphNodeMap[src].m_neighbors;
}

void IsaGraph::AddEdge (uint32_t src, uint32_t dest)
{
  NS_LOG_FUNCTION (this);
  vector<Ptr<GraphNode>> srcNeighbors = (this)->m_graphNodeMap[src].m_neighbors;
  vector<Ptr<GraphNode>> destParents = (this)->m_graphNodeMap[dest].m_parents;

  Ptr<GraphNode> srcNode = &((this)->m_graphNodeMap[src]);
  Ptr<GraphNode> destNode = &((this)->m_graphNodeMap[dest]);

  if (count(srcNeighbors.begin(),srcNeighbors.end(),destNode) == 0)
    {
      (this)->m_graphNodeMap[src].m_neighbors.push_back (destNode);
    }

  if (count(destParents.begin(),destParents.end(),srcNode) == 0)
    {
      (this)->m_graphNodeMap[src].m_neighbors.push_back (srcNode);
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
  for (vector<Ptr<GraphNode>>::const_iterator it = graphNode.m_neighbors.begin ();
         it != graphNode.m_neighbors.end (); ++it)
      {
        tempNodeId= it->operator ->()->m_head->GetId();
        vector<Ptr<GraphNode>> tempParents = (this)->m_graphNodeMap[tempNodeId].m_parents;
        if (count(tempParents.begin(),tempParents.end(),&graphNode) == 0)
          {
            (this)->m_graphNodeMap[tempNodeId].m_parents.push_back(&graphNode);
          }
      }

  // update the neighbors of newly adding graph node parents
  for (vector<Ptr<GraphNode>>::const_iterator it = graphNode.m_parents.begin ();
         it != graphNode.m_parents.end (); ++it)
      {
        tempNodeId= it->operator ->()->m_head->GetId();
        vector<Ptr<GraphNode>> tempNeighbors = (this)->m_graphNodeMap[tempNodeId].m_parents;
        if (count(tempNeighbors.begin(),tempNeighbors.end(),&graphNode) == 0)
          {
            (this)->m_graphNodeMap[tempNodeId] .m_neighbors.push_back(&graphNode);
          }
      }
}

void IsaGraph::RemoveGraphNode(uint32_t id)
{
  NS_LOG_FUNCTION (this);
  GraphNode graphNode = (this)->m_graphNodeMap[id];
  uint32_t tempNodeId;

  // remove graph node from it's neighbors (parents vector)
  for (vector<Ptr<GraphNode>>::const_iterator it1 = graphNode.m_neighbors.begin ();
         it1 != graphNode.m_neighbors.end (); ++it1)
      {
        for (vector<Ptr<GraphNode>>::const_iterator it2 = graphNode.m_parents.begin ();
                 it2 != graphNode.m_parents.end (); ++it2)
              {
                if(it1->operator ->() == it2->operator ->())
                  {
                    tempNodeId = it2->operator ->()->m_head->GetId();
                    (this)->m_graphNodeMap[tempNodeId].m_parents.erase
                                    ((this)->m_graphNodeMap[tempNodeId].m_parents.begin()+it2);
                  }
              }
      }

  // update the neighbors of newly adding graph node parents
  for (vector<Ptr<GraphNode>>::const_iterator it1 = graphNode.m_parents.begin ();
         it1 != graphNode.m_parents.end (); ++it1)
      {
        for (vector<Ptr<GraphNode>>::const_iterator it2 = graphNode.m_neighbors.begin ();
                 it2 != graphNode.m_neighbors.end (); ++it2)
              {
                if(it1->operator ->() == it2->operator ->())
                  {
                    tempNodeId = it2->operator ->()->m_head->GetId();
                    (this)->m_graphNodeMap[tempNodeId].m_neighbors.erase
                                    ((this)->m_graphNodeMap[tempNodeId].m_neighbors.begin()+it2);
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

Ptr<Node> IsaGraph::GetGraphNodeHead (uint32_t id)
{
  NS_LOG_FUNCTION (this);
  return (this)->m_graphNodeMap[id].m_head;
}

GraphNode IsaGraph::GetGraphNode (uint32_t id)
{
  NS_LOG_FUNCTION (this);
  return (this)->m_graphNodeMap[id];
}

void IsaGraph::AddGateway(uint32_t id)
{
  NS_LOG_FUNCTION (this);
  (this)->m_gateway = &((this)->m_graphNodeMap[id]);
}

Ptr<GraphNode> IsaGraph::GetGetway (void)
{
  NS_LOG_FUNCTION (this);
  return (this)->m_gateway;
}

void IsaGraph::AddAccessPoint(uint32_t id)
{
  NS_LOG_FUNCTION (this);
  (this)->m_accessPoints.push_back(&((this)->m_graphNodeMap[id]));
}

vector<Ptr<GraphNode>> IsaGraph::GetAccessPoints (void)
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
      NS_LOG_UNCOND ("\n Adjacency list of Vertex " << it->second.m_head->GetId () << "\n head ");
      vector<Ptr<GraphNode> > tempNodeList = it->second.m_neighbors;

      while (!tempNodeList.empty ())
        {
          NS_LOG_UNCOND ("-> " << tempNodeList.back ()->m_head->GetId ());
          tempNodeList.pop_back ();
        }
      NS_LOG_UNCOND ("\n");

      tempNodeList = it->second.m_parents;

      while (!tempNodeList.empty ())
        {
          NS_LOG_UNCOND ("-> " << tempNodeList.back ()->m_head->GetId ());
          tempNodeList.pop_back ();
        }
      NS_LOG_UNCOND ("\n");
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
      vector<Ptr<GraphNode> > tempEdges = (it->second).m_parents;
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
  return (this)->BroadcastGraph (G, (this)->m_graphNodeMap);
}

bool IsaGraph::BroadcastGraph (Ptr<IsaGraph> G, map <uint32_t, GraphNode> edgesForS)
{
  NS_LOG_FUNCTION (this);

  bool S_2 = false;                 ///< vector for nodes have at least two edges from VB
  bool S_1 = false;                 ///< vector for nodes have one edge from VB
  GraphNode nodeMinHop;             ///< Graph Node with minimum hops from the gateway
  GraphNode nodeMaxoutgoingEdges;   ///< Graph Node with maximum outgoing edges
  uint32_t maxOutgoingEdges = 0;    ///< Number of outgoing edges of the maximum outgoing edges node

  nodeMaxoutgoingEdges = (this)->GetGetway().operator *();
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
              uint32_t nextNode = (it->second).m_parents[i].operator -> ()->m_head->GetId ();
              if ((this)->m_graphNodeMap.count (nextNode))
                {
                  tempParents.push_back ((this)->m_graphNodeMap[nextNode]);
                }
            }

          if (tempParents.size () >= 2)
            {
              sort (tempParents.begin (), tempParents.end (),Compare_Average_Hop);
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
      NS_LOG_UNCOND ("S_2 Node:" << nodeMinHop.m_head->GetId ()); // (Temporary)
      (this)->AddNode (nodeMinHop.m_head);
      (this)->SetHopCount (nodeMinHop.m_head->GetId (), nodeMinHop.m_avgHopCount);
      NS_LOG_UNCOND ("Avg Hop Count:" << nodeMinHop.m_avgHopCount); // (Temporary)
      GraphNode tempNode = edgesForS[nodeMinHop.m_head->GetId ()];
      NS_LOG_UNCOND ("Edge 1 :" << tempNode.m_parents[0]->m_head->GetId ()); // (Temporary)
      NS_LOG_UNCOND ("Edge 2 :" << tempNode.m_parents[1]->m_head->GetId ()); // (Temporary)
      (this)->AddEdge (tempNode.m_parents[0]->m_head->GetId (), nodeMinHop.m_head->GetId ());
      (this)->AddEdge (tempNode.m_parents[1]->m_head->GetId (), nodeMinHop.m_head->GetId ());
      (this)->SetReliability(nodeMinHop.m_head->GetId ());
    }
  else if (S_1)
    {
      NS_LOG_UNCOND ("S_1 Node:" << nodeMaxoutgoingEdges.m_head->GetId ()); // (Temporary)
      (this)->AddNode (nodeMaxoutgoingEdges.m_head);
      (this)->SetHopCount (nodeMaxoutgoingEdges.m_head->GetId (), nodeMaxoutgoingEdges.m_avgHopCount);
      NS_LOG_UNCOND ("Avg Hop Count:" << nodeMaxoutgoingEdges.m_avgHopCount); // (Temporary)
      GraphNode tempNode = edgesForS[nodeMaxoutgoingEdges.m_head->GetId ()];
      NS_LOG_UNCOND ("Edge only 1 :" << tempNode.m_parents[0]->m_head->GetId ()); // (Temporary)
      (this)->AddEdge (tempNode.m_parents[0]->m_head->GetId (), nodeMaxoutgoingEdges.m_head->GetId ());
    }

  if ((this)->GetNumofNodes () < G->GetNumofNodes ())
    {
      (this)->BroadcastGraph (G, edgesForS);
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
      vector<Ptr<GraphNode> > tempNodeList = G->m_graphNodeMap[it->first].m_neighbors;
      tempEdgesOfS[it->first].m_neighbors = tempNodeList;
      tempEdgesOfS[it->first].m_parents = G->m_graphNodeMap[it->first].m_parents;

      while (!tempNodeList.empty ())
        {
          uint32_t nextNode = tempNodeList.back ()->m_head->GetId ();
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
  Ptr<Node> gateWay = G->m_gateway->m_head;
  S->AddNode(gateWay);
  for (vector<Ptr<GraphNode>>::const_iterator it = G->m_accessPoints.begin ();
               it != G->m_accessPoints.end (); ++it)
            {
                Ptr<IsaGraph> G_AP = CreateObject<IsaGraph> ();                      ///< Downlink graph creation for access points
                G_AP->AddNode(gateWay);
                G_AP->AddGateway(gateWay->GetId());                               // Set gate way of the Downlink graph
                Ptr<Node> accessPoint = it->operator ->()->m_head;
                S->AddNode(accessPoint);
                G_AP->AddNode(accessPoint);
                G_AP->AddAccessPoint(accessPoint->GetId());                       // Set access point of the Downlink graph
                G_AP->AddEdge(gateWay->GetId(), accessPoint->GetId());            // adding edge e(g,i)
                G_AP->SetHopCount(accessPoint->GetId(), 1);
                downlinkGraphs_G[accessPoint->GetId()] = G_AP;                      // Set Gi (Downlink graph for ith node) - For Access points
            }
  // ***** end ***** create Downlink graphs for all access points of the graph G

  map <uint32_t, GraphNode> edgesForS;                ///< vector to determine the number of parents of the respective node

  while (S->GetNumofNodes() != G->GetNumofNodes ())      // while S != V (Nodes of the Graph G)
    {
      edgesForS = S->UpdateSVector (G, G->m_graphNodeMap);       // Nodes and edges that need to be consider for the Downlink graph creation

      GraphNode nodeMinHopPv;           ///< Graph Node with minimum hops from the gateway and satisfying the three conditions
      GraphNode nodeMinHopS_2;           ///< Graph Node with minimum hops from the gateway and satisfying the three conditions
      GraphNode nodeMinHopS_1;          ///< Graph Node with minimum hops from the gateway and which not satisfying the three conditions

      // initially set the hop counts as the maximum possible +1
      uint32_t notPossibleHopCount = G->GetNumofNodes ()+1;
      nodeMinHopS_2.m_avgHopCount = notPossibleHopCount;
      nodeMinHopPv.m_avgHopCount = notPossibleHopCount;
      nodeMinHopS_1.m_avgHopCount = notPossibleHopCount;

      for (map<uint32_t, GraphNode>::const_iterator it = edgesForS.begin ();
             it != edgesForS.end (); ++it)
          {
            if (S->m_graphNodeMap.count (it->first) != 1)   // check whether node is already in the explored node list or not
              {
                GraphNode tempNode;                     // Node which having two parents considering at the moment (temporary Node v)
                vector<Ptr<GraphNode>> tempParents;     // temporary vector for  all the parents of the tempNode in S (explored Nodes)
                tempNode.m_head = it->second.m_head;

                for (uint32_t i = 0; i < (it->second).m_parents.size (); ++i)
                  {
                    uint32_t nextNode = (it->second).m_parents[i].operator -> ()->m_head->GetId ();
                    if (S->m_graphNodeMap.count (nextNode))
                      {
                        tempParents.push_back (&S->m_graphNodeMap[nextNode]);
                      }
                  }

                if (tempParents.size () >= 2)              //nodes having at least two edges from S
                  {
                    //iterate over all the edges to find the edge pair that need to add to the Downlink graph of the respective node
                    vector<Ptr<GraphNode>> tempNew = tempParents;               ///< copy of temParents vector

                    while (!tempNew.empty())
                      {
                        Ptr<GraphNode> u1 = tempNew.back();                // get the first parent of the node v
                        uint32_t u1Id = u1->m_head->GetId();
                        tempNew.pop_back();

                        tempNode.m_parents[0] = u1->m_head;       // add parent u1 to the parents list

                        for (vector<Ptr<GraphNode>>::const_iterator it = tempNew.begin ();
                             it != tempNew.end (); ++it)
                          {
                            Ptr<GraphNode> u2 = it->operator *();         //get the next parent of the node v
                            uint32_t u2Id = u2->m_head->GetId();
                            tempNode.m_parents[1] = u2->m_head;
                            tempNode.m_avgHopCount = (downlinkGraphs_G[u1Id]->GetHopCount(u1Id) + downlinkGraphs_G[u2Id]->GetHopCount(u2Id))/2;

                            if(G->C1Condition(u1, u2) && (G->C2Condition(u1, u2, downlinkGraphs_G)
                                || G->C3Condition(u1, u2, downlinkGraphs_G)))
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
                          }
                      }
                    // end of iteration over all edge pairs of v
                  }
                else if (tempParents.size () == 1)              //nodes having one edge from S
                  {
                    Ptr<GraphNode> u1 = tempParents.back();                // get the first parent of the node v
                    uint32_t u1Id = u1->m_head->GetId();

                    if(nodeMinHopS_1.m_avgHopCount>downlinkGraphs_G[u1Id]->GetHopCount(u1Id))
                      {
                        nodeMinHopS_1.m_avgHopCount = downlinkGraphs_G[u1Id]->GetHopCount(u1Id);    // setting the average hop count of parents to S_1 Node
                      }
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
          downlinkGraphs_G = ConstructDownlinkGraphs(G,&nodeMinHopS_2, downlinkGraphs_G);
          S->AddNode(nodeMinHopS_2.m_head);
        }
      else if(nodeMinHopS_1.m_avgHopCount < notPossibleHopCount)
        {
          nodeMinHopS_1.m_avgHopCount++;                  // hop count of u1 considered for above algorithm, but hv is +1 to it.
          uint32_t nodeId = nodeMinHopS_1.m_head->GetId();
          uint32_t nodeParentId = nodeMinHopS_1.m_parents[0]->m_head->GetId();

          Ptr<IsaGraph> G_v = CreateObject<IsaGraph> ();                      ///< Downlink graph creation for the selected Node
          G_v = downlinkGraphs_G[nodeParentId];
          G_v->AddGraphNode(nodeMinHopS_1);             // add graph node S_1
          G_v->AddEdge(nodeParentId, nodeId);           // Adding edge of u1 and v
          downlinkGraphs_G[nodeId] = G_v;               // Add Downlink graph to the graph list
          S->AddNode(nodeMinHopS_1.m_head);
        }

    } // end of while loop for S != V

  return downlinkGraphs_G;
}

map <uint32_t, Ptr<IsaGraph>> IsaGraph::ConstructDownlinkGraphs (Ptr<IsaGraph> G, Ptr<GraphNode> v, map <uint32_t, Ptr<IsaGraph>> downlinkGraphs)
{
  Ptr<IsaGraph> Gv = CreateObject<IsaGraph> ();                      ///< initial downlink graph (Gv) creation with v

  Ptr<GraphNode> u1 = v->m_parents[0];            // first parent of the node v
  Ptr<GraphNode> u2 = v->m_parents[1];            // second parent of the node v

  uint32_t vId = v->m_head->GetId();              // Id of the Node v
  uint32_t u1Id = u1->m_head->GetId();            // Id of the first parent of v
  uint32_t u2Id = u2->m_head->GetId();            // Id of the second parent of v

  vector<Ptr<GraphNode>> u2Parents = downlinkGraphs[u2Id]->GetGraphNode(u2Id).m_parents;    // parents of node u1 in it's respective Downlink graph
  vector<Ptr<GraphNode>> u1Parents = downlinkGraphs[u1Id]->GetGraphNode(u1Id).m_parents;    // parents of node u1 in it's respective Downlink graph
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
  if ((this)->C1Condition(u1, u2) && (this)->C2Condition(u1, u2, G))
    {
      if (count(u2Parents.begin(),u2Parents.end(),u1->m_head) >= 1)
        {
          /*  Check u1-> u2 is in Gu2
           *
           *           u1<--->u2 (Gu2)
           *            |     |
           *             > v <
           */

          Gv = downlinkGraphs[u2Id];
          Gv->AddGraphNode(v.operator *());         //Add Node v
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
          Gv = downlinkGraphs[u1Id];
          Gv->AddGraphNode(v.operator *());         //Add Node v
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
  else if ((this)->C1Condition(u1, u2) && (this)->C3Condition(u1, u2,downlinkGraphs))
    {
      for (vector<Ptr<GraphNode>>::const_iterator it1 = u2Parents.begin ();
                it1 != u2Parents.end (); ++it1)
        {
          vector<Ptr<GraphNode>> neighborsOfU2Parent = it1->operator ->()->m_neighbors;
          for (vector<Ptr<GraphNode>>::const_iterator it2 = neighborsOfU2Parent.begin ();
                          it2 != neighborsOfU2Parent.end (); ++it2)
            {
              if (it2->operator ->() == u1)
                {
                  Gv = downlinkGraphs[u2Id];
                  Gv->AddNode(u1->m_head);                                    //Add node u1 to Gv
                  Gv->AddEdge(it1->operator ->()->m_head->GetId(), u1Id);     //Add edge parent of u2 with u1
                  Gv->AddGraphNode(v.operator *());                           //Add Node v
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
          for (vector<Ptr<GraphNode>>::const_iterator it1 = u1Parents.begin ();
                                it1 != u1Parents.end (); ++it1)
            {
              vector<Ptr<GraphNode>> neighborsOfU1Parent = it1->operator ->()->m_neighbors;
              for (vector<Ptr<GraphNode>>::const_iterator it2 = neighborsOfU1Parent.begin ();
                              it2 != neighborsOfU1Parent.end (); ++it2)
                {
                  if (it2->operator ->() == u2)
                    {
                      Gv = downlinkGraphs[u1Id];
                      Gv->AddNode(u2->m_head);                                    //Add node u1 to Gv
                      Gv->AddEdge(it1->operator ->()->m_head->GetId(), u2Id);     //Add edge parent of u2 with u1
                      Gv->AddGraphNode(v.operator *());                           //Add Node v
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
                               G->m_graphNodeMap[u1Id].m_neighbors.end (), &(G->m_graphNodeMap[u2Id])) >= 1);
      bool edgeU2toU1 = (count(G->m_graphNodeMap[u2Id].m_neighbors.begin(),
                               G->m_graphNodeMap[u2Id].m_neighbors.end (), &(G->m_graphNodeMap[u1Id])) >= 1);
      if(edgeU1toU2 && edgeU2toU1)
        {
          if(u1HopCount < u2HopCount)
            {
              Gv = downlinkGraphs[u1Id];
              Gv->AddNode(u2->m_head);                  //Add node u1 to Gv
            }
          else
            {
              Gv = downlinkGraphs[u2Id];
              Gv->AddNode(u1->m_head);                  //Add node u1 to Gv
            }
          Gv->AddGraphNode(v.operator *());         //Add Node v
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
              Gv = downlinkGraphs[u1Id];
              Gv->AddGraphNode(v.operator *());         //Add Node v
              Gv->AddEdge(u1Id, vId);                   //Add edge u1 ---> v
            }
          else
            {
              Gv = downlinkGraphs[u2Id];
              Gv->AddGraphNode(v.operator *());         //Add Node v
              Gv->AddEdge(u2Id, vId);                   //Add edge u2 ---> v
            }
          downlinkGraphs[vId] = Gv;
        }
      else if(edgeU1toU2)
        {
          Gv = downlinkGraphs[u1Id];
          Gv->AddNode(u2->m_head);                                    //Add node u1 to Gv
          Gv->AddGraphNode(v.operator *());                           //Add Node v
          Gv->AddEdge(u1Id, vId);                   //Add edge u1 ---> v
          Gv->AddEdge(u2Id, vId);                   //Add edge u2 ---> v
          Gv->AddEdge(u1Id, u2Id);                  //Add edge u1 ---> u2
          downlinkGraphs[vId] = Gv;
        }
      else
        {
          Gv = downlinkGraphs[u2Id];
          Gv->AddNode(u1->m_head);                                    //Add node u1 to Gv
          Gv->AddGraphNode(v.operator *());                           //Add Node v
          Gv->AddEdge(u1Id, vId);                   //Add edge u1 ---> v
          Gv->AddEdge(u2Id, vId);                   //Add edge u2 ---> v
          Gv->AddEdge(u2Id, u1Id);                  //Add edge u2 ---> u1
          downlinkGraphs[vId] = Gv;
        }
    }
  return downlinkGraphs;
}

//*- C1 v has at least two parents u1, u2, and they form a cycle.
bool IsaGraph::C1Condition (Ptr<GraphNode> u1, Ptr<GraphNode> u2)
{
  NS_LOG_FUNCTION (this);

  uint32_t u1Id = u1->m_head->GetId();        ///< node Id of the first parent of the respective node v
  uint32_t u2Id = u2->m_head->GetId();        ///< node Id of the second parent of the respective node v

  // check u1 and u2 not representing the same node
  if (u1Id != u2Id)
    {
      /*  check (u2 is a neighbor of u1) and (u2 is a parent of u1)
       *
       *           u2<---->(u1)
       *
       */
      if (count(u1->m_neighbors.begin(),u1->m_neighbors.end(),u2->m_head) >= 1)
        {
          if (count(u1->m_parents.begin(),u1->m_parents.end(),u2->m_head) >= 1)
            {
              return true;
            }
        }
  }

  return false;
}

//*- C2 u1 is u2’s parent in u2’s "local" Downlink graph (Node and it's immediate parents)
bool IsaGraph::C2Condition (Ptr<GraphNode> u1, Ptr<GraphNode> u2, Ptr<IsaGraph> G)
{
  NS_LOG_FUNCTION (this);

  uint32_t u1Id = u1->m_head->GetId();        ///< node Id of the first parent of the respective node v
  uint32_t u2Id = u2->m_head->GetId();        ///< node Id of the second parent of the respective node v

  // check u1 and u2 not representing the same node
  if (u1Id != u2Id)
    {
      /*  check u1 is a parent of u2's local Downlink graph in G
       *
       *           x1<---->x2 (check if x1 of x2 is u1)
       *            |      |
       *             > u2 <
       */
      vector<Ptr<Node>> u2Parents = G->GetGraphNode(u2Id).m_parents;

      if (count(u2Parents.begin(),u2Parents.end(),u1->m_head) >= 1)
        {
          return true;
        }
    }

  return false;
}

//*- C3 u2 (u1) has at least one parent from the cycle in Gu1 (Gu2)
bool IsaGraph::C3Condition (Ptr<GraphNode> u1, Ptr<GraphNode> u2, map <uint32_t, Ptr<IsaGraph>> downlinkGraphs)
{
  NS_LOG_FUNCTION (this);

  uint32_t u1Id = u1->m_head->GetId();       ///< node Id of the first parent of the respective node v
  uint32_t u2Id = u2->m_head->GetId();       ///< node Id of the second parent of the respective node v

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
      vector<Ptr<GraphNode>> u2Parents = downlinkGraphs[u2Id]->GetGraphNode(u2Id).m_parents;

      for (vector<Ptr<GraphNode>>::const_iterator it = u2Parents.begin ();
                it != u2Parents.end (); ++it)
        {
          if (count(u1->m_parents.begin(),u1->m_parents.end(),it->operator ->()) >= 1)
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
      vector<Ptr<GraphNode>> u1Parents = downlinkGraphs[u1Id]->GetGraphNode(u1Id).m_parents;

      for (vector<Ptr<GraphNode>>::const_iterator it = u1Parents.begin ();
                it != u1Parents.end (); ++it)
        {
          if (count(u2->m_parents.begin(),u2->m_parents.end(),it->operator ->()) >= 1)
            {
              return true;
            }
        }
    }

  return false;
}

} // namespace ns3
