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
  (this)->m_graphNodeMap[src].m_neighbors.push_back (&((this)->m_graphNodeMap[dest]));
  (this)->m_graphNodeMap[dest].m_parents.push_back (&((this)->m_graphNodeMap[src]));
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
        (this)->m_graphNodeMap[tempNodeId] .m_parents.push_back(&((this)->m_graphNodeMap[nodeId]));
      }

  // update the neighbors of newly adding graph node parents
  for (vector<Ptr<GraphNode>>::const_iterator it = graphNode.m_parents.begin ();
         it != graphNode.m_parents.end (); ++it)
      {
        tempNodeId= it->operator ->()->m_head->GetId();
        (this)->m_graphNodeMap[tempNodeId] .m_neighbors.push_back(&((this)->m_graphNodeMap[nodeId]));
      }

}

void IsaGraph::RemoveGraphNode(uint32_t id)
{
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

map <uint32_t, Ptr<IsaGraph>> IsaGraph::ReliableDownlinkGraphs (Ptr<IsaGraph> G, map <uint32_t, Ptr<IsaGraph>> downlinkGraphs)
{
  NS_LOG_FUNCTION (this);
  return (this)->DownlinkGraphs(G, (this)->m_graphNodeMap, downlinkGraphs);
}

map <uint32_t, Ptr<IsaGraph>> IsaGraph::DownlinkGraphs (Ptr<IsaGraph> G, map <uint32_t, GraphNode> edgesForS, map <uint32_t, Ptr<IsaGraph>> downlinkGraphs)
{
  NS_LOG_FUNCTION (this);

  bool S_2 = false;                 ///< vector for nodes have at least two edges from VB
  bool S_1 = false;                 ///< vector for nodes have one edge from VB
  GraphNode nodeMinHopSr;           ///< Graph Node with minimum hops from the gateway and satisfying the three conditions
  GraphNode nodeMinHopS_1;          ///< Graph Node with minimum hops from the gateway and which not satisfying the three conditions
  GraphNode nodeMaxoutgoingEdges;   ///< Graph Node with maximum outgoing edges
  uint32_t maxOutgoingEdges = 0;    ///< Number of outgoing edges of the maximum outgoing edges node

  nodeMaxoutgoingEdges = (this)->GetGetway().operator *();
  edgesForS = (this)->UpdateSVector (G, edgesForS);
  bool minHopSr = false;                                      ///< For Sr nodes (S_2 nodes satisfying three conditions)
  bool minHopS_1 = false;                                     ///< For S' (not Sr) nodes
  nodeMinHopSr.m_avgHopCount = 0;
  nodeMinHopS_1.m_avgHopCount = 0;

  for (map<uint32_t, GraphNode>::const_iterator it = edgesForS.begin ();
       it != edgesForS.end (); ++it)
    {
      if ((this)->m_graphNodeMap.count (it->first) != 1)
        { //considering only the V - VB nodes
          double hop_count;
          vector<Ptr<GraphNode>> tempParents;

          for (uint32_t i = 0; i < (it->second).m_parents.size (); ++i)
            {
              uint32_t nextNode = (it->second).m_parents[i].operator -> ()->m_head->GetId ();
              if ((this)->m_graphNodeMap.count (nextNode))
                {
                  tempParents.push_back (&(this)->m_graphNodeMap[nextNode]);
                }
            }

          if (tempParents.size () >= 2)              //nodes having at least two edges from S
            {
              DownlinkEdgesForSelection Edges = (this)->ThreeConditions(G, downlinkGraphs, tempParents);
              if ((Edges.m_Sr) && (!minHopSr || (nodeMinHopSr.m_avgHopCount > Edges.m_avgHopCount)))
                {
                  nodeMinHopSr.m_avgHopCount = Edges.m_avgHopCount;
                  nodeMinHopSr.m_head = it->first;
                  nodeMinHopSr.m_parents[0] = Edges.m_u1->m_head;
                  nodeMinHopSr.m_parents[1] = Edges.m_u2->m_head;
                  minHopSr = true;
                }
              else if (!minHopSr && (!minHopS_1 || (nodeMinHopS_1.m_avgHopCount > Edges.m_avgHopCount)))
                {
                  nodeMinHopS_1.m_avgHopCount = Edges.m_avgHopCount;
                  nodeMinHopS_1.m_head = it->first;
                  nodeMinHopS_1.m_parents[0] = Edges.m_u1->m_head;
                  nodeMinHopS_1.m_parents[1] = Edges.m_u2->m_head;
                  minHopS_1 = true;
                }
            }
          else if (tempParents.size () == 1 && !S_2)              //nodes having one edge from S
            {
              //Code required *********************************
            }
        }
    }

  if (minHopSr)
    {
      (this)->AddNode (nodeMinHopSr.m_head);
      NS_LOG_UNCOND ("S_2 Node:" << nodeMinHopSr.m_head->GetId ());   //temporary
      (this)->SetHopCount (nodeMinHopSr.m_head->GetId (), nodeMinHopSr.m_avgHopCount);
      NS_LOG_UNCOND ("Avg Hop Count:" << nodeMinHopSr.m_avgHopCount); //temporary
      NS_LOG_UNCOND ("Edge 1 :" << nodeMinHopSr.m_parents[0]->m_head->GetId ()); //temporary
      NS_LOG_UNCOND ("Edge 2 :" << nodeMinHopSr.m_parents[1]->m_head->GetId ()); //temporary
      (this)->AddEdge (nodeMinHopSr.m_parents[0]->m_head->GetId (), nodeMinHopSr.m_head->GetId());
      (this)->AddEdge (nodeMinHopSr.m_parents[1]->m_head->GetId (), nodeMinHopSr.m_head->GetId());
      (this)->SetReliability(nodeMinHopSr.m_head->GetId ());
      downlinkGraphs = (this)->ConstructDownlinkGraphs(G, &(nodeMinHopSr), downlinkGraphs);
    }
  else if (minHopS_1)
    {
      (this)->AddNode (nodeMinHopS_1.m_head);
      NS_LOG_UNCOND ("S_2 Node:" << nodeMinHopS_1.m_head->GetId ());   //temporary
      (this)->SetHopCount (nodeMinHopS_1.m_head->GetId (), nodeMinHopS_1.m_avgHopCount);
      NS_LOG_UNCOND ("Avg Hop Count:" << nodeMinHopS_1.m_avgHopCount); //temporary
      NS_LOG_UNCOND ("Edge 1 :" << nodeMinHopS_1.m_parents[0]->m_head->GetId ()); //temporary
      NS_LOG_UNCOND ("Edge 2 :" << nodeMinHopS_1.m_parents[1]->m_head->GetId ()); //temporary
      (this)->AddEdge (nodeMinHopS_1.m_parents[0]->m_head->GetId(), nodeMinHopS_1.m_head->GetId());
      (this)->AddEdge (nodeMinHopS_1.m_parents[1]->m_head->GetId(), nodeMinHopS_1.m_head->GetId());
      (this)->SetReliability(nodeMinHopS_1.m_head->GetId ());
      downlinkGraphs = (this)->ConstructDownlinkGraphs(G, &(nodeMinHopS_1), downlinkGraphs);
    }
  else if (S_1)
    {

      //Code required *********************************

    }

  if ((this)->GetNumofNodes () < G->GetNumofNodes ())
    {
      (this)->DownlinkGraphs (G, edgesForS, downlinkGraphs);
    }

  return downlinkGraphs;
}

map <uint32_t, Ptr<IsaGraph>> IsaGraph::ConstructDownlinkGraphs (Ptr<IsaGraph> G, Ptr<GraphNode> v, map <uint32_t, Ptr<IsaGraph>> downlinkGraphs)
{
  Ptr<IsaGraph> Gv = CreateObject<IsaGraph> ();                      ///< initial downlink graph (Gv) creation with v
  Ptr<IsaGraph> S = CreateObject<IsaGraph> ();                       ///< initiate a graph to store the traversed nodes
  Ptr<IsaGraph> VvminusS = CreateObject<IsaGraph> ();                ///< To track down the nodes in Vv but not in the S

  Gv->AddGraphNode(v.operator *());                                  // add v with e(u1,v) & e(u2,v) edges and calculated average hop count
  uint32_t u1 = v->m_parents[0]->m_head->GetId();                            ///< get u1 node id (u1 and u2 nodes are the immediate parents of node v)
  Ptr<IsaGraph> Gu1 = downlinkGraphs[u1];                            ///< initiate a graph to copy Gu1 reliable graph

  // Updating Gv graph with the Gu1 (reliable downlink graph of node u1)
  for (map<uint32_t, GraphNode>::const_iterator it = Gu1->m_graphNodeMap.begin ();
       it != Gu1->m_graphNodeMap.end (); ++it)
    {
      Gv->AddGraphNode(it->second);
    }

  S->AddGraphNode(Gu1->GetGetway().operator *());                    // adding gateway to the traversed nodes
  S->AddGateway(Gu1->GetGetway()->m_head->GetId());
  S->AddGraphNode(Gv->m_graphNodeMap[u1]);                           // adding u1 to the traversed nodes
  S->AddGraphNode(Gv->GetGraphNode(v->m_head->GetId()));             // adding v to the traversed nodes

  // Adding access points vector to the S vector
  S->m_accessPoints = Gu1->m_accessPoints;                           // this need to be re-consider if two AP clusters are serving the node v

  VvminusS = Gv;                                                     // Vv-S = Gv = g, u1, u2 (if exist), Gu1, v
  VvminusS->RemoveGraphNode(v->m_head->GetId());                     // Vv-S = g, u1, u2 (if exist), Gu1

  // Gu1 - reliable downlink graph of node u1
  if (v->m_parents.size() != 1)       // if u2 is not null
    {
      uint32_t u2 = v->m_parents[1]->m_head->GetId();                         ///< get u2 node id (u1 and u2 nodes are the immediate parents of node v)

      // begin() - e(u1,u2) edge setup
      if(count(G->m_graphNodeMap[u1].m_neighbors.begin(), G->m_graphNodeMap[u1].m_neighbors.end (), &(G->m_graphNodeMap[u2])))
        {
          Gv->AddEdge(u1, u2);
        }

      if(count(G->m_graphNodeMap[u2].m_neighbors.begin(), G->m_graphNodeMap[u2].m_neighbors.end (), &(G->m_graphNodeMap[u1])))
        {
          Gv->AddEdge(u2, u1);
        }
      // end() - e(u1,u2) edge setup

      // add missing nodes from Gu2 to Gv
      Ptr<IsaGraph> Gu2 = downlinkGraphs[u2];

      for (map<uint32_t, GraphNode>::const_iterator it = Gu2->m_graphNodeMap.begin ();
           it != Gu2->m_graphNodeMap.end (); ++it)
        {
          if (!Gv->m_graphNodeMap.count(it->first))
            {
              Gv->AddGraphNode(it->second);                           // Gv = g, u1, u2, Gu1, Gu2, v
              VvminusS->AddGraphNode(it->second);                     // Vv-S = g, u1, u2 (if exist), Gu1, Gu2 (if exist)
            }
        }

      S->AddGraphNode(Gv->m_graphNodeMap[u2]);                        // adding u2 to the traversed nodes
      VvminusS->RemoveGraphNode(u2);                                  // Vv-S = g, u1, Gu1, Gu2 (if exist) [no u2]
    }

  VvminusS->RemoveGraphNode(S->GetGetway()->m_head->GetId());         // Vv-S = u1, Gu1, Gu2 (if exist) [no g, u2]
  VvminusS->RemoveGraphNode(u1);                                      // Vv-S = Gu1, Gu2 (if exist) [no g, u1, u2]

  map <uint32_t, GraphNode> edgesForS;

  while (Gv->GetNumofNodes() > S->GetNumofNodes())            // while nodes of S is not equal to nodes of Gv
  {
    // begin() - check whether Access points vector has two edges to S or not *************************
    uint32_t outgoingEdgesFromAccessPoints = 0;                    ///< outgoing edges from access points vector to traversed nodes
    for (map<uint32_t, GraphNode>::const_iterator it1 = S->m_graphNodeMap.begin ();
               it1 != S->m_graphNodeMap.end (); ++it1)
            {
            for (vector<Ptr<GraphNode>>::const_iterator it2 = G->GetAccessPoints().begin ();
                     it2 != G->GetAccessPoints().end (); ++it2)
                  {
                    // find for node having a edge from access points vector
                    if(count(it2->operator ->()->m_neighbors.begin(),it2->operator ->()->m_neighbors.end(),
                             it1->second.m_head))
                      {
                        ++outgoingEdgesFromAccessPoints;
                      }
                  }
            }

    // if two or more outgoing edges are available from access point vector
    if(outgoingEdgesFromAccessPoints >= 2)
      {
        for (vector<Ptr<GraphNode>>::const_iterator it = G->GetAccessPoints().begin ();
                     it != G->GetAccessPoints().end (); ++it)
                  {
                    S->AddGraphNode(it->operator *());
                  }
        break;
      }
    // end() - check whether Access points vector has two edges to S or not *************************

    edgesForS = S->UpdateSVector (G, edgesForS);

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
               // code is required...
             }
           else if (tempParents.size () == 1 && !S_2)
             {
               // code is required...
             }
         }
     }// end of for loop for edges of S vector
  } // end of while loop


  return downlinkGraphs;
}

DownlinkEdgesForSelection IsaGraph::ThreeConditions (Ptr<IsaGraph> G, map <uint32_t,
                                                     Ptr<IsaGraph>> downlinkGraphs, vector<Ptr<GraphNode>> tempParents)
{
  DownlinkEdgesForSelection Edges;  ///< structure for selected edges of downlink graph of v
  Edges.m_Sr = false;
  Edges.m_avgHopCount = 0;

  bool C1;                          ///< C1 v has at least two parents u1, u2, and they form a cycle.
  bool C2;                          ///< u1 is u2’s parent in u2’s local downlink graph. (u1 and u2 form a directed cycle)
  bool C3;                          ///< u2 (u1) has at least one parent from the cycle in Gu1 (Gu2)

  vector<Ptr<GraphNode>> tempNew = tempParents;   ///< copy of temParents vector
  pair <Ptr<GraphNode>, Ptr<GraphNode>> tempU;        ///< tempU temporary selection for u1 and u2 (immid1ate parents of v)
  uint32_t u1 = 0;                            ///< id of u1 node
  uint32_t u2 = 0;                            ///< id of u2 node

  double pairWiseHopCount = 0;
  double tempPairWiseHopCount = 0;

  while(!tempNew.empty())
    {

      //initially all conditions are zero
      C1 = false;
      C2 = false;
      C3 = false;

      tempU.first = tempNew.back();
      u1 = (tempU.first)->m_head->GetId();
      tempNew.pop_back();

      for (vector<Ptr<GraphNode>>::const_iterator it = tempNew.begin ();
           it != tempNew.end (); ++it)
        {
          tempU.second = it;
          u2 = (tempU.second)->m_head->GetId();
          if (u1 != u2)
            {
              C1 = true;
            }

          if ((count(tempU.first->m_neighbors.begin(),tempU.first->m_neighbors.end(),tempU.second) >= 1) &&
                        (count(tempU.first->m_parents.begin(),tempU.first->m_parents.end(),tempU.second) >= 1))
                      {
                        C2 = true;
                      }

          if ((((downlinkGraphs[u1])->m_graphNodeMap.count(u2)) >= 1) ||
                        (((downlinkGraphs[u2])->m_graphNodeMap.count(u1)) >= 1))
                      {
                        C3 = true;
                      }

          if (C1 && C2 && C3)
            {
              Edges.m_Sr = true;
            }

          tempPairWiseHopCount = ((this)->m_graphNodeMap[u1].m_avgHopCount +
              (this)->m_graphNodeMap[u1].m_avgHopCount)/2;

          if ((pairWiseHopCount==0) || (pairWiseHopCount > tempPairWiseHopCount))
            {
              pairWiseHopCount = tempPairWiseHopCount;
              Edges.m_u1 = tempU.first;
              Edges.m_u2 = tempU.second;
              Edges.m_avgHopCount = pairWiseHopCount + 1;
            }
        }
    }

  return Edges;
}

} // namespace ns3
