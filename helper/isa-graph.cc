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

vector<Ptr<Node> > IsaGraph::GetEdges (Ptr<Node> src)
{
  NS_LOG_FUNCTION (this);
  return (this)->m_graphNodeMap[src->GetId ()].m_neighbors;
}

void IsaGraph::AddEdge (Ptr<Node> src, Ptr<Node> dest)
{
  NS_LOG_FUNCTION (this);
  (this)->m_graphNodeMap[src->GetId ()].m_neighbors.push_back (dest);
  (this)->m_graphNodeMap[dest->GetId ()].m_parents.push_back (src);
}

void IsaGraph::AddGraphNode (Ptr<Node> src)
{
  NS_LOG_FUNCTION (this);
  uint32_t nodeId = src->GetId ();
  (this)->m_graphNodeMap[nodeId].m_head = src;
  (this)->m_graphNodeMap[nodeId].m_avgHopCount = 0;
}

uint32_t IsaGraph::GetNumofNodes (void)
{
  NS_LOG_FUNCTION (this);
  return (this)->m_graphNodeMap.size ();
}

Ptr<Node> IsaGraph::GetGraphSrcNode (uint32_t id)
{
  NS_LOG_FUNCTION (this);
  return (this)->m_graphNodeMap[id].m_head;
}

GraphNode IsaGraph::GetGraphNode (uint32_t id)
{
  NS_LOG_FUNCTION (this);
  return (this)->m_graphNodeMap[id];
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
      vector<Ptr<Node> > tempNodeList = it->second.m_neighbors;

      while (!tempNodeList.empty ())
        {
          NS_LOG_UNCOND ("-> " << tempNodeList.back ()->GetId ());
          tempNodeList.pop_back ();
        }
      NS_LOG_UNCOND ("\n");

      tempNodeList = it->second.m_parents;

      while (!tempNodeList.empty ())
        {
          NS_LOG_UNCOND ("-> " << tempNodeList.back ()->GetId ());
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

  nodeMaxoutgoingEdges = (this)->m_graphNodeMap[0];     // gateway node ID = 0
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
                  NS_LOG_UNCOND("Selected Next Node: "<<nextNode);
                  NS_LOG_UNCOND("count check: "<<(this)->m_graphNodeMap.count (nextNode));
                  tempParents.push_back ((this)->m_graphNodeMap[nextNode]);
                }
            }

          if (tempParents.size () >= 2)
            {
              NS_LOG_UNCOND("tempParents size: "<<tempParents.size ());
              sort (tempParents.begin (), tempParents.end (),Compare_Average_Hop);
              NS_LOG_UNCOND("Here");
              NS_LOG_UNCOND("hop_count 0 :" << tempParents[0].m_head->GetId ());
              NS_LOG_UNCOND("hop_count 1 :" << tempParents[1].m_head->GetId ());
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
      NS_LOG_UNCOND ("S_2 Node:" << nodeMinHop.m_head->GetId ());
      (this)->AddGraphNode (nodeMinHop.m_head);
      (this)->SetHopCount (nodeMinHop.m_head->GetId (), nodeMinHop.m_avgHopCount);
      NS_LOG_UNCOND ("Avg Hop Count:" << nodeMinHop.m_avgHopCount);
      GraphNode tempNode = edgesForS[nodeMinHop.m_head->GetId ()];
      NS_LOG_UNCOND ("Edge 1 :" << tempNode.m_parents[0].operator -> ()->GetId ());
      NS_LOG_UNCOND ("Edge 2 :" << tempNode.m_parents[1].operator -> ()->GetId ());
      (this)->AddEdge (tempNode.m_parents[0], nodeMinHop.m_head);
      (this)->AddEdge (tempNode.m_parents[1], nodeMinHop.m_head);
      (this)->SetReliability(nodeMinHop.m_head->GetId ());
    }
  else if (S_1)
    {
      NS_LOG_UNCOND ("S_1 Node:" << nodeMaxoutgoingEdges.m_head->GetId ());
      (this)->AddGraphNode (nodeMaxoutgoingEdges.m_head);
      (this)->SetHopCount (nodeMaxoutgoingEdges.m_head->GetId (), nodeMaxoutgoingEdges.m_avgHopCount);
      NS_LOG_UNCOND ("Avg Hop Count:" << nodeMaxoutgoingEdges.m_avgHopCount);
      GraphNode tempNode = edgesForS[nodeMaxoutgoingEdges.m_head->GetId ()];
      NS_LOG_UNCOND ("Edge only 1 :" << tempNode.m_parents[0].operator -> ()->GetId ());
      (this)->AddEdge (tempNode.m_parents[0], nodeMaxoutgoingEdges.m_head);
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
  map <uint32_t, GraphNode> tempEdgesofS = edgesForS;

  for (map<uint32_t, GraphNode>::const_iterator it = edgesForS.begin ();
       it != edgesForS.end (); ++it)
    {
      bool newNode = false;
      //update neighbors and parents
      vector<Ptr<Node> > tempNodeList = G->m_graphNodeMap[it->first].m_neighbors;
      tempEdgesofS[it->first].m_neighbors = tempNodeList;
      tempEdgesofS[it->first].m_parents = G->m_graphNodeMap[it->first].m_parents;

      while (!tempNodeList.empty ())
        {
          uint32_t nextNode = tempNodeList.back ()->GetId ();
          //Node will only consider for selection if it's a having new neighbors
          if ((this)->m_graphNodeMap.count (nextNode) != 1)
            {
              tempEdgesofS[nextNode] = G->m_graphNodeMap[nextNode];
              newNode = true;
            }
          tempNodeList.pop_back ();
        }

      /* all nodes do not having new neighbors will be removed from the selection vector.
       * This is to reduce the iteration times in edgesforS vector
       */
      if (!newNode)
        {
          tempEdgesofS.erase (it->first);
        }
    }
  //NS_LOG_UNCOND("tempEdgesofS size: "<<tempEdgesofS.size());

  //update the edgesForS vector
  return tempEdgesofS;
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
      NS_LOG_UNCOND ("Came Here");
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

} // namespace ns3
