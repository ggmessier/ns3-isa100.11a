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

#include "ns3/isa-graph.h"
#include "ns3/log.h"
#include "ns3/type-id.h"
#include <algorithm>

NS_LOG_COMPONENT_DEFINE ("IsaGraph");

using namespace std;

namespace ns3{

NS_OBJECT_ENSURE_REGISTERED (IsaGraph);

bool CompareAverageHop (const GraphNode & e1, const GraphNode & e2)
{
  if (e1.m_avgHopCount < e2.m_avgHopCount)
    {
      return true;
    }
  else if(e1.m_avgHopCount > e2.m_avgHopCount)
    {
      return false;
    }
  else if(e1.m_weight > e2.m_weight)
    {
      NS_LOG_UNCOND("e1.m_weight > e2.m_weight");
      return true;
    }
  return false;
//  return ((e1.m_avgHopCount < e2.m_avgHopCount) || ((e1.m_avgHopCount == e2.m_avgHopCount) && (e1.m_weight >= e2.m_weight)));
//  return (e1.m_avgHopCount < e2.m_avgHopCount);
};

TypeId IsaGraph::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::IsaGraph")
    .SetParent<Object> ()
    .AddConstructor<IsaGraph> ()

    .AddAttribute ("minLoad", "flag to identify whether minimum load or normal graph.",
                   BooleanValue (false),
                   MakeBooleanAccessor (&IsaGraph::m_minLoadGraph),
                   MakeBooleanChecker ())
  ;
  return tid;
}

IsaGraph::IsaGraph ()
{
  NS_LOG_FUNCTION (this);
  m_gateway = 0;
  m_graphID = 0;
//  m_minLoadGraph = false;
}

IsaGraph::IsaGraph (NodeContainer c)
{
  NS_LOG_FUNCTION (this);
  m_gateway = 0;
  m_graphID = 0;

  for (NodeContainer::Iterator i = c.Begin (); i != c.End (); ++i)
    {
      uint32_t nodeId = i->operator -> ()->GetId ();
      (this)->m_graphNodeMap[nodeId].m_head = i->operator -> ();
      (this)->m_graphNodeMap[nodeId].m_avgHopCount = 0;
      (this)->m_graphNodeMap[nodeId].m_reliability = false;
      (this)->m_graphNodeMap[nodeId].m_numTimeSlots = 25;
      (this)->m_graphNodeMap[nodeId].m_weight = 0;
    }
}

IsaGraph::~IsaGraph ()
{
  NS_LOG_FUNCTION (this);
}

void IsaGraph::SetGraphId (uint32_t id)
{
  NS_LOG_FUNCTION (this);
  (this)->m_graphID = id;
}

uint32_t IsaGraph::GetGraphId ()
{
  NS_LOG_FUNCTION (this);
  return (this)->m_graphID;
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
  (this)->m_graphNodeMap[nodeId].m_weight = 0;
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

Ptr<Node> IsaGraph::GetGateway (void)
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

void IsaGraph::SetTimeSlots (uint32_t id, uint32_t timeSlots)
{
  NS_LOG_FUNCTION (this);
  (this)->m_graphNodeMap[id].m_numTimeSlots = timeSlots;
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

void IsaGraph::SetWeight (uint32_t id, uint32_t weight)
{
  NS_LOG_FUNCTION (this);
  (this)->m_graphNodeMap[id].m_weight = weight;
}

uint32_t IsaGraph::GetWeight (uint32_t id)
{
  NS_LOG_FUNCTION (this);
  return (this)->m_graphNodeMap[id].m_weight;
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

} // namespace ns3
