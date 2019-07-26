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

NS_LOG_COMPONENT_DEFINE ("Isa100BroadcastGraph");

using namespace std;

namespace ns3 {

bool IsaGraph::ReliableBroadcastGraph (Ptr<IsaGraph> G)
{
  NS_LOG_FUNCTION (this);

  // Grpah itself has considered S and it will initially include the getway and the access points
  bool S_2;                             ///< vector for nodes have at least two edges from VB
  bool S_1;                             ///< vector for nodes have one edge from VB
  GraphNode nodeMinHop;                 ///< Graph Node with minimum hops from the gateway
  GraphNode nodeMaxoutgoingEdges;       ///< Graph Node with maximum outgoing edges
  uint32_t maxOutgoingEdges;            ///< Number of outgoing edges of the maximum outgoing edges node

  (this)->SetGraphId (65535);
  /*
   *          S
   *       ------
   *        |  |      // edgesForS is to find all the edges coming from S (traversed nodes) to V-S (need to traverse)
   *        v  v
   *       ------
   *        V -S
   *
   */
  map <uint32_t, GraphNode> edgesForS  = (this)->m_graphNodeMap;

  vector<GraphNode> tempParents;

  while ((this)->GetNumofNodes () < G->GetNumofNodes ())         // S != V
    {
      S_2 = false;
      S_1 = false;
      maxOutgoingEdges = 0;
      nodeMaxoutgoingEdges = (this)->GetGraphNode ((this)->GetGateway ()->GetId ());
      edgesForS = (this)->UpdateSVector (G, edgesForS);

      for (map<uint32_t, GraphNode>::const_iterator it = edgesForS.begin ();
           it != edgesForS.end (); ++it)
        {
          if ((this)->m_graphNodeMap.count (it->first) != 1)
            { //considering only the V - VB nodes
              double hop_count;
              tempParents.clear ();
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
//                  if(tempParents.size () > 2)
//                    {
//                      NS_LOG_UNCOND("tempParents.size () "<<tempParents.size ());
//                      for(int k = 0; k <tempParents.size (); k++)
//                        {
//                          NS_LOG_UNCOND("tempParents info: id: "<<tempParents[k].m_head->GetId());
//                          NS_LOG_UNCOND("tempParents info: hop count: "<<tempParents[k].m_avgHopCount);
//                          NS_LOG_UNCOND("tempParents info: weight: "<<tempParents[k].m_weight);
//                        }
//                    }
                  hop_count = 0.5 * ((this)->m_graphNodeMap[tempParents[0].m_head->GetId ()].m_avgHopCount
                                     + (this)->m_graphNodeMap[tempParents[1].m_head->GetId ()].m_avgHopCount) + 1;
                  edgesForS[(it->first)].m_parents[0] = tempParents[0].m_head;
                  edgesForS[(it->first)].m_parents[1] = tempParents[1].m_head;
                  G->m_graphNodeMap[it->first].m_avgHopCount = hop_count;
                  if (!S_2 || (nodeMinHop.m_avgHopCount > hop_count))
                    {
                      nodeMinHop = G->m_graphNodeMap[it->first];
                      nodeMinHop.m_weight = G->m_graphNodeMap[it->first].m_weight;  //Rajith Added
                      S_2 = true;
                    }
                }
              else if (tempParents.size () == 1 && !S_2)
                {
                  uint32_t outgoingEdges = 0;
                  hop_count = (this)->m_graphNodeMap[tempParents[0].m_head->GetId ()].m_avgHopCount + 1;
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
                          nodeMaxoutgoingEdges.m_weight = G->m_graphNodeMap[it->first].m_weight;  //Rajith Added
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
          (this)->SetWeight (nodeMinHop.m_head->GetId (), nodeMinHop.m_weight);
          GraphNode tempNode = edgesForS[nodeMinHop.m_head->GetId ()];
          (this)->AddEdge (tempNode.m_parents[0]->GetId (), nodeMinHop.m_head->GetId ());
          (this)->AddEdge (tempNode.m_parents[1]->GetId (), nodeMinHop.m_head->GetId ());
          (this)->SetReliability (nodeMinHop.m_head->GetId ());
          if ((this)->GetWeight (tempNode.m_parents[0]->GetId ()) > 0)
            {
              (this)->m_graphNodeMap[tempNode.m_parents[0]->GetId ()].m_weight--;
            }
          if ((this)->GetWeight (tempNode.m_parents[1]->GetId ()) > 0)
            {
              (this)->m_graphNodeMap[tempNode.m_parents[1]->GetId ()].m_weight--;
            }
        }
      else if (S_1)
        {
          (this)->AddNode (nodeMaxoutgoingEdges.m_head);
          (this)->SetHopCount (nodeMaxoutgoingEdges.m_head->GetId (), nodeMaxoutgoingEdges.m_avgHopCount);
          (this)->SetWeight (nodeMaxoutgoingEdges.m_head->GetId (), nodeMaxoutgoingEdges.m_weight);
          GraphNode tempNode = edgesForS[nodeMaxoutgoingEdges.m_head->GetId ()];
          (this)->AddEdge (tempNode.m_parents[0]->GetId (), nodeMaxoutgoingEdges.m_head->GetId ());
          if ((this)->GetWeight (tempNode.m_parents[0]->GetId ()) > 0)
            {
              (this)->m_graphNodeMap[tempNode.m_parents[0]->GetId ()].m_weight--;
            }
        }
    }

  return (S_1 || S_2);
}

} // namespace ns3

