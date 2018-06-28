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
 *
 */

#ifndef ISA_GRAPH_H
#define ISA_GRAPH_H

#include "ns3/core-module.h"
#include "ns3/isa100-11a-module.h"
#include <algorithm>

using namespace std;

namespace ns3 {

/**
 * Graph model of ISA100.11a graph routing.
 *- ISAGraph Class is the graph model to use for the ISA100.11a network routing.
 *
 */

/** Defining Node information structure for graph routing.
 *
 */
typedef struct GraphNodeT
{
  Ptr<Node> m_head;                   ///< Pointer for the node.
  double m_avgHopCount;               ///< Average hop count from the Gateway, Calculated by Average of parents +1.
  vector <Ptr<GraphNodeT> > m_neighbors;    ///< Neighbors of the head node.
  vector <Ptr<GraphNodeT> > m_parents;      ///< Parents of the head node.
  bool m_reliability;                 ///< Reliability status of the head node in the graph.
} GraphNode;

/** Defining structure for selected edges of downlink graph of v
 *
 */
typedef struct
{
  Ptr<GraphNode> m_u1;                     ///< Pointer for the selected node first parent.
  Ptr<GraphNode> m_u2;                     ///< Pointer for the selected node second parent.
  double m_avgHopCount;               ///< Average hop count from the Gateway, Calculated by pair vlaue of parents +1.
  bool m_Sr;                          ///< Whether to include in reliable selection
} DownlinkEdgesForSelection;

/** Comparator to support graph Nodes sorting based on average hop count
 *
 */
bool Compare_Average_Hop (const GraphNode & e1, const GraphNode & e2)
{
  if (e1.m_avgHopCount != e2.m_avgHopCount)
    {
      return (e1.m_avgHopCount < e2.m_avgHopCount);
    }
  return ((e1.m_head < e2.m_head) && (e1.m_neighbors < e2.m_neighbors)
      && (e1.m_parents < e2.m_parents) && (e1.m_reliability < e2.m_reliability));
}

class IsaGraph;

/** Class that stores the ISA-100 graph information.
 * - Includes reliable broadcasting, uplink and downlink graph creation.
 */
class IsaGraph : public Node
{
public:
  static TypeId GetTypeId (void);

  IsaGraph ();

  IsaGraph (NodeContainer c);

  virtual ~IsaGraph ();

  /** Add/Get edges of the graph
   * GetEdges return all the neighbors of src Node
   *
   * @param src Pointer for the node/ Edge initiating node pointer
   * @param dest Edge ending node pointer
   */
  void AddEdge (uint32_t src, uint32_t dest);
  vector<Ptr<GraphNode> > GetEdges (uint32_t src);

  /** Add/ Get/ Remove node of the graph
   * GetGraphSrcNode return the pointer for the Node with ID = id
   * GetGraphNode return the graph Node struct data for the Node with ID = id
   *
   * @param src Pointer for the node/ Edge initiating node pointer
   * @param dest Edge ending node pointer
   * @param id ID of the Node
   */
  void AddGraphNode (GraphNode graphNode);
  void AddNode(Ptr<Node> src);
  Ptr<Node> GetGraphNodeHead (uint32_t id);
  GraphNode GetGraphNode (uint32_t id);
  void RemoveGraphNode(uint32_t id);

  /** Set/ Get gateway of the graph
   *- return the gateway graph nodes
   *
   * @param id ID of the Node
   */
  void AddGateway(uint32_t id);
  Ptr<GraphNode> GetGetway (void);

  /** Set/ Get Access point nodes of the graph
   *- return vector of access point graph nodes
   *
   * @param id ID of the Node
   */
  void AddAccessPoint(uint32_t id);
  vector<Ptr<GraphNode>> GetAccessPoints (void);

  /** Get number of nodes in graph
   *
   * @param src Pointer for the node/ Edge initiating node pointer
   * @param dest Edge ending node pointer
   */
  uint32_t GetNumofNodes (void);

  /** Set reliability status of the Node
   *- In broadcast graph if a node has at least two parents reliability is true
   *- In uplink graph if a node has two neighbors reliability is true
   *
   * @param id ID of the Node
   */
  void SetReliability (uint32_t id);

  /** ~~~~~~~~~~~[Temporary]~~~~~~~~~~~ Print the graph
   *
   */
  void PrintGraph ();

  /** Flip the edges of the graph
   * Direction of the edge will be flipped and parents and neighbors will be updated.
   * Average hop count will remain same.
   *
   */
  Ptr<IsaGraph> FlipEdge ();

  /** Set/Get hop count of a node
   *
   * @param id ID of the Node
   * @param hopCount Average hop count of the Node
   */
  void SetHopCount (uint32_t id, double hopCount);
  double GetHopCount (uint32_t id);

  /** Reliable broadcast graph creation
   *
   * @param G graph to create the reliable broadcast graph
   * @param edgesForS map of node with
   */
  bool ReliableBroadcastGraph (Ptr<IsaGraph> G);
  bool BroadcastGraph (Ptr<IsaGraph> G, map <uint32_t, GraphNode> edgesForS);

  /** Update the selection vector for reliable graph creation
   *
   * @param G graph to create the reliable broadcast graph
   * @param edgesForS graph nodes for the selection criteria of reliable graphs
   */
  map <uint32_t, GraphNode> UpdateSVector (Ptr<IsaGraph> G, map <uint32_t, GraphNode> edgesForS);

  /** Reliable uplink graph creation
   *
   * @param G graph to create the reliable uplink graph
   */
  bool ReliableUplinkGraph (Ptr<IsaGraph> G);

  /** Reliable downlink graphs creation
   *- return pointers to the downlink graphs for each node (map with their ids)
   *
   * @param G graph to create the reliable downlink graphs
   * @param edgesForS graph nodes for the selection criteria of reliable graphs
   * @param downlinkGraphs List of reliable downlink graphs
   * @param v node that required to construct the downlink graph
   */
  map <uint32_t, Ptr<IsaGraph>> ReliableDownlinkGraphs (Ptr<IsaGraph> G);
  map <uint32_t, Ptr<IsaGraph>> ConstructDownlinkGraphs (Ptr<IsaGraph> G, Ptr<GraphNode> v, map <uint32_t, Ptr<IsaGraph>> downlinkGraphs);

  /** return selected edges for respective downlink graph if all three conditions are satisfied
   *- C1 v has at least two parents u1, u2, and they form a cycle.
   *- C2 u1 is u2’s parent in u2’s "local" downlink graph (Node and it's immediate parents).
   *- C3 u2 (u1) has at least one parent from the cycle in Gu1 (Gu2)
   *
   * @param G graph to create the reliable downlink graphs
   * @param downlinkGraphs List of reliable downlink graphs
   * @param tempParents temporary vector to store the parents of a selected node of EdgesOfS
   */
  bool C1Condition (Ptr<GraphNode> u1, Ptr<GraphNode> u2);
  bool C2Condition (Ptr<GraphNode> u1, Ptr<GraphNode> u2, Ptr<IsaGraph> G);
  bool C3Condition (Ptr<GraphNode> u1, Ptr<GraphNode> u2, map <uint32_t, Ptr<IsaGraph>> downlinkGraphs);

private:
  map <uint32_t, GraphNode> m_graphNodeMap;       ///< Map of graph nodes with their node IDs.
  vector<Ptr<GraphNode>> m_accessPoints;          ///< Access point nodes of the graph
  Ptr<GraphNode> m_gateway;                       ///< gateway of the graph

};

} // namespace ns3

#endif /* ISA100_11A_HELPER_H */
