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
typedef struct
{
  Ptr<Node> m_head;                   ///< Pointer for the node.
  double m_avgHopCount;               ///< Average hop count from the Gateway, Calculated by Average of parents +1.
  vector <Ptr<Node> > m_neighbors;    ///< Neighbors of the head node.
  vector <Ptr<Node> > m_parents;      ///< Parents of the head node.
  bool m_reliability;                 ///< Reliability status of the head node in the graph.
} GraphNode;

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
  void AddEdge (Ptr<Node> src, Ptr<Node> dest);
  vector<Ptr<Node> > GetEdges (Ptr<Node> src);

  /** Add/Get node of the graph
   * GetGraphSrcNode return the pointer for the Node with ID = id
   * GetGraphNode return the graph Node struct data for the Node with ID = id
   *
   * @param src Pointer for the node/ Edge initiating node pointer
   * @param dest Edge ending node pointer
   * @param id ID of the Node
   */
  void AddGraphNode (Ptr<Node> src);
  Ptr<Node> GetGraphSrcNode (uint32_t id);
  GraphNode GetGraphNode (uint32_t id);

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

  /** Relaible broadcast graph creation
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

  /** Relaible uplink graph creation
   *
   * @param G graph to create the reliable uplink graph
   */
  bool ReliableUplinkGraph (Ptr<IsaGraph> G);

private:
  map <uint32_t, GraphNode> m_graphNodeMap;       ///< Map of graph nodes with their node IDs.

};

} // namespace ns3

#endif /* ISA100_11A_HELPER_H */
