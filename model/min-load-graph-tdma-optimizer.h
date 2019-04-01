/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
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
 * Author:   Rajith Madduma Bandarage <rajith.maddumabandar@ucalgary.ca>
 */


#ifndef MIN_LOAD_GRAPH_TDMA_OPTIMIZER
#define MIN_LOAD_GRAPH_TDMA_OPTIMIZER

#include "ns3/tdma-optimizer-base.h"
#include "ns3/ptr.h"
#include "ns3/isa-graph.h"

using namespace std;

namespace ns3{

  /** Defining Node information structure for graph routing.
   *
   */
typedef struct
{
  Ptr<Node> m_head;                  ///< Pointer for the node.
  double m_normalizedLoad;        ///< LAMDA - temporary normalized load
  uint32_t m_lastHop;    ///< (H) - track the last hop devices
//  double m_normalizedBackupLoad;        ///< LAMDA - temporary normalized load
  vector<uint32_t> m_backupPath;      ///< (P) - backup paths
  double m_flowRate;    ///< flow rate of the node
  double m_initialBatteryEnergy;  ///< initial battery energy value

} MinLoadVertex;

const double INF_DOUBLE = std::numeric_limits<double>::max();

/**
 * \class MinLoadGraphTdmaOptimzer
 *
 * \brief Wu's Greedy Heuristic TDMA scheduler.
 *
 * This class is based on a variant of the breadth first minimum hop search algorithm.
 *
 */
class MinLoadGraphTdmaOptimzer : public TdmaOptimizerBase
{
public:

  static TypeId GetTypeId (void);

  MinLoadGraphTdmaOptimzer ();

  ~MinLoadGraphTdmaOptimzer ();

  /** Perform some initial calculations required by the optimizer.
   *   @param c node container with all nodes
   *   @param propModel the propagation loss model
   */

  void SetupOptimization (NodeContainer c, Ptr<PropagationLossModel> propModel);

  /** Solve for the packet flows using a minimum hop breadth first search algorithm.
   * @return packetFlows A matrix of packet flows between nodes.
   */
  virtual std::vector< std::vector< int > > SolveTdma (void);

  /** Set Edge Weights to Han's Graph Algorithms (Not Required)
   *
   * @param edgeWeight information of the edge. i.e., source and destination node ID pairs
   */
  void SetEdgeWeights (vector<pair<uint32_t,uint32_t>> edgeWeight);

private:

  /** Create a graph from the container information..
     *
     * @param c node container with all nodes
     */
  void GraphCreation(NodeContainer c);

  map<uint32_t, MinLoadVertex>  MinLoadGraphRoute(map<uint32_t, MinLoadVertex> vertexVect, uint32_t routeIndexIt, uint32_t src, uint32_t dst);

  map<uint32_t, MinLoadVertex>  MinLoadSourceRoute(map<uint32_t, MinLoadVertex> vertexVect, uint32_t routeIndexIt, uint32_t src, uint32_t dst);

  std::map<uint32_t, MinLoadVertex> m_vertexVector;   ///< vertex information for the algorithm; map<nodeID, MinLoadVertex>
  std::map<uint32_t, MinLoadVertex> m_rawVertex;   ///< vertex information for the algorithm; map<nodeID, MinLoadVertex> with normLoad INF
  std::map<uint32_t, double> m_normalizedLoadMap;   ///< temporary normalized load vector; map<m_routeIndexIt, Load>
  uint32_t m_routeIndexIt;        //< route index iterator
  matrixUInt_t m_routeIndexMat;     //< route index iterator is stored in a matrix. (i -> j)
  map<uint32_t, vector<uint32_t>> m_primaryPath;  //< Last hop vector; map<m_routeIndexIt, map<v, u>>
  map<uint32_t, vector<uint32_t>> m_backUpPath;  //< Last hop vector; map<m_routeIndexIt, map<v, u>>

};


}
#endif /* MIN_LOAD_GRAPH_TMDA_OPTIMIZER */
