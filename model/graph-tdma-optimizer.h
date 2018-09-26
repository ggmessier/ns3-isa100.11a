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


#ifndef GRAPH_TDMA_OPTIMIZER
#define GRAPH_TDMA_OPTIMIZER

#include "ns3/tdma-optimizer-base.h"
//#include "ns3/isa-graph.h"
#include "ns3/ptr.h"

using namespace std;

namespace ns3{

//class IsaGraph;

/**
 * \class GraphTdmaOptimzer
 *
 * \brief A minimum hop TDMA scheduler.
 *
 * This class is based on a variant of the breadth first minimum hop search algorithm.
 *
 */
class GraphTdmaOptimzer : public TdmaOptimizerBase
{
public:

  static TypeId GetTypeId (void);

  GraphTdmaOptimzer ();

  ~GraphTdmaOptimzer ();

  /** Perform some initial calculations required by the optimizer.
   *   @param c node container with all nodes
   *   @param propModel the propagation loss model
   */

  void SetupOptimization (NodeContainer c, Ptr<PropagationLossModel> propModel);

  /** Solve for the packet flows using a minimum hop breadth first search algorithm.
   * @return packetFlows A matrix of packet flows between nodes.
   */
  virtual std::vector< std::vector< int > > SolveTdma (void);

//  virtual std::map <uint32_t, Ptr<IsaGraph>> GetGraphMap(void);
//  virtual Ptr<IsaGraph> GetGraph(void);


private:

  /** Create a graph from the container information..
     *
     * @param c node container with all nodes
     */
  void GraphCreation(NodeContainer c); // Rajith

//  std::map <uint32_t, Ptr<IsaGraph>> m_graphMap;
//  Ptr<IsaGraph> m_graph;

};


}
#endif /* MINHOP_TMDA_OPTIMIZER */
