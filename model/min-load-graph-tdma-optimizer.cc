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

#include "ns3/graph-tdma-optimizer.h"

#include "ns3/log.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"
#include "ns3/integer.h"
#include "ns3/ptr.h"
//
#include "ns3/zigbee-trx-current-model.h"
#include "ns3/mobility-model.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/isa100-net-device.h"
#include "ns3/isa100-dl.h"
#include "ns3/isa100-battery.h"
#include "ns3/isa-graph.h"

#include <algorithm>
#include <limits>
#include <vector>


NS_LOG_COMPONENT_DEFINE ("MinLoadGraphTdmaOptimzer");

using namespace ns3;
using namespace std;

NS_OBJECT_ENSURE_REGISTERED (MinLoadGraphTdmaOptimzer);

TypeId MinLoadGraphTdmaOptimzer::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::MinLoadGraphTdmaOptimzer")
    .SetParent<TdmaOptimizerBase> ()
    .AddConstructor<MinLoadGraphTdmaOptimzer> ()

  ;

  return tid;
}

MinLoadGraphTdmaOptimzer::MinLoadGraphTdmaOptimzer () : TdmaOptimizerBase ()
{
  NS_LOG_FUNCTION (this);
  m_numNodes = 0;
  m_routeIndexIt = 1;
}

MinLoadGraphTdmaOptimzer::~MinLoadGraphTdmaOptimzer ()
{
  NS_LOG_FUNCTION (this);

}

void MinLoadGraphTdmaOptimzer::SetupOptimization (NodeContainer c, Ptr<PropagationLossModel> propModel)
{
  NS_LOG_FUNCTION (this);

  // Setup the common base properties
  TdmaOptimizerBase::SetupOptimization (c, propModel);

  // initial graph creation based on all available links
  TdmaOptimizerBase::GraphCreation (c);

}

vector< vector<int> > MinLoadGraphTdmaOptimzer::SolveTdma (void)
{
  NS_LOG_FUNCTION (this);
  vector< vector<int> > flows (m_numNodes);

  uint32_t gwID = m_graph->GetGateway()->GetId();
  uint32_t numNodes = m_graph->GetNumofNodes();

  // UPLINK routes creation
  std::map<uint32_t, MinLoadVertex> vertex;

  map<uint32_t, vector<uint32_t> > primaryPath;
  map<uint32_t, vector<vector<uint32_t> > > backupPath;
  map<uint32_t, vector<uint32_t> > tempPrimaryPath;
  map<uint32_t, vector<vector<uint32_t> > > tempBackupPath;
  map<uint32_t, map<uint32_t, double> > load;

  //!!!!! minimum threshold need to be calculated using the minimum rate and the maximum initial battery value
  // since power consumption of gateway and access points considered as zero, used node 3 values
  double minLoadThreshold = m_vertexVector[1].m_flowRate * m_rxEnergyBackupGeneralExpected / m_vertexVector[1].m_initialBatteryEnergy;

  std::map<uint32_t, MinLoadVertex> tempVertex = m_vertexVector;

  double maxLoad = INF_DOUBLE;
  double preMaxLoad = 0;
  bool optSolFound = false;

  int countEq = 0;
  // iterative algorithm stops if the maximum normalized load increases or the decrease of maximum normalized load
  // is less than a threshold
  while (!optSolFound)
    {
      primaryPath = tempPrimaryPath;
      backupPath = tempBackupPath;

      //!!!!! this need to be run for the highest rate to the lowest rate - here all rates consider same
      for (uint32_t i = 3; i < numNodes; i++)
        {
          // remove the previously allocated normalized load prior to calculate the new load
          for (uint32_t k = 0; k < primaryPath[i].size (); k++)
            {
              if (!load[i].empty ())
                {
                  tempVertex[primaryPath[i][k]].m_normalizedLoad -= load[i][primaryPath[i][k]];
                }
            }

          vertex = MinLoadGraphRoute (tempVertex, m_routeIndexIt, i, gwID);

          NS_LOG_DEBUG ("PATH " << i << " -> " << gwID);
          uint32_t hop = i;
          if (vertex[i].m_normalizedLoad != INF_DOUBLE)
            {
              tempPrimaryPath[i].clear ();
              tempBackupPath[i].clear ();

              while (1)
                {
                  tempPrimaryPath[i].push_back (hop);
                  NS_LOG_DEBUG (hop);

                  // track the load incremented
                  load[i][hop] = vertex[hop].m_normalizedLoad - tempVertex[hop].m_normalizedLoad;

                  tempVertex[hop].m_normalizedLoad = vertex[hop].m_normalizedLoad;
                  //              NS_LOG_UNCOND("Normalized Load: "<<tempVertex[hop].m_normalizedLoad);

                  tempBackupPath[i].push_back (vertex[hop].m_backupPath);

                  if (hop == gwID)
                    {
                      break;
                    }

                  hop = vertex[hop].m_lastHop;
                }

            }
          else
            {
              for (uint32_t k = 0; k < primaryPath[i].size (); k++)
                {
                  if (!load[i].empty ())
                    {
                      tempVertex[primaryPath[i][k]].m_normalizedLoad += load[i][primaryPath[i][k]];
                    }
                }
            }
        }

      preMaxLoad = maxLoad;
      maxLoad = 0;
      for (uint32_t i = 3; i < numNodes; i++)
        {
          if (tempVertex[i].m_normalizedLoad > maxLoad)
            {
              maxLoad = tempVertex[i].m_normalizedLoad;
            }

          NS_LOG_DEBUG ("m_normalizedLoad: " << tempVertex[i].m_normalizedLoad);
        }
      NS_LOG_DEBUG ("Max Load: " << maxLoad << " Pre Max: " << preMaxLoad);
      double diff = maxLoad - preMaxLoad;
      NS_LOG_DEBUG ("diff: " << diff);

      if (diff == 0)
        {
          countEq++;
        }
      else
        {
          countEq = 0;
        }

      if ((diff > 0 && diff > minLoadThreshold) || (diff < 0 && (-1) * diff < minLoadThreshold) || countEq > 3)
        {
          optSolFound = true;
        }

      m_vertexVector = tempVertex;
    }

  if (maxLoad < preMaxLoad)
    {
      primaryPath = tempPrimaryPath;
      backupPath = tempBackupPath;
    }

  for (uint32_t i = 3; i < numNodes; i++)
    {
      m_routeIndexMat[i][gwID] = m_routeIndexIt;
      m_ULEx.push_back (primaryPath[i]);
      m_ULSh.push_back (backupPath[i]);

      NS_LOG_DEBUG ("****** PRIMARY PATH " << i << " -> " << gwID);
      for (uint32_t k = 0; k < primaryPath[i].size (); k++)
        {
          NS_LOG_DEBUG (primaryPath[i][k]);
        }

      NS_LOG_DEBUG ("****** BACKUP PATH " << i << " -> " << gwID);
      for (uint32_t k = 0; k < backupPath[i].size (); k++)
        {
          for (uint32_t m = 0; m < backupPath[i][k].size (); m++)
            {
              NS_LOG_DEBUG (backupPath[i][k][m]);
            }
        }

      m_routeIndexIt++;
    }

  return flows;
}

map<uint32_t, MinLoadVertex> MinLoadGraphTdmaOptimzer::MinLoadGraphRoute (map<uint32_t, MinLoadVertex> vertexVect, uint32_t routeIndexIt, uint32_t src, uint32_t dst)
{
  NS_LOG_FUNCTION (this);
  // set normalized load to INF of all nodes
  map<uint32_t, MinLoadVertex> vertex = vertexVect;

  if (src == dst)
    {
      return vertexVect;
    }

  // gateway (node 0) and Access points (node 1 & 2) do not consume battery energy for packet reception
  // connections are wired
  double Erx = m_rxEnergyGeneralExpected;
  if (dst == 0 || dst == 1 || dst == 2)
    {
      Erx = 0;
    }

  vector<uint32_t> queue;

  // add v to Q (ALG1 of Wu's)
  for (map<uint32_t, MinLoadVertex>::const_iterator it = vertex.begin (); it != vertex.end (); ++it)
    {
      vertex[it->first].m_normalizedLoad = INF_DOUBLE;       // set temporary normalized load to infinity
      vertex[it->first].m_backupPath.clear ();                // set backup path to NULL
      vertex[it->first].m_lastHop = 0;                       // set last hop to NULL
      queue.push_back (it->first);
    }

  // LAMDA_d = GAMMA_d + r * E_r/ B_d (ALG1 of Wu's)
  vertex[dst].m_normalizedLoad = vertexVect[dst].m_normalizedLoad +
    vertex[dst].m_flowRate * Erx / vertex[dst].m_initialBatteryEnergy;

  uint32_t u; // node with the minimum normalized load (LAMDA)

  // while Q is not empty do (ALG1 of Wu's)
  while (!queue.empty ())
    {
      u = queue.back (); // node with the minimum normalized load (LAMDA)

      // iterate over the vertices to find the minimum normalized load
      double min = INF_DOUBLE;
      int index_u = 0; // index of the node u
      int index_min = 0;
      for (uint32_t i = 0; i < queue.size (); i++)
        {
          if (vertex[queue[i]].m_normalizedLoad < min)
            {
              min = vertex[queue[i]].m_normalizedLoad;
              u = queue[i];
              index_min = index_u;
            }
          index_u++;
        }
//    NS_LOG_UNCOND("MLGR min: "<<min<<" "<<index_min<<" "<<u);
      // remove u from the Q
      queue.erase (queue.begin () + index_min);

      // minimum normalized load is INF or u is the source node return the normalized load information
      if (min == INF_DOUBLE || u == src)
        {
          return vertex;
        }

      // iterate over neighbor v of u (minimum load node) within the Q
      vector <Ptr<Node> > neighborsOfU = m_graph->GetGraphNodeMap ()[u].m_neighbors;
      for (vector<Ptr<Node> >::const_iterator it = neighborsOfU.begin (); it != neighborsOfU.end (); ++it)
        {
          uint32_t neighborV = it->operator -> ()->GetId ();

          // check node v in the Q
          if (count (queue.begin (),queue.end (),neighborV) > 0)
            {
              // normalized load in the back up path
              map<uint32_t, MinLoadVertex> vertexBackup = vertexVect;

              uint32_t j = u;
              while (j != dst && vertex[u].m_normalizedLoad != INF_DOUBLE)
                {
                  vertexBackup[j].m_normalizedLoad = vertex[j].m_normalizedLoad;
                  j = vertex[j].m_lastHop;
                }
              vertexBackup = MinLoadSourceRoute (vertexBackup, routeIndexIt, neighborV, dst);

              vector<uint32_t> tempBackupPath;
              if (vertexBackup[neighborV].m_normalizedLoad != INF_DOUBLE)
                {
                  // update backup path vector
                  uint32_t j = neighborV;
                  tempBackupPath.push_back (j);
                  while (j != dst)
                    {
                      tempBackupPath.push_back (vertexBackup[j].m_lastHop);
                      j = vertexBackup[j].m_lastHop;
                    }

                  // gateway (node 0) and Access points (node 1 & 2) do not consume battery energy for packet transmission and reception
                  // connections are wired
                  double Et = m_txEnergyExpected[neighborV][u];
                  double Er = m_rxEnergyExpected[neighborV][u];

                  if (neighborV == 0 || neighborV == 1 || neighborV == 2)
                    {
                      Et = 0;
                      Er = 0;
                    }

                  double newNormLoad = vertexVect[neighborV].m_normalizedLoad  +
                    vertex[neighborV].m_flowRate * (Et + Er) / vertex[neighborV].m_initialBatteryEnergy;
                  double alt = max (vertex[u].m_normalizedLoad, max (newNormLoad, vertexBackup[neighborV].m_normalizedLoad ));
                  if (alt < vertex[neighborV].m_normalizedLoad)
                    {
                      vertex[neighborV].m_normalizedLoad = alt;
                      vertex[neighborV].m_lastHop = u;
                      vertex[neighborV].m_backupPath = tempBackupPath;
                    }
                }
            }
        }

    }

  return vertex;

}

map<uint32_t, MinLoadVertex> MinLoadGraphTdmaOptimzer::MinLoadSourceRoute (map<uint32_t, MinLoadVertex> vertexVect, uint32_t routeIndexIt, uint32_t src, uint32_t dst)
{
  NS_LOG_FUNCTION (this);
  map<uint32_t, MinLoadVertex> vertex = vertexVect;
  vector<uint32_t> queue;

  // gateway (node 0) and Access points (node 1 & 2) do not consume battery energy for packet reception
  // connections are wired
  double Erb = m_rxEnergyBackupGeneralExpected;
  if (dst == 0 || dst == 1 || dst == 2)
    {
      Erb = 0;
    }

  // add v to Q (ALG1 of Wu's)
  for (map<uint32_t, MinLoadVertex>::const_iterator it = vertex.begin (); it != vertex.end (); ++it)
    {
      vertex[it->first].m_normalizedLoad = INF_DOUBLE;       // set temporary normalized load to infinity
      vertex[it->first].m_lastHop = 0;                       // set last hop to NULL
      queue.push_back (it->first);
    }

  // LAMDA_d = GAMMA_d + r * E_r/ B_d (ALG1 of Wu's)
  vertex[dst].m_normalizedLoad = vertexVect[dst].m_normalizedLoad +
    vertex[dst].m_flowRate * Erb / vertex[dst].m_initialBatteryEnergy;

  uint32_t u; // node with the minimum normalized load (LAMDA)

  // while Q is not empty do (ALG1 of Wu's)
  while (!queue.empty ())
    {
      u = queue.back ();    // node with the minimum normalized load (LAMDA)

      // iterate over the vertices to find the minimum normalized load
      double min = INF_DOUBLE;
      int index_u = 0;  // index of the node u
      int index_min = 0;
      for (uint32_t i = 0; i < queue.size (); i++)
        {
          if (vertex[queue[i]].m_normalizedLoad < min)
            {
              min = vertex[queue[i]].m_normalizedLoad;
              u = queue[i];
              index_min = index_u;
            }
          index_u++;
        }
      // remove u from the Q
      queue.erase (queue.begin () + index_min);

      // minimum normalized load is INF or u is the source node return the normalized load information
      if (min == INF_DOUBLE || u == src)
        {
          return vertex;
        }

      // iterate over neighbor v of u (minimum load node) within the Q
      vector <Ptr<Node> > neighborsOfU = m_graph->GetGraphNodeMap ()[u].m_neighbors;
      for (vector<Ptr<Node> >::const_iterator it = neighborsOfU.begin (); it != neighborsOfU.end (); ++it)
        {
          uint32_t neighborV = it->operator -> ()->GetId ();
//          NS_LOG_UNCOND("MLSR neighborV "<<neighborV);
          if (count (queue.begin (),queue.end (),neighborV) > 0)
            {
              // gateway (node 0) and Access points (node 1 & 2) do not consume battery energy for packet reception
              // connections are wired
              double Erbv = m_rxEnergyBackupExpected[neighborV][u];
              if (dst == 0 || dst == 1 || dst == 2)
                {
                  Erbv = 0;
                }

              double newNormLoad = vertexVect[neighborV].m_normalizedLoad +
                vertex[neighborV].m_flowRate * Erbv / vertex[neighborV].m_initialBatteryEnergy;
              double alt = max (vertex[u].m_normalizedLoad, newNormLoad);
              if (alt < vertex[neighborV].m_normalizedLoad)
                {
                  vertex[neighborV].m_normalizedLoad = alt;
                  vertex[neighborV].m_lastHop = u;
                }
            }
        }
    }

  return vertex;
}
