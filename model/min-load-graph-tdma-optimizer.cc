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

MinLoadGraphTdmaOptimzer::MinLoadGraphTdmaOptimzer () : TdmaOptimizerBase()
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
  TdmaOptimizerBase::SetupOptimization(c, propModel);
  GraphCreation(c);

}

void MinLoadGraphTdmaOptimzer::GraphCreation(NodeContainer c)
{
  NS_LOG_FUNCTION (this);

  m_graph = CreateObject<IsaGraph>(c);
  uint32_t gwID = 0;
  uint32_t ap1ID = 1;
  uint32_t ap2ID = 2;

  m_graph->AddGateway(gwID);

  // Rajith:: code required to modify to support any number of access points
  m_graph->AddAccessPoint(ap1ID);
  m_graph->AddAccessPoint(ap2ID);

  uint32_t numNodes = c.GetN();
  TimeValue slotDurationV;
  UintegerValue tempNumSlotsV;
  uint32_t numSlotsV;

  rowUInt_t routeIndexRw(numNodes,0);
  m_routeIndexMat.resize(numNodes, routeIndexRw);

  for (uint32_t nNode = 0; nNode < numNodes; nNode++)
    {
      m_graph->AddNode(c.Get(nNode));
    }

  m_graph->AddEdge(gwID, ap1ID);
  m_graph->AddEdge(gwID, ap2ID);
//  m_graph->AddEdge(ap1ID, ap2ID);

  m_graph->AddEdge(ap1ID, gwID);
  m_graph->AddEdge(ap2ID, gwID);
  m_graph->AddEdge(ap2ID, ap1ID);

  m_graph->SetHopCount(gwID,0);
  m_graph->SetHopCount(ap1ID,1);
  m_graph->SetHopCount(ap2ID,1);

  for (uint32_t parent = 0; parent < numNodes; parent++)
    {
      Ptr<Isa100NetDevice> devPtr = c.Get(parent)->GetDevice(0)->GetObject<Isa100NetDevice>();
      devPtr->GetDl()->GetAttribute("SuperFrameSlotDuration", slotDurationV);
      m_slotDuration = slotDurationV.Get();

      devPtr->GetDl()->GetAttribute("SuperFramePeriod", tempNumSlotsV);
      numSlotsV = tempNumSlotsV.Get();

      // populate vertex vector and normalized load vector - Wu's Algorithm
      m_rawVertex[parent].m_head = c.Get(parent);
      m_rawVertex[parent].m_initialBatteryEnergy = devPtr->GetBattery()->GetEnergy();
      m_rawVertex[parent].m_flowRate = 1/(numSlotsV*m_slotDuration.GetSeconds());
      m_rawVertex[parent].m_normalizedLoad = 0;

      m_vertexVector[parent] = m_rawVertex[parent];
      m_rawVertex[parent].m_normalizedLoad = INF_DOUBLE;
//      m_normalizedLoadMap[parent] = 0;

      m_graph->SetTimeSlots(parent, numSlotsV);

      for (uint32_t nNode = 1; nNode < numNodes; nNode++)
        {
          if(parent != 0 && parent != nNode && m_txPowerDbm[parent][nNode] <= m_maxTxPowerDbm - 5)
            {
              m_graph->AddEdge(parent, nNode);
              NS_LOG_UNCOND("Edge: "<<parent<<" "<<nNode);
            }
        }
    }

  // UPLINK routes creation
  Ptr<IsaGraph> G = CreateObject<IsaGraph> ();
  std::map<uint32_t, MinLoadVertex> vertex;
//  std::map<uint32_t, MinLoadVertex> vertex = m_vertexVector;
  for(uint32_t i = 3; i < numNodes; i++)
    {
      m_routeIndexMat[i][gwID] = m_routeIndexIt;
      G.operator *() = m_graph.operator *();
//      vertex = MinLoadGraphRoute(vertex, m_routeIndexIt, i, gwID);
      vertex = MinLoadGraphRoute(m_vertexVector, m_routeIndexIt, i, gwID);

      NS_LOG_UNCOND("****** PRIMARY PATH "<<i<<" -> "<<gwID);
      uint32_t j = i;
      m_primaryPath[m_routeIndexIt].push_back(j);
      NS_LOG_UNCOND(j);
      while (j != gwID && vertex[i].m_normalizedLoad != INF_DOUBLE)
        {
          m_primaryPath[m_routeIndexIt].push_back(vertex[j].m_lastHop);
          m_vertexVector[j].m_normalizedLoad = vertex[j].m_normalizedLoad;
          j = vertex[j].m_lastHop;
          NS_LOG_UNCOND(j);
        }

      NS_LOG_UNCOND(" BACKUP PATH "<<i<<" -> "<<gwID);
      j = i;
      m_backUpPath[m_routeIndexIt] = vertex[i].m_backupPath;
      for (uint32_t k = 0; k < m_backUpPath[m_routeIndexIt].size(); k++)
        {
          NS_LOG_UNCOND(m_backUpPath[m_routeIndexIt][k]);
        }

      m_routeIndexIt++;
    }

}

vector< vector<int> > MinLoadGraphTdmaOptimzer::SolveTdma (void)
{
  NS_LOG_FUNCTION (this);

  vector< vector<int> > flows(m_numNodes);

  return flows;

}

map<uint32_t, MinLoadVertex> MinLoadGraphTdmaOptimzer::MinLoadGraphRoute(map<uint32_t, MinLoadVertex> vertexVect, uint32_t routeIndexIt, uint32_t src, uint32_t dst)
{
  map<uint32_t, MinLoadVertex> vertex = m_rawVertex;

  if (src == dst)
    return vertexVect;

  vector<uint32_t> queue;

  // LAMDA_d = GAMMA_d + r * E_r/ B_d (ALG1 of Wu's)
  vertex[dst].m_normalizedLoad = vertexVect[dst].m_normalizedLoad +
      vertex[dst].m_flowRate*m_rxEnergyGeneralExpected/vertex[dst].m_initialBatteryEnergy;

  // add v to Q (ALG1 of Wu's)
  for (map<uint32_t, MinLoadVertex>::const_iterator it = vertex.begin ();it != vertex.end (); ++it)
    {
      queue.push_back(it->first);
    }

  uint32_t u; // node with the minimum normalized load (LAMDA)
//  Ptr<IsaGraph> G = CreateObject<IsaGraph> ();
//  G.operator *() = graph.operator *();

//  NS_LOG_UNCOND("Nodes: "<<G->GetNumofNodes());

  // while Q is not empty do (ALG1 of Wu's)
  while (!queue.empty())
  {
    u = queue.back();    // node with the minimum normalized load (LAMDA)

    // iterate over the vertices to find the minimum normalized load
    double min = INF_DOUBLE;
    int index_u = 0;  // index of the node u
    int index_min = 0;
    for (uint32_t i = 0; i < queue.size(); i++)
      {
        if(vertex[queue[i]].m_normalizedLoad < min)
          {
            min = vertex[queue[i]].m_normalizedLoad;
            u = queue[i];
            index_min = index_u;
          }
        index_u++;
      }
//    NS_LOG_UNCOND("MLGR min: "<<min<<" "<<index_min<<" "<<u);
    queue.erase(queue.begin () + index_min);

//    vertexVect[u] = vertex[u];
//    NS_LOG_UNCOND("***** MLGR u "<<u<<" "<<vertex[u].m_normalizedLoad);
    // (if LAMDA_u is INF_DOUBLE return INF_DOUBLE) & (if u is source return LAMDA_u) (ALG1 of Wu's)
//    if (min == INF_DOUBLE || u == src)
//        return vertex;
    if (min == INF_DOUBLE || u == src)
      {
//        NS_LOG_UNCOND("MLGR min "<<min<<" u "<<u);
//        return vertexVect;
        return vertex;
      }

    vector <Ptr<Node> > neighborsOfU = m_graph->GetGraphNodeMap()[u].m_neighbors;

    for (vector<Ptr<Node>>::const_iterator it = neighborsOfU.begin ();it != neighborsOfU.end (); ++it)
      {
        uint32_t neighborV = it->operator ->()->GetId();
//        NS_LOG_UNCOND("***** MLGR ****** neighborV "<<neighborV);

        if (count(queue.begin(),queue.end(),neighborV) > 0)
          {
            map<uint32_t, MinLoadVertex> vertexBackup = vertexVect;

            uint32_t j = u;
            while (j != dst && vertex[u].m_normalizedLoad != INF_DOUBLE)
              {
                vertexBackup[j].m_normalizedLoad = vertex[j].m_normalizedLoad;
                j = vertex[j].m_lastHop;
              }
//            vertexBackup = MinLoadSourceRoute(vertexVect, routeIndexIt, neighborV, dst);     // Need to sort out this graph issue
            vertexBackup = MinLoadSourceRoute(vertexBackup, routeIndexIt, neighborV, dst);     // Need to sort out this graph issue

//            for (map<uint32_t, MinLoadVertex>::const_iterator it = tempVertex.begin ();it != tempVertex.end (); ++it)
//              {
//                if (it->second.m_normalizedLoad != INF_DOUBLE)
//                  {
//                    NS_LOG_UNCOND("tempVertex src: "<<src<<" dst: "<<dst<<" "<<it->first<<" norm Load "<<it->second.m_normalizedLoad
//                                  <<" Last Hop: "<<it->second.m_lastHop);
//                  }
//              }
//            NS_LOG_UNCOND("tempVertex[neighborV].m_normalizedLoad "<<tempVertex[neighborV].m_normalizedLoad);
            vector<uint32_t> tempBackupPath;
            if (vertexBackup[neighborV].m_normalizedLoad != INF_DOUBLE)
              {
                // update backup path vector
                uint32_t j = neighborV;
                tempBackupPath.push_back(j);
                while (j != dst)
                  {
                    tempBackupPath.push_back(vertexBackup[j].m_lastHop);
                    j = vertexBackup[j].m_lastHop;
                  }

                double newNormLoad = vertexVect[neighborV].m_normalizedLoad  +
                    vertex[neighborV].m_flowRate*(m_txEnergyExpected[neighborV][u] + m_rxEnergyExpected[neighborV][u])/
                    vertex[neighborV].m_initialBatteryEnergy;
                double alt = max(vertex[u].m_normalizedLoad, max(newNormLoad, vertexBackup[neighborV].m_normalizedLoad ));
//                if (alt < m_vertexVector[neighborV].m_normalizedLoad || m_vertexVector[neighborV].m_normalizedLoad == 0)
                if (alt < vertex[neighborV].m_normalizedLoad)
                  {
                    vertex[neighborV].m_normalizedLoad = alt;
//                    NS_LOG_UNCOND("neighborV "<<neighborV<<" Load "<<alt);
                    vertex[neighborV].m_lastHop = u;
//                    NS_LOG_UNCOND("LastHop "<<u);
                    vertex[neighborV].m_backupPath = tempBackupPath;
                  }
              }
          }
      }

//    G->RemoveGraphNode(u);
  }

//  for (map<uint32_t, MinLoadVertex>::const_iterator it = vertex.begin ();it != vertex.end (); ++it)
//    {
//      if (it->second.m_tempNormalizedLoad != INF_DOUBLE)
//        {
//          NS_LOG_UNCOND("src: "<<src<<" dst: "<<dst<<" "<<it->first<<" -----MLGR---- norm Load "<<it->second.m_tempNormalizedLoad
//                        <<" Last Hop: "<<it->second.m_lastHop);
//        }
//    }

  return vertex;

}

map<uint32_t, MinLoadVertex> MinLoadGraphTdmaOptimzer::MinLoadSourceRoute(map<uint32_t, MinLoadVertex> vertexVect, uint32_t routeIndexIt, uint32_t src, uint32_t dst)
{
  map<uint32_t, MinLoadVertex> vertex = m_rawVertex;
  vector<uint32_t> queue;

  // LAMDA_d = GAMMA_d + r * E_r/ B_d (ALG1 of Wu's)
  vertex[dst].m_normalizedLoad = vertexVect[dst].m_normalizedLoad +
      vertex[dst].m_flowRate*m_rxEnergyBackupGeneralExpected/vertex[dst].m_initialBatteryEnergy;

  // add v to Q (ALG1 of Wu's)
  for (map<uint32_t, MinLoadVertex>::const_iterator it = vertex.begin ();it != vertex.end (); ++it)
    {
      queue.push_back(it->first);
    }

  uint32_t u; // node with the minimum normalized load (LAMDA)
//  Ptr<IsaGraph> G = CreateObject<IsaGraph> ();
//  G.operator *() = graph.operator *();

//  NS_LOG_UNCOND("Nodes: "<<G->GetNumofNodes());

  // while Q is not empty do (ALG1 of Wu's)
  while (!queue.empty())
  {
      u = queue.back();    // node with the minimum normalized load (LAMDA)
//      NS_LOG_UNCOND("u: "<<u);

      // iterate over the vertices to find the minimum normalized load
      double min = INF_DOUBLE;
      int index_u = 0;  // index of the node u
      int index_min = 0;
      for (uint32_t i = 0; i < queue.size(); i++)
        {
          if(vertex[queue[i]].m_normalizedLoad < min)
            {
              min = vertex[queue[i]].m_normalizedLoad;
              u = queue[i];
              index_min = index_u;
            }
          index_u++;
        }
//      NS_LOG_UNCOND("MLSR min: "<<min<<" "<<index_min<<" "<<u);
      queue.erase(queue.begin () + index_min);

//      vertexVect[u] = vertex[u];
//      NS_LOG_UNCOND("MLSR u "<<u<<" "<<vertex[u].m_normalizedLoad);
      // (if LAMDA_u is INF_DOUBLE return INF_DOUBLE) & (if u is source return LAMDA_u) (ALG1 of Wu's)
      //    if (min == INF_DOUBLE || u == src)
      //        return vertex;
      if (min == INF_DOUBLE || u == src)
        {
  //        NS_LOG_UNCOND("MLGR min "<<min<<" u "<<u);
//          return vertexVect;
          return vertex;
        }

      vector <Ptr<Node> > neighborsOfU = m_graph->GetGraphNodeMap()[u].m_neighbors;

      for (vector<Ptr<Node>>::const_iterator it = neighborsOfU.begin ();it != neighborsOfU.end (); ++it)
        {
          uint32_t neighborV = it->operator ->()->GetId();
//          NS_LOG_UNCOND("MLSR neighborV "<<neighborV);
          if (count(queue.begin(),queue.end(),neighborV) > 0)
            {
              double newNormLoad = vertexVect[neighborV].m_normalizedLoad +
                  vertex[neighborV].m_flowRate*(m_rxEnergyBackupExpected[neighborV][u])/vertex[neighborV].m_initialBatteryEnergy;
              double alt = max(vertex[u].m_normalizedLoad, newNormLoad);
//              NS_LOG_UNCOND("MSLR vertex[u].m_normalizedLoad "<<vertex[u].m_normalizedLoad<<" newNormLoad "<<newNormLoad);
              if (alt < vertex[neighborV].m_normalizedLoad)
                {
                  vertex[neighborV].m_normalizedLoad = alt;
                  vertex[neighborV].m_lastHop = u;
//                  NS_LOG_UNCOND("MSLR neighborV "<<neighborV<<" Load "<<alt);
//                  NS_LOG_UNCOND("MLSR LastHop "<<u);
                }
            }
        }
  }

  return vertex;
}
