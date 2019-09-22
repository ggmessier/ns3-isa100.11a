/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 The University Of Calgary- FISHLAB
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
 * Author:   Michael Herrmann <mjherrma@ucalgary.ca>
 */

#include "ns3/convex-integer-tdma-optimizer.h"

#include "ns3/boolean.h"
#include "ns3/integer.h"
#include "ns3/double.h"
#include "ns3/log.h"

#include "ns3/zigbee-trx-current-model.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/mobility-model.h"
#include "ns3/isa100-net-device.h"
#include "ns3/isa100-dl.h"
#include "ns3/isa100-processor.h"

#include <ilcplex/ilocplex.h>
#include <algorithm>
#include <vector>

#include "ns3/isa-graph.h"

ILOSTLBEGIN

  NS_LOG_COMPONENT_DEFINE ("ConvexIntTdmaOptimizer");

namespace ns3 {

struct nodeElement
{
  std::vector<NetworkLink *> inLinks;
  std::vector<NetworkLink *> outLinks;
};

NS_OBJECT_ENSURE_REGISTERED (ConvexIntTdmaOptimizer);

TypeId ConvexIntTdmaOptimizer::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::ConvexIntTdmaOptimizer")
    .SetParent<TdmaOptimizerBase> ()
    .AddConstructor<ConvexIntTdmaOptimizer> ()

  ;

  return tid;
}

ConvexIntTdmaOptimizer::ConvexIntTdmaOptimizer () : TdmaOptimizerBase ()
{
  NS_LOG_FUNCTION (this);
}

ConvexIntTdmaOptimizer::~ConvexIntTdmaOptimizer ()
{
  NS_LOG_FUNCTION (this);
}

void ConvexIntTdmaOptimizer::SetupOptimization (NodeContainer c, Ptr<PropagationLossModel> propModel)
{
  NS_LOG_FUNCTION (this);

  // Setup the common base properties
  TdmaOptimizerBase::SetupOptimization (c, propModel);

  // initial graph creation based on all available links
  TdmaOptimizerBase::GraphCreation (c);

  // Indicate that optimization has been setup
  m_isSetup = true;
}

std::vector< std::vector<int> > ConvexIntTdmaOptimizer::SolveTdma (void)
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT_MSG (m_isSetup, "TDMA Optimizer: Must setup optimization before calling Solve!");

  typedef IloArray<IloIntVarArray> IntVarMatrix;

  vector< vector<int> > flows (m_numNodes);

  // Note: For now, hard code the optimizer to only use one multi-frame.  Will need to expand the scheduler to change this.
  // - The multi-frame version solves the next frame with initial energies updated to reflect how much energy was used up
  //   in the first frame.
  m_numMultiFrames = 1;
  NS_LOG_DEBUG ("** Hard coded to 1 multiframe.");


  // Solve a schedule for 'numMultiFrames' frames
  for (m_currMultiFrame = 0; m_currMultiFrame < m_numMultiFrames; m_currMultiFrame++)
    {
      NS_LOG_DEBUG ("---------------- Solving Frame " << (uint16_t)m_currMultiFrame << " ----------------");

      tdmaSchedule currFrameSchd;

      // Determine initial energies if solving frame 0 (otherwise they are updated at the end of the previous)
      if (m_currMultiFrame == 0)
        {
          for (uint8_t i = 0; i < m_numNodes; i++)
            {
              if (i <= 2)
                {
                  m_frameInitEnergiesJ.push_back (m_initialEnergy * 1e300);
                }
              else
                {
                  m_frameInitEnergiesJ.push_back (m_initialEnergy);
                }
            }
        }

      IloEnv env;
      try
        {
          IloModel model (env);

          // Variables for optimization
          // Packet flows and max energy
          IntVarMatrix pktFlowsVars (env, m_numNodes);
          IntVarMatrix numSlotsVars (env, m_numNodes);
          IloNumVar lifetimeInvVar (env, 0.0, IloInfinity, "1_div_Lifetime"); // in seconds
          IloNumVarArray nodeEnergies (env);

          char flowName[16];
          char linkName[16];
          char nodeE[16];

          // Iterate through all combinations of nodes to create variables
          for (IloInt i = 0; i < m_numNodes; i++)
            {

              // Initialize node energy consumption variable
              sprintf (nodeE, "E_used_%li", i);
              nodeEnergies.add (IloNumVar (env, 0, m_frameInitEnergiesJ[i], nodeE));

              // Initialize ith row of flows array
              pktFlowsVars[i] = IloIntVarArray (env);
              numSlotsVars[i] = IloIntVarArray (env);

              for (IloInt j = 0; j < m_numNodes; j++)
                {

                  sprintf (flowName, "W_%li_%li", i, j);
                  sprintf (linkName, "L_%li_%li", i, j);

                  // Node doesn't transmit to itself
                  if (i == j)
                    {
                      pktFlowsVars[i].add (IloIntVar (env, 0, 0, flowName));
                      numSlotsVars[i].add (IloIntVar (env, 0, 0, linkName));
                    }

                  // Node transmits to a different one
                  else
                    {
                      // Check if node is beyond range (cost to tx a bit is higher than allowed)
                      if (m_txEnergyByte[i][j] > m_maxTxEnergyByte)
                        {
                          pktFlowsVars[i].add (IloIntVar (env, 0, 0, flowName));
                          numSlotsVars[i].add (IloIntVar (env, 0, 0, linkName));
                        }

                      else
                        {
                          // Sink node does not transmit
                          if (i == m_sinkIndex)
                            {
                              pktFlowsVars[i].add (IloIntVar (env, 0, 0, flowName));
                              numSlotsVars[i].add (IloIntVar (env, 0, 0, linkName));
                            }

                          else
                            {
                              pktFlowsVars[i].add (IloIntVar (env, 0, IloIntMax, flowName));
                              numSlotsVars[i].add (IloIntVar (env, 0, IloIntMax, linkName));
                            }
                        }
                    }
                }
            }

          // Create constraints
          for (uint32_t i = 0; i < m_numNodes; i++)
            {
              IloExpr sumLinkTimes (env);
              IloExpr sumFlowsOut (env);
              IloExpr sumFlowsIn (env);
              IloExpr sumEnergyTx (env);
              IloExpr sumEnergyRx (env);
              IloExpr sumEnergyIdle (env);

              for (int j = 0; j < m_numNodes; j++)
                {
                  // TDMA sum of assigned link times
                  sumLinkTimes += pktFlowsVars[i][j] * m_numBytesPkt * 8 / m_bitRate;

                  // Sum of flows
                  sumFlowsOut += pktFlowsVars[i][j];
                  sumFlowsIn  += pktFlowsVars[j][i];

                  // Energy
                  sumEnergyTx += m_txEnergyByte[i][j] * pktFlowsVars[i][j] * m_numBytesPkt;
                  sumEnergyRx += m_rxEnergyByte * pktFlowsVars[j][i] * m_numBytesPkt;

                }

              // TDMA constraint
              model.add (sumLinkTimes <= m_usableSlotDuration.GetSeconds () * m_numTimeslots);

              if (i != m_sinkIndex)
                {
                  // conservation of flow constraint
                  model.add (sumFlowsIn + m_numPktsNode == sumFlowsOut);

                  // conservation of energy
                  model.add (sumEnergyTx + sumEnergyRx + sumEnergyIdle == nodeEnergies[i]);

                  // max inverse lifetime constraint
                  model.add (nodeEnergies[i] / (m_frameInitEnergiesJ[i] * m_slotDuration.GetSeconds () * m_numTimeslots) <= lifetimeInvVar);
                }
            }

          // Specify objective (minimize the maximum lifetime inverse out of all nodes)
          model.add (IloMinimize (env, lifetimeInvVar));

          IloCplex cplex (model);

          // Disable output, might be nice to integrate with ns3 logging as well if possible
          cplex.setOut (env.getNullStream ());

          // Might be nice to figure out how to integrate this output with NS_LOG levels
          // Exports the model. Can use LP, SAV or MPS format

          cplex.setParam (IloCplex::EpGap, 0.01); // Integer gap tolerance
          cplex.setParam (IloCplex::MIPDisplay, 2); // Display level output
          cplex.setParam (IloCplex::TiLim, 60 * 5); // Max optimization time (in sec)

          // Solve the optimization
          if (!cplex.solve ())
            {
              NS_FATAL_ERROR ("Failed to optimize LP: " << cplex.getStatus ());
            }

          // Obtain results
          IloNumArray flowVals (env);
          IloNumArray linkVals (env);
          IloNum objVal;
          std::vector<NetworkLink> linkList;
          std::vector<uint16_t> linksNumPkts;
          double lifetimeResult;

          NS_ASSERT_MSG (cplex.getStatus () == IloAlgorithm::Optimal, "Convex solver couldn't find optimal solution!");
          objVal = cplex.getObjValue ();
          lifetimeResult = 1 / objVal;

          NS_LOG_DEBUG (" Solution status = " << cplex.getStatus ());
          NS_LOG_DEBUG (" Solution value, Lifetime Inverse  = " << objVal);
          NS_LOG_DEBUG (std::fixed << std::setprecision (2) << " Calculated lifetime value   = " << lifetimeResult);

          for (int i = 0; i < m_numNodes; i++)
            {
              flows[i].assign (m_numNodes,0);
            }


          for (int i = 0; i < m_numNodes; i++)
            {

              if (i != 0 )
                {

                  cplex.getValues (flowVals,pktFlowsVars[i]);
                  //    NS_LOG_DEBUG(" Packet Flow Values for Node " << i << "  =  " << flowVals);

                  /* These lines would be used to update initial energy for the next frame optimization.
                   *  IloNum usedEnergy = cplex.getValue (nodeEnergies[i]) / energyScale;
                   *  m_frameInitEnergiesJ[i] -= usedEnergy;
                   */

                  for (int j = 0; j < m_numNodes; j++)
                    {
                      flows[i][j] += ceil ((double)flowVals[j] / m_packetsPerSlot);
                    }
                }
            }

        }
      catch (IloAlgorithm::CannotExtractException& e) {
        NS_LOG_DEBUG ("CannotExtractExpection: " << e);
        IloExtractableArray failed = e.getExtractables ();
        for (IloInt i = 0; i < failed.getSize (); i++)
          {
            NS_LOG_DEBUG ("\t" << failed[i]);
          }
        NS_FATAL_ERROR ("Concert Fatal Error.");
      }
      catch (IloException& e) {
          NS_FATAL_ERROR ("Concert exception caught: " << e );
        }
      catch (...) {
        NS_FATAL_ERROR ("Unknown exception caught");
      }

      env.end ();
    }

  NS_LOG_DEBUG (" Flow matrix:");
  std::stringstream ss;

  for (int i = 0; i < m_numNodes; i++)
    {

      ss.str ( std::string () );
      ss << "Node " << i << ": ";

      for (int j = 0; j < m_numNodes; j++)
        {

          if (flows[i][j] != 0)
            {
              ss << j << "(" << flows[i][j] << ",";
            }

          // Determine number of packets per slot for each link.
          flows[i][j] = ceil ((double)flows[i][j] / m_packetsPerSlot);

          if (flows[i][j] != 0)
            {
              ss << flows[i][j] << "), ";
            }
        }

      NS_LOG_DEBUG ( ss.str () );
    }



  return flows;
}

void ConvexIntTdmaOptimizer::PopulateBackup (std::vector< std::vector<int> > flows)
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT_MSG (m_isSetup, "TDMA Optimizer: Must setup optimization before calling PopulateBackup!");

  uint32_t gwID = m_graph->GetGateway()->GetId();
  uint32_t numNodes = m_graph->GetNumofNodes();

  // Flow matrix to primaryPath Vector conversion (including vertex loading)
  std::map<uint32_t, std::vector<uint32_t> > primaryPath = FlowMatrixToPath (flows);

  // populate the primary paths and their normalized loads
  double Et, Er;
  m_routeIndexIt = 0;
  rowUInt_t routeIndexRw (numNodes,0);
  m_routeIndexMat.resize (numNodes, routeIndexRw);

  for (uint32_t i = 0; i < numNodes; i++)
    {
      if (i >= 3)
        {
          m_ULEx.push_back (primaryPath[i]);   // primary UPLINK paths included.
          m_routeIndexIt++;
          // source -> destination route index
          m_routeIndexMat[i][gwID] = m_routeIndexIt;
        }

      uint32_t txNode, nextNode;

      for (uint32_t j = 0; j < primaryPath[i].size (); j++)
        {
          if (j == 0)
            {
              txNode = primaryPath[i][j];
              NS_LOG_DEBUG ("************ PRIMARY PATH ************" << txNode );
              continue;
            }
          nextNode = primaryPath[i][j];
          NS_LOG_DEBUG (nextNode);

          // node 0, 1 and 2 powered by external power source. Therefore, power consumption is not considered.
          if (txNode == 0 || txNode == 1 || txNode == 2)
            {
              Et = 0;
            }
          else
            {
              Et = m_txEnergyExpected[txNode][nextNode];
              m_vertexVector[txNode].m_normalizedLoad = m_vertexVector[txNode].m_normalizedLoad  +
                  m_vertexVector[txNode].m_flowRate * Et / m_vertexVector[txNode].m_initialBatteryEnergy;
            }

          if (nextNode == 0 || nextNode == 1 || nextNode == 2)
            {
              Er = 0;
            }
          else
            {
              Er = m_rxEnergyExpected[txNode][nextNode];
              m_vertexVector[nextNode].m_normalizedLoad = m_vertexVector[nextNode].m_normalizedLoad  +
                  m_vertexVector[nextNode].m_flowRate * Er / m_vertexVector[nextNode].m_initialBatteryEnergy;
            }

          txNode = nextNode;
        }
    }

  // populate the backup paths
  std::map<uint32_t, MinLoadVertex> vertex;

  std::map<uint32_t, std::vector<std::vector<uint32_t> > > backupPath;
  std::map<uint32_t, std::vector<std::vector<uint32_t> > > tempBackupPath;
  // backup path route index -> node -> load
  std::map<uint32_t, std::map<uint32_t, double> > load;

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
      backupPath = tempBackupPath;

      uint32_t srcNodePrimaryPath;

      // iterate over the primary path
      for (std::map<uint32_t, std::vector<uint32_t> >::const_iterator it = primaryPath.begin ();
          it != primaryPath.end (); ++it)
        {
          srcNodePrimaryPath = it->first;

          // root index of the route source -> destination
          uint32_t rootIndex = m_routeIndexMat[srcNodePrimaryPath][gwID];

          // remove the previously allocated normalized load prior to calculate the new load
          for (uint32_t j = 0; j < backupPath[srcNodePrimaryPath].size (); j++)
            {
              for (uint32_t k = 0; k < backupPath[srcNodePrimaryPath][j].size (); k++)
                {
                  if (!load[rootIndex].empty ())
                    {
                      // node of the backup path
                      uint32_t nodeInBPath = backupPath[srcNodePrimaryPath][j][k];
                      tempVertex[nodeInBPath].m_normalizedLoad -= load[rootIndex][nodeInBPath];
                    }
                }
            }

          std::vector<uint32_t> primaryPath = it->second;
          tempBackupPath[srcNodePrimaryPath].clear ();
          uint32_t nodeInPrimaryPath;

          // iterate over the primary paths
          for (uint32_t i = 0; i < primaryPath.size (); i++)
            {
              // node of primary path
              nodeInPrimaryPath = primaryPath[i];

              // this is to avoid finding a backup path for gateway node (destination)
              if (nodeInPrimaryPath == gwID)
                {
                  continue;
                }

              std::vector<uint32_t>  tempBPathVect;
              // generate the respective backup path
              vertex = BackupPath (tempVertex, primaryPath, nodeInPrimaryPath, gwID);

              // last Hop parameter of the vertex (.m_lastHop) is the backup path of nodeInPPath
              if (vertex[nodeInPrimaryPath].m_normalizedLoad != INF_DOUBLE)
                {
                  NS_LOG_DEBUG("********** Path of: "<<nodeInPrimaryPath);
                  // initial node of the backup path
                  uint32_t hop = nodeInPrimaryPath;

                  while (true)
                    {
                      NS_LOG_DEBUG (hop);

                      // track the load incremented
                      load[rootIndex][hop] = vertex[hop].m_normalizedLoad -
                          tempVertex[hop].m_normalizedLoad;

                      tempVertex[hop].m_normalizedLoad = vertex[hop].m_normalizedLoad;

                      tempBPathVect.push_back (hop);

                      if (hop == gwID)
                        {
                          break;
                        }

                      hop = vertex[hop].m_lastHop;
                    }
                }
              tempBackupPath[srcNodePrimaryPath].push_back (tempBPathVect);
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
      backupPath = tempBackupPath;
    }

  for (uint32_t i = 3; i < numNodes; i++)
    {
//      m_routeIndexMat[i][gwID] = m_routeIndexIt;
      m_ULSh.push_back (backupPath[i]);

      NS_LOG_DEBUG ("CONVEX: ****** BACKUP PATH ******" << i << " -> " << gwID);
      for (uint32_t k = 0; k < backupPath[i].size (); k++)
        {
          NS_LOG_DEBUG ("SUB "<< backupPath[i][k][0] << " -> " << gwID);
          for (uint32_t m = 0; m < backupPath[i][k].size (); m++)
            {
              NS_LOG_DEBUG (backupPath[i][k][m]);
            }
        }
    }
}

map<uint32_t, vector<uint32_t> > ConvexIntTdmaOptimizer::FlowMatrixToPath (std::vector< std::vector<int> > packetFlows)
{
  int numNodes = packetFlows.size ();

  map<int, int> txSum;
  vector<vector<int>> rxNodes (numNodes);
//  vector<vector<int>> flows = packetFlows;

  for (int i = 0; i < numNodes; i++)
    {
      txSum[i] = 0;
      for (int j = 0; j < numNodes; j++)
        {
          if (packetFlows[i][j] > 0)
            {
              txSum[i] += packetFlows[i][j];
              rxNodes[i].push_back(j);
            }
        }
      NS_LOG_DEBUG(" txSum: "<<i<<" "<<txSum[i]);
    }

  map<uint32_t, vector<uint32_t> > primaryPath;

  int node;
  int txSumZeroCount = 0;
  while (txSumZeroCount < numNodes)
    {
      for (int i = 0; i < numNodes; i++)
        {
          if (txSum[i] == 1)
            {
              int dst = -1;
              int src = i;
              node = src;

              primaryPath[src].push_back(node);

              while (dst != 0)
                {
                  dst = rxNodes[node].back();

                  packetFlows[node][dst]--;            // Indicate this edge has been scheduled

                  txSum[node]--;
                  if (packetFlows[node][dst] == 0)
                    rxNodes[node].pop_back();

                  node = dst;
                  primaryPath[src].push_back(node);

                  txSumZeroCount = 0;
                }
            }
          else if (txSum[i] == 0)
            {
              txSumZeroCount++;
            }
        }
    }

  return primaryPath;
}

map<uint32_t, MinLoadVertex> ConvexIntTdmaOptimizer::BackupPath (std::map<uint32_t, MinLoadVertex> vertexVect,
                                                                      std::vector<uint32_t> primaryPath, uint32_t src, uint32_t dst)
{
  NS_LOG_DEBUG(" primary path First "<<primaryPath[0]<<" src "<<src<<" dst "<<dst);

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

  std::map<uint32_t, MinLoadVertex> tempVertex =  vertex;

  // LAMDA_d = GAMMA_d + r * E_r/ B_d (ALG1 of Wu's)
  vertex[dst].m_normalizedLoad = vertexVect[dst].m_normalizedLoad +
    vertex[dst].m_flowRate * Erb / vertex[dst].m_initialBatteryEnergy;

  uint32_t u; // node with the minimum normalized load (LAMDA)

  // while Q is not empty do (ALG1 of Wu's)
  while (!queue.empty ())
    {
      // iterate over the vertices to find the minimum normalized load
      double min = INF_DOUBLE;
      int index_min = 0;  //index of the node with minimum normalized load in the Queue

      for (uint32_t index = 0; index < queue.size (); index++)
        {
          if (vertex[queue[index]].m_normalizedLoad < min)
            {
              min = vertex[queue[index]].m_normalizedLoad;
              u = queue[index];
              index_min = index;
            }
        }

      // if no node found with finite value check for a minimum load in temporary vertex vector
      if (min == INF_DOUBLE)
        {
          for (uint32_t index = 0; index < queue.size (); index++)
            {
              if (tempVertex[queue[index]].m_normalizedLoad < min)
                {
                  min = tempVertex[queue[index]].m_normalizedLoad;
                  u = queue[index];
                  index_min = index;
                }
            }

          //set the last hop nodes vertex vector
          if (min != INF_DOUBLE)
            {
              uint32_t index = index_min;
              uint32_t lastHop;
              double normalizedLoadLastHop = INF_DOUBLE;

              while (normalizedLoadLastHop == INF_DOUBLE)
                {
                  vertex[queue[index]].m_normalizedLoad = tempVertex[queue[index]].m_normalizedLoad;
                  lastHop = tempVertex[queue[index]].m_lastHop;
                  vertex[queue[index]].m_lastHop = lastHop;
                  normalizedLoadLastHop = vertex[lastHop].m_normalizedLoad;
                }
            }
        }
      double K1 = vertex[u].m_normalizedLoad*100000;
      double K2 = tempVertex[u].m_normalizedLoad*100000;
      uint32_t hopK1 = vertex[u].m_lastHop;
      uint32_t hopK2 = tempVertex[u].m_lastHop;

      NS_LOG_DEBUG("u: "<<u<<" Load "<<K1<<" tempLoad "<<K2<<" Hop "<<hopK1<<" tempHop "<<hopK2);
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
          NS_LOG_DEBUG("neighborV "<<neighborV);
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

              bool inPPath = count (primaryPath.begin (),primaryPath.end (),neighborV);

              double K = alt*100000;  // temporary

              if (alt < vertex[neighborV].m_normalizedLoad && !inPPath)
                {
                  vertex[neighborV].m_normalizedLoad = alt;
                  vertex[neighborV].m_lastHop = u;

                  NS_LOG_DEBUG(" Condition 1: "<<K<<" Hop "<<u);
                }
              else if (alt < vertex[neighborV].m_normalizedLoad && alt < tempVertex[neighborV].m_normalizedLoad)
                {
                  tempVertex[neighborV].m_normalizedLoad = alt;
                  tempVertex[neighborV].m_lastHop = u;

                  NS_LOG_DEBUG(" Condition 2: "<<K<<" Hop "<<u);
                }
            }
        }
    }

  return vertex;
}

} // namespace ns3
