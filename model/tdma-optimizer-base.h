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


#ifndef TDMA_OPTIMIZER_BASE_H
#define TDMA_OPTIMIZER_BASE_H

#include "ns3/nstime.h"
#include "ns3/node-container.h"
//#include "ns3/isa-graph.h"
#include <map>

typedef std::vector<double> row_t;
typedef std::vector<row_t> matrix_t;

typedef std::vector<unsigned int> rowUInt_t;
typedef std::vector<rowUInt_t> matrixUInt_t;

namespace ns3 {
class PropagationLossModel;
class IsaGraph;

struct NetworkLink {
  uint8_t txNode;
  uint8_t rxNode;
  double txPowerDbm;
  uint16_t numPkts;
};

typedef enum
{
  TDMA_MIN_HOP = 0,
  TDMA_GOLDSMITH = 1,
  TDMA_CONVEX_INT = 2,
  TDMA_CONVEX_SLOT_C = 3,
  TDMA_GRAPH = 4,
  TDMA_MIN_LOAD = 5
} OptimizerSelect;

typedef std::vector<std::vector<NetworkLink> > tdmaSchedule;

class TdmaOptimizerBase : public Object
{
public:

  static TypeId GetTypeId (void);

  TdmaOptimizerBase ();

  ~TdmaOptimizerBase ();

  /** Perform some initial calculations that are required by all derived class optimizers.
   * @param c the node container for all networks nodes. Some assumptions:
   *            - The sink node is the first node in the container
   *            - All nodes are using the same phy configuration
   * @param propModel the ns3 propagation loss model
   */
  virtual void SetupOptimization (NodeContainer c, Ptr<PropagationLossModel> propModel);

  /** Pure virtual function that triggers the optimizer solution of the routing problem.
   * @return flowMatrix A matrix of packet flows between nodes.
   */
  virtual std::vector< std::vector<int> > SolveTdma (void) ;

  void SetEdgeWeights (std::vector<std::pair<uint32_t,uint32_t>> edgeWeight);

  // Attributes for the HAN's Graph Algorithm
  std::map <uint32_t, Ptr<IsaGraph>> m_graphMap;  // all the graphs
  Ptr<IsaGraph> m_graph;        // pointer for the main graph (initial)
  std::vector<std::pair<uint32_t,uint32_t>> m_edgeWeightTDMA;   // edge weight graph if necessary

  // Attributes for Wu's Algorithm
  std::vector<std::vector<uint32_t>> m_ULEx;  //< UL paths vector; Exclusive; vector< path source -> destination >
  std::vector<std::vector<uint32_t>> m_ULSh;  //< UL paths vector; Shared; vector< path source -> destination >
  std::vector<std::vector<uint32_t>> m_DLEx;  //< DL paths vector; Exclusive; vector< path source -> destination >
  std::vector<std::vector<uint32_t>> m_DLSh;  //< DL paths vector; Shared; vector< path source -> destination >
//  // size of UL DL EX SHARED slots for Wu's ALGO
//  std::vector<uint32_t> m_flowBoundaries;     // UL-EX [0] UL-SHARED [1] DL-EX [2] DL-SHARED [3]

protected:

  uint16_t m_numNodes;        ///< Number of nodes in the network (including sink)
  Time m_slotDuration;        ///< Duration of a timeslot
  Time m_usableSlotDuration;  ///< Duration of the usable tx portion of a timeslot
  uint16_t m_numTimeslots;    ///< Total number of timeslots within a TDMA frame
  bool m_isSetup;             ///< Indicates if the optimization has been setup
  uint8_t m_currMultiFrame;   ///< The frame in a multi-frame sf which is being currently solved
  std::vector<double> m_frameInitEnergiesJ; ///< The initial energy of the current multi-frame
  double m_bitRate;           ///< The bit rate used in transmission
  double m_minRxPowerDbm;     ///< Minimum rx power required for communication to occur (dBm)
  double m_noiseFloorDbm;     ///< Noise floor (dBm), signals below this are insignificant/non-interfering
  double m_initialEnergy;   ///< The initial energy a node has (Joules).
  int m_packetsPerSlot; ///< Number of packets that can be transmitted per timeslot.
  double m_maxTxPowerDbm; ///< Maximum transmit power (dBm).


  // Attributes
  uint16_t m_sinkIndex;       ///< The index of the sink node.
  uint8_t m_numMultiFrames;  ///< Number of unique frames within a superframe
  uint16_t m_numBytesPkt;    ///< The number of bytes per packet (whole packet, not just payload)
  uint8_t m_numPktsNode;     ///< Number of packets each sensor node must send within a frame
  bool m_multiplePacketsPerSlot;   ///< Indicates if the node can transmit multiple packets per slot.
  double m_rxSensitivityDbm; ///< Receiver sensitivity (dBm).

  matrix_t m_txEnergyBit;  ///< A matrix[i][j] for the tx energy per bit (Joules/bit) for each link (i->j).
  matrix_t m_txPowerDbm;    ///< A matrix[i][j] for the tx power required to transmit on each link (i->j).
  double m_maxTxEnergyBit; ///< The maximum energy which can be used to transmit one bit (Joules/bit)
  double m_rxEnergyBit;    ///< The amount of energy to receive one bit (Joules/bit).

  matrix_t m_txEnergyByte;  ///< A matrix[i][j] for the tx energy per byte (Joules/byte) for each link (i->j).
  double m_maxTxEnergyByte; ///< The maximum energy which can be used to transmit one byte (Joules/byte)
  double m_rxEnergyByte;    ///< The amount of energy to receive one byte (Joules/byte).

  // Attributes for the Wu's algorithm
  matrix_t m_packetReceptionRatio; ///< A matrix[i][j] for Packet Reception Ratio (PPR) as a rate (e.g.: 0.90 for 90%) for each link (i->j).
  matrix_t m_txEnergyExpected; ///< A matrix[i][j] for based on the Wu's Expected TX energy for each link (i->j).
  matrix_t m_rxEnergyExpected; ///< A matrix[i][j] for based on the Wu's Expected RX energy for each link (i->j).
  matrix_t m_txEnergyBackupExpected; ///< A matrix[i][j] for based on the Wu's Expected TX energy for each backup path (i->j).
  matrix_t m_rxEnergyBackupExpected; ///< A matrix[i][j] for based on the Wu's Expected RX energy for each backup path (i->j).
  double m_rxEnergyGeneralExpected; ///< General value based on the Wu's Expected RX energy for any link.
  double m_rxEnergyBackupGeneralExpected; ///< General value based on the Wu's Expected RX energy for any link of backup paths.

};


}

#endif /* TDMA_OPTIMIZER_BASE_H */
