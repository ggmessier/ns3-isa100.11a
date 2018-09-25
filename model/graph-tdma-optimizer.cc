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
//#include "ns3/isa-graph.h"

#include <ilcplex/ilocplex.h>
#include <algorithm>


NS_LOG_COMPONENT_DEFINE ("GraphTdmaOptimzer");

using namespace ns3;
using namespace std;

//struct nodeElement {
//  std::vector<NetworkLink *> inLinks;
//  std::vector<NetworkLink *> outLinks;
//};

NS_OBJECT_ENSURE_REGISTERED (GraphTdmaOptimzer);

TypeId GraphTdmaOptimzer::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::GraphTdmaOptimzer")
    .SetParent<TdmaOptimizerBase> ()
    .AddConstructor<GraphTdmaOptimzer> ()

    ;

  return tid;
}

GraphTdmaOptimzer::GraphTdmaOptimzer () : TdmaOptimizerBase()
{
  NS_LOG_FUNCTION (this);
  m_numNodes = 0;
}

GraphTdmaOptimzer::~GraphTdmaOptimzer ()
{
  NS_LOG_FUNCTION (this);

}

void GraphTdmaOptimzer::SetupOptimization (NodeContainer c, Ptr<PropagationLossModel> propModel)
{
  NS_LOG_FUNCTION (this);

  // Setup the common base properties
  TdmaOptimizerBase::SetupOptimization(c, propModel);

}

void GraphTdmaOptimzer::GraphCreation(NodeContainer c)
{
  NS_LOG_FUNCTION (this);

  Ptr<IsaGraph> G = CreateObject<IsaGraph>(c);

  G->AddGateway(0);

  // Rajith:: code required to modify to support any number of access points
  G->AddAccessPoint(1);
  G->AddAccessPoint(2);

  uint16_t numNodes = c.GetN();
  TimeValue slotDurationV;
  UintegerValue tempNumSlotsV;
  int numSlotsV;

  for (uint16_t parent = 0; parent < numNodes; parent++)
    {

      Ptr<Isa100NetDevice> devPtr = c.Get(parent)->GetDevice(0)->GetObject<Isa100NetDevice>();
      devPtr->GetDl()->GetAttribute("SuperFrameSlotDuration", slotDurationV);
      m_slotDuration = slotDurationV.Get();

      devPtr->GetDl()->GetAttribute("SuperFramePeriod", tempNumSlotsV);
      numSlotsV = tempNumSlotsV.Get();

      G->GetGraphNodeMap()[parent].m_numTimeSlots = numSlotsV;

      for (uint16_t nNode = 0; nNode < numNodes; nNode++)
        {
          if(parent != nNode && m_txPowerDbm[parent][nNode] <= m_maxTxPowerDbm)
            {
              G->AddEdge(parent, nNode);
            }
        }
    }

  m_graph = G;

  Ptr<Node> gateWay = c.Get(0);
  Ptr<Node> acessPoint_1 = c.Get(1);
  Ptr<Node> acessPoint_2 = c.Get(2);

  //Initialize the reliable broadcast graph
  NodeContainer nc;
  nc.Add(gateWay);
  nc.Add(acessPoint_1);
  nc.Add(acessPoint_2);

  Ptr<IsaGraph> G_B = CreateObject<IsaGraph>(nc);      ///< Broadcast graph creation

  G_B->AddGateway(gateWay->GetId());
  G_B->AddAccessPoint(acessPoint_1->GetId());
  G_B->AddAccessPoint(acessPoint_2->GetId());

  G_B->AddEdge(gateWay->GetId(),acessPoint_1->GetId());
  G_B->AddEdge(gateWay->GetId(),acessPoint_2->GetId());
  G_B->SetHopCount(gateWay->GetId(),0);
  G_B->SetHopCount(acessPoint_1->GetId(),1);
  G_B->SetHopCount(acessPoint_2->GetId(),1);

  Ptr<IsaGraph> G_U = CreateObject<IsaGraph>(nc);      ///< Uplink graph creation

  G_U.operator *() = G_B.operator *();

//  bool reliableGraphB =
  G_B->ReliableBroadcastGraph(G);
//  bool reliableGraphU =
  G_U->ReliableUplinkGraph(G);

  map <uint32_t, Ptr<IsaGraph>> downlinkGraphs = G->ReliableDownlinkGraphs(G);
  downlinkGraphs[65535] = G_B;
  downlinkGraphs[0] = G_U;

  m_graphMap = downlinkGraphs;
}

vector< vector<int> > GraphTdmaOptimzer::SolveTdma (void)
{
  NS_LOG_FUNCTION (this);

  vector< vector<int> > flows(m_numNodes);

  return flows;

}


