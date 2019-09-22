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


NS_LOG_COMPONENT_DEFINE ("GraphTdmaOptimzer");

using namespace ns3;
using namespace std;

NS_OBJECT_ENSURE_REGISTERED (GraphTdmaOptimzer);

TypeId GraphTdmaOptimzer::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::GraphTdmaOptimzer")
    .SetParent<TdmaOptimizerBase> ()
    .AddConstructor<GraphTdmaOptimzer> ()

  ;

  return tid;
}

GraphTdmaOptimzer::GraphTdmaOptimzer () : TdmaOptimizerBase ()
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
  TdmaOptimizerBase::SetupOptimization (c, propModel);

  // initial graph creation based on all available links
  TdmaOptimizerBase::GraphCreation (c);

}

vector< vector<int> > GraphTdmaOptimzer::SolveTdma (void)
{
  NS_LOG_FUNCTION (this);

  vector< vector<int> > flows (m_numNodes);

  Ptr<Node> gateWay = m_graph->GetGateway();
  Ptr<Node> acessPoint_1 = m_graph->GetGraphNode(1).m_head;
  Ptr<Node> acessPoint_2 = m_graph->GetGraphNode(2).m_head;

  //Initialize the reliable broadcast graph
  NodeContainer nc;
  nc.Add (gateWay);
  nc.Add (acessPoint_1);
  nc.Add (acessPoint_2);

  Ptr<IsaGraph> G_B = CreateObject<IsaGraph> (nc);      ///< Broadcast graph creation

  G_B->AddGateway (gateWay->GetId ());
  G_B->AddAccessPoint (acessPoint_1->GetId ());
  G_B->AddAccessPoint (acessPoint_2->GetId ());

  G_B->AddEdge (gateWay->GetId (),acessPoint_1->GetId ());
  G_B->AddEdge (gateWay->GetId (),acessPoint_2->GetId ());
  G_B->SetHopCount (gateWay->GetId (),0);
  G_B->SetHopCount (acessPoint_1->GetId (),1);
  G_B->SetHopCount (acessPoint_2->GetId (),1);

  Ptr<IsaGraph> G_U = CreateObject<IsaGraph> (nc);      ///< Uplink graph creation

  G_U->AddGateway (gateWay->GetId ());
  G_U->AddAccessPoint (acessPoint_1->GetId ());
  G_U->AddAccessPoint (acessPoint_2->GetId ());

  G_U->AddEdge (acessPoint_1->GetId (), gateWay->GetId ());
  G_U->AddEdge (acessPoint_2->GetId (), gateWay->GetId ());
  G_U->SetHopCount (gateWay->GetId (),0);
  G_U->SetHopCount (acessPoint_1->GetId (),1);
  G_U->SetHopCount (acessPoint_2->GetId (),1);

  G_U->ReliableUplinkGraph (m_graph);

  m_graphMap[0] = G_U;

  return flows;
}


