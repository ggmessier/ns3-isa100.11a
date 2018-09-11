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
 */

#include "isa-graph.h"
#include "ns3/log.h"
#include "ns3/type-id.h"
#include <algorithm>

NS_LOG_COMPONENT_DEFINE ("Isa100DownlinkGraph");

using namespace std;

namespace ns3 {

map <uint32_t, Ptr<IsaGraph>> IsaGraph::ReliableDownlinkGraphs (Ptr<IsaGraph> G)
{
  NS_LOG_FUNCTION (this);

  Ptr<IsaGraph> S = CreateObject<IsaGraph>();        ///< graph S to count the nodes already explored.

  map <uint32_t, Ptr<IsaGraph>> downlinkGraphs_G;               ///< Downlink graphs of the graph G

  // ***** begin ***** create Downlink graphs for all access points of the graph G
  Ptr<Node> gateWay = G->GetGateway();
  S->AddNode(gateWay);

  vector<uint32_t > tempAccessPoints = G->GetAccessPoints();

  while (!tempAccessPoints.empty ())
   {
     uint32_t nextAccessPointID = tempAccessPoints.back ();

     Ptr<IsaGraph> G_AP = CreateObject<IsaGraph> ();                      ///< Downlink graph creation for access points
     G_AP->AddNode(gateWay);
     G_AP->AddGateway(gateWay->GetId());                                  // Set gate way of the Downlink graph
     Ptr<Node> accessPoint = G->GetGraphNode(nextAccessPointID).m_head;
     S->AddNode(accessPoint);
     G_AP->AddNode(accessPoint);
     G_AP->AddAccessPoint(accessPoint->GetId());                          // Set access point of the Downlink graph
     G_AP->AddEdge(gateWay->GetId(), accessPoint->GetId());               // adding edge e(g,i)
     G_AP->SetHopCount(accessPoint->GetId(), 1);
     downlinkGraphs_G[accessPoint->GetId()] = G_AP;                       // Set Gi (Downlink graph for ith node) - For Access points
     downlinkGraphs_G[accessPoint->GetId()]->m_graphID = accessPoint->GetId();

     tempAccessPoints.pop_back ();
   }

  // ***** end ***** create Downlink graphs for all access points of the graph G

  map <uint32_t, GraphNode> edgesForS = S->m_graphNodeMap;                ///< vector to determine the number of parents of the respective node

  while (S->GetNumofNodes() < G->GetNumofNodes ())      // while S != V (Nodes of the Graph G)
    {
      GraphNode nodeMinHopPv;           ///< Graph Node with minimum hops from the gateway and satisfying the three conditions
      GraphNode nodeMinHopS_2;           ///< Graph Node with minimum hops from the gateway and satisfying the three conditions
      GraphNode nodeMinHopS_1;          ///< Graph Node with minimum hops from the gateway and which not satisfying the three conditions

      edgesForS = S->UpdateSVector (G, edgesForS);       // Nodes and edges that need to be consider for the Downlink graph creation

      // initially set the hop counts as the maximum possible +1
      uint32_t notPossibleHopCount = G->GetNumofNodes ()+1;
      nodeMinHopS_2.m_avgHopCount = notPossibleHopCount;
      nodeMinHopPv.m_avgHopCount = notPossibleHopCount;
      nodeMinHopS_1.m_avgHopCount = notPossibleHopCount;

      for (map<uint32_t, GraphNode>::const_iterator it = edgesForS.begin ();
             it != edgesForS.end (); ++it)
          {
            if (S->m_graphNodeMap.count (it->first) == 0)   // check whether node is already in the explored node list or not
              {
                GraphNode tempNode;                     // Node which having two parents considering at the moment (temporary Node v)
                vector<GraphNode> tempParents;     // temporary vector for  all the parents of the tempNode in S (explored Nodes)
                tempNode.m_head = it->second.m_head;

                for (uint32_t i = 0; i < (it->second).m_parents.size (); ++i)
                  {
                    uint32_t nextNode = (it->second).m_parents[i].operator -> ()->GetId ();
                    if (S->m_graphNodeMap.count (nextNode))
                      {
                        tempParents.push_back (S->m_graphNodeMap[nextNode]);
                      }
                  }

                if (tempParents.size () >= 2)              //nodes having at least two edges from S
                  {
                    //iterate over all the edges to find the edge pair that need to add to the Downlink graph of the respective node
                    vector<GraphNode> tempNew = tempParents;               ///< copy of temParents vector

                    while (!tempNew.empty())
                      {
                        GraphNode u1;
                        u1.m_head = tempNew.back().m_head;            // get the first parent of the node v
                        uint32_t u1Id = u1.m_head->GetId();
                        tempNew.pop_back();

                        tempNode.m_parents.push_back(u1.m_head);    // add parent u1 to the parents list

                        // consider u1 above with all other parents (u2)
                        for (vector<GraphNode>::const_iterator tempU2 = tempNew.begin ();
                             tempU2 != tempNew.end (); ++tempU2)
                          {
                            GraphNode u2;
                            u2.m_head = tempU2->m_head;
                            uint32_t u2Id = u2.m_head->GetId();
                            tempNode.m_parents.push_back(u2.m_head);
                            tempNode.m_avgHopCount = (downlinkGraphs_G[u1Id]->GetHopCount(u1Id) +
                                downlinkGraphs_G[u2Id]->GetHopCount(u2Id))/2;

                            if(G->C1Condition(u1.m_head, u2.m_head) && (G->C2Condition(u1.m_head, u2.m_head)
                                || G->C3Condition(u1.m_head, u2.m_head, downlinkGraphs_G)))
                              {
                                if(nodeMinHopPv.m_avgHopCount>tempNode.m_avgHopCount)
                                  {
                                    nodeMinHopPv = tempNode;
                                  }
                              }
                            else if (nodeMinHopS_2.m_avgHopCount>tempNode.m_avgHopCount)
                              {
                                nodeMinHopS_2 = tempNode;
                              }
                            tempNode.m_parents.pop_back();    //remove u2 from parents list to consider new parent
                          }
                        tempNode.m_parents.pop_back();      //remove u1 from parents list to consider new parent
                      }
                    // end of iteration over all edge pairs of v
                  }
                else if (tempParents.size () == 1)              //nodes having one edge from S
                  {
                    GraphNode u1;
                    u1.m_head = tempParents.back().m_head;                // get the first parent of the node v
                    uint32_t u1Id = u1.m_head->GetId();
                    tempNode.m_parents.push_back(u1.m_head);    // add parent u1 to the parents list

                    if(nodeMinHopS_1.m_avgHopCount>downlinkGraphs_G[u1Id]->GetHopCount(u1Id))
                      {
                        nodeMinHopS_1 = tempNode;
                        nodeMinHopS_1.m_avgHopCount = downlinkGraphs_G[u1Id]->GetHopCount(u1Id);    // setting the average hop count of parents to S_1 Node
                      }

                    tempNode.m_parents.pop_back();      //remove u1 from parents list to consider new parent
                  }
              }
          }// end of iterator of selected edges (edgesForS)

      if (nodeMinHopPv.m_avgHopCount < notPossibleHopCount)
        {
          nodeMinHopPv.m_reliability = true;            // C1 and (C2 or C3) condition satisfying. Therefore, Node is having reliable downlink graph
          nodeMinHopS_2 = nodeMinHopPv;
        }

      if(nodeMinHopS_2.m_avgHopCount < notPossibleHopCount)
        {
          nodeMinHopS_2.m_avgHopCount++;                  // hop count of u1u2 average considered for above algorithm, but hv is +1 to it.
          downlinkGraphs_G = (this)->ConstructDownlinkGraphs(G,nodeMinHopS_2, downlinkGraphs_G);
          S->AddNode(nodeMinHopS_2.m_head);
        }
      else if(nodeMinHopS_1.m_avgHopCount < notPossibleHopCount)
        {
          nodeMinHopS_1.m_avgHopCount++;                  // hop count of u1 considered for above algorithm, but hv is +1 to it.
          uint32_t nodeId = nodeMinHopS_1.m_head->GetId();
          uint32_t nodeParentId = nodeMinHopS_1.m_parents[0]->GetId();

          Ptr<IsaGraph> G_v = CreateObject<IsaGraph> ();                      ///< Downlink graph creation for the selected Node
          G_v.operator *() = downlinkGraphs_G[nodeParentId].operator *();
          G_v->AddGraphNode(nodeMinHopS_1);             // add graph node S_1
          G_v->AddEdge(nodeParentId, nodeId);           // Adding edge of u1 and v
          downlinkGraphs_G[nodeId] = G_v;               // Add Downlink graph to the graph list
          downlinkGraphs_G[nodeId]->m_graphID = nodeId;
          S->AddNode(nodeMinHopS_1.m_head);
        }

    } // end of while loop for S != V

  return downlinkGraphs_G;
}

map <uint32_t, Ptr<IsaGraph>> IsaGraph::ConstructDownlinkGraphs (Ptr<IsaGraph> G, GraphNode v, map <uint32_t, Ptr<IsaGraph>> downlinkGraphs)
{
  Ptr<IsaGraph> Gv = CreateObject<IsaGraph> ();                      ///< initial downlink graph (Gv) creation with v

  Ptr<Node> u1 = v.m_parents[0];            // first parent of the node v
  Ptr<Node> u2 = v.m_parents[1];            // second parent of the node v

  uint32_t vId = v.m_head->GetId();              // Id of the Node v
  uint32_t u1Id = u1->GetId();            // Id of the first parent of v
  uint32_t u2Id = u2->GetId();            // Id of the second parent of v

  vector<Ptr<Node>> u2Parents = downlinkGraphs[u2Id]->GetGraphNode(u2Id).m_parents;    // parents of node u1 in it's respective Downlink graph
  vector<Ptr<Node>> u1Parents = downlinkGraphs[u1Id]->GetGraphNode(u1Id).m_parents;    // parents of node u1 in it's respective Downlink graph
  uint32_t u1HopCount = downlinkGraphs[u1Id]->GetGraphNode(u1Id).m_avgHopCount;             // hop count of u1 in it's Downlink graph
  uint32_t u2HopCount = downlinkGraphs[u2Id]->GetGraphNode(u2Id).m_avgHopCount;             // hop count of u2 in it's Downlink graph

  /*  check u1 and u2 satisfy C1 and C2 conditions
   *
   *             u1 <   <--->X (in G)
   *             |   |       |
   *             |    >  u2 <
   *             |       |
   *              >  v  <
   */
  if (G->C1Condition(u1, u2) && G->C2Condition(u1, u2))
    {
      if (count(u2Parents.begin(),u2Parents.end(),u1) >= 1)
        {
          /*  Check u1-> u2 is in Gu2
           *
           *           u1<--->u2 (Gu2)
           *            |     |
           *             > v <
           */
          Gv.operator *() = downlinkGraphs[u2Id].operator *();
          Gv->AddGraphNode(v);                      //Add Node v
          Gv->AddEdge(u1Id, vId);                   //Add edge u1 ---> v
          Gv->AddEdge(u2Id, vId);                   //Add edge u2 ---> v
          Gv->AddEdge(u2Id, u1Id);                  //Add edge u2 ---> u1
          downlinkGraphs[vId] = Gv;
          downlinkGraphs[vId]->m_graphID = vId;
        }
      else
        {
          /*
           *     (Gu1) u1<--->u2
           *            |     |
           *             > v <
           */
          Gv.operator *() = downlinkGraphs[u1Id].operator *();
          Gv->AddGraphNode(v);                      //Add Node v
          Gv->AddNode(u2);                          //Add Node u2
          Gv->AddEdge(u1Id, vId);                   //Add edge u1 ---> v
          Gv->AddEdge(u2Id, vId);                   //Add edge u2 ---> v
          Gv->AddEdge(u2Id, u1Id);                  //Add edge u2 ---> u1
          Gv->AddEdge(u1Id, u2Id);                  //Add edge u1 ---> u2
          downlinkGraphs[vId] = Gv;
          downlinkGraphs[vId]->m_graphID = vId;
        }
    }
  /*  u1 (u2) has at least one parent from the cycle in Gu2 (Gu1)
   *
   *             x1   <--->  x2 (find x1 which is parent of u1 (u2))
   *           /    \        |
   *          |       >  u2 < (u1)
   *          |         | |
   *           > u1 <-->  |   (u2)
   *              |       |
   *               >  v  <
   *
   */
  else if (G->C1Condition(u1, u2) && G->C3Condition(u1, u2, downlinkGraphs))
    {
      for (vector<Ptr<Node>>::const_iterator it1 = u2Parents.begin ();
                it1 != u2Parents.end (); ++it1)
        {
          vector<Ptr<Node>> neighborsOfU2Parent = downlinkGraphs[u2Id]->
              GetGraphNode(it1->operator ->()->GetId()).m_neighbors;
          for (vector<Ptr<Node>>::const_iterator it2 = neighborsOfU2Parent.begin ();
                          it2 != neighborsOfU2Parent.end (); ++it2)
            {
              if (it2->operator ->() == u1)
                {
                  Gv.operator *() = downlinkGraphs[u2Id].operator *();
                  Gv->AddNode(u1);                                    //Add node u1 to Gv
                  Gv->AddEdge(it1->operator ->()->GetId(), u1Id);     //Add edge parent of u2 with u1
                  Gv->AddGraphNode(v);                           //Add Node v
                  Gv->AddEdge(u1Id, vId);                   //Add edge u1 ---> v
                  Gv->AddEdge(u2Id, vId);                   //Add edge u2 ---> v
                  Gv->AddEdge(u2Id, u1Id);                  //Add edge u2 ---> u1
                  Gv->AddEdge(u1Id, u2Id);                  //Add edge u1 ---> u2
                  downlinkGraphs[vId] = Gv;
                  downlinkGraphs[vId]->m_graphID = vId;
                }
            }
        }

      if (u2HopCount < u1HopCount)
        {
          for (vector<Ptr<Node>>::const_iterator it1 = u1Parents.begin ();
                                it1 != u1Parents.end (); ++it1)
            {
              vector<Ptr<Node>> neighborsOfU1Parent = downlinkGraphs[u1Id]->
                  GetGraphNode(it1->operator ->()->GetId()).m_neighbors;
              for (vector<Ptr<Node>>::const_iterator it2 = neighborsOfU1Parent.begin ();
                              it2 != neighborsOfU1Parent.end (); ++it2)
                {
                  if (it2->operator ->() == u2)
                    {
                      Gv.operator *() = downlinkGraphs[u1Id].operator *();
                      Gv->AddNode(u2);                                    //Add node u1 to Gv
                      Gv->AddEdge(it1->operator ->()->GetId(), u2Id);     //Add edge parent of u2 with u1
                      Gv->AddGraphNode(v);                           //Add Node v
                      Gv->AddEdge(u1Id, vId);                   //Add edge u1 ---> v
                      Gv->AddEdge(u2Id, vId);                   //Add edge u2 ---> v
                      Gv->AddEdge(u2Id, u1Id);                  //Add edge u2 ---> u1
                      Gv->AddEdge(u1Id, u2Id);                  //Add edge u1 ---> u2
                      downlinkGraphs[vId] = Gv;
                      downlinkGraphs[vId]->m_graphID = vId;
                    }
                }
            }
        }
    }
  else
    {
      bool edgeU1toU2 = (count(G->m_graphNodeMap[u1Id].m_neighbors.begin(),
                               G->m_graphNodeMap[u1Id].m_neighbors.end (), u2) >= 1);
      bool edgeU2toU1 = (count(G->m_graphNodeMap[u2Id].m_neighbors.begin(),
                               G->m_graphNodeMap[u2Id].m_neighbors.end (), u1) >= 1);
      if(edgeU1toU2 && edgeU2toU1)
        {
          if(u1HopCount < u2HopCount)
            {
              Gv.operator *() = downlinkGraphs[u1Id].operator *();
              Gv->AddNode(u2);                  //Add node u1 to Gv
            }
          else
            {
              Gv.operator *() = downlinkGraphs[u2Id].operator *();
              Gv->AddNode(u1);                  //Add node u1 to Gv
            }
          Gv->AddGraphNode(v);         //Add Node v
          Gv->AddEdge(u1Id, vId);                   //Add edge u1 ---> v
          Gv->AddEdge(u2Id, vId);                   //Add edge u2 ---> v
          Gv->AddEdge(u2Id, u1Id);                  //Add edge u2 ---> u1
          Gv->AddEdge(u1Id, u2Id);                  //Add edge u1 ---> u2
        }
      else if(!edgeU1toU2 && !edgeU2toU1)
        {
          if(u1HopCount < u2HopCount)
            {
              Gv.operator *() = downlinkGraphs[u1Id].operator *();
              v.m_parents.erase(v.m_parents.end());
              Gv->AddGraphNode(v);         //Add Node v
              Gv->AddEdge(u1Id, vId);                   //Add edge u1 ---> v
            }
          else
            {
              Gv.operator *() = downlinkGraphs[u2Id].operator *();
              v.m_parents.erase(v.m_parents.begin());
              Gv->AddGraphNode(v);         //Add Node v
              Gv->AddEdge(u2Id, vId);                   //Add edge u2 ---> v
            }
        }
      else if(edgeU1toU2)
        {
          Gv.operator *() = downlinkGraphs[u1Id].operator *();
          Gv->AddNode(u2);                                    //Add node u1 to Gv
          Gv->AddGraphNode(v);                           //Add Node v
          Gv->AddEdge(u1Id, vId);                   //Add edge u1 ---> v
          Gv->AddEdge(u2Id, vId);                   //Add edge u2 ---> v
          Gv->AddEdge(u1Id, u2Id);                  //Add edge u1 ---> u2
        }
      else
        {
          Gv.operator *() = downlinkGraphs[u2Id].operator *();
          Gv->AddNode(u1);                                    //Add node u1 to Gv
          Gv->AddGraphNode(v);                           //Add Node v
          Gv->AddEdge(u1Id, vId);                   //Add edge u1 ---> v
          Gv->AddEdge(u2Id, vId);                   //Add edge u2 ---> v
          Gv->AddEdge(u2Id, u1Id);                  //Add edge u2 ---> u1
        }
      downlinkGraphs[vId] = Gv;
      downlinkGraphs[vId]->m_graphID = vId;
    }
  return downlinkGraphs;
}

//*- C1 v has at least two parents u1, u2, and they form a cycle.
bool IsaGraph::C1Condition (Ptr<Node> u1, Ptr<Node> u2)
{
  NS_LOG_FUNCTION (this);

  uint32_t u1Id = u1->GetId();        ///< node Id of the first parent of the respective node v
  uint32_t u2Id = u2->GetId();        ///< node Id of the second parent of the respective node v

  GraphNode u1Gn = (this)->m_graphNodeMap[u1Id];

  // check u1 and u2 not representing the same node
  if (u1Id != u2Id)
    {
      /*  check (u2 is a neighbor of u1) and (u2 is a parent of u1)
       *
       *           u2<---->(u1)
       *
       */
      if (count(u1Gn.m_neighbors.begin(),u1Gn.m_neighbors.end(),u2) >= 1)
        {
          if (count(u1Gn.m_parents.begin(),u1Gn.m_parents.end(),u2) >= 1)
            {
              return true;
            }
        }
  }

  return false;
}

//*- C2 u1 is u2’s parent in u2’s "local" Downlink graph (Node and it's immediate parents)
bool IsaGraph::C2Condition (Ptr<Node> u1, Ptr<Node> u2)
{
  NS_LOG_FUNCTION (this);

  uint32_t u1Id = u1->GetId();        ///< node Id of the first parent of the respective node v
  uint32_t u2Id = u2->GetId();        ///< node Id of the second parent of the respective node v

  // check u1 and u2 not representing the same node
  if (u1Id != u2Id)
    {
      /*  check u1 is a parent of u2's local Downlink graph in G
       *
       *           x1<---->x2 (check if x1 of x2 is u1)
       *            |      |
       *             > u2 <
       */
      vector<Ptr<Node>> u2Parents = (this)->GetGraphNode(u2Id).m_parents;

      if (count(u2Parents.begin(),u2Parents.end(),u1) >= 1)
        {
          return true;
        }
    }

  return false;
}

//*- C3 u2 (u1) has at least one parent from the cycle in Gu1 (Gu2)
bool IsaGraph::C3Condition (Ptr<Node> u1, Ptr<Node> u2, map <uint32_t, Ptr<IsaGraph>> downlinkGraphs)
{
  NS_LOG_FUNCTION (this);

  uint32_t u1Id = u1->GetId();       ///< node Id of the first parent of the respective node v
  uint32_t u2Id = u2->GetId();       ///< node Id of the second parent of the respective node v

  GraphNode u1Gn = (this)->m_graphNodeMap[u1Id];
  GraphNode u2Gn = (this)->m_graphNodeMap[u2Id];

  // check u1 and u2 not representing the same node
  if (u1Id != u2Id)
    {
      /*  u1 has at least one parent from the cycle in Gu2
       *
       *           x1  <--->x2 (check if x1 is parent of u1)
       *          |  |      |
       *          |   > u2 <
       *          |
       *           > u1
       */
      vector<Ptr<Node>> u2Parents = downlinkGraphs[u2Id]->GetGraphNode(u2Id).m_parents;

      for (vector<Ptr<Node>>::const_iterator it = u2Parents.begin ();
                it != u2Parents.end (); ++it)
        {
          if (count(u1Gn.m_parents.begin(),u1Gn.m_parents.end(),it->operator ->()) >= 1)
            {
              return true;
            }
        }

      /*  u2 has at least one parent from the cycle in Gu1
       *
       *           x1  <--->x2 (check if x1 is parent of u2)
       *          |  |      |
       *          |   > u1 <
       *          |
       *           > u2
       */
      vector<Ptr<Node>> u1Parents = downlinkGraphs[u1Id]->GetGraphNode(u1Id).m_parents;

      for (vector<Ptr<Node>>::const_iterator it = u1Parents.begin ();
                it != u1Parents.end (); ++it)
        {
          if (count(u2Gn.m_parents.begin(),u2Gn.m_parents.end(),it->operator ->()) >= 1)
            {
              return true;
            }
        }
    }

  return false;
}

} // namespace ns3


