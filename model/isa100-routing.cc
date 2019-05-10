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
 * Authors: Geoffrey Messier <gmessier@ucalgary.ca>
 *          Michael Herrmann <mjherrma@ucalgary.ca>
 */

#include "ns3/isa100-routing.h"

#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"
#include "ns3/integer.h"
#include "ns3/double.h"
#include "ns3/packet.h"
#include "ns3/node-container.h"

#include "ns3/isa100-dl-header.h"
#include "ns3/isa100-dl-trailer.h"
#include "ns3/isa100-net-device.h"
#include "ns3/isa100-dl.h"
#include "ns3/zigbee-trx-current-model.h"
#include "ns3/isa100-battery.h"

#include <iomanip>

NS_LOG_COMPONENT_DEFINE ("Isa100Routing");

namespace ns3 {


// --- Isa100RoutingAlgorithm Base Class ---

NS_OBJECT_ENSURE_REGISTERED (Isa100RoutingAlgorithm);

TypeId Isa100RoutingAlgorithm::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::Isa100RoutingAlgorithm")
    .SetParent<Object> ()

    .AddAttribute ("Address","16 bit DL address of node.",
                   Mac16AddressValue (),
                   MakeMac16AddressAccessor (&Isa100RoutingAlgorithm::m_address),
                   MakeMac16AddressChecker ())
  ;

  return tid;
}


Isa100RoutingAlgorithm::Isa100RoutingAlgorithm ()
{
//  m_routingMethod = SOURCE;
}

Isa100RoutingAlgorithm::~Isa100RoutingAlgorithm ()
{
}

Mac16Address
Isa100RoutingAlgorithm::AttemptAnotherLink (uint8_t destInd, std::vector<Mac16Address> attemptedLinks)
{
  return Mac16Address ("ff:ff");
}
//
//void Isa100RoutingAlgorithm::SetRoutingMethod(RoutingMethod routingMethod)
//{
//  m_routingMethod = routingMethod;
//}


NS_OBJECT_ENSURE_REGISTERED (Isa100SourceRoutingAlgorithm);

// --- Isa100SourceRoutingAlgorithm ---
TypeId Isa100SourceRoutingAlgorithm::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::Isa100SourceRoutingAlgorithm")
    .SetParent<Isa100RoutingAlgorithm> ()
    .AddConstructor<Isa100SourceRoutingAlgorithm> ()

  ;

  return tid;
}



Isa100SourceRoutingAlgorithm::Isa100SourceRoutingAlgorithm (uint32_t numDests, std::string *initTable)
  : Isa100RoutingAlgorithm (),m_numDests (numDests)
{
  NS_LOG_FUNCTION (this);

  m_numHops = new uint32_t[m_numDests];
  m_table = new Mac16Address*[m_numDests];

  std::string addressStr;

  for (uint32_t iDest = 0; iDest < m_numDests; iDest++)
    {

      NS_LOG_DEBUG (" Dest: " << iDest);

      // Determine the number of routing table entries by counting the semicolons
      int numEntries = 0;
      int startSearch = 0;
      while ( ( startSearch = initTable[iDest].find (":",startSearch + 1) ) != std::string::npos)
        {
          numEntries++;
        }

      NS_LOG_DEBUG (" Num Hop Entries: " << numEntries);

      // Each address string is 6 chars (address plus space) except the last one which is just address.
      m_table[iDest] = new Mac16Address[numEntries];

      uint32_t iEnd, iHop = 0, iStart = 0;
      for (iEnd = 0; iEnd < initTable[iDest].length (); iEnd++)
        {

          if (initTable[iDest][iEnd] == ' ')
            {

              addressStr = initTable[iDest].substr (iStart,iEnd - iStart);
              NS_LOG_DEBUG ("  Hop: " << addressStr);

              m_table[iDest][iHop++] = Mac16Address ( addressStr.c_str () );
              iStart = iEnd + 1;
            }
        }

      addressStr = initTable[iDest].substr (iStart,std::string::npos);

      NS_LOG_DEBUG ("  Hop: " << addressStr);
      m_table[iDest][iHop++] = Mac16Address ( addressStr.c_str () );

      m_numHops[iDest] = iHop;

      NS_LOG_DEBUG ("  Total Hops: " << m_numHops[iDest]);
    }
}

Isa100SourceRoutingAlgorithm::Isa100SourceRoutingAlgorithm ()
  : Isa100RoutingAlgorithm ()
{
  m_table = 0;
  m_numDests = 0;
  m_numHops = 0;
}

Isa100SourceRoutingAlgorithm::~Isa100SourceRoutingAlgorithm ()
{
  for (uint32_t iDel = 0; iDel < m_numDests; iDel++)
    {
      delete[] m_table[iDel];
    }
  delete[] m_table;
  delete[] m_numHops;
}


void Isa100SourceRoutingAlgorithm::PrepTxPacketHeader (Isa100DlHeader &header)
{
  NS_LOG_FUNCTION (this);

  uint8_t buffer[4];
  Mac16Address addr = header.GetDaddrDestAddress ();
  addr.CopyTo (buffer);

  // Populate DROUT sub-header.
  uint8_t destNodeInd = buffer[1];
  NS_LOG_DEBUG (" Sending to node " << static_cast<uint16_t> (destNodeInd));

  for (uint32_t iHop = 0; iHop < m_numHops[destNodeInd]; iHop++)
    {
      header.SetSourceRouteHop (iHop,m_table[destNodeInd][iHop]);
    }

  bool initialPacket = (m_address == header.GetDaddrSrcAddress ());
  if (initialPacket)
    {
      header.SetSeqNum(m_nextSeqNum);
      m_nextSeqNum++;
    }

  // Set header for first hop.
  header.SetSrcAddrFields (0,m_address);
  header.SetDstAddrFields (0,m_table[destNodeInd][0]);

}

void Isa100SourceRoutingAlgorithm::ProcessRxPacket (Ptr<Packet> packet, bool &forwardPacketOn)
{
  NS_LOG_FUNCTION (this << m_address);

  NS_LOG_DEBUG (" Input packet " << *packet);

  // Remove the header so that it can be modified.
  Isa100DlHeader header;
  packet->RemoveHeader (header);

  Mac16Address finalDestAddr = header.GetDaddrDestAddress ();
  Mac16Address nextHopAddr = header.PopNextSourceRoutingHop ();

  NS_LOG_DEBUG (" Final Dest Addr: " << finalDestAddr << ", Next Hop Addr: " << nextHopAddr);

  forwardPacketOn = (m_address != finalDestAddr);

  // Set MHR source and destination addresses for next hop.
  if (forwardPacketOn)
    {
      header.SetSrcAddrFields (0,m_address);
      header.SetDstAddrFields (0,nextHopAddr);
    }

  // Return the modified header to the packet.
  packet->AddHeader (header);

  NS_LOG_DEBUG (" Output packet " << *packet);

}

NS_OBJECT_ENSURE_REGISTERED (Isa100GraphRoutingAlgorithm);

// --- Isa100GraphRoutingAlgorithm ---
TypeId Isa100GraphRoutingAlgorithm::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::Isa100GraphRoutingAlgorithm")
    .SetParent<Isa100RoutingAlgorithm> ()
    .AddConstructor<Isa100GraphRoutingAlgorithm> ()

  ;

  return tid;
}

//Isa100GraphRoutingAlgorithm::Isa100GraphRoutingAlgorithm (std::map<uint32_t, std::vector<Mac16Address> > initTable)
//  : Isa100RoutingAlgorithm ()
//{
//  NS_LOG_FUNCTION (this);
//
//  m_table = initTable;
//  m_nextSeqNum = 0;
//  m_counter = 1;
//
//}

Isa100GraphRoutingAlgorithm::Isa100GraphRoutingAlgorithm (std::map<uint32_t, std::vector<std::vector<Mac16Address>>> initTable)
  : Isa100RoutingAlgorithm ()
{
  NS_LOG_FUNCTION (this);

  m_table = initTable;
  m_nextSeqNum = 0;
  m_counter = 1;

}

Isa100GraphRoutingAlgorithm::Isa100GraphRoutingAlgorithm ()
  : Isa100RoutingAlgorithm ()
{
}

Isa100GraphRoutingAlgorithm::~Isa100GraphRoutingAlgorithm ()
{
}

void Isa100GraphRoutingAlgorithm::PrepTxPacketHeader (Isa100DlHeader &header)
{
  NS_LOG_FUNCTION (this);

  uint8_t buffer[4];
  Mac16Address addr = header.GetDaddrDestAddress ();
  addr.CopyTo (buffer);

  // Populate DROUT sub-header.
  uint8_t destNodeInd = buffer[1];
//  NS_LOG_UNCOND (m_address<<" Sending to node " << static_cast<uint32_t> (destNodeInd));

//  for(uint32_t iHop=0; iHop < m_table[destNodeInd].size(); iHop++){
//      NS_LOG_UNCOND("***** m_table size: "<<m_table[destNodeInd].size()<<" cg: "<<m_table[destNodeInd][iHop]<<" nextNode: "<<NextNeighbor(m_table[destNodeInd][iHop]));
////      NS_LOG_UNCOND("cg: "<<m_table[destNodeInd][iHop]);
////      NS_LOG_UNCOND("nextNode: "<<NextNeighbor(m_table[destNodeInd][iHop]));
//  }

//  int subCounter = m_counter;
//  int iHop = 0;
//  while (iHop < m_table[destNodeInd].size())
//    {
//      header.SetGraphRouteHop(iHop++,m_table[destNodeInd][subCounter]);
////      NS_LOG_UNCOND("m_table size: "<<m_table[destNodeInd].size()<<" cg: "<<m_table[destNodeInd][subCounter]<<
////                    " nextNode: "<<NextNeighbor(m_table[destNodeInd][subCounter])<<
////                    " iHop "<<iHop);
//      if(++subCounter >=  m_table[destNodeInd].size())
//        subCounter = 0;
//    }
  int subCounter = m_counter;
  int iHop = 0;
  while (iHop < m_table[destNodeInd][subCounter].size())
    {
//      NS_LOG_UNCOND("iHop "<<iHop);
      header.SetGraphRouteHop(iHop,m_table[destNodeInd][subCounter][iHop]);
//      NS_LOG_UNCOND("table2 graph route "<<m_table2[destNodeInd][subCounter][iHop]);
      iHop++;
    }
//  for (unsigned int i = 0; i < m_table2[destNodeInd][subCounter].size(); i++)
//    {
//      header.SetGraphRouteHop(iHop++,m_table2[destNodeInd][subCounter][i]);
//    }
//      header.SetGraphRouteHop(iHop++,m_table[destNodeInd][subCounter]);
//      NS_LOG_UNCOND("m_table size: "<<m_table[destNodeInd].size()<<" cg: "<<m_table[destNodeInd][subCounter]<<
//                    " nextNode: "<<NextNeighbor(m_table[destNodeInd][subCounter])<<
//                    " iHop "<<iHop);
//      if(++subCounter >=  m_table2[destNodeInd].size())
//        subCounter = 0;
//    }

  bool initialPacket = (m_address == header.GetDaddrSrcAddress ());
  if (initialPacket)
    {
      header.SetSeqNum(m_nextSeqNum);
      m_nextSeqNum++;
    }

  // Set header for first hop.
  header.SetSrcAddrFields (0,m_address);
  header.SetDstAddrFields (0,NextNeighbor(header.GetGraphRouteHop(0)));

//  NS_LOG_UNCOND("NextNeighbor(header.GetGraphRouteHop(0)  "<<NextNeighbor(header.GetGraphRouteHop(0)));

//  m_counter += 2;
  m_counter ++;
  if(m_counter >= m_table[destNodeInd].size())
    m_counter = 0;

}

void Isa100GraphRoutingAlgorithm::ProcessRxPacket (Ptr<Packet> packet, bool &forwardPacketOn)
{
  NS_LOG_FUNCTION (this << m_address);

  NS_LOG_DEBUG (" Input packet " << *packet);

  // Remove the header so that it can be modified.
  Isa100DlHeader header;
  packet->RemoveHeader (header);

  Mac16Address finalDestAddr = header.GetDaddrDestAddress ();

  NS_LOG_DEBUG ("RX First Source Address: " << header.GetDaddrSrcAddress ());

  forwardPacketOn = (m_address != finalDestAddr);

  // Set MHR source and destination addresses for next hop.
  if (forwardPacketOn)
    {
      NS_LOG_DEBUG ("RX forwardPacketOn: YES");
      uint8_t buffer[4];
      finalDestAddr.CopyTo (buffer);

      uint8_t destNodeInd = buffer[1];
      NS_LOG_DEBUG ("RX -> TX SetSourceRouteHop to node " << static_cast<uint32_t> (destNodeInd));

//      for(uint32_t iHop=0; iHop < m_table[destNodeInd].size(); iHop++)
//        {
//          NS_LOG_UNCOND("m_table size: "<<m_table[destNodeInd].size()<<" cg: "<<m_table[destNodeInd][iHop]<<" nextNode: "<<NextNeighbor(m_table[destNodeInd][iHop]));
//          header.SetGraphRouteHop(iHop,m_table[destNodeInd][iHop]);
//        }

//      NS_LOG_UNCOND("RX counter: "<<m_counter);

//          NS_LOG_UNCOND("RX header.GetGraphRouteHop: "<<header.GetGraphRouteHop(iHop));

//      for(uint32_t iHop=0; iHop < header.GetNumOfGraphRouteHop(); iHop++)
//        {
//          NS_LOG_UNCOND("RX hops: "<<to_string(header.GetNumOfGraphRouteHop())<<" "<<header.GetGraphRouteHop(iHop));
//        }

      Mac16Address nextHopGraph = header.PopNextSourceRoutingHop ();
//      NS_LOG_UNCOND("nextHopGraph: "<<nextHopGraph);
//      header.SetGraphRouteHop(header.GetNumOfGraphRouteHop()-1, nextHopGraph);

//      if(m_counter < m_table[destNodeInd].size())
//        m_counter++;
//      else
//        m_counter = 0;

      header.SetSrcAddrFields (0,m_address);
//      header.SetDstAddrFields (0,NextNeighbor(m_table[destNodeInd][m_counter]));
//      header.SetDstAddrFields (0,NextNeighbor(header.GetGraphRouteHop(0)));
      header.SetDstAddrFields (0,NextNeighbor(nextHopGraph));

//      if(m_counter < m_table[destNodeInd].size())
//        m_counter++;
//      else
//        m_counter = 0;
    }

  NS_LOG_DEBUG ("RX First Source Address Before AAdd Header: " << header.GetDaddrSrcAddress ());
  // Return the modified header to the packet.
  packet->AddHeader (header);

  NS_LOG_DEBUG (" Output packet " << *packet);

}

void Isa100GraphRoutingAlgorithm::DeleteTableEntry (Mac16Address nodeAddress)
{
  NS_LOG_FUNCTION (this << m_address);

//  NS_LOG_DEBUG("Isa100GraphRoutingAlgorithm::DeleteTableEntry: "<<nodeAddress);
//  NS_LOG_DEBUG("table size: "<<m_table.size ());
//
//  for (uint32_t tableEntry = 0; tableEntry < m_table.size (); tableEntry++)
//    {
//      NS_LOG_DEBUG("m_table[tableEntry] size: "<<m_table[tableEntry].size ());
//      for (std::vector<Mac16Address>::const_iterator it = m_table[tableEntry].begin ();
//           it != m_table[tableEntry].end (); ++it)
//        {
//          NS_LOG_DEBUG("iterator Address: "<<it->Allocate());
//          if (nodeAddress == it->Allocate())
//            {
//              m_table[tableEntry].erase (it);
//            }
//        }
//    }
}

void Isa100SourceRoutingAlgorithm::DeleteTableEntry (Mac16Address nodeAddress)
{
  NS_LOG_FUNCTION (this << m_address);

}

//void Isa100GraphRoutingAlgorithm::SetGraphTable(map<Mac16Address, pair<Mac16Address, Mac16Address>> graphTable)
//{
//  m_graphTable = graphTable;
//}

void Isa100GraphRoutingAlgorithm::SetGraphTable(std::map<Mac16Address, std::pair<Mac16Address, std::vector<Mac16Address>>> graphTable)
{
  m_graphTable = graphTable;
}

//void Isa100GraphRoutingAlgorithm::SetTable(std::map<uint32_t, std::vector<std::vector<Mac16Address>>> table)
//{
//  m_table2 = table;
//}

//Mac16Address Isa100GraphRoutingAlgorithm::NextGraphID (Mac16Address graphID)
//{
//  NS_LOG_FUNCTION (this);
//  Mac16Address graph = Mac16Address ("FF:FF");
//
//  if(m_graphTable.count(graphID))
//    {
//      graph = m_graphTable[graphID].first;
//    }
//
//  return graph;
//}

vector<Mac16Address> Isa100GraphRoutingAlgorithm::BackUpGraphSequence (Mac16Address graphID)
{
  NS_LOG_FUNCTION (this);
  vector<Mac16Address> backUpGraphSequence {Mac16Address ("ff:ff")};

  if(m_graphTable.count(graphID))
    {
      backUpGraphSequence = m_graphTable[graphID].second;
    }
  return backUpGraphSequence;
}


Mac16Address Isa100GraphRoutingAlgorithm::NextNeighbor (Mac16Address graphID)
{
  NS_LOG_FUNCTION (this);
  Mac16Address nodeAddr = Mac16Address ("ff:ff");

  if(m_graphTable.count(graphID))
    {
      nodeAddr = m_graphTable[graphID].first;
    }

//  NS_LOG_UNCOND("m_address: "<<m_address<<" cGraph: "<<graphID<<" nextNodeAddr: "<<nodeAddr);
  return nodeAddr;
}

void Isa100SourceRoutingAlgorithm::SetGraphTable(std::map<Mac16Address, std::pair<Mac16Address, std::vector<Mac16Address>>> graphTable)
{

}

//void Isa100SourceRoutingAlgorithm::SetTable(std::map<uint32_t, std::vector<std::vector<Mac16Address>>> table)
//{
//
//}

vector<Mac16Address> Isa100SourceRoutingAlgorithm::BackUpGraphSequence (Mac16Address graphID)
{
  vector<Mac16Address> backUpGraphSequence {Mac16Address ("ff:ff")};
  return backUpGraphSequence;
}

Mac16Address Isa100SourceRoutingAlgorithm::NextNeighbor (Mac16Address graphID)
{
  return Mac16Address ("ff:ff");
}

} // namespace ns3
