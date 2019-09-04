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
      header.SetSeqNum (m_nextSeqNum);
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

void Isa100SourceRoutingAlgorithm::SetGraphTable (std::map<Mac16Address, Mac16Address > graphTable)
{

}

Mac16Address Isa100SourceRoutingAlgorithm::NextNeighbor (Mac16Address graphID)
{
  return Mac16Address ("ff:ff");
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

Isa100GraphRoutingAlgorithm::Isa100GraphRoutingAlgorithm (std::map<uint32_t, std::vector<std::vector<Mac16Address> > > initTable)
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

  if (m_table[destNodeInd].size () == 1)
    {
      m_counter = 0;
    }

  int subCounter = m_counter;
  uint8_t iHop = 0;
  while (iHop < m_table[destNodeInd][subCounter].size ())
    {
      header.SetGraphRouteHop (iHop,m_table[destNodeInd][subCounter][iHop]);
      iHop++;
    }

  // if initial packet set the sequence number to current next available sequence number (This need to move to DL layer)
  bool initialPacket = (m_address == header.GetDaddrSrcAddress ());
  if (initialPacket)
    {
      header.SetSeqNum (m_nextSeqNum);
      m_nextSeqNum++;
    }

  // Set header for first hop.
  header.SetSrcAddrFields (0,m_address);
  header.SetDstAddrFields (0,NextNeighbor (header.GetGraphRouteHop (0)));

  m_counter++;
  if (m_counter >= m_table[destNodeInd].size ())
    {
      m_counter = 0;
    }

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

      Mac16Address nextHopGraph = header.PopNextSourceRoutingHop ();

      header.SetSrcAddrFields (0,m_address);
      header.SetDstAddrFields (0,NextNeighbor (nextHopGraph));
    }

  NS_LOG_DEBUG ("RX First Source Address Before AAdd Header: " << header.GetDaddrSrcAddress ());
  // Return the modified header to the packet.
  packet->AddHeader (header);

  NS_LOG_DEBUG (" Output packet " << *packet);

}

void Isa100GraphRoutingAlgorithm::SetGraphTable (std::map<Mac16Address, Mac16Address > graphTable)
{
  m_graphTable = graphTable;
}

Mac16Address Isa100GraphRoutingAlgorithm::NextNeighbor (Mac16Address graphID)
{
  NS_LOG_FUNCTION (this);
  Mac16Address nodeAddr = Mac16Address ("ff:ff");

  if (m_graphTable.count (graphID))
    {
      nodeAddr = m_graphTable[graphID];
    }

  return nodeAddr;
}

NS_OBJECT_ENSURE_REGISTERED (Isa100MinLoadRoutingAlgorithm);

// --- Isa100GraphRoutingAlgorithm ---
TypeId Isa100MinLoadRoutingAlgorithm::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::Isa100MinLoadRoutingAlgorithm")
    .SetParent<Isa100RoutingAlgorithm> ()
    .AddConstructor<Isa100MinLoadRoutingAlgorithm> ()

  ;

  return tid;
}

Isa100MinLoadRoutingAlgorithm::Isa100MinLoadRoutingAlgorithm (std::map<uint32_t, std::vector<std::vector<Mac16Address> > > initTable)
  : Isa100RoutingAlgorithm ()
{
  NS_LOG_FUNCTION (this);

  m_table = initTable;
  m_nextSeqNum = 0;

}

Isa100MinLoadRoutingAlgorithm::Isa100MinLoadRoutingAlgorithm ()
  : Isa100RoutingAlgorithm ()
{
}

Isa100MinLoadRoutingAlgorithm::~Isa100MinLoadRoutingAlgorithm ()
{
}

void Isa100MinLoadRoutingAlgorithm::PrepTxPacketHeader (Isa100DlHeader &header)
{
  NS_LOG_FUNCTION (this);

  uint8_t buffer[4];
  Mac16Address addr = header.GetDaddrDestAddress ();
  addr.CopyTo (buffer);

  // Populate DROUT sub-header.
  uint8_t destNodeInd = buffer[1];

  // iterate over the table to set the graph routing Hop sequence
  int iHop = 0;
  for (uint32_t i = 0; i < m_table[destNodeInd].size (); i++)
    {
      for (uint32_t j = 0; j < m_table[destNodeInd].size (); j++)
        {
          header.SetGraphRouteHop (iHop,m_table[destNodeInd][i][j]);
          iHop++;
        }
    }

  // if initial packet set the sequence number to current next available sequence number (This need to move to DL layer)
  bool initialPacket = (m_address == header.GetDaddrSrcAddress ());
  if (initialPacket)
    {
      header.SetSeqNum (m_nextSeqNum);
      m_nextSeqNum++;
    }

  // Set header for first hop.
  header.SetSrcAddrFields (0,m_address);
  header.SetDstAddrFields (0,NextNeighbor (header.GetGraphRouteHop (0)));

}

void Isa100MinLoadRoutingAlgorithm::ProcessRxPacket (Ptr<Packet> packet, bool &forwardPacketOn)
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

      Mac16Address nextHopGraph = header.PopNextSourceRoutingHop ();

      header.SetSrcAddrFields (0,m_address);
      header.SetDstAddrFields (0,NextNeighbor (nextHopGraph));
    }

  NS_LOG_DEBUG ("RX First Source Address Before AAdd Header: " << header.GetDaddrSrcAddress ());
  // Return the modified header to the packet.
  packet->AddHeader (header);

  NS_LOG_DEBUG (" Output packet " << *packet);

}


void Isa100MinLoadRoutingAlgorithm::SetGraphTable (std::map<Mac16Address, Mac16Address > graphTable)
{
  m_graphTable = graphTable;
}


Mac16Address Isa100MinLoadRoutingAlgorithm::NextNeighbor (Mac16Address graphID)
{
  NS_LOG_FUNCTION (this);
  Mac16Address nodeAddr = Mac16Address ("ff:ff");

  if (m_graphTable.count (graphID))
    {
      nodeAddr = m_graphTable[graphID];
    }

  return nodeAddr;
}

} // namespace ns3
