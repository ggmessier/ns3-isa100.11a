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
 * Authors:
 * Geoffrey Messier <gmessier@ucalgary.ca>
 * Michael Herrmann <mjherrma@ucalgary.ca>
 */


#ifndef ISA100_ROUTING_H
#define ISA100_ROUTING_H

#include "ns3/object.h"
#include "ns3/mac16-address.h"
#include <map>

namespace ns3 {
class NodeContainer;
class Packet;
class Isa100NetDevice;
class Isa100DlHeader;

///** Next Hop information for the graph routing.
// *
// */
//typedef struct{
//  uint16_t nextGraphID;  ///< current graph ID
//  Mac16Address nextNeighbor;  ///< neighbors for the next transmission
//} NextHop;

///** Current Hop information for the graph routing.
// *
// */
//typedef struct{
//  uint16_t GraphID;  ///< current graph ID
//  bool primaryPath;   ///< flag to identify whether path is primary or back up
//} CurrentHop;

/** Indicates the routing method used
 *
 */
typedef enum{
  SOURCE,
  GRAPH,
  MINLOAD
} RoutingMethod;

class Isa100RoutingAlgorithm : public Object{
public:

  static TypeId GetTypeId (void);

  Isa100RoutingAlgorithm();

  virtual ~Isa100RoutingAlgorithm();

  /** Populate header at source with any information required by routing algorithm.
   *
   * @param header The header object to be worked on.
   */
  virtual void PrepTxPacketHeader(Isa100DlHeader &header) = 0;

  /** Process a received packet to determine if it needs to be forwarded.
   * - If it does need to be forwarded on, this function will also make the
   *   necessary modifications to the packet header.
   *
   * @param packet Pointer to the received packet.
   * @param forwardPacketOn Reference to boolean that is true if the packet must be sent onward.
   */
  virtual void ProcessRxPacket(Ptr<Packet> packet, bool &forwardPacketOn) = 0;

  /** Determine if it's possible to attempt another link, given that a transmission has failed
   *   - This function should be overridden if used
   *
   * @param destInd The index of the final destination node
   * @param attempedLinks A list of already attempted links
   *
   * @return The base function always returns ff:ff
   */
  virtual Mac16Address AttemptAnotherLink(uint8_t destInd, std::vector<Mac16Address> attemptedLinks);

  virtual void DeleteTableEntry(Mac16Address nodeAddress) = 0; // Rajith

//  virtual Mac16Address AddressMatch (std::vector<Mac16Address> graphList) = 0;

//  RoutingMethod m_routingMethod;
//
//  void SetRoutingMethod(RoutingMethod routingMethod);

  virtual void SetGraphTable(std::map<Mac16Address, std::pair<Mac16Address, Mac16Address>> graphTable) = 0;

  virtual void SetTable(std::map<uint32_t, std::vector<std::vector<Mac16Address>>> table) = 0;

  virtual Mac16Address NextGraphID (Mac16Address graphID) = 0;

  virtual Mac16Address NextNeighbor (Mac16Address graphID) = 0;

protected:

	Mac16Address m_address; ///< Address of this node.

};

class Isa100SourceRoutingAlgorithm : public Isa100RoutingAlgorithm
{
public:

  static TypeId GetTypeId (void);

  Isa100SourceRoutingAlgorithm();

  /** Constructor.
   *
   * @param initNumDests The number of destinations the node can reach using source routing.
   * @param initTable Array of strings containing the multi-hop paths to reach each destination.  Each address is a string in XX:XX format.
   */
  Isa100SourceRoutingAlgorithm(uint32_t initNumDests, std::string *initTable);

  ~Isa100SourceRoutingAlgorithm();

  /** Populate header at source with any information required by routing algorithm.
   * - Only works if the header contains a valid destination address.
   *
   * @param header The header object to be worked on.
   */
  void PrepTxPacketHeader(Isa100DlHeader &header);

  /** Process a received packet to determine if it needs to be forwarded.
   * - If it does need to be forwarded on, this function will also make the
   *   necessary modifications to the packet header.
   *
   * @param packet Pointer to the received packet.
   * @param forwardPacketOn Reference to boolean that is true if the packet must be sent onward.
   */
  void ProcessRxPacket(Ptr<Packet> packet, bool &forwardPacketOn);

  void DeleteTableEntry(Mac16Address nodeAddress); // Rajith

  void SetGraphTable(std::map<Mac16Address, std::pair<Mac16Address, Mac16Address>> graphTable);

  void SetTable(std::map<uint32_t, std::vector<std::vector<Mac16Address>>> table);

  Mac16Address NextGraphID (Mac16Address graphID);

  Mac16Address NextNeighbor (Mac16Address graphID);

private:

  Mac16Address **m_table;
  uint32_t m_numDests;
  uint32_t *m_numHops;
  uint8_t m_nextSeqNum;

};

// Added by Rajith
class Isa100GraphRoutingAlgorithm : public Isa100RoutingAlgorithm
{
public:

  static TypeId GetTypeId (void);

  Isa100GraphRoutingAlgorithm();

  /** Constructor.
   *
   * @param initNumDests The number of destinations the node can reach using source routing.
   * @param initTable Array of strings containing the multi-hop paths to reach each destination.  Each address is a string in XX:XX format.
   */
  Isa100GraphRoutingAlgorithm(std::map<uint32_t, std::vector<Mac16Address>> initTable);

  ~Isa100GraphRoutingAlgorithm();

  /** Populate header at source with any information required by routing algorithm.
   * - Only works if the header contains a valid destination address.
   *
   * @param header The header object to be worked on.
   */
  void PrepTxPacketHeader(Isa100DlHeader &header);

  /** Process a received packet to determine if it needs to be forwarded.
   * - If it does need to be forwarded on, this function will also make the
   *   necessary modifications to the packet header.
   *
   * @param packet Pointer to the received packet.
   * @param forwardPacketOn Reference to boolean that is true if the packet must be sent onward.
   */
  void ProcessRxPacket(Ptr<Packet> packet, bool &forwardPacketOn);

//  void RoutingTableConfiguration(std::map<uint16_t, std::vector<Mac16Address>> initTable);

  void DeleteTableEntry(Mac16Address nodeAddress); // Rajith

  void SetGraphTable(std::map<Mac16Address, std::pair<Mac16Address, Mac16Address>> graphTable);

  void SetTable(std::map<uint32_t, std::vector<std::vector<Mac16Address>>> table);

  Mac16Address NextGraphID (Mac16Address graphID);

  Mac16Address NextNeighbor (Mac16Address graphID);

private:

  std::map<uint32_t, std::vector<Mac16Address>> m_table;    ///<  map <destination ID, vector<graphIDs>>
  std::map<uint32_t, std::vector<std::vector<Mac16Address>>> m_table2;    ///<  map <destination ID, vector<graphIDs>>
  ///< map <current graphID, NextHop(next graphID, next neighbor)>
  std::map<Mac16Address, std::pair<Mac16Address, Mac16Address>> m_graphTable;
  uint8_t m_nextSeqNum;
  uint32_t m_counter;
//  Mac16Address **m_table;
//  uint32_t m_numDests;
//  uint32_t *m_numHops;

};

}

#endif /* ISA100_ROUTING_H */
