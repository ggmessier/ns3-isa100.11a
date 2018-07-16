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
 *
 */

#ifndef ISA100_UPLINK_GRAPH_H
#define ISA100_UPLINK_GRAPH_H

#include "ns3/core-module.h"
#include "ns3/isa100-11a-module.h"
#include <algorithm>

using namespace std;

namespace ns3 {

/**
 * Packet scheduling algorithm for graph routing
 *- ISAGraphScheduling Class is the scheduling model to use for the ISA100.11a network scheduling.
 *
 */

class Isa100UplinkGraph;

/** Class that stores the ISA-100 graph information.
 * - Includes reliable broadcasting, uplink and downlink graph creation.
 */
class Isa100UplinkGraph : public Object
{
public:
  static TypeId GetTypeId (void);

  Isa100UplinkGraph ();

  virtual ~Isa100UplinkGraph ();

};

} // namespace ns3

#endif /* ISA100_11A_DOWNLINK_GRAPH_H */
