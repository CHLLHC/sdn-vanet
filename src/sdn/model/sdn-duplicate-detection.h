/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 Haoliang Chen
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
 * Author: Haoliang Chen  <chl41993@gmail.com>
 */

#ifndef SDN_DUPLICATE_DETECTION_H
#define SDN_DUPLICATE_DETECTION_H
#include <list>
#include <unordered_map>
#include "ns3/hash.h"
#include "ns3/packet.h"
#include "ns3/ptr.h"

namespace ns3{
namespace sdn{

class Duplicate_Detection
{
public:
  Duplicate_Detection (uint32_t size = 256);

  bool CheckThis (Ptr<const Packet> p);

private:
  uint32_t m_size;
  std::list<uint64_t> m_container;
  std::unordered_map<uint64_t, std::list<uint64_t>::iterator > m_containerMap;
};


}
}


#endif //SDN_DUPLICATE_DETECTION_H
