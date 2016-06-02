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

#include "sdn-duplicate-detection.h"

namespace ns3{
namespace sdn{

Duplicate_Detection::Duplicate_Detection (uint32_t size):
    m_size (size) {};

bool
Duplicate_Detection::CheckThis (Ptr<const Packet> p)
{
  /*uint32_t sizep = p->GetSize();
  uint8_t* buffer = new uint8_t[sizep];
  p->CopyData(buffer, sizep);
  uint64_t hashV = Hash64 ((char*)(buffer), sizep);
  delete buffer;
*/
  uint64_t hashV = p->GetUid();

  if (m_containerMap.find (hashV) != m_containerMap.end ())
    {
      m_container.erase (m_containerMap[hashV]);
      m_container.push_front (hashV);
      m_containerMap[hashV] = m_container.begin ();
      return true;
    }
  else
    {
      m_container.push_front (hashV);
      m_containerMap[hashV] = m_container.begin ();
      if (m_container.size () > m_size)
        {
          m_containerMap.erase (*m_container.rbegin());
          m_container.pop_back ();
        }
      return false;
    }
}

}
}

