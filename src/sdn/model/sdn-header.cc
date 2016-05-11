/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2015 Haoliang Chen
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

#include <cmath>

#include "ns3/assert.h"
#include "ns3/log.h"

#include "sdn-header.h"

#define IPV4_ADDRESS_SIZE 4
#define SDN_PKT_HEADER_SIZE 8
#define SDN_MSG_HEADER_SIZE 8
#define SDN_HELLO_HEADER_SIZE 28
#define SDN_APPOINTMENT_HEADER_SIZE 8
#define SDN_ACKHELLO_HEADER_SIZE 44
#define SDN_DONTFORWARD_HEADER_SIZE 8
#define SDN_DONTFORWARD_TUPLE_SIZE 4
#define SDN_LC2LC_HEADER_SIZE 8
#define SDN_LC2LC_TUPLE_SIZE 4

NS_LOG_COMPONENT_DEFINE ("SdnHeader");

namespace ns3 {
namespace sdn {

float
rIEEE754 (uint32_t emf)
{
  union{
    float f;
    uint32_t b;
  } u;
  u.b = emf;
  return (u.f);
}

uint32_t
IEEE754 (float dec)
{
  union{
    float f;
    uint32_t b;
  } u;
  u.f = dec;
  return (u.b);
}

// ---------------- SDN Packet -------------------------------
NS_OBJECT_ENSURE_REGISTERED (PacketHeader);



PacketHeader::PacketHeader () :
    m_packetLength (0),
    m_packetSequenceNumber (0)
{
}

PacketHeader::~PacketHeader ()
{
}

TypeId
PacketHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::sdn::PacketHeader")
    .SetParent<Header> ()
    .AddConstructor<PacketHeader> ()
  ;
  return (tid);
}
TypeId
PacketHeader::GetInstanceTypeId (void) const
{
  return (GetTypeId ());
}

uint32_t 
PacketHeader::GetSerializedSize (void) const
{
  return (SDN_PKT_HEADER_SIZE);
}

void 
PacketHeader::Print (std::ostream &os) const
{
  /// \todo
}

void
PacketHeader::Serialize (Buffer::Iterator start) const
{
  Buffer::Iterator i = start;
  i.WriteHtonU32 (this->originator.Get());
  i.WriteHtonU16 (m_packetLength);
  i.WriteHtonU16 (m_packetSequenceNumber);
}

uint32_t
PacketHeader::Deserialize (Buffer::Iterator start)
{
  Buffer::Iterator i = start;
  uint32_t add_temp = i.ReadNtohU32();
  this->originator.Set(add_temp);
  m_packetLength  = i.ReadNtohU16 ();
  m_packetSequenceNumber = i.ReadNtohU16 ();
  return (GetSerializedSize ());
}

// ---------------- SDN Message -------------------------------

NS_OBJECT_ENSURE_REGISTERED (MessageHeader);

MessageHeader::MessageHeader ()
  : m_messageType (MessageHeader::MessageType (0)),
    m_vTime (0),
    m_timeToLive (0),
    m_messageSequenceNumber (0),
    m_messageSize (0)
{
}

MessageHeader::~MessageHeader ()
{
}

TypeId
MessageHeader::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::sdn::MessageHeader")
    .SetParent<Header> ()
    .AddConstructor<MessageHeader> ()
  ;
  return (tid);
}
TypeId
MessageHeader::GetInstanceTypeId (void) const
{
  return (GetTypeId ());
}

uint32_t
MessageHeader::GetSerializedSize (void) const
{
  uint32_t size = SDN_MSG_HEADER_SIZE;
  switch (m_messageType)
    {
    case HELLO_MESSAGE:
      NS_LOG_DEBUG ("Hello Message Size: " << size << " + " 
            << m_message.hello.GetSerializedSize ());
      size += m_message.hello.GetSerializedSize ();
      break;
    case APPOINTMENT_MESSAGE:
      size += m_message.appointment.GetSerializedSize ();
      break;
    case ACKHELLO_MESSAGE:
      size += m_message.ackhello.GetSerializedSize ();
      break;
    case DONT_FORWARD:
      size += m_message.dontforward.GetSerializedSize ();
      break;
    case LC2LC:
      size += m_message.lc2lc.GetSerializedSize ();
      break;
    default:
      NS_ASSERT (false);
    }
  return (size);
}

void 
MessageHeader::Print (std::ostream &os) const
{
  /// \todo
}

void
MessageHeader::Serialize (Buffer::Iterator start) const
{
  Buffer::Iterator i = start;
  i.WriteU8 (m_messageType);
  i.WriteU8 (m_vTime);
  i.WriteHtonU16 (GetSerializedSize ());
  i.WriteHtonU16 (m_timeToLive);
  i.WriteHtonU16 (m_messageSequenceNumber);

  switch (m_messageType)
    {
    case HELLO_MESSAGE:
      m_message.hello.Serialize (i);
      break;
    case APPOINTMENT_MESSAGE:
      m_message.appointment.Serialize (i);
      break;
    case ACKHELLO_MESSAGE:
      m_message.ackhello.Serialize (i);
      break;
    case DONT_FORWARD:
      m_message.dontforward.Serialize (i);
      break;
    case LC2LC:
      m_message.lc2lc.Serialize (i);
      break;
    default:
      NS_ASSERT (false);
    }

}

uint32_t
MessageHeader::Deserialize (Buffer::Iterator start)
{
  uint32_t size;
  Buffer::Iterator i = start;
  m_messageType  = (MessageType) i.ReadU8 ();
  NS_ASSERT (m_messageType >= HELLO_MESSAGE && m_messageType <= LC2LC);
  m_vTime  = i.ReadU8 ();
  m_messageSize  = i.ReadNtohU16 ();
  m_timeToLive  = i.ReadNtohU16 ();
  m_messageSequenceNumber = i.ReadNtohU16 ();
  size = SDN_MSG_HEADER_SIZE;
  switch (m_messageType)
    {
    case HELLO_MESSAGE:
      size += 
        m_message.hello.Deserialize (i, m_messageSize - SDN_MSG_HEADER_SIZE);
      break;
    case APPOINTMENT_MESSAGE:
      size +=
        m_message.appointment.Deserialize (i, m_messageSize - SDN_MSG_HEADER_SIZE);
      break;
    case ACKHELLO_MESSAGE:
      size +=
        m_message.ackhello.Deserialize (i, m_messageSize - SDN_MSG_HEADER_SIZE);
      break;
    case DONT_FORWARD:
      size +=
        m_message.dontforward.Deserialize (i, m_messageSize - SDN_MSG_HEADER_SIZE);
      break;
    case LC2LC:
      size +=
        m_message.lc2lc.Deserialize (i, m_messageSize - SDN_MSG_HEADER_SIZE);
      break;

    default:
      NS_ASSERT (false);
    }
  return (size);
}


// ---------------- SDN HELLO Message -------------------------------

uint32_t 
MessageHeader::Hello::GetSerializedSize (void) const
{
  return (SDN_HELLO_HEADER_SIZE);
}

void 
MessageHeader::Hello::Print (std::ostream &os) const
{
  /// \todo
}

void
MessageHeader::Hello::Serialize (Buffer::Iterator start) const
{
  Buffer::Iterator i = start;

  i.WriteHtonU32 (this->ID.Get());
  i.WriteHtonU32 (this->position.X);
  i.WriteHtonU32 (this->position.Y);
  i.WriteHtonU32 (this->position.Z);
  i.WriteHtonU32 (this->velocity.X);
  i.WriteHtonU32 (this->velocity.Y);
  i.WriteHtonU32 (this->velocity.Z);

}

uint32_t
MessageHeader::Hello::Deserialize (Buffer::Iterator start, 
  uint32_t messageSize)
{
  Buffer::Iterator i = start;

  NS_ASSERT (messageSize == SDN_HELLO_HEADER_SIZE);

  uint32_t add_temp = i.ReadNtohU32();
  this->ID.Set(add_temp);
  this->position.X = i.ReadNtohU32();
  this->position.Y = i.ReadNtohU32();
  this->position.Z = i.ReadNtohU32();
  this->velocity.X = i.ReadNtohU32();
  this->velocity.Y = i.ReadNtohU32();
  this->velocity.Z = i.ReadNtohU32();

  return (messageSize);
}

// ---------------- SDN Appointment Message -------------------------------

void
MessageHeader::Appointment::Print (std::ostream &os) const
{
  //TODO
}

uint32_t
MessageHeader::Appointment::GetSerializedSize () const
{
  return SDN_APPOINTMENT_HEADER_SIZE;
}

void
MessageHeader::Appointment::Serialize (Buffer::Iterator start) const
{
  Buffer::Iterator i = start;

  i.WriteHtonU32 (this->ID.Get());
  uint32_t at = 0;
  if (ATField == FORWARDER)
    at = 0xFFFF;
  i.WriteHtonU32 (at);
}

uint32_t
MessageHeader::Appointment::Deserialize (Buffer::Iterator start, uint32_t messageSize)
{
  Buffer::Iterator i = start;

  uint32_t ip_temp = i.ReadNtohU32();
  this->ID.Set (ip_temp);
  uint32_t at = i.ReadNtohU32();
  if (at)
    this->ATField = FORWARDER;
  else
    this->ATField = NORMAL;
  return (messageSize);
}


// ---------------- SDN AckHello Message -------------------------------

uint32_t
MessageHeader::AckHello::GetSerializedSize (void) const
{
  return (SDN_ACKHELLO_HEADER_SIZE);
}

void
MessageHeader::AckHello::Print (std::ostream &os) const
{
  /// \todo
}

void
MessageHeader::AckHello::Serialize (Buffer::Iterator start) const
{
  Buffer::Iterator i = start;

  i.WriteHtonU32 (this->ID.Get());
  i.WriteHtonU32 (this->position.X);
  i.WriteHtonU32 (this->position.Y);
  i.WriteHtonU32 (this->position.Z);
  i.WriteHtonU32 (this->velocity.X);
  i.WriteHtonU32 (this->velocity.Y);
  i.WriteHtonU32 (this->velocity.Z);
  i.WriteHtonU32 (this->start.X);
  i.WriteHtonU32 (this->start.Y);
  i.WriteHtonU32 (this->end.X);
  i.WriteHtonU32 (this->end.Y);
}

uint32_t
MessageHeader::AckHello::Deserialize (Buffer::Iterator start,
  uint32_t messageSize)
{
  Buffer::Iterator i = start;

  NS_ASSERT (messageSize == SDN_ACKHELLO_HEADER_SIZE);

  uint32_t add_temp = i.ReadNtohU32();
  this->ID.Set(add_temp);
  this->position.X = i.ReadNtohU32();
  this->position.Y = i.ReadNtohU32();
  this->position.Z = i.ReadNtohU32();
  this->velocity.X = i.ReadNtohU32();
  this->velocity.Y = i.ReadNtohU32();
  this->velocity.Z = i.ReadNtohU32();
  this->start.X = i.ReadNtohU32();
  this->start.Y = i.ReadNtohU32();
  this->end.X = i.ReadNtohU32();
  this->end.Y = i.ReadNtohU32();

  return (messageSize);
}

// ---------------- DONT_FORWARD Message -------------------------------

uint32_t
MessageHeader::DontForward::GetSerializedSize (void) const
{
  return (SDN_DONTFORWARD_HEADER_SIZE +
    this->list.size () * SDN_DONTFORWARD_TUPLE_SIZE);
}

void
MessageHeader::DontForward::Print (std::ostream &os) const
{
  /// \todo
}

void
MessageHeader::DontForward::Serialize (Buffer::Iterator start) const
{
  Buffer::Iterator i = start;

  i.WriteHtonU32 (this->ID.Get());
  i.WriteHtonU32 (this->list_size);

  for (std::vector<Ipv4Address>::const_iterator cit = this->list.begin ();
       cit != this->list.end (); ++cit)
    {
      i.WriteHtonU32 (cit->Get());
    }
}

uint32_t
MessageHeader::DontForward::Deserialize (Buffer::Iterator start,
  uint32_t messageSize)
{
  Buffer::Iterator i = start;

  this->list.clear ();
  NS_ASSERT (messageSize >= SDN_DONTFORWARD_HEADER_SIZE);

  uint32_t add_temp = i.ReadNtohU32();
  this->ID.Set(add_temp);
  this->list_size = i.ReadNtohU32 ();

  NS_ASSERT ((messageSize - SDN_DONTFORWARD_HEADER_SIZE) %
    (SDN_DONTFORWARD_TUPLE_SIZE) == 0);

  uint32_t numTuples = (messageSize - SDN_DONTFORWARD_HEADER_SIZE)
    / (SDN_DONTFORWARD_TUPLE_SIZE);
  NS_ASSERT ( numTuples == this->list_size );

  for (uint32_t n = 0; n < numTuples; ++n)
  {
    uint32_t temp = i.ReadNtohU32();
    this->list.push_back (Ipv4Address (temp));
   }

  return (messageSize);
}

// ---------------- LC2LC Message -------------------------------

uint32_t
MessageHeader::Lc2Lc::GetSerializedSize (void) const
{
  return (SDN_LC2LC_HEADER_SIZE +
    this->list.size () * SDN_LC2LC_TUPLE_SIZE);
}

void
MessageHeader::Lc2Lc::Print (std::ostream &os) const
{
  /// \todo
}

void
MessageHeader::Lc2Lc::Serialize (Buffer::Iterator start) const
{
  Buffer::Iterator i = start;

  i.WriteHtonU32 (this->ID.Get());
  i.WriteHtonU32 (this->list_size);

  for (std::vector<Ipv4Address>::const_iterator cit = this->list.begin ();
       cit != this->list.end (); ++cit)
    {
      i.WriteHtonU32 (cit->Get());
    }
}

uint32_t
MessageHeader::Lc2Lc::Deserialize (Buffer::Iterator start,
  uint32_t messageSize)
{
  Buffer::Iterator i = start;

  this->list.clear ();
  NS_ASSERT (messageSize >= SDN_LC2LC_HEADER_SIZE);

  uint32_t add_temp = i.ReadNtohU32();
  this->ID.Set(add_temp);
  this->list_size = i.ReadNtohU32 ();

  NS_ASSERT ((messageSize - SDN_LC2LC_HEADER_SIZE) %
    (SDN_DONTFORWARD_TUPLE_SIZE) == 0);

  uint32_t numTuples = (messageSize - SDN_LC2LC_HEADER_SIZE)
    / (SDN_LC2LC_TUPLE_SIZE);
  NS_ASSERT ( numTuples == this->list_size );

  for (uint32_t n = 0; n < numTuples; ++n)
  {
    uint32_t temp = i.ReadNtohU32();
    this->list.push_back (Ipv4Address (temp));
   }

  return (messageSize);
}

}
}  // namespace sdn, ns3

