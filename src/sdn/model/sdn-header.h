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

#ifndef SDN_HEADER_H
#define SDN_HEADER_H

#include <stdint.h>
#include <vector>
#include "ns3/header.h"
#include "ns3/ipv4-address.h"
#include "ns3/nstime.h"
#include "ns3/vector.h"

namespace ns3 {
namespace sdn {

enum AppointmentType {NORMAL, FORWARDER};
float     rIEEE754 (uint32_t emf);
uint32_t  IEEE754 (float dec);

// Packet Format
//
//    The basic layout of any packet in SDN is as follows (omitting IP and
//    UDP headers):
//
//        0                   1                   2                   3
//        0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
//       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//       |                          originator                           |
//       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//       |         Packet Length         |    Packet Sequence Number     |
//       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//       |  Message Type |     Vtime     |         Message Size          |
//       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//       |          Time To Live         |    Message Sequence Number    |
//       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//       |                                                               |
//       :                            MESSAGE                            :
//       |                                                               |
//       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//       |  Message Type |     Vtime     |         Message Size          |
//       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//       |          Time To Live         |    Message Sequence Number    |
//       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//       |                                                               |
//       :                            MESSAGE                            :
//       |                                                               |
//       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//       :                                                               :
//                (etc.)
class PacketHeader : public Header
{
public:
  PacketHeader ();
  virtual ~PacketHeader ();

  Ipv4Address originator;

  void SetPacketLength (uint16_t length)
  {
    m_packetLength = length;
  }
  uint16_t GetPacketLength () const
  {
    return (m_packetLength);
  }

  void SetPacketSequenceNumber (uint16_t seqnum)
  {
    m_packetSequenceNumber = seqnum;
  }
  uint16_t GetPacketSequenceNumber () const
  {
    return (m_packetSequenceNumber);
  }

private:
  uint16_t m_packetLength;
  uint16_t m_packetSequenceNumber;

public:
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual void Print (std::ostream &os) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);
};


//        0                   1                   2                   3
//        0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
//       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//       |  Message Type |     Vtime     |         Message Size          |
//       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//       |          Time To Live         |    Message Sequence Number    |
//       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//       |                                                               |
//       :                            MESSAGE                            :
//       |                                                               |
//       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

class MessageHeader : public Header
{
public:

  enum MessageType {
    HELLO_MESSAGE,
    APPOINTMENT_MESSAGE,
    ACKHELLO_MESSAGE,
    DONT_FORWARD,
    LC2LC
  };

  MessageHeader ();
  virtual ~MessageHeader ();

  void SetMessageType (MessageType messageType)
  {
    m_messageType = messageType;
  }
  MessageType GetMessageType () const
  {
    return (m_messageType);
  }

  void SetVTime (Time time)
  {
    //vTime is useless by far
    m_vTime = 0;
  }
  Time GetVTime () const
  {
    return Seconds(m_vTime);
  }

  void SetTimeToLive (uint16_t timeToLive)
  {
    m_timeToLive = timeToLive;
  }
  uint16_t GetTimeToLive () const
  {
    return (m_timeToLive);
  }

  void SetMessageSequenceNumber (uint16_t messageSequenceNumber)
  {
    m_messageSequenceNumber = messageSequenceNumber;
  }
  uint16_t GetMessageSequenceNumber () const
  {
    return (m_messageSequenceNumber);
  }

   void SetMessageSize (uint16_t messageSize)
   {
     m_messageSize = messageSize;
   }
   uint16_t GetMessageSize () const
   {
     return (m_messageSize);
   }

private:
  MessageType m_messageType;
  uint8_t m_vTime;
  uint16_t m_timeToLive;
  uint16_t m_messageSequenceNumber;
  uint16_t m_messageSize;

public:
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual void Print (std::ostream &os) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (Buffer::Iterator start) const;
  virtual uint32_t Deserialize (Buffer::Iterator start);

  //  HELLO Message Format
  //
  //        0                   1                   2                   3
  //        0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
  //
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                      ID (SCH IP Address)                      |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                           Position X                          |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                           Position Y                          |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                           Position Z                          |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                           Velocity X                          |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                           Velocity Y                          |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                           Velocity Z                          |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+


  struct Hello
  {
    Ipv4Address ID;
    
    struct Position{
      uint32_t X, Y, Z;
    };
    
    struct Velocity{
      uint32_t X, Y, Z;
    };
    
    Position position;
    void SetPosition(double x, double y, double z)
    {
      this->position.X = IEEE754(x);
      this->position.Y = IEEE754(y);
      this->position.Z = IEEE754(z);
    }
    
    void GetPosition(double &x, double &y, double &z) const
    {
      x = rIEEE754(this->position.X);
      y = rIEEE754(this->position.Y);
      z = rIEEE754(this->position.Z);
    }
    
    Vector3D GetPosition() const
    {
      return Vector3D(rIEEE754(this->position.X),
                      rIEEE754(this->position.Y),
                      rIEEE754(this->position.Z));
    }

    Velocity velocity;
    void SetVelocity(double x, double y, double z)
    {
      this->velocity.X = IEEE754(x);
      this->velocity.Y = IEEE754(y);
      this->velocity.Z = IEEE754(z);
    }
    
    void GetVelocity(double &x, double &y, double &z) const
    {
      x = rIEEE754(this->velocity.X);
      y = rIEEE754(this->velocity.Y);
      z = rIEEE754(this->velocity.Z);
    }
    
    Vector3D GetVelocity() const
    {
      return Vector3D(rIEEE754(this->velocity.X),
                      rIEEE754(this->velocity.Y),
                      rIEEE754(this->velocity.Z));
    }

    void Print (std::ostream &os) const;
    uint32_t GetSerializedSize (void) const;
    void Serialize (Buffer::Iterator start) const;
    uint32_t Deserialize (Buffer::Iterator start, uint32_t messageSize);
  };

  //  Appointment Message Format
  //    One Appointment is for one car only.
  //    The proposed format of a appointment message is as follows:
  //
  //        0                   1                   2                   3
  //        0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                             Car ID                            |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                       Appointment Type                        |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+


  struct Appointment
  {
    Ipv4Address ID;
    AppointmentType ATField;

    void Print (std::ostream &os) const;
    uint32_t GetSerializedSize (void) const;
    void Serialize (Buffer::Iterator start) const;
    uint32_t Deserialize (Buffer::Iterator start, uint32_t messageSize);
  };


  //  AckHELLO Message Format
  //
  //        0                   1                   2                   3
  //        0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
  //
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                        ID (IP Address)                        |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                           Position X                          |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                           Position Y                          |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                           Position Z                          |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                           Velocity X                          |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                           Velocity Y                          |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                           Velocity Z                          |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                     ControllArea_Start X                      |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                     ControllArea_Start Y                      |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                       ControllArea_End X                      |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                       ControllArea_End Y                      |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

  struct AckHello
  {
    Ipv4Address ID;

    struct Position{
      uint32_t X, Y, Z;
    };

    struct Velocity{
      uint32_t X, Y, Z;
    };

    struct ControllArea_Start{
      uint32_t X, Y;
    };

    struct ControllArea_End{
      uint32_t X, Y;
    };


    Position position;
    void SetPosition(double x, double y, double z)
    {
      this->position.X = IEEE754(x);
      this->position.Y = IEEE754(y);
      this->position.Z = IEEE754(z);
    }

    void GetPosition(double &x, double &y, double &z) const
    {
      x = rIEEE754(this->position.X);
      y = rIEEE754(this->position.Y);
      z = rIEEE754(this->position.Z);
    }

    Vector3D GetPosition() const
    {
      return Vector3D(rIEEE754(this->position.X),
                      rIEEE754(this->position.Y),
                      rIEEE754(this->position.Z));
    }

    Velocity velocity;
    void SetVelocity(double x, double y, double z)
    {
      this->velocity.X = IEEE754(x);
      this->velocity.Y = IEEE754(y);
      this->velocity.Z = IEEE754(z);
    }

    void GetVelocity(double &x, double &y, double &z) const
    {
      x = rIEEE754(this->velocity.X);
      y = rIEEE754(this->velocity.Y);
      z = rIEEE754(this->velocity.Z);
    }

    Vector3D GetVelocity() const
    {
      return Vector3D(rIEEE754(this->velocity.X),
                      rIEEE754(this->velocity.Y),
                      rIEEE754(this->velocity.Z));
    }

    ControllArea_Start start;
    void SetControllArea_Start(Vector2D area_start)
    {
      this->start.X = IEEE754(area_start.x);
      this->start.Y = IEEE754(area_start.y);
    }

    Vector2D GetControllArea_Start() const
    {
      return Vector2D(rIEEE754(this->start.X),
                      rIEEE754(this->start.Y));
    }

    ControllArea_End end;
    void SetControllArea_End(Vector2D area_end)
    {
      this->end.X = IEEE754(area_end.x);
      this->end.Y = IEEE754(area_end.y);
    }

    Vector2D GetControllArea_End() const
    {
      return Vector2D(rIEEE754(this->end.X),
                      rIEEE754(this->end.Y));
    }

    void Print (std::ostream &os) const;
    uint32_t GetSerializedSize (void) const;
    void Serialize (Buffer::Iterator start) const;
    uint32_t Deserialize (Buffer::Iterator start, uint32_t messageSize);
  };


  //  DONT_FORWARD Message Format
  //
  //        0                   1                   2                   3
  //        0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
  //
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                        ID (IP Address)                        |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                           List Size                           |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                       ID1 (IP Address)                        |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                       ID2 (IP Address)                        |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       :                                                               :
  //       :                               :

  struct DontForward
  {
    Ipv4Address ID;
    uint32_t list_size;
    std::vector<Ipv4Address> list;

    void Print (std::ostream &os) const;
    uint32_t GetSerializedSize (void) const;
    void Serialize (Buffer::Iterator start) const;
    uint32_t Deserialize (Buffer::Iterator start, uint32_t messageSize);
  };



  //  LC2LC Message Format
  //
  //        0                   1                   2                   3
  //        0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
  //
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                      SourceLC (IP Address)                    |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                           List Size                           |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                       ID1 (IP Address)                        |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       |                       ID2 (IP Address)                        |
  //       +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
  //       :                                                               :
  //       :                               :

  struct Lc2Lc
  {
    Ipv4Address ID;
    uint32_t list_size;
    std::vector<Ipv4Address> list;

    void Print (std::ostream &os) const;
    uint32_t GetSerializedSize (void) const;
    void Serialize (Buffer::Iterator start) const;
    uint32_t Deserialize (Buffer::Iterator start, uint32_t messageSize);
  };


private:
  struct
  {
    Hello hello;
    Appointment appointment;
    AckHello ackhello;
    DontForward dontforward;
    Lc2Lc lc2lc;
  } m_message; // union not allowed

public:

  Hello& GetHello ()
  {
    if (m_messageType == 0)
      {
        m_messageType = HELLO_MESSAGE;
      }
    else
      {
        NS_ASSERT (m_messageType == HELLO_MESSAGE);
      }
    return (m_message.hello);
  }

  Appointment& GetAppointment ()
  {
    if (m_messageType == 0)
      {
        m_messageType = APPOINTMENT_MESSAGE;
      }
    else
      {
        NS_ASSERT (m_messageType == APPOINTMENT_MESSAGE);
      }
    return (m_message.appointment);
  }

  AckHello& GetAckHello ()
  {
    if (m_messageType == 0)
      {
        m_messageType = ACKHELLO_MESSAGE;
      }
    else
      {
        NS_ASSERT (m_messageType == ACKHELLO_MESSAGE);
      }
    return (m_message.ackhello);
  }

  DontForward& GetDontForward ()
  {
    if (m_messageType == 0)
      {
        m_messageType = DONT_FORWARD;
      }
    else
      {
        NS_ASSERT (m_messageType == DONT_FORWARD);
      }
    return (m_message.dontforward);
  }

  Lc2Lc& GetLc2Lc ()
  {
    if (m_messageType == 0)
      {
        m_messageType = LC2LC;
      }
    else
      {
        NS_ASSERT (m_messageType == LC2LC);
      }
    return (m_message.lc2lc);
  }

  const Hello& GetHello () const
  {
    NS_ASSERT (m_messageType == HELLO_MESSAGE);
    return (m_message.hello);
  }

  const Appointment& GetAppointment () const
  {
    NS_ASSERT (m_messageType == APPOINTMENT_MESSAGE);
    return (m_message.appointment);
  }

  const AckHello& GetAckHello () const
  {
    NS_ASSERT (m_messageType == ACKHELLO_MESSAGE);
    return (m_message.ackhello);
  }

  const DontForward& GetDontForward () const
  {
    NS_ASSERT (m_messageType == DONT_FORWARD);
    return (m_message.dontforward);
  }

  const Lc2Lc& GetLc2Lc () const
  {
    NS_ASSERT (m_messageType == LC2LC);
    return (m_message.lc2lc);
  }
};

static inline std::ostream& operator<< (std::ostream& os, const PacketHeader & packet)
{
  packet.Print (os);
  return (os);
}

static inline std::ostream& operator<< (std::ostream& os, const MessageHeader & message)
{
  message.Print (os);
  return (os);
}

typedef std::vector<MessageHeader> MessageList;

static inline std::ostream& operator<< (std::ostream& os, const MessageList & messages)
{
  os << "[";
  for (std::vector<MessageHeader>::const_iterator messageIter = messages.begin ();
       messageIter != messages.end (); messageIter++)
    {
      messageIter->Print (os);
      if (messageIter+1 != messages.end ())
        os << ", ";
    }
  os << "]";
  return (os);
}


}//namespace sdn
}//namespace ns3


#endif /* SDN_HEADER_H */
