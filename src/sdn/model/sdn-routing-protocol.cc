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
 * Authors: Haoliang Chen <chl41993@gmail.com>
 */


///
/// \brief Implementation of SDN agent on car side 
/// and related classes.
///
/// This is the main file of this software because SDN's behaviour is
/// implemented here.
///

#define NS_LOG_APPEND_CONTEXT                                   \
  if (GetObject<Node> ()) { std::clog << "[node " << GetObject<Node> ()->GetId () << "] "; }


#include "sdn-routing-protocol.h"
#include "ns3/socket-factory.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/names.h"
#include "ns3/inet-socket-address.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/ipv4-routing-table-entry.h"
#include "ns3/ipv4-route.h"
#include "ns3/boolean.h"
#include "ns3/uinteger.h"
#include "ns3/enum.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/ipv4-header.h"

#include <ostream>

/********** Useful macros **********/

///
/// \brief Gets the delay between a given time and the current time.
///
/// If given time is previous to the current one, then this macro returns
/// a number close to 0. This is used for scheduling events at a certain moment.
///
#define DELAY(time) (((time) < (Simulator::Now ())) ? Seconds (0.000001) : \
                     (time - Simulator::Now () + Seconds (0.000001)))






/********** Miscellaneous constants **********/

/// Maximum allowed jitter.
#define SDN_MAXJITTER          (m_helloInterval.GetSeconds () / 4)
/// Random number between [0-SDN_MAXJITTER] used to jitter SDN packet transmission.
#define JITTER (Seconds (m_uniformRandomVariable->GetValue (0, SDN_MAXJITTER)))


#define SDN_MAX_SEQ_NUM        65535


#define SDN_PORT_NUMBER 419
/// Maximum number of messages per packet.
#define SDN_MAX_MSGS    64


#define INFHOP 2147483647

namespace ns3 {
namespace sdn {

NS_LOG_COMPONENT_DEFINE ("SdnRoutingProtocol");

//Double ABS
double
dabs (double x)
{
  return x > 0 ? x : -x;
}

std::string
Ipv4toString (const Ipv4Address& address)
{
  std::ostringstream buffer("");
  address.Print (buffer);
  return buffer.str ();
}

/********** SDN controller class **********/

NS_OBJECT_ENSURE_REGISTERED (RoutingProtocol);

TypeId
RoutingProtocol::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::sdn::RoutingProtocol")
    .SetParent<Ipv4RoutingProtocol> ()
    .AddConstructor<RoutingProtocol> ();
  return tid;
}


RoutingProtocol::RoutingProtocol ()
  :
    m_packetSequenceNumber (SDN_MAX_SEQ_NUM),
    m_messageSequenceNumber (SDN_MAX_SEQ_NUM),
    m_helloInterval (Seconds(1)),
    m_minAPInterval (Seconds (1)),
    m_ipv4 (0),
    m_helloTimer (Timer::CANCEL_ON_DESTROY),
    m_apTimer (Timer::CANCEL_ON_DESTROY),
    m_queuedMessagesTimer (Timer::CANCEL_ON_DESTROY),
    m_SCHinterface (0),
    m_CCHinterface (0),
    m_nodetype (OTHERS),
    m_appointmentResult (NORMAL),
    m_linkEstablished (false),
    m_numArea (0),
    m_isPadding (false),
    m_numAreaVaild (false),
    m_road_length (814),//MagicNumber
    m_signal_range (419),
    m_safety_raito (0.9),
    m_lc_controllArea_vaild (false),
    m_norespond_hm (0),
    m_car_lc_ack_vaild (false),
    m_lowerbound (0),
    m_algorithm (Binary_Search)
{
  m_uniformRandomVariable = CreateObject<UniformRandomVariable> ();
}

RoutingProtocol::~RoutingProtocol ()
{
  
}

void
RoutingProtocol::SetCCHInterface (uint32_t interface)
{
  m_CCHinterface = interface;
  m_CCHAddress = m_ipv4->GetAddress (m_CCHinterface, 0).GetLocal ();
  Ipv4InterfaceAddress temp_if_add = m_ipv4->GetAddress (m_CCHinterface, 0);
  AddEntry (temp_if_add.GetLocal (),
            Ipv4Address (temp_if_add.GetMask ().Get ()),
            temp_if_add.GetLocal (),
            m_CCHinterface);
}

void
RoutingProtocol::SetSCHInterface (uint32_t interface)
{
  m_SCHinterface = interface;
  m_SCHAddress = m_ipv4->GetAddress (m_SCHinterface, 0).GetLocal ();
  Ipv4InterfaceAddress temp_if_add = m_ipv4->GetAddress (m_SCHinterface, 0);
  AddEntry (temp_if_add.GetLocal (),
            Ipv4Address (temp_if_add.GetMask ().Get ()),
            temp_if_add.GetLocal (),
            m_SCHinterface);
}

void
RoutingProtocol::SetAlgo (Algo which)
{
  m_algorithm = which;
}

int64_t
RoutingProtocol::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  m_uniformRandomVariable->SetStream (stream);
  return 1;
}

void
RoutingProtocol::DoInitialize ()
{

  Ipv4Address loopback ("127.0.0.1");

  bool canRunSdn = false;
  //Install RecvSDN  Only on CCH channel.
  if (m_CCHAddress != Ipv4Address::GetZero ())
    {
      // Create a socket to listen only on this interface
      Ptr<Socket> socket = Socket::CreateSocket (GetObject<Node> (),
                                                 UdpSocketFactory::GetTypeId ());
      // TRUE
      socket->SetAllowBroadcast (true);
      InetSocketAddress
        inetAddr (m_ipv4->GetAddress (m_CCHinterface, 0).GetLocal (), SDN_PORT_NUMBER);
      socket->SetRecvCallback (MakeCallback (&RoutingProtocol::RecvSDN,  this));
      if (socket->Bind (inetAddr))
        {
          NS_FATAL_ERROR ("Failed to bind() SDN socket");
        }
      socket->BindToNetDevice (m_ipv4->GetNetDevice (m_CCHinterface));
      m_socketAddresses[socket] = m_ipv4->GetAddress (m_CCHinterface, 0);

      canRunSdn = true;
    }

  Init_NumArea();
  if(canRunSdn)
    {
      HelloTimerExpire ();
      APTimerExpire ();
      NS_LOG_DEBUG ("SDN on node (Car) " << m_CCHAddress << " started");
    }
}

void
RoutingProtocol::Clear_m_table ()
{
  NS_LOG_FUNCTION_NOARGS();
  m_table.clear();
}

void
RoutingProtocol::RemoveEntry (Ipv4Address const &dest)
{
  m_table.erase (dest);
}

void
RoutingProtocol::AddEntry (const Ipv4Address &dest,
                           const Ipv4Address &mask,
                           const Ipv4Address &next,
                           uint32_t interface)
{
  NS_LOG_FUNCTION(this << dest << next << interface << mask << m_CCHAddress);
  RoutingTableEntry RTE;
  RTE.destAddr = dest;
  RTE.mask = mask;
  RTE.nextHop = next;
  RTE.interface = interface;
  m_table[dest] = RTE;
}

void
RoutingProtocol::AddEntry (const Ipv4Address &dest,
                           const Ipv4Address &mask,
                           const Ipv4Address &next,
                           const Ipv4Address &interfaceAddress)
{
  NS_LOG_FUNCTION(this << dest << next << interfaceAddress << mask << m_CCHAddress);

  NS_ASSERT (m_ipv4);

  for (uint32_t i = 0; i < m_ipv4->GetNInterfaces(); ++i)
   for (uint32_t j = 0; j< m_ipv4->GetNAddresses(i); ++j)
     {
       if (m_ipv4->GetAddress(i,j).GetLocal() == interfaceAddress)
         {
           AddEntry(dest, mask, next, i);
           return;
         }
     }
  //ERROR NO MATCHING INTERFACES
  NS_ASSERT(false);
}

bool
RoutingProtocol::Lookup(Ipv4Address const &dest,
                        RoutingTableEntry &outEntry) const
{
  std::map<Ipv4Address, RoutingTableEntry>::const_iterator it =
    m_table.find(dest);
  if (it != m_table.end())
    {
      outEntry = it->second;
      return true;
    }
  else
    {
      Ipv4Mask MaskTemp;
      uint16_t max_prefix;
      bool max_prefix_meaningful = false;
      for (it = m_table.begin();it!=m_table.end(); ++it)
        {
          MaskTemp.Set (it->second.mask.Get ());
          if (MaskTemp.IsMatch (dest, it->second.destAddr))
            {
              if (!max_prefix_meaningful)
                {
                  max_prefix_meaningful = true;
                  max_prefix = MaskTemp.GetPrefixLength ();
                  outEntry = it->second;
                }
              if (max_prefix_meaningful && (max_prefix < MaskTemp.GetPrefixLength ()))
                {
                  max_prefix = MaskTemp.GetPrefixLength ();
                  outEntry = it->second;
                }
            }
        }
      if (max_prefix_meaningful)
        return true;
      else
        return false;
    }

}

Ptr<Ipv4Route>
RoutingProtocol::RouteOutput (Ptr<Packet> p,
             const Ipv4Header &header,
             Ptr<NetDevice> oif,
             Socket::SocketErrno &sockerr)
{
  NS_LOG_FUNCTION (this << " " << m_ipv4->GetObject<Node> ()->GetId () << " " << header.GetDestination () << " " << oif);
  Ptr<Ipv4Route> rtentry;
  RoutingTableEntry entry;
  if (Lookup (header.GetDestination (), entry))
    {
      //std::cout<<"found!"<<std::endl;
      uint32_t interfaceIdx = entry.interface;
      if (oif && m_ipv4->GetInterfaceForDevice (oif) != static_cast<int> (interfaceIdx))
        {
          // We do not attempt to perform a constrained routing searchTx_Data_Pkts
          // if the caller specifies the oif; we just enforce that
          // that the found route matches the requested outbound interface
          NS_LOG_DEBUG ("SDN node " << m_CCHAddress
                                     << ": RouteOutput for dest=" << header.GetDestination ()
                                     << " Route interface " << interfaceIdx
                                     << " does not match requested output interface "
                                     << m_ipv4->GetInterfaceForDevice (oif));
          sockerr = Socket::ERROR_NOROUTETOHOST;
          std::cout<<"RoutingProtocol::RouteOutput->does not match requested output interface"<<std::endl;
          return rtentry;
        }
      rtentry = Create<Ipv4Route> ();
      rtentry->SetDestination (header.GetDestination ());
      // the source address is the interface address that matches
      // the destination address (when multiple are present on the
      // outgoing interface, one is selected via scoping rules)
      NS_ASSERT (m_ipv4);
      uint32_t numOifAddresses = m_ipv4->GetNAddresses (interfaceIdx);
      NS_ASSERT (numOifAddresses > 0);
      Ipv4InterfaceAddress ifAddr;
      if (numOifAddresses == 1) {
          ifAddr = m_ipv4->GetAddress (interfaceIdx, 0);
        } else {
          NS_FATAL_ERROR ("XXX Not implemented yet:  IP aliasing and SDN");
        }
      rtentry->SetSource (ifAddr.GetLocal ());
      rtentry->SetGateway (entry.nextHop);
      rtentry->SetOutputDevice (m_ipv4->GetNetDevice (interfaceIdx));
      sockerr = Socket::ERROR_NOTERROR;
      NS_LOG_DEBUG ("SDN node " << m_CCHAddress
                                 << ": RouteOutput for dest=" << header.GetDestination ()
                                 << " --> nextHop=" << entry.nextHop
                                 << " interface=" << entry.interface);
      NS_LOG_DEBUG ("Found route to " << rtentry->GetDestination () << " via nh " << rtentry->GetGateway () << " with source addr " << rtentry->GetSource () << " and output dev " << rtentry->GetOutputDevice ());
    }
  else
    {
      NS_LOG_DEBUG ("SDN node " << m_CCHAddress
                                 << ": RouteOutput for dest=" << header.GetDestination ()
                                 << " No route to host");
      sockerr = Socket::ERROR_NOROUTETOHOST;
      std::cout<<"RoutingProtocol::RouteOutput->No route to host"<<std::endl;
    }
  return rtentry;
}

bool
RoutingProtocol::RouteInput(Ptr<const Packet> p,
                            const Ipv4Header &header,
                            Ptr<const NetDevice> idev,
                            UnicastForwardCallback ucb,
                            MulticastForwardCallback mcb,
                            LocalDeliverCallback lcb,
                            ErrorCallback ecb)
{
  NS_LOG_FUNCTION (this << " " << m_ipv4->GetObject<Node> ()->GetId () << " " << header.GetDestination ());
  Ipv4Address dest = header.GetDestination();
  Ipv4Address sour = header.GetSource();

  // Consume self-originated packets
  if (IsMyOwnAddress (sour) == true)
    {
      return true;
    }

  if (header.GetTtl () == 0)
    {
      return true;
    }

  // Local delivery
  NS_ASSERT (m_ipv4->GetInterfaceForDevice (idev) >= 0);
  uint32_t iif = m_ipv4->GetInterfaceForDevice (idev);
  if ((m_appointmentResult == FORWARDER)&&(iif == m_SCHinterface))
    {
      std::cout<<Ipv4toString (m_SCHAddress)<<"Get"<<p->GetUid ()<<"From"<<Ipv4toString (sour)<<std::flush;
    }

  if (m_ipv4->IsDestinationAddress (dest, iif))
    {

      //Local delivery
      if (!lcb.IsNull ())
        {
          NS_LOG_LOGIC ("Broadcast local delivery to " << dest);
          lcb (p, header, iif);
        }
      else
        {
          NS_LOG_ERROR ("Unable to deliver packet locally due to null callback");
          ecb (p, header, Socket::ERROR_NOROUTETOHOST);
          return false;
        }

      //Broadcast forward
      if ((iif == m_SCHinterface) && (m_nodetype == CAR)
           && (m_appointmentResult == FORWARDER) && (!IsInDontForward (sour)))
        {
          NS_LOG_LOGIC ("Forward broadcast");
          Ptr<Ipv4Route> broadcastRoute = Create<Ipv4Route> ();
          broadcastRoute->SetGateway (dest);//broadcast
          broadcastRoute->SetOutputDevice (m_ipv4->GetNetDevice (m_SCHinterface));
          Ipv4Header ipHeader = header;
          ipHeader.SetSource (m_SCHAddress); //To Prevent Brocast Storm, m_CCHAddress is for CCH.
          ipHeader.SetTtl (ipHeader.GetTtl () - 1);
          std::cout<<"FOR"<<std::flush;
          if (ipHeader.GetTtl ()!=0)
            {
              std::cout<<"WARD"<<std::flush;
              if (!m_DD.CheckThis (p))
                ucb (broadcastRoute, p, ipHeader);
              //std::cout<<"FORWARD,UCB,TTL:"<<int(ipHeader.GetTtl())<<std::endl;
            }
        }
      else
        {
          if ((m_appointmentResult == FORWARDER)&&(iif == m_SCHinterface))
            {/*
              if (m_nodetype != CAR)
                std::cout<<"m_nodetype != CAR,"<<std::flush;
              if (IsInDontForward (sour))
                std::cout<<"IsInDontForward (sour),"<<std::flush;
              if (DD_result)
                std::cout<<"DD_result,"<<std::flush;*/
            }
        }
      if ((m_appointmentResult == FORWARDER)&&(iif == m_SCHinterface))
        {
          std::cout<<std::endl;
        }

      if ((m_appointmentResult == FORWARDER)&&((!IsInMyArea (m_mobility->GetPosition ()))||(!m_lc_controllArea_vaild)))
        {
          std::cout<<"!IsInMyArea"<<Ipv4toString (m_CCHAddress)
                   <<",Pos:"<<m_mobility->GetPosition ().x<<","<<m_mobility->GetPosition ().y
                   <<","<<m_mobility->GetPosition ().z<<" Start:"<<m_lc_start.x<<","
                   <<m_lc_start.y<<";End:"<<m_lc_end.x<<","<<m_lc_end.y<<std::endl;
          m_appointmentResult = NORMAL;
        }
      return true;
    }
  //Drop
  return true;
}

void
RoutingProtocol::NotifyInterfaceUp (uint32_t i)
{}
void
RoutingProtocol::NotifyInterfaceDown (uint32_t i)
{}
void
RoutingProtocol::NotifyAddAddress (uint32_t interface, Ipv4InterfaceAddress address)
{}
void
RoutingProtocol::NotifyRemoveAddress (uint32_t interface, Ipv4InterfaceAddress address)
{}

void
RoutingProtocol::SetIpv4 (Ptr<Ipv4> ipv4)
{
  NS_ASSERT (ipv4 != 0);
  NS_ASSERT (m_ipv4 == 0);
  NS_LOG_DEBUG ("Created sdn::RoutingProtocol");
  m_helloTimer.SetFunction 
    (&RoutingProtocol::HelloTimerExpire, this);
  m_queuedMessagesTimer.SetFunction 
    (&RoutingProtocol::SendQueuedMessages, this);
  m_apTimer.SetFunction
    (&RoutingProtocol::APTimerExpire, this);

  m_packetSequenceNumber = SDN_MAX_SEQ_NUM;
  m_messageSequenceNumber = SDN_MAX_SEQ_NUM;


  m_ipv4 = ipv4;
}

void
RoutingProtocol::PrintRoutingTable (Ptr<OutputStreamWrapper> stream) const
{
  std::ostream* os = stream->GetStream ();
  *os << "Destination\t\tMask\t\tNextHop\t\tInterface\tDistance\n";

  for (std::map<Ipv4Address, RoutingTableEntry>::const_iterator iter = 
       m_table.begin ();
       iter != m_table.end (); ++iter)
    {
      *os << iter->first << "\t\t";
      *os << iter->second.mask << "\t\t";
      *os << iter->second.nextHop << "\t\t";
      if (Names::FindName (m_ipv4->GetNetDevice (iter->second.interface)) != "")
        {
          *os << 
          Names::FindName (m_ipv4->GetNetDevice (iter->second.interface)) << 
          "\t\t";
        }
      else
        {
          *os << iter->second.interface << "\t\t";
        }
      *os << "\n";
    }
}

void RoutingProtocol::DoDispose ()
{
  m_ipv4 = 0;

  for (std::map< Ptr<Socket>, Ipv4InterfaceAddress >::iterator iter =
       m_socketAddresses.begin ();
       iter != m_socketAddresses.end (); ++iter)
    {
      iter->first->Close ();
    }
  m_socketAddresses.clear ();
  m_table.clear();

  Ipv4RoutingProtocol::DoDispose ();
}

// SDN packets actually send here.
void
RoutingProtocol::SendPacket (Ptr<Packet> packet,
                             const MessageList &containedMessages)
{
  NS_LOG_DEBUG ("SDN node " << m_CCHAddress << " sending a SDN packet");
  // Add a header
  sdn::PacketHeader header;
  header.originator = m_CCHAddress;//CCH Address
  header.SetPacketLength (header.GetSerializedSize () + packet->GetSize ());
  header.SetPacketSequenceNumber (GetPacketSequenceNumber ());
  packet->AddHeader (header);

  // Trace it
  m_txPacketTrace (header, containedMessages);

  // Send it
  for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator i =
         m_socketAddresses.begin (); i != m_socketAddresses.end (); ++i)
    {
      Ipv4Address bcast = i->second.GetLocal ().GetSubnetDirectedBroadcast (i->second.GetMask ());
      i->first->SendTo (packet, 0, InetSocketAddress (bcast, SDN_PORT_NUMBER));
    }
}

uint16_t
RoutingProtocol::GetPacketSequenceNumber ()
{
  m_packetSequenceNumber = (m_packetSequenceNumber + 1) % (SDN_MAX_SEQ_NUM + 1);
  return m_packetSequenceNumber;
}

uint16_t
RoutingProtocol::GetMessageSequenceNumber ()
{
  m_messageSequenceNumber = (m_messageSequenceNumber + 1) % (SDN_MAX_SEQ_NUM + 1);
  return m_messageSequenceNumber;
}

//
// \brief Processes an incoming %SDN packet (Car Side).
void
RoutingProtocol::RecvSDN (Ptr<Socket> socket)
{
  Ptr<Packet> receivedPacket;
  Address sourceAddress;
  receivedPacket = socket->RecvFrom (sourceAddress);

  InetSocketAddress inetSourceAddr = InetSocketAddress::ConvertFrom (sourceAddress);
  Ipv4Address senderIfaceAddr = inetSourceAddr.GetIpv4 ();
  Ipv4Address receiverIfaceAddr = m_socketAddresses[socket].GetLocal ();
  NS_ASSERT (receiverIfaceAddr != Ipv4Address ());
  NS_LOG_DEBUG ("SDN node " << m_CCHAddress
                << " received a SDN packet from "
                << senderIfaceAddr << " to " << receiverIfaceAddr);

  // All routing messages are sent from and to port RT_PORT,
  // so we check it.
  NS_ASSERT (inetSourceAddr.GetPort () == SDN_PORT_NUMBER);

  Ptr<Packet> packet = receivedPacket;

  sdn::PacketHeader sdnPacketHeader;
  packet->RemoveHeader (sdnPacketHeader);
  NS_ASSERT (sdnPacketHeader.GetPacketLength () >= sdnPacketHeader.GetSerializedSize ());
  uint32_t sizeLeft = sdnPacketHeader.GetPacketLength () - sdnPacketHeader.GetSerializedSize ();

  MessageList messages;

  while (sizeLeft)
    {
      MessageHeader messageHeader;
      if (packet->RemoveHeader (messageHeader) == 0)
        NS_ASSERT (false);

      sizeLeft -= messageHeader.GetSerializedSize ();

      NS_LOG_DEBUG ("SDN Msg received with type "
                    << std::dec << int (messageHeader.GetMessageType ())
                    << " TTL=" << int (messageHeader.GetTimeToLive ())
                    << " SeqNum=" << messageHeader.GetMessageSequenceNumber ());
      messages.push_back (messageHeader);
    }

  m_rxPacketTrace (sdnPacketHeader, messages);
  
  for (MessageList::const_iterator messageIter = messages.begin ();
       messageIter != messages.end (); ++messageIter)
    {
      const MessageHeader &messageHeader = *messageIter;
      // If ttl is less than or equal to zero, or
      // the receiver is the same as the originator,
      // the message must be silently dropped
      if ((messageHeader.GetTimeToLive () == 0)||(IsMyOwnAddress (sdnPacketHeader.originator)))
        {
          // ignore it
          packet->RemoveAtStart (messageHeader.GetSerializedSize ());
          continue;
        }


      switch (messageHeader.GetMessageType ())
        {
          case sdn::MessageHeader::HELLO_MESSAGE:
            NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                          << "s SDN node " << m_CCHAddress
                          << " received Routing message of size "
                          << messageHeader.GetSerializedSize ());
            //Car Node should discare Hello_Message
            if (GetType() == LOCAL_CONTROLLER)
              ProcessHM (messageHeader);
            break;

          case sdn::MessageHeader::APPOINTMENT_MESSAGE:
            NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                          << "s SDN node " << m_CCHAddress
                          << " received Appointment message of size "
                          << messageHeader.GetSerializedSize ());
            if (GetType() == CAR)
              ProcessAppointment (messageHeader);
            break;

          case sdn::MessageHeader::ACKHELLO_MESSAGE:
            NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                          << "s SDN node " << m_CCHAddress
                          << " received ACKHELLO message of size "
                          << messageHeader.GetSerializedSize ());
            if (GetType() == CAR)
              ProcessAckHello (messageHeader);
            break;

          case sdn::MessageHeader::DONT_FORWARD:
            NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                          << "s SDN node " << m_CCHAddress
                          << " received DONT_FORWARD message of size "
                          << messageHeader.GetSerializedSize ());
            if (GetType() == CAR)
              ProcessDontForward (messageHeader);
            break;

          case sdn::MessageHeader::LC2LC:
            NS_LOG_DEBUG (Simulator::Now ().GetSeconds ()
                          << "s SDN node " << m_CCHAddress
                          << " received LC2LC message of size "
                          << messageHeader.GetSerializedSize ());
            if (GetType() == LOCAL_CONTROLLER)
              ProcessLc2Lc (messageHeader, senderIfaceAddr);
            break;
          default:
            NS_LOG_DEBUG ("SDN message type " <<
                          int (messageHeader.GetMessageType ()) <<
                          " not implemented");
        }

    }
    
}// End of RecvSDN

void
RoutingProtocol::HelloTimerExpire ()
{
  if (GetType() == CAR)
    {
      if (ShouldISendHello ())
        SendHello ();
      m_helloTimer.Schedule (m_helloInterval);
    }
}

void
RoutingProtocol::APTimerExpire ()
{
  if (GetType() == LOCAL_CONTROLLER)
    {
      ComputeRoute ();
    }
}

void
RoutingProtocol::QueueMessage (const sdn::MessageHeader &message, Time delay)
{
   m_queuedMessages.push_back (message);
  if (not m_queuedMessagesTimer.IsRunning ())
    {
      m_queuedMessagesTimer.SetDelay (delay);
      m_queuedMessagesTimer.Schedule ();
    }
}

// NS3 is not multithread, so mutex is unnecessary.
// Here, messages will queue up and send once numMessage is equl to SDN_MAX_MSGS.
// This function will NOT add a header to each message
void
RoutingProtocol::SendQueuedMessages ()
{
  Ptr<Packet> packet = Create<Packet> ();
  int numMessages = 0;

  NS_LOG_DEBUG ("SDN node " << m_CCHAddress << ": SendQueuedMessages");
  MessageList msglist;

  for (std::vector<sdn::MessageHeader>::const_iterator message = m_queuedMessages.begin ();
       message != m_queuedMessages.end ();
       ++message)
    {
      Ptr<Packet> p = Create<Packet> ();
      p->AddHeader (*message);
      packet->AddAtEnd (p);
      msglist.push_back (*message);
      if (++numMessages == SDN_MAX_MSGS)
        {
          SendPacket (packet, msglist);
          msglist.clear ();
          // Reset variables for next packet
          numMessages = 0;
          packet = Create<Packet> ();
        }
    }

  if (packet->GetSize ())
    {
      SendPacket (packet, msglist);
    }

  m_queuedMessages.clear ();
}

void
RoutingProtocol::SendHello ()
{
  NS_LOG_FUNCTION (this);
  sdn::MessageHeader msg;
  Time now = Simulator::Now ();
  msg.SetVTime (m_helloInterval);
  msg.SetTimeToLive (41993);//Just MY Birthday.
  msg.SetMessageSequenceNumber (GetMessageSequenceNumber ());
  msg.SetMessageType (sdn::MessageHeader::HELLO_MESSAGE);

  sdn::MessageHeader::Hello &hello = msg.GetHello ();
  hello.ID = m_SCHAddress;
  Vector pos = m_mobility->GetPosition ();
  Vector vel = m_mobility->GetVelocity ();
  hello.SetPosition (pos.x, pos.y, pos.z);
  hello.SetVelocity (vel.x, vel.y, vel.z);

  NS_LOG_DEBUG ( "SDN HELLO_MESSAGE sent by node: " << hello.ID
                 << "   at " << now.GetSeconds() << "s");
  QueueMessage (msg, JITTER);
}

void
RoutingProtocol::SendAppointment ()
{
  NS_LOG_FUNCTION (this);

  for (std::map<Ipv4Address, CarInfo>::const_iterator cit = m_lc_info.begin ();
       cit != m_lc_info.end (); ++cit)
    {
      sdn::MessageHeader msg;
      Time now = Simulator::Now ();
      msg.SetVTime (m_helloInterval);
      msg.SetTimeToLive (41993);//Just MY Birthday.
      msg.SetMessageSequenceNumber (GetMessageSequenceNumber ());
      msg.SetMessageType (sdn::MessageHeader::APPOINTMENT_MESSAGE);
      sdn::MessageHeader::Appointment &appointment = msg.GetAppointment ();
      appointment.ID = cit->first;
      appointment.ATField = cit->second.appointmentResult;
      QueueMessage (msg, JITTER);
    }
}

void
RoutingProtocol::SendAckHello (const Ipv4Address& ID)
{
  NS_LOG_FUNCTION (this);
  sdn::MessageHeader msg;
  Time now = Simulator::Now ();
  msg.SetVTime (m_helloInterval);
  msg.SetTimeToLive (41993);//Just MY Birthday.
  msg.SetMessageSequenceNumber (GetMessageSequenceNumber ());
  msg.SetMessageType (sdn::MessageHeader::ACKHELLO_MESSAGE);

  sdn::MessageHeader::AckHello &ackhello = msg.GetAckHello ();
  ackhello.ID = ID;
  Vector3D pos = m_lc_info[ID].Position;
  Vector3D vel = m_lc_info[ID].Velocity;
  ackhello.SetPosition (pos.x, pos.y, pos.z);
  ackhello.SetVelocity (vel.x, vel.y, vel.z);
  ackhello.SetControllArea_Start (m_lc_start);
  ackhello.SetControllArea_End (m_lc_end);
  /*std::cout<<Ipv4toString (ackhello.ID)<<",SEND "
           <<m_lc_start.x<<","<<m_lc_start.y<<";"
           <<m_lc_end.x<<","<<m_lc_end.y<<std::endl;*/

  NS_LOG_DEBUG ( "SDN ACKHELLO_MESSAGE sent by node: " << ackhello.ID
                 << "   at " << now.GetSeconds() << "s");
  QueueMessage (msg, JITTER);
}

void
RoutingProtocol::SendDontForward (const Ipv4Address& ID)
{
  NS_LOG_FUNCTION (this);
  sdn::MessageHeader msg;
  Time now = Simulator::Now ();
  msg.SetVTime (m_helloInterval);
  msg.SetTimeToLive (41993);//Just MY Birthday.
  msg.SetMessageSequenceNumber (GetMessageSequenceNumber ());
  msg.SetMessageType (sdn::MessageHeader::DONT_FORWARD);
  sdn::MessageHeader::DontForward &dontforward = msg.GetDontForward ();
  dontforward.ID = ID;
  //dontforward.list.clear ();
  dontforward.list = m_lc_info[ID].list_of_dont_forward;
  dontforward.list_size = dontforward.list.size ();
  //dontforward.list_size = 0;
  QueueMessage (msg, JITTER);
}

void
RoutingProtocol::SendLc2Lc ()
{
  NS_LOG_FUNCTION (this);
  sdn::MessageHeader msg;
  Time now = Simulator::Now ();
  msg.SetVTime (m_helloInterval);
  msg.SetTimeToLive (41993);//Just MY Birthday.
  msg.SetMessageSequenceNumber (GetMessageSequenceNumber ());
  msg.SetMessageType (sdn::MessageHeader::LC2LC);
  sdn::MessageHeader::Lc2Lc &lc2lc = msg.GetLc2Lc ();
  lc2lc.ID = m_CCHAddress;//CCH Address.

  for (std::list<Ipv4Address>::const_iterator cit = m_forward_chain.begin ();
       cit != m_forward_chain.end (); ++cit)
    {
      lc2lc.list.push_back (*cit);
    }

  lc2lc.list_size = lc2lc.list.size ();
  QueueMessage (msg, JITTER);
}

void
RoutingProtocol::ProcessHM (const sdn::MessageHeader &msg)
{
  if (IsInMyArea (msg.GetHello ().GetPosition ()))
    {
      Ipv4Address ID = msg.GetHello ().ID;
      std::map<Ipv4Address, CarInfo>::iterator it = m_lc_info.find (ID);

      if (it != m_lc_info.end ())
        {
          it->second.Active = true;
          Time oldT = it->second.LastActive;
          it->second.LastActive = Simulator::Now ();
          Vector3D oldP = it->second.Position;
          Vector3D oldV = it->second.Velocity;
          it->second.Position = msg.GetHello ().GetPosition ();
          it->second.Velocity = msg.GetHello ().GetVelocity ();
          if (it->second.Velocity.x+it->second.Velocity.y+it->second.Velocity.z < 1)//To privent Dead Point
            {
              it->second.Velocity.z = 10;
            }
          if (it->second.appointmentResult == FORWARDER)
            {
              std::cout<<Ipv4toString (ID)<<",("<<it->second.Position.x<<","<<it->second.Position.y<<")"<<std::endl;
              double delta_t = it->second.LastActive.GetSeconds () - oldT.GetSeconds ();
              Vector3D pridict_pos = Vector3D(oldP.x + oldV.x * delta_t,
                                              oldP.y + oldV.y * delta_t,
                                              oldP.z + oldV.z * delta_t);
              double distance = CalculateDistance (pridict_pos, it->second.Position );

              if (!CheckLink ())//((distance > ((1-m_safety_raito)/2) * m_signal_range))
                {
                  std::cout<<"miscalculation on FORWARDER Car:"<<Ipv4toString (ID)<<std::endl;
                  m_linkEstablished = false;
                  ComputeRoute ();
                }
            }
        }
      else
        {
          CarInfo CI_temp;
          CI_temp.Active = true;
          CI_temp.LastActive = Simulator::Now ();
          CI_temp.Position = msg.GetHello ().GetPosition ();
          CI_temp.Velocity = msg.GetHello ().GetVelocity ();
          if (CI_temp.Velocity.x+CI_temp.Velocity.y+CI_temp.Velocity.z < 1)//To privent Dead Point
            {
              CI_temp.Velocity.z = 10;
            }
          m_lc_info[ID] = CI_temp;
        }

      SendAckHello (ID);
    }
  else
    {
      Ipv4Address ID = msg.GetHello ().ID;
      std::map<Ipv4Address, CarInfo>::iterator it = m_lc_info.find (ID);
      if (it != m_lc_info.end ())
      {
        it->second.Active = true;
        Time oldT = it->second.LastActive;
        it->second.LastActive = Simulator::Now ();
        Vector3D oldP = it->second.Position;
        Vector3D oldV = it->second.Velocity;
        it->second.Position = msg.GetHello ().GetPosition ();
        it->second.Velocity = msg.GetHello ().GetVelocity ();
        if (it->second.Velocity.x+it->second.Velocity.y+it->second.Velocity.z < 1)//To privent Dead Point
          {
            it->second.Velocity.z = 10;
          }
        if (it->second.appointmentResult == FORWARDER)
          {
             ComputeRoute ();
          }
      }
    }
  //std::cout<<"V:"<<m_lc_info[ID].Velocity.x<<std::endl;
}

void
RoutingProtocol::ProcessAppointment (const sdn::MessageHeader &msg)
{
  NS_LOG_FUNCTION (msg);
  const sdn::MessageHeader::Appointment &appointment = msg.GetAppointment ();
  if (IsMyOwnAddress (appointment.ID))
    {
      switch (appointment.ATField)
      {
        case NORMAL:
          break;
        case FORWARDER:
          std::cout<<Ipv4toString (m_SCHAddress)<<"GET APPOINTMENT."<<std::endl;
          break;
        default:
          std::cout<<"RoutingProtocol::ProcessAppointment->ERROR TYPE"<<std::endl;
      }
      m_appointmentResult = appointment.ATField;
    }
}

void
RoutingProtocol::ProcessAckHello (const sdn::MessageHeader &msg)
{
  NS_LOG_FUNCTION (msg);
  const sdn::MessageHeader::AckHello &ackhello = msg.GetAckHello ();
  if (IsMyOwnAddress (ackhello.ID))
    {
      m_car_lc_ack_pos = ackhello.GetPosition ();
      m_car_lc_ack_vel = ackhello.GetVelocity ();
      m_car_lc_ack_time = Simulator::Now ();
      m_car_lc_ack_vaild = true;
      Vector2D new_start = ackhello.GetControllArea_Start ();
      Vector2D new_end = ackhello.GetControllArea_End ();
      if (m_lc_controllArea_vaild)
        {
          if ((m_lc_start.x != new_start.x)||(m_lc_start.y != new_start.y)||
              (m_lc_end.x != new_end.x)||(m_lc_end.y != new_end.y)) //Come to an new area~
            {
              std::cout<<"NEWAREA"<<Ipv4toString (ackhello.ID)<<std::endl;
              m_appointmentResult = NORMAL;
            }
        }
      m_lc_start = new_start;
      m_lc_end = new_end;
      /*std::cout<<Ipv4toString (ackhello.ID)<<",ACK "
               <<m_lc_start.x<<","<<m_lc_start.y<<";"
               <<m_lc_end.x<<","<<m_lc_end.y<<std::endl;*/
      m_lc_controllArea_vaild = true;
    }
}

void
RoutingProtocol::ProcessDontForward (const sdn::MessageHeader &msg)
{
  NS_LOG_FUNCTION (msg);

  const sdn::MessageHeader::DontForward &dontforward = msg.GetDontForward ();
  if (IsMyOwnAddress (dontforward.ID))
    {
      Time now = Simulator::Now();
      NS_LOG_DEBUG ("@" << now.GetSeconds() << ":Node " << m_CCHAddress
                    << "ProcessDontForward.");

      NS_ASSERT (dontforward.list_size >= 0);

      m_dont_forward.clear ();
      std::cout<<"Car:"<<Ipv4toString (m_SCHAddress)<<"("<<m_mobility->GetPosition ().x
          <<","<<m_mobility->GetPosition ().y<<"):::";
      for (std::vector<Ipv4Address>::const_iterator cit = dontforward.list.begin ();
           cit != dontforward.list.end (); ++cit)
        {
          m_dont_forward.insert (*cit);
          std::cout<<Ipv4toString (*cit)<<";";
        }
      std::cout<<std::endl;
    }
}

void
RoutingProtocol::ProcessLc2Lc (const sdn::MessageHeader &msg, const Ipv4Address &sour)
{
  NS_LOG_FUNCTION (msg);
  //std::cout<<"RoutingProtocol::ProcessLc2Lc!RoutingProtocol::ProcessLc2Lc!"<<std::endl;
  const sdn::MessageHeader::Lc2Lc &lc2lc = msg.GetLc2Lc ();
  if (lc2lc.ID == sour)
    {
      //std::cout<<"lc2lc.ID == sour!lc2lc.ID == sour!"<<std::endl;
      Time now = Simulator::Now();
      NS_LOG_DEBUG ("@" << now.GetSeconds() << ":Node " << m_CCHAddress
                    << "ProcessLc2Lc.");

      NS_ASSERT (lc2lc.list_size >= 0);

      if (m_lc_headNtail.find (lc2lc.ID) != m_lc_headNtail.end ())
        {
          m_lc_headNtail[lc2lc.ID].clear ();
        }

      std::set<Ipv4Address> forward_chain_set;
      for (std::list<Ipv4Address>::const_iterator cit = m_forward_chain.begin ();
           cit != m_forward_chain.end (); ++cit)
        {
          forward_chain_set.insert (*cit);
        }

      bool reCompute = false;
      for (std::vector<Ipv4Address>::const_iterator cit = lc2lc.list.begin ();
           cit != lc2lc.list.end (); ++cit)
        {
          if ((m_linkEstablished)&&(forward_chain_set.find (*cit)!=forward_chain_set.end ()))
            {
              m_linkEstablished = false;
              reCompute = true;
            }
          if (m_lc_info.find (*cit) != m_lc_info.end ())
            {
              m_lc_info.erase (*cit);
            }
          m_lc_headNtail[lc2lc.ID].push_back (*cit);
        }

      if (m_linkEstablished)
        {
          for (std::list<Ipv4Address>::const_iterator cit = m_forward_chain.begin ();
               cit != m_forward_chain.end (); ++cit)
            {
              CalcDontForward (*cit);
              SendDontForward (*cit);
            }
        }
      if (reCompute)
        {
          ComputeRoute ();
        }

    }
}

void
RoutingProtocol::ComputeRoute ()
{
  RemoveLeaved (); //Remove Stale Tuple

  if (m_algorithm == Yangs_Algo)
    {
      if (!m_linkEstablished)
        {
          std::cout<<"Do_Init_Compute"<<std::endl;
          Do_Init_Compute ();
        }
      else
        {
          std::cout<<"Do_Update"<<std::endl;
          Do_Update ();
        }
      if (m_linkEstablished)
        {
          SendLc2Lc ();
          std::cout<<Ipv4toString (m_CCHAddress)<<" SendLC2LC"<<std::endl;
          std::cout<<"CHAIN:"<<std::endl;
          for (std::list<Ipv4Address>::const_iterator cit = m_forward_chain.begin ();
               cit != m_forward_chain.end (); ++cit)
            {
              std::cout<<Ipv4toString((*cit))<<"("<<CalcDist (m_lc_info[*cit].GetPos (),m_lc_start)<<"),"<<std::endl;
              CalcDontForward (*cit);
              SendDontForward (*cit);
            }
          std::cout<<"SendAppointment"<<std::endl;
          SendAppointment ();
        }
      Reschedule ();
    }
  else
    {
      std::cout<<"BinarySearch"<<std::endl;
      BinarySearch ();
      if (m_linkEstablished)
        {
          std::cout<<Ipv4toString (m_CCHAddress)<<"SendLC2LC"<<std::endl;
          SendLc2Lc ();
          std::cout<<"CHAIN:"<<std::endl;
          for (std::list<Ipv4Address>::const_iterator cit = m_forward_chain.begin ();
               cit != m_forward_chain.end (); ++cit)
            {
              std::cout<<Ipv4toString((*cit))<<"("<<CalcDist (m_lc_info[*cit].GetPos (),m_lc_start)<<"),"<<std::endl;
              CalcDontForward (*cit);
              SendDontForward (*cit);
            }
          SendAppointment ();
          std::cout<<"SendAppointment"<<std::endl;
        }
      BSReschedule ();
    }
  std::cout<<"CR DONE"<<std::endl;
}//RoutingProtocol::ComputeRoute

bool
RoutingProtocol::IsMyOwnAddress (const Ipv4Address & a) const
{
  for (std::map<Ptr<Socket>, Ipv4InterfaceAddress>::const_iterator j =
         m_socketAddresses.begin (); j != m_socketAddresses.end (); ++j)
    {
      Ipv4InterfaceAddress iface = j->second;
      if (a == iface.GetLocal ())
        {
          return true;
        }
    }
  if (a == m_SCHAddress)
    {
      return true;
    }
  return false;
}

void
RoutingProtocol::SetMobility (Ptr<MobilityModel> mobility)
{
  m_mobility = mobility;
}

bool
RoutingProtocol::IsInDontForward (const Ipv4Address& id) const
{
  return (m_dont_forward.find (id) != m_dont_forward.end ());
}


void
RoutingProtocol::SetType (NodeType nt)
{
  m_nodetype = nt;
}

NodeType
RoutingProtocol::GetType () const
{
  return m_nodetype;
}

ShortHop
RoutingProtocol::GetShortHop(const Ipv4Address& IDa, const Ipv4Address& IDb)
{
  double const va = GetProjection (m_lc_info[IDa].Velocity),
               vb = GetProjection (m_lc_info[IDb].Velocity);
  //Predict
  double const da = CalcDist (m_lc_info[IDa].GetPos (), m_lc_start),
               db = CalcDist (m_lc_info[IDb].GetPos (), m_lc_start);
  // time to b left
  double temp;

  const double safe_range = m_signal_range * m_safety_raito;

  if (vb > 0)
    {
      temp = (m_road_length - db) / vb;
    }
  else
    {
      //b is fixed.
      temp = (m_road_length - da) / va;
    }
  double const t2bl = temp;
  if ((db - da < safe_range) && (dabs((db + vb*t2bl)-(da + va*t2bl)) < safe_range))
    {
      ShortHop sh;
      sh.nextID = IDb;
      sh.hopnumber = m_lc_info[IDb].minhop + 1;
      sh.isTransfer = false;
      return sh;
    }//if ((pxb -  ...
  else
    {
      ShortHop sh;
      sh.isTransfer = true;
      sh.t = 0; // Time when connection loss
      sh.hopnumber = INFHOP;
      if (db - da < safe_range)
        {
          if (vb > va)
            {
              sh.t = (safe_range + da - db) / (vb - va);
            }
          else
            {
              sh.t = (safe_range + db - da) / (va - vb);
            }
        }
      //Find another car
      for (std::map<Ipv4Address, CarInfo>::const_iterator cit = m_lc_info.begin ();
           cit != m_lc_info.end (); ++cit)
        {
          double const vc = GetProjection (cit->second.Velocity);
          //pxc when t
          double const tdc = CalcDist (cit->second.GetPos (), m_lc_start) + vc * sh.t;
          //pxa and pxb when t
          double const tda = da + va * sh.t,
                       tdb = db + vb * sh.t;
          //t2bl minus t
          double const t2blmt = t2bl - sh.t;
          if ((tda<tdc)&&(tdc<tdb)&&(tdc-tda<safe_range)&&(tdb-tdc<safe_range))
            {
              if ((dabs((tdb + vb*t2blmt)-(tdc + vc*t2blmt)) < safe_range)&&
                  (dabs((tdc + vc*t2blmt)-(tda + va*t2blmt)) < safe_range))
                {
                  sh.IDa = IDa;
                  sh.IDb = IDb;
                  sh.proxyID = cit->first;
                  sh.hopnumber = m_lc_info[IDb].minhop + 2;
                  return sh;
                }//if ((abs((tpxb ...
            }//if ((tpxa ...
        }//for (std::map<I ...
      return sh;
    }//else
}

int
RoutingProtocol::GetArea (Vector3D position) const
{
  double distance = CalcDist (position, m_lc_start);
  double road_length = m_road_length;
  //0.5r ~ r ~ r ~...~ r ~ r ~ last (if length_of_last<=0.5r, last={0.5r}; else last = {padding_area, 0.5r});
  if (distance < 0.5*m_signal_range)
    {
      //std::cout<<"RET1"<<std::endl;
      return 0;
    }
  else
    {
      road_length -= 0.5*m_signal_range;
      int numOfTrivialArea = road_length / m_signal_range;
      double remain = road_length - (numOfTrivialArea * m_signal_range);
      if (remain<0.5*m_signal_range)
        numOfTrivialArea--;

      distance -= 0.5*m_signal_range;
      if (distance < numOfTrivialArea * m_signal_range)
        {
          return (distance / m_signal_range) + 1;
        }
      else
        {
          if (road_length - distance < 0.5*m_signal_range)
            {
              if (isPaddingExist())
                return numOfTrivialArea + 2;
              else
                return numOfTrivialArea + 1;
            }
          else
            {
              return numOfTrivialArea + 1;
            }
        }
    }
}

int
RoutingProtocol::GetNumArea () const
{
  return m_numArea;
}

void
RoutingProtocol::Init_NumArea ()
{
  int ret;
  double road_length = m_road_length;
  if (road_length < 0.5*m_signal_range)
    {
      ret = 1;
    }
  else
    {
      road_length -= 0.5*m_signal_range;
      int numOfTrivialArea = road_length / m_signal_range;
      double last_length = road_length - (m_signal_range * numOfTrivialArea);
      if (last_length < 0.5*m_signal_range)
        {
          numOfTrivialArea--;
          last_length += m_signal_range;
        }
      if ((last_length-0.5*m_signal_range) < 1e-10)//last_length == 0.5r
        {
          ret = 1 + numOfTrivialArea + 1;//First Area + TrivialArea + LastArea;
          m_isPadding = false;
        }
      else
        if (last_length > m_signal_range)//r<last_length
          {
            ret = 1 + numOfTrivialArea + 2;//First Area + TrivialArea + paddingArea(>0.5r) +LastArea(0.5r);
            m_isPadding = true;
          }
        else//0.5r<last_length<r
          {
            ret = 1 + numOfTrivialArea + 2;//First Area + TrivialArea + PaddingArea(<0.5r) +LastArea(0.5r);
            m_isPadding = true;
          }
    }
  m_numArea = ret;
  m_numAreaVaild = true;
}

bool
RoutingProtocol::isPaddingExist () const
{
  return m_isPadding;
}

void
RoutingProtocol::RemoveLeaved()
{
  Time now = Simulator::Now ();
  std::map<Ipv4Address, CarInfo>::iterator it = m_lc_info.begin ();
  std::vector<Ipv4Address> pendding;
  while (it != m_lc_info.end ())
    {
      if (!IsInMyArea (it->second.GetPos ()))
        {
          //std::cout<<"!IsInMyArea (it->second.GetPos ())"<<std::endl;
          pendding.push_back (it->first);
        }
      ++it;
    }
  for (std::vector<Ipv4Address>::iterator it = pendding.begin ();
      it != pendding.end(); ++it)
    {
      m_lc_info.erase((*it));
    }
}

void
RoutingProtocol::SetSignalRange (double signal_range)
{
  m_signal_range = signal_range;
}

void
RoutingProtocol::BinarySearch ()
{
  SortByDistance ();//LC_END->LC_START
  double LowerBound = 0;
  double UpperBound = FindUpperBound ();
  std::cout<<"Upper:"<<UpperBound<<std::endl;
  uint32_t BS_count = 0;
  while (UpperBound - LowerBound > 0.5)
    {
      double mid = (UpperBound + LowerBound)/2;
      ++BS_count;
      if (TestResult (mid))
        {
          LowerBound = mid;
        }
      else
        {
          UpperBound = mid;
        }
    }

  ++BS_count;
  std::cout<<"BS_count:"<<BS_count<<",LowerBound:"<<LowerBound<<std::endl;

  //SetForwardChain
  if (TestResult (LowerBound))//Double Check and Set m_linkEstablished/m_lc_info
    {
      if (LowerBound > 1)
        {
          m_linkEstablished = true;
          //build chain
          double const LengthOfFirstArea = 0.5 * m_signal_range;
          Ipv4Address The_Car,
                      ZERO = Ipv4Address::GetZero ();
          double minhop = INFHOP;
          int pos = m_bs_sort.size () - 1;
          while (pos>=0)
            {
              if (m_bs_sort[pos].second > LengthOfFirstArea)
                {
                  break;
                }
              if (m_lc_info[m_bs_sort[pos].first].minhop < minhop)
                {
                  minhop = m_lc_info[m_bs_sort[pos].first].minhop;
                  The_Car = m_bs_sort[pos].first;
                }
              --pos;
            }
          m_theFirstCar = The_Car;
          m_forward_chain.clear ();
          ResetAppointmentResult ();
          while (The_Car != ZERO)
            {
              m_forward_chain.push_back (The_Car);
              m_lc_info[The_Car].appointmentResult = FORWARDER;
              The_Car = m_lc_info[The_Car].ID_of_minhop;
            }

          m_lowerbound = LowerBound;
        }
      else
        m_linkEstablished = false;
    }
  else
    {
      m_linkEstablished = false;
    }
}

bool
RoutingProtocol::TestResult (double result)
{
  //INIT
  bool flag = true;//whether we still processing the last area
  double const LengthOfLastArea = 0.5 * m_signal_range;
  for (std::vector<std::pair<Ipv4Address, double> >::const_iterator cit = m_bs_sort.begin ();
       cit != m_bs_sort.end (); ++cit)
    {
      if (flag)
        {
          double const d = (m_road_length - cit->second);
          if (d < LengthOfLastArea)
            {
              double const v = GetProjection (m_lc_info[cit->first].Velocity);
              m_lc_info[cit->first].ID_of_minhop = Ipv4Address::GetZero ();
              if ((v * result + cit->second)<m_road_length)
                m_lc_info[cit->first].minhop = 1;
              else
                m_lc_info[cit->first].minhop = INFHOP;
            }
          else
            {
              flag = false;
              m_lc_info[cit->first].ID_of_minhop = Ipv4Address::GetZero ();
              m_lc_info[cit->first].minhop = INFHOP;
            }
        }
      else
        {
          m_lc_info[cit->first].ID_of_minhop = Ipv4Address::GetZero ();
          m_lc_info[cit->first].minhop = INFHOP;
        }
    }


  //DP
  for (uint32_t i = 0; i<m_bs_sort.size () ;++i)
    {
      //std::cout<<Ipv4toString (m_bs_sort[i].first)<<","<<m_bs_sort[i].second<<std::endl;
      if ((m_road_length - m_bs_sort[i].second) >= LengthOfLastArea)
        {
          for (int j = i - 1; j >= 0; --j)
            {
              if (m_bs_sort[j].second - m_bs_sort[i].second > m_road_length * m_safety_raito)
                {
                  break;
                }
              double const vi = GetProjection (m_lc_info[m_bs_sort[i].first].Velocity);
              double const vj = GetProjection (m_lc_info[m_bs_sort[j].first].Velocity);
              double const hppi = (vi * (2 - m_safety_raito) * result + m_bs_sort[i].second);//High Predict Position of I
              double const lppi = (vi * m_safety_raito * result + m_bs_sort[i].second);//Low Predict Position of I
              bool const b4ileft = hppi < m_road_length;
              double const hppj = (vj * (2 - m_safety_raito) * result + m_bs_sort[j].second);//High Predict Position of J
              double const lppj = (vj * result + m_bs_sort[j].second);//Low Predict Position of J
              bool const b4jleft = hppj < m_road_length;
              bool const b4ijlost = (dabs (hppi - lppj) < m_signal_range)
                                  &&(dabs (lppi - hppj) < m_signal_range);
              if (b4ileft && b4jleft && b4ijlost)
                {
                  if (m_lc_info[m_bs_sort[i].first].minhop >  m_lc_info[m_bs_sort[j].first].minhop + 1)
                    {
                      m_lc_info[m_bs_sort[i].first].minhop = m_lc_info[m_bs_sort[j].first].minhop + 1;
                      m_lc_info[m_bs_sort[i].first].ID_of_minhop = m_bs_sort[j].first;
                    }
                }
            }
        }
    }

  //Check the first Area
  bool ret = false;
  for (int i = m_bs_sort.size()-1; i>=0; --i)
    {
      if (m_bs_sort[i].second < 0.5*m_signal_range)
        {
          if (m_lc_info[m_bs_sort[i].first].minhop != INFHOP)
            {
              ret = true;
              break;
            }
        }
      else
        {
          break;
        }
    }
  return ret;
}


double
RoutingProtocol::FindUpperBound ()
{
  double ret = 0;
  double const LengthOfFirstArea = 0.5 * m_signal_range;
  for (std::vector<std::pair<Ipv4Address, double> > ::const_reverse_iterator crit = m_bs_sort.rbegin ();
       crit != m_bs_sort.rend (); ++crit)
    {
      double const v = GetProjection (m_lc_info[crit->first].Velocity);
      double const d = crit->second;
      if (d>LengthOfFirstArea)
        break;
      double t2l = (LengthOfFirstArea - d) / v;
      if (t2l > ret)
        ret = t2l;
    }
  return ret;
}

void
RoutingProtocol::SortByDistance ()
{
  m_bs_sort.clear ();

  std::list<std::pair<double, Ipv4Address> > templist;

  for (std::map<Ipv4Address, CarInfo>::const_iterator cit = m_lc_info.begin ();
      cit != m_lc_info.end (); ++cit)
    {

      Vector3D temp3D = cit->second.GetPos ();
      double distance = CalculateDistance (Vector2D(temp3D.x, temp3D.y), m_lc_start);

      templist.push_back (std::pair<double, Ipv4Address>(distance, cit->first));
    }

  templist.sort (RoutingProtocol::Comp);

  for (std::list<std::pair<double, Ipv4Address> >::const_iterator cit = templist.begin ();
       cit != templist.end (); ++cit)
    {
      //std::cout<<cit->second.Get () % 256<<":"<<cit->first<<std::endl;
      m_bs_sort.push_back (std::pair<Ipv4Address, double>(cit->second, cit->first));
    }
}

void
RoutingProtocol::BSReschedule ()
{
  if (!m_linkEstablished)
    {
      if (m_apTimer.IsRunning ())
        {
          m_apTimer.Remove ();
        }
      m_apTimer.Schedule (m_minAPInterval);
      //std::cout<<"Reschedule:"<<m_minAPInterval.GetSeconds ()<<"s."<<std::endl;
    }
  else
    {
      if (m_apTimer.IsRunning ())
        {
          m_apTimer.Remove ();
        }
      m_apTimer.Schedule (Seconds (m_lowerbound));
    }
}



void
RoutingProtocol::Do_Init_Compute ()
{
  std::cout<<"Partition"<<std::endl;
  Partition ();

  std::cout<<"SetN_Init"<<std::endl;
  SetN_Init ();

  std::cout<<"OtherSet_Init"<<std::endl;
  OtherSet_Init ();

  /*std::cout<<"Next:";
  for (std::map<Ipv4Address, CarInfo>::const_iterator cit=m_lc_info.begin ();
       cit!=m_lc_info.end (); ++cit)
    {
      std::cout<<cit->first.Get ()%256<<"->"<<cit->second.ID_of_minhop.Get ()%256<<","<<cit->second.minhop<<";";
    }
  std::cout<<std::endl;
*/
  std::cout<<"SelectNode"<<std::endl;
  SelectNode ();
  std::cout<<"Do_Init_Compute DONE"<<std::endl;
}

void
RoutingProtocol::Do_Update ()
{
  std::cout<<"Partition"<<std::endl;
  Partition ();
  std::cout<<"CalcSetZero"<<std::endl;
  CalcSetZero ();
  std::cout<<"SelectNewNodeInAreaZero"<<std::endl;
  SelectNewNodeInAreaZero ();
  std::cout<<"Do_Update DONE"<<std::endl;
  if (!m_linkEstablished)
    {
      std::cout<<"!m_linkEstablished"<<std::endl;
      Do_Init_Compute ();
    }
}

void
RoutingProtocol::Reschedule ()
{
  if (!m_linkEstablished)
    {
      if (m_apTimer.IsRunning ())
        {
          m_apTimer.Remove ();
        }
      m_apTimer.Schedule (m_minAPInterval);
      //std::cout<<"Reschedule:"<<m_minAPInterval.GetSeconds ()<<"s."<<std::endl;
    }
  else
    {
      double vx = GetProjection (m_lc_info[m_theFirstCar].Velocity);
      double dx = CalcDist (m_lc_info[m_theFirstCar].GetPos (), m_lc_start);
      double t2l;
      if (vx < 1e-7)
        {
          t2l = m_minAPInterval.GetSeconds ();
        }
      else
        {
          t2l= (0.5 * m_signal_range - dx) / vx;
        }
      if (m_apTimer.IsRunning ())
        {
          m_apTimer.Remove ();
        }
      if (t2l<1e-5)
        {
          t2l = m_minAPInterval.GetSeconds ();
        }
      if (m_apTimer.IsRunning ())
        {
          m_apTimer.Remove ();
        }
      m_apTimer.Schedule(Seconds(t2l));
      //std::cout<<"Reschedule:"<<t2l<<"s."<<"p:"<<dx<<",v:"<<vx<<std::endl;
    }
}

void
RoutingProtocol::CalcDontForward (const Ipv4Address& ID)
{
  CarInfo& carinfo = m_lc_info[ID];
  carinfo.list_of_dont_forward.clear ();
  for (std::list<Ipv4Address>::const_iterator cit = m_forward_chain.begin ();
       cit != m_forward_chain.end (); ++cit)
    {
      if ((m_lc_info[*cit].ID_of_minhop != ID)&&(*cit != ID))
        {
          carinfo.list_of_dont_forward.push_back (*cit);
        }
    }
  std::cout<<"ID:"<<Ipv4toString (ID)<<",front:"<<Ipv4toString (m_forward_chain.front())<<std::endl;
  if (ID != m_forward_chain.front ())
    {
      for (std::map<Ipv4Address, std::list<Ipv4Address> >::const_iterator cit = m_lc_headNtail.begin ();
           cit != m_lc_headNtail.end (); ++cit)
        {
          for (std::list<Ipv4Address>::const_iterator cit2 = cit->second.begin ();
               cit2 != cit->second.end (); ++cit2)
            {
              carinfo.list_of_dont_forward.push_back (*cit2);
            }
        }
    }
  else
    {
      for (std::map<Ipv4Address, std::list<Ipv4Address> >::const_iterator cit = m_lc_headNtail.begin ();
           cit != m_lc_headNtail.end (); ++cit)
        {
          bool flag = false;
          for (std::list<Ipv4Address>::const_reverse_iterator crit2 = cit->second.rbegin ();
               crit2 != cit->second.rend (); ++crit2)
            {
              if (flag)
                {
                  carinfo.list_of_dont_forward.push_back (*crit2);
                }
              else
                {
                  flag = true;//Skip the r-first (last) one.
                }
            }
        }
    }
}

void
RoutingProtocol::Partition ()
{
  m_Sections.clear ();
  int numArea = GetNumArea();
  for (int i = 0; i < numArea; ++i)
    {
      m_Sections.push_back (std::set<Ipv4Address> ());
    }
  for (std::map<Ipv4Address, CarInfo>::const_iterator cit = m_lc_info.begin ();
       cit != m_lc_info.end(); ++cit)
    {
      m_Sections[GetArea (cit->second.GetPos ())].insert (cit->first);
    }
  //std::cout<<m_lc_info.size ()<<std::endl;
  /*
  for (int i = 0; i < numArea; ++i)
    {
      std::cout<<"Section "<<i<<": ";
      for (std::set<Ipv4Address>::const_iterator cit = m_Sections[i].begin ();
           cit != m_Sections[i].end (); ++cit)
        {
          std::cout<<cit->Get ()%256<<",";
        }
      std::cout<<std::endl;
    }
    */
}

void
RoutingProtocol::SetN_Init ()
{
  int numArea = GetNumArea();
  for (std::set<Ipv4Address>::const_iterator cit = m_Sections[numArea-1].begin ();
       cit != m_Sections[numArea-1].end (); ++cit)
    {
      m_lc_info[(*cit)].minhop = 1;
      m_lc_info[(*cit)].ID_of_minhop = Ipv4Address::GetZero ();
    }
}

void
RoutingProtocol::OtherSet_Init ()
{
  int numArea = GetNumArea();
  //m_lc_info.clear (); WTF?
  for (int area = numArea - 2; area >= 0; --area)
    {
      //Reser m_lc_info->minhop / m_lc_info->ID_of_minhop
      for (std::set<Ipv4Address>::const_iterator cit = m_Sections[area].begin ();
           cit != m_Sections[area].end (); ++cit)
        {
          m_lc_info[(*cit)].minhop = INFHOP;
          m_lc_info[(*cit)].ID_of_minhop = Ipv4Address::GetZero ();
        }
      m_lc_shorthop.clear();
      SortByDistance (area);
      CalcShortHopOfArea (area, area + 1);
      if ((area == numArea - 3) && isPaddingExist ())
        {
          CalcShortHopOfArea (area, area + 2);
        }
      CalcIntraArea (area);
    }
}

void
RoutingProtocol::SelectNode ()
{
  //4-1
  ResetAppointmentResult ();
  uint32_t thezero = 0;
  Ipv4Address The_Car(thezero);
  uint32_t minhop_of_tc = INFHOP;
  double best_pos = m_signal_range;

  //First Area
  for (std::set<Ipv4Address>::const_iterator cit = m_Sections[0].begin ();
      cit != m_Sections[0].end (); ++cit)
    {
      CarInfo& temp_info = m_lc_info[*cit];
      double temp_dist = CalcDist (temp_info.GetPos (), m_lc_start);
      if (temp_info.minhop < minhop_of_tc)
        {
          minhop_of_tc = temp_info.minhop;
          best_pos = temp_dist;
          The_Car = *cit;
        }
      else
        if ((temp_info.minhop == minhop_of_tc)&&(temp_dist < best_pos)&&(minhop_of_tc < INFHOP))
          {
            best_pos = temp_dist;
            The_Car = *cit;
          }
    }
  m_theFirstCar = The_Car;
  Ipv4Address ZERO = Ipv4Address::GetZero ();
  if (The_Car != ZERO)
    {
      m_linkEstablished = true;
    }
  else
    {
      m_linkEstablished = false;
    }
  m_forward_chain.clear ();
  //std::cout<<"Chain ";
  while (The_Car != ZERO)
    {
      m_forward_chain.push_back (The_Car);
      //double oldp = CalcDist (m_lc_info[The_Car].GetPos (), m_lc_start);
      //std::cout<<The_Car.Get () % 256<<"("<<oldp<<","<<m_lc_info[The_Car].minhop<<")";
      m_lc_info[The_Car].appointmentResult = FORWARDER;
      The_Car = m_lc_info[The_Car].ID_of_minhop;
      /*if (The_Car != ZERO)
        {
          std::cout<<"<-"<<CalcDist (m_lc_info[The_Car].GetPos (), m_lc_start) - oldp<<"->";
        }*/
    }
  //std::cout<<std::endl;
  if (m_theFirstCar != Ipv4Address::GetZero ())
    {
      double vx = GetProjection (m_lc_info[m_theFirstCar].Velocity);
      double dx = CalcDist (m_lc_info[m_theFirstCar].GetPos (), m_lc_start);
      double t2l;
      if (vx < 1e-7)
        {
          m_linkEstablished = false;
        }
      else
        {
          t2l= (0.5 * m_signal_range - dx) / vx;
          if (t2l<1)
            m_linkEstablished = false;
        }
    }
}

void
RoutingProtocol::SortByDistance (int area)
{
  //InsertSort
  m_list4sort.clear ();

  std::list<std::pair<double, Ipv4Address> > templist;

  for (std::set<Ipv4Address>::const_iterator cit = m_Sections[area].begin ();
      cit != m_Sections[area].end (); ++cit)
    {

      Vector3D temp3D = m_lc_info[*cit].GetPos ();
      double distance = CalculateDistance (Vector2D(temp3D.x, temp3D.y), m_lc_start);

      templist.push_back (std::pair<double, Ipv4Address>(distance, *cit));
    }

  templist.sort (RoutingProtocol::Comp);

  for (std::list<std::pair<double, Ipv4Address> >::const_iterator cit = templist.begin ();
       cit != templist.end (); ++cit)
    {
      //std::cout<<cit->second.Get () % 256<<":"<<cit->first<<std::endl;
      m_list4sort.push_back (cit->second);
    }
}

void
RoutingProtocol::CalcShortHopOfArea (int fromArea, int toArea)
{
  for (std::list<Ipv4Address>::const_iterator cit = m_list4sort.begin ();
       cit != m_list4sort.end (); ++cit)
    {
      for (std::set<Ipv4Address>::const_iterator cit2 = m_Sections[toArea].begin ();
           cit2 != m_Sections[toArea].end (); ++cit2)
        {
          m_lc_shorthop[*cit].push_back (GetShortHop (*cit,*cit2));
        }

      UpdateMinHop (*cit);
    }
}

void
RoutingProtocol::CalcIntraArea (int area)
{
  CalcShortHopOfArea (area, area);
}

void
RoutingProtocol::UpdateMinHop (const Ipv4Address &ID)
{
  uint32_t theminhop = INFHOP;
  Ipv4Address IDofminhop = Ipv4Address::GetZero ();
  for (std::list<ShortHop>::const_iterator cit = m_lc_shorthop[ID].begin ();
       cit != m_lc_shorthop[ID].end (); ++cit)
    {
      if (cit->hopnumber < theminhop)
        {
          theminhop = cit->hopnumber;
          if (cit->isTransfer)
            {
              IDofminhop = cit->proxyID;
            }
          else
            {
              IDofminhop = cit->nextID;
            }
        }
    }
  if (theminhop < m_lc_info[ID].minhop)
    {
      m_lc_info[ID].ID_of_minhop = IDofminhop;
      m_lc_info[ID].minhop = theminhop;
    }
}

void
RoutingProtocol::ResetAppointmentResult ()
{
  for (std::map<Ipv4Address, CarInfo>::iterator it = m_lc_info.begin ();
       it != m_lc_info.end (); ++it)
    {
      it->second.appointmentResult = NORMAL;
    }
}

void
RoutingProtocol::CalcSetZero ()
{
  m_lc_shorthop.clear();
  SortByDistance (0);
  if (GetNumArea () > 1)
    CalcShortHopOfArea (0,1);
  if ((GetNumArea () == 3)&&(isPaddingExist ()))
    CalcShortHopOfArea (0,2);
  CalcIntraArea (0);
}

void
RoutingProtocol::SelectNewNodeInAreaZero ()
{
  //std::cout<<"BP1"<<std::endl;
  uint32_t thezero = 0;
  Ipv4Address The_Car (thezero);
  uint32_t minhop_of_tc = INFHOP;
  double best_pos = m_signal_range;
  for (std::set<Ipv4Address>::const_iterator cit = m_Sections[0].begin ();
       cit != m_Sections[0].end (); ++cit)
    {
      CarInfo& temp_info = m_lc_info[*cit];
      if (temp_info.minhop < minhop_of_tc)
        {
          minhop_of_tc = temp_info.minhop;
          best_pos = CalcDist (temp_info.GetPos (), m_lc_start);
          The_Car = *cit;
        }
      else
        if (temp_info.minhop == minhop_of_tc)
          {
            double temp_dist = CalcDist (temp_info.GetPos (), m_lc_start);
            if (temp_dist < best_pos)
              {
                best_pos = temp_dist;
                The_Car = *cit;
              }
          }
    }
  //std::cout<<"BP2"<<std::endl;
  if (The_Car != Ipv4Address::GetZero ())//! NO CAR IN AREA 0 HAVE a link to area 1
    {
      if (m_lc_info[The_Car].ID_of_minhop == m_theFirstCar)
        {
          m_theFirstCar = The_Car;
          m_lc_info[The_Car].appointmentResult = FORWARDER;
          //std::cout<<The_Car.Get () % 256<<"m_lc_info[The_Car].ID_of_minhop == m_theFirstCar"<<std::endl;
          m_forward_chain.pop_back ();
          m_forward_chain.push_front (The_Car);
        }
      else
        {
          m_forward_chain.clear ();
          m_linkEstablished = false;
          ResetAppointmentResult ();
          m_theFirstCar = The_Car;
          //std::cout<<GetNumArea();
          //std::cout<<"Chain ";
          //std::cout<<"BP3"<<std::endl;
          while (m_lc_info.find (The_Car) != m_lc_info.end ())
            {
              //std::cout<<Ipv4toString (The_Car)<<std::endl;
              m_forward_chain.push_back (The_Car);
              //double oldp = CalcDist (m_lc_info[The_Car].GetPos (), m_lc_start);
              //std::cout<<The_Car.Get () % 256<<"("<<oldp<<","<<m_lc_info[The_Car].minhop<<")";
              m_lc_info[The_Car].appointmentResult = FORWARDER;
              if (GetArea (m_lc_info[The_Car].GetPos ()) == GetNumArea() - 1)
                {
                  m_linkEstablished = true;
                  break;
                }
              The_Car = m_lc_info[The_Car].ID_of_minhop;
              /*if ((The_Car != Ipv4Address::GetZero ())&&(m_lc_info.find (The_Car) != m_lc_info.end ()))
                {
                  std::cout<<"<-"<<CalcDist (m_lc_info[The_Car].GetPos (), m_lc_start) - oldp<<"->";
                }*/
            }
          //std::cout<<std::endl;
        }
      //std::cout<<"BP4"<<std::endl;
      //Time to short?
      double vx = GetProjection (m_lc_info[m_theFirstCar].Velocity);
      double dx = CalcDist (m_lc_info[m_theFirstCar].GetPos (), m_lc_start);
      double t2l = (0.5 * m_signal_range - dx) / vx;
      if (t2l < 1)
        {
          m_linkEstablished = false;
          //std::cout<<"T2L:"<<t2l<<std::endl;
        }
    }
  else
    {
      m_linkEstablished = false;
    }
}

void
RoutingProtocol::SetControllArea (Vector2D start, Vector2D end)
{
  m_lc_start = start;
  m_lc_end = end;
  m_road_length = CalculateDistance (start, end);
  m_lc_controllArea_vaild = true;
}

bool
RoutingProtocol::ShouldISendHello ()
{
  if (m_appointmentResult == FORWARDER)
    {
      return true;
    }

  if (!m_lc_controllArea_vaild || !m_car_lc_ack_vaild)
    {
      //std::cout<<"notVaild"<<std::endl;
      return true;
    }

  Time now = Simulator::Now ();
  double delta_t = now.GetSeconds () - m_car_lc_ack_time.GetSeconds ();
  Vector3D pridict_pos = Vector3D(m_car_lc_ack_pos.x + m_car_lc_ack_vel.x * delta_t,
                                  m_car_lc_ack_pos.y + m_car_lc_ack_vel.y * delta_t,
                                  m_car_lc_ack_pos.z + m_car_lc_ack_vel.z * delta_t);
  Vector3D now_pos = m_mobility->GetPosition ();
  double distance = CalculateDistance (pridict_pos, now_pos);
  bool ret = false;

  if ((distance > ((1-m_safety_raito)/2) * m_signal_range))
    {
      m_car_lc_ack_vaild = false;
      //std::cout<<"DIS,YES!"<<distance<<" "<<std::endl;
      ret = true;
    }

  if (!IsInMyArea (now_pos))
    {
      if (m_appointmentResult == FORWARDER)
        {
          //std::cout<<"FORWARDER--------------BUSTED!"<<std::endl;
        }
      else
        {
          //std::cout<<"NORMAL BUSTED!"<<std::endl;
        }
      m_lc_controllArea_vaild = false;
      m_appointmentResult = NORMAL;
      ret = true;
    }

  return ret;
}

bool
RoutingProtocol::IsInMyArea (Vector3D pos) const
{
  double t = m_lc_start.y,
         b = m_lc_end.y,
         l = m_lc_start.x,
         r = m_lc_end.x;
  double temp;

  if (t>b)
    {
      temp = t;
      t = b;
      b = temp;
    }
  if (l>r)
    {
      temp = l;
      l = r;
      r = temp;
    }

  if ((l<=pos.x)&&(pos.x<=r)&&(t<=pos.y)&&(pos.y<=b))
    {
      return true;
    }
  /*std::cout<<t<<","<<pos.y<<","<<b<<";"<<l<<","<<pos.x<<","<<r<<std::endl;
  if (m_lc_controllArea_vaild)
    std::t<<"%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"<<std::endl;*/
  return false;
}

bool
RoutingProtocol::Comp (const std::pair<double, Ipv4Address> &p1, const std::pair<double, Ipv4Address> &p2)
{
  return p1.first > p2.first;
}

double
RoutingProtocol::CalcDist (const Vector3D &pos1, const Vector2D &pos2)
{
  return CalculateDistance (Vector2D (pos1.x, pos1.y), pos2);
}

double
RoutingProtocol::GetProjection (const Vector3D &vel) const
{
  double vx = vel.x,
         vy = vel.y;
  double rx = m_lc_end.x - m_lc_start.x,
         ry = m_lc_end.y - m_lc_start.y;
  return (vx*rx + vy*ry)/(sqrt (pow (rx,2.0) + pow (ry, 2.0)));
}

bool
RoutingProtocol::CheckLink ()
{
  bool ret = true;
  std::list<Ipv4Address>::const_iterator fwci = m_forward_chain.begin ();
  fwci++;
  while (fwci != m_forward_chain.end ())
    {
      Ipv4Address pid = *(fwci--);
      Ipv4Address tid = *(fwci++);
      if (CalculateDistance(m_lc_info[pid].Position, m_lc_info[tid].Position) > m_safety_raito * m_signal_range)
        {
          std::cout<<"LINK BREAK"<<std::endl;
          ret = false;
          break;
        }
      fwci++;
    }
  return ret;
}


} // namespace sdn
} // namespace ns3


