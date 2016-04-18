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

#ifndef SDN_IMPL_H
#define SDN_IMPL_H

#include "sdn-header.h"

#include "ns3/object.h"
#include "ns3/packet.h"
#include "ns3/node.h"
#include "ns3/socket.h"
#include "ns3/event-garbage-collector.h"
#include "ns3/random-variable-stream.h"
#include "ns3/timer.h"
#include "ns3/traced-callback.h"
#include "ns3/ipv4.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/mobility-module.h"
//#include "sdn-duplicate-detection.h"

#include <vector>
#include <map>
#include <set>

namespace ns3 {
namespace sdn {

std::string
Ipv4toString (const Ipv4Address& address);

enum NodeType {CAR, LOCAL_CONTROLLER, OTHERS};

/// An SDN's routing table entry.
struct RoutingTableEntry
{
  RoutingTableEntry () : // default values
                           destAddr (uint32_t(0)),
                           nextHop (uint32_t(0)),
                           mask (uint32_t(0)),
                           interface (0) {};

  Ipv4Address destAddr; ///< Address of the destination subnet.
  Ipv4Address nextHop; ///< Address of the next hop.
  Ipv4Address mask; ///< mask of the destination subnet.
  uint32_t interface; ///< Interface index.
};

// A struct for LC to hold Information that got from cars
class CarInfo
{
public:

  CarInfo () :
    Active (false)
  {
    minhop = INFINITY;
    ID_of_minhop = Ipv4Address::GetZero ();
    appointmentResult = AppointmentType::NORMAL;
  };

  //Get position by this time
  Vector3D GetPos () const
  {
    double t = Simulator::Now ().GetSeconds () - LastActive.GetSeconds ();
    double x = this->Position.x + this->Velocity.x * t,
           y = this->Position.y + this->Velocity.y * t,
           z = this->Position.z + this->Velocity.z * t;
    return Vector3D(x, y, z);
  }
  Vector3D Position;
  Vector3D Velocity;
  Time LastActive;//Timeout indicator
  bool Active;
  std::vector<RoutingTableEntry> R_Table;
  uint32_t minhop;
  Ipv4Address ID_of_minhop;
  AppointmentType appointmentResult;
  std::vector<Ipv4Address> list_of_dont_forward;
};

struct ShortHop
{
  ShortHop ()
  {
    hopnumber = INFINITY;
    isTransfer = false;
    nextID = Ipv4Address::GetZero ();
    IDa = Ipv4Address::GetZero ();
    IDb = Ipv4Address::GetZero ();
    proxyID = Ipv4Address::GetZero ();
    t = 0;
  };

  Ipv4Address nextID;
  uint32_t hopnumber;
  bool isTransfer;
  Ipv4Address IDa, IDb, proxyID;
  double t; //in secends
};

class RoutingProtocol;

/// \brief SDN routing protocol for IPv4
///
class RoutingProtocol : public Ipv4RoutingProtocol
{
public:
  static TypeId GetTypeId (void);//implemented

  RoutingProtocol ();//implemented
  virtual ~RoutingProtocol ();//implemented

  ///
  /// \brief Set the SDN main address to the first address on the indicated
  ///        interface
  /// \param interface IPv4 interface index
  ///
  void SetSCHInterface (uint32_t interface);//implemented
  void SetCCHInterface (uint32_t interface);//implemented

  ///
  /// Dump the routing table
  /// to logging output (NS_LOG_DEBUG log level).  If logging is disabled,
  /// this function does nothing.
  ///
  void Dump (void);//implemented

  /**
   * Return the list of routing table entries discovered by SDN
   **/
  std::vector<RoutingTableEntry> GetRoutingTableEntries () const;//implemented

 /**
  * Assign a fixed random variable stream number to the random variables
  * used by this model.  Return the number of streams (possibly zero) that
  * have been assigned.
  *
  * \param stream first stream index to use
  * \return the number of stream indices assigned by this model
  */
  int64_t AssignStreams (int64_t stream);//implemented

private:
  std::set<uint32_t> m_interfaceExclusions;

public:
  std::set<uint32_t> GetInterfaceExclusions () const
  {
    return (m_interfaceExclusions);
  }
  void SetInterfaceExclusions (std::set<uint32_t> exceptions);//implemented

protected:
  virtual void DoInitialize (void);//implemented
private:
  std::map<Ipv4Address, RoutingTableEntry> m_table; ///< Data structure for the routing table. (Use By Mainly by CAR Node, but LC needs it too)

  std::map<Ipv4Address, CarInfo> m_lc_info;///for LC
  std::map<Ipv4Address, std::list<Ipv4Address> > m_lc_headNtail;


  EventGarbageCollector m_events;
	
  /// Packets sequence number counter.
  uint16_t m_packetSequenceNumber;
  /// Messages sequence number counter.
  uint16_t m_messageSequenceNumber;

  /// HELLO messages' emission interval.
  Time m_helloInterval;
  /// Routing messages' emission interval.
  Time m_rmInterval;
  /// minimum ap message emission interval
  Time m_minAPInterval;

  Ptr<Ipv4> m_ipv4;

  void Clear ();//implemented
  uint32_t GetSize () const { return (m_table.size ()); }
  void RemoveEntry (const Ipv4Address &dest);//implemented
  void AddEntry (const Ipv4Address &dest,
                 const Ipv4Address &mask,
                 const Ipv4Address &next,
                 uint32_t interface);//implemented
  void AddEntry (const Ipv4Address &dest,
                 const Ipv4Address &mask,
                 const Ipv4Address &next,
                 const Ipv4Address &interfaceAddress);//implemented
  bool Lookup (const Ipv4Address &dest,
               RoutingTableEntry &outEntry) const;//implemented

  // From Ipv4RoutingProtocol
  virtual Ptr<Ipv4Route> RouteOutput (Ptr<Packet> p,
                                      const Ipv4Header &header,
                                      Ptr<NetDevice> oif,
                                      Socket::SocketErrno &sockerr);//implemented
  virtual bool RouteInput (Ptr<const Packet> p,
                           const Ipv4Header &header,
                           Ptr<const NetDevice> idev,
                           UnicastForwardCallback ucb,
                           MulticastForwardCallback mcb,
                           LocalDeliverCallback lcb,
                           ErrorCallback ecb);//implemented
  virtual void NotifyInterfaceUp (uint32_t interface);//implemented
  virtual void NotifyInterfaceDown (uint32_t interface);//implemented
  virtual void NotifyAddAddress (uint32_t interface, Ipv4InterfaceAddress address);//implemented
  virtual void NotifyRemoveAddress (uint32_t interface, Ipv4InterfaceAddress address);//implemented
  virtual void SetIpv4 (Ptr<Ipv4> ipv4);//implemented
  virtual void PrintRoutingTable (Ptr<OutputStreamWrapper> stream) const;//implemented

  void DoDispose ();//implemented

  void SendPacket (Ptr<Packet> packet, const MessageList &containedMessages);//implemented

  /// Increments packet sequence number and returns the new value.
  inline uint16_t GetPacketSequenceNumber ();//implemented
  /// Increments message sequence number and returns the new value.
  inline uint16_t GetMessageSequenceNumber ();//implemented

  void RecvSDN (Ptr<Socket> socket);//implemented

  //Ipv4Address GetMainAddress (Ipv4Address iface_addr) const;

  // Timer handlers
  Timer m_helloTimer;
  void HelloTimerExpire ();//implemented

  Timer m_rmTimer;
  void RmTimerExpire ();//implemented

  Timer m_apTimer;
  void APTimerExpire ();

  /// A list of pending messages which are buffered awaiting for being sent.
  sdn::MessageList m_queuedMessages;
  Timer m_queuedMessagesTimer; // timer for throttling outgoing messages

  void QueueMessage (const sdn::MessageHeader &message, Time delay);//implemented
  void SendQueuedMessages ();//implemented
  void SendHello ();//implemented
  void SendRoutingMessage (); //Fullfilled
  void SendAppointment ();
  void SendAckHello (const Ipv4Address& ID);
  void SendDontForward (const Ipv4Address& ID);
  void SendLc2Lc ();

  void ProcessLc2Lc (const sdn::MessageHeader &msg, const Ipv4Address& sour);
  void ProcessDontForward (const sdn::MessageHeader &msg);
  void ProcessAckHello (const sdn::MessageHeader &msg);
  void ProcessAppointment (const sdn::MessageHeader &msg);
  void ProcessRm (const sdn::MessageHeader &msg);//implemented
  void ProcessHM (const sdn::MessageHeader &msg); //implemented

  void ComputeRoute ();//

  /// Check that address is one of my interfaces
  bool IsMyOwnAddress (const Ipv4Address & a) const;//implemented

private:
  Ipv4Address m_mainAddress;
  uint32_t m_SCHinterface;
  uint32_t m_CCHinterface;
  // One socket per interface, each bound to that interface's address
  // (reason: for VANET-SDN we need to distinguish CCH and SCH interfaces)
  std::map< Ptr<Socket>, Ipv4InterfaceAddress > m_socketAddresses;

  TracedCallback <const PacketHeader &,
                  const MessageList &> m_rxPacketTrace;
  TracedCallback <const PacketHeader &,
                  const MessageList &> m_txPacketTrace;
  TracedCallback <uint32_t> m_routingTableChanged;

  /// Provides uniform random variables.
  Ptr<UniformRandomVariable> m_uniformRandomVariable;  

  // Mobility module for Vanet
  Ptr<MobilityModel> m_mobility;

public:
  void SetMobility (Ptr<MobilityModel> mobility);//implemented

private:
  NodeType m_nodetype;
  //Only node type CAR use this(below)
  AppointmentType m_appointmentResult;
  std::set<Ipv4Address> m_dont_forward;
  bool IsInDontForward (const Ipv4Address& id) const;

  std::list<Ipv4Address> m_forward_chain;


public:
  void SetType (NodeType nt); //implemented
  NodeType GetType () const; //implemented

private:
  bool m_linkEstablished;
  std::vector< std::set<Ipv4Address> > m_Sections;
  ShortHop GetShortHop (const Ipv4Address& IDa, const Ipv4Address& IDb);
  void LCAddEntry( const Ipv4Address& ID,
                   const Ipv4Address& dest,
                   const Ipv4Address& mask,
                   const Ipv4Address& next);
  void ClearAllTables ();

  int GetArea (Vector3D position) const;
  int GetNumArea () const;
  void Init_NumArea();
  int m_numArea;
  bool m_isPadding;
  bool m_numAreaVaild;

  bool isPaddingExist () const;

  void RemoveTimeOut ();

  double m_road_length;
  double m_signal_range;
  double m_safety_raito;

public:
  void SetSignalRange (double signal_range);

private:
  void BinarySearch ();
  bool TestResult (double result);
  void SortByDistance ();
  double FindUpperBound ();
  void BSReschedule ();

  void Do_Init_Compute ();
  void Do_Update ();
  void Reschedule ();
  void CalcDontForward (const Ipv4Address& ID);

  void Partition ();
  void SetN_Init ();
  void OtherSet_Init ();
  void SelectNode ();

  void SortByDistance (int area);
  void CalcShortHopOfArea (int fromArea, int toArea);
  void CalcIntraArea (int area);
  void UpdateMinHop (const Ipv4Address &ID);
  //ResetAppointmentResult In m_lc_info;
  void ResetAppointmentResult ();
  std::list<Ipv4Address> m_list4sort;
  std::map<Ipv4Address, std::list<ShortHop> > m_lc_shorthop;

  void CalcSetZero ();
  void SelectNewNodeInAreaZero ();

  Ipv4Address m_theFirstCar;//Use by Reschedule (), SelectNewNodeInAreaZero(); Assign by SelectNode ();

  Vector2D m_lc_start, m_lc_end;//both Car and LC will use this.
  bool m_lc_controllArea_vaild;
  uint32_t m_norespond_hm;
public:
  void SetControllArea (Vector2D start, Vector2D end);

private:

  Vector3D m_car_lc_ack_pos, m_car_lc_ack_vel;
  Time m_car_lc_ack_time;
  bool m_car_lc_ack_vaild;

  bool ShouldISendHello();

  bool IsInMyArea (Vector3D pos) const;
  static bool Comp (const std::pair<double, Ipv4Address> &p1, const std::pair<double, Ipv4Address> &p2);
  static double CalcDist (const Vector3D &pos1, const Vector2D &pos2);
  double GetProjection (const Vector3D &vel) const;
  std::vector<std::pair<Ipv4Address, double> > m_bs_sort;
  double m_lowerbound;
};


}
}  // namespace ns3

#endif /* SDN_IMPL_H */
