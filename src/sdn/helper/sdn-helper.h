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
 * Author: Haoliang Chen <chl41993@gmail.com>
 */
#ifndef SDN_HELPER_H
#define SDN_HELPER_H

#include "ns3/object-factory.h"
#include "ns3/node.h"
#include "ns3/node-container.h"
#include "ns3/ipv4-routing-helper.h"
#include "ns3/mobility-module.h"
#include "ns3/sdn-routing-protocol.h"
#include <map>
#include <set>

namespace ns3 {

/**
 * \brief Helper class that adds SDN routing to nodes.
 *
 * This class is expected to be used in conjunction with 
 * ns3::InternetStackHelper::SetRoutingHelper
 */
class SdnHelper : public Ipv4RoutingHelper
{
public:
  /**
   * Create an SdnHelper that makes life easier for people who want to install
   * SDN routing to nodes.
   */
  SdnHelper ();

  /**
   * \brief Construct an SdnHelper from another previously initialized instance
   * (Copy Constructor).
   */
  SdnHelper (const SdnHelper &);

  /**
   * \internal
   * \returns pointer to clone of this SdnHelper
   * 
   * This method is mainly for internal use by the other helpers;
   * clients are expected to free the dynamic memory allocated by this method
   */
  SdnHelper* Copy (void) const;


  /**
   * \param node the node on which the routing protocol will run
   * \returns a newly-created routing protocol
   *
   * This method will be called by ns3::InternetStackHelper::Install
   */
  virtual Ptr<Ipv4RoutingProtocol> Create (Ptr<Node> node) const;

  /**
   * \param name the name of the attribute to set
   * \param value the value of the attribute to set.
   *
   * This method controls the attributes of ns3::olsr::RoutingProtocol
   */
  void Set (std::string name, const AttributeValue &value);

 /**
  * Assign a fixed random variable stream number to the random variables
  * used by this model.  Return the number of streams (possibly zero) that
  * have been assigned.  The Install() method of the InternetStackHelper
  * should have previously been called by the user.
  *
  * \param stream first stream index to use
  * \param c NodeContainer of the set of nodes for which the SdnRoutingProtocol
  *          should be modified to use a fixed stream
  * \return the number of stream indices assigned by this helper
  */
  int64_t AssignStreams (NodeContainer c, int64_t stream);


  /**
   * Set Node type to nodes
   */
  void SetNodeTypeMap (Ptr<Node> node, sdn::NodeType nt);

  /*
   * Set Road length and signal range
   */
  void SetSR(double signal_range);

private:
  /**
   * \internal
   * \brief Assignment operator declared private and not implemented to disallow
   * assignment and prevent the compiler from happily inserting its own.
   */
  SdnHelper &operator = (const SdnHelper &o);
  ObjectFactory m_agentFactory;
  std::map< Ptr<Node>, sdn::NodeType > m_ntmap;
  double m_sr;
};

} // namespace ns3

#endif /* SDN_HELPER_H */
