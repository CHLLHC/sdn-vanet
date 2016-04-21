/*
 * SDN.h
 *
 *  Created on: Oct 13, 2015
 *      Author: chl
 */

#ifndef SDN_H
#define SDN_H

#include <string>
#include <fstream>
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/ns2-mobility-helper.h"
#include "ns3/network-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/olsr-helper.h"
#include "ns3/aodv-helper.h"
#include "ns3/dsdv-module.h"
#include "ns3/dsr-module.h"

//FINALLY!
#include "ns3/sdn-helper.h"

#include "ns3/vanetmobility-helper.h"

#include <unordered_set>
#include <unordered_map>
#include <vector>

using namespace ns3;

class VanetSim
{
public:
	VanetSim();
	~VanetSim();
	void Simulate(int argc, char *argv[], std::string todo);
protected:
	void SetDefault();
	void ParseArguments(int argc, char *argv[]);
	void LoadTraffic();
	void ConfigNode();
	void ConfigChannels();
	void ConfigDevices();
	void ConfigMobility();
	void ConfigApp();
	void ConfigTracing();
	void Run();
	void ProcessOutputs();
	bool CheckActive(Node node);
	void Look_at_clock();
private:
	Ptr<Socket> source;
	std::string traceFile;
	std::string logFile;
	std::string phyMode;
	std::string lossModle;
	std::string homepath;
	std::string folder;
	std::ofstream os;
	std::ofstream dos;//For Delay File
	double freq1;//SCH
	double freq2;//CCH
	double txp1;//SCH
	double txp2;//CCH
	double range1;//SCH
	double range2;//CCH
	uint32_t packetSize; // bytes
	uint32_t numPackets;
	double interval; // seconds
	bool verbose;
	int mod;//1=SDN Other=OLSR
	uint32_t nodeNum;
	double duration;
	YansWifiPhyHelper m_SCHPhy, m_CCHPhy;
	NodeContainer m_nodes;//Cars + Controller + Source + Sink
	NetDeviceContainer m_SCHDevices, m_CCHDevices;
	Ipv4InterfaceContainer m_SCHInterfaces, m_CCHInterfaces;
	//////////TongJi////////////
	uint32_t Rx_Routing_Bytes, Tx_Routing_Bytes;
	uint32_t RX_Routing_Pkts, TX_Routing_Pkts;
	uint32_t Rx_Data_Bytes, Tx_Data_Bytes;
	uint32_t Rx_Data_Pkts, Tx_Data_Pkts;
  uint32_t old_Rx_Data_Pkts, old_Tx_Data_Pkts;
	uint32_t Unique_RX_Pkts, old_Unique_RX_Pkts;

  uint32_t Rx_Data_Pkts2;
  uint32_t old_Rx_Data_Pkts2;
  uint32_t Unique_RX_Pkts2, old_Unique_RX_Pkts2;

  uint32_t Rx_Data_Pkts3;
  uint32_t old_Rx_Data_Pkts3;
  uint32_t Unique_RX_Pkts3, old_Unique_RX_Pkts3;

	uint32_t m_port;
	ApplicationContainer m_source, m_sink, m_cars, m_controller;
	Ptr<ns3::vanetmobility::VANETmobility> VMo;
	void ReceiveDataPacket (Ptr<Socket> socket);
	void ReceiveDataPacket2 (Ptr<Socket> socket);
	void ReceiveDataPacket3 (Ptr<Socket> socket);
	void SendDataPacket ();
	void TXTrace (Ptr<const Packet> newpacket);
	std::unordered_set<uint64_t> dup_det;
	std::unordered_set<uint64_t> dup_det2;
	std::unordered_set<uint64_t> dup_det3;

	std::unordered_map<uint64_t, Time> delay;
	std::vector<int64_t> delay_vector;
	std::vector<int64_t> per_sec_delay_vector;
  std::vector<int64_t> delay_vector2;
  std::vector<int64_t> per_sec_delay_vector2;
  std::vector<int64_t> delay_vector3;
  std::vector<int64_t> per_sec_delay_vector3;
  std::string m_todo;
  std::string m_ds;//DataSet
};




#endif /* SDN_H */
