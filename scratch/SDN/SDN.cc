/*
 * SDN.cc
 *
 *  Created on: Oct 9, 2015
 *      Author: chl
 */
/*
  ./waf --run "SDN"
*/
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <dirent.h>//DIR*
#include "SDN.h"
#include <string> //config connect

NS_LOG_COMPONENT_DEFINE ("SDN");


using namespace ns3;

VanetSim::VanetSim()
{
	traceFile = "";
	logFile = "SDN.log";
	phyMode = "OfdmRate27MbpsBW10MHz";
	lossModle = "ns3::FriisPropagationLossModel";
	freq1 = 5.860e9;  //802.11p SCH CH172
	freq2 = 5.890e9;  //802.11p CCH CH178
	txp1 = 23;  // dBm SCH
	txp2 = 31;  // CCH
	range1 = 400.0;//SCH
	range2 = 1000.0;//CCH
	packetSize = 1000; // bytes
	numPackets = 1;
	interval = 0.1; // seconds
	verbose = false;
	mod = 1;
	duration = -1;
	nodeNum = 0;
	Rx_Data_Bytes = 0;
	Rx_Data_Pkts = 0;
	Rx_Data_Pkts2 = 0;
	Rx_Data_Pkts3 = 0;
	Rx_Routing_Bytes = 0;
	RX_Routing_Pkts = 0;
	Tx_Data_Bytes = 0;
	Tx_Data_Pkts = 0;
	Tx_Routing_Bytes = 0;
	TX_Routing_Pkts = 0;
	Unique_RX_Pkts = 0;
	Unique_RX_Pkts2 = 0;
	Unique_RX_Pkts3 = 0;
	m_port = 65419;
	homepath = ".";//getenv("HOME");
	folder="SDNData";
	old_Rx_Data_Pkts = 0;
	old_Unique_RX_Pkts = 0;
  old_Rx_Data_Pkts2 = 0;
  old_Unique_RX_Pkts2 = 0;
  old_Rx_Data_Pkts3 = 0;
  old_Unique_RX_Pkts3 = 0;
	old_Tx_Data_Pkts = 0;
}

VanetSim::~VanetSim()
{
	os.close();
	dos.close();
}

void VanetSim::Simulate(int argc, char *argv[])
{
  std::ios_base::sync_with_stdio (false);
	SetDefault();
	ParseArguments(argc, argv);
	LoadTraffic();
	ConfigNode();
	ConfigChannels();
	ConfigDevices();
	ConfigMobility();
	ConfigApp();
	ConfigTracing();
	Run();
	ProcessOutputs();
	std::cout<<std::endl;
}

void VanetSim::SetDefault()
{
	//Handle By Constructor
}

void VanetSim::ParseArguments(int argc, char *argv[])
{
	CommandLine cmd;
//	cmd.AddValue ("traceFile", "Ns2 movement trace file", traceFile);
//	cmd.AddValue ("nodeNum", "Number of nodes", nodeNum);
	cmd.AddValue ("duration", "Duration of Simulation", duration);
//	cmd.AddValue ("logFile", "Log file", logFile);
	cmd.AddValue ("folder", "Working Directory", folder);
	cmd.AddValue ("txp1", "TX power for SCH", txp1);
	cmd.AddValue ("txp2", "TX power for CCH", txp2);
	cmd.AddValue ("range1", "Range for SCH", range1);
	cmd.AddValue ("range2", "Range for CCH", range2);
	cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);
	cmd.AddValue ("mod", "0=olsr 1=bs-sdn(DEFAULT) 2=aodv 3=dsdv 4=dsr 5=yangs-sdn", mod);
	cmd.AddValue ("ds", "DataSet", m_ds);
	cmd.Parse (argc,argv);

	// Fix non-unicast data rate to be the same as that of unicast
	Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",
	                      StringValue (phyMode));


	switch (mod)
	{
	  case 2:
      m_todo = "AODV";
      break;
    case 3:
      m_todo = "DSDV";
      break;
    case 4:
      m_todo = "DSR";
      break;
    case 0:
      m_todo = "OLSR";
      break;
    case 5:
      m_todo = "YANGS-SDN";
      break;
    default:
      m_todo = "BS-SDN";
      mod = 1;
      break;
	}

}

void VanetSim::LoadTraffic()
{
	DIR* dir = NULL;
	//DIR* subdir=NULL;
	std::string temp(homepath+"/"+folder);
	if((dir = opendir(temp.data()))==NULL)
		NS_FATAL_ERROR("Cannot open input path "<<temp.data()<<", Aborted.");



	std::string sumo_net = temp + "/input.net.xml";
	std::string sumo_route = temp + "/input.rou.xml";
	std::string sumo_fcd = temp + "/input.fcd.xml";

	std::string output = temp + "/" + m_todo + "_" + m_ds + "_result_new.txt";
	std::string delay_output = temp + "/" + m_todo + "_" + m_ds +"_delay.txt";


	os.open(output.data(),std::ios::out);
	dos.open(delay_output.data(),std::ios::out);

	ns3::vanetmobility::VANETmobilityHelper mobilityHelper;
	VMo=mobilityHelper.GetSumoMObility(sumo_net,sumo_route,sumo_fcd);

	nodeNum = VMo->GetNodeSize();

  std::cout<<"Mode: "<<m_todo<<"DataSet:  "<<m_ds<<std::endl;
  os<<"Mode:  "<<m_todo<<"DataSet:  "<<m_ds<<std::endl;
  dos<<"Mode: "<<m_todo<<"  DataSet:  "<<m_ds<<std::endl;
}



void VanetSim::ConfigNode()
{
	m_nodes.Create(nodeNum+7);//Cars + Controller + Source + Sink + LC2 + LC3 + Sink2 + Sink3
	/*Only Apps Are Different Between Different kind of Nodes*/
	// Name nodes
	for (uint32_t i = 0; i < nodeNum; ++i)
	{
		std::ostringstream os;
		os << "vehicle-" << i;
		Names::Add(os.str(), m_nodes.Get(i));
	}
	Names::Add("Controller",m_nodes.Get(nodeNum));
	Names::Add("Source",m_nodes.Get(nodeNum+1));
	Names::Add("Sink",m_nodes.Get(nodeNum+2));
	Names::Add("LC2",m_nodes.Get(nodeNum+3));
	Names::Add("LC3",m_nodes.Get(nodeNum+4));
	Names::Add("Sink2",m_nodes.Get(nodeNum+5));
	Names::Add("Sink3",m_nodes.Get(nodeNum+6));
}

void VanetSim::ConfigChannels()
{
	//===channel
	YansWifiChannelHelper SCHChannel;
	SCHChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
	SCHChannel.AddPropagationLoss(lossModle,"Frequency", DoubleValue(freq1));
	YansWifiChannelHelper CCHChannel;
	CCHChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
	CCHChannel.AddPropagationLoss(lossModle,"Frequency", DoubleValue(freq2));


	// the channelg
	Ptr<YansWifiChannel> SCH = SCHChannel.Create();
	Ptr<YansWifiChannel> CCH = CCHChannel.Create();

	//===wifiphy
	m_SCHPhy =  YansWifiPhyHelper::Default ();
	m_SCHPhy.SetChannel (SCH);
	m_SCHPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11_RADIO);
	m_CCHPhy =  YansWifiPhyHelper::Default ();
	m_CCHPhy.SetChannel (CCH);
	m_CCHPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11_RADIO);

	// 802.11p mac
	NqosWaveMacHelper SCH80211pMac = NqosWaveMacHelper::Default ();
	Wifi80211pHelper SCH80211p = Wifi80211pHelper::Default ();
	NqosWaveMacHelper CCH80211pMac = NqosWaveMacHelper::Default ();
	Wifi80211pHelper CCH80211p = Wifi80211pHelper::Default ();

	SCH80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
										"DataMode",StringValue (phyMode),
										"ControlMode",StringValue (phyMode));
	CCH80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
											"DataMode",StringValue (phyMode),
											"ControlMode",StringValue (phyMode));

	// Set Tx Power For The SCH
	m_SCHPhy.Set ("TxPowerStart",DoubleValue (txp1));
	m_SCHPhy.Set ("TxPowerEnd", DoubleValue (txp1));
	m_SCHDevices = SCH80211p.Install(m_SCHPhy, SCH80211pMac, m_nodes);

	// CCH
	m_CCHPhy.Set ("TxPowerStart",DoubleValue (txp2));
	m_CCHPhy.Set ("TxPowerEnd", DoubleValue (txp2));
	m_CCHDevices = CCH80211p.Install(m_CCHPhy, CCH80211pMac, m_nodes);

}

void VanetSim::ConfigDevices()
{
	//Done in ConfigChannels()
}

void VanetSim::ConfigMobility()
{
/*	Ns2MobilityHelper ns2 = Ns2MobilityHelper (traceFile);
	ns2.Install (m_nodes.Begin(),m_nodes.End()-3);
	// configure movements for Car node, while reading trace file
	Ptr<MobilityModel> Temp = m_nodes.Get(nodeNum);//Controller
	Temp->SetPosition(Vector(0.0, 0.0, 0.0));
	Temp = m_nodes.Get(nodeNum+1);//source
	Temp->SetPosition(Vector(5.1, 0.0, 0.0));
	Temp = m_nodes.Get(nodeNum+2);//Sink
*/
	VMo->Install();
	double rt = VMo->GetReadTotalTime();
	if (duration<0)
	{
		duration = rt;
	}
	Time temp_now = Simulator::Now();
	std::cout<<"Now?"<<temp_now.GetSeconds ()<<std::endl;
	Ptr<MobilityModel> Temp = m_nodes.Get(nodeNum)->GetObject<MobilityModel>();//Controller
	Temp->SetPosition(Vector(500.0, 0.0, 0.0));
	Temp = m_nodes.Get(nodeNum+1)->GetObject<MobilityModel>();//source
	Temp->SetPosition(Vector(5.1, 0.0, 0.0));
	Temp = m_nodes.Get(nodeNum+2)->GetObject<MobilityModel>();//Sink
	Temp->SetPosition(Vector(1000.0, 0.0, 0.0));
	Temp = m_nodes.Get(nodeNum+3)->GetObject<MobilityModel>();//LC2
	Temp->SetPosition(Vector(1000.0, 500.0, 0.0));
  Temp = m_nodes.Get(nodeNum+4)->GetObject<MobilityModel>();//LC3
  Temp->SetPosition(Vector(1500.0, 1000.0, 0.0));

  Temp = m_nodes.Get(nodeNum+5)->GetObject<MobilityModel>();//Sink2
  Temp->SetPosition(Vector(1000.0, 1000.0, 0.0));
  Temp = m_nodes.Get(nodeNum+6)->GetObject<MobilityModel>();//Sink3
  Temp->SetPosition(Vector(2000.0, 1000.0, 0.0));

}

void VanetSim::ConfigApp()
{
	//===Routing
	InternetStackHelper internet;
	OlsrHelper olsr;
	SdnHelper sdn;
	AodvHelper aodv;
	DsdvHelper dsdv;
	DsrHelper dsr;
	DsrMainHelper dsrMain;
	switch (mod)
	  {
      case 0:
        internet.SetRoutingHelper(olsr);
        internet.Install (m_nodes);
        std::cout<<"OLSR"<<std::endl;
        os<<"OLSR"<<std::endl;
        break;
      case 2:
        internet.SetRoutingHelper(aodv);
        internet.Install (m_nodes);
        std::cout<<"AODV"<<std::endl;
        os<<"AODV"<<std::endl;
        break;
      case 3:
        internet.SetRoutingHelper(dsdv);
        internet.Install (m_nodes);
        std::cout<<"DSDV"<<std::endl;
        os<<"DSDV"<<std::endl;
        break;
      case 4:
        internet.Install (m_nodes);
        dsrMain.Install (dsr, m_nodes);
        std::cout<<"DSR"<<std::endl;
        os<<"DSR"<<std::endl;
        break;
      default:
        for (uint32_t i = 0; i<nodeNum; ++i)
          {
            sdn.SetNodeTypeMap (m_nodes.Get (i), sdn::CAR);
          }
        sdn.SetNodeTypeMap (m_nodes.Get (nodeNum), sdn::LOCAL_CONTROLLER);
        sdn.SetNodeTypeMap (m_nodes.Get (nodeNum+1), sdn::OTHERS);//Source
        sdn.SetNodeTypeMap (m_nodes.Get (nodeNum+2), sdn::OTHERS);//Sink

        sdn.SetNodeTypeMap (m_nodes.Get (nodeNum+3), sdn::OTHERS);//LC2
        sdn.SetNodeTypeMap (m_nodes.Get (nodeNum+4), sdn::OTHERS);//LC3
        sdn.SetNodeTypeMap (m_nodes.Get (nodeNum+5), sdn::OTHERS);//Sink2
        sdn.SetNodeTypeMap (m_nodes.Get (nodeNum+6), sdn::OTHERS);//Sink3
        sdn.SetSR (range1);
        internet.SetRoutingHelper(sdn);
        internet.Install (m_nodes);
        std::cout<<"SetRoutingHelper Done"<<std::endl;
	  }


	std::cout<<"internet.Install Done"<<std::endl;

	//===IP ADDRESS
	Ipv4AddressHelper ipv4S;
	NS_LOG_INFO ("Assign IP Addresses.");
	ipv4S.SetBase ("10.1.0.0", "255.255.0.0");//SCH
	m_SCHInterfaces = ipv4S.Assign (m_SCHDevices);
	std::cout<<"IPV4S Assigned"<<std::endl;

	Ipv4AddressHelper ipv4C;
	if ((mod ==1)||(mod ==5))
	{
		NS_LOG_INFO ("Assign IP-C Addresses.");
		ipv4C.SetBase("192.168.0.0","255.255.0.0");//CCH
		m_CCHInterfaces = ipv4C.Assign(m_CCHDevices);
		std::cout<<"IPV4C Assigned"<<std::endl;
		for (uint32_t i = 0;i<m_nodes.GetN ();++i)
		  {
		    //std::cout<<"m_nodes.GetN () "<<i<<std::endl;
		    Ptr<sdn::RoutingProtocol> routing =
		        m_nodes.Get (i)->GetObject<sdn::RoutingProtocol> ();
        routing->SetCCHInterface (m_CCHInterfaces.Get (i).second);
		    routing->SetSCHInterface (m_SCHInterfaces.Get (i).second);
		  }

		sdn::Algo ToBeSet = sdn::Binary_Search;
		if (mod == 5)
		  {
		    ToBeSet = sdn::Yangs_Algo;
		  }
		Ptr<sdn::RoutingProtocol> routing =
		            m_nodes.Get (nodeNum)->GetObject<sdn::RoutingProtocol> ();
		routing->SetControllArea (Vector2D (0,0), Vector2D (1000,-10));
		routing->SetAlgo (ToBeSet);
		routing = m_nodes.Get (nodeNum+3)->GetObject<sdn::RoutingProtocol> ();
		routing->SetControllArea (Vector2D (1000,0), Vector2D (1010,1000));
		routing->SetAlgo (ToBeSet);
		routing = m_nodes.Get (nodeNum+4)->GetObject<sdn::RoutingProtocol> ();
		routing->SetControllArea (Vector2D (1010,1000), Vector2D (2000,990));
		routing->SetAlgo (ToBeSet);
	}


	//===Traffic
	//source

	//onoff
	Address remote;
	if ((mod == 1)||(mod==5))
	  {
	    std::pair<Ptr<Ipv4>, uint32_t> RetValue = m_SCHInterfaces.Get (nodeNum+1);
	    Ipv4InterfaceAddress theinterface = RetValue.first->GetAddress (RetValue.second, 0);
	    Ipv4Address bcast = theinterface.GetLocal ().GetSubnetDirectedBroadcast (theinterface.GetMask ());
	    remote = InetSocketAddress(bcast, m_port);
	  }
	else
	  {
	    remote = InetSocketAddress(m_SCHInterfaces.GetAddress (nodeNum+6), m_port);
	  }



	OnOffHelper Source("ns3::UdpSocketFactory",remote);//SendToSink
	Source.SetAttribute("OffTime",StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));
	//Source.SetAttribute("PacketSize", UintegerValue (packetSize));

	m_source = Source.Install(m_nodes.Get(nodeNum+1));//Insatll on Source
	m_source.Stop(Seconds(duration));//Default Start time is 0.

	std::string temp = "/NodeList/"+std::to_string (nodeNum+1)+"/ApplicationList/0/$ns3::OnOffApplication/Tx";
	Config::ConnectWithoutContext (
	    temp,
	    MakeCallback(&VanetSim::TXTrace, this));
	/*
	TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
	source = Socket::CreateSocket (m_nodes.Get(nodeNum+1), tid);
	Simulator::Schedule(Seconds(0.0), &VanetSim::SendDataPacket, this);
	*/

	//sink
	TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
	Ptr<Socket> sink = Socket::CreateSocket (m_nodes.Get(nodeNum+2), tid);//The Sink
  //HearALL;
	InetSocketAddress local = InetSocketAddress(Ipv4Address::GetZero (),m_port);
	sink->Bind(local);
	sink->SetRecvCallback(MakeCallback(&VanetSim::ReceiveDataPacket, this));

  //sink2
  TypeId tid2 = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> sink2 = Socket::CreateSocket (m_nodes.Get(nodeNum+5), tid2);//The Sink
  //HearALL;
  InetSocketAddress local2 = InetSocketAddress(Ipv4Address::GetZero (),m_port);
  sink2->Bind(local2);
  sink2->SetRecvCallback(MakeCallback(&VanetSim::ReceiveDataPacket2, this));

  //sink3
  TypeId tid3 = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> sink3 = Socket::CreateSocket (m_nodes.Get(nodeNum+6), tid3);//The Sink
  //HearALL;
  InetSocketAddress local3 = InetSocketAddress(Ipv4Address::GetZero (),m_port);
  sink3->Bind(local3);
  sink3->SetRecvCallback(MakeCallback(&VanetSim::ReceiveDataPacket3, this));


	//Trace
	m_SCHPhy.EnablePcap ("pcap/sdn-vanet", m_SCHDevices);
}

void VanetSim::ReceiveDataPacket(Ptr<Socket> socket)
{
	Ptr<Packet> packet;
	while ((packet = socket->Recv()))
	  {
	    uint64_t uid = packet->GetUid ();
	    if (dup_det.find (uid) == dup_det.end ())
	      {
	        Unique_RX_Pkts++;
	        dup_det.insert (uid);

	        Time now = Simulator::Now ();
	        int64_t temp = now.GetMicroSeconds () - delay[uid].GetMicroSeconds ();
	        delay_vector.push_back (temp);
	        per_sec_delay_vector.push_back (temp);
	      }
      //Rx_Data_Bytes += packet->GetSize();
      Rx_Data_Pkts++;
      //std::cout<<"."<<std::flush;
	  }
}

void VanetSim::ReceiveDataPacket2(Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  while ((packet = socket->Recv()))
    {
      uint64_t uid = packet->GetUid ();
      if (dup_det2.find (uid) == dup_det2.end ())
        {
          Unique_RX_Pkts2++;
          dup_det2.insert (uid);

          Time now = Simulator::Now ();
          int64_t temp = now.GetMicroSeconds () - delay[uid].GetMicroSeconds ();
          delay_vector2.push_back (temp);
          per_sec_delay_vector2.push_back (temp);
        }
      //Rx_Data_Bytes += packet->GetSize();
      Rx_Data_Pkts2++;
      //std::cout<<":"<<std::flush;
    }
}

void VanetSim::ReceiveDataPacket3(Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  while ((packet = socket->Recv()))
    {
      uint64_t uid = packet->GetUid ();
      if (dup_det3.find (uid) == dup_det3.end ())
        {
          Unique_RX_Pkts3++;
          dup_det3.insert (uid);

          Time now = Simulator::Now ();
          int64_t temp = now.GetMicroSeconds () - delay[uid].GetMicroSeconds ();
          delay_vector3.push_back (temp);
          dos<<temp<<std::endl;
          per_sec_delay_vector3.push_back (temp);
        }
      //Rx_Data_Bytes += packet->GetSize();
      Rx_Data_Pkts3++;
      //std::cout<<"/"<<std::flush;
    }
}

void VanetSim::SendDataPacket()
{
	/*Ptr<Packet> packet = Create<Packet> (packetSize);
	source->SendTo(packet, 0, )
	Simulator::Schedule(Seconds(interval), &VanetSim::SendDataPacket, this);*/
	//TODO
}

void VanetSim::ConfigTracing()
{
	//TODO
}

void VanetSim::ProcessOutputs()
{
	std::cout<<"Tx_Data_Pkts:   "<<Tx_Data_Pkts<<std::endl;
	std::cout<<"Rx_Data_Pkts3:   "<<Rx_Data_Pkts3<<std::endl;
	std::cout<<"Unique_RX_Pkts3: "<<Unique_RX_Pkts3<<std::endl;

	os<<"Result:"<<std::endl;
  os<<"Tx_Data_Pkts:   "<<Tx_Data_Pkts<<std::endl;
  os<<"Rx_Data_Pkts3:   "<<Rx_Data_Pkts3<<std::endl;
  os<<"Unique_RX_Pkts3: "<<Unique_RX_Pkts3<<std::endl;


	if (!delay_vector3.empty ())
	  {
      int64_t best = delay_vector3[0],
              worst = delay_vector3[0];
      double avg = 0;
      for (std::vector<int64_t>::const_iterator cit = delay_vector3.begin ();
           cit != delay_vector3.end ();++cit)
        {
          if (*cit<best)
            {
              best = *cit;
            }

          if (*cit>worst)
            {
              worst = *cit;
            }
          avg += *cit;
        }

      avg /= delay_vector3.size();
      std::cout<<"Best delay:   "<<best<<"us"<<std::endl;
      std::cout<<"Worst delay:   "<<worst<<"us"<<std::endl;
      std::cout<<"Avg delay: "<<avg<<"us"<<std::endl;
      os<<"Best delay:   "<<best<<"us"<<std::endl;
      os<<"Worst delay:   "<<worst<<"us"<<std::endl;
      os<<"Avg delay: "<<avg<<"us"<<std::endl;
	  }
	else
	  {
	    std::cout<<"NO PACKETS WERE RECEIVED."<<std::endl;
	    os<<"NO PACKETS WERE RECEIVED."<<std::endl;
	    dos<<"NO PACKETS WERE RECEIVED."<<std::endl;
	  }
}

void VanetSim::Run()
{
	Simulator::Schedule(Seconds(0.0), &VanetSim::Look_at_clock, this);
	std::cout << "Starting simulation for " << duration << " s ..."<< std::endl;
	os << "Starting simulation for " << duration << " s ..."<< std::endl;
	Simulator::Stop(Seconds(duration));
	Simulator::Run();
	Simulator::Destroy();

}

void VanetSim::Look_at_clock()
{
	std::cout<<"Now:"<<Simulator::Now().GetSeconds();
	std::cout<<"  Mode: "<<m_todo<<"  ,DataSet: "<<m_ds<<std::endl;
  std::cout<<"Tx_Data_Pkts:   "<<Tx_Data_Pkts<<",   "<<Tx_Data_Pkts - old_Tx_Data_Pkts<<std::endl;
  std::cout<<"Rx_Data_Pkts:   "<<Rx_Data_Pkts<<",   "<<Rx_Data_Pkts - old_Rx_Data_Pkts<<std::endl;
  std::cout<<"Unique_RX_Pkts: "<<Unique_RX_Pkts<<",   "<<Unique_RX_Pkts - old_Unique_RX_Pkts<<std::endl;

  //std::cout<<"Rx_Data_Pkts2:   "<<Rx_Data_Pkts2<<",   "<<Rx_Data_Pkts2 - old_Rx_Data_Pkts2<<std::endl;
  //std::cout<<"Unique_RX_Pkts2: "<<Unique_RX_Pkts2<<",   "<<Unique_RX_Pkts2 - old_Unique_RX_Pkts2<<std::endl;

  //std::cout<<"Rx_Data_Pkts3:   "<<Rx_Data_Pkts3<<",   "<<Rx_Data_Pkts3 - old_Rx_Data_Pkts3<<std::endl;
  //std::cout<<"Unique_RX_Pkts3: "<<Unique_RX_Pkts3<<",   "<<Unique_RX_Pkts3 - old_Unique_RX_Pkts3<<std::endl;

  os<<"Now:  "<<Simulator::Now().GetSeconds()
  <<"Tx_Data_Pkts:   "<<Tx_Data_Pkts
  <<"Rx_Data_Pkts3:   "<<Rx_Data_Pkts3
  <<"Unique_RX_Pkts3: "<<Unique_RX_Pkts3<<std::endl;

  old_Unique_RX_Pkts3 = Unique_RX_Pkts3;
  old_Rx_Data_Pkts3 = Rx_Data_Pkts3;
  old_Tx_Data_Pkts  = Tx_Data_Pkts;

	/*Ptr<MobilityModel> Temp = m_nodes.Get (nodeNum)->GetObject<MobilityModel>();
  std::cout<<Temp->GetPosition().x<<","<<Temp->GetPosition().y<<","<<Temp->GetPosition().z<<std::endl;
  std::cout<<Temp->GetVelocity().x<<","<<Temp->GetVelocity().y<<","<<Temp->GetVelocity().z<<std::endl;
  Temp = m_nodes.Get (nodeNum+1)->GetObject<MobilityModel>();
  std::cout<<Temp->GetPosition().x<<","<<Temp->GetPosition().y<<","<<Temp->GetPosition().z<<std::endl;
  std::cout<<Temp->GetVelocity().x<<","<<Temp->GetVelocity().y<<","<<Temp->GetVelocity().z<<std::endl;
  Temp = m_nodes.Get (nodeNum+2)->GetObject<MobilityModel>();
  std::cout<<Temp->GetPosition().x<<","<<Temp->GetPosition().y<<","<<Temp->GetPosition().z<<std::endl;
  std::cout<<Temp->GetVelocity().x<<","<<Temp->GetVelocity().y<<","<<Temp->GetVelocity().z<<std::endl;
  */
	/*
	os<<"Now:"<<Simulator::Now().GetSeconds()<<std::endl;
	Ptr<OutputStreamWrapper> osw = Create<OutputStreamWrapper> (&std::cout);
	m_nodes.Get(nodeNum+1)->GetObject<Ipv4>()->GetRoutingProtocol()->PrintRoutingTable(osw);
	Ptr<OutputStreamWrapper> osw2 = Create<OutputStreamWrapper> (&os);
	m_nodes.Get(nodeNum+1)->GetObject<Ipv4>()->GetRoutingProtocol()->PrintRoutingTable(osw2);
	*/
	/*2  Ptr<MobilityModel> Temp;
	Vector vt;
	for (int i = 0;i<=nodeNum+2;++i)
	{
		Temp = m_nodes.Get(i)->GetObject<MobilityModel>();
		vt = Temp->GetPosition();
		std::cout<<i<<":"<<vt.x<<","<<vt.y<<","<<vt.z<<";"<<std::flush;
	}
	std::cout<<std::endl;*/
	//ProcessOutputs();

	Simulator::Schedule(Seconds(1.0), &VanetSim::Look_at_clock, this);
}

void
VanetSim::TXTrace (Ptr<const Packet> newpacket)
{
  Tx_Data_Pkts++;
  Tx_Data_Bytes += newpacket->GetSize ();
  //std::cout<<"ANOTHER ONE!HAHAHA"<<std::endl;

  Time now = Simulator::Now ();
  delay[newpacket->GetUid ()] = now;

}

// Example to use ns2 traces file in ns3
int main (int argc, char *argv[])
{
  VanetSim SDN_test;
  SDN_test.Simulate(argc, argv);
	return 0;
}



