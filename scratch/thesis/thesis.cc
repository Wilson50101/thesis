/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
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
 */

//
// Network topology
//
//           10Mb/s, 10ms       10Mb/s, 10ms
//       n0-----------------n1-----------------n2
//
//
// - Tracing of queues and packet receptions to file 
//   "tcp-large-transfer.tr"
// - pcap traces also generated in the following files
//   "tcp-large-transfer-$n-$i.pcap" where n and i represent node and interface
// numbers respectively
//  Usage (e.g.): ./waf --run tcp-large-transfer

#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <string>
#include <algorithm>
#include <iomanip>

#include "ns3/core-module.h"
#include "ns3/applications-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "global_environment.h"
#include "install_mobility.h"
#include "My_UE_Node.h"
#include "MyAlgo.h"
#include "Channel.h"
#include "print.h"
#include "DynamicLB.h"
#include "ProposedMethod.h"

using namespace ns3;
// using namespace std;

NS_LOG_COMPONENT_DEFINE ("TcpLargeTransfer");
std::vector<double> Received(1, 0);
std::vector<double> theTime(1, 0);

std::vector<std::vector<int>> AP_Association_Matrix( RF_AP_Num + VLC_AP_Num , std::vector<int> (UE_Num + 1,0));                             //χ(i,u)
std::vector<std::vector<double>> TDMA_Matrix( RF_AP_Num + VLC_AP_Num,std::vector<double> (UE_Num +1 ,0));                               //ρ(i,u)

std::vector<std::vector<double>> Handover_Efficiency_Matrix( RF_AP_Num + VLC_AP_Num,std::vector<double> (RF_AP_Num + VLC_AP_Num,0));    //η(i,j)

std::vector<std::vector<double>> RF_Channel_Gain_Matrix(RF_AP_Num,std::vector<double> (UE_Num,0));
std::vector<std::vector<double>> VLC_Channel_Gain_Matrix(VLC_AP_Num,std::vector<double> (UE_Num,0));

std::vector<std::vector<double>> RF_SINR_Matrix(RF_AP_Num,std::vector<double> (UE_Num,0));
std::vector<std::vector<double>> VLC_SINR_Matrix(VLC_AP_Num,std::vector<double> (UE_Num,0));

std::vector<std::vector<double>> RF_DataRate_Matrix( RF_AP_Num + VLC_AP_Num,std::vector<double> (UE_Num,0));
std::vector<std::vector<double>> VLC_DataRate_Matrix(VLC_AP_Num,std::vector<double> (UE_Num,0));

std::vector<double> recorded_avg_datarate_per_UE(UE_Num , 0);
std::vector<double> recorded_satification_per_UE(UE_Num , 0);

static const uint32_t totalTxBytes = 10000000;
static uint32_t currentTxBytes = 0;
static const uint32_t writeSize = 1040;
static int state = 0 ;
uint8_t data[writeSize];



void StartFlow (Ptr<Socket>, Ipv4Address, uint16_t);
void WriteUntilBufferFull (Ptr<Socket>, uint32_t);

std::string intToString(const int& num){
	std::stringstream ss;
	ss << num;
	return ss.str();
}

static void RxEndAddress(Ptr<const Packet> p, const Address &address) { // used for tracing and calculating throughput
    Received.push_back(Received.back() + p->GetSize()); // appends on the received packet to the received data up until that packet and adds that total to the end of the vector
    //total+=p->GetSize();
    theTime.push_back(Simulator::Now().GetSeconds()); // keeps track of the time during simulation that a packet is received
    double throughput = ((Received.back() * 8)) / theTime.back(); //goodput calculation
    std::cout << "Received.back() is :" << Received.back() << std::endl;
    //std::cout << "Sum of p->GetSize() is :" << total << std::endl;
    std::cout << "Rx throughput value is :" << throughput << std::endl;
    std::cout << "Current time is :" << theTime.back() << std::endl;
    std::cout<< "IP: "<<InetSocketAddress::ConvertFrom(address).GetIpv4 ()<<" received size: "<<p->GetSize()<<" at: "<< Simulator::Now().GetSeconds()<<"s"<< std::endl;
}

// void change_dev_rate(NetDeviceContainer & channel){
//   DynamicCast<PointToPointNetDevice>(channel.Get(0))->SetDataRate(DataRate("500Mbps"));
//   DynamicCast<PointToPointNetDevice>(channel.Get(1))->SetDataRate(DataRate("500Mbps"));
// }

void Dynamic_Update_to_NextState(
  NodeContainer  & RF_AP_Nodes , 
  NodeContainer  & VLC_AP_Nodes , 
  NodeContainer  & UE_Nodes,
  std::vector<My_UE_Node> & myUElist){

  // std::cout<<Simulator::Now()<<" Result of State"<< state <<std::endl;
  
  
  
  #if(PROPOSED_METHOD)

  Proposed_DynamicLB( state , RF_AP_Nodes , VLC_AP_Nodes , UE_Nodes , 
  RF_Channel_Gain_Matrix , VLC_Channel_Gain_Matrix ,
  RF_SINR_Matrix , VLC_SINR_Matrix , 
  RF_DataRate_Matrix , VLC_DataRate_Matrix,
  Handover_Efficiency_Matrix , AP_Association_Matrix , TDMA_Matrix , myUElist);

  #else
  
  Benchmark_DynamicLB( state , RF_AP_Nodes , VLC_AP_Nodes , UE_Nodes , 
  RF_Channel_Gain_Matrix , VLC_Channel_Gain_Matrix ,
  RF_SINR_Matrix , VLC_SINR_Matrix , 
  RF_DataRate_Matrix , VLC_DataRate_Matrix,
  Handover_Efficiency_Matrix , AP_Association_Matrix , myUElist);

  #endif
  
  //再多用recorded_avg_datarate_per_UE記錄每個UE的歷史平均速率
  //此舉是爲了提供給Simulator::Run()之後的程式取得datarate
  //否則直接從myUElist去取用不知爲何都是0
  for(int i = 0; i < myUElist.size(); i++)
  {
    
    recorded_avg_datarate_per_UE[i] = myUElist[i].Get_Avg_DataRate();

    double avg_of_satislevel = 0;

    for(int j = 0 ; j < myUElist[i].Get_satisfication_level_History().size() ; j++){
      avg_of_satislevel += myUElist[i].Get_satisfication_level_History()[j];

      //std::cout<<"UE "<<i<<" demand ="<<myUElist[i].Get_Required_DataRate()<<" satislevel:"<<myUElist[i].Get_satisfication_level_History()[j]<<std::endl;
    }

    //std::cout<< myUElist[i].Get_satisfication_level_History().size()<<std::endl;
    avg_of_satislevel /= myUElist[i].Get_satisfication_level_History().size();

    recorded_satification_per_UE[i] = avg_of_satislevel;
    
  }
    



  if(!Simulator::IsFinished())

    Simulator::Schedule(MilliSeconds(Tp),& Dynamic_Update_to_NextState, RF_AP_Nodes , VLC_AP_Nodes , UE_Nodes, myUElist);
  
}

int main (int argc, char *argv[])
{
  // Users may find it convenient to turn on explicit debugging
  // for selected modules; the below lines suggest how to do this
  //  LogComponentEnable("TcpL4Protocol", LOG_LEVEL_ALL);
  //  LogComponentEnable("TcpSocketImpl", LOG_LEVEL_ALL);
  //  LogComponentEnable("PacketSink", LOG_LEVEL_ALL);
  //  LogComponentEnable("TcpLargeTransfer", LOG_LEVEL_ALL);

  CommandLine cmd;
  cmd.Parse (argc, argv);

  Config::SetDefault("ns3::TcpSocket::SegmentSize",UintegerValue(1000));
  Config::SetDefault("ns3::TcpSocket::SndBufSize",UintegerValue(1000000000));
  Config::SetDefault("ns3::TcpSocket::RcvBufSize",UintegerValue(1000000000));

  // initialize the tx buffer.
  for(uint32_t i = 0; i < writeSize; ++i)
    {
      char m = toascii (97 + i % 26);
      data[i] = m;
    }

  /** Create RF AP Node **/
  NodeContainer RF_AP_Nodes;
  
  RF_AP_Nodes.Create (RF_AP_Num);
  
  install_RF_AP_mobility(RF_AP_Nodes);
  
  #if DEBUG_MODE
    print_RF_AP_position(RF_AP_Nodes);    //for debug use
  #endif
  
  /** Create VLC AP Nodes **/
  NodeContainer VLC_AP_Nodes;
  
  VLC_AP_Nodes.Create (VLC_AP_Num);
  
  install_VLC_AP_mobility(VLC_AP_Nodes);
  
  #if DEBUG_MODE
    print_VLC_AP_position(VLC_AP_Nodes);  //for debug use
  #endif
  
  
  /** Create UE Nodes **/
  NodeContainer UE_Nodes;
  
  UE_Nodes.Create (UE_Num);
  
  install_UE_mobility(UE_Nodes);
  
  #if DEBUG_MODE
    print_UE_position(UE_Nodes);          //for debug use
  #endif

  //生成自定義的UElist
  std::vector<My_UE_Node> myUElist = Initialize_My_UE_Node_list(UE_Nodes);
  
  // std::cout<<"Demand for each UE : "<<std::endl;
  // for(int i = 0 ; i < myUElist.size() ; i++){
  //       std::cout<<"id:"<<myUElist[i].GetID()<<" "<<myUElist[i].Get_Required_DataRate()<<" Mbps"<<std::endl;
  // }
  
  


  // Benchmark_DynamicLB(state , RF_AP_Nodes , VLC_AP_Nodes , UE_Nodes , 
  // RF_Channel_Gain_Matrix , VLC_Channel_Gain_Matrix ,
  // RF_SINR_Matrix , VLC_SINR_Matrix , 
  // RF_DataRate_Matrix , VLC_DataRate_Matrix,
  // Handover_Efficiency_Matrix , AP_Association_Matrix , myUElist);


  /** add ip/tcp stack to all nodes.**/
  // InternetStackHelper internet;
  // internet.InstallAll ();
  

 

  // /** set p2p helper **/
  // PointToPointHelper p2p;
  // p2p.SetDeviceAttribute ("DataRate", StringValue ("5Mbps"));
  // p2p.SetChannelAttribute ("Delay", TimeValue (MilliSeconds (0.1)));

  
  
  // /*
  //   Channel[i][j] 放的是 AP i - UE j的link
  //   該link用Netdevicecontainer表達
  //   Netdevicecontainer.Get(0) = transmitter device
  //   Netdevicecontainer.Get(1) = receiver device
  // */
  
  // std::vector<std::vector<NetDeviceContainer>> VLC_Channel(VLC_AP_Num,std::vector<NetDeviceContainer> (UE_Num));  
  // for(int i=0;i<VLC_AP_Num;i++){
  //   for(int j=0;j<UE_Num;j++){
  //     VLC_Channel[i][j]=p2p.Install(VLC_AP_Nodes.Get(i),UE_Nodes.Get(j));
  //   }
  // }
   
  // /** Later, we add IP addresses. **/
  // std::vector<Ipv4InterfaceContainer> ipvec;
  // Ipv4AddressHelper ipv4;
  // for(int i=0;i<VLC_AP_Num;i++){
  //   for(int j=0;j<UE_Num;j++){
  //     std::string addressStr = "10."+intToString(i+1) +"." + intToString(j+1) + ".0";
  //     ipv4.SetBase(addressStr.c_str(),"255.255.255.0");
  //     ipvec.push_back(ipv4.Assign (VLC_Channel[i][j]));
      
  //     #if DEBUG_MODE
  //       std::cout<<ipvec.back().GetAddress(1)<<" ";
  //     #endif
  //   }
  //   #if DEBUG_MODE
  //     std::cout << std::endl;
  //   #endif
  // }


  
  // // set serve Port
  // uint16_t servPort = 50000;

  // // Create a packet sink to receive these packets on n2...
  // PacketSinkHelper sink ("ns3::TcpSocketFactory",
  //                        InetSocketAddress (Ipv4Address::GetAny (), servPort));

  // ApplicationContainer apps ;
  // apps.Add(sink.Install(UE_Nodes));
  // apps.Start (Seconds (0.0));
  // apps.Stop (Seconds (6.0));



  // // Create and bind the socket...
  // std::vector<std::vector<Ptr<Socket>>> localSockets(VLC_AP_Num,std::vector<Ptr<Socket> > (UE_Num));
  // for(int i=0;i<VLC_AP_Num;i++){
  //   for(int j=0;j<UE_Num;j++){
  //     localSockets[i][j] = Socket::CreateSocket (VLC_AP_Nodes.Get (i), TcpSocketFactory::GetTypeId ()); 
  //     localSockets[i][j]->Bind();
  //   }
  // }  
  // // Trace changes to the congestion window
  // // Config::ConnectWithoutContext ("/NodeList/0/$ns3::TcpL4Protocol/SocketList/0/CongestionWindow", MakeCallback (&CwndTracer));

  // ApplicationContainer::Iterator i;
  // for (i = apps.Begin (); i != apps.End (); ++i){
  //         (*i)->TraceConnectWithoutContext("Rx", MakeCallback(&RxEndAddress));
  // }
  // // ...and schedule the sending "Application"; This is similar to what an 
  // // ns3::Application subclass would do internally.

  // int ipvec_index=0;
  // for(int i=0;i<VLC_AP_Num;i++){
  //   for(int j=0;j<UE_Num;j++){
  //     Simulator::Schedule(Seconds(0.0),&StartFlow,localSockets[i][j],ipvec[ipvec_index++].GetAddress(1), servPort);
  //   }
  // }

  Simulator::Schedule(Seconds(0.0),& Dynamic_Update_to_NextState, RF_AP_Nodes , VLC_AP_Nodes , UE_Nodes, myUElist);
  
  
  Simulator::Stop (Minutes(2));
  Simulator::Run ();

  // #if DEBUG_MODE
  //   print_RF_DataRate_Matrix(RF_DataRate_Matrix);
  //   print_VLC_DataRate_Matrix(VLC_DataRate_Matrix);
  // #endif

  ////////////////////////////////
  ////                        ////
  //// system avg throughput  ////
  ////                        ////
  ////////////////////////////////
  double sumTP=0;
  for(int i=0;i<UE_Num;i++)
  {
    sumTP += recorded_avg_datarate_per_UE[i];
  }
    double sys_avg_rate = sumTP / UE_Num ;
  
 
  ////////////////////////////////
  ////                        ////
  ////   system fairness      ////
  ////                        ////
  ////////////////////////////////
  double fairness;
  double squareofsum = 0;
  double sumofsquare = 0;
  for(int i=0 ; i<UE_Num;i++){

      squareofsum += recorded_avg_datarate_per_UE[i];
      sumofsquare += pow(recorded_avg_datarate_per_UE[i],2);
  }

  squareofsum = pow(squareofsum,2);
  fairness = squareofsum / (UE_Num * sumofsquare);
  //std::cout<<"System Fairness index ="<< fairness <<std::endl;
 
  ////////////////////////////////////////////////
  ////                                        ////
  ////   system avg satisfication level       ////
  ////                                        ////
  ////////////////////////////////////////////////
  
  //平均每個UE的滿意度
  double sys_avg_satis_level = 0;
  double outage = 0;
  for(int i = 0 ; i<recorded_satification_per_UE.size(); i++)
  {
    sys_avg_satis_level += recorded_satification_per_UE[i];
    if(recorded_satification_per_UE[i] < satis_threshold ) 
      outage++;
  }
  sys_avg_satis_level /= UE_Num ;

  sort(myUElist.begin(),myUElist.end(),[](My_UE_Node a,My_UE_Node b){return a.Get_Required_DataRate() > b.Get_Required_DataRate();});
  // for(int i=0;i<myUElist.size();i++)
  // {
  //   std::cout<<"UE id = "<<myUElist[i].GetID()<<" Demand ="<<myUElist[i].Get_Required_DataRate()<<" Avg DR = "<<recorded_avg_datarate_per_UE[myUElist[i].GetID()]<<" satislevel = "<<recorded_satification_per_UE[myUElist[i].GetID()]<<std::endl;
  // }

  ////////////////////////////////
  ////                        ////
  ////   output to .csv file  ////
  ////                        ////
  ////////////////////////////////
  std::fstream outFile;
	outFile.open("/home/hsnk/repos/ns-3-allinone/ns-3.25/scratch/thesis/output.csv", std::ios::out|std::ios::app);
  
  if(!outFile.is_open())

    std::cout<<"file not open";
  
  else
  {
    //std::cout<<state<<std::endl;
    outFile << sumTP << ',' << sys_avg_rate  << ',' << fairness << ',' << sys_avg_satis_level << ','<< outage / UE_Num << ','; 
    
    for(int i=0; i < RF_AP_Num + VLC_AP_Num ; i++)
      outFile << std::setiosflags(std::ios::fixed)<< std::setprecision(4) << ((double) AP_Association_Matrix[i][UE_Num] ) / ( state * UE_Num )<<',';
    
    outFile << std::endl;
  }

  outFile.close();


  Simulator::Destroy ();
}


//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//begin implementation of sending "Application"
void StartFlow (Ptr<Socket> localSocket,
                Ipv4Address servAddress,
                uint16_t servPort)
{
  NS_LOG_LOGIC ("Starting flow at time " <<  Simulator::Now ().GetSeconds ());
  localSocket->Connect (InetSocketAddress (servAddress, servPort)); //connect

  // tell the tcp implementation to call WriteUntilBufferFull again
  // if we blocked and new tx buffer space becomes available
  //localSocket->SetSendCallback (MakeCallback (&WriteUntilBufferFull));
  WriteUntilBufferFull (localSocket, localSocket->GetTxAvailable ());
  currentTxBytes=0;
}

void WriteUntilBufferFull (Ptr<Socket> localSocket, uint32_t txSpace)
{
  while (currentTxBytes < totalTxBytes && localSocket->GetTxAvailable () > 0) 
    {
      uint32_t left = totalTxBytes - currentTxBytes;
      uint32_t dataOffset = currentTxBytes % writeSize;
      uint32_t toWrite = writeSize - dataOffset;
      toWrite = std::min (toWrite, left);
      toWrite = std::min (toWrite, localSocket->GetTxAvailable ());
      int amountSent = localSocket->Send (&data[dataOffset], toWrite, 0);
      if(amountSent < 0)
        {
          // we will be called again when new tx space becomes available.
          return;
        }
      currentTxBytes += amountSent;
    }
  localSocket->Close ();
}

