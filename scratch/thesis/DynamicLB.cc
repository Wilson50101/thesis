#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "My_UE_Node.h"
#include "DynamicLB.h"
#include "Channel.h"
#include "print.h"

//執行dynamic的load balancing
void DynamicLB(
  NodeContainer  & RF_AP_Nodes , 
  NodeContainer  & VLC_AP_Nodes , 
  NodeContainer  & UE_Nodes,
  std::vector<std::vector<double>> & RF_Channel_Gain_Matrix , 
  std::vector<std::vector<double>> & VLC_Channel_Gain_Matrix ,
  std::vector<std::vector<double>> & RF_SINR_Matrix ,
  std::vector<std::vector<double>> & VLC_SINR_Matrix ,
  std::vector<std::vector<double>> & RF_DataRate_Matrix ,
  std::vector<std::vector<double>> & VLC_DataRate_Matrix,
  std::vector<std::vector<double>> & Handover_Efficiency_Matrix ,
  std::vector<std::vector<int>>    & AP_Association_Matrix ,                          
  std::vector<std::vector<double>> & TDMA_Matrix ,
  std::vector<My_UE_Node> & myUElist)
{  
    for(int i = 0 ; i < myUElist.size() ; i++){
        std::cout<<"id:"<<myUElist[i].GetID()<<" "<<myUElist[i].Get_Required_DataRate()<<std::endl;
    }
        

    //先做PreCalculation算出目前的各種Matrix
    DoPreCalculation(RF_AP_Nodes , VLC_AP_Nodes , UE_Nodes,
    RF_Channel_Gain_Matrix , VLC_Channel_Gain_Matrix ,
    RF_SINR_Matrix , VLC_SINR_Matrix ,
    RF_DataRate_Matrix , VLC_DataRate_Matrix,
    Handover_Efficiency_Matrix);

    // Simulator::Schedule(Seconds(0.5) , & DynamicLB , RF_AP_Nodes , VLC_AP_Nodes , UE_Nodes , 
    // RF_Channel_Gain_Matrix , VLC_Channel_Gain_Matrix ,
    // RF_SINR_Matrix , VLC_SINR_Matrix , 
    // RF_DataRate_Matrix , VLC_DataRate_Matrix,
    // Handover_Efficiency_Matrix , AP_Association_Matrix , TDMA_Matrix , myUElist);
  
}




//執行PreCalulation，以供後續演算法進行決策
//要計算的項目有
//1. Channel gain Matrix (RF/VLC)
//2. SINR Matrix (RF/VLC)
//3. ideal DataRate of pair<AP i,UE j> 說明 ： 考慮的是 AP i 只服務 UE j，給其全部resource
//4. handover efficiency matrix 
void DoPreCalculation(
  NodeContainer  & RF_AP_Nodes , 
  NodeContainer  & VLC_AP_Nodes , 
  NodeContainer  & UE_Nodes,
  std::vector<std::vector<double>> & RF_Channel_Gain_Matrix , 
  std::vector<std::vector<double>> & VLC_Channel_Gain_Matrix ,
  std::vector<std::vector<double>> & RF_SINR_Matrix ,
  std::vector<std::vector<double>> & VLC_SINR_Matrix ,
  std::vector<std::vector<double>> & RF_DataRate_Matrix ,
  std::vector<std::vector<double>> & VLC_DataRate_Matrix,
  std::vector<std::vector<double>> & Handover_Efficiency_Matrix)
{
  
  //算RF/VLC的Channel gain
  Calculate_RF_Channel_Gain_Matrix(RF_AP_Nodes,UE_Nodes,RF_Channel_Gain_Matrix);
  Calculate_VLC_Channel_Gain_Matrix(VLC_AP_Nodes,UE_Nodes,VLC_Channel_Gain_Matrix);
  
  #if DEBUG_MODE
    print_RF_Channel_Gain_Matrix(RF_Channel_Gain_Matrix);
    print_VLC_Channel_Gain_Matrix(VLC_Channel_Gain_Matrix);
  #endif
  

  //算RF/VLC的SINR
  Calculate_RF_SINR_Matrix(RF_Channel_Gain_Matrix,RF_SINR_Matrix);
  Calculate_VLC_SINR_Matrix(VLC_Channel_Gain_Matrix,VLC_SINR_Matrix);
  
  #if DEBUG_MODE
    print_RF_SINR_Matrix(RF_SINR_Matrix);
    print_VLC_SINR_Matrix(VLC_SINR_Matrix);
  #endif

  //算RF/VLC的DataRate
  Calculate_RF_DataRate_Matrix(RF_SINR_Matrix , RF_DataRate_Matrix);
  Calculate_VLC_DataRate_Matrix(VLC_SINR_Matrix , VLC_DataRate_Matrix);
  
  #if DEBUG_MODE
    print_RF_DataRate_Matrix(RF_DataRate_Matrix);
    print_VLC_DataRate_Matrix(VLC_DataRate_Matrix);
  #endif

  //算Handover_Efficiency_Matrix
  Calculate_Handover_Efficiency_Matrix(Handover_Efficiency_Matrix);
  
  #if DEBUG_MODE
    print_Handover_Efficiency_Matrix(Handover_Efficiency_Matrix);
  #endif
}