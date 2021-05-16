#ifndef PROPOSED_METHOD_H
#define PROPOSED_METHOD_H

#include "My_UE_Node.h"

void Proposed_DynamicLB(
  int & state,
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
  std::vector<std::vector<double>> & TDMA_Matrix,                            
  std::vector<My_UE_Node> & myUElist);

void Proposed_LB_state0(
  int & state,
  std::vector<std::vector<double>> & RF_DataRate_Matrix ,
  std::vector<std::vector<double>> & VLC_DataRate_Matrix,
  std::vector<std::vector<double>> & Handover_Efficiency_Matrix,
  std::vector<std::vector<int>>    & AP_Association_Matrix ,
  std::vector<std::vector<double>> & TDMA_Matrix,                           
  std::vector<My_UE_Node> & myUElist);
 
void Proposed_LB_stateN( 
  int & state,
  std::vector<std::vector<double>> & RF_DataRate_Matrix ,
  std::vector<std::vector<double>> & VLC_DataRate_Matrix,
  std::vector<std::vector<double>> & Handover_Efficiency_Matrix,
  std::vector<std::vector<int>>    & AP_Association_Matrix ,
  std::vector<std::vector<double>> & TDMA_Matrix,                           
  std::vector<My_UE_Node> & myUElist);

void maximize_throughput(
  int & state, 
  std::vector<std::vector<double>> & RF_DataRate_Matrix ,
  std::vector<std::vector<double>> & VLC_DataRate_Matrix,
  std::vector<std::vector<double>> & Handover_Efficiency_Matrix,
  std::vector<std::vector<int>>    & AP_Association_Matrix ,  
  std::vector<std::vector<double>> & TDMA_Matrix,                            
  std::vector<My_UE_Node> & myUElist);

void share_by_propotion(
  int & state,
  std::vector<std::vector<double>> & RF_DataRate_Matrix ,
  std::vector<std::vector<double>> & VLC_DataRate_Matrix,
  std::vector<std::vector<double>> & Handover_Efficiency_Matrix,
  std::vector<std::vector<int>>    & AP_Association_Matrix ,  
  std::vector<std::vector<double>> & TDMA_Matrix,                            
  std::vector<My_UE_Node> & myUElist);

void save_lower_throughputUE( 
  int & state,
  std::vector<std::vector<double>> & RF_DataRate_Matrix ,
  std::vector<std::vector<double>> & VLC_DataRate_Matrix,
  std::vector<std::vector<double>> & Handover_Efficiency_Matrix,
  std::vector<std::vector<int>>    & AP_Association_Matrix ,  
  std::vector<std::vector<double>> & TDMA_Matrix,                            
  std::vector<My_UE_Node> & myUElist);

#endif