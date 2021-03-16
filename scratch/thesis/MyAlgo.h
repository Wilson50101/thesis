#ifndef MY_ALGO_H
#define MY_ALGO_H

#include <iostream>
#include <fstream>
#include <string>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"

#include "global_environment.h"
#include "My_UE_Node.h"

using namespace ns3;

std::vector<My_UE_Node> Initialize_My_UE_Node_list(NodeContainer  & UE_Nodes);

////////////////////////////////////////////////////////
/////////          SINR                        /////////
////////////////////////////////////////////////////////
void Calculate_RF_SINR_Matrix(std::vector<std::vector<double>>  & RF_Channel_Gain_Matrix,std::vector<std::vector<double>> & RF_SINR_Matrix);

void Calculate_VLC_SINR_Matrix(std::vector<std::vector<double>>  & VLC_Channel_Gain_Matrix,std::vector<std::vector<double>> & VLC_SINR_Matrix);

double Estimate_one_RF_SINR(std::vector<std::vector<double>>  & RF_Channel_Gain_Matrix , int RF_AP_Index , int UE_index);

double Estimate_one_VLC_SINR(std::vector<std::vector<double>> & VLC_Channel_Gain_Matrix, int VLC_AP_Index , int UE_index);

void print_RF_SINR_Matrix(std::vector<std::vector<double>>  & RF_SINR_Matrix);

void print_VLC_SINR_Matrix(std::vector<std::vector<double>>  & VLC_SINR_Matrix);

////////////////////////////////////////////////////////
/////////          Data rate                   /////////
////////////////////////////////////////////////////////
void Calculate_RF_DataRate_Matrix(std::vector<std::vector<double>>  & RF_SINR_Matrix,std::vector<std::vector<double>> & RF_DataRate_Matrix);

void Calculate_VLC_DataRate_Matrix(std::vector<std::vector<double>>  & VLC_SINR_Matrix,std::vector<std::vector<double>> & VLC_DataRate_Matrix);

void print_RF_DataRate_Matrix(std::vector<std::vector<double>>  & RF_DataRate_Matrix);

void print_VLC_DataRate_Matrix(std::vector<std::vector<double>>  & VLC_DataRate_Matrix);
#endif