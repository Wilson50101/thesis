#ifndef CHANNEL_H
#define CHANNEL_H

#include <iostream>
#include <fstream>
#include <string>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "global_environment.h"

using namespace ns3;

////////////////////////////////////////////////////////
/////////   distance & angle calculation       /////////
////////////////////////////////////////////////////////

double RadtoDeg(const double & radian);

double DegtoRad(const double & degree);

double GetDistance_AP_UE(Ptr<Node> AP, Ptr<Node> UE);

double Get_Incidence_Angle_AP_UE(Ptr<Node> AP, Ptr<Node> UE);

////////////////////////////////////////////////////////
/////////          Channel Gain                /////////
////////////////////////////////////////////////////////

void Calculate_RF_Channel_Gain_Matrix(NodeContainer  & RF_AP_Node , NodeContainer  & UE_Nodes ,std::vector<std::vector<double>>  & RF_Channel_Gain_Matrix);

void Calculate_VLC_Channel_Gain_Matrix(NodeContainer  & VLC_AP_Nodes , NodeContainer  &UE_Nodes ,std::vector<std::vector<double>>  &VLC_Channel_Gain_Matrix);

double Estimate_one_RF_Channel_Gain(Ptr<Node> RF_AP , Ptr<Node> UE);

double Estimate_one_VLC_Channel_Gain(Ptr<Node> VLC_AP , Ptr<Node> UE);

////////////////////////////////////////////////////////
/////////          SINR                        /////////
////////////////////////////////////////////////////////
void Calculate_RF_SINR_Matrix(std::vector<std::vector<double>>  & RF_Channel_Gain_Matrix,std::vector<std::vector<double>> & RF_SINR_Matrix);

void Calculate_VLC_SINR_Matrix(std::vector<std::vector<double>>  & VLC_Channel_Gain_Matrix,std::vector<std::vector<double>> & VLC_SINR_Matrix);

double Estimate_one_RF_SINR(std::vector<std::vector<double>>  & RF_Channel_Gain_Matrix , int RF_AP_Index , int UE_index);

double Estimate_one_VLC_SINR(std::vector<std::vector<double>> & VLC_Channel_Gain_Matrix, int VLC_AP_Index , int UE_index);


////////////////////////////////////////////////////////
/////////          Data rate                   /////////
////////////////////////////////////////////////////////
void Calculate_RF_DataRate_Matrix(std::vector<std::vector<double>>  & RF_SINR_Matrix,std::vector<std::vector<double>> & RF_DataRate_Matrix);

void Calculate_VLC_DataRate_Matrix(std::vector<std::vector<double>>  & VLC_SINR_Matrix,std::vector<std::vector<double>> & VLC_DataRate_Matrix);

////////////////////////////////////////////////////////
/////////         Handover_Efficiency          /////////
////////////////////////////////////////////////////////

void Calculate_Handover_Efficiency_Matrix(std::vector<std::vector<double>> & Handover_Efficiency_Matrix);
#endif