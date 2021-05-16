#ifndef PRINT_H
#define PRINT_H

#include "global_environment.h"
#include "My_UE_Node.h"

void print_RF_Channel_Gain_Matrix(std::vector<std::vector<double>>  & RF_Channel_Gain_Matrix);

void print_VLC_Channel_Gain_Matrix(std::vector<std::vector<double>>  & VLC_Channel_Gain_Matrix);

void print_RF_SINR_Matrix(std::vector<std::vector<double>>  & RF_SINR_Matrix);

void print_VLC_SINR_Matrix(std::vector<std::vector<double>>  & VLC_SINR_Matrix);

void print_RF_DataRate_Matrix(std::vector<std::vector<double>>  & RF_DataRate_Matrix);

void print_VLC_DataRate_Matrix(std::vector<std::vector<double>>  & VLC_DataRate_Matrix);

void print_Handover_Efficiency_Matrix(std::vector<std::vector<double>> & Handover_Efficiency_Matrix);

void print_AP_Association_Matrix(std::vector<std::vector<int>>  & AP_Association_Matrix);

void print_TDMA_Matrix(std::vector<std::vector<double>> & TDMA_Matrix);

#endif