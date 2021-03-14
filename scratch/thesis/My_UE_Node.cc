#include <iostream>
#include <fstream>
#include <string>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "My_UE_Node.h"

using namespace ns3;
My_UE_Node::My_UE_Node(int id,Vector in_pos,double required_rate){
    Node_ID=id;
    pos = in_pos;
    required_datarate = required_rate;
    associated_AP = -1;
    SINR = 0;
}

// void 
// My_UE_Node::Set_My_UE_Node_ID(int id){
//     Node_ID = id;
// }
int 
My_UE_Node::GetID(void){
    return Node_ID;
}

void 
My_UE_Node::SetPosition(Vector pos_from_mobilitymodel){
    pos=pos_from_mobilitymodel;
}

Vector 
My_UE_Node::GetPosition(void){
    return pos;
}

void 
My_UE_Node::Set_Required_DataRate(double data_rate_in_Mbps){
    required_datarate = data_rate_in_Mbps;
}

double 
My_UE_Node::Get_Required_DataRate(void){
    return required_datarate;
}

void 
My_UE_Node::Set_Associated_AP(int associated_AP_index){
    associated_AP = associated_AP_index;
}

int 
My_UE_Node::Get_Associated_AP(void){
    return associated_AP;
}

void 
My_UE_Node::Set_SINR(double in_SINR){
    SINR = in_SINR;
}

double 
My_UE_Node::Get_SINR(void){
    return SINR;
}

void 
My_UE_Node::AddCurrentRound_Achievable_DataRate(double data_rate_in_Mbps){
    achievable_datarate.push_back(data_rate_in_Mbps);
}

std::vector<double> 
My_UE_Node::Get_Achievable_DataRate_History(void){
    return achievable_datarate;
}

void 
My_UE_Node::AddCurrentRound_satisfication_level(double satis_level){
    satisfication_level.push_back(satis_level);
}

std::vector<double> 
My_UE_Node::Get_satisfication_level_History(void){
    return satisfication_level;
}


