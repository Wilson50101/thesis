////////////////////////////////////////////////////////////////////////////////////////////
//// Q1 : 這份code在幹嘛？                                                                 ///
//// A1 : 我自己定義的UE Node                                                              ///
////                                                                                     ///
//// Q2 : 爲什麼要自定義？                                                                  ///
//// A2 : 每個UE都必須記錄很多訊息例如連到的AP,獲得的資源量以及throughput等等                     ///
////      NS3原生的Node類別做不到記錄這些事情，所以我幹脆自幹一個class來記錄，方便我寫演算法         ///                                                              
////////////////////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <fstream>
#include <string>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "My_UE_Node.h"

using namespace ns3;

//建構子，初始化用
My_UE_Node::My_UE_Node(int id,Vector in_pos,double required_rate){
    
    Node_ID = id;
    pos = in_pos;
    required_datarate = required_rate;
    avg_throughput = 0 ;                  
    prev_associated_AP = -1;            //-1代表沒有上一輪AP
    now_associated_AP = -1;             //-1代表這一輪還沒分配AP
    time_fraction = 0 ;                 
    SINR = 0;                          

}


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
My_UE_Node::Set_Avg_Throughput(double data_rate_in_Mbps){
    avg_throughput = data_rate_in_Mbps;
}

double 
My_UE_Node::Get_Avg_Throughput(void){
    return avg_throughput;
}

void 
My_UE_Node::Set_Now_Associated_AP(int associated_AP_index){
    prev_associated_AP = now_associated_AP;
    now_associated_AP = associated_AP_index;
}

int 
My_UE_Node::Get_Now_Associated_AP(void){
    return now_associated_AP;
}


void 
My_UE_Node::Set_Time_Fraction(double timefrac){
    time_fraction = timefrac;
}

double 
My_UE_Node::Get_Time_Fraction(void){
    return time_fraction;
}


int 
My_UE_Node::Get_Prev_Associated_AP(void){
    return prev_associated_AP;
}


void 
My_UE_Node::Set_SINR(double in_SINR){
    SINR = in_SINR;
}

double 
My_UE_Node::Get_SINR(void){
    return SINR;
}

//新增當前iteration的throughput到該UE的throughput歷史記錄中
void 
My_UE_Node::Add_Curr_iteration_Throughput(double data_rate_in_Mbps){
    
    //新增到歷史記錄的vector中
    throughput_per_iteration.push_back(data_rate_in_Mbps);


    //此外，每當多了一筆歷史記錄就必須要一起更新歷史平均值,即avg throughput
    //先計算歷史throughput總和
    double sum = 0;
    for(int i =  0 ; i < throughput_per_iteration.size() ; i++) sum += throughput_per_iteration[i];

    //歷史平均 = 歷史總和 / 樣本個數
    Set_Avg_Throughput( sum /  throughput_per_iteration.size());

}

//取得這個UE的Throughput歷史記錄
std::vector<double> 
My_UE_Node::Get_Throughput_History(void){
    return throughput_per_iteration;
}

//新增此輪滿意度到該UE的歷史滿意度記錄中
void 
My_UE_Node::Add_Curr_satisfication_level(double satis_level){
    satisfication_level.push_back(satis_level);
}

//取得此UE的滿意度之歷史記錄
std::vector<double> 
My_UE_Node::Get_satisfication_level_History(void){
    return satisfication_level;
}


