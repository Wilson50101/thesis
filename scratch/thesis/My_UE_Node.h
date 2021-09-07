#ifndef MY_NODE_H
#define MY_NODE_H

#include <iostream>
#include <fstream>
#include <string>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "global_environment.h"

using namespace ns3;

class My_UE_Node {
    
    public :
        My_UE_Node(int id,Vector pos,double required_rate);

        // void Set_My_UE_Node_ID(int id);
        int GetID(void);

        void SetPosition(Vector pos_from_mobilitymodel);
        Vector GetPosition(void);

        void Set_Required_DataRate(double data_rate_in_Mbps);
        double Get_Required_DataRate(void);

        void Set_Avg_Throughput(double data_rate_in_Mbps);
        double Get_Avg_Throughput(void);

        void Set_Now_Associated_AP(int associated_AP_index);
        int Get_Now_Associated_AP(void);

        void Set_Time_Fraction(double timefrac);
        double Get_Time_Fraction(void);


        int Get_Prev_Associated_AP(void);

        void Set_SINR(double in_SINR);
        double Get_SINR(void);

        void Add_Curr_iteration_Throughput(double data_rate_in_Mbps);
        std::vector<double> Get_Throughput_History(void);

        void Add_Curr_satisfication_level(double satis_level);
        std::vector<double> Get_satisfication_level_History(void);
        
        

    private :
    
        int Node_ID;                                 //UE id
        Vector pos;                                  //UE位置 用向量記錄
        double required_datarate;                    //記錄demand大小
        double avg_throughput;                       //記錄平均吞吐量
        //[0 , RF_AP_Num-1] 代表RF AP的index
        //[RF_AP_Num , RF_AP_Num+VLC_AP_Num-1] 代表VLC AP的index
        int prev_associated_AP ;                     //上一輪連到的AP 
        int now_associated_AP;                       //這一輪連到的AP   
        double time_fraction;                        //獲得的時間資源(0~1)
        double SINR;                                 //此輪的SINR
        std::vector<double> throughput_per_iteration;//每一輪的throughput
        std::vector<double> satisfication_level;     //每一輪的滿意度

};
#endif