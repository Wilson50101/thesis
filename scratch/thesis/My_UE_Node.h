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

        void Set_Associated_AP(int associated_AP_index);
        int Get_Associated_AP(void);
        
        void Set_SINR(double in_SINR);
        double Get_SINR(void);

        void AddCurrentRound_Achievable_DataRate(double data_rate_in_Mbps);
        std::vector<double> Get_Achievable_DataRate_History(void);

        void AddCurrentRound_satisfication_level(double satis_level);
        std::vector<double> Get_satisfication_level_History(void);
        
        

    private :
    
        int Node_ID;                                 //user id
        Vector pos;                                  //user position
        double required_datarate;                    //records required datarate (demand)
        int associated_AP;                           //records AP in now round     
        double SINR;                                 //records SINR in now round
        std::vector<double> achievable_datarate;     //records data rate foreach round
        std::vector<double> satisfication_level;     //records satification foreach round

};
#endif