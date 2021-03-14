#include <iostream>
#include <fstream>
#include <string>
#include <chrono> // seed

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "My_UE_Node.h"
#include "My_UE_Node_Algo.h"

std::vector<My_UE_Node>  Initialize_My_UE_Node_list(NodeContainer  & UE_Nodes){
    
    std::vector<My_UE_Node> myUElist;

    for(int i=0;i<UE_Num;i++){
    
    //先生成required datarate
    //採用C++的uniform distribution generator
    std::uniform_real_distribution<double> unif(0.0 , global_require_data_rate);                    //0~150Mbps
    std::default_random_engine re(std::chrono::system_clock::now().time_since_epoch().count());
    double generated_required_datarate = unif(re);

    //再取得第i個UE的位置
    Ptr<MobilityModel> UE_MobilityModel = (UE_Nodes.Get(i))->GetObject<MobilityModel> ();
    Vector pos = UE_MobilityModel->GetPosition ();

    //新增My_UE_Node加入myUElist中
    myUElist.push_back(My_UE_Node(i,pos,generated_required_datarate));
  }
    return myUElist ; 
}