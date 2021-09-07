////////////////////////////////////////////////////////////////////////////////////////////
//// Q1 : 這份code在幹嘛？                                                                 ///
//// A1 : 對一個含有My_UE_Node的vector做初始化                                              ///
////                                                                                     ///
//// Q2 : 爲什麼要自定義？                                                                  ///
//// A2 : 每個UE都必須記錄很多訊息例如連到的AP,獲得的資源量以及throughput等等                     ///
////      NS3原生的Node類別做不到記錄這些事情，所以我幹脆自幹一個class來記錄，方便我寫演算法         ///                                                              
////////////////////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <fstream>
#include <string>
#include <chrono> // seed

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "My_UE_Node.h"
#include "My_UE_Node_List.h"

std::vector<My_UE_Node>  Initialize_My_UE_Node_list(NodeContainer  & UE_Nodes){
    
    std::vector<My_UE_Node> myUElist;
    //先生成required datarate
    //採用C++的uniform distribution generator

    // 期望值 = avg_require_data_rate = 30 Mbps， 標準差 = 6 來做隨機
    // 這樣一來required data rate大約是 10 ~ 40Mbps不等
    // std::normal_distribution <double> norm(avg_require_data_rate , 6 );

    //demand有高有低
    std::normal_distribution <double> low( low_demand , 2 );
    std::normal_distribution <double> high( high_demand , 2 );
    std::default_random_engine re(std::chrono::system_clock::now().time_since_epoch().count());
  
    for(int i=0;i<UE_Num;i++){
   
      double generated_required_datarate;
  
      //如果是高demand
      if( i < UE_Num * ratioHDU)
        
        generated_required_datarate = high(re);
      
      //否則爲低demand
      else
        
        generated_required_datarate = low(re);

      //再取得第i個UE的位置
      Ptr<MobilityModel> UE_MobilityModel = (UE_Nodes.Get(i))->GetObject<MobilityModel> ();
      Vector pos = UE_MobilityModel->GetPosition ();

      //新增My_UE_Node加入myUElist中
      myUElist.push_back(My_UE_Node(i,pos,generated_required_datarate));
  }
    return myUElist ; 
}

