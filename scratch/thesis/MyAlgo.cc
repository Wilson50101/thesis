#include <iostream>
#include <fstream>
#include <string>
#include <chrono> // seed

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "My_UE_Node.h"
#include "MyAlgo.h"

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


//計算整個RF SINR matrix
void Calculate_RF_SINR_Matrix(std::vector<std::vector<double>> & RF_Channel_Gain_Matrix,std::vector<std::vector<double>> & RF_SINR_Matrix){
    for(int i=0 ; i < RF_AP_Num ; i++){
        for(int j=0 ; j < UE_Num ; j++){
            RF_SINR_Matrix[i][j] = Estimate_one_RF_SINR(RF_Channel_Gain_Matrix , i , j);
        }
    }
}

//計算整個VLC SINR matrix
void Calculate_VLC_SINR_Matrix(std::vector<std::vector<double>> & VLC_Channel_Gain_Matrix,std::vector<std::vector<double>> & VLC_SINR_Matrix){
    for(int i=0 ; i < VLC_AP_Num ; i++){
        for(int j=0 ; j < UE_Num ; j++){
            VLC_SINR_Matrix[i][j] = Estimate_one_VLC_SINR(VLC_Channel_Gain_Matrix ,  i , j );
        }
    }
}

//計算某個 pair(RF_AP,UE)的SINR
//Note : 實際上paper只有一個RF_AP，所以等同SNR

double Estimate_one_RF_SINR(std::vector<std::vector<double>>  & RF_Channel_Gain_Matrix , int RF_AP_Index , int UE_Index){
  
  //只有一個RF_AP，所以interference = 0
  double interference = 0;

  //再計算出noise項
  double noise = RF_AP_Bandwidth * Nr ;

  //再計算出SINR項
  double SINR = pow(RF_Channel_Gain_Matrix[RF_AP_Index][UE_Index],2) * RF_AP_Power / (noise + interference);

  return SINR;

}

//計算某個 pair(VLC_AP,UE)的SINR
double Estimate_one_VLC_SINR(std::vector<std::vector<double>> & VLC_Channel_Gain_Matrix, int VLC_AP_Index , int UE_Index){

  //先把其他AP和該UE的channel gain納入幹擾項
  double interference = 0;
  for(int i = 0 ; i < VLC_AP_Num ; i++ ){
      if(i!=VLC_AP_Index)
        interference +=  pow(kappa * VLC_AP_Popt * VLC_Channel_Gain_Matrix[i][UE_Index],2);
  }

  //再計算出noise項
  double noise = VLC_AP_Bandwidth * Nl ;

  //再計算出SINR項
  double SINR = pow(kappa * VLC_AP_Popt * VLC_Channel_Gain_Matrix[VLC_AP_Index][UE_Index],2)/ (noise + interference);

  return SINR;
}

//印RF SINR
void print_RF_SINR_Matrix(std::vector<std::vector<double>>  & RF_SINR_Matrix){
    std::cout<<"RF SINR Matrix as below : "<<std::endl;
    
    for(int i=0 ; i < RF_AP_Num ; i++){
       
        for(int j=0 ; j < UE_Num ; j++){
            std::cout<<RF_SINR_Matrix[i][j]<<" ";
        }
        
        std::cout<<std::endl;
    }

    std::cout<<std::endl;
}

//印VLC SINR
void print_VLC_SINR_Matrix(std::vector<std::vector<double>>  & VLC_SINR_Matrix){
    std::cout<<"VLC SINR Matrix as below : "<<std::endl;
    
    for(int i=0 ; i < VLC_AP_Num ; i++){
        
        for(int j=0 ; j < UE_Num ; j++){
            
            std::cout<<VLC_SINR_Matrix[i][j]<<" ";
        
        }
        
        std::cout<<std::endl;
    }

    std::cout<<std::endl;
}

//計算RF DataRate
void Calculate_RF_DataRate_Matrix(std::vector<std::vector<double>>  & RF_SINR_Matrix,std::vector<std::vector<double>> & RF_DataRate_Matrix){

  for(int i=0 ; i < RF_AP_Num ; i++){
    
    for(int j=0 ; j < UE_Num ; j++){
      
      RF_DataRate_Matrix[i][j] = RF_AP_Bandwidth * log2(1 + RF_SINR_Matrix[i][j]) ;
    
    }
  }
}

//計算VLC DataRate
void Calculate_VLC_DataRate_Matrix(std::vector<std::vector<double>> & VLC_SINR_Matrix,std::vector<std::vector<double>> & VLC_DataRate_Matrix){

  for(int i=0 ; i < VLC_AP_Num ; i++){
    
    for(int j=0 ; j < UE_Num ; j++){
      
      VLC_DataRate_Matrix[i][j] = 0.5 * VLC_AP_Bandwidth * log2(1 + VLC_SINR_Matrix[i][j]) ;
    
    }
  }
}

//印RF DataRate
void print_RF_DataRate_Matrix(std::vector<std::vector<double>>  & RF_DataRate_Matrix){
  
  std::cout<<"RF DataRate Matrix as below : "<<std::endl;
    
    for(int i=0 ; i < RF_AP_Num ; i++){
        
        for(int j=0 ; j < UE_Num ; j++){
            
            std::cout<<RF_DataRate_Matrix[i][j]<<" ";
        
        }
        
        std::cout<<std::endl;
    }

    std::cout<<std::endl;
}

//印VLC DataRate
void print_VLC_DataRate_Matrix(std::vector<std::vector<double>>  & VLC_DataRate_Matrix){
  
  std::cout<<"VLC DataRate Matrix as below : "<<std::endl;
    
    for(int i=0 ; i < VLC_AP_Num ; i++){
        
        for(int j=0 ; j < UE_Num ; j++){
            
            std::cout<<VLC_DataRate_Matrix[i][j]<<" ";
        
        }
        
        std::cout<<std::endl;
    }

    std::cout<<std::endl;
}