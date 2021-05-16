/*
 *  這份code單純用來印matrix值 
 *  以供debug之用
 * 
 * */
#include <iostream>
#include <fstream>
#include <string>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "print.h"



//印RF Channel gain
void print_RF_Channel_Gain_Matrix(std::vector<std::vector<double>>  & RF_Channel_Gain_Matrix){
    std::cout<<"RF Channel Gain Matrix as below : "<<std::endl;
    
    for(int i=0 ; i < RF_AP_Num ; i++){
       
        for(int j=0 ; j < UE_Num ; j++){
            std::cout<<RF_Channel_Gain_Matrix[i][j]<<" ";
        }
        
        std::cout<<std::endl;
    }

    std::cout<<std::endl;
}

//印VLC Channel gain
void print_VLC_Channel_Gain_Matrix(std::vector<std::vector<double>>  & VLC_Channel_Gain_Matrix){
    std::cout<<"VLC Channel Gain Matrix as below : "<<std::endl;
    
    for(int i=0 ; i < VLC_AP_Num ; i++){
        
        for(int j=0 ; j < UE_Num ; j++){
            
            std::cout<<VLC_Channel_Gain_Matrix[i][j]<<" ";
        
        }

        std::cout<<std::endl;
    }

    std::cout<<std::endl;
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


//印RF DataRate
void print_RF_DataRate_Matrix(std::vector<std::vector<double>>  & RF_DataRate_Matrix){
  
  std::cout<<"RF DataRate Matrix as below : "<<std::endl;
    
    for(int i=0 ; i < RF_AP_Num ; i++){
        
        for(int j=0 ; j < UE_Num ; j++){
            
            std::cout << RF_DataRate_Matrix[i][j] << " ";
            
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
            
            std::cout << VLC_DataRate_Matrix[i][j] << " ";
           
        }
        
        std::cout<<std::endl;
    }

    std::cout<<std::endl;
}

//印 Handover_Efficiency eta
void print_Handover_Efficiency_Matrix(std::vector<std::vector<double>> & Handover_Efficiency_Matrix){
     
    std::cout<<"Handover_Efficiency_Matrix as below : "<<std::endl;
    
    for(int i=0 ; i < RF_AP_Num + VLC_AP_Num ; i++){
        
        for(int j=0 ; j < RF_AP_Num + VLC_AP_Num ; j++){
            
            std::cout<<Handover_Efficiency_Matrix[i][j]<<" ";
        
        }
        
        std::cout<<std::endl;
    }

    std::cout<<std::endl;
}

//印 AP association matrix
void print_AP_Association_Matrix(std::vector<std::vector<int>>  & AP_Association_Matrix){
     
    std::cout<<"AP_Association_Matrix as below : "<<std::endl;
    
    for(int i=0 ; i < RF_AP_Num + VLC_AP_Num ; i++){
        
        for(int j=0 ; j < UE_Num ; j++){
            
            std::cout<<AP_Association_Matrix[i][j]<<" ";
        
        }
        
        std::cout<<std::endl;
    }

    std::cout<<std::endl;
}

//印 TDMA matrix
void print_TDMA_Matrix(std::vector<std::vector<double>> & TDMA_Matrix){
     
    std::cout<<"TDMA_Matrix as below : "<<std::endl;
    
    for(int i=0 ; i < RF_AP_Num + VLC_AP_Num ; i++){
        
        double sum = 0;
        for(int j=0 ; j < UE_Num + 1 ; j++){
            
            if(j>0)
                sum+=TDMA_Matrix[i][j];
            
            std::cout<<TDMA_Matrix[i][j]<<" ";
        
        }
        
        std::cout<<"total ="<<sum<<std::endl;
    }

    std::cout<<std::endl;
}