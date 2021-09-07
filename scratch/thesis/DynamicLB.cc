////////////////////////////////////////////////////////////////////////////////////////////
//// Q : 這份code在幹嘛？                                                                  ///
//// A : 這是我的實驗對照組ref 11的做法,姑且叫他benchmark or DLB                              ///
////////////////////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <set>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "My_UE_Node.h"
#include "DynamicLB.h"
#include "Channel.h"
#include "print.h"

//執行Benchmark的Dynamic Load balancing 
//即我論文ref11的Algo1
void Benchmark_DynamicLB(
  int & state,
  NodeContainer  & RF_AP_Nodes , 
  NodeContainer  & VLC_AP_Nodes , 
  NodeContainer  & UE_Nodes,
  std::vector<std::vector<double>> & RF_Channel_Gain_Matrix , 
  std::vector<std::vector<double>> & VLC_Channel_Gain_Matrix ,
  std::vector<std::vector<double>> & RF_SINR_Matrix ,
  std::vector<std::vector<double>> & VLC_SINR_Matrix ,
  std::vector<std::vector<double>> & RF_DataRate_Matrix ,
  std::vector<std::vector<double>> & VLC_DataRate_Matrix,
  std::vector<std::vector<double>> & Handover_Efficiency_Matrix ,
  std::vector<std::vector<int>>    & AP_Association_Matrix ,                          
  std::vector<My_UE_Node> & myUElist)
{  

    //先做PreCalculation算出目前的各種SINR DataMatrix
    PreCalculation(RF_AP_Nodes , VLC_AP_Nodes , UE_Nodes,
    RF_Channel_Gain_Matrix , VLC_Channel_Gain_Matrix ,
    RF_SINR_Matrix , VLC_SINR_Matrix ,
    RF_DataRate_Matrix , VLC_DataRate_Matrix,
    Handover_Efficiency_Matrix);

    
    //初始先用RSS配置
    if(state == 0)
      
      Benchmark_RSS_state0(RF_DataRate_Matrix , VLC_DataRate_Matrix , Handover_Efficiency_Matrix , AP_Association_Matrix , myUElist);

    //若不是初始state則採用paper的allocation
    else
      
      //即Ref11的Algo2
      Benchmark_LB_stateN(RF_DataRate_Matrix , VLC_DataRate_Matrix , Handover_Efficiency_Matrix , AP_Association_Matrix , myUElist);

   


    #if DEBUG_MODE
    //實驗用 : 用來記錄每個AP平均服務的UE數來觀察不同AP的loading
    for(int i=0 ; i < RF_AP_Num + VLC_AP_Num ; i++){
      
      int served = 0;
      for(int j = 0 ; j < UE_Num  ; j++){
          served += AP_Association_Matrix[i][j];
      }
      //std::cout<<"AP "<<i<<" served "<<served<<" UE in state"<<state<<std::endl;
      AP_Association_Matrix[i][UE_Num] += served ; 
    }
    //print_AP_Association_Matrix(AP_Association_Matrix);
    #endif

    #if DEBUG_MODE
    //DEBUG用:印每個UE的歷史datarate
    for(int i = 0 ; i < myUElist.size() ; i++){
        std::cout<<"id:"<<myUElist[i].GetID()<<" History DateRate =";
        std::vector<double> history = myUElist[i].Get_Throughput_History();
        for(int j=0 ;j < state ;j++)
          std::cout<<history[j]<<" ";
        std::cout<<std::endl;
    }
    std::cout<<std::endl;
    #endif
    

    //爲下一個iteration做準備
    state ++;

}




//執行PreCalulation，以供後續演算法進行決策
//要計算的項目有
//1. Channel gain Matrix (RF/VLC)
//2. SINR Matrix (RF/VLC)
//3. Achievable DataRate of pair<AP i,UE j> 說明 ： 考慮的是 AP i 只服務 UE j，給其全部resource的理性值
//4. handover efficiency matrix 
void PreCalculation(
  NodeContainer  & RF_AP_Nodes , 
  NodeContainer  & VLC_AP_Nodes , 
  NodeContainer  & UE_Nodes,
  std::vector<std::vector<double>> & RF_Channel_Gain_Matrix , 
  std::vector<std::vector<double>> & VLC_Channel_Gain_Matrix ,
  std::vector<std::vector<double>> & RF_SINR_Matrix ,
  std::vector<std::vector<double>> & VLC_SINR_Matrix ,
  std::vector<std::vector<double>> & RF_DataRate_Matrix ,
  std::vector<std::vector<double>> & VLC_DataRate_Matrix,
  std::vector<std::vector<double>> & Handover_Efficiency_Matrix)
{
  
  /** Algo1 , Line3 **/
  //算RF/VLC的Channel gain
  Calculate_RF_Channel_Gain_Matrix(RF_AP_Nodes,UE_Nodes,RF_Channel_Gain_Matrix);
  Calculate_VLC_Channel_Gain_Matrix(VLC_AP_Nodes,UE_Nodes,VLC_Channel_Gain_Matrix);
  
  #if DEBUG_MODE
    print_RF_Channel_Gain_Matrix(RF_Channel_Gain_Matrix);
    print_VLC_Channel_Gain_Matrix(VLC_Channel_Gain_Matrix);
  #endif
  

  //算RF/VLC的SINR
  Calculate_RF_SINR_Matrix(RF_Channel_Gain_Matrix,RF_SINR_Matrix);
  Calculate_VLC_SINR_Matrix(VLC_Channel_Gain_Matrix,VLC_SINR_Matrix);
  
  #if DEBUG_MODE
    print_RF_SINR_Matrix(RF_SINR_Matrix);
    print_VLC_SINR_Matrix(VLC_SINR_Matrix);
  #endif

  /** Algo1 , Line4 **/
  //算RF/VLC的DataRate
  Calculate_RF_DataRate_Matrix(RF_SINR_Matrix , RF_DataRate_Matrix);
  Calculate_VLC_DataRate_Matrix(VLC_SINR_Matrix , VLC_DataRate_Matrix);
  
  #if DEBUG_MODE
    print_RF_DataRate_Matrix(RF_DataRate_Matrix);
    print_VLC_DataRate_Matrix(VLC_DataRate_Matrix);
  #endif

  /** Algo1 , Line5 **/
  //算Handover_Efficiency_Matrix
  Calculate_Handover_Efficiency_Matrix(Handover_Efficiency_Matrix);
  
  #if DEBUG_MODE
    print_Handover_Efficiency_Matrix(Handover_Efficiency_Matrix);
  #endif
}


//首輪採取RSS
void Benchmark_RSS_state0( 
  std::vector<std::vector<double>> & RF_DataRate_Matrix ,
  std::vector<std::vector<double>> & VLC_DataRate_Matrix,
  std::vector<std::vector<double>> & Handover_Efficiency_Matrix,
  std::vector<std::vector<int>>    & AP_Association_Matrix ,                          
  std::vector<My_UE_Node> & myUElist){
  
  
  //beta_u記錄AP selection的結果
  //Algorithm2有分beta_1_u for LiFi AP,beta_2_u for WiFi AP
  //Algorithm2做法:每個UE都先連最快的LiFi，平分後速度不達標的UE再連最快的WiFi
  //因此不一定要分beta1 beta2,統一用beta表示，先連LiFi,然後再對速度不達標的UE們改連WiFi
  std::vector<int> beta_u (UE_Num,-1) ;                           //記錄每個UE的最快AP，有可能是LiFi or WiFi(看最後被踢到哪種AP set)
  std::vector<double> potentialRate (UE_Num,0) ;                  //承上,記錄beta_u能提供多少速度，只作爲是否要踢去連WiFi的參考依據
  std::vector<int> served_UE_Num(RF_AP_Num + VLC_AP_Num , 0);     //記錄每個AP有多少人連,爲了到時候平分資源用
  
  //////////////////////////////////
  //////////////////////////////////
  ////                          ////
  ////     AP selection Phase   ////
  ////                          ////
  //////////////////////////////////
  //////////////////////////////////

  //Step 1:篇歷所有UE,先幫每個UE找到最快的LiFi AP
  for(int ue_index = 0 ; ue_index < myUElist.size() ; ue_index ++){

    //檢查所有VLC AP
    for(int i = 0 ; i < VLC_AP_Num ; i ++){
      
      //newRate ： 求得如果連到此VLC AP，則可以有多少速度 
      double newRate =  VLC_DataRate_Matrix[i][ue_index] ;

      //若有利則更新
      if( newRate > potentialRate[ue_index]){
        
        potentialRate[ue_index] = newRate;
        
        beta_u[ue_index] = i + RF_AP_Num;
      }
    }
  }

  //計算每個LiFi AP各服務多少UE
  for(int ue_index = 0 ; ue_index < myUElist.size() ; ue_index ++)
    
      served_UE_Num[beta_u[ue_index]]++ ;

  

  //Step 2:把平分後速度不達標(date rate < threshold)的UE改連至最快的RF AP
  for(int ue_index = 0 ; ue_index < myUElist.size() ; ue_index ++)
  {
    
    //資源平分，所以potential rate要除以該AP服務的UE數
    potentialRate[ue_index] = potentialRate[ue_index] / served_UE_Num[beta_u[ue_index]] ;
    
    //若平分後不達threshold則改連WiFi
    if(potentialRate[ue_index] < threshold)
    {
    
      //清除此UE在先前檢查LiFi AP的結果
      beta_u[ue_index] = -1 ;
      potentialRate[ue_index] = 0;
      
      //一樣也是挑最快的RF_AP
      for(int i = 0 ; i < RF_AP_Num ; i ++)
      {
      
        double newRate =  RF_DataRate_Matrix[i][ue_index];

        //若有利則更新
        if( newRate > potentialRate[ue_index])
        {
          potentialRate[ue_index] = newRate;
          beta_u[ue_index] = i;
        }
      }

    } 
  }
  
  /** AP selection 完成 保存結果 **/
  Benchmark_Update_APSelection_Result( myUElist , beta_u , served_UE_Num , AP_Association_Matrix);
 
    

  //////////////////////////////////
  //////////////////////////////////
  ////                          ////
  ////  Resource Allocation     ////
  ////                          ////
  //////////////////////////////////
  //////////////////////////////////

  //開始做resource allocation
  //分配時間比例(VLC)或是頻寬(RF)
  //Note : 根據eq.16 進行計算

  //這裏的Throughtput存的是最後能達到的DataRate
  //之前的potentialRate比較像是判斷能不能連LiFi的標準而已
  //最後真的理想值還是要再重算一次
  std::vector<double> Throughput (UE_Num,0) ;  
  
  //Resource allocation Method
  for(int ue_index = 0 ; ue_index < myUElist.size() ; ue_index ++)
  {

      //第一輪沒有過去歷史平均的概念
      //所以我選擇 state0採用簡單的平分處理
      
      if(beta_u[ue_index] < RF_AP_Num)

        Throughput[ue_index] = RF_DataRate_Matrix[beta_u[ue_index]][ue_index] / served_UE_Num[beta_u[ue_index]] ;
  

        
      else

        Throughput[ue_index] = VLC_DataRate_Matrix[beta_u[ue_index] - RF_AP_Num ][ue_index] / served_UE_Num[beta_u[ue_index]] ;
      
  }
  

  Benchmark_Update_RA_Result(myUElist , Throughput) ;

}

//第二輪 ～ 第N輪的LB
//基本上程式結構和state0一樣,但是state1以後需要考慮handover
//故跟data rate有關系的地方都必須多乘上handover的trasmission efficiency
void Benchmark_LB_stateN( 
  std::vector<std::vector<double>> & RF_DataRate_Matrix ,
  std::vector<std::vector<double>> & VLC_DataRate_Matrix,
  std::vector<std::vector<double>> & Handover_Efficiency_Matrix,
  std::vector<std::vector<int>>    & AP_Association_Matrix ,                          
  std::vector<My_UE_Node> & myUElist)
{
  //此段宣告與state0時相同，故省略注解
  std::vector<int> beta_u (UE_Num,-1) ;                           
  std::vector<double> potentialRate (UE_Num,0) ;                  
  std::vector<int> served_UE_Num(RF_AP_Num + VLC_AP_Num , 0);     
  
  //////////////////////////////////
  //////////////////////////////////
  ////                          ////
  ////     AP selection         ////
  ////                          ////
  //////////////////////////////////
  //////////////////////////////////

  //Step 1:篇歷所有UE,先幫每個UE找到最快的LiFi AP
  for(int ue_index = 0 ; ue_index < myUElist.size() ; ue_index ++){

    //記錄前一輪AP是誰，用Now是因爲前一輪AP尚未更新，還儲存在Now_Associated_AP裡
    //做完AP selection後才會更新到prev_associated_ap裏面
    int prev_AP = myUElist[ue_index].Get_Now_Associated_AP();

    //檢查所有VLC AP
    for(int i = 0 ; i < VLC_AP_Num ; i ++){
      
                        //注意這裡比起state0多乘這項
      double newRate =  Handover_Efficiency_Matrix[prev_AP][i + RF_AP_Num] * VLC_DataRate_Matrix[i][ue_index] ;

      //若有利則更新
      if( newRate > potentialRate[ue_index]){
        
        potentialRate[ue_index] = newRate;
        
        beta_u[ue_index] = i + RF_AP_Num;
      }
    }
  }

  //計算每個LiFi AP各服務多少UE
  for(int ue_index = 0 ; ue_index < myUElist.size() ; ue_index ++)
    
      served_UE_Num[beta_u[ue_index]]++ ;

  

  //Step 2:把平分後速度不達標(date rate < threshold)的UE改連至最快的RF AP
  for(int ue_index = 0 ; ue_index < myUElist.size() ; ue_index ++)
  {
    
    //資源平分，所以potential rate要除以該AP服務的UE數
    potentialRate[ue_index] = potentialRate[ue_index] / served_UE_Num[beta_u[ue_index]] ;
    
    //若平分後不達threshold則改連WiFi
    if(potentialRate[ue_index] < threshold)
    {
    
      beta_u[ue_index] = -1 ;

      potentialRate[ue_index] = 0;
      
      int prev_AP = myUElist[ue_index].Get_Now_Associated_AP();

      //一樣也是挑最快的RF_AP
      for(int i = 0 ; i < RF_AP_Num ; i ++)
      {
      
        double newRate =  Handover_Efficiency_Matrix[prev_AP][i] * RF_DataRate_Matrix[i][ue_index];

        //若有利則更新
        if( newRate > potentialRate[ue_index])
        {
          potentialRate[ue_index] = newRate;
          beta_u[ue_index] = i;
        }
      }

    } 
  }
  
  /** AP selection 完成 保存結果 **/
  Benchmark_Update_APSelection_Result( myUElist , beta_u , served_UE_Num , AP_Association_Matrix);
 
    

  //////////////////////////////////
  //////////////////////////////////
  ////                          ////
  ////  Resource Allocation     ////
  ////                          ////
  //////////////////////////////////
  //////////////////////////////////

  
  //開始做resource allocation
  //分配時間比例(VLC)或是頻寬(RF)
  //Note : 根據eq.16 進行計算

  //Throughput 存的是 最後能達到的DataRate
  //之前的potentialRate比較像是判斷能不能連LiFi的標準而已
  //最後真的理想值還是要再重算一次
  std::vector<double> Throughput (UE_Num,0) ;  
  

  /** RF Bandwidth Allocation 前置作業 **/ 
  //這個資訊等等RF Bandwidth allocation會用到
  double Reciprocal_Sum_Of_RF_UE = 0 ;  //所有連到WiFi的user之 avg datarate 的倒數和
  for(int ue_index = 0 ; ue_index < myUElist.size() ; ue_index ++)
      if(beta_u[ue_index] < RF_AP_Num)
        Reciprocal_Sum_Of_RF_UE += (1 / myUElist[ue_index].Get_Avg_Throughput());


  /** Algo2 , Line11 **/
  for(int ue_index = 0 ; ue_index < myUElist.size() ; ue_index ++){

    //先Get到這個UE上一輪連哪個AP(RF AP index = [0 , RF_AP_Num - 1] / VLC AP index = [RF_AP_Num , RF_AP_Num + VLC_AP_Num - 1])
    //這裏不用Get_Now_Associated_AP(),是因爲剛剛第229行已經update過AP了
    //現在要找上一輪AP要call Prev才能得到
    int prev_AP = myUElist[ue_index].Get_Prev_Associated_AP();

    //如果是連WiFi AP
    if(beta_u[ue_index] < RF_AP_Num){
        
      //需要做bandwidth allocation
      
      //by eq.14
      double bw = (1 / myUElist[ue_index].Get_Avg_Throughput()) / Reciprocal_Sum_Of_RF_UE ;

      //by eq.16
      Throughput[ue_index] = bw * Handover_Efficiency_Matrix[prev_AP][beta_u[ue_index]] * RF_DataRate_Matrix[beta_u[ue_index]][ue_index] ;
    }
    

    //否則就是連到LiFi AP
    else{
     
        // 可能會有換手發生，故速度需考慮Handover_Efficiency eta
        Throughput[ue_index] = Handover_Efficiency_Matrix[prev_AP][beta_u[ue_index]] * VLC_DataRate_Matrix[beta_u[ue_index]- RF_AP_Num][ue_index];

        //resource 採等分給服務的UE
        //所以服務多少UE就除以幾
        Throughput[ue_index] =  Throughput[ue_index] / served_UE_Num[myUElist[ue_index].Get_Now_Associated_AP()];
    }
  }

  Benchmark_Update_RA_Result(myUElist , Throughput) ;
}


/**
  完成AP Selection後
  
  1.更新所有UE的 : 
    1. 此輪AP
    2. 上輪AP 
  
  2. 依照最新的APS結果，更新所有AP的服務UE數
**/
void Benchmark_Update_APSelection_Result( std::vector<My_UE_Node> & myUElist , std::vector<int> & beta_u , std::vector<int> & served_UE_Num , std::vector<std::vector<int>>  & AP_Association_Matrix )
{
  /** 先將新的AP到My_UE_Node中 **/
  for(int ue_index = 0 ; ue_index < myUElist.size() ; ue_index ++)
    
    myUElist[ue_index].Set_Now_Associated_AP(beta_u[ue_index]);
  
   
  /** 因爲可能有人被踢去WiFi，所以要重算served_UE_Num，以供等等Resource Allocation用 **/
  //清0
  for(int i = 0 ; i < RF_AP_Num + VLC_AP_Num ; i ++)  served_UE_Num[i] = 0;
  //重算
  for(int ue_index = 0 ; ue_index < myUElist.size() ; ue_index ++)  served_UE_Num[myUElist[ue_index].Get_Now_Associated_AP()]++;


  /** 更新AP_Association_Matrix , 對benchmark來說沒用處，但proposed Algo會需要用到 **/
  //清0
  for(int i = 0 ; i < RF_AP_Num + VLC_AP_Num ; i ++) 
    for(int j = 0 ; j < UE_Num ; j ++)
        AP_Association_Matrix[i][j] = 0 ;

  //重算
  for(int ue_index = 0 ; ue_index < myUElist.size() ; ue_index ++)  
    AP_Association_Matrix[myUElist[ue_index].Get_Now_Associated_AP()][ue_index] = 1 ;

}

//完成Resource Allocation後
//更新所有UE的 : 
//1. 此輪achievable datarate
//2. 歷史平均avg data rate
//3. 此輪滿意度
void Benchmark_Update_RA_Result( std::vector<My_UE_Node> & myUElist , std::vector<double> & Throughput)
{
  
  //Resource allocation結束
  //將結果存回myUElist
  for(int ue_index = 0 ; ue_index < myUElist.size() ; ue_index ++)
  {
    
    //存這回合的achievable datarate
    //這行會順便update avg datarate 即 Algo2 , Line12
    // std::cout<<"UE "<<ue_index<<"Required "<<myUElist[ue_index].Get_Required_DataRate()<<" obtain datarate :"<<Throughput[ue_index]<<std::endl;
    myUElist[ue_index].Add_Curr_iteration_Throughput(Throughput[ue_index]);


    //update這一輪的滿意度
    //滿意度公式 : satisfication_level = min(1,achievable rate / required rate)
    double Curr_satisfication_level = Throughput[ue_index] / myUElist[ue_index].Get_Required_DataRate();
    
    if(Curr_satisfication_level > 1) 
      Curr_satisfication_level = 1;
    
    myUElist[ue_index].Add_Curr_satisfication_level(Curr_satisfication_level);

  }

}