////////////////////////////////////////////////////////////////////////////////////////////
//// Q1 : 這份code在幹嘛？                                                                 ///
//// A1 : 我所提出的做法                                                                   ///
////                                                                                     ///                                                             
////////////////////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <set>
#include <map>
#include <iomanip>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "My_UE_Node.h"
#include "DynamicLB.h"
#include "ProposedMethod.h"
#include "Channel.h"
#include "print.h"

//執行Proposed的Dynamic Load balancing 
//即paper的Algo1
void Proposed_DynamicLB(
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
  std::vector<std::vector<double>> & TDMA_Matrix,                        
  std::vector<My_UE_Node> & myUElist)
{  

        

    //先做PreCalculation算出目前的各種SINR DataMatrix
    //PreCalculation兩種做法都一樣，故直接抓Benchmark來用，不用再重寫
    PreCalculation(RF_AP_Nodes , VLC_AP_Nodes , UE_Nodes,
    RF_Channel_Gain_Matrix , VLC_Channel_Gain_Matrix ,
    RF_SINR_Matrix , VLC_SINR_Matrix ,
    RF_DataRate_Matrix , VLC_DataRate_Matrix,
    Handover_Efficiency_Matrix);


    //Proposed method 
    //初始state採用state0的做法(沒有考慮handover)
    if(state == 0)
    
        Proposed_LB_state0(state , RF_DataRate_Matrix , VLC_DataRate_Matrix , Handover_Efficiency_Matrix , AP_Association_Matrix , TDMA_Matrix , myUElist);

    //若不是初始state則採用stateN(有考慮handover)
    else      
    
        Proposed_LB_stateN(state , RF_DataRate_Matrix , VLC_DataRate_Matrix , Handover_Efficiency_Matrix , AP_Association_Matrix , TDMA_Matrix , myUElist);
   

    
    
    std::cout<<std::setiosflags(std::ios::fixed);
    
    //實驗用 : 用來記錄AP平均服務的UE數
    for(int i=0 ; i < RF_AP_Num + VLC_AP_Num ; i++){
      
      int served = 0;
      for(int j = 0 ; j < UE_Num  ; j++){
          served += AP_Association_Matrix[i][j];
      }
      
      //std::cout<<"AP "<<i<<" served "<<served<<" UE in state"<<state<<std::endl;
      
      AP_Association_Matrix[i][UE_Num] += served ; 
    }

    #if DEBUG_MODE

    print_AP_Association_Matrix(AP_Association_Matrix);
    print_TDMA_Matrix(TDMA_Matrix);
    print_Handover_Efficiency_Matrix(Handover_Efficiency_Matrix);
    print_RF_DataRate_Matrix(RF_DataRate_Matrix);
    print_VLC_DataRate_Matrix(VLC_DataRate_Matrix);
    
    #endif
    
    sort(myUElist.begin(),myUElist.end(),[](My_UE_Node a,My_UE_Node b){return a.Get_Required_DataRate() > b.Get_Required_DataRate();});


    #if DEBUG_MODE
    
    // 印每個UE的歷史datarate
    for(int i = 0 ; i < myUElist.size() ; i++){
        std::cout<<"State : "<<state<<" UE :"<<myUElist[i].GetID()<<" Demand = "<<myUElist[i].Get_Required_DataRate() <<" linkde to ";
        (myUElist[i].Get_Now_Associated_AP() < RF_AP_Num ) ? (std::cout << "RF AP " << myUElist[i].Get_Now_Associated_AP()) : (std::cout<<"VLC AP "<< myUElist[i].Get_Now_Associated_AP() - RF_AP_Num );
        std::cout<<"  DateRate = "<<myUElist[i].Get_Throughput_History().back();
        std::cout<<"  DateRate / demand = "<<myUElist[i].Get_Throughput_History().back()/myUElist[i].Get_Required_DataRate()<<std::endl;
        
    }
    std::cout<<std::endl;
    #endif

    state ++;

}



//Proposed第一輪沒有AP handover
void Proposed_LB_state0(
  int & state,
  std::vector<std::vector<double>> & RF_DataRate_Matrix ,
  std::vector<std::vector<double>> & VLC_DataRate_Matrix,
  std::vector<std::vector<double>> & Handover_Efficiency_Matrix,
  std::vector<std::vector<int>>    & AP_Association_Matrix ,  
  std::vector<std::vector<double>> & TDMA_Matrix,                            
  std::vector<My_UE_Node> & myUElist)
{
    //clear AP association matrix （新一輪的開始要重新算）
    for(int i = 0 ; i < RF_AP_Num + VLC_AP_Num ; i++)
        for(int j = 0 ; j < UE_Num ; j++)
            AP_Association_Matrix[i][j] = 0;  

    

    //clear TDMA matrix （新一輪的開始要重新算）
    for(int i = 0 ; i < RF_AP_Num + VLC_AP_Num ; i++){
        
        //forall i ,ρ(i,0) = 1 代表每個AP目前都還沒被人連(故剩餘時間比例 = 1)
        TDMA_Matrix[i][0] = 1;  

        //目前都還沒有分時間
        for(int j = 1 ; j < UE_Num +1 ; j++)
            TDMA_Matrix[i][j] = 0;  

    }

    //並根據Required datarate做大到小排序
    //爲了讓demand大的先挑
    
    sort(myUElist.begin(),myUElist.end(),[](My_UE_Node a,My_UE_Node b){return a.Get_Required_DataRate() > b.Get_Required_DataRate();});
  
  
    ////////////////////////////////
    ////////////////////////////////
    ////                        ////
    ////      stage 1 HDFDA     ////
    ////                        ////
    ////////////////////////////////
    ////////////////////////////////
    
    //初步先配置盡可能滿足demand的量
    //若ap有多餘資源則stage2再分配
    
    /* AP selection Phase */

    //照demand 大->小順序,給每個UE挑
    for(int u = 0 ; u < myUElist.size() ; u++){

        //Avaliable_Resource_Matrix:即我論文提到所謂"max A-Q"的A Matrix
        std::vector<double> Avaliable_Resource_Matrix(RF_AP_Num + VLC_AP_Num);
        
        //每當一個UE挑完AP的後,A Matrix的情況都會變化，所以每次都要更新最新的情況
        for(int i = 0 ; i < RF_AP_Num ; i++) Avaliable_Resource_Matrix[i] = TDMA_Matrix[i][0];                                  //copy RF AP部分
        for(int i = 0 ; i < VLC_AP_Num ; i++) Avaliable_Resource_Matrix[i + RF_AP_Num] = TDMA_Matrix[i + RF_AP_Num][0];         //copy VLC AP部分

        /** 計算A-Q 這裏我們用A matrix扣掉相對應的Q **/
        //其意義：如果UE u 連到AP i且分得恰可滿足demand的資源後, AP i還會剩下多少residual resource
        //Note : 注意sort過後的index u未必是原本的UE Node ID
        
        //A-=Q
        for(int i = 0 ; i < RF_AP_Num ; i++) 
            Avaliable_Resource_Matrix[i] -= myUElist[u].Get_Required_DataRate() / RF_DataRate_Matrix[i][myUElist[u].GetID()];
        
        //A-=Q
        for(int i = 0 ; i < VLC_AP_Num ; i++) 
            Avaliable_Resource_Matrix[i + RF_AP_Num]-= myUElist[u].Get_Required_DataRate() / VLC_DataRate_Matrix[i][myUElist[u].GetID()];
    

        //接下來要選max A-Q的AP，因爲我剩的資源越多，越有機會服務更多人
        //用Beta記錄max A-Q的AP
        int Beta = 0;
        
        //max_residual記錄max A-Q的值
        double max_residual = Avaliable_Resource_Matrix[0];

         
        /* 從RF AP set找出max A-Q的RF AP 即論文的Eq 17 */
        
        //Beta_w 記錄剩餘資源最大的RF AP
        int Beta_w = 0;
            
        //記錄該資源剩餘量
        double max_RF_residual = Avaliable_Resource_Matrix[0];
            
        //遍歷RF AP更新最佳選擇
        for(int i = 0 ; i < RF_AP_Num ; i++)
            if(Avaliable_Resource_Matrix[i] > max_RF_residual){
                
                Beta_w = i;
                
                max_RF_residual = Avaliable_Resource_Matrix[i];
            }
        
        
        /* 從VLC AP set找出max A-Q的VLC AP 即論文的Eq 16 */

        //記錄剩餘資源最大的VLC AP
        int Beta_l = RF_AP_Num;
            
        //記錄該資源剩餘量
        double max_VLC_residual = Avaliable_Resource_Matrix[RF_AP_Num];
            
        //遍歷VLC AP更新最佳選擇
        for(int i = 0 ; i < VLC_AP_Num ; i++)
            if(Avaliable_Resource_Matrix[i + RF_AP_Num] > max_VLC_residual){
                
                Beta_l = i + RF_AP_Num;
                
                max_VLC_residual = Avaliable_Resource_Matrix[i + RF_AP_Num];
            }



        /*接下來要決定到底要連Beta_l還是Beta_w ： 看誰能提供的滿意度高就連誰，如果一樣大優先連lifi*/
        
        //連到Beta_l可提供的throughput
        double potential_throughput_of_Beta_l = TDMA_Matrix[Beta_l][0] * VLC_DataRate_Matrix[Beta_l - RF_AP_Num][myUElist[u].GetID()]; 

        //連到Beta_l可提供的滿意度,上限爲1
        double potential_satisfaction_of_Beta_l = potential_throughput_of_Beta_l / myUElist[u].Get_Required_DataRate();
        
        if(potential_satisfaction_of_Beta_l > 1) 
            potential_satisfaction_of_Beta_l = 1;
        
        //連到Beta_w可提供的throughput
        double potential_throughput_of_Beta_w = TDMA_Matrix[Beta_w][0] * RF_DataRate_Matrix[Beta_w][myUElist[u].GetID()]; 

        //連到Beta_w可提供的滿意度,上限爲1
        double potential_satisfaction_of_Beta_w = potential_throughput_of_Beta_w / myUElist[u].Get_Required_DataRate();
        
        if(potential_satisfaction_of_Beta_w > 1) 
            potential_satisfaction_of_Beta_w = 1;

        
        //找能提供較大滿意度的人連
        if(potential_satisfaction_of_Beta_l >= potential_satisfaction_of_Beta_w){
            
            Beta = Beta_l;
            
            max_residual = max_VLC_residual;
        }
        else{
            
            Beta = Beta_w;
            
            max_residual = max_RF_residual;
        }
      

        //連到chosen ap
        AP_Association_Matrix[Beta][myUElist[u].GetID()] = 1;
        
        //確定AP selection結果，記錄到myUElist中
        myUElist[u].Set_Now_Associated_AP(Beta);
        


        /* Resource Allocation Phase */

        //先盡可能配置滿足其demand的資源給ue u
        //如果無法滿足demand，則chosen ap剩的全給ue u
        
        //如果剩餘資源量爲負，則代表無法滿足ue u的demand.故剩下的資源全給ue u,導致分配後剩餘資源 = 0
        if(max_residual < 0)
            max_residual = 0;
        
        //這行的想法是 ρ(Beta,u) (要分配的資源量) =   ρ(Beta,0)(目前所剩資源量) - max_residual(分配後的剩餘資源量)
        TDMA_Matrix[Beta][myUElist[u].GetID()+1] = TDMA_Matrix[Beta][0] - max_residual;

       

        //在myUElist[u]中更新分得時間比例
        //Note : 此時的值未必是最終值，之後再做residual resource allocation時也許會有變化
        myUElist[u].Set_Time_Fraction(TDMA_Matrix[Beta][myUElist[u].GetID()+1]);
        
        
        //更新ρ(Beta,0)(目前所剩資源量) ，即要再扣掉剛剛分配的量，
        TDMA_Matrix[Beta][0] -= TDMA_Matrix[Beta][myUElist[u].GetID()+1];

        
    }

    #if DEBUG_MODE
        std::cout<<"TDMA Matrix after stage 1"<<std::endl;
        print_TDMA_Matrix(TDMA_Matrix);
    #endif

    ////////////////////////////////
    ////////////////////////////////
    ////                        ////
    ////    stage 2   RRA開始    ////
    ////                        ////
    ////////////////////////////////
    ////////////////////////////////
    
    //此階段要做的事：對於尚有資源未分配完的AP，將其資源分配給底下的UE，不要留下冗餘資源
    //有2種分配方法：
        //1. maximize throughtput : 把剩餘資源全部分給datarate最快的UE，目的是想要衝throughput
        //2. demand propotional ： 剩餘資源依照demand的比例來分,demand大的人分得的throughput多一點

    //先將UElist 依照NodeID 小到大 sort
    //這樣是爲了for loop的index和Matrix的index對應，例如：index u = k 即代表 NodeID = k
    sort(myUElist.begin(),myUElist.end(),[](My_UE_Node a,My_UE_Node b){return a.GetID() < b.GetID();});
    
    //用參數 RESIDUAL_RA_METHOD 來控制要採用何種分配方法
    
    //1. maximize throughtput(MST)
    #if (RESIDUAL_RA_METHOD == 1)
        MST(state , RF_DataRate_Matrix , VLC_DataRate_Matrix , Handover_Efficiency_Matrix, AP_Association_Matrix , TDMA_Matrix , myUElist);
    #endif

    //2. demand proportional(DP)
    #if (RESIDUAL_RA_METHOD == 2)
        DP(state , RF_DataRate_Matrix , VLC_DataRate_Matrix , Handover_Efficiency_Matrix, AP_Association_Matrix , TDMA_Matrix , myUElist);
    #endif 
    
    #if DEBUG_MODE
        std::cout<<"TDMA Matrix after stage 2"<<std::endl;
        print_TDMA_Matrix(TDMA_Matrix);
    #endif
    
    ////////////////////////////////
    ////////////////////////////////
    ////                        ////
    ////        RRA結束          ////
    ////                        ////
    ////////////////////////////////
    ////////////////////////////////

    //將最後RA結果存回myUElist
    //APS結果已在前面就更新了
    for(int ue_index = 0 ; ue_index < myUElist.size() ; ue_index ++)
    {
        //抓出此UE連到哪個AP
        int linked_ap = myUElist[ue_index].Get_Now_Associated_AP();
        
      
        
        //更新每個UE的time_fraction
        myUElist[ue_index].Set_Time_Fraction(TDMA_Matrix[linked_ap][ue_index+1]);

       
        
        //存這回合的achievable datarate
        //若連到WiFi
        if(linked_ap < RF_AP_Num)
            
            myUElist[ue_index].Add_Curr_iteration_Throughput(myUElist[ue_index].Get_Time_Fraction() * RF_DataRate_Matrix[linked_ap][ue_index]);
        
        //否則是連到LiFi
        else
                                                                                                                                
            myUElist[ue_index].Add_Curr_iteration_Throughput( myUElist[ue_index].Get_Time_Fraction() * VLC_DataRate_Matrix[linked_ap - RF_AP_Num][ue_index]);
        
        

        //update這一輪的滿意度
        //滿意度公式 : satisfication_level = min(1,achievable rate / required rate)
        double Curr_satisfication_level = myUElist[ue_index].Get_Throughput_History().back() / myUElist[ue_index].Get_Required_DataRate();
    
        if(Curr_satisfication_level > 1) 
            Curr_satisfication_level = 1;
    
        myUElist[ue_index].Add_Curr_satisfication_level(Curr_satisfication_level);

    }

}

//Proposed第二輪以後需要考慮AP handover的效應
//故code部分跟state0相比，只是計算throughput時要多乘上transmission efficiency而已，其他都一樣
void Proposed_LB_stateN( 
  int & state,
  std::vector<std::vector<double>> & RF_DataRate_Matrix ,
  std::vector<std::vector<double>> & VLC_DataRate_Matrix,
  std::vector<std::vector<double>> & Handover_Efficiency_Matrix,
  std::vector<std::vector<int>>    & AP_Association_Matrix ,  
  std::vector<std::vector<double>> & TDMA_Matrix,                            
  std::vector<My_UE_Node> & myUElist)
{
    

    //clear AP association matrix （新一輪的開始要重新算）
    for(int i = 0 ; i < RF_AP_Num + VLC_AP_Num ; i++)
        for(int j = 0 ; j < UE_Num ; j++)
            AP_Association_Matrix[i][j] = 0;  

    

    //clear TDMA matrix （新一輪的開始要重新算）
    for(int i = 0 ; i < RF_AP_Num + VLC_AP_Num ; i++){
        
        //forall i ,ρ(i,0) = 1 代表每個AP目前都還沒被人連(故剩餘時間比例 = 1)
        TDMA_Matrix[i][0] = 1;  

        //目前都還沒有分配時間
        for(int j = 1 ; j < UE_Num +1 ; j++)
            TDMA_Matrix[i][j] = 0;  

    }

    
    //並根據Required datarate做大到小排序
    //爲了讓demand大的先挑
    sort(myUElist.begin(),myUElist.end(),[](My_UE_Node a,My_UE_Node b){return a.Get_Required_DataRate() > b.Get_Required_DataRate();});

  
    ////////////////////////////////
    ////////////////////////////////
    ////                        ////
    ////   stage 1    HDFDA     ////
    ////                        ////
    ////////////////////////////////
    ////////////////////////////////
    
    //初步先配置盡可能滿足demand的量
    //若ap有多餘資源則stage2再分配
    
    /* AP selection Phase */

    //照demand 大->小順序,給每個UE挑
    for(int u = 0 ; u < myUElist.size() ; u++)
    {

        //先copy每個AP還剩下多少時間比例可以分給別人
        std::vector<double> Avaliable_Resource_Matrix(RF_AP_Num + VLC_AP_Num);
        for(int i = 0 ; i < RF_AP_Num ; i++) Avaliable_Resource_Matrix[i] = TDMA_Matrix[i][0];                                  //copy RF AP部分
        for(int i = 0 ; i < VLC_AP_Num ; i++) Avaliable_Resource_Matrix[i + RF_AP_Num] = TDMA_Matrix[i + RF_AP_Num][0];         //copy VLC AP部分

        //計算如果UE u 連到AP i且分得恰可滿足demand的資源後, AP i還會剩下多少residual resource
        //Note : 注意sort過後的index u未必是原本的UE Node ID
        //Note2 : 現在要考慮handover

        //算WiFi 部分
        for(int i = 0 ; i < RF_AP_Num ; i++) 
            
            Avaliable_Resource_Matrix[i] -= myUElist[u].Get_Required_DataRate() / 
            (Handover_Efficiency_Matrix[myUElist[u].Get_Now_Associated_AP()][i] * RF_DataRate_Matrix[i][myUElist[u].GetID()]);
                                                    //用get_now_associated_ap是因爲這一輪還沒更新AP
        
        //算LiFi部分
        for(int i = 0 ; i < VLC_AP_Num ; i++) 
            
            Avaliable_Resource_Matrix[i + RF_AP_Num]-= myUElist[u].Get_Required_DataRate() / 
            (Handover_Efficiency_Matrix[myUElist[u].Get_Now_Associated_AP()][i + RF_AP_Num] * VLC_DataRate_Matrix[i][myUElist[u].GetID()]);
    

         //接下來要選max A-Q的AP，因爲我剩的資源越多，越有機會服務更多人
        //用Beta記錄max A-Q的AP
        int Beta = 0;
        
        //max_residual記錄max A-Q的值
        double max_residual = Avaliable_Resource_Matrix[0];

         
        /* 從RF AP set找出max A-Q的RF AP 即論文的Eq 17 */
        
        //Beta_w 記錄剩餘資源最大的RF AP
        int Beta_w = 0;
            
        //記錄該資源剩餘量
        double max_RF_residual = Avaliable_Resource_Matrix[0];
            
        //遍歷RF AP更新最佳選擇
        for(int i = 0 ; i < RF_AP_Num ; i++)
            if(Avaliable_Resource_Matrix[i] > max_RF_residual){
                
                Beta_w = i;
                
                max_RF_residual = Avaliable_Resource_Matrix[i];
            }
        
        
        /* 從VLC AP set找出max A-Q的VLC AP 即論文的Eq 16 */

        //記錄剩餘資源最大的VLC AP
        int Beta_l = RF_AP_Num;
            
        //記錄該資源剩餘量
        double max_VLC_residual = Avaliable_Resource_Matrix[RF_AP_Num];
            
        //遍歷VLC AP更新最佳選擇
        for(int i = 0 ; i < VLC_AP_Num ; i++)
            if(Avaliable_Resource_Matrix[i + RF_AP_Num] > max_VLC_residual){
                
                Beta_l = i + RF_AP_Num;
                
                max_VLC_residual = Avaliable_Resource_Matrix[i + RF_AP_Num];
            }


        /*接下來要決定到底要連Beta_l還是Beta_w ： 看誰能提供的滿意度高就連誰，如果一樣大優先連lifi*/
        /*又因爲同一個UE demand一樣，所以比較滿意度等同於比throughput 我們這邊簡單一點 比throughput就好*/

        //連到Beta_l可提供的throughput
        double potential_throughput_of_Beta_l = TDMA_Matrix[Beta_l][0] * Handover_Efficiency_Matrix[myUElist[u].Get_Now_Associated_AP()][Beta_l] * VLC_DataRate_Matrix[Beta_l - RF_AP_Num][myUElist[u].GetID()];


        //連到Beta_l可提供的滿意度,上限爲1
        double potential_satisfaction_of_Beta_l = potential_throughput_of_Beta_l / myUElist[u].Get_Required_DataRate();
        
        if(potential_satisfaction_of_Beta_l > 1) 
            potential_satisfaction_of_Beta_l = 1;
        
        
        //連到Beta_w可提供的throughput
        double potential_throughput_of_Beta_w = TDMA_Matrix[Beta_w][0] * Handover_Efficiency_Matrix[myUElist[u].Get_Now_Associated_AP()][Beta_w] * RF_DataRate_Matrix[Beta_w][myUElist[u].GetID()];

        //連到Beta_w可提供的滿意度,上限爲1
        double potential_satisfaction_of_Beta_w = potential_throughput_of_Beta_w / myUElist[u].Get_Required_DataRate();
        
        if(potential_satisfaction_of_Beta_w > 1) 
            potential_satisfaction_of_Beta_w = 1;

        
        //找能提供較大滿意度的人連
        if(potential_satisfaction_of_Beta_l >= potential_satisfaction_of_Beta_w){
            
            Beta = Beta_l;
            
            max_residual = max_VLC_residual;
        }
        else{
            
            Beta = Beta_w;
            
            max_residual = max_RF_residual;
        }
        
        //確定AP selection結果
        //連到Beta
        AP_Association_Matrix[Beta][myUElist[u].GetID()] = 1;

        //更新：新的AP到myUElist
        myUElist[u].Set_Now_Associated_AP(Beta);
        
      
        /* Resource Allocation Phase */

        //先盡可能配置滿足其demand的資源給ue u
        //如果無法滿足demand，則chosen ap剩的全給ue u
        
        //如果剩餘資源量爲負，則代表無法滿足ue u的demand.故剩下的資源全給ue u,導致分配後剩餘資源 = 0
        if(max_residual < 0)
            max_residual = 0;
        
        //這行的想法是 ρ(Beta,u) (要分配的資源量) =   ρ(Beta,0)(目前所剩資源量) - max_residual(分配後的剩餘資源量)
        TDMA_Matrix[Beta][myUElist[u].GetID() +1] = TDMA_Matrix[Beta][0] - max_residual;
   
       
        //在myUElist[u]中更新分得時間比例
        //Note : 此時的值未必是最終值，之後再做residual resource allocation時也許會有變化
        myUElist[u].Set_Time_Fraction(TDMA_Matrix[Beta][myUElist[u].GetID()+1]);
        
        
        //更新ρ(Beta,0)(目前所剩資源量) ，即要再扣掉剛剛分配的量，
        TDMA_Matrix[Beta][0] -= TDMA_Matrix[Beta][myUElist[u].GetID()+1];

        
    } 

    ////////////////////////////////
    ////////////////////////////////
    ////                        ////
    ////   stage 2  RRA開始      ////
    ////                        ////
    ////////////////////////////////
    ////////////////////////////////

   

    //將UElist 依照NodeID 小到大 sort
    //這樣是爲了for loop的index和Matrix的index對應，例如：index u = k 即代表 NodeID = k
    sort(myUElist.begin(),myUElist.end(),[](My_UE_Node a,My_UE_Node b){return a.GetID() < b.GetID();});
    
    //用參數 RESIDUAL_RA_METHOD 來控制要採用何種分配方法
    
    //1. maximize throughtput(MST)
    #if (RESIDUAL_RA_METHOD == 1)
        MST(state , RF_DataRate_Matrix , VLC_DataRate_Matrix , Handover_Efficiency_Matrix, AP_Association_Matrix , TDMA_Matrix , myUElist);
    #endif

    //2. demand proportional(DP)
    #if (RESIDUAL_RA_METHOD == 2)
        DP(state , RF_DataRate_Matrix , VLC_DataRate_Matrix , Handover_Efficiency_Matrix, AP_Association_Matrix , TDMA_Matrix , myUElist);
    #endif 
    
    #if DEBUG_MODE
        std::cout<<"TDMA Matrix after stage 2"<<std::endl;
        print_TDMA_Matrix(TDMA_Matrix);
    #endif
   

    ////////////////////////////////
    ////////////////////////////////
    ////                        ////
    ////        RRA結束          ////
    ////                        ////
    ////////////////////////////////
    ////////////////////////////////

    //將最後RA結果存回myUElist
    //APS結果已在前面就更新了
    for(int ue_index = 0 ; ue_index < UE_Num ; ue_index ++)
    {
        //抓出此UE連到哪個AP
        int linked_ap = myUElist[ue_index].Get_Now_Associated_AP();
        
        //更新每個UE的time_fraction
        myUElist[ue_index].Set_Time_Fraction(TDMA_Matrix[linked_ap][ue_index+1]);

        //存這回合的achievable datarate
        //若連到WiFi
        if(linked_ap < RF_AP_Num)
      
            myUElist[ue_index].Add_Curr_iteration_Throughput(Handover_Efficiency_Matrix[myUElist[ue_index].Get_Prev_Associated_AP()][linked_ap] * myUElist[ue_index].Get_Time_Fraction() * RF_DataRate_Matrix[linked_ap][ue_index]);
        
         
        //否則連到LiFi
        else                                                                                                                                                                                                          // - RF_AP_Num是爲了對應到正確的VLC Matrix位置
     
            myUElist[ue_index].Add_Curr_iteration_Throughput(Handover_Efficiency_Matrix[myUElist[ue_index].Get_Prev_Associated_AP()][linked_ap] * myUElist[ue_index].Get_Time_Fraction() * VLC_DataRate_Matrix[linked_ap - RF_AP_Num][ue_index]);
       
      
        

        //update這一輪的滿意度
        //滿意度公式 : satisfication_level = min(1,achievable rate / required rate)
        double Curr_satisfication_level = myUElist[ue_index].Get_Throughput_History().back() / myUElist[ue_index].Get_Required_DataRate();
    
        if(Curr_satisfication_level > 1) 
            Curr_satisfication_level = 1;
    
        myUElist[ue_index].Add_Curr_satisfication_level(Curr_satisfication_level);

    }

}

/*
    MST：每個AP將所有剩餘資源分配給其底下effective data rate最高的人

    EX ： 假設 AP i 服務 UE 1,2,3
    ρ(i,0) 剩 0.1
    r(i,1) = 25Mb/s
    r(i,2) = 50Mb/s
    r(i,3) = 75Mb/s
    則：
    UE 3 額外獲得 0.1
*/
void MST( 
  int & state,
  std::vector<std::vector<double>> & RF_DataRate_Matrix ,
  std::vector<std::vector<double>> & VLC_DataRate_Matrix,
  std::vector<std::vector<double>> & Handover_Efficiency_Matrix,
  std::vector<std::vector<int>>    & AP_Association_Matrix ,  
  std::vector<std::vector<double>> & TDMA_Matrix,                            
  std::vector<My_UE_Node> & myUElist){

   
        
        //檢查所有WiFi AP
        for(int i = 0 ; i < RF_AP_Num ; i++)
        {
            
            //若有資源未分配完
            if(TDMA_Matrix[i][0] > 0)
            {
                

                //記錄有最大速度的UE
                int best_rate_user = -1 ;
                
                //記錄best_rate_user的速度
                double best_rate = 0 ;

                //檢查所有UE
                for(int ue = 0 ; ue < UE_Num  ; ue ++)
                {
                    
                    //若AP i有連到這個ue
                    if(AP_Association_Matrix[i][ue] == 1)
                    {

                        //初始state不必考慮handover
                        if(state == 0)
                        {
                            //則要檢查其速度是否比之前的best_rate_user高，若有利則更新
                            if(RF_DataRate_Matrix[i][ue] > best_rate)
                            {
                                best_rate_user = ue;
                                best_rate = RF_DataRate_Matrix[i][ue];
                            }

                        }
                        //非初始state則需要多考慮handover
                        else
                        {
                        
                            if(Handover_Efficiency_Matrix[myUElist[ue].Get_Prev_Associated_AP()][i] * RF_DataRate_Matrix[i][ue] > best_rate)
                            {
                                best_rate_user = ue;
                                best_rate = Handover_Efficiency_Matrix[myUElist[ue].Get_Prev_Associated_AP()][i] * RF_DataRate_Matrix[i][ue];
                            }
                        }
                       
                    }
                }
                
                
                //若的確存在這種user才做
                //Note : 因爲可能這個ap不服務任何人，這樣best_rate_user就會是 -1，但是實際上不存在user -1，如需要防止這樣情況發生
                if(best_rate_user > -1)
                {
                    //剩下的資源全數分給best_rate_user
                    //ue best_rate_user 對應到 在TDMA_Matrix的index是 best_rate_user+1，因爲TDMA_Matrix[i][0]另作它用，TDMA_Matrix[i][1]才是屬於 myUElist[0]
                    TDMA_Matrix[i][best_rate_user + 1] += TDMA_Matrix[i][0];

                    //在myUElist[best_rate_user]中更新分得時間比例
                    //Note : 此時的值即是最終值
                    myUElist[best_rate_user].Set_Time_Fraction(TDMA_Matrix[i][best_rate_user + 1]);

                    //分完就沒了
                    TDMA_Matrix[i][0] = 0;
                }
            }
        }
        
        //同理，檢查所有LiFi AP
        for(int i = 0 ; i < VLC_AP_Num ; i++)
        {
            
            //若有資源未分配完
            if(TDMA_Matrix[i + RF_AP_Num][0] > 0)
            {
                

                //記錄有最大速度的UE
                int best_rate_user = -1 ;
                
                //記錄best_rate_user的速度
                double best_rate = 0 ;

                //檢查所有UE
                for(int ue = 0 ; ue < UE_Num  ; ue ++)
                {
                    
                    //若AP i有連到這個ue
                    if(AP_Association_Matrix[i + RF_AP_Num][ue] == 1)
                    {
                        
                        if(state == 0)
                        {
                            //則要檢查其速度是否比之前的best_rate_user高，若有利則更新
                            if(VLC_DataRate_Matrix[i][ue] > best_rate)
                            {
                                best_rate_user = ue;
                                best_rate = VLC_DataRate_Matrix[i][ue];
                            }
                        }
                       else
                       {
                            if(Handover_Efficiency_Matrix[myUElist[ue].Get_Prev_Associated_AP()][i + RF_AP_Num] * VLC_DataRate_Matrix[i][ue] > best_rate)
                            {
                                best_rate_user = ue;
                                best_rate = Handover_Efficiency_Matrix[myUElist[ue].Get_Prev_Associated_AP()][i + RF_AP_Num] * VLC_DataRate_Matrix[i][ue];
                            }
                       }
                    }
                }
                
                //若的確存在這種user才做
                //Note : 因爲可能這個ap不服務任何人，這樣best_rate_user就會是-1，但是實際上不存在user -1，如需要防止這樣情況發生
                if(best_rate_user > -1)
                {
                    //剩下的資源全數分給best_rate_user
                    TDMA_Matrix[i + RF_AP_Num][best_rate_user + 1] += TDMA_Matrix[i + RF_AP_Num][0];
                    
                    //在myUElist[best_rate_user]中更新分得時間比例
                    //Note : 此時的值即是最終值
                    myUElist[best_rate_user].Set_Time_Fraction(TDMA_Matrix[i + RF_AP_Num][best_rate_user + 1]);
                    
                    //分完就沒了
                    TDMA_Matrix[i + RF_AP_Num][0] = 0;
                }
            }
        }
    
}
/*
    根據demand比例分配剩餘資源
    例子可以看我論文
*/

void DP( 
  int & state,
  std::vector<std::vector<double>> & RF_DataRate_Matrix ,
  std::vector<std::vector<double>> & VLC_DataRate_Matrix,
  std::vector<std::vector<double>> & Handover_Efficiency_Matrix,
  std::vector<std::vector<int>>    & AP_Association_Matrix ,  
  std::vector<std::vector<double>> & TDMA_Matrix,                            
  std::vector<My_UE_Node> & myUElist){
        

        //檢查所有WiFi AP
        for(int i = 0 ; i < RF_AP_Num ; i++)
        {
            
            //若有資源未分配完
            if(TDMA_Matrix[i][0] > 0)
            {
                
                //以demand/effective data rate爲權重分配剩餘資原
                //記錄分母 = 權重之總和
                double Denominator = 0.0;
                
                std::map<int,double> WeightofUE;

                //檢查所有UE
                for(int ue = 0 ; ue < UE_Num  ; ue ++)
                {
                    
                    //若AP i有連到這個ue
                    if(AP_Association_Matrix[i][ue] == 1)
                    {

                        //初始state不必考慮handover
                        if(state == 0)
                        {
                            
                                //則更新分母
                                Denominator += myUElist[ue].Get_Required_DataRate() / RF_DataRate_Matrix[i][ue] ;
                            
                                //更新該ue的分子
                                WeightofUE[ue] = myUElist[ue].Get_Required_DataRate() / RF_DataRate_Matrix[i][ue] ;
                            
                        }
                        //非初始state則需要多考慮handover
                        else
                        {
                           
                                //更新分母
                                Denominator += (myUElist[ue].Get_Required_DataRate() / Handover_Efficiency_Matrix[myUElist[ue].Get_Prev_Associated_AP()][i]  * RF_DataRate_Matrix[i][ue]);
    
                                //更新該ue的分子
                                WeightofUE[ue] = (myUElist[ue].Get_Required_DataRate() / Handover_Efficiency_Matrix[myUElist[ue].Get_Prev_Associated_AP()][i]  * RF_DataRate_Matrix[i][ue]);

                        }

                    
                    }
                }

                
                //通過WeightofUE取得每個UE的分子，再處剛剛算出來的分母，即爲每個UE可額外分得的資源量
                std::map<int, double>::iterator it;
                for(it = WeightofUE.begin() ; it != WeightofUE.end() ; it++)
		        {    

                    
                    TDMA_Matrix[i][it->first + 1] += it->second / Denominator * TDMA_Matrix[i][0];
                                        
                    //在myUElist[it->first]中更新分得時間比例
                    //Note : 此時的值即是最終值
                    myUElist[it->first].Set_Time_Fraction(TDMA_Matrix[i][it->first + 1]);

                }
                
                //剩餘的資源皆分完
                TDMA_Matrix[i][0] = 0;
            }
        }
        
        //同理，檢查所有LiFi AP
        for(int i = 0 ; i < VLC_AP_Num ; i++)
        {
            
            //若有資源未分配完
            if(TDMA_Matrix[i + RF_AP_Num][0] > 0)
            {
                
                double Denominator = 0.0;
                
                std::map<int,double> WeightofUE;

                //檢查所有UE
                for(int ue =  0; ue < UE_Num  ; ue ++)
                {
                    
                    //若AP i有連到這個ue
                    if(AP_Association_Matrix[i + RF_AP_Num][ue] == 1)
                    {

                        //初始state不必考慮handover
                        if(state == 0)
                        {

                                //更新分母
                                Denominator +=  myUElist[ue].Get_Required_DataRate() / VLC_DataRate_Matrix[i][ue];
                                
                                //更新該ue的分子
                                WeightofUE[ue] = myUElist[ue].Get_Required_DataRate() / VLC_DataRate_Matrix[i][ue];
                           
                        }
                        //非初始state則需要多考慮handover
                        else
                        {
                                //更新分母
                                Denominator += myUElist[ue].Get_Required_DataRate() / (Handover_Efficiency_Matrix[myUElist[ue].Get_Prev_Associated_AP()][i + RF_AP_Num] * VLC_DataRate_Matrix[i][ue]);
        
                                //更新該ue的分子
                                WeightofUE[ue] = myUElist[ue].Get_Required_DataRate() / (Handover_Efficiency_Matrix[myUElist[ue].Get_Prev_Associated_AP()][i + RF_AP_Num] * VLC_DataRate_Matrix[i][ue]);

                        }
                       
                    }

                     
                }
                //通過WeightofUE取得每個UE的分子，再處剛剛算出來的分母，即爲每個UE可額外分得的資源量
                std::map<int, double>::iterator it;
                for(it = WeightofUE.begin() ; it != WeightofUE.end() ; it++)
		        {    
                    //注意在TDMA_Matrix中，UEindex從1開始，故it->first要+1
                    TDMA_Matrix[i + RF_AP_Num][it->first + 1] += it->second / Denominator * TDMA_Matrix[i + RF_AP_Num][0];
                     
                    //在myUElist[it->first]中更新分得時間比例
                    //Note : 此時的值即是最終值
                    myUElist[it->first].Set_Time_Fraction(TDMA_Matrix[i + RF_AP_Num][it->first + 1]);
                }
                
                //剩餘的資源皆分完
                TDMA_Matrix[i + RF_AP_Num][0] = 0;
                
            }
        }

}