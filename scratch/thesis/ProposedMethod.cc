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
      
      std::cout<<"AP "<<i<<" served "<<served<<" UE in state"<<state<<std::endl;
      
      AP_Association_Matrix[i][UE_Num] += served ; 
    }

    print_AP_Association_Matrix(AP_Association_Matrix);
    print_TDMA_Matrix(TDMA_Matrix);
    // // print_Handover_Efficiency_Matrix(Handover_Efficiency_Matrix);
    print_RF_DataRate_Matrix(RF_DataRate_Matrix);
    print_VLC_DataRate_Matrix(VLC_DataRate_Matrix);
    
    
    sort(myUElist.begin(),myUElist.end(),[](My_UE_Node a,My_UE_Node b){return a.Get_Required_DataRate() > b.Get_Required_DataRate();});

    // 印每個UE的歷史datarate
    for(int i = 0 ; i < myUElist.size() ; i++){
        std::cout<<"State : "<<state<<" UE :"<<myUElist[i].GetID()<<" Demand = "<<myUElist[i].Get_Required_DataRate() <<" linkde to ";
        (myUElist[i].Get_Now_Associated_AP() < RF_AP_Num ) ? (std::cout << "RF AP " << myUElist[i].Get_Now_Associated_AP()) : (std::cout<<"VLC AP "<< myUElist[i].Get_Now_Associated_AP() - RF_AP_Num );
        std::cout<<"  DateRate = "<<myUElist[i].Get_Achievable_DataRate_History().back()<<std::endl;
        
    }

    std::cout<<std::endl;

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

    //並根據Required datarate做小到大排序
    //爲了讓demand小的先挑
    
    sort(myUElist.begin(),myUElist.end(),[](My_UE_Node a,My_UE_Node b){return a.Get_Required_DataRate() > b.Get_Required_DataRate();});
  
  
    ////////////////////////////////
    ////////////////////////////////
    ////                        ////
    ////        stage 1         ////
    ////                        ////
    ////////////////////////////////
    ////////////////////////////////
    
    //初步先配置盡可能滿足demand的量
    //若ap有多餘資源則stage2再分配
    
    //照demand 大->小順序,給每個UE挑
    for(int u = 0 ; u < myUElist.size() ; u++){

        //先copy每個AP還剩下多少時間比例可以分給別人
        std::vector<double> copy_ap_residual_resource(RF_AP_Num + VLC_AP_Num);
        for(int i = 0 ; i < RF_AP_Num ; i++) copy_ap_residual_resource[i] = TDMA_Matrix[i][0];                                  //copy RF AP部分
        for(int i = 0 ; i < VLC_AP_Num ; i++) copy_ap_residual_resource[i + RF_AP_Num] = TDMA_Matrix[i + RF_AP_Num][0];         //copy VLC AP部分

        //計算如果UE u 連到AP i且分得恰可滿足demand的資源後, AP i還會剩下多少residual resource
        //Note : 注意sort過後的index u未必是原本的UE Node ID
        for(int i = 0 ; i < RF_AP_Num ; i++) copy_ap_residual_resource[i] -= myUElist[u].Get_Required_DataRate() / RF_DataRate_Matrix[i][myUElist[u].GetID()];
        for(int i = 0 ; i < VLC_AP_Num ; i++) copy_ap_residual_resource[i + RF_AP_Num]-= myUElist[u].Get_Required_DataRate() / VLC_DataRate_Matrix[i][myUElist[u].GetID()];
    
        //選擇可以剩餘最多資源的AP，剩的資源越多，越有機會服務更多人
        int chosen_ap = 0;
        double max_residual = copy_ap_residual_resource[0];
        #if 0 
        for(int i = 0 ; i < RF_AP_Num + VLC_AP_Num ; i++)
            if(copy_ap_residual_resource[i] > max_residual){
                chosen_ap = i;
                max_residual = copy_ap_residual_resource[i];
            }
        
        #else

        
        //找出rho - D / R 最大的RF AP
        
            //記錄剩餘資源最大的RF AP
            int chosen_RF_AP = 0;
            
            //記錄該資源剩餘量
            double max_RF_residual = copy_ap_residual_resource[0];
            
            //遍歷RF AP更新最佳選擇
            for(int i = 0 ; i < RF_AP_Num ; i++)
                if(copy_ap_residual_resource[i] > max_RF_residual)
                {
                    chosen_RF_AP = i;
                    max_RF_residual = copy_ap_residual_resource[i];
                }
        
        
        //同理 找出rho - D / R 最大的VLC AP

            //記錄剩餘資源最大的VLC AP
            int chosen_VLC_AP = RF_AP_Num;
            
            //記錄該資源剩餘量
            double max_VLC_residual = copy_ap_residual_resource[RF_AP_Num];
            
            //遍歷VLC AP更新最佳選擇
            for(int i = 0 ; i < VLC_AP_Num ; i++)
                if(copy_ap_residual_resource[i + RF_AP_Num] > max_VLC_residual)
                {
                    chosen_VLC_AP = i + RF_AP_Num;
                    max_VLC_residual = copy_ap_residual_resource[i + RF_AP_Num];
                }



        

        //如果chosen VLC AP可"完全滿足"此demand,則選擇VLC AP
        if(max_VLC_residual >=0)
        {
            chosen_ap = chosen_VLC_AP;
            max_residual = max_VLC_residual;
        }

        //否則代表chosen VLC AP無法完全滿足 
        else
        {  

            //看看chsoen RF AP 能不能完全滿足
            //可的話連RF AP
            if(max_RF_residual>=0)
            {
                chosen_ap = chosen_RF_AP;
                max_residual = max_RF_residual;
            }

            //否則代表VLC RF皆無法完全滿足此demand
            //接下來方法可再跟老師討論
            //方法1 比較這兩個AP誰能提供比較大的Throughput(而非achievable datarate),大的代表能給這個demand比較高的滿意度
            //方法2 用threshold來篩

            //先寫方法1
            else
            {
                //VLC可提供的throughput
                double estimate_VLC_throughput = TDMA_Matrix[chosen_VLC_AP][0] * VLC_DataRate_Matrix[chosen_VLC_AP - RF_AP_Num][myUElist[u].GetID()]; 

                //RF可提供的throughput
                double estimate_RF_throughput = TDMA_Matrix[chosen_RF_AP][0] * RF_DataRate_Matrix[chosen_RF_AP][myUElist[u].GetID()]; 


                //找能提供較大throughput的人連
                if(estimate_VLC_throughput >= estimate_RF_throughput)
                {
                    chosen_ap = chosen_VLC_AP;
                    max_residual = max_VLC_residual;
                }
                else
                {
                    chosen_ap = chosen_RF_AP;
                    max_residual = max_RF_residual;
                }
            }
          
        }
            
        #endif

        //連到chosen ap
        AP_Association_Matrix[chosen_ap][myUElist[u].GetID()] = 1;
        
        //Proposed method確定之後就不會被踢走了，所以可以直接確定AP selection結果
        myUElist[u].Set_Now_Associated_AP(chosen_ap);
        
        //先盡可能配置滿足其demand的資源給ue u
        //如果無法滿足demand，則chosen ap剩的全給ue u
        
        //如果剩餘資源量爲負，則代表無法滿足ue u的demand.故剩下的資源全給ue u,導致分配後剩餘資源 = 0
        if(max_residual < 0)
            max_residual = 0;
        
        //這行的想法是 ρ(chosen_ap,u) (要分配的資源量) =   ρ(chosen_ap,0)(目前所剩資源量) - max_residual(分配後的剩餘資源量)
        TDMA_Matrix[chosen_ap][myUElist[u].GetID()+1] = TDMA_Matrix[chosen_ap][0] - max_residual;

        // if(chosen_ap < RF_AP_Num)
        // {
        //     if( TDMA_Matrix[chosen_ap][myUElist[u].GetID()+1] > satis_threshold * myUElist[u].Get_Required_DataRate() * RF_DataRate_Matrix[chosen_ap][myUElist[u].GetID()])
        //     {
        //         TDMA_Matrix[chosen_ap][myUElist[u].GetID()+1]  = satis_threshold * myUElist[u].Get_Required_DataRate() * RF_DataRate_Matrix[chosen_ap][myUElist[u].GetID()];
        //     }
        // }
        // else
        // {
        //     if( TDMA_Matrix[chosen_ap][myUElist[u].GetID()+1] > satis_threshold * myUElist[u].Get_Required_DataRate() * VLC_DataRate_Matrix[chosen_ap - RF_AP_Num][myUElist[u].GetID()])
        //     {
        //         TDMA_Matrix[chosen_ap][myUElist[u].GetID()+1]  = satis_threshold * myUElist[u].Get_Required_DataRate() * VLC_DataRate_Matrix[chosen_ap - RF_AP_Num][myUElist[u].GetID()];
        //     }
        // }


        //在myUElist[u]中更新分得時間比例
        //Note : 此時的值未必是最終值，之後再做residual resource allocation時也許會有變化
        myUElist[u].Set_Time_Fraction(TDMA_Matrix[chosen_ap][myUElist[u].GetID()+1]);
        
        
        //更新ρ(chosen_ap,0)(目前所剩資源量) ，即要再扣掉剛剛分配的量，
        TDMA_Matrix[chosen_ap][0] -= TDMA_Matrix[chosen_ap][myUElist[u].GetID()+1];

        
    }


    ////////////////////////////////
    ////////////////////////////////
    ////                        ////
    ////        stage 2         ////
    ////                        ////
    ////////////////////////////////
    ////////////////////////////////
    
    #if DEBUG_MODE
        std::cout<<"TDMA Matrix after stage 1"<<std::endl;
        print_TDMA_Matrix(TDMA_Matrix);
    #endif
    
    //此階段要做的事：對於尚有資源未分配完的AP，將其資源分配給底下的UE，不要留下冗餘資源
    //有三種分配方法：
    
    //1. maximize throughtput : 把剩餘資源全部分給datarate最快的UE，目的是想要衝throughput

    //fairness 再細分成2種：
    //2. share by propotion ： 剩餘資源依照data rate^(-1)的比例來分,使得stage1獲得throughput多的人分少一點，反之分得多一點-->考量點是fairness
    //3. save low datarate first : 和a.不同b., 優先分給throughput最低的UEs，使得其throughput上升到和次低UE相同，不斷重復直到分完-->和a.主要差別：throughput最高的UE基本上不會分到多餘資源

    //再將UElist 依照NodeID 小到大 sort
    //這樣是爲了for loop的index和Matrix的index對應，例如：index u = k 即代表 NodeID = k
    sort(myUElist.begin(),myUElist.end(),[](My_UE_Node a,My_UE_Node b){return a.GetID() < b.GetID();});
    
    //用參數 RESIDUAL_RA_METHOD 來控制要採用何種分配方法
    #if (RESIDUAL_RA_METHOD == 1)//1. maximize throughtput
        maximize_throughput(state , RF_DataRate_Matrix , VLC_DataRate_Matrix , Handover_Efficiency_Matrix, AP_Association_Matrix , TDMA_Matrix , myUElist);
    #endif

    #if (RESIDUAL_RA_METHOD == 2)//2. share by propotion
        share_by_propotion(state , RF_DataRate_Matrix , VLC_DataRate_Matrix , Handover_Efficiency_Matrix, AP_Association_Matrix , TDMA_Matrix , myUElist);
    #endif
    
    #if (RESIDUAL_RA_METHOD == 3)//3. save low datarate first
        save_lower_throughputUE(state , RF_DataRate_Matrix , VLC_DataRate_Matrix , Handover_Efficiency_Matrix, AP_Association_Matrix , TDMA_Matrix , myUElist);
    #endif
    
    #if DEBUG_MODE
        std::cout<<"TDMA Matrix after stage 2"<<std::endl;
        print_TDMA_Matrix(TDMA_Matrix);
    #endif
    ////////////////////////////////
    ////////////////////////////////
    ////                        ////
    ////        stage 3         ////
    ////                        ////
    ////////////////////////////////
    ////////////////////////////////

    //residual Resource allocation結束
    //將最後RA結果存回myUElist
    //APS結果已在前面就更新了
    for(int ue_index = 0 ; ue_index < myUElist.size() ; ue_index ++)
    {
        //抓出此UE連到哪個AP
        int linked_ap = myUElist[ue_index].Get_Now_Associated_AP();
        
        #if DEBUG
            std::cout<<"UE "<< myUElist[ue_index].GetID() <<" linked ap = "<<linked_ap<<std::endl;
        #endif
        
        //更新每個UE的time_fraction
        myUElist[ue_index].Set_Time_Fraction(TDMA_Matrix[linked_ap][ue_index+1]);

        #if DEBUG_MODE
            std::cout<<"time resource = "<<TDMA_Matrix[linked_ap][ue_index+1]<<std::endl;
        #endif
        
        //存這回合的achievable datarate
        //若連到WiFi
        if(linked_ap < RF_AP_Num)
            
            myUElist[ue_index].Add_Curr_Achievable_DataRate(myUElist[ue_index].Get_Time_Fraction() * RF_DataRate_Matrix[linked_ap][ue_index]);
        
        //否則是連到LiFi
        else
                                                                                                                                
            myUElist[ue_index].Add_Curr_Achievable_DataRate( myUElist[ue_index].Get_Time_Fraction() * VLC_DataRate_Matrix[linked_ap - RF_AP_Num][ue_index]);
        
        

        //update這一輪的滿意度
        //滿意度公式 : satisfication_level = min(1,achievable rate / required rate)
        double Curr_satisfication_level = myUElist[ue_index].Get_Achievable_DataRate_History().back() / myUElist[ue_index].Get_Required_DataRate();
    
        if(Curr_satisfication_level > 1) 
            Curr_satisfication_level = 1;
    
        myUElist[ue_index].Add_Curr_satisfication_level(Curr_satisfication_level);

    }

}

//Proposed第二輪以後需要考慮AP handover的效應
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

        //目前都還沒有分時間
        for(int j = 1 ; j < UE_Num +1 ; j++)
            TDMA_Matrix[i][j] = 0;  

    }

    //並根據Required datarate做小到大排序
    //爲了讓demand小的先挑
    
    sort(myUElist.begin(),myUElist.end(),[](My_UE_Node a,My_UE_Node b){return a.Get_Required_DataRate() > b.Get_Required_DataRate();});

  
    ////////////////////////////////
    ////////////////////////////////
    ////                        ////
    ////        stage 1         ////
    ////                        ////
    ////////////////////////////////
    ////////////////////////////////
    
    //初步先配置盡可能滿足demand的量
    //若ap有多餘資源則stage2再分配
    
    //照demand 大->小順序,給每個UE挑
    for(int u = 0 ; u < myUElist.size() ; u++)
    {

        //先copy每個AP還剩下多少時間比例可以分給別人
        std::vector<double> copy_ap_residual_resource(RF_AP_Num + VLC_AP_Num);
        for(int i = 0 ; i < RF_AP_Num ; i++) copy_ap_residual_resource[i] = TDMA_Matrix[i][0];                                  //copy RF AP部分
        for(int i = 0 ; i < VLC_AP_Num ; i++) copy_ap_residual_resource[i + RF_AP_Num] = TDMA_Matrix[i + RF_AP_Num][0];         //copy VLC AP部分

        //計算如果UE u 連到AP i且分得恰可滿足demand的資源後, AP i還會剩下多少residual resource
        //Note : 注意sort過後的index u未必是原本的UE Node ID
        //Note2 : 現在要考慮handover

        //算WiFi 部分
        for(int i = 0 ; i < RF_AP_Num ; i++) 
            
            copy_ap_residual_resource[i] -= myUElist[u].Get_Required_DataRate() / 
            (Handover_Efficiency_Matrix[myUElist[u].Get_Now_Associated_AP()][i] * RF_DataRate_Matrix[i][myUElist[u].GetID()]);
                                                    //用get_now_associated_ap是因爲這一輪還沒更新AP
        
        //算LiFi部分
        for(int i = 0 ; i < VLC_AP_Num ; i++) 
            
            copy_ap_residual_resource[i + RF_AP_Num]-= myUElist[u].Get_Required_DataRate() / 
            (Handover_Efficiency_Matrix[myUElist[u].Get_Now_Associated_AP()][i + RF_AP_Num] * VLC_DataRate_Matrix[i][myUElist[u].GetID()]);
    

        //選擇可以剩餘最多資源的AP，剩的資源越多，越有機會服務更多人
        int chosen_ap = 0;
        double max_residual = copy_ap_residual_resource[0];
        #if 0   //原方法 
        for(int i = 0 ; i < RF_AP_Num + VLC_AP_Num ; i++)
            if(copy_ap_residual_resource[i] > max_residual){
                chosen_ap = i;
                max_residual = copy_ap_residual_resource[i];
            }
        
        #else
        //找出rho - D / R 最大的RF AP
        
            //記錄剩餘資源最大的RF AP
            int chosen_RF_AP = 0;
            
            //記錄該資源剩餘量
            double max_RF_residual = copy_ap_residual_resource[0];
            
            //遍歷RF AP更新最佳選擇
            for(int i = 0 ; i < RF_AP_Num ; i++)
                if(copy_ap_residual_resource[i] > max_RF_residual)
                {
                    chosen_RF_AP = i;
                    max_RF_residual = copy_ap_residual_resource[i];
                }
        
        
        //同理 找出rho - D / R 最大的VLC AP

            //記錄剩餘資源最大的VLC AP
            int chosen_VLC_AP = RF_AP_Num;
            
            //記錄該資源剩餘量
            double max_VLC_residual = copy_ap_residual_resource[RF_AP_Num];
            
            //遍歷VLC AP更新最佳選擇
            for(int i = 0 ; i < VLC_AP_Num ; i++)
                if(copy_ap_residual_resource[i + RF_AP_Num] > max_VLC_residual)
                {
                    chosen_VLC_AP = i + RF_AP_Num;
                    max_VLC_residual = copy_ap_residual_resource[i + RF_AP_Num];
                }



        

        //如果chosen VLC AP可"完全滿足"此demand,則選擇VLC AP
        if(max_VLC_residual >=0)
        {
            chosen_ap = chosen_VLC_AP;
            max_residual = max_VLC_residual;
        }

        //否則代表chosen VLC AP無法完全滿足 
        else
        {  

            //看看chsoen RF AP 能不能完全滿足
            //可的話連RF AP
            if(max_RF_residual>=0)
            {
                chosen_ap = chosen_RF_AP;
                max_residual = max_RF_residual;
            }

            //否則代表VLC RF皆無法完全滿足此demand
            //接下來方法可再跟老師討論
            //方法1 比較這兩個AP誰能提供比較大的Throughput(而非achievable datarate),大的代表能給這個demand比較高的滿意度
            //方法2 用threshold來篩

            //先寫方法1
            else
            {
                //VLC可提供的throughput
                double estimate_VLC_throughput = TDMA_Matrix[chosen_VLC_AP][0] * Handover_Efficiency_Matrix[myUElist[u].Get_Now_Associated_AP()][chosen_VLC_AP] * VLC_DataRate_Matrix[chosen_VLC_AP - RF_AP_Num][myUElist[u].GetID()]; 

                //RF可提供的throughput
                double estimate_RF_throughput = TDMA_Matrix[chosen_RF_AP][0] *  Handover_Efficiency_Matrix[myUElist[u].Get_Now_Associated_AP()][chosen_RF_AP] * RF_DataRate_Matrix[chosen_RF_AP][myUElist[u].GetID()]; 


                //找能提供較大throughput的人連
                if(estimate_VLC_throughput >= estimate_RF_throughput)
                {
                    chosen_ap = chosen_VLC_AP;
                    max_residual = max_VLC_residual;
                }
                else
                {
                    chosen_ap = chosen_RF_AP;
                    max_residual = max_RF_residual;
                }
            }
          
        }
            
        #endif

        
           
        

        //連到chosen ap
        AP_Association_Matrix[chosen_ap][myUElist[u].GetID()] = 1;
        
        //Proposed method確定之後就不會被踢走了，所以可以直接確定AP selection結果
        myUElist[u].Set_Now_Associated_AP(chosen_ap);
        
        //std::cout << "UE " << myUElist[u].GetID() <<" demand = "<<myUElist[u].Get_Required_DataRate() <<" linked to AP "<<myUElist[u].Get_Now_Associated_AP()<<std::endl;
      
        
        //先盡可能配置滿足其demand的資源給ue u
        //如果無法滿足demand，則chosen ap剩的全給ue u
        
        //如果剩餘資源量爲負，則代表無法滿足ue u的demand.故剩下的資源全給ue u,導致分配後剩餘資源 = 0
        if(max_residual < 0)
            max_residual = 0;
        
        //這行的想法是 ρ(chosen_ap,u) (要分配的資源量) =   ρ(chosen_ap,0)(目前所剩資源量) - max_residual(分配後的剩餘資源量)
        TDMA_Matrix[chosen_ap][myUElist[u].GetID() +1] = TDMA_Matrix[chosen_ap][0] - max_residual;
   
        // if(chosen_ap < RF_AP_Num)
        // {
        //     if( TDMA_Matrix[chosen_ap][myUElist[u].GetID()+1] > satis_threshold * Handover_Efficiency_Matrix[myUElist[u].Get_Prev_Associated_AP()][myUElist[u].Get_Now_Associated_AP()] * myUElist[u].Get_Required_DataRate() * RF_DataRate_Matrix[chosen_ap][myUElist[u].GetID()])
        //     {
        //         TDMA_Matrix[chosen_ap][myUElist[u].GetID()+1]  = satis_threshold * Handover_Efficiency_Matrix[myUElist[u].Get_Prev_Associated_AP()][myUElist[u].Get_Now_Associated_AP()] * myUElist[u].Get_Required_DataRate() * RF_DataRate_Matrix[chosen_ap][myUElist[u].GetID()];
        //     }
        // }
        // else
        // {
        //     if( TDMA_Matrix[chosen_ap][myUElist[u].GetID()+1] > satis_threshold * Handover_Efficiency_Matrix[myUElist[u].Get_Prev_Associated_AP()][myUElist[u].Get_Now_Associated_AP()] * myUElist[u].Get_Required_DataRate() * VLC_DataRate_Matrix[chosen_ap - RF_AP_Num][myUElist[u].GetID()])
        //     {
        //         TDMA_Matrix[chosen_ap][myUElist[u].GetID()+1]  = satis_threshold * Handover_Efficiency_Matrix[myUElist[u].Get_Prev_Associated_AP()][myUElist[u].Get_Now_Associated_AP()] * myUElist[u].Get_Required_DataRate() * VLC_DataRate_Matrix[chosen_ap - RF_AP_Num][myUElist[u].GetID()];
        //     }
        // }
        //在myUElist[u]中更新分得時間比例
        //Note : 此時的值未必是最終值，之後再做residual resource allocation時也許會有變化
        myUElist[u].Set_Time_Fraction(TDMA_Matrix[chosen_ap][myUElist[u].GetID()+1]);
        
        
        //更新ρ(chosen_ap,0)(目前所剩資源量) ，即要再扣掉剛剛分配的量，
        TDMA_Matrix[chosen_ap][0] -= TDMA_Matrix[chosen_ap][myUElist[u].GetID()+1];

        
    }
    #if 1
        std::cout<<"TDMA Matrix after stage 1"<<std::endl;
        print_TDMA_Matrix(TDMA_Matrix);

        std::cout<<"throughput after stage 1"<<std::endl;
        for(int i = 0 ; i < myUElist.size() ; i++){
            
            std::cout<<"State : "<<state<<" UE :"<<myUElist[i].GetID()<<" Demand = "<<myUElist[i].Get_Required_DataRate() <<" linked to ";
            (myUElist[i].Get_Now_Associated_AP() < RF_AP_Num ) ? (std::cout << "RF AP " << myUElist[i].Get_Now_Associated_AP()) : (std::cout<<"VLC AP "<< myUElist[i].Get_Now_Associated_AP() - RF_AP_Num );
            std::cout<<"  DateRate = ";
            double rate = Handover_Efficiency_Matrix[myUElist[i].Get_Prev_Associated_AP()][myUElist[i].Get_Now_Associated_AP()] * TDMA_Matrix[myUElist[i].Get_Now_Associated_AP()][myUElist[i].GetID()+1];
          
            (myUElist[i].Get_Now_Associated_AP() < RF_AP_Num ) ? rate = rate * RF_DataRate_Matrix[myUElist[i].Get_Now_Associated_AP()][myUElist[i].GetID()] : rate = rate * VLC_DataRate_Matrix[myUElist[i].Get_Now_Associated_AP() - RF_AP_Num][myUElist[i].GetID()];
            std::cout<<rate<<std::endl;
        
        }
    #endif
 

    ////////////////////////////////
    ////////////////////////////////
    ////                        ////
    ////        stage 2         ////
    ////                        ////
    ////////////////////////////////
    ////////////////////////////////

    //此階段要做的事：對於尚有資源未分配完的AP，將其資源分配給底下的UE，不要留下冗餘資源
    //有三種分配方法：
    
    //1. maximize throughtput : 把剩餘資源全部分給datarate最快的UE，目的是想要衝throughput

    //fairness 再細分成2種：
    //2. share by propotion ： 剩餘資源依照data rate^(-1)的比例來分,使得stage1獲得throughput多的人分少一點，反之分得多一點-->考量點是fairness
    //3. save low datarate first : 和a.不同b., 優先分給throughput最低的UEs，使得其throughput上升到和次低UE相同，不斷重復直到分完-->和a.主要差別：throughput最高的UE基本上不會分到多餘資源

    //再將UElist 依照NodeID 小到大 sort
    //這樣是爲了for loop的index和Matrix的index對應，例如：index u = k 即代表 NodeID = k
    sort(myUElist.begin(),myUElist.end(),[](My_UE_Node a,My_UE_Node b){return a.GetID() < b.GetID();});

    // double sysnow=0;
    // std::vector<double> advancedrate(UE_Num,0);
    // for(int i = 0 ; i < myUElist.size() ; i++){
    //     std::cout<<"UE"<<myUElist[i].GetID()<<" after stage1 obtain avg throughput = "; 
    //     double dr = Handover_Efficiency_Matrix[myUElist[i].Get_Prev_Associated_AP()][myUElist[i].Get_Now_Associated_AP()] * TDMA_Matrix[myUElist[i].Get_Now_Associated_AP()][myUElist[i].GetID()+1];
    //     if(myUElist[i].Get_Now_Associated_AP() < RF_AP_Num)
    //         dr = dr * RF_DataRate_Matrix[myUElist[i].Get_Now_Associated_AP()][myUElist[i].GetID()];
    //     else
    //         dr = dr * VLC_DataRate_Matrix[myUElist[i].Get_Now_Associated_AP() - RF_AP_Num][myUElist[i].GetID()];
        
    //     std::cout<<dr<<std::endl;
    //     sysnow += dr;
    //     advancedrate[myUElist[i].GetID()] = dr;
    // }

   
    
    
    //用參數 RESIDUAL_RA_METHOD 來控制要採用何種分配方法
    #if (RESIDUAL_RA_METHOD == 1)//1. maximize throughtput
        maximize_throughput(state , RF_DataRate_Matrix , VLC_DataRate_Matrix , Handover_Efficiency_Matrix, AP_Association_Matrix , TDMA_Matrix , myUElist);
    #endif

    #if (RESIDUAL_RA_METHOD == 2)//2. share by propotion
        share_by_propotion(state , RF_DataRate_Matrix , VLC_DataRate_Matrix , Handover_Efficiency_Matrix, AP_Association_Matrix , TDMA_Matrix , myUElist);
    #endif
    
    #if (RESIDUAL_RA_METHOD == 3)//3. save low datarate first
        save_lower_throughputUE(state , RF_DataRate_Matrix , VLC_DataRate_Matrix , Handover_Efficiency_Matrix, AP_Association_Matrix , TDMA_Matrix , myUElist);
    #endif

    #if DEBUG_MODE
        std::cout<<"TDMA Matrix after stage 2"<<std::endl;
        print_TDMA_Matrix(TDMA_Matrix);
    #endif
   
    // std::cout<<std::endl;
    // double sysextra = 0;
    // for(int i = 0 ; i < myUElist.size() ; i++){
    //     std::cout<<"UE"<<myUElist[i].GetID()<<" after stage2 obtain avg throughput = "; 
    //     double dr = Handover_Efficiency_Matrix[myUElist[i].Get_Prev_Associated_AP()][myUElist[i].Get_Now_Associated_AP()] * TDMA_Matrix[myUElist[i].Get_Now_Associated_AP()][myUElist[i].GetID()+1];
    //     if(myUElist[i].Get_Now_Associated_AP() < RF_AP_Num)
    //         dr = dr * RF_DataRate_Matrix[myUElist[i].Get_Now_Associated_AP()][myUElist[i].GetID()];
    //     else
    //         dr = dr * VLC_DataRate_Matrix[myUElist[i].Get_Now_Associated_AP() - RF_AP_Num][myUElist[i].GetID()];
    //     sysextra +=dr; 
        
    //     std::cout<<dr<<" advanced obtain : "<< dr - advancedrate[myUElist[i].GetID()]<<std::endl;
    // }

    // std::cout<<"totally obtain :" <<sysextra-sysnow<<std::endl;

    ////////////////////////////////
    ////////////////////////////////
    ////                        ////
    ////        stage 3         ////
    ////                        ////
    ////////////////////////////////
    ////////////////////////////////

    //residual Resource allocation結束
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
            
            myUElist[ue_index].Add_Curr_Achievable_DataRate(Handover_Efficiency_Matrix[myUElist[ue_index].Get_Prev_Associated_AP()][linked_ap] * myUElist[ue_index].Get_Time_Fraction() * RF_DataRate_Matrix[linked_ap][ue_index]);
        
        //否則是連到LiFi
        else
                                                                                                                                // - RF_AP_Num是爲了對應到正確的VLC Matrix位置
            myUElist[ue_index].Add_Curr_Achievable_DataRate(Handover_Efficiency_Matrix[myUElist[ue_index].Get_Prev_Associated_AP()][linked_ap] * myUElist[ue_index].Get_Time_Fraction() * VLC_DataRate_Matrix[linked_ap - RF_AP_Num][ue_index]);
        
        

        //update這一輪的滿意度
        //滿意度公式 : satisfication_level = min(1,achievable rate / required rate)
        double Curr_satisfication_level = myUElist[ue_index].Get_Achievable_DataRate_History().back() / myUElist[ue_index].Get_Required_DataRate();
    
        if(Curr_satisfication_level > 1) 
            Curr_satisfication_level = 1;
    
        myUElist[ue_index].Add_Curr_satisfication_level(Curr_satisfication_level);

    }

}

/*
    分配給速度最高的人

    EX ： 假設 AP i 服務 UE 1,2,3
    ρ(i,0) 剩 0.1
    r(i,1) = 25Mb/s
    r(i,2) = 50Mb/s
    r(i,3) = 75Mb/s
    則：
    UE 3 額外獲得 0.1
*/
void maximize_throughput( 
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
    根據速度倒數比分配剩餘資源

    EX ： 假設 AP i 服務 UE 1,2,3
    ρ(i,0) 剩 0.1
    r(i,1) = 25Mb/s
    r(i,2) = 50Mb/s
    r(i,3) = 75Mb/s
    則：
    UE 1 額外獲得 0.1 *（1/25） / （1/25 + 1/50 + 1/75）
    UE 2 額外獲得 0.1 *（1/50） / （1/25 + 1/50 + 1/75）
    UE 3 額外獲得 0.1 *（1/75） / （1/25 + 1/50 + 1/75）
*/

void share_by_propotion( 
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
                

                //std::cout<<"share propotion of RF AP "<<i<< std::setiosflags(std::ios::fixed)<< std::setprecision(5)<<std::endl;

                //記錄此AP下UE速度的倒數和，作爲分母
                double Reciprocal_Sum_Of_DataRate = 0.0;
                
                std::map<int,double> UE_to_DataRate;

                //檢查所有UE
                for(int ue = 0 ; ue < UE_Num  ; ue ++)
                {
                    
                    //若AP i有連到這個ue
                    if(AP_Association_Matrix[i][ue] == 1)
                    {

                        //初始state不必考慮handover
                        if(state == 0)
                        {
                            #if PROPOTION_BY_ACHE_DR
                                
                                //則更新分母
                                Reciprocal_Sum_Of_DataRate += (1 / RF_DataRate_Matrix[i][ue]);
                            
                                //更新該ue的分子
                                UE_to_DataRate[ue] = 1 / RF_DataRate_Matrix[i][ue];
                            
                            #else
                             
                                //則更新分母
                                Reciprocal_Sum_Of_DataRate += 1 / (TDMA_Matrix[i][ue+1] * pow(RF_DataRate_Matrix[i][ue],2));
                            
                                //更新該ue的分子
                                UE_to_DataRate[ue] = 1 / (TDMA_Matrix[i][ue+1] * pow(RF_DataRate_Matrix[i][ue],2));
                            
                            #endif

                           

                            
                        }
                        //非初始state則需要多考慮handover
                        else
                        {
                            
                            #if PROPOTION_BY_ACHE_DR

                                //更新分母
                                Reciprocal_Sum_Of_DataRate += (1 / (Handover_Efficiency_Matrix[myUElist[ue].Get_Prev_Associated_AP()][i],2 * RF_DataRate_Matrix[i][ue],2));
    
                                //更新該ue的分子
                                UE_to_DataRate[ue] = 1 / (Handover_Efficiency_Matrix[myUElist[ue].Get_Prev_Associated_AP()][i] * RF_DataRate_Matrix[i][ue],2);

                                // std::cout<<"UE "<< ue <<" achevable DR = "<<Handover_Efficiency_Matrix[myUElist[ue].Get_Prev_Associated_AP()][i] * RF_DataRate_Matrix[i][ue]<< " propotion = " << UE_to_DataRate[ue]<<std::endl;
                            
                            #else
                            
                                //更新分母
                                Reciprocal_Sum_Of_DataRate += (1 / (pow(Handover_Efficiency_Matrix[myUElist[ue].Get_Prev_Associated_AP()][i],2)  * TDMA_Matrix[i][ue+1] * pow(RF_DataRate_Matrix[i][ue],2)));
    
                                //更新該ue的分子
                                UE_to_DataRate[ue] = 1 / (pow(Handover_Efficiency_Matrix[myUElist[ue].Get_Prev_Associated_AP()][i],2)  * TDMA_Matrix[i][ue+1] * pow(RF_DataRate_Matrix[i][ue],2));

                               //std::cout<<"UE "<< ue <<" achevable DR = "<<Handover_Efficiency_Matrix[myUElist[ue].Get_Prev_Associated_AP()][i] * TDMA_Matrix[i][ue+1] * RF_DataRate_Matrix[i][ue]<< " propotion = " << UE_to_DataRate[ue]<<std::endl;
                            
                            #endif

                          
                        }

                    
                    }
                }

                //std::cout<<"Reciprocal_Sum_Of_DataRate = "<< Reciprocal_Sum_Of_DataRate <<std::endl;
                
                //通過UE_to_DataRate取得每個UE的分子，再處剛剛算出來的分母，即爲每個UE可額外分得的資源量
                std::map<int, double>::iterator it;
                for(it = UE_to_DataRate.begin() ; it != UE_to_DataRate.end() ; it++)
		        {    

                    
                    TDMA_Matrix[i][it->first + 1] += it->second / Reciprocal_Sum_Of_DataRate * TDMA_Matrix[i][0];
                    
                    std::cout<<"UE " <<it->first<<" obtain "<< it->second << " / " << Reciprocal_Sum_Of_DataRate << " * " << TDMA_Matrix[i][0] << " = " <<it->second / Reciprocal_Sum_Of_DataRate * TDMA_Matrix[i][0]<<std::endl;
                    
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
                
                //記錄此AP下UE速度的倒數和，作爲分母
                double Reciprocal_Sum_Of_DataRate = 0.0;
                
                std::map<int,double> UE_to_DataRate;

                //檢查所有UE
                for(int ue =  0; ue < UE_Num  ; ue ++)
                {
                    
                    //若AP i有連到這個ue
                    if(AP_Association_Matrix[i + RF_AP_Num][ue] == 1)
                    {

                        //初始state不必考慮handover
                        if(state == 0)
                        {

                            #if PROPOTION_BY_ACHE_DR
                                //則更新分母
                                Reciprocal_Sum_Of_DataRate += (1 / VLC_DataRate_Matrix[i][ue]);
                                
                                //更新該ue的分子
                                UE_to_DataRate[ue] = 1 / VLC_DataRate_Matrix[i][ue];
                            #else
                                //則更新分母
                                Reciprocal_Sum_Of_DataRate += 1 /(TDMA_Matrix[i+RF_AP_Num][ue+1] * pow(VLC_DataRate_Matrix[i][ue],2));
                                
                                //更新該ue的分子
                                UE_to_DataRate[ue] = 1 / (TDMA_Matrix[i+RF_AP_Num][ue+1] * pow(VLC_DataRate_Matrix[i][ue],2));
                            #endif
                            

                        }
                        //非初始state則需要多考慮handover
                        else
                        {

                            #if PROPOTION_BY_ACHE_DR
                                //更新分母
                                Reciprocal_Sum_Of_DataRate += (1 / (Handover_Efficiency_Matrix[myUElist[ue].Get_Prev_Associated_AP()][i + RF_AP_Num] * VLC_DataRate_Matrix[i][ue]));
        
                                //更新該ue的分子
                                UE_to_DataRate[ue] = 1 / (Handover_Efficiency_Matrix[myUElist[ue].Get_Prev_Associated_AP()][i + RF_AP_Num] * VLC_DataRate_Matrix[i][ue]);
                            #else
                                //更新分母
                                Reciprocal_Sum_Of_DataRate += (1 / (pow(Handover_Efficiency_Matrix[myUElist[ue].Get_Prev_Associated_AP()][i + RF_AP_Num],2) * TDMA_Matrix[i+RF_AP_Num][ue+1] * pow(VLC_DataRate_Matrix[i][ue],2)));
        
                                //更新該ue的分子
                                UE_to_DataRate[ue] = 1 / (pow(Handover_Efficiency_Matrix[myUElist[ue].Get_Prev_Associated_AP()][i + RF_AP_Num],2) * TDMA_Matrix[i+RF_AP_Num][ue+1] * pow(VLC_DataRate_Matrix[i][ue],2));

                            #endif
                        }
                       
                    }

                     
                }
                //通過UE_to_DataRate取得每個UE的分子，再處剛剛算出來的分母，即爲每個UE可額外分得的資源量
                std::map<int, double>::iterator it;
                for(it = UE_to_DataRate.begin() ; it != UE_to_DataRate.end() ; it++)
		        {    
                    //注意在TDMA_Matrix中，UEindex從1開始，故it->first要+1
                    TDMA_Matrix[i + RF_AP_Num][it->first + 1] += it->second / Reciprocal_Sum_Of_DataRate * TDMA_Matrix[i + RF_AP_Num][0];
                     
                    //在myUElist[it->first]中更新分得時間比例
                    //Note : 此時的值即是最終值
                    myUElist[it->first].Set_Time_Fraction(TDMA_Matrix[i + RF_AP_Num][it->first + 1]);
                }
                
                //剩餘的資源皆分完
                TDMA_Matrix[i + RF_AP_Num][0] = 0;
                
            }
        }

}

/*
    優先分配給低速度的UE，將其拉到和次低者一樣
    直到做不到上述操作爲止，剩下極少量就同爲最低者平分

    EX ： 假設 AP i 服務 UE 1,2,3 , 不計handover efficiency
    ρ(i,0) 剩 0.1
    
    throughput(i,1) = ρ(i,1) * r(i,1) = 10Mb/s
    throughput(i,2) = ρ(i,2) * r(i,2) = 20Mb/s
    throughput(i,3) = ρ(i,3) * r(i,3) = 30Mb/s
    
    則：
    round 1 :
        UE 1 額外獲得 x1 使得 ( ρ(i,1) + x1 ) * r(i,1) = UE2的throughput 20Mb /s
        ρ(i,0) -= x1
    round 2 :

    if(可追上UE3的throughput)
        UE 1 額外獲得 y1 使得 ( ρ(i,1) + x1 + y1 ) * r(i,1) = UE3的throughput 30Mb /s
        ρ(i,0) -= y1
        UE 2 額外獲得 y2 使得 ( ρ(i,2) + y2 )      * r(i,2) = UE3的throughput 30Mb /s
        ρ(i,0) -= y2
    else
        UE1,UE2平分ρ(i,0)

// */
// void save_lower_throughputUE( 
//   int & state,
//   std::vector<std::vector<double>> & RF_DataRate_Matrix ,
//   std::vector<std::vector<double>> & VLC_DataRate_Matrix,
//   std::vector<std::vector<double>> & Handover_Efficiency_Matrix,
//   std::vector<std::vector<int>>    & AP_Association_Matrix ,  
//   std::vector<std::vector<double>> & TDMA_Matrix,                            
//   std::vector<My_UE_Node> & myUElist){

     
//         //檢查所有WiFi AP
//         for(int i = 0 ; i < RF_AP_Num ; i++)
//         {
            
//             //若有資源未分配完
//             if(TDMA_Matrix[i][0] > 0)
//             {

//                 //vector<pair>來用當作分配的依據
//                 //pair.first = 被服務的UE之 ueid(index)
//                 //pair.second = pair.first 在stage1獲得的throughput;
//                 //稍後要根據pair.second做sorting
//                 std::vector<std::pair<int,double> > servedUE_of_AP; 
                
//                 //檢查所有UE
//                 for(int ue = 0 ; ue < UE_Num ; ue ++)
//                 {
                    
//                     //若AP i有連到這個ue
//                     if(AP_Association_Matrix[i][ue] == 1)
//                     {
 
//                         //stage1_TP代表目前配置下得到的throughput
//                         double stage1_TP ;
                        
//                         //初始state不必考慮handover
//                         if(state == 0)
                            
//                             //stage1_TP代表目前配置下得到的throughput
//                             stage1_TP =  TDMA_Matrix[i][ue + 1] * RF_DataRate_Matrix[i][ue]; 

                         
//                         //非初始state則需要多考慮handover
//                         else
                       
//                             //stage1_TP代表目前配置下得到的throughput
//                             stage1_TP =  Handover_Efficiency_Matrix[myUElist[ue].Get_Prev_Associated_AP()][i] * TDMA_Matrix[i][ue + 1] * RF_DataRate_Matrix[i][ue]; 
                      
                          
//                         //將此UE和其stage1_TP加入servedUE_of_AP
//                         servedUE_of_AP.push_back(std::make_pair(ue,stage1_TP));
//                     }

                    
//                 }
                
//                 //將此AP服務的UE們依照stage1獲得的throughput升序排列
//                 //目的是先拯救throughput最低的人
//                 sort(servedUE_of_AP.begin(),servedUE_of_AP.end(),[](std::pair<int,double> a,std::pair<int,double> b){return a.second < b.second;});
                
                
                
//                 //如果此AP只服務1個UE，則全部給它
//                 if(servedUE_of_AP.size() == 1)
//                 {
                    
//                     //全給那個UE
//                     TDMA_Matrix[i][servedUE_of_AP.back().first + 1] += TDMA_Matrix[i][0];


//                     //分完就沒有了
//                     TDMA_Matrix[i][0] = 0;
//                 }

//                 //否則代表此AP服務2個以上UE
//                 else //if(servedUE_of_AP.size() > 1)
//                 {
//                     //i控制現在最多分配到第幾個UE爲止
//                     for(int u = 1 ; u < servedUE_of_AP.size() ; u++)
//                     {
//                         //need代表此輪分配需要佔用多少資源
//                         double need = 0;

//                         //extra記錄要多給哪個UE多少資源
//                         //extra.first : UE index
//                         //extra.second : 分配給extra.first的額外資源
//                         std::map<int,double> extra;

//                         //這一輪要將額外資源分配給前u個UE
//                         for(int k = 0 ; k < u ; k++)
//                         {

//                             //last_ap記錄servedUE_of_AP[k]放的UE，其上一輪AP是誰
//                             int last_ap = myUElist[servedUE_of_AP[k].first].Get_Prev_Associated_AP();
                            
//                             //初始state不必考慮handover
//                             if(state == 0)
                
//                                 //計算如果要將throughput_lowest拉到與throughput_secondlowest相等，需要額外給多少資源
//                                 extra[servedUE_of_AP[k].first] = ( servedUE_of_AP[u].second - servedUE_of_AP[k].second ) / RF_DataRate_Matrix[i][servedUE_of_AP[k].first];

                        
                            
                            
//                             //非初始state則需要多考慮handover
//                             else
                        
//                                 //計算如果要將throughput_lowest拉到與throughput_secondlowest相等，需要額外給多少資源
//                                 extra[servedUE_of_AP[k].first] = ( servedUE_of_AP[u].second - servedUE_of_AP[k].second ) / Handover_Efficiency_Matrix[last_ap][i] * RF_DataRate_Matrix[i][servedUE_of_AP[k].first];

                            

//                             //記錄總需求量
//                             need += extra[servedUE_of_AP[k].first];


//                         }


//                         //如果剛剛算出來的需求量 > 剩餘的資源量
//                         //就代表沒辦法照剛剛的結果配置，基本上剩很少很少了
//                         //則改成給UE們平分
//                         if(need > TDMA_Matrix[i][0])
//                         {
//                             //大家平分TDMA_Matrix[i][0]
//                             for(int j = 0 ; j < servedUE_of_AP.size() ; j++)
//                             {
//                                 TDMA_Matrix[i][servedUE_of_AP[j].first + 1] += TDMA_Matrix[i][0] / servedUE_of_AP.size();
//                             }

//                             //大家分完就沒有了
//                             TDMA_Matrix[i][0] = 0;

//                         }

//                         //否則代表剩餘資源量 - need還剩，就還會有下一輪
//                         else
//                         {
//                             //每個UE照剛剛計算的方式分配extra資源
//                             for(int j = 0 ; j < servedUE_of_AP.size() ; j++)
//                             {
//                                 TDMA_Matrix[i][servedUE_of_AP[j].first + 1] += extra[servedUE_of_AP[j].first];
//                             }

//                             //因爲ρ(i,0) > need
//                             //ρ(i,0) - need > 0
//                             //代表還會有下一輪要分配
//                             TDMA_Matrix[i][0] -= need;
//                         }

//                         //若這輪分完
//                         //就break不必再做下一輪
//                         if(TDMA_Matrix[i][0] ==0)
//                             break;

//                     }

//                     //應該不太會有做完上述分配後，還留下資源的可能
//                     //不過還是寫個防呆
//                     //若真的不巧還有剩則平分給此AP連到的所有UE
//                     if(TDMA_Matrix[i][0] > 0)
//                     {
//                         //大家平分TDMA_Matrix[i][0]
//                         for(int j = 0 ; j < servedUE_of_AP.size() ; j++)
//                         {
//                             TDMA_Matrix[i][servedUE_of_AP[j].first + 1] += TDMA_Matrix[i][0] / servedUE_of_AP.size();
//                         }

//                         //大家分完就沒有了
//                         TDMA_Matrix[i][0] = 0;
//                     }
//                 }


//             }
//         }
        
//         //同理，檢查所有LiFi AP
//         for(int i = 0 ; i < VLC_AP_Num ; i++)
//         {
            
//             //若有資源未分配完
//             if(TDMA_Matrix[i + RF_AP_Num][0] > 0)
//             {

//                 //vector<pair>來用當作分配的依據
//                 //pair.first = 被服務的UE之 ueid(index)
//                 //pair.second = pair.first 在stage1獲得的throughput;
//                 //稍後要根據pair.second做sorting
//                 std::vector<std::pair<int,double> > servedUE_of_AP; 
                
//                 //檢查所有UE
//                 for(int ue = 0 ; ue < UE_Num ; ue ++)
//                 {
                    
//                     //若AP i有連到這個ue
//                     if(AP_Association_Matrix[i + RF_AP_Num][ue] == 1)
//                     {
 
//                         //stage1_TP代表目前配置下得到的throughput
//                         double stage1_TP ;
                        
//                         //初始state不必考慮handover
//                         if(state == 0)
                            
//                             //stage1_TP代表目前配置下得到的throughput
//                             stage1_TP =  TDMA_Matrix[i + RF_AP_Num][ue + 1] * VLC_DataRate_Matrix[i][ue]; 

                         
//                         //非初始state則需要多考慮handover
//                         else
                       
//                             //stage1_TP代表目前配置下得到的throughput
//                             stage1_TP =  Handover_Efficiency_Matrix[myUElist[ue].Get_Prev_Associated_AP()][i + RF_AP_Num] 
//                             * TDMA_Matrix[i + RF_AP_Num][ue + 1] * VLC_DataRate_Matrix[i][ue]; 
                      
                          
//                         //將此UE和其stage1_TP加入servedUE_of_AP
//                         servedUE_of_AP.push_back(std::make_pair(ue,stage1_TP));
//                     }
//                 }
                
//                 //將此AP服務的UE們依照stage1獲得的throughput升序排列
//                 //目的是先拯救throughput最低的人
//                 sort(servedUE_of_AP.begin(),servedUE_of_AP.end(),[](std::pair<int,double> a,std::pair<int,double> b){return a.second < b.second;});
                
                
//                 //如果此AP只服務1個UE，則全部給它
//                 if(servedUE_of_AP.size() == 1)
//                 {
                    
//                     //全給那個UE
//                     TDMA_Matrix[i + RF_AP_Num][servedUE_of_AP.back().first + 1] += TDMA_Matrix[i + RF_AP_Num][0];


//                     //分完就沒有了
//                     TDMA_Matrix[i + RF_AP_Num][0] = 0;
//                 }
//                 else
//                 {
//                     //i控制現在最多分配到第幾個UE爲止
//                     for(int u = 1 ; u < servedUE_of_AP.size() ; u++)
//                     {
//                         //need代表此輪分配需要佔用多少資源
//                         double need = 0;

//                         //extra記錄要多給哪個UE多少資源
//                         //extra.first : UE index
//                         //extra.second : 分配給extra.first的額外資源
//                         std::map<int,double> extra;

//                         //這一輪要將額外資源分配給前u個UE
//                         for(int k = 0 ; k < u ; k++)
//                         {

//                             //last_ap記錄servedUE_of_AP[k]放的UE，其上一輪AP是誰
//                             int last_ap = myUElist[servedUE_of_AP[k].first].Get_Prev_Associated_AP();
                            
//                             //初始state不必考慮handover
//                             if(state == 0)
                
//                                 //計算如果要將throughput_lowest拉到與throughput_secondlowest相等，需要額外給多少資源
//                                 extra[servedUE_of_AP[k].first] = ( servedUE_of_AP[u].second - servedUE_of_AP[k].second ) / VLC_DataRate_Matrix[i][servedUE_of_AP[k].first];

                        
                            
                            
//                             //非初始state則需要多考慮handover
//                             else
                        
//                                 //計算如果要將throughput_lowest拉到與throughput_secondlowest相等，需要額外給多少資源
//                                 extra[servedUE_of_AP[k].first] = ( servedUE_of_AP[u].second - servedUE_of_AP[k].second ) / Handover_Efficiency_Matrix[last_ap][i + RF_AP_Num] 
//                                 * VLC_DataRate_Matrix[i][servedUE_of_AP[k].first];

                            

//                             //記錄總需求量
//                             need += extra[servedUE_of_AP[k].first];


//                         }


//                         //如果剛剛算出來的需求量 > 剩餘的資源量
//                         //就代表沒辦法照剛剛的結果配置，基本上剩很少很少了
//                         //則改成給UE們平分
//                         if(need > TDMA_Matrix[i + RF_AP_Num][0])
//                         {
//                             //大家平分TDMA_Matrix[i][0]
//                             for(int j = 0 ; j < servedUE_of_AP.size() ; j++)
//                             {
//                                 TDMA_Matrix[i + RF_AP_Num][servedUE_of_AP[j].first + 1] += TDMA_Matrix[i + RF_AP_Num][0] / servedUE_of_AP.size();
//                             }

//                             //大家分完就沒有了
//                             TDMA_Matrix[i + RF_AP_Num][0] = 0;

//                         }

//                         //否則代表剩餘資源量 - need還剩，就還會有下一輪
//                         else
//                         {
//                             //每個UE照剛剛計算的方式分配extra資源
//                             for(int j = 0 ; j < servedUE_of_AP.size() ; j++)
//                             {
//                                 TDMA_Matrix[i + RF_AP_Num][servedUE_of_AP[j].first + 1] += extra[servedUE_of_AP[j].first];
//                             }

//                             //因爲ρ(i,0) > need
//                             //ρ(i,0) - need > 0
//                             //代表還會有下一輪要分配
//                             TDMA_Matrix[i + RF_AP_Num][0] -= need;
//                         }

//                         //若這輪分完
//                         //就break不必再做下一輪
//                         if(TDMA_Matrix[i + RF_AP_Num][0] ==0)
//                             break;

//                     }
                    

//                     //應該不太會有做完上述分配後，還留下資源的可能
//                     //不過還是寫個防呆
//                     //若真的不巧還有剩則平分給此AP連到的所有UE
//                     if(TDMA_Matrix[i + RF_AP_Num][0] > 0)
//                     {
//                         //大家平分TDMA_Matrix[i][0]
//                         for(int j = 0 ; j < servedUE_of_AP.size() ; j++)
//                         {
//                             TDMA_Matrix[i + RF_AP_Num][servedUE_of_AP[j].first + 1] += TDMA_Matrix[i][0] / servedUE_of_AP.size();
//                         }

//                         //大家分完就沒有了
//                         TDMA_Matrix[i + RF_AP_Num][0] = 0;
//                     }
//                 }
              

//             }
//         }

// }

