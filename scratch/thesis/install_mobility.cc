////////////////////////////////////////////////////////////////////////////////////////////
//// Q : 這份code在幹嘛？                                                                  ///
//// A : 這裏是要爲AP跟UE安裝NS3的移動模型的                                                  ///
////      AP不動而UE會依照設定的Model動                                                     ///
////////////////////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <fstream>
#include <string>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "install_mobility.h"
#include "global_environment.h"

using namespace ns3;

/*
    安裝RF_AP的移動模型

    Note : 在房間內AP分布如鏈接的圖

    https://imgur.com/PTNsJoK

    依照上圖方式配置位置
   
    其中
    房間被長寬皆被 每行or每列的2個RF_AP 分成3段
*/
void install_RF_AP_mobility(NodeContainer & RF_AP_Nodes){
    
    MobilityHelper RF_AP_Mobility;
    
    double delta = room_size /(RF_AP_per_row + 1); //1 row 被 n個AP 分成 n+1段距離

    /** 設定RF_AP的初始位置 **/
    Ptr < ListPositionAllocator > RF_AP_Pos_list = CreateObject<ListPositionAllocator>();
    
    for(int i=0;i<RF_AP_Num;i++){

        //x,y這樣設 ， AP位置才會長得像鏈接圖示這樣
        double x = ( i % RF_AP_per_row +1) * delta;
        double y = ( RF_AP_per_row - i / RF_AP_per_row ) * delta;

        //但是要注意實際上房間左下角是（-room_size/2 , -room_size/2）
        //上面的的假設是房間左下角是（0,0）
        //所以x,y都還要做平移的坐標修正
        x-=room_size/2;
        y-=room_size/2;


        //將（x,y）加入 position list中
        RF_AP_Pos_list->Add(Vector(x,y,RF_AP_height));
    }

    //用剛剛的postion list 設定 PositionAllocator
    RF_AP_Mobility.SetPositionAllocator(RF_AP_Pos_list);


    //RF AP靜止不動
    RF_AP_Mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    RF_AP_Mobility.Install(RF_AP_Nodes);
}

/*
    安裝VLC_AP的移動模型

   
    Note : 在房間內AP分布如鏈接的圖

    https://imgur.com/PTNsJoK

    依照上圖方式配置位置

   其中
   1.兩AP間
   2.AP和牆壁之間
   差距皆爲1 delta
   
*/
void install_VLC_AP_mobility(NodeContainer & VLC_AP_Nodes){
    
    MobilityHelper VLC_AP_Mobility;
    
    double delta = room_size /(VLC_AP_per_row + 1); //1 row 被 n個AP 分成 n+1段距離

    /** 設定VLC_AP的初始位置 **/
    Ptr < ListPositionAllocator > VLC_AP_Pos_list = CreateObject<ListPositionAllocator>();
    
    for(int i=0;i<VLC_AP_Num;i++){

        //x,y這樣設 ， AP位置才會長得像上面圖示這樣
        double x = ( i % VLC_AP_per_row +1) * delta;
        double y = ( VLC_AP_per_row - i / VLC_AP_per_row ) * delta;

        //但是要注意實際上房間左下角是（-room_size/2 , -room_size/2）
        //上面的的假設是房間左下角是（0,0）
        //所以x,y都還要做平移的坐標修正
        x-=room_size/2;
        y-=room_size/2;


        //將（x,y）加入 position list中
        VLC_AP_Pos_list->Add(Vector(x,y,VLC_AP_height));
    }

    //用剛剛的postion list 設定 PositionAllocator
    VLC_AP_Mobility.SetPositionAllocator(VLC_AP_Pos_list);


    //VLC AP靜止不動
    VLC_AP_Mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    VLC_AP_Mobility.Install(VLC_AP_Nodes);
}

#if !PRESSURE
//安裝UE的移動模型
void install_UE_mobility(NodeContainer & UE_Nodes){
    
    //正常的MobilityModel
    MobilityHelper UE_Mobility;
    
    //be used to save position of each UE
    Ptr < ListPositionAllocator > UE_Pos_list = CreateObject<ListPositionAllocator>();


    /** 對每個UE都給他一個隨機位置 **/
    srand( time(NULL) );
    for(int i = 0; i < UE_Num ; i++){

            //UE位置隨機生成
            //x = 0 ~ room size
            //y = 0 ~ room size

			    double x = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX / room_size));
			    double y = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX / room_size));
			
            
            //但是要注意實際上房間左下角是（-room_size/2 , -room_size/2）
            //上面的的假設是房間左下角是（0,0）
            //所以x,y都還要做平移的坐標修正
            x -= room_size / 2;
            y -= room_size / 2;

            UE_Pos_list->Add(Vector(x, y, UE_height));
    }

    //用剛剛的postion list 設定 PositionAllocator
    UE_Mobility.SetPositionAllocator(UE_Pos_list);


    //UE會移動 採用R2D的移動模型
    //每隔2秒改變方向跟速率
    //速度分布則是介於 0 ～ 2 m/s 呈均勻分布
    //範圍則是在房間裏面 
    //x 範圍 =  -room_size / 2 ~ room_size / 2
    //y 範圍 =  -room_size / 2 ~ room_size / 2
    
    //UE正常分布
    UE_Mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
                             "Mode", StringValue ("Time"),
                             "Time", StringValue ("2.5s"),
                             "Speed", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=2.0]"),
                             "Direction",StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=6.283184]"),
                             "Bounds", RectangleValue (Rectangle (-room_size / 2, room_size / 2, -room_size / 2, room_size / 2)));
  

    UE_Mobility.Install (UE_Nodes);

                   
}
#endif



//壓力測試下的UE移動模型
//有一半比例的UE會擁擠在被限制的區域，而其他UE還是維持正常
//論文有100%UE被限制的，我沒留，想重現的人可以從這個改
#if PRESSURE
void install_UE_mobility(NodeContainer & UE_Nodes){
    
    /////////////////////////////////////////////////////////////////
    /////////////         Normal UE : 沒有被限制的UE      /////////////
    /////////////////////////////////////////////////////////////////

    //正常UE的MobilityModel
    MobilityHelper UE_Mobility;
    
    //存放正常UE的起始位置
    Ptr < ListPositionAllocator > UE_Pos_list = CreateObject<ListPositionAllocator>();

    //隨機給出正常UE的起始位置
    //假設有一半的UE爲正常的，所以index to UE_Num/2
    srand( time(NULL) );
    for(int i = 0; i < UE_Num / 2 ; i++){

            //UE位置隨機生成
            //x = 0 ~ room size
            //y = 0 ~ room size
			
            double x = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX / room_size));
			double y = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX / room_size));
			
            //但是要注意實際上房間左下角是（-room_size/2 , -room_size/2）
            //上面的的假設是房間左下角是（0,0）
            //所以x,y都還要做平移的坐標修正
            x -= room_size / 2;
            y -= room_size / 2;

            UE_Pos_list->Add(Vector(x, y, UE_height));
    }

    //用剛剛的postion list 設定 PositionAllocator
    UE_Mobility.SetPositionAllocator(UE_Pos_list);

    //正常UE的移動模型設定
    //UE會移動 採用R2D的移動模型
    //每隔2秒改變方向跟速率
    //速度分布則是介於 0 ～ 2 m/s 呈均勻分布
    //範圍則是在房間裏面 
    //x 範圍 =  -room_size / 2 ~ room_size / 2
    //y 範圍 =  -room_size / 2 ~ room_size / 2
    UE_Mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
                             "Mode", StringValue ("Time"),
                             "Time", StringValue ("2.5s"),
                             "Speed", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=2.0]"),
                             "Direction",StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=6.283184]"),
                             "Bounds", RectangleValue (Rectangle (-room_size / 2, room_size / 2, -room_size / 2, room_size / 2)));



    /////////////////////////////////////////////////////////////////
    /////////////     Pressed UE : 被關在限制區域的UE      /////////////
    /////////////////////////////////////////////////////////////////


    //被關起來UE的MobilityModel
    MobilityHelper UE_Mobility_Pressed;

    //被關起來的UE的起始位置
    Ptr < ListPositionAllocator > UE_Pos_list_Pressed = CreateObject<ListPositionAllocator>();

    //隨機給出被限制UE的起始位置
    //因爲假設有一半UE不正常所以所以index 0~UE_Num/2
    for(int i = 0; i < UE_Num / 2; i++){

            //UE位置隨機生成
            //x = 0 ~ room size
            //y = 0 ~ room size
			
            //x,y坐標比起正常的UE多除2 代表這些被限制的UE 起始位置會被限制在左下角
            double x = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX / room_size)) / 2;
			double y = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX / room_size)) / 2;
			
            //但是要注意實際上房間左下角是（-room_size/2 , -room_size/2）
            //上面的的假設是房間左下角是（0,0）
            //所以x,y都還要做平移的坐標修正
            x -= room_size / 2;
            y -= room_size / 2;

            UE_Pos_list_Pressed->Add(Vector(x, y, UE_height));
    }

    //用剛剛的postion list 設定 PositionAllocator
    UE_Mobility_Pressed.SetPositionAllocator(UE_Pos_list_Pressed);

    UE_Mobility_Pressed.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
                             "Mode", StringValue ("Time"),
                             "Time", StringValue ("2.5s"),
                             "Speed", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=2.0]"),
                             "Direction",StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=6.283184]"),
                             "Bounds", RectangleValue (Rectangle (-room_size / 2, 0, -room_size / 2, 0)));

   
    //依照Normal/Pressed的交錯順序爲UE設定移動模型   
    for(int i=0 ; i < UE_Num ; i++){
        if(i%2 == 0)
            
            UE_Mobility.Install (UE_Nodes.Get(i));
        
        else

            UE_Mobility_Pressed.Install(UE_Nodes.Get(i));
    }

   

                   
}
#endif

void print_RF_AP_position(NodeContainer &RF_AP_Nodes){
  int RF_AP_Index = 1;
  for (NodeContainer::Iterator it = RF_AP_Nodes.Begin (); it != RF_AP_Nodes.End (); ++it){
        Ptr<MobilityModel> RF_MobilityModel = (*it)->GetObject<MobilityModel> ();
        Vector pos = RF_MobilityModel->GetPosition ();
        std::cout<<"Position of RF_AP "<< RF_AP_Index ++ <<" =("<<pos.x<<","<<pos.y<<","<<pos.z<<")"<<std::endl;
  }
  std::cout<<std::endl;
}

void print_VLC_AP_position(NodeContainer &VLC_AP_Nodes){
  int VLC_AP_Index = 1;
  for (NodeContainer::Iterator it = VLC_AP_Nodes.Begin (); it != VLC_AP_Nodes.End (); ++it){
        Ptr<MobilityModel> VLC_MobilityModel = (*it)->GetObject<MobilityModel> ();
        Vector pos = VLC_MobilityModel->GetPosition ();
        std::cout<<"Position of VLC_AP "<< VLC_AP_Index ++ <<" =("<<pos.x<<","<<pos.y<<","<<pos.z<<")"<<std::endl;
  }
  std::cout<<std::endl;
}

void print_UE_position(NodeContainer &UE_Nodes){
  int UE_Index = 1;
  for (NodeContainer::Iterator it = UE_Nodes.Begin (); it != UE_Nodes.End (); ++it){
        Ptr<MobilityModel> UE_MobilityModel = (*it)->GetObject<MobilityModel> ();
        Vector pos = UE_MobilityModel->GetPosition ();
        std::cout<<"Position of UE "<< UE_Index ++ <<" =("<<pos.x<<","<<pos.y<<","<<pos.z<<")"<<std::endl;
  }
  std::cout<<std::endl;
}