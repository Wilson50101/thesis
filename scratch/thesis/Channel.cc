// Q : 這份code在幹嘛？
// A : 算channel gain , SINR , datarate , handover efficiency 等要用到的 matrix

#include <iostream>
#include <fstream>
#include <string>
#include <chrono> //seed
#include <cmath>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "Channel.h"

//弧度轉角度
double RadtoDeg(const double & radian){
    return radian * 180 / PI;
}

//角度轉弧度
double DegtoRad(const double & degree){
    return degree * PI / 180;
}

//計算整個RF channel gain matrix
void Calculate_RF_Channel_Gain_Matrix(NodeContainer  & RF_AP_Node , NodeContainer  & UE_Nodes ,std::vector<std::vector<double>>  & RF_Channel_Gain_Matrix){
    for(int i=0 ; i < RF_AP_Num ; i++){
        for(int j=0 ; j < UE_Num ; j++){
            RF_Channel_Gain_Matrix[i][j] = Estimate_one_RF_Channel_Gain(RF_AP_Node.Get(i),UE_Nodes.Get(j));
        }
    }
}

//計算整個VLC channel gain matrix
void Calculate_VLC_Channel_Gain_Matrix(NodeContainer  & VLC_AP_Nodes , NodeContainer  &UE_Nodes ,std::vector<std::vector<double>>  &VLC_Channel_Gain_Matrix){
    for(int i=0 ; i < VLC_AP_Num ; i++){
        for(int j=0 ; j < UE_Num ; j++){
            VLC_Channel_Gain_Matrix[i][j] = Estimate_one_VLC_Channel_Gain(VLC_AP_Nodes.Get(i),UE_Nodes.Get(j));
        }
    }
}

//計算某個 pair(RF_AP,UE)的Channel gain
/*
    RF Channel gain 公式 ： 
    Gμ,α(f) = sqrt(10^(−L(d)/10)) * hr,

    hr is modelled by a Rayleigh variate on each OFDM sub-carrier with variance 2.46 dB
    
    L(d) = L(d0) + 10vlog10(d/d0) + X,
        L(d0) = 47.9 dB
        d0 = 1 m
        v = 1.6
        X is the shadowing component which is assumed to be a zero mean Gaussian distributed random variable with a standard deviation of 1.8 dB
 */
double Estimate_one_RF_Channel_Gain(Ptr<Node> RF_AP,Ptr<Node> UE){
    
    double d = GetDistance_AP_UE(RF_AP,UE);
    double d0 = 1 ; 
    double v = 1.6 ;

    //X is zero mean Gaussian distributed random variable with a standard deviation of 1.8 dB
    std::normal_distribution<double> Gaussian (0.0,1.8);    //normal distribution 即 Gaussian distribution       
    std::default_random_engine distribution(std::chrono::system_clock::now().time_since_epoch().count());
    double X = Gaussian(distribution);

   

    double L_d0 = 47.9 ;

    //算得L(d)
    double L_d = L_d0 +  10 * v * log10(d/d0) + X ;

    //Gμ,α(f) = sqrt(10^(−L(d)/10)) * hr,
    double rf_channel_gain = sqrt(pow(10,((-1) * L_d / 10))) ;

    //todo : hr我先沒有乘，因爲hr是Rayleigh distribution 這個分布C++沒有函數支援

    return rf_channel_gain;
}

//計算某個 pair(VLC_AP,UE)的Channel gain
double Estimate_one_VLC_Channel_Gain(Ptr<Node> VLC_AP , Ptr<Node> UE){
    
    double incidence_angle = Get_Incidence_Angle_AP_UE(VLC_AP,UE);
    
    
    double VLC_concentrator_gain = pow( VLC_refractive_index , 2 ) / pow(sin(DegtoRad(VLC_field_of_view / 2)),2);
    
    if( RadtoDeg(incidence_angle) >= VLC_field_of_view / 2)
      VLC_concentrator_gain = 0;

    //若入射角 >= FOV/2則Channel gain爲0
    if( RadtoDeg(incidence_angle) >= VLC_field_of_view / 2)
        return 0;

    //否則Channel gain不爲0,再來算

    //lambertian raidation coefficient m = -ln2 / ln(cos(Φ1/2))
    const double lambertian_coefficient = (-1) / (log(cos(DegtoRad(VLC_PHI_half))));    //cos只吃弧度,所以要轉
    
    /*  輻射角角度同入射角
        輻射角 (incidence) AP射出的角度

            AP  --------
               |\      |
               | \     |
               |Φ \    |
               |   \   |
               |    \Ψ |
               |     \ |
               |      \|
                       UE

        Φ即為輻射角,又因為兩條垂直線平行,所以Φ=Ψ
        故這裡直接用入射角值assign即可

    */
    const double irradiance_angle = incidence_angle;

    //取得AP的位置
    Ptr<MobilityModel> VLC_AP_MobilityModel = VLC_AP->GetObject<MobilityModel> ();
    Vector VLC_AP_Pos = VLC_AP_MobilityModel->GetPosition ();
    
    //取得UE的位置
    Ptr<MobilityModel> UE_MobilityModel = UE->GetObject<MobilityModel> ();
    Vector UE_Pos = UE_MobilityModel->GetPosition ();

    const double height_diff = VLC_AP_Pos.z - UE_Pos.z;

    const double distance = GetDistance_AP_UE(VLC_AP,UE);

    double channel_gain = ( lambertian_coefficient + 1) / ( 2 * PI * pow(distance,2) ) * VLC_receiver_area ; // first term

    channel_gain  = channel_gain * VLC_concentrator_gain; // second term g(theta) 
    
    channel_gain  = channel_gain * VLC_filter_gain ; // third term Ts(theta) 

    channel_gain  = channel_gain * pow( cos(irradiance_angle) , lambertian_coefficient ); // forth term [cos(phi)]^m

    channel_gain  = channel_gain * cos(incidence_angle); // last term

    return channel_gain;
}


//計算某個 pair(AP,UE)在3維空間的距離 
double GetDistance_AP_UE(Ptr<Node> AP ,Ptr<Node> UE){
    
    //取得AP的位置
    Ptr<MobilityModel> AP_MobilityModel = AP->GetObject<MobilityModel> ();
    Vector AP_Pos = AP_MobilityModel->GetPosition ();
    
    //取得UE的位置
    Ptr<MobilityModel> UE_MobilityModel = UE->GetObject<MobilityModel> ();
    Vector UE_Pos = UE_MobilityModel->GetPosition ();

    //MobilityModel有內建的計算距離公式
    return AP_MobilityModel->GetDistanceFrom (UE_MobilityModel);
}


//計算某個 pair(VLC_AP,UE)之間的入射角弧度 （incidence angle in Rad） 
/*
        這裡採用餘弦定理來算角度，如下圖所示:

            plane_dist : c
            AP -------
               \      |
                \     |
  hypotenuse:b   \    |   height_diff:a
                  \   |
                   \θ |
                    \ |
                     \|
                     UE

        a^2+b^2-c^2=2ab*cosθ
        通過邊長反推角度
        θ=cos^(-1)( (a^2+b^2-c^2)/2ab )
*/
double Get_Incidence_Angle_AP_UE(Ptr<Node> AP, Ptr<Node> UE){
    
    //取得AP的位置
    Ptr<MobilityModel> AP_MobilityModel = AP->GetObject<MobilityModel> ();
    Vector AP_Pos = AP_MobilityModel->GetPosition ();
    
    //取得UE的位置
    Ptr<MobilityModel> UE_MobilityModel = UE->GetObject<MobilityModel> ();
    Vector UE_Pos = UE_MobilityModel->GetPosition ();
    
    
    //高度差 a
    const double height_diff = AP_Pos.z - UE_Pos.z; 
    
    //xy在平面上的距離 c
    double dx=AP_Pos.x-UE_Pos.x;
    double dy=AP_Pos.y-UE_Pos.y;
    const double plane_diff = sqrt(pow(dx,2)+pow(dy,2)); 

    //斜邊 b
    const double hypotenuse = sqrt(pow(height_diff,2)+pow(plane_diff,2));

    //已知三角形三邊a,b,c，求ab兩邊之間夾角角度-->採用餘弦定理
    const double angle = acos((pow(height_diff,2) + pow(hypotenuse,2) - pow(plane_diff,2)) / (2*height_diff*hypotenuse));


    //回傳的是弧度
    return angle;
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

//計算handover後的有效傳輸比例Handover efficiency Matrix
/*
    Matrix :
        R0 V0.....
    R0
    V0
    ..
    ..
    ..
*/
void Calculate_Handover_Efficiency_Matrix(std::vector<std::vector<double>> & Handover_Efficiency_Matrix){
  
  // TODO：我這裡寫兩種算法到時候在跟老師討論要採用哪一種

  // 做法1 ： 參考Mobility Management for Hybrid LiFi and WiFi Networks in the Presence of Light-path Blockage
  // 將HHO跟VHO分開計算
  // 這種做法較爲合理，但不是我的benmark的做法
  
  //要算出Handover_Efficiency_Matrix之前要先隨機出Handover_Overhead才可計算
  std::vector<std::vector<int>> Handover_Overhead_Matrix(RF_AP_Num + VLC_AP_Num,std::vector<int> (RF_AP_Num + VLC_AP_Num,0));

  #if ! BENCHMARK_HO_DESIGN
  //Handover分兩種
  //假設前提 :  Tp = 500ms
  //Vertical handover overhead LiFi <-> WiFi之間互相切換所產生的 cost  : 平均時間 250ms
  //Horizontal handover overhead LiFi <-> LiFi or WiFi <-> WiFi 之間互相切換所產生的 cost  : 平均時間 100ms
  
  std::poisson_distribution<int> TVHO (250);       //Vertical handover overhead 平均爲250ms的poisson distribution
  std::poisson_distribution<int> THHO (100);       //Horizontal handover overhead  平均爲100ms的poisson distribution
  std::default_random_engine generator(std::chrono::system_clock::now().time_since_epoch().count());
  

  // 給 Handover_Overhead_Matrix 隨機的overhead
  for(int i=0 ; i < RF_AP_Num + VLC_AP_Num ; i++){
    
    for(int j=0 ; j < RF_AP_Num + VLC_AP_Num ; j++){
      
      //如果i==j代表沒有AP沒變，沒有換手，也就沒有overhead
      if( i == j )
        Handover_Overhead_Matrix[i][j] = 0;

      //i!=j代表AP有變，產生換手效應，overhead有兩種可能
      else
      {
          //如果是RF AP（index 0 ~ RF_AP_Num-1）之間切換 --> 屬於 Horizontal handover overhead
          if(i < RF_AP_Num && j < RF_AP_Num)
            
            //THHO隨機出一個數賦值
            Handover_Overhead_Matrix[i][j] = THHO(generator);

          
          //如果是VLC AP (index RF_AP_Num ~ RF_AP_Num + VLC_AP_Num-1）之間切換 --> 也屬於 Horizontal handover overhead
          else if(i >= RF_AP_Num && j >= RF_AP_Num)
            
            //THHO隨機出一個數賦值
            Handover_Overhead_Matrix[i][j] = THHO(generator);

          //若皆非以上兩者情況-->則爲LiFi <-> WiFi AP之間切換 --> 屬於 Vertical handover overhead
          else  
            //TVHO隨機出一個數賦值
            Handover_Overhead_Matrix[i][j] = TVHO(generator);
      } 
    }
  }
  #else
  // 做法2 ： 參考我的benchmark -> Dynamic Load Balancing for Hybrid Li-Fi and RF Indoor Networks
  // 並沒有分HHO跟VHO，overhead的mean 25～175ms不等
  // 做法1比較合理，但是這是benchmark做法，先寫起來放在看要怎麼樣
  
  std::poisson_distribution<int> HO (meanHO);      
  std::default_random_engine generator(std::chrono::system_clock::now().time_since_epoch().count());
  
  // 給 Handover_Overhead_Matrix 隨機的overhead
  for(int i=0 ; i < RF_AP_Num + VLC_AP_Num ; i++){
    
    for(int j=0 ; j < RF_AP_Num + VLC_AP_Num ; j++){
      
      //如果i==j代表沒有AP沒變，沒有換手，也就沒有overhead
      if( i == j )
        Handover_Overhead_Matrix[i][j] = 0;

      //i!=j代表AP有變，產生換手效應，有overhead產生
      else
        
        //隨機出一個數賦值
        Handover_Overhead_Matrix[i][j] = HO(generator);
    } 
  }
  #endif

  //接下來要計算 Handover_Efficiency_Matrix 即納入overhead後 實際上的有效傳輸比例
  for(int i=0 ; i < RF_AP_Num + VLC_AP_Num ; i++){
    
    for(int j=0 ; j < RF_AP_Num + VLC_AP_Num ; j++){
        
        //handover efficiency = max(0 , 1 - overhead / 每一輪的period Tp)
        Handover_Efficiency_Matrix[i][j] = (1 - ((double)Handover_Overhead_Matrix[i][j] / Tp)) > 0 ? 1 - ((double)Handover_Overhead_Matrix[i][j] / Tp) : 0;

    }
  
  }

}
