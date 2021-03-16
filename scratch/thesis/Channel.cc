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
    double rf_channel_gain = sqrt(pow(10,(-1) * L_d /10)) ;

    //todo : hr我先沒有乘，因爲hr是Rayleigh distribution 這個分布C++沒有函數支援

    return rf_channel_gain;
}

//計算某個 pair(VLC_AP,UE)的Channel gain
double Estimate_one_VLC_Channel_Gain(Ptr<Node> VLC_AP , Ptr<Node> UE){
    double incidence_angle = Get_Incidence_Angle_AP_UE(VLC_AP,UE);
    
    //若入射角 >= FOV/2則Channel gain爲0
    if( RadtoDeg(incidence_angle) >= VLC_field_of_view/2 )
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

    double channel_gain = VLC_receiver_area * ( lambertian_coefficient + 1) / ( 2 * PI * pow(distance,2) ); // first term

    channel_gain  = channel_gain * pow( cos(irradiance_angle) , lambertian_coefficient ); // second term

    channel_gain  = channel_gain * VLC_filter_gain * VLC_concentrator_gain; // third and fourth term

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