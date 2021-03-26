#ifndef GLOBAL_ENVIROMENT_H
#define GLOBAL_ENVIROMENT_H
#define DEBUG_MODE 1            //控制是否要印出debug message
#define PI 3.14
#define BENCHMARK_HO_DESIGN 1   //控制handover overhead 要使用何種設定?
////////////////////////////////////////////////////////
/////////        ENVIRONMENT CONSTANTS          ////////
////////////////////////////////////////////////////////

// room size 40*40*3.5
const double room_size=40;

// - RF AP number : 1                              
const int RF_AP_Num = 1;   

// - VLC AP number : 4*4=16                              
const int VLC_AP_Num = 16;   

// - foreach row , VLC AP number = sqrt(VLC_AP_Num) //assume VLC_AP_Num is a square num
const int VLC_AP_per_row = 4;

// - UE number : 10-70 step 10                   
const int UE_Num = 40;

// -Time period in each round : 500 ms = 0.5s
const double Tp = 500; 

// -mean of handover overhead : 25ms ~ 175ms
const double meanHO = 75; 

////////////////////////////////////////////////////////
/////////          Each RF AP                  ////////
////////////////////////////////////////////////////////
                                                                    
// - height 3.5m                                
const double RF_AP_height = 3.5;               

// - power 1w
const int RF_AP_Power = 1;

// - RF bandwidth = 20Mhz
const int RF_AP_Bandwidth = 20;

// - AWGN power spectral density Nr    10^-21 A2/Hz = 10^-15 A2/MHz
const double Nr = 1e-15;

////////////////////////////////////////////////////////
/////////          Each VLC AP                  ////////
////////////////////////////////////////////////////////
                                                                    
// - height 3.5m                                
const double VLC_AP_height = 3.5;               

// - average trasmitted optical power per VLC AP : 9W
const int VLC_AP_Popt = 9;

// - VLC bandwidth = 40Mhz
const int VLC_AP_Bandwidth = 40;

// - power spectral density Nl    10^-21 A2/Hz = 10^-15 A2/MHz
const double  Nl = 1e-15;

// - Optical to electric conversion efficiency, kappa = 0.53 A/W
const double kappa = 0.53 ;

////////////////////////////////////////////////////////
/////////          Each UE                      ////////
////////////////////////////////////////////////////////
                                                              
//   - height 0.85m                            
const double UE_height = 0.85;                  

//   - Avg required data rate
//   - paper上的範圍約 29 ~ 33Mb/s = 232 ~ 264 Mbps
//   - 目前先取 30Mb/s = 240 Mps 
const double avg_require_data_rate = 240 ; // Mbps

//   - threshold = 6~12 Mb/s = 48 ~ 96Mbps
//   - 目前先定 9 Mb/s = 72Mbps
const double threshold = 72 ;//Mbps


////////////////////////////////////////////////////////
/////////         VLC CHANNEL CONSTANTS         ////////
////////////////////////////////////////////////////////

// FOV range 70-130 step 10
const double VLC_field_of_view = 180;

// semi-angle at half-illumination (phi_1/2) 60
const int VLC_PHI_half = 60;

// gain of optical filter (g_of(psi))         1
const int VLC_filter_gain = 1;

// refractive index = 1.5
const double VLC_refractive_index = 1.5;

// physical area for PD receiver              1 cm^2 = 0.0001 m^2
const double VLC_receiver_area = 0.0001;

// reflection efficiency (rho)                0.75
const double VLC_reflect_efficiency = 0.75;





#endif