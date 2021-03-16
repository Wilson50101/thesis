#ifndef GLOBAL_ENVIROMENT_H
#define GLOBAL_ENVIROMENT_H
#define DEBUG_MODE 1            //控制是否要印出debug message
#define PI 3.14
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

// - power 20W
const int VLC_AP_Power = 20;

// - VLC bandwidth = 40Mhz
const int VLC_AP_Bandwidth = 40;

// - AWGN power spectral density Nl    10^-21 A2/Hz = 10^-15 A2/MHz
const double  Nl = 1e-15;



////////////////////////////////////////////////////////
/////////          Each UE                      ////////
////////////////////////////////////////////////////////
                                                              
//   - height 0.85m                            
const double UE_height = 0.85;                  

//   - required data rate
const double global_require_data_rate = 150 ;//Mbps




////////////////////////////////////////////////////////
/////////         VLC CHANNEL CONSTANTS         ////////
////////////////////////////////////////////////////////

// FOV range 70-130 step 10
const double VLC_field_of_view = 120;

// semi-angle at half-illumination (phi_1/2) 60
const int VLC_PHI_half = 60;

// gain of optical filter (g_of(psi))         1
const int VLC_filter_gain = 1;

// gain of optical concentrator (g_oc(psi))   1
const int VLC_concentrator_gain = 1;

// physical area for PD receiver              1 cm^2 = 0.0001 m^2
const double VLC_receiver_area = 0.0001;

// reflection efficiency (rho)                0.75
const double VLC_reflect_efficiency = 0.75;





#endif