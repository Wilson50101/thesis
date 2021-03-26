#ifndef MY_ALGO_H
#define MY_ALGO_H

#include <iostream>
#include <fstream>
#include <string>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"

#include "global_environment.h"
#include "My_UE_Node.h"

using namespace ns3;

std::vector<My_UE_Node> Initialize_My_UE_Node_list(NodeContainer  & UE_Nodes);

#endif