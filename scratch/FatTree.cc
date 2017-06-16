/*
 * Author: Morteza Kheirkhah <m.kheirkhah@sussex.ac.uk>
 */
#include <ctime>
#include <sys/time.h>
#include <stdint.h>
#include <fstream>
#include <string>
#include <cassert>
#include <iostream>
#include <iomanip>
#include "ns3/log.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/internet-module.h"
#include "ns3/netanim-module.h"
#include "ns3/mobility-module.h"
#include "ns3/callback.h"
#include "ns3/string.h"

using namespace ns3;
NS_LOG_COMPONENT_DEFINE("FatTree");

typedef enum
{
  Core, Aggr, Tor, Host, Core_Aggr, Core_Aggr_Tor, Core_Aggr_Tor_Host
} Layers_t;

typedef enum
{
  NONE, PERMUTATION, STRIDE, RANDOM, SHORT_FLOW, INCAST
} TrafficMatrix_t;

typedef enum
{
  TCP, MPTCP, SCATTER, MMPTCP
} SocketType_t;

typedef enum
{
  SHORT, LARGE, BACKGROUND
} FlowType_t;

std::map<string, TrafficMatrix_t> stringToTrafficMatrix;
std::map<string, SocketType_t> stringToSocketType;
std::map<string, Layers_t> stringToHotSpotLayers;
vector<ApplicationContainer> sourceShortFlowApps;
vector<ApplicationContainer> sourceLargeFlowApps;
vector<ApplicationContainer> sinkApps;

// Setup general parameters
string g_topology = "FT";
string g_simName  = "S0";
string g_linkCapacity = "100Mbps";  // 100Mbps
string g_linkDelay = "20us";        // RTT is 240us
uint32_t g_flowSize = 0;            // 0 Unlimited used for large flows
uint32_t g_shortFlowSize = 70000;   // 70Kb
uint32_t g_simTime = 20;             // 20 Seconds (default)
uint32_t g_seed = 0;                // RNG seed
double g_lamda = 256;               // 250 ms (Poisson arrival)
uint32_t g_FlowSizeThresh = 100000; // 100KB
SocketType_t g_socketType = MMPTCP;
TrafficMatrix_t g_trafficMatrix = SHORT_FLOW;
TrafficMatrix_t g_shortFlowTM = PERMUTATION;
TrafficMatrix_t g_backgroundFlowTM = NONE;

uint32_t g_incastNumber = 10;
double g_incastHeartbeat = 0.5;     // 500ms
uint32_t g_connxLimit = 33;         // 33% large flows if it being used
string g_simInstance = "0";
string g_simStartTime = "NULL";
uint32_t g_dupAckThresh = 0;
uint32_t g_minRTO = 200;
bool g_enableAutoDupAck = true;
bool g_enableMinRTO = false;
bool g_enableLimitedTx = false;
bool g_enableLfPlotting = false;
bool g_enableSfPlotting = false;
bool g_alphaPerAck = false;
double g_arrivalUpperBound = 0.2;
bool g_shortFlowTCP = false;
bool g_enableHotspots = false;
uint32_t g_hotspotRatio = 15;       // 15% of total Nodes in the Container
uint32_t g_hotspotQueueSize = 50;
Layers_t g_hotspotLayer = Core;
double g_shortFlowStartTime = 0.0;
bool g_switchCondition = false;
string g_switchMode = "FlowSize";

double g_sfLowerSize = 1 * 1024;        //1024B    - 1KB (Only for MMPTCP)
double g_sfUpperSize = 1 * 1024 * 1024; //1048576B - 1MB (Only for MMPTCP)

uint32_t g_rGap = 50;

bool g_shortFlowSizeUniRandom = true;
bool g_shortFlowSizeDist = false;

uint16_t g_torLayerConnx = 0;
uint16_t g_aggrLayerConnx = 0;
uint16_t g_coreLayerConnx = 0;

uint32_t AllFlow[3];
uint32_t LargeFlow[3];
uint32_t ShortFlow[3];

// Setup MPTCP parameters
string g_cc = "RTT_Compensator"; // Uncoupled_TCPs, Linked_Increases, Fully_Coupled
uint32_t g_subflows = 8;
uint32_t g_shortFlowSubflows = 1;
uint32_t g_initialCWND = 1;

// Setup topology parameters
uint32_t g_ratio = 1;
uint32_t g_K = 8;                           // Ports of all switches:       8
uint32_t g_numPod = g_K;                    // Pods:                        8
uint32_t g_numHost = (g_K / 2) * g_ratio;   // Host per ToR switch:         4
uint32_t g_numToR = g_K / 2;                // ToR  switches per pod:       4
uint32_t g_numAggr = g_K / 2;               // Aggr switches per pod:       4
uint32_t g_numCore = g_K / 2;               // Core switches in group:      2
uint32_t g_numGroup = g_K / 2;              // Core switches in group:      2
uint32_t g_totalHost = ((g_K * g_K * g_K) / 4) * g_ratio;
uint32_t g_totalToR = g_numToR * g_numPod;
uint32_t g_totalAggr = g_numAggr * g_numPod;
uint32_t g_totalCore = g_numCore * g_numGroup;

struct cdfStruct
{ // contains points of the CDF
  double yCDF;
  int xFlowSize;
};
vector<cdfStruct> cdfData;  // flow size CDF

// Function to create address string from numbers
uint32_t total_bytes[200][120];
uint32_t drop_bytes [200][120];

// [Second][Metrics][Node][Dev]
double core_data[25][2][1024][64];
double aggr_data[25][2][1024][64];
double tor_data [25][2][1024][64];
double host_data[25][2][1024][2];
//
double totalCoreUtil, totalCoreLoss, totalAggrUtil, totalAggrLoss, totalTorUtil, totalTorLoss, totalHostUtil, totalHostLoss = 0.0;
double meanCoreUtil, meanCoreLoss, meanAggrUtil, meanAggrLoss, meanTorUtil, meanTorLoss, meanHostUtil, meanHostLoss = 0.0;

// NodeContainer to use for link utilization and loss rate
NodeContainer core_c;
NodeContainer Aggr_c;
NodeContainer Tor_c;
NodeContainer Host_c;

void SimHeaderWritter(Ptr<OutputStreamWrapper> stream);
void SimFooterWritter(Ptr<OutputStreamWrapper> stream);

string
isActive(bool param)
{
  if (param)
    return "On";
  else
    return "Off";
}

uint64_t
GetLinkRate(string linkRate)
{
  DataRate tmp(linkRate);
  return tmp.GetBitRate();
}

// PERMUTATION, STRIDE, RANDOM, SHORT_FLOW
void
SetupStringToTM()
{
  stringToTrafficMatrix["PERMUTATION"] = PERMUTATION;
  stringToTrafficMatrix["SHORT_FLOW"] = SHORT_FLOW;
  stringToTrafficMatrix["STRIDE"] = STRIDE;
  stringToTrafficMatrix["RANDOM"] = RANDOM;
  stringToTrafficMatrix["INCAST"] = INCAST;
  stringToTrafficMatrix["NONE"] = NONE;
}

void
SetupStringToST()
{
  stringToSocketType["TCP"] = TCP;
  stringToSocketType["MPTCP"] = MPTCP;
  stringToSocketType["MMPTCP"] = MMPTCP;
  stringToSocketType["SCATTER"] = SCATTER;
}

void
SetupStringToHSL()
{
  stringToHotSpotLayers["Host"] = Host;
  stringToHotSpotLayers["Tor"]  = Tor;
  stringToHotSpotLayers["Aggr"] = Aggr;
  stringToHotSpotLayers["Core"] = Core;
}

string
GetKeyFromValueTM(TrafficMatrix_t tm)
{
  map<string, TrafficMatrix_t>::const_iterator it = stringToTrafficMatrix.begin();
  for (; it != stringToTrafficMatrix.end(); it++)
    {
      if (it->second == tm)
        return it->first;
    }
  return "";
}

string
GetKeyFromValueST(SocketType_t st)
{
  map<string, SocketType_t>::const_iterator it = stringToSocketType.begin();
  for (; it != stringToSocketType.end(); it++)
    {
      if (it->second == st)
        return it->first;
    }
  return "";
}

string
GetKeyFromValueHSL(Layers_t tl)
{
  map<string, Layers_t>::const_iterator it = stringToHotSpotLayers.begin();
  for (; it != stringToHotSpotLayers.end(); it++)
    {
      if (it->second == tl)
        return it->first;
    }
  return "";
}

string
SetupSimFileName(string input)
{
  ostringstream oss;
  oss.str("");
  oss << g_simName << "_" << g_topology << "_"<< g_totalHost <<  "_" <<GetKeyFromValueST(g_socketType) << "_" << GetKeyFromValueTM(g_trafficMatrix) << "_" << input << "_" << g_simInstance
      << ".data";
  string tmp = oss.str();
  oss.str("");
  return tmp;
}

uint32_t
GetReTxThresh(Ipv4Address ipv4Src, Ipv4Address ipv4Dst)
{
  uint8_t src[4];
  ipv4Src.Serialize(src);
  uint8_t dst[4];
  ipv4Dst.Serialize(dst);
  if (src[1] == dst[1] && src[2] == dst[2])
    {
      return 0;
    }
  else if (src[1] == dst[1])
    {
      return g_numAggr; // 4
    }
  else
    {
      return g_totalCore; // 16
    }
}

void
cmAnalisys(FlowType_t ft, Ipv4Address ipv4Src, Ipv4Address ipv4Dst)
{
  uint8_t src[4];
  ipv4Src.Serialize(src);
  uint8_t dst[4];
  ipv4Dst.Serialize(dst);

  switch (ft)
    {
  case SHORT:
    if (src[1] == dst[1] && src[2] == dst[2])
      {
        ShortFlow[0]++;
        AllFlow[0]++;
      }
    else if (src[1] == dst[1])
      {
        ShortFlow[1]++;
        AllFlow[1]++;
      }
    else if (src[1] != dst[1])
      {
        ShortFlow[2]++;
        AllFlow[2]++;
      }
    else
      exit(1);
    break;
  case LARGE:
    if (src[1] == dst[1] && src[2] == dst[2])
      {
        LargeFlow[0]++;
        AllFlow[0]++;
      }
    else if (src[1] == dst[1])
      {
        LargeFlow[1]++;
        AllFlow[1]++;
      }
    else if (src[1] != dst[1])
      {
        LargeFlow[2]++;
        AllFlow[2]++;
      }
    else
      exit(1);
    break;
  default:
    exit(1);
    break;
    }
}

void
OutPutCore()
{
  Ptr<OutputStreamWrapper> stream = Create<OutputStreamWrapper>(SetupSimFileName("CORE"), std::ios::out);
  ostream *os = stream->GetStream();
  for (uint32_t s = 0; s <= g_simTime; s++)                          // [Seconds]
    {
      *os << s;
      for (uint32_t m = 0; m < 2; m++)                               // [Metrics]
        {
          for (uint32_t n = 0; n < core_c.GetN(); n++)               // [Node]
            {
              for (uint32_t d = 1; d <= g_K; d++)                    // [Dev]
                {

                  if (s == 0)
                    *os << " " << "Link" << d;
                  else
                    {
                      *os << "    " << core_data[s][m][n][d];
                      if (m == 0)            // Utilization
                        totalCoreUtil += core_data[s][m][n][d];
                      else if (m == 1)       // Loss
                        totalCoreLoss += core_data[s][m][n][d];
                    }
                }
            }
        }
      *os << endl;
    }
}

void
OutPutAggr()
{
  Ptr<OutputStreamWrapper> stream_aggr = Create<OutputStreamWrapper>(SetupSimFileName("AGGR"), std::ios::out);
  ostream *os_aggr = stream_aggr->GetStream();
  for (uint32_t s = 0; s <= g_simTime; s++)              // [Seconds]
    {
      *os_aggr << s;
      for (uint32_t m = 0; m < 2; m++)                   // [Metrics]
        {
          for (uint32_t n = 0; n < Aggr_c.GetN(); n++)   // [Node]
            {
              for (uint32_t d = 1; d <= g_K; d++)        // [Dev]
                {

                  if (s == 0)
                    *os_aggr << " " << "Link" << d;
                  else
                    {
                      *os_aggr << "    " << aggr_data[s][m][n][d];
                      if (m == 0)       // Utilization
                        totalAggrUtil += aggr_data[s][m][n][d];
                      else if (m == 1)  // Loss
                        totalAggrLoss += aggr_data[s][m][n][d];
                    }
                }
            }
        }
      *os_aggr << endl;
    }
}

void
OutPutTor()
{
  Ptr<OutputStreamWrapper> stream_tor = Create<OutputStreamWrapper>(SetupSimFileName("TOR"), std::ios::out);
  ostream *os_tor = stream_tor->GetStream();
  for (uint32_t s = 0; s <= g_simTime; s++)                           // [Seconds]
    {
      *os_tor << s;
      for (uint32_t m = 0; m < 2; m++)                                // [Metrics]
        {
          for (uint32_t n = 0; n < Tor_c.GetN(); n++)                 // [Node]
            {
              for (uint32_t d = 1; d <= g_K; d++)                     // [Dev]
                {
                  if (s == 0)
                    *os_tor << " " << "Link" << d;
                  else
                    {
                      *os_tor << "    " << tor_data[s][m][n][d];
                      if (m == 0)            // ToR Utilization
                        totalTorUtil += tor_data[s][m][n][d];
                      else if (m == 1)       // ToR Loss
                        totalTorLoss += tor_data[s][m][n][d];
                    }
                }
            }
        }
      *os_tor << endl;
    }
}

void
OutPutHost(){
  Ptr<OutputStreamWrapper> stream_host = Create<OutputStreamWrapper>(SetupSimFileName("HOST"), std::ios::out);
    ostream *os_host = stream_host->GetStream();
    for (uint32_t s = 0; s <= g_simTime; s++)                   // [Seconds]
      {
        *os_host << s;
        for (uint32_t m = 0; m < 2; m++)                        // [Metrics]
          {
            for (uint32_t n = 0; n < Host_c.GetN(); n++)        // [Node]
              {
                for (uint32_t d = 1; d <= 1; d++)               // [Dev]
                  {
                    if (s == 0)
                      *os_host << " " << "Link" << d;
                    else
                      {
                        *os_host << "    " << host_data[s][m][n][d];
                        if (m == 0)       // host Utilization
                          totalHostUtil += host_data[s][m][n][d];
                        else if (m == 1)  // host Loss
                          totalHostLoss += host_data[s][m][n][d];
                      }
                  }
              }
          }
        *os_host << endl;
      }
}

void
PrintCMStat()
{
  const char * format = "%s \t%.1f \t%.1f \t%.1f \n";
  printf("\n");
  printf("FlowType\tTOR\tAggr\tCore\n");
  printf("--------\t----\t----\t----\n");
  printf(format, "Large   ", round(LargeFlow[0]), round(LargeFlow[1]), round(LargeFlow[2]));
  printf(format, "Short   ", round(ShortFlow[0]), round(ShortFlow[1]), round(ShortFlow[2]));
  printf(format, "All     ", round(AllFlow[0]), round(AllFlow[1]), round(AllFlow[2]));
  printf("\n");
}

void
PrintSimParams()
{
  string ltx = isActive(g_enableLimitedTx);
  string mrto = isActive(g_enableMinRTO);
  string autoDupack = isActive(g_enableAutoDupAck);
  string shortFlowPlot = isActive(g_enableSfPlotting);
  string largeFlowPlot = isActive(g_enableLfPlotting);
  string alphaPerAck = isActive(g_alphaPerAck);
  string hotspot = isActive(g_enableHotspots);
  cout << endl;
  cout << "Socket Type      : " << GetKeyFromValueST(g_socketType).c_str() << endl;
  cout << "Traffic Matrix   : " << GetKeyFromValueTM(g_trafficMatrix).c_str() << endl;
  cout << "ShortFlow TM     : " << GetKeyFromValueTM(g_shortFlowTM).c_str() << endl;
  cout << "Bandwidth Ratio  : " << g_ratio << ":1" << endl;
  cout << "LargeFlows       : " << g_connxLimit << "% of totalhosts" << endl;
  cout << "Seed             : " << g_seed << endl;
  cout << "Instance         : " << g_simInstance << endl;
  cout << "LimitedTx        : " << ltx.c_str() << endl;
  cout << "RTOmin           : " << mrto.c_str() << endl;
  cout << "Auto DupAck      : " << autoDupack.c_str() << endl;
  cout << "ShortFlow Plot   : " << shortFlowPlot.c_str() << endl;
  cout << "LargeFlow Plot   : " << largeFlowPlot.c_str() << endl;
  cout << "Alpha per ACK    : " << alphaPerAck.c_str() << endl;
  cout << "Hotspot          : " << hotspot.c_str() << endl;
  cout << "Incast Number    : " << g_incastNumber << endl;
}

void
SimTimeMonitor()
{
  NS_LOG_UNCOND("ClockTime: " << Simulator::Now().GetSeconds());
  double now = Simulator::Now().GetSeconds();
  cout << "[" << g_simName << "](" << g_topology << "){" << g_totalHost << "}[" << GetKeyFromValueST(g_socketType) << "]{"
      << GetKeyFromValueTM(g_trafficMatrix)
      << "} -> SimClock: " << now << endl;
  if (now < g_simTime)
    Simulator::Schedule(Seconds(0.1), &SimTimeMonitor);
}

vector<connection*>*
GetShortCM(vector<connection*>* CM)
{
  vector<connection*>* ret = new vector<connection*>();
  vector<connection*>::iterator it;

  for (it = (*CM).begin(); it != (*CM).end(); it++)
    {
      if ((*it)->large == true)
        {
          continue;
        }
      else
        ret->push_back((*it));
    }
  return ret;
}

uint32_t
GetShortFlowSize()
{
   if (g_socketType == MMPTCP)
    {
      static UniformVariable uniformFlowSize (g_sfLowerSize/1024, g_sfUpperSize/1024);  // 1KB ~ 1024KB, lasting max ~20ms @ 100Mbps
      return static_cast<uint32_t> (uniformFlowSize.GetValue () * 1024);  // 1024~ 1048576 B
    }
   static UniformVariable uniformFlowSize (1, g_FlowSizeThresh/1024); // 1KB ~ 1024KB, lasting max ~20ms @ 100Mbps
   return static_cast<uint32_t> (uniformFlowSize.GetValue () * 1024); // 1024~ 1048576 B
}

bool
GetCDFData()
{
    cdfStruct temp = {0.0, 0};
    cdfData.assign(12, temp);
    cdfData.at(0).xFlowSize  = 1; cdfData.at(0).yCDF  = 0.0;
    cdfData.at(1).xFlowSize  = 1; cdfData.at(1).yCDF  = 0.5;
    cdfData.at(2).xFlowSize  = 2; cdfData.at(2).yCDF  = 0.6;
    cdfData.at(3).xFlowSize  = 3; cdfData.at(3).yCDF  = 0.7;
    cdfData.at(4).xFlowSize  = 5; cdfData.at(4).yCDF  = 0.75;
    cdfData.at(5).xFlowSize  = 7; cdfData.at(5).yCDF  = 0.8;
    cdfData.at(6).xFlowSize  = 40;  cdfData.at(6).yCDF  = 0.8125;
    cdfData.at(7).xFlowSize  = 72;  cdfData.at(7).yCDF  = 0.8250;
    cdfData.at(8).xFlowSize  = 137; cdfData.at(8).yCDF  = 0.85;
    cdfData.at(9).xFlowSize  = 267; cdfData.at(9).yCDF  = 0.9;
    cdfData.at(10).xFlowSize = 1187;  cdfData.at(10).yCDF = 0.95;
    cdfData.at(11).xFlowSize = 2107;  cdfData.at(11).yCDF = 1.0;
    return true;
}

uint64_t
ExtrapolateFlowSize (double c)
{ // finds the correct flow size to attribute to a particular value of CDF
  // returns the size of the flow to be generated, in packets
  double diff = 1.0;  // maximum difference for any two CDF value is 1
  int index = 0;
  int size = 12; // size of the CDF array

  // find the closest value of CDF to c
  for (int i = 0; i < size; i++)
    {
      if (abs (cdfData.at (i).yCDF - c) < diff)
        {
          // new smallest found
          diff = abs (cdfData.at (i).yCDF - c);
          index = i;
        }
      else
        {
          break;
        }
    }
  // use linear extrapolation to find a value between the points
  // use this equation X = (Y - Yk-1)/(Yk - Yk-1) * (Xk - Xk-1) + Xk-1
  // X is the desired flow size and Y is the actual CDF from random number
  // need to assign only correct values for X, Y, Xk, Yk, Xk-1, Yk-1
  double x, y, xk, yk, xk1, yk1;
  y = c;
  diff = -(cdfData.at (index).yCDF - y);
  if (diff == 0.0)
    { // rare, but can happen, give the exact flow size from CDF
      xk = cdfData.at (index).xFlowSize;
      yk = cdfData.at (index).yCDF;
      xk1 = 0.0;
      yk1 = 0.0;
    }
  else if (diff < 0.0)
    { // between index and previous sample
      xk = cdfData.at (index).xFlowSize;
      yk = cdfData.at (index).yCDF;
      xk1 = cdfData.at (index - 1).xFlowSize;
      yk1 = cdfData.at (index - 1).yCDF;
    }
  else if (diff > 0.0)
    { // between index and next sample
      xk = cdfData.at (index + 1).xFlowSize;
      yk = cdfData.at (index + 1).yCDF;
      xk1 = cdfData.at (index).xFlowSize;
      yk1 = cdfData.at (index).yCDF;
    }
  // calculate flow size
  if (y == 0)
    {
      x = 1;    // unlikely but can cause problem if not accounted for
    }
  else
    {
      x = ((y - yk1) / (yk - yk1)) * (xk - xk1) + xk1;  // see above for comment pls
    }
  // return flow size in packets
  return (uint64_t) round (x);
}

void
ShortFlowConfigForIncast(vector<connection*>* CM, const NodeContainer &allHosts)
{
  cout << "INCAST SCENARIO START - SimulationNow: " << Simulator::Now().GetSeconds() << endl;
  int cmSize = (*CM).size();
  connection* connx;
  double nextStop;
  for (int i = 0; i < cmSize; i++)
    {
      connx = (*CM).at(i);

      int src = connx->src;
      int dst = connx->dst;

      assert(connx->large == false);

      Ptr<Node> srcNode = allHosts.Get(src);
      Ptr<Ipv4> ipv4Src = srcNode->GetObject<Ipv4>();
      Ipv4InterfaceAddress ipv4InterfaceAddressSrc = ipv4Src->GetAddress(1, 0);
      Ipv4Address ipv4AddressSrc = ipv4InterfaceAddressSrc.GetLocal();

      // dst setup
      Ptr<Node> dstNode = allHosts.Get(dst);
      Ptr<Ipv4> ipv4Dst = dstNode->GetObject<Ipv4>();
      Ipv4InterfaceAddress ipv4InterfaceAddressDst = ipv4Dst->GetAddress(1, 0);
      Ipv4Address ipv4AddressDst = ipv4InterfaceAddressDst.GetLocal();

      // Assign flowId
      int flowId = srcNode->GetNApplications();

      // Source
      MpTcpBulkSendHelper source("ns3::TcpSocketFactory", InetSocketAddress(Ipv4Address(ipv4AddressDst), dstNode->GetId() + 1));
      source.SetAttribute("MaxBytes", UintegerValue(g_shortFlowSize));
      source.SetAttribute("FlowId", UintegerValue(flowId));
      source.SetAttribute("MaxSubflows", UintegerValue(g_shortFlowSubflows));
      source.SetAttribute("FlowType", StringValue("Short"));
      string tempString = SetupSimFileName("RESULT");
      source.SetAttribute("OutputFileName", StringValue(tempString));

      if (g_enableAutoDupAck && (g_socketType == MMPTCP || g_socketType == SCATTER))
        {
          g_dupAckThresh = GetReTxThresh(ipv4AddressSrc, ipv4AddressDst);
        }
      source.SetAttribute("DupAck", UintegerValue(g_dupAckThresh));
      cmAnalisys(SHORT, ipv4AddressSrc, ipv4AddressDst);
      ApplicationContainer tmp = source.Install(srcNode);

      // Start
      tmp.Get(0)->SetStartTime(Seconds(0));

      // STOP
      double diff = (double) g_simTime - Simulator::Now().GetSeconds();
      nextStop = (diff > 0) ? diff : 0;
      tmp.Get(0)->SetStopTime(Seconds(nextStop));

      sourceShortFlowApps.push_back(tmp);
      cout << "[" << GetKeyFromValueST(g_socketType) << "]{" << GetKeyFromValueTM(g_shortFlowTM) << "} StartNow: "
                    << Simulator::Now().GetSeconds() << " Stop: " << nextStop << " (" << src << " -> " << dst << ") DupAck: "
                    << g_dupAckThresh << endl;
    }
  // Schedule next arrival
  if (Simulator::Now().GetSeconds() < (static_cast<double>(g_simTime) - g_arrivalUpperBound))
    {
      if (((static_cast<double>(g_simTime) - g_arrivalUpperBound) - Simulator::Now().GetSeconds()) > g_incastHeartbeat)
        {
          cout << "INCAST SCENARIO NEXT ARRIVAL - SimulationNow: "
              << Simulator::Now().GetSeconds() << " NextIncastHeartbeat: " << Simulator::Now().GetSeconds()+ g_incastHeartbeat << endl;
          Simulator::Schedule(Seconds(g_incastHeartbeat), &ShortFlowConfigForIncast, CM, allHosts);
        }
    }
}

void
ShortFlowConfig(vector<connection*>* CM, const NodeContainer &allHosts)
{
  int cmSize = (*CM).size();
  int pos;
  connection* connx;
  pos = rand() % cmSize;
  connx = (*CM).at(pos);
  int src = connx->src;
  int dst = connx->dst;
  assert(connx->large == false);

  // src setup
  Ptr<Node> srcNode = allHosts.Get(src);
  Ptr<Ipv4> ipv4Src = srcNode->GetObject<Ipv4>();
  Ipv4InterfaceAddress ipv4InterfaceAddressSrc = ipv4Src->GetAddress(1, 0);
  Ipv4Address ipv4AddressSrc = ipv4InterfaceAddressSrc.GetLocal();

  // dst setup
  Ptr<Node> dstNode = allHosts.Get(dst);
  Ptr<Ipv4> ipv4Dst = dstNode->GetObject<Ipv4>();
  Ipv4InterfaceAddress ipv4InterfaceAddressDst = ipv4Dst->GetAddress(1, 0);
  Ipv4Address ipv4AddressDst = ipv4InterfaceAddressDst.GetLocal();

  // Assign flowId
  int flowId = srcNode->GetNApplications();

  if (g_shortFlowSizeUniRandom)
    g_shortFlowSize = GetShortFlowSize (); // @ShortFlowConfig()
  if (g_shortFlowSizeDist)
    { // It throws an error if both --sfrand and --sfdist are active
      assert (g_shortFlowSizeUniRandom == 0);
      g_shortFlowSize = ExtrapolateFlowSize (drand48 ()) * 1400; // bytes
    }

  if (g_socketType == MMPTCP)
    {
      g_shortFlowSubflows = g_subflows;
    }

  // Source
  MpTcpBulkSendHelper source("ns3::TcpSocketFactory", InetSocketAddress(Ipv4Address(ipv4AddressDst), dstNode->GetId() + 1));
  source.SetAttribute("MaxBytes", UintegerValue(g_shortFlowSize));
  source.SetAttribute("FlowId", UintegerValue(flowId));
  source.SetAttribute("MaxSubflows", UintegerValue(g_shortFlowSubflows));
  source.SetAttribute("FlowType", StringValue("Short"));
  string tempString = SetupSimFileName("RESULT");
  source.SetAttribute("OutputFileName", StringValue(tempString));

  if (g_enableAutoDupAck && (g_socketType == MMPTCP || g_socketType == SCATTER))
    {
      g_dupAckThresh = GetReTxThresh(ipv4AddressSrc, ipv4AddressDst);
    }
  source.SetAttribute("DupAck", UintegerValue(g_dupAckThresh));
  cmAnalisys(SHORT, ipv4AddressSrc, ipv4AddressDst);
  ApplicationContainer tmp = source.Install(srcNode);

  // Start
  tmp.Get(0)->SetStartTime(Seconds(0));

  // STOP
  double diff = (double) g_simTime - Simulator::Now().GetSeconds();
  double nextStop = (diff > 0) ? diff : 0;
  tmp.Get(0)->SetStopTime(Seconds(nextStop));

  sourceShortFlowApps.push_back(tmp); //sourceShortFlowApps[src][dst].push_back(tmp);

  // Schedule next arrival
  if (Simulator::Now().GetSeconds() <  (static_cast<double>(g_simTime) - g_arrivalUpperBound))
    {
      double nextEvent = exponential(g_lamda);
      cout << "[" << GetKeyFromValueST(g_socketType) << "]{" << GetKeyFromValueTM(g_shortFlowTM) << "} StartNow: "
          << Simulator::Now().GetSeconds() << " Stop: " << nextStop << " (" << src << " -> " << dst << ") DupAck: "
          << g_dupAckThresh << " NextArrival: " << nextEvent << endl;
      Simulator::Schedule(Seconds(nextEvent), &ShortFlowConfig, CM, allHosts);
    }
}

Ptr<Queue>
FindQueue(Ptr<NetDevice> dev)
{
  PointerValue ptr;
  dev->GetAttribute("TxQueue", ptr);
  return ptr.Get<Queue>();
}

void
CreateHotSpotPerNode(const NodeContainer &nc, string layer)
{
  UintegerValue limit;
  vector<uint32_t> randC;
  uint32_t randomNode;
  uint32_t numHotSpots = (nc.GetN() * g_hotspotRatio) / 100;
  cout << "\nNo." << layer << "s :" << nc.GetN() << " HotspotRatio: " << g_hotspotRatio << "% => No."<< layer << "'s with hotspot: " << numHotSpots << endl;
  for (uint32_t loop = 0; loop < numHotSpots; loop++)
    {
      do
        {
          randomNode = rand() % (nc.GetN());
        }
      while (find(randC.begin(), randC.end(), randomNode) != randC.end());
      randC.push_back(randomNode);

      for (uint32_t j = 1; j < nc.Get(randomNode)->GetNDevices(); j++)
        {
              Ptr<Queue> txQueue = FindQueue(nc.Get(randomNode)->GetDevice(j));
              txQueue->SetAttribute("MaxPackets", UintegerValue(g_hotspotQueueSize));
              txQueue->GetAttribute("MaxPackets", limit);
              cout << layer << "-" << randomNode << " Link-" << j << " QueueSize: " << limit.Get() << endl;
        }
    }
}

// [Second][Node][Dev][Metrics]
void
SetupTraces(const Layers_t &layer)
{
  uint32_t T = (uint32_t) Simulator::Now().GetSeconds();
  if (layer == Core || layer >= Core_Aggr)
    {
      for (uint32_t i = 0; i < core_c.GetN(); i++)
        { // j should 1 as lookback interface should not be counted
          for (uint32_t j = 1; j < core_c.Get(i)->GetNDevices(); j++)
            {
              Ptr<Queue> txQueue = FindQueue(core_c.Get(i)->GetDevice(j));
              uint32_t totalDropBytes = txQueue->GetTotalDroppedBytes();
              uint32_t totalRxBytes = txQueue->GetTotalReceivedBytes();
              uint32_t totalBytes = totalRxBytes + totalDropBytes;
              core_data[T][0][i][j] = (((double) totalRxBytes * 8 * 100) / GetLinkRate(g_linkCapacity)); // Link Utilization
              core_data[T][1][i][j] = (((double) totalDropBytes / totalBytes) * 100); // LossRate
              // Make sure util is not nan
              if (isNaN(core_data[T][0][i][j]))
                core_data[T][0][i][j] = 0;
              // Make sure loss is not nan
              if (isNaN(core_data[T][1][i][j]))
                core_data[T][1][i][j] = 0;
              // Reset txQueue
              txQueue->ResetStatistics();
            }
        }
    }
  if (layer == Aggr || layer >= Core_Aggr)
    {
      for (uint32_t i = 0; i < Aggr_c.GetN(); i++)
        { // j = 1 as loop back interface should not be counted
          for (uint32_t j = 1; j < Aggr_c.Get(i)->GetNDevices(); j++)
            {
              Ptr<Queue> txQueue = FindQueue(Aggr_c.Get(i)->GetDevice(j));
              uint32_t totalDropBytes = txQueue->GetTotalDroppedBytes();
              uint32_t totalRxBytes = txQueue->GetTotalReceivedBytes();
              uint32_t totalBytes = totalRxBytes + totalDropBytes;
              aggr_data[T][0][i][j] = (((double) totalRxBytes * 8 * 100) / GetLinkRate(g_linkCapacity));  // Link Utilization
              aggr_data[T][1][i][j] = (((double) totalDropBytes / totalBytes) * 100);  // LossRate
              // Make sure aggregation util is not nan
              if (isNaN(aggr_data[T][0][i][j]))
                aggr_data[T][0][i][j] = 0;
              // Make sure aggregation loss is not nan
              if (isNaN(aggr_data[T][1][i][j]))
                aggr_data[T][1][i][j] = 0;
              txQueue->ResetStatistics();
            }
        }
    }
  if (layer == Tor || layer >= Core_Aggr_Tor)
    {
      for (uint32_t i = 0; i < Tor_c.GetN(); i++)
        { // dev = 1 as loop back interface should not be counted here.
          for (uint32_t j = 1; j < Tor_c.Get(i)->GetNDevices(); j++)
            {
              Ptr<Queue> txQueue = FindQueue(Tor_c.Get(i)->GetDevice(j));
              uint32_t totalDropBytes = txQueue->GetTotalDroppedBytes();
              uint32_t totalRxBytes = txQueue->GetTotalReceivedBytes();
              uint32_t totalBytes = totalRxBytes + totalDropBytes;
              tor_data[T][0][i][j] = (((double) totalRxBytes * 8 * 100) / GetLinkRate(g_linkCapacity));
              tor_data[T][1][i][j] = (((double) totalDropBytes / totalBytes) * 100);
              // Make sure ToR util is not nan
              if (isNaN(tor_data[T][0][i][j]))
                tor_data[T][0][i][j] = 0;
              // Make sure ToR loss is not nan
              if (isNaN(tor_data[T][1][i][j]))
                tor_data[T][1][i][j] = 0;
              txQueue->ResetStatistics();
            }
        }
    }
  if (layer == Host || layer >= Core_Aggr_Tor_Host)
    {
      for (uint32_t i = 0; i < Host_c.GetN(); i++)
        { // dev = 1 as loop back interface should not be counted here.
          for (uint32_t j = 1; j < Host_c.Get(i)->GetNDevices(); j++)
            {
              Ptr<Queue> txQueue = FindQueue(Host_c.Get(i)->GetDevice(j));
              uint32_t totalDropBytes = txQueue->GetTotalDroppedBytes();
              uint32_t totalRxBytes = txQueue->GetTotalReceivedBytes();
              uint32_t totalBytes = totalRxBytes + totalDropBytes;
              double util = (((double) totalRxBytes * 8 * 100) / GetLinkRate(g_linkCapacity));
              double loss = (((double) totalDropBytes / totalBytes) * 100);
              if (isNaN(util))
                util = 0;
              if (isNaN(loss))
                loss = 0;
              host_data[T][0][i][j] = util;
              host_data[T][1][i][j] = loss;

              txQueue->ResetStatistics();
            }
        }
    }
  if (T < g_simTime)
    Simulator::Schedule(Seconds(1), &SetupTraces, layer);
}

string
GetDateTimeNow()
{
  time_t T = time(0);
  struct tm* now = localtime(&T);
  string simStartDate = asctime(now);
  return simStartDate.substr(0, 24);
}

void
SetSimStartTime()
{
  g_simStartTime = GetDateTimeNow();
}

string
GetSimStartTime()
{
  return g_simStartTime;
}

void
SimHeaderWritter(Ptr<OutputStreamWrapper> stream)
{
  ostream *os = stream->GetStream ();
  *os << "SimStart [" << g_simStartTime << "] simName[" << g_simName
      << "] Topology[" << g_topology << "] TotalHost[" << g_totalHost
      << "] SocketType[" << GetKeyFromValueST (g_socketType) << "] bwRatio[1:"
      << g_ratio << "] TrafficMatrix[" << GetKeyFromValueTM (g_trafficMatrix)
      << "] FlowSizeThresh[" << g_FlowSizeThresh << "] ShortFlowTM["
      << GetKeyFromValueTM (g_shortFlowTM) << "] BackgroundFlowTM["
      << GetKeyFromValueTM (g_backgroundFlowTM) << "] BackgroundFlowLimit["
      << g_connxLimit << "] Seed[" << g_seed << "] simInstance["
      << g_simInstance << "] Lamda[" << g_lamda << "] SimPeriod[" << g_simTime
      << "s] HostPerToR[" << g_numHost << "] CC[" << g_cc << "] FlowSize["
      << g_flowSize << "] ShortFlowSize[" << g_shortFlowSize << "] LinkRate["
      << g_linkCapacity << "] LinkDelay[" << g_linkDelay << "] AutoDupAck["
      << isActive (g_enableAutoDupAck) << "] DupAckThresh["
      << g_dupAckThresh + 3 << "] isLimitedTx[" << isActive (g_enableLimitedTx)
      << "] isRTOm[" << isActive (g_enableMinRTO) << "] RTOmin[ " << g_minRTO
      << "] isHotspot[" << isActive (g_enableHotspots) << "]" << endl;
}

void
SimFooterWritter(Ptr<OutputStreamWrapper> stream)
{
  ostream *os = stream->GetStream();
  *os << "SimEnd [" << GetDateTimeNow() << "] AllFlows[" << sourceLargeFlowApps.size() + sourceShortFlowApps.size()
      << "] LargeFlow[" << sourceLargeFlowApps.size() << "] ShortFlows[" << sourceShortFlowApps.size() << "]  CoreUtil["
      << meanCoreUtil << "] CoreLoss[" << meanCoreLoss << "] AggrUtil[" << meanAggrUtil << "] AggrLoss[" << meanAggrLoss << "]"
      << endl;
}

void
SimOverallResultWritter()
{
  Ptr<OutputStreamWrapper> stream = Create<OutputStreamWrapper>(SetupSimFileName("OVERALL"), std::ios::out | std::ios::app);
  SimHeaderWritter(stream);
  ostream *os = stream->GetStream();
  meanCoreUtil = totalCoreUtil / (g_simTime * g_totalCore * g_K);
  meanCoreLoss = totalCoreLoss / (g_simTime * g_totalCore * g_K);
  if (isNaN(meanCoreUtil))
    meanCoreUtil = 0;
  if (isNaN(meanCoreLoss))
    meanCoreLoss = 0;
  meanAggrUtil = totalAggrUtil / (g_simTime * g_totalAggr * g_K);
  meanAggrLoss = totalAggrLoss / (g_simTime * g_totalAggr * g_K);
  if (isNaN(meanAggrUtil))
    meanAggrUtil = 0;
  if (isNaN(meanAggrLoss))
    meanAggrLoss = 0;
  meanTorUtil = totalTorUtil / (g_simTime * g_totalToR * (g_numAggr + g_numHost));
  meanTorLoss = totalTorLoss / (g_simTime * g_totalToR * (g_numAggr + g_numHost));
  if (isNaN(meanTorUtil))
    meanTorUtil = 0;
  if (isNaN(meanTorLoss))
    meanTorLoss = 0;
  meanHostUtil = totalHostUtil / (g_simTime * g_totalHost);
  meanHostLoss = totalHostLoss / (g_simTime * g_totalHost);
  if (isNaN(meanHostUtil))
    meanHostUtil = 0;
  if (isNaN(meanHostLoss))
    meanHostLoss = 0;
  *os << "CoreUtil [!" << meanCoreUtil << "!]\nCoreLoss [@" << meanCoreLoss << "@]\nAggrUtil [#" << meanAggrUtil
      << "#]\nAggrLoss [$" << meanAggrLoss << "$]\nTorUtil [%" << meanTorUtil << "%]\nTorLoss [^" << meanTorLoss
      << "^]\nHostUtil [&" << meanHostUtil << "&]\nHostLoss [*" << meanHostLoss << "*]" << endl;
  SimFooterWritter(stream);
}

void
SimResultHeaderGenerator()
{
  Ptr<OutputStreamWrapper> streamSimParam = Create<OutputStreamWrapper>(SetupSimFileName("RESULT"),
      std::ios::out | std::ios::app);
  SimHeaderWritter(streamSimParam);
}

void
SimResultFooterGenerator()
{
  Ptr<OutputStreamWrapper> streamSimParam = Create<OutputStreamWrapper>(SetupSimFileName("RESULT"),
      std::ios::out | std::ios::app);
  SimFooterWritter(streamSimParam);
}

// Setup cmd callbacks
bool
SetNumPort(std::string input)
{
  cout << "SwitchPort: " << g_K << " -> " << input << endl;
  g_K = atoi(input.c_str());
  // Setup topology parameters
  g_numPod = g_K;                    // Pods:                        8
  g_numHost = (g_K / 2) * g_ratio;   // Host per ToR switch:         4
  g_numToR = g_K / 2;                // ToR swtiches per pod:        4
  g_numAggr = g_K / 2;               // Aggr switches per pod:       4
  g_numCore = g_K / 2;               // Core switches in group:      2
  g_numGroup = g_K / 2;              // Core switches in group:      2
  g_totalHost = ((g_K * g_K * g_K) / 4) * g_ratio;
  g_totalToR = g_numToR * g_numPod;
  g_totalAggr = g_numAggr * g_numPod;
  g_totalCore = g_numCore * g_numGroup;
  return true;
}

void
HotSpotGenerator()
{
  if (g_enableHotspots)
    {
      switch (g_hotspotLayer)
        {
      case Core:
        CreateHotSpotPerNode(core_c, "Core");
        break;
      case Aggr:
        CreateHotSpotPerNode(Aggr_c, "Aggr");
        break;
      case Tor:
        CreateHotSpotPerNode(Tor_c, "Tor");
        break;
      case Host:
        CreateHotSpotPerNode(Host_c, "Host");
        break;
      default:
        CreateHotSpotPerNode(core_c, "Core");
        break;
        }
    }
}

bool
SetNumSubflow(std::string input)
{
  cout << "Subflows         : " << g_subflows << " -> " << input << endl;
  g_subflows = atoi(input.c_str());
  return true;
}

bool
SetCongestionControl(std::string input)
{
  cout << "CongestionControl: " << g_cc << " -> " << input << endl;
  g_cc = input;
  return true;
}

bool
SetFlowSize(std::string input)
{
  cout << "FlowSize         : " << g_flowSize << " -> " << input << endl;
  g_flowSize = atoi(input.c_str());
  return true;
}

bool
SetShortFlowSize(std::string input)
{
  g_shortFlowSize = atoi(input.c_str()) * 1000;
  cout << "ShortFlowSize    : " << g_shortFlowSize << endl;
  return true;
}

bool
SetSimTime(std::string input)
{
  cout << "SimDuration      : " << g_simTime << " -> " << input << endl;
  g_simTime = atoi(input.c_str());
  return true;
}

bool
SetLamda(std::string input)
{
  cout << "Lamda            : " << g_lamda << " -> " << input << endl;
  g_lamda = atoi(input.c_str());
  return true;
}

bool
SetTrafficMatrix(std::string input)
{
  cout << "TrafficMatrix    : " << GetKeyFromValueTM(g_trafficMatrix) << " -> " << input << endl;
  if (stringToTrafficMatrix.count(input) != 0)
    {
      g_trafficMatrix = stringToTrafficMatrix[input];
    }
  else
    NS_FATAL_ERROR("Input for setting up traffic matrix has spelling issue - try again!");
  return true;
}

bool
SetSFTM(std::string input)
{
  cout << "ShortFlowTM      : " << GetKeyFromValueTM(g_shortFlowTM) << " -> " << input << endl;
  if (stringToTrafficMatrix.count(input) != 0)
    {
      g_shortFlowTM = stringToTrafficMatrix[input];
    }
  else
    {
      cerr << "Input for setting up short flow raffic matrix has spelling issue - try again!" << endl;
    }
  return true;
}

bool
SetSocketType(std::string input)
{
  cout << "SocketType       : " << GetKeyFromValueST(g_socketType) << " -> " << input << endl;
  if (stringToSocketType.count(input) != 0)
    {
      g_socketType = stringToSocketType[input];
    }
  else
    NS_FATAL_ERROR("Input for setting up socket type has spelling issue - try again!");
  return true;
}

bool
SetSimInstance(std::string input)
{
  cout << "SimInstance      : " << g_simInstance << " -> " << input << endl;
  g_simInstance = input;
  return true;
}

bool
SetDupAck(std::string input)
{
  cout << "SetDupAck        : " << g_dupAckThresh << " -> " << input << endl;
  g_dupAckThresh = atoi(input.c_str());
  return true;
}

bool
SetAutoDupAck(std::string input)
{
  cout << "AutoDupAck       : " << g_enableAutoDupAck << " -> " << input << endl;
  g_enableAutoDupAck = atoi(input.c_str());
  return true;
}

bool
SetMinRto(std::string input)
{
  cout << "RTOmin           : " << g_enableMinRTO << " -> " << input << endl;
  g_enableMinRTO = atoi(input.c_str());
  return true;
}

bool
SetLimitedTx(std::string input)
{
  cout << "LimitedTx        : " << g_enableLimitedTx << " -> " << input << endl;
  g_enableLimitedTx = atoi(input.c_str());
  return true;
}

bool
SetRTO(std::string input)
{
  cout << "RTO              : " << g_minRTO << " -> " << input << endl;
  g_minRTO = atoi(input.c_str());
  return true;
}

bool
SetConnxLimit(std::string input)
{
  cout << "ConnectionLimit  : " << g_connxLimit << " -> " << input << endl;
  g_connxLimit = atoi(input.c_str());
  return true;
}

bool
SetRatio(std::string input)
{
  cout << "Bandwidth ratio  : " << g_ratio << " -> " << input << endl;
  g_ratio = atoi(input.c_str());
  g_numHost = (g_K / 2) * g_ratio;
  g_totalHost = ((g_K * g_K * g_K) / 4) * g_ratio;
  return true;
}

bool
SetLinkRate(std::string input)
{
  cout << "Link rate        : " << g_linkCapacity << " -> " << input+"Mbps" << endl;
  g_linkCapacity = input+"Mbps";
  return true;
}

bool
SetSimName(std::string input)
{
  cout << "SimName          : " << g_simName << " -> " << input << endl;
  g_simName = input;
  return true;
}

bool
SetSFSF(std::string input)
{
  cout << "shortFlowSubflows: " << g_shortFlowSubflows << " -> " << input << endl;
  g_shortFlowSubflows = atoi(input.c_str());
  return true;
}

bool
SetIW(std::string input)
{
  cout << "InitalCWND       : " << g_initialCWND << " -> " << input << endl;
  g_initialCWND = atoi(input.c_str());
  return true;
}

bool
SetLfPlot(std::string input)
{
  cout << "LargeFlowPlotting: " << g_enableLfPlotting << " -> " << input << endl;
  g_enableLfPlotting = atoi(input.c_str());
  return true;
}

bool
SetSfPlot(std::string input)
{
  cout << "ShortFlowPlotting: " << g_enableSfPlotting << " -> " << input << endl;
  g_enableSfPlotting = atoi(input.c_str());
  return true;
}

bool
SetAlphaPerAck(std::string input)
{
  cout << "SetAlphaPerAck   : " << g_alphaPerAck << " -> " << input << endl;
  g_alphaPerAck = atoi(input.c_str());
  return true;
}

bool
SetRandomGap(std::string input)
{
  cout << "SetRandomGap     : " << g_rGap << " -> " << input << endl;
  g_rGap = atoi(input.c_str());
  return true;
}

bool
SetArrivalUpperBound(std::string input)
{
  cout << "SetUpperBound    : " << g_arrivalUpperBound << " -> " << input << endl;
  g_arrivalUpperBound = atof(input.c_str());
  return true;
}

bool
SetShortFlowTCP(std::string input)
{
  cout << "ShortFlowTCP     : " << g_shortFlowTCP << " -> " << input << endl;
  g_shortFlowTCP = atoi(input.c_str());
  return true;
}

bool
SetHotSpot(std::string input)
{
  cout << "HotSpot          : " << g_enableHotspots << " -> " << input << endl;
  g_enableHotspots = atoi(input.c_str());
  return true;
}

bool
SetHotSpotDTQ(std::string input)
{
  cout << "HotSpotDTQ       : " << g_hotspotQueueSize << " -> " << input << endl;
  g_hotspotQueueSize = atoi(input.c_str());
  return true;
}

bool
SetHotSpotRatio(std::string input)
{
  cout << "HotSpotRatio     : " << g_hotspotRatio << " -> " << input << endl;
  g_hotspotRatio = atoi(input.c_str());
  return true;
}

bool
SetHotSpotLayer(std::string input)
{
  cout << "HotSpotLayer     : " << GetKeyFromValueHSL(g_hotspotLayer) << " -> " << input << endl;
  if (stringToHotSpotLayers.count(input) != 0)
    {
      g_hotspotLayer = stringToHotSpotLayers[input];
    }
  else
    exit(1);
  return true;
}

bool
SetSFST(std::string input)
{
  cout << "SetSFST          : " << g_shortFlowStartTime << " -> " << input << endl;
  g_shortFlowStartTime = atof(input.c_str());
  return true;
}

bool
SetFlowSizeThresh(std::string input)
{
  g_FlowSizeThresh = atoi(input.c_str());
  g_FlowSizeThresh = g_FlowSizeThresh * 1000;
  cout << "SetFlowSizeThresh: " << g_FlowSizeThresh << endl;
  return true;
}

bool
SetSC(std::string input)
{
  cout << "SetSwitchCond    : " << g_switchCondition << " -> " << input << endl;
  g_switchCondition = atoi(input.c_str());
  return true;
}

bool
SetSwitchingMode(std::string input)
{
  cout << "SetSwitchMode    : " << g_switchMode << " -> " << input << endl;
  g_switchMode = input;
  return true;
}

bool
SetIncastNumber(std::string input){
  cout << "Incast Number    : " << g_incastNumber << " -> " << input << endl;
  g_incastNumber = atoi(input.c_str());
  return true;
}

bool
SetIncastHeartbeat(std::string input)
{
  cout << "Incast Heartbeat : " << g_incastHeartbeat << " -> " << input << endl;
  g_incastHeartbeat = atof(input.c_str());
  return true;
}

bool
SetShortFlowSizeUniRandom (std::string input)
{
  cout << "ShortFlowSizeRand: " << g_shortFlowSizeUniRandom << " -> " << input << endl;
  g_shortFlowSizeUniRandom = atoi (input.c_str ());
  return true;
}

bool
SetShortFlowSizeDist (std::string input)
{
  cout << "ShortFlowSizeDist: " << g_shortFlowSizeDist << " -> " << input << endl;
  g_shortFlowSizeDist = atoi (input.c_str ());
  return true;
}

// Main
int
main(int argc, char *argv[])
{
  SetupStringToTM(); // Should be done before cmd parsing
  SetupStringToST();
  SetupStringToHSL();
  SetSimStartTime();

  // Enable log components
  LogComponentEnable("FatTree", LOG_ALL);

  // Set up command line parameters
  CommandLine cmd;
  cmd.AddValue("sp", "Number of Switch Port", MakeCallback(SetNumPort));
  cmd.AddValue("sf", "Number of MPTCP SubFlows", MakeCallback(SetNumSubflow));
  cmd.AddValue("cc", "MPTCP Congestion Control algorithm", MakeCallback(SetCongestionControl));
  cmd.AddValue("fs", "Flow Size", MakeCallback(SetFlowSize));
  cmd.AddValue("sfs", "Short Flow Size", MakeCallback(SetShortFlowSize));
  cmd.AddValue("st", "Simulation Time", MakeCallback(SetSimTime));
  cmd.AddValue("lamda", "Set lamda param for poisson process", MakeCallback(SetLamda));
  cmd.AddValue("cm", "Set traffic matrix", MakeCallback(SetTrafficMatrix));
  cmd.AddValue("sfcm", "Set short flow traffic matrix", MakeCallback(SetSFTM));
  cmd.AddValue("socket", "Set socket type ", MakeCallback(SetSocketType));
  cmd.AddValue("i", "Set simulation instance number as a string", MakeCallback(SetSimInstance));
  cmd.AddValue("dupack", "Duplicate acknowledgement threshold", MakeCallback(SetDupAck));
  cmd.AddValue("autodupack", "Set DupAckThresh automatically, by default is true (1)", MakeCallback(SetAutoDupAck));
  cmd.AddValue("mrto", "Set minimum RTO", MakeCallback(SetMinRto));
  cmd.AddValue("ltx", "Set Limited transmit", MakeCallback(SetLimitedTx));
  cmd.AddValue("rto", "Set minRTO", MakeCallback(SetRTO));
  cmd.AddValue("cl", "Set connection limit for large flows", MakeCallback(SetConnxLimit));
  cmd.AddValue("ratio", "Set over subscription ratio", MakeCallback(SetRatio));
  cmd.AddValue("lr", "Set p2p link rate", MakeCallback(SetLinkRate));
  cmd.AddValue("sim", "Set sim name", MakeCallback(SetSimName));
  cmd.AddValue("sfsf", "Set num of subflows for shortflow", MakeCallback(SetSFSF));
  cmd.AddValue("iw", "Set initial cwnd of initial subflow of MMPTCP", MakeCallback(SetIW));
  cmd.AddValue("lfplot", "Activate plotting at MpTcpSocketBase", MakeCallback(SetLfPlot));
  cmd.AddValue("sfplot", "Activate short flow plotting at MpTcpSocketBase", MakeCallback(SetSfPlot));
  cmd.AddValue("apa", " update alpha per ack", MakeCallback(SetAlphaPerAck));
  cmd.AddValue("rgap", "Set rando gap between subflows setup", MakeCallback(SetRandomGap));
  cmd.AddValue("aub", "Set arrival upper bound", MakeCallback(SetArrivalUpperBound));
  cmd.AddValue("sftcp", "Enable short flow to use TCP", MakeCallback(SetShortFlowTCP));
  cmd.AddValue("hs", "Enable hotspots", MakeCallback(SetHotSpot));
  cmd.AddValue("hsdtq", "hotspots droptail queue size", MakeCallback(SetHotSpotDTQ));
  cmd.AddValue("hsratio", "hotspots ratio", MakeCallback(SetHotSpotRatio));
  cmd.AddValue("hsl", "hotspots layer", MakeCallback(SetHotSpotLayer));
  cmd.AddValue("sfst", "Short flow start time", MakeCallback(SetSFST));
  cmd.AddValue("sfth", "Switching point", MakeCallback(SetFlowSizeThresh));
  cmd.AddValue("sc", "Switching Condition", MakeCallback(SetSC));
  cmd.AddValue("sm", "Switching Mode", MakeCallback(SetSwitchingMode));
  cmd.AddValue("in", "Number of workers in Incast scenrio", MakeCallback(SetIncastNumber));
  cmd.AddValue("inhb", "Incast arrival for short flows", MakeCallback(SetIncastHeartbeat));
  cmd.AddValue("sfrand","Short Flow Size Uniform Random", MakeCallback(SetShortFlowSizeUniRandom));
  cmd.AddValue("sfdist","Short Flow Size Distribution", MakeCallback(SetShortFlowSizeDist));

  cmd.Parse(argc, argv);

  // Set up default simulation parameters
  Config::SetDefault("ns3::Ipv4GlobalRouting::FlowEcmpRouting", BooleanValue(true));
  Config::SetDefault("ns3::TcpSocket::SegmentSize", UintegerValue(1400));
  Config::SetDefault("ns3::TcpSocket::DelAckCount", UintegerValue(0));
  Config::SetDefault("ns3::DropTailQueue::Mode", StringValue("QUEUE_MODE_PACKETS"));
  Config::SetDefault("ns3::DropTailQueue::MaxPackets", UintegerValue(100));
//Config::SetDefault("ns3::MpTcpBulkSendApplication::MaxBytes", UintegerValue(g_flowSize));
  Config::SetDefault("ns3::MpTcpSocketBase::LargePlotting", BooleanValue(g_enableLfPlotting));
  Config::SetDefault("ns3::MpTcpSocketBase::ShortPlotting", BooleanValue(g_enableSfPlotting));
  Config::SetDefault("ns3::MpTcpSocketBase::AlphaPerAck", BooleanValue(g_alphaPerAck));
  Config::SetDefault("ns3::MpTcpSocketBase::RandomGap", UintegerValue(g_rGap));
  Config::SetDefault("ns3::MpTcpSocketBase::ShortFlowTCP", BooleanValue(g_shortFlowTCP));


  switch (g_socketType)
    {
  case MPTCP:
    Config::SetDefault("ns3::TcpL4Protocol::SocketType", TypeIdValue(MpTcpSocketBase::GetTypeId()));
    Config::SetDefault("ns3::MpTcpSocketBase::CongestionControl", StringValue(g_cc));
    Config::SetDefault("ns3::MpTcpSocketBase::MaxSubflows", UintegerValue(g_subflows)); // Sink
    Config::SetDefault("ns3::MpTcpBulkSendApplication::MaxSubflows", UintegerValue((uint8_t)g_subflows));//Source
    Config::SetDefault("ns3::MpTcpSocketBase::PathManagement", StringValue("NdiffPorts"));
    break;
  case SCATTER:
    Config::SetDefault("ns3::TcpL4Protocol::SocketType", TypeIdValue(PacketScatterSocketBase::GetTypeId()));
    Config::SetDefault("ns3::MpTcpSocketBase::PathManagement", StringValue("Default"));
    break;
  case MMPTCP:
    Config::SetDefault("ns3::TcpSocket::InitialCwnd", UintegerValue(g_initialCWND));
    if (g_enableMinRTO)
      {
        Config::SetDefault("ns3::MMpTcpSocketBase::IsRTOmin", BooleanValue(g_enableMinRTO));
        Config::SetDefault("ns3::MMpTcpSocketBase::RTOmin", UintegerValue(g_minRTO));
      }
    Config::SetDefault("ns3::MMpTcpSocketBase::IsLimitedTx", BooleanValue(g_enableLimitedTx));
    Config::SetDefault("ns3::TcpL4Protocol::SocketType", TypeIdValue(MMpTcpSocketBase::GetTypeId()));
  //Config::SetDefault("ns3::MMpTcpSocketBase::DupAckThresh", UintegerValue(g_dupAckThresh));
    Config::SetDefault("ns3::MpTcpSocketBase::CongestionControl", StringValue(g_cc));
    Config::SetDefault("ns3::MpTcpSocketBase::MaxSubflows", UintegerValue(g_subflows)); // Sink
    Config::SetDefault("ns3::MpTcpBulkSendApplication::MaxSubflows", UintegerValue((uint8_t)g_subflows)); //Source
    Config::SetDefault("ns3::MpTcpSocketBase::PathManagement", StringValue("Default"));
    Config::SetDefault("ns3::MMpTcpSocketBase::FlowSizeThresh", UintegerValue(g_FlowSizeThresh));
    Config::SetDefault("ns3::MMpTcpSocketBase::SwitchingCondition", BooleanValue(g_switchCondition));
    Config::SetDefault("ns3::MMpTcpSocketBase::SwitchingMode", StringValue(g_switchMode));
    break;
  case TCP:
    Config::SetDefault("ns3::TcpL4Protocol::SocketType", TypeIdValue(MpTcpSocketBase::GetTypeId()));
    Config::SetDefault("ns3::MpTcpSocketBase::MaxSubflows", UintegerValue(1)); // For the sink
    Config::SetDefault("ns3::MpTcpBulkSendApplication::MaxSubflows", UintegerValue(1)); // TCP need one subflow only
    Config::SetDefault("ns3::MpTcpSocketBase::CongestionControl", StringValue("Uncoupled_TCPs"));
    Config::SetDefault("ns3::MpTcpSocketBase::PathManagement", StringValue("Default"));
    break;
  default:
    break;
    }

  g_seed = static_cast<uint32_t>(atoi(g_simInstance.c_str()));
  cout << "Seed             : " << g_seed << endl;
  srand(g_seed);

  SimResultHeaderGenerator();

  cout << endl;
  cout << "switchPorts      :  " << g_K << endl;
  cout << "totalCores       :  " << g_totalCore << endl;
  cout << "totalAggrs       :  " << g_totalAggr << endl;
  cout << "totalToRs        :  " << g_totalToR << endl;
  cout << "totalHosts       :  " << g_totalHost << endl;

  PrintSimParams();

  InternetStackHelper internet;

// ------------------ Topology Construction ------------------
  NodeContainer allHosts;

// Host Layer Nodes
  NodeContainer host[g_numPod][g_numToR]; // NodeContainer for hosts
  for (uint32_t i = 0; i < g_numPod; i++)
    {
      for (uint32_t j = 0; j < g_numToR; j++)
        { // host[g_numPod][g_numToR]
          host[i][j].Create(g_numHost); // 20 hosts per each ToR switch
          internet.Install(host[i][j]);
          allHosts.Add(host[i][j]);     // Add all server to GlobalHostContainer
          Host_c.Add(host[i][j]);       // Add all server to Host_c for link utilization
        }
    }
// Access layer Nodes
  NodeContainer tor[g_numPod];          // NodeContainer for ToR switches
  for (uint32_t i = 0; i < g_numPod; i++)
    {
      tor[i].Create(g_numToR);
      internet.Install(tor[i]);
      Tor_c.Add(tor[i]);
    }
// Aggregation layer Nodes
  NodeContainer aggr[g_numPod];         // NodeContainer for aggregation switches
  for (uint32_t i = 0; i < g_numPod; i++)
    {
      aggr[i].Create(g_numAggr);
      internet.Install(aggr[i]);
      Aggr_c.Add(aggr[i]);
    }
// Core Layer Nodes
  NodeContainer core[g_numGroup];       // NodeContainer for core switches
  for (uint32_t i = 0; i < g_numGroup; i++)
    {
      core[i].Create(g_numCore);
      internet.Install(core[i]);
      core_c.Add(core[i]);
    }
// -----------------------------------------------------------
  PointToPointHelper p2p;
  p2p.SetDeviceAttribute ("DataRate", StringValue (g_linkCapacity));
  p2p.SetChannelAttribute ("Delay", StringValue (g_linkDelay));

  Ipv4AddressHelper ipv4Address;

//=========== Connect hosts to ToRs ===========//
  NetDeviceContainer hostToTorNetDevice[g_numPod][g_numToR][g_numHost];
  stringstream ss;
  for (uint32_t p = 0; p < g_numPod; p++)
    {
      for (uint32_t t = 0; t < g_numToR; t++)
        {
          ss.str("");
          ss << "10." << p << "." << t << "." << "0";
          string tmp = ss.str();
          const char* address = tmp.c_str();
          ipv4Address.SetBase(address, "255.255.255.252");
          for (uint32_t h = 0; h < g_numHost; h++)
            {
              hostToTorNetDevice[p][t][h] = p2p.Install(NodeContainer(host[p][t].Get(h), tor[p].Get(t)));
              ipv4Address.Assign(hostToTorNetDevice[p][t][h]);
              Ipv4Address newNet = ipv4Address.NewNetwork();
              NS_LOG_UNCOND(newNet);
            }
        }
    }NS_LOG_INFO("Finished connecting tor switches and hosts");
//=========== Connect aggregate switches to edge switches ===========//
  NetDeviceContainer ae[g_numPod][g_numAggr][g_numToR];
  ipv4Address.SetBase("20.20.0.0", "255.255.255.0");
  for (uint32_t p = 0; p < g_numPod; p++)
    {
      for (uint32_t a = 0; a < g_numAggr; a++) // number of aggr switch per pod
        {
          for (uint32_t t = 0; t < g_numToR; t++)
            {
              ae[p][a][t] = p2p.Install(aggr[p].Get(a), tor[p].Get(t));

              ipv4Address.Assign(ae[p][a][t]);
              ipv4Address.NewNetwork();
            }
        }
    }NS_LOG_INFO("Finished connecting tor switches and aggregation");

//=========== Connect core switches to aggregate switches ===========//
  NetDeviceContainer ca[g_numGroup][g_numCore][g_numPod];
  ipv4Address.SetBase("30.30.0.0", "255.255.255.0");
  for (uint32_t g = 0; g < g_numGroup; g++)
    {
      for (uint32_t c = 0; c < g_numCore; c++)
        {
          for (uint32_t p = 0; p < g_numPod; p++)
            {
              ca[g][c][p] = p2p.Install(core[g].Get(c), aggr[p].Get(g));
              ipv4Address.Assign(ca[g][c][p]);
              ipv4Address.NewNetwork();
            }
        }
    }NS_LOG_INFO("Finished connecting core and aggregation");

// Populate Global Routing
  Ipv4GlobalRoutingHelper::PopulateRoutingTables();

  SetupTraces(Core_Aggr_Tor_Host);

//=========== Initialize settings for On/Off Application ===========//
// SINK application - It would be closed doubled the time of source closure!
  cout << "\nSink App Install on following nodes: " << endl;
  for (uint32_t i = 0; i < allHosts.GetN(); i++)
    {
      MpTcpPacketSinkHelper sink("ns3::TcpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), (allHosts.Get(i)->GetId() + 1)));
      ApplicationContainer tmp = sink.Install(allHosts.Get(i));
      tmp.Start(Seconds(0.0));
      tmp.Stop(Seconds(g_simTime + 20));
      sinkApps.push_back(tmp);
      cout << allHosts.Get(i)->GetId() << " ";
    }
  cout << "\n" <<endl;

  ConnectionMatrix* conns = new ConnectionMatrix((int) g_totalHost);

  switch (g_trafficMatrix)
    {
  case PERMUTATION:
    conns->setPermutation();
    break;
  case STRIDE:
    conns->setStride(g_numToR * g_numHost);
    break;
  case RANDOM:
    conns->setRandomConnection(g_totalHost);
    break;
  case INCAST:
    conns->setIncast(g_incastNumber);
    break;
  case SHORT_FLOW:
    if (g_shortFlowTM == PERMUTATION)
      conns->setPermutation();
    else if (g_shortFlowTM == STRIDE)
      conns->setStride(g_numToR * g_numHost);
    else if (g_shortFlowTM == RANDOM)
      conns->setRandomConnection(g_totalHost);
    else if (g_shortFlowTM == INCAST)
      conns->setIncast(g_incastNumber);
    break;
  default:
    break;
    }

  // Large flows SOURCE applications - They would be closed by g_simTime
  vector<connection*>* CM = conns->getAllConnections();
  uint32_t totalLargeConnx;
  vector<int> tmpVec;
  int pos;
  //
  if (g_trafficMatrix == SHORT_FLOW)
    {
      totalLargeConnx = static_cast<uint32_t>((CM->size() * g_connxLimit) / 100);

      if (g_shortFlowTM == INCAST)
       totalLargeConnx = 0;

      cout << "\nCMSize: " << (int) CM->size() << " LargeFlowCM: " << totalLargeConnx << " => " << g_connxLimit
          << "% of total flows" << endl;
    }
  else // Other TM (PERMUTATION, STRIDE, RANDOM); all for large flows
    {
      totalLargeConnx = CM->size();
      cout << "CMSize: " << (int) CM->size() << " LargeFlowCM: " << totalLargeConnx << " 100% of total flows" << endl;
    }
  //
  for (uint32_t i = 0; i < totalLargeConnx; i++)
    {
      do
        {
          pos = rand() % CM->size();
        }
      while (find(tmpVec.begin(), tmpVec.end(), pos) != tmpVec.end());
      tmpVec.push_back(pos);
      connection* connection = (*CM).at(pos);
      int src = connection->src;
      int dst = connection->dst;
      connection->large = true;
      //
      Ptr<Node> srcNode = allHosts.Get(src);
      Ptr<Ipv4> ipv4Client = srcNode->GetObject<Ipv4>();
      Ipv4InterfaceAddress iaddrClient = ipv4Client->GetAddress(1, 0);
      Ipv4Address ipv4AddrClient = iaddrClient.GetLocal();
      //
      Ptr<Node> randomServerNode = allHosts.Get(dst);
      uint32_t flowId = allHosts.Get(src)->GetNApplications();
      Ptr<Ipv4> ipv4Server = randomServerNode->GetObject<Ipv4>();
      Ipv4InterfaceAddress iaddrServer = ipv4Server->GetAddress(1, 0);
      Ipv4Address ipv4AddrServer = iaddrServer.GetLocal();

      cout << "LargeFlowCM(" << i + 1 << ") " << allHosts.Get(src)->GetId() << " -> " << randomServerNode->GetId()
          << " DupAck: " << g_dupAckThresh << endl;
      MpTcpBulkSendHelper source("ns3::TcpSocketFactory",
          InetSocketAddress(Ipv4Address(ipv4AddrServer), randomServerNode->GetId() + 1));
      source.SetAttribute("MaxBytes", UintegerValue(g_flowSize));
      source.SetAttribute("FlowId", UintegerValue(flowId));
      //source.SetAttribute("MaxSubflows", UintegerValue(8)); // TCP would not work if it is uncommented
      source.SetAttribute("FlowType", StringValue("Large"));
      source.SetAttribute("OutputFileName", StringValue(SetupSimFileName("RESULT")));

      if (g_enableAutoDupAck && (g_socketType == MMPTCP || g_socketType == SCATTER))
        {
          g_dupAckThresh = GetReTxThresh(ipv4AddrClient, ipv4AddrServer);
        }
      source.SetAttribute("DupAck", UintegerValue(g_dupAckThresh));
      cmAnalisys(LARGE, ipv4AddrClient, ipv4AddrServer);
      sourceLargeFlowApps.push_back(source.Install(allHosts.Get(src)));
    }
  cout << "\nLarge Flow Source App Installs on following nodes: " << endl;
  for (uint32_t i = 0; i < sourceLargeFlowApps.size(); i++)
    {
      Ptr<MpTcpBulkSendApplication> mptcpBulk = DynamicCast<MpTcpBulkSendApplication>(sourceLargeFlowApps[i].Get(0));
      cout << mptcpBulk->GetNode()->GetId() << " ";
      sourceLargeFlowApps[i].Start(Seconds(0.0));
      sourceLargeFlowApps[i].Stop(Seconds(g_simTime));
    }
  cout << "\n" << endl;

  vector<connection*>* shortCM = GetShortCM(CM);
  if (g_trafficMatrix == SHORT_FLOW && g_shortFlowTM != INCAST)
    {
      Simulator::Schedule(Seconds(g_shortFlowStartTime), &ShortFlowConfig, shortCM, allHosts);
    }
  else if (g_trafficMatrix == SHORT_FLOW && g_shortFlowTM == INCAST)
    {
      Simulator::Schedule(Seconds(g_shortFlowStartTime), &ShortFlowConfigForIncast, shortCM, allHosts);
    }

  SimTimeMonitor();

  Simulator::Schedule(Seconds(g_shortFlowStartTime), &HotSpotGenerator);

  PrintCMStat();
  NS_LOG_INFO ("Run Simulation.");
  Simulator::Stop(Seconds(g_simTime + 40));
  Simulator::Run();

  cout << Simulator::Now().GetSeconds() << " -> Generate Out puts"<< endl;

  OutPutCore();
  OutPutAggr();
  OutPutTor();
  OutPutHost();

  SimOverallResultWritter();
  SimResultFooterGenerator(); // OveralResultWritter should be called first!
  //-----------------------------------------------------------------------//
  Simulator::Destroy();
  NS_LOG_INFO ("Done.");
  cout << Simulator::Now().GetSeconds() << " END "<< endl;
  return 0;
}
