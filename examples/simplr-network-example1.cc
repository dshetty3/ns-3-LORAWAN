#include "ns3/command-line.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/end-device-lora-phy.h"
#include "ns3/end-device-lorawan-mac.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/gateway-lorawan-mac.h"
#include "ns3/log.h"
#include "ns3/lora-helper.h"
#include "ns3/lorawan-mac-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/node-container.h"
#include "ns3/one-shot-sender-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/simulator.h"
#include "ns3/callback.h"
#include "ns3/string.h"
#include "ns3/rectangle.h"
#include <algorithm>
#include <ctime>
#include <cstdlib>
#include <fstream>
#include <numeric> 

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE("ParallelReceptionExample");

uint32_t totalPackets = 0;
uint32_t receivedPackets = 0;

void PacketReceivedCallback(Ptr<Packet const> packet, unsigned int frequencyIndex) {
    if (rand() % 100 < 90) {
        NS_LOG_INFO("Packet dropped due to simulated packet loss");
        return;
    }

    receivedPackets++;
    NS_LOG_INFO("Packet received successfully");
}

int main(int argc, char* argv[]) {
    const int numRuns = 10; 
    std::vector<double> prrResults;

    for (int j = 0; j < numRuns; j++) {
        srand(static_cast<unsigned int>(time(0)) + j);

        totalPackets = 0;
        receivedPackets = 0;

        LogComponentEnable("ParallelReceptionExample", LOG_LEVEL_ALL);
        LogComponentEnable("GatewayLoraPhy", LOG_LEVEL_ALL);
        LogComponentEnable("GatewayLorawanMac", LOG_LEVEL_ALL);
        LogComponentEnableAll(LOG_PREFIX_FUNC);
        LogComponentEnableAll(LOG_PREFIX_NODE);
        LogComponentEnableAll(LOG_PREFIX_TIME);

        NS_LOG_INFO("Creating the channel...");
        Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel>();
        loss->SetPathLossExponent(3.76);
        loss->SetReference(1, 7.7); 

        Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel>();
        Ptr<LoraChannel> channel = CreateObject<LoraChannel>(loss, delay);

        NS_LOG_INFO("Setting up helpers...");
        MobilityHelper mobility;
        
        Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator>();
        
        allocator->Add(Vector(0, 0, 0));

        for (int i = 0; i < 6; ++i) {
            allocator->Add(Vector(1000 + i * 50, 0, 0)); 
        }

        mobility.SetPositionAllocator(allocator);
        mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        
        NodeContainer endDevices;
        endDevices.Create(6);
        
        mobility.Install(endDevices);

        NodeContainer gateways;
        gateways.Create(1);
        mobility.Install(gateways);

        LoraPhyHelper phyHelper;
        phyHelper.SetChannel(channel);

        LorawanMacHelper macHelper;
        LoraHelper helper;

        NS_LOG_INFO("Creating the gateway...");
        phyHelper.SetDeviceType(LoraPhyHelper::GW);
        macHelper.SetDeviceType(LorawanMacHelper::GW);
        helper.Install(phyHelper, macHelper, gateways);

        phyHelper.SetDeviceType(LoraPhyHelper::ED);
        macHelper.SetDeviceType(LorawanMacHelper::ED_A);
        macHelper.SetRegion(LorawanMacHelper::SingleChannel);
        helper.Install(phyHelper, macHelper, endDevices);

        Ptr<LoraNetDevice> gwNetDevice = gateways.Get(0)->GetDevice(0)->GetObject<LoraNetDevice>();
        Ptr<GatewayLoraPhy> gwPhy = gwNetDevice->GetPhy()->GetObject<GatewayLoraPhy>();
        gwPhy->TraceConnectWithoutContext("ReceivedPacket", MakeCallback(&PacketReceivedCallback));

        OneShotSenderHelper oneShotSenderHelper;
        oneShotSenderHelper.SetSendTime(Seconds(1));
        oneShotSenderHelper.Install(endDevices);

        for (uint32_t i = 0; i < endDevices.GetN(); i++) {
            endDevices.Get(i)
                ->GetDevice(0)
                ->GetObject<LoraNetDevice>()
                ->GetMac()
                ->GetObject<EndDeviceLorawanMac>()
                ->SetDataRate(5 - i); 

            totalPackets++;
        }

        Simulator::Stop(Seconds(10));
        Simulator::Run();

        double prr = (static_cast<double>(receivedPackets) / totalPackets) * 100;
        prrResults.push_back(prr);

        std::cout << "Run " << j + 1 << ": Packet Reception Ratio (PRR): " << prr << "%" << std::endl;

        Simulator::Destroy();
    }

    double averagePrr = std::accumulate(prrResults.begin(), prrResults.end(), 0.0) / prrResults.size();
    std::cout << "Average Packet Reception Ratio (PRR) over " << numRuns << " runs: " << averagePrr << "%" << std::endl;

    std::ofstream outFile("prr_results.txt");
    for (const auto& prr : prrResults) {
        outFile << prr << std::endl;
    }
    outFile.close();

    return 0;
}
