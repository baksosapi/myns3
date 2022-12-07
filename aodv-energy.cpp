
#include <fstream>
#include <iostream>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/aodv-module.h"
#include "ns3/olsr-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/dsr-module.h"
#include "ns3/applications-module.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/flow-monitor-module.h"
// energy params
#include "ns3/basic-energy-source-helper.h"
#include "ns3/basic-energy-source.h"
#include "ns3/wifi-radio-energy-model.h"
#include "ns3/energy-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-radio-energy-model-helper.h"

using namespace ns3;
using namespace dsr;

NS_LOG_COMPONENT_DEFINE ("manet-routing-compare");

class RoutingExperiment {
public:
    RoutingExperiment();

    void Run(int nSinks, double txp, std::string CSVfileName);

    std::string CommandSetup(int argc, char **argv);

private:
    Ptr<Socket> SetupPacketReceive(Ipv4Address addr, Ptr<Node> node);

    void ReceivePacket(Ptr<Socket> socket);

    void CheckThroughput();

    uint32_t port;
    uint32_t bytesTotal;
    uint32_t packetsReceived;

    std::string m_CSVfileName;
    int m_nSinks;
    std::string m_protocolName;
    double m_txp;
    bool m_traceMobility;
    uint32_t m_protocol;
};

RoutingExperiment::RoutingExperiment()
        : port(9),
          bytesTotal(0),
          packetsReceived(0),
          m_CSVfileName("manet-routing.output.csv"),
          m_traceMobility(false),
          m_protocol(2) // 2 = AODV
{
}

static inline std::string
PrintReceivedPacket(Ptr<Socket> socket, Ptr<Packet> packet, Address senderAddress) {
    std::ostringstream oss;

    oss << Simulator::Now().GetSeconds() << " " << socket->GetNode()->GetId();

    if (InetSocketAddress::IsMatchingType(senderAddress)) {
        InetSocketAddress addr = InetSocketAddress::ConvertFrom(senderAddress);
        oss << " received one packet from " << addr.GetIpv4();
    } else {
        oss << " received one packet!";
    }
    return oss.str();
}

void
RoutingExperiment::ReceivePacket(Ptr<Socket> socket) {
    Ptr<Packet> packet;
    Address senderAddress;
    while ((packet = socket->RecvFrom(senderAddress))) {
        bytesTotal += packet->GetSize();
        packetsReceived += 1;
        NS_LOG_UNCOND (PrintReceivedPacket(socket, packet, senderAddress));
    }
}

void
RoutingExperiment::CheckThroughput() {
    double kbs = (bytesTotal * 8.0) / 1000;
    bytesTotal = 0;

    std::ofstream out(m_CSVfileName.c_str(), std::ios::app);

    out << (Simulator::Now()).GetSeconds() << ","
        << kbs << ","
        << packetsReceived << ","
        << m_nSinks << ","
        << m_protocolName << ","
        << m_txp << ""
        << std::endl;

    out.close();
    packetsReceived = 0;
    Simulator::Schedule(Seconds(1.0), &RoutingExperiment::CheckThroughput, this);
}

Ptr<Socket>
RoutingExperiment::SetupPacketReceive(Ipv4Address addr, Ptr<Node> node) {
    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
    Ptr<Socket> sink = Socket::CreateSocket(node, tid);
    InetSocketAddress local = InetSocketAddress(addr, port);
    sink->Bind(local);
    sink->SetRecvCallback(MakeCallback(&RoutingExperiment::ReceivePacket, this));

    return sink;
}

std::string
RoutingExperiment::CommandSetup(int argc, char **argv) {
    CommandLine cmd;
    cmd.AddValue("CSVfileName", "The name of the CSV output file name", m_CSVfileName);
    cmd.AddValue("traceMobility", "Enable mobility tracing", m_traceMobility);
    cmd.AddValue("protocol", "1=OLSR;2=AODV;3=DSDV;4=DSR", m_protocol);
    cmd.Parse(argc, argv);
    return m_CSVfileName;
}

/// Trace function for remaining energy at node.
void
RemainingEnergy(double oldValue, double remainingEnergy) {
    NS_LOG_UNCOND (Simulator::Now().GetSeconds()
                           << "s Current remaining energy = " << remainingEnergy << "J");
}

/// Trace function for total energy consumption at node.
void
TotalEnergy(double oldValue, double totalEnergy) {
    NS_LOG_UNCOND (Simulator::Now().GetSeconds()
                           << "s Total energy consumed by radio = " << totalEnergy << "J");
}


int
main(int argc, char *argv[]) {
    RoutingExperiment experiment;
    std::string CSVfileName = experiment.CommandSetup(argc, argv);

    //blank out the last output file and write the column headers
    std::ofstream out(CSVfileName.c_str());
    out << "SimulationSecond," <<
        "ReceiveRate," <<
        "PacketsReceived," <<
        "NumberOfSinks," <<
        "RoutingProtocol," <<
        "TransmissionPower" <<
        std::endl;
    out.close();

    int nSinks = 10;
    double txp = 7.5;

    experiment.Run(nSinks, txp, CSVfileName);
}

void
RoutingExperiment::Run(int nSinks, double txp, std::string CSVfileName) {
    Packet::EnablePrinting();
    m_nSinks = nSinks;
    m_txp = txp;
    m_CSVfileName = CSVfileName;

    int nWifis = 20; //node

    double TotalTime = 120.0;// time
    std::string rate("2048bps");//flow rate
    std::string phyMode("DsssRate11Mbps");
    std::string tr_name("manet-routing-compare");
    int nodeSpeed = 5; //in m/s
    int nodePause = 0; //in s
    m_protocolName = "protocol";
    uint32_t SentPackets = 01;
    uint32_t ReceivedPackets = 01;
    uint32_t LostPackets = 01;

    Config::SetDefault("ns3::OnOffApplication::PacketSize", StringValue("64"));
    Config::SetDefault("ns3::OnOffApplication::DataRate", StringValue(rate));

    //Set Non-unicastMode rate to unicast mode
    Config::SetDefault("ns3::WifiRemoteStationManager::NonUnicastMode", StringValue(phyMode));

    NodeContainer adhocNodes;
    adhocNodes.Create(nWifis);

    // Set up wifi phy and channel using helpers
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211b);

    YansWifiPhyHelper wifiPhy;
    wifiPhy.SetErrorRateModel("ns3::NistErrorRateModel");
    YansWifiChannelHelper wifiChannel;
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss("ns3::FriisPropagationLossModel");
    wifiPhy.SetChannel(wifiChannel.Create());

    // Set up mac and disable rate control
    WifiMacHelper wifiMac;
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode", StringValue(phyMode),
                                 "ControlMode", StringValue(phyMode));

    wifiPhy.Set("TxPowerStart", DoubleValue(txp));
    wifiPhy.Set("TxPowerEnd", DoubleValue(txp));

    wifiMac.SetType("ns3::AdhocWifiMac");
    NetDeviceContainer adhocDevices = wifi.Install(wifiPhy, wifiMac, adhocNodes);

    MobilityHelper mobilityAdhoc;
    [[maybe_unused]] int64_t streamIndex = 0; // used to get consistent mobility across scenarios

    ObjectFactory pos;
    pos.SetTypeId("ns3::RandomRectanglePositionAllocator");
    pos.Set("X", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=300.0]"));
    pos.Set("Y", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=1500.0]"));

    Ptr<PositionAllocator> taPositionAlloc = pos.Create()->GetObject<PositionAllocator>();
    streamIndex += taPositionAlloc->AssignStreams(streamIndex);

    std::stringstream ssSpeed;
    ssSpeed << "ns3::UniformRandomVariable[Min=0.0|Max=" << nodeSpeed << "]";
    std::stringstream ssPause;
    ssPause << "ns3::ConstantRandomVariable[Constant=" << nodePause << "]";
    mobilityAdhoc.SetMobilityModel("ns3::RandomWaypointMobilityModel",
                                   "Speed", StringValue(ssSpeed.str()),
                                   "Pause", StringValue(ssPause.str()),
                                   "PositionAllocator", PointerValue(taPositionAlloc));
    mobilityAdhoc.SetPositionAllocator(taPositionAlloc);
    mobilityAdhoc.Install(adhocNodes);
    streamIndex += mobilityAdhoc.AssignStreams(adhocNodes, streamIndex);
//    NS_UNUSED (streamIndex); // From this point, streamIndex is unused

/** -------- Energy Source and Device Energy Model
/** Energy Model **/
    /* energy source */
    BasicEnergySourceHelper basicSourceHelper;
    // Set energy source
    basicSourceHelper.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(1.0));
    // install source
    EnergySourceContainer sources = basicSourceHelper.Install(adhocNodes);
    /* device energy model */
    WifiRadioEnergyModelHelper radioEnergyHelper;
    // configure radio energy model
    radioEnergyHelper.Set("TxCurrentA", DoubleValue(0.0174));
    // install device model
    DeviceEnergyModelContainer deviceModels = radioEnergyHelper.Install(adhocDevices, sources);
    /***************************************************************************/

    //step 4:
/** connect trace sources **/
    /***************************************************************************/
    // all sources are connected to node 1
    // energy source
    Ptr<BasicEnergySource> basicSourcePtr = DynamicCast<BasicEnergySource>(sources.Get(1));
    basicSourcePtr->TraceConnectWithoutContext("RemainingEnergy", MakeCallback(&RemainingEnergy));
    // device energy model
    Ptr<DeviceEnergyModel> basicRadioModelPtr =
            basicSourcePtr->FindDeviceEnergyModels("ns3::WifiRadioEnergyModel").Get(0);
    NS_ASSERT (basicRadioModelPtr != nullptr);
    basicRadioModelPtr->TraceConnectWithoutContext("TotalEnergyConsumption", MakeCallback(&TotalEnergy));
    /***************************************************************************/

    AodvHelper aodv;
    OlsrHelper olsr;
    DsdvHelper dsdv;
    DsrHelper dsr;
    DsrMainHelper dsrMain;
    Ipv4ListRoutingHelper list;
    InternetStackHelper internet;

    switch (m_protocol) {
        case 1:
            list.Add(olsr, 100);
            m_protocolName = "OLSR";
            break;
        case 2:
            list.Add(aodv, 100);
            m_protocolName = "AODV";
            break;
        case 3:
            list.Add(dsdv, 100);
            m_protocolName = "DSDV";
            break;
        case 4:
            m_protocolName = "DSR";
            break;
        default:
            NS_FATAL_ERROR ("No such protocol:" << m_protocol);
    }

    if (m_protocol < 4) {
        internet.SetRoutingHelper(list);
        internet.Install(adhocNodes);
    } else if (m_protocol == 4) {
        internet.Install(adhocNodes);
        dsrMain.Install(dsr, adhocNodes);
    }

    NS_LOG_INFO ("assigning ip address");

    Ipv4AddressHelper addressAdhoc;
    addressAdhoc.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer adhocInterfaces;
    adhocInterfaces = addressAdhoc.Assign(adhocDevices);

    OnOffHelper onoff1("ns3::UdpSocketFactory", Address());
    onoff1.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1.0]"));
    onoff1.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.0]"));

    for (int i = 0; i < nSinks; i++) {
        Ptr<Socket> sink = SetupPacketReceive(adhocInterfaces.GetAddress(i), adhocNodes.Get(i));

        AddressValue remoteAddress(InetSocketAddress(adhocInterfaces.GetAddress(i), port));
        onoff1.SetAttribute("Remote", remoteAddress);

        Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable>();
        ApplicationContainer temp = onoff1.Install(adhocNodes.Get(i + nSinks));
        temp.Start(Seconds(var->GetValue(100.0, 101.0)));
        temp.Stop(Seconds(TotalTime));
    }

    std::stringstream ss;
    ss << nWifis;
    std::string nodes = ss.str();

    std::stringstream ss2;
    ss2 << nodeSpeed;
    std::string sNodeSpeed = ss2.str();

    std::stringstream ss3;
    ss3 << nodePause;
    std::string sNodePause = ss3.str();

    std::stringstream ss4;
    ss4 << rate;
    std::string sRate = ss4.str();

    //NS_LOG_INFO ("Configure Tracing.");
    //tr_name = tr_name + "_" + m_protocolName +"_" + nodes + "nodes_" + sNodeSpeed + "speed_" + sNodePause + "pause_" + sRate + "rate";

    AsciiTraceHelper ascii;
    Ptr<OutputStreamWrapper> osw = ascii.CreateFileStream(tr_name + ".tr");
    wifiPhy.EnableAsciiAll(osw);
    //AsciiTraceHelper ascii;
    MobilityHelper::EnableAsciiAll(ascii.CreateFileStream(tr_name + ".mob"));

    //Ptr<FlowMonitor> flowmon;
    //FlowMonitorHelper flowmonHelper;
    //flowmon = flowmonHelper.InstallAll ();
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    NS_LOG_INFO ("Run Simulation.");

    CheckThroughput();

    Simulator::Stop(Seconds(TotalTime));
    Simulator::Run();


    int j = 0;
    float AvgThroughput = 0;
    Time Jitter;
    Time Delay;

    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();

    for (auto & stat : stats) {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(stat.first);
        NS_LOG_UNCOND("Src Addr" << t.sourceAddress << " Dst Addr " << t.destinationAddress);
        SentPackets = SentPackets + (stat.second.txPackets);
        ReceivedPackets = ReceivedPackets + (stat.second.rxPackets);
        LostPackets = LostPackets + (stat.second.txPackets - stat.second.rxPackets);
        AvgThroughput = AvgThroughput + (stat.second.rxBytes * 8.0 /
                (stat.second.timeLastRxPacket.GetSeconds() - stat.second.timeFirstTxPacket.GetSeconds())/1024);
        Delay = Delay + (stat.second.delaySum);
        j = j + 1;

    }

    AvgThroughput = AvgThroughput / j;
    NS_LOG_UNCOND("--------Total Results of the simulation----------" << std::endl);
    NS_LOG_UNCOND("Throughput =" << AvgThroughput << "Kbps");
    NS_LOG_UNCOND("End to End Delay =" << Delay);
    NS_LOG_UNCOND("Packet Deliveryratio =" << ((ReceivedPackets * 100) / SentPackets) << "%");
    NS_LOG_UNCOND("Packet Loss ratio =" << ((LostPackets * 100) / SentPackets) << "%");

    monitor->SerializeToXmlFile("manet-routing.xml", true, true);

    for (auto iter = deviceModels.Begin(); iter != deviceModels.End(); iter++) {
        double energyConsumed = (*iter)->GetTotalEnergyConsumption();
        NS_LOG_UNCOND ("End of simulation (" << Simulator::Now().GetSeconds()
                                             << "s) Total energy consumed by radio = " << energyConsumed << "J");
        NS_ASSERT (energyConsumed <= 1.0);
    }

    Simulator::Destroy();

    monitor->SerializeToXmlFile("manet-routing.xml", true, true);

}
