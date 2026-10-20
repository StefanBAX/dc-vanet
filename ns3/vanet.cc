/*
 * Urban VANET: beacon reliability and multi-hop alert delivery (Data Communications, Project 4)
 *
 * N vehicles move along a SUMO trace (Ns2MobilityHelper); one RSU stands at the
 * central intersection (500, 500). All nodes use IEEE 802.11p (10 MHz, channel 178,
 * 5.89 GHz), ad-hoc MAC, 6 Mbit/s, 20 dBm, log-distance path loss.
 *
 *  - Beacons: every vehicle broadcasts a 300 B UDP beacon at --beaconHz (single hop).
 *  - Alerts:  every 5 s a random vehicle sends a 500 B UDP alert, unicast to the RSU,
 *             delivered by the routing protocol (--routing=aodv|olsr).
 *
 * Per-packet logs (only for packets sent after the warm-up):
 *  beacon_tx.csv  t_ns,sender,seq,x,y,nb100,nb200,nb300     (nbXXX = vehicles in that distance bin at tx)
 *  beacon_rx.csv  t_ns,receiver,sender,seq,dist_m
 *  alert_tx.csv   id,t_ns,src,dist_rsu_m
 *  alert_rx.csv   id,t_ns,src,ttl,hops
 *  cbr.csv        node,busy_s,window_s,cbr                   (PHY state TX, RX or CCA_BUSY)
 *  routing.csv    node,ctrl_pkts,ctrl_bytes                  (AODV UDP/654 or OLSR UDP/698 sent)
 *  run_info.txt   channel number, width and frequency actually used (P0 check)
 *
 * Build: copy to <ns-3>/scratch/vanet.cc, then ./ns3 build
 * Run:   ./ns3 run "vanet --mobility=mobility.tcl --nVeh=30 --routing=aodv --run=1 --outDir=out"
 */

#include "ns3/aodv-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/olsr-module.h"
#include "ns3/wifi-module.h"

#include <array>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("Vanet");

namespace
{
const uint16_t BEACON_PORT = 9999;
const uint16_t ALERT_PORT = 7777;
const uint16_t AODV_PORT = 654;
const uint16_t OLSR_PORT = 698;
const uint8_t ALERT_TTL = 64;

uint32_t g_nVeh = 0;
double g_warmup = 20.0;
double g_simTime = 120.0;
uint32_t g_beaconSize = 300;
uint32_t g_alertSize = 500;
Time g_beaconPeriod;

NodeContainer g_veh;
Ptr<Node> g_rsu;
Ipv4Address g_rsuAddr;
std::vector<Ptr<Socket>> g_beaconTx;
std::vector<Ptr<Socket>> g_alertTx;
std::vector<uint32_t> g_beaconSeq;
std::vector<double> g_busy;      // busy seconds after the warm-up, per node
std::vector<uint64_t> g_ctrlPkts; // routing control packets sent after the warm-up, per node
std::vector<uint64_t> g_ctrlBytes;
uint32_t g_alertId = 0;
Ptr<UniformRandomVariable> g_rng;

std::ofstream g_beaconTxLog, g_beaconRxLog, g_alertTxLog, g_alertRxLog;

bool
Measuring()
{
    return Simulator::Now().GetSeconds() >= g_warmup;
}

Vector
Pos(Ptr<Node> n)
{
    return n->GetObject<MobilityModel>()->GetPosition();
}

double
Dist2d(const Vector& a, const Vector& b)
{
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

// --- payload layout (first 24 bytes, rest is padding) -------------------------------
struct Header
{
    uint32_t a;  // beacon: sender      | alert: id
    uint32_t b;  // beacon: seq         | alert: src
    int64_t tNs; // transmission time
    float x;     // beacon: sender position at tx (alerts: unused)
    float y;
};

Ptr<Packet>
MakePacket(const Header& h, uint32_t size)
{
    std::vector<uint8_t> buf(size, 0);
    std::memcpy(buf.data(), &h, sizeof(Header));
    return Create<Packet>(buf.data(), size);
}

Header
ReadHeader(Ptr<Packet> p)
{
    Header h{};
    p->CopyData(reinterpret_cast<uint8_t*>(&h), sizeof(Header));
    return h;
}

// --- beacons ------------------------------------------------------------------------
void
SendBeacon(uint32_t i)
{
    const Vector me = Pos(g_veh.Get(i));
    const uint32_t seq = ++g_beaconSeq[i];
    Header h{i, seq, Simulator::Now().GetNanoSeconds(), static_cast<float>(me.x), static_cast<float>(me.y)};
    g_beaconTx[i]->Send(MakePacket(h, g_beaconSize));

    if (Measuring())
    {
        // how many vehicles were in each distance bin at transmission time (denominator of PDR)
        std::array<uint32_t, 3> nb{0, 0, 0};
        for (uint32_t j = 0; j < g_nVeh; ++j)
        {
            if (j == i)
            {
                continue;
            }
            const double d = Dist2d(me, Pos(g_veh.Get(j)));
            if (d < 300.0)
            {
                nb[static_cast<size_t>(d / 100.0)]++;
            }
        }
        g_beaconTxLog << h.tNs << ',' << i << ',' << seq << ',' << std::fixed << std::setprecision(1) << me.x
                      << ',' << me.y << ',' << nb[0] << ',' << nb[1] << ',' << nb[2] << '\n';
    }
    if (Simulator::Now() + g_beaconPeriod < Seconds(g_simTime))
    {
        Simulator::Schedule(g_beaconPeriod, &SendBeacon, i);
    }
}

void
ReceiveBeacon(Ptr<Socket> socket)
{
    const uint32_t me = socket->GetNode()->GetId();
    Address from;
    while (Ptr<Packet> p = socket->RecvFrom(from))
    {
        const Header h = ReadHeader(p);
        if (h.a == me || h.tNs < static_cast<int64_t>(g_warmup * 1e9))
        {
            continue;
        }
        const Vector pos = Pos(socket->GetNode());
        const double d = Dist2d(pos, Vector(h.x, h.y, 0));
        g_beaconRxLog << Simulator::Now().GetNanoSeconds() << ',' << me << ',' << h.a << ',' << h.b << ','
                      << std::fixed << std::setprecision(1) << d << '\n';
    }
}

// --- alerts -------------------------------------------------------------------------
void
SendAlert(Time period)
{
    const uint32_t src = g_rng->GetInteger(0, g_nVeh - 1);
    const uint32_t id = ++g_alertId;
    Header h{id, src, Simulator::Now().GetNanoSeconds(), 0.0F, 0.0F};
    g_alertTx[src]->Send(MakePacket(h, g_alertSize));
    g_alertTxLog << id << ',' << h.tNs << ',' << src << ',' << std::fixed << std::setprecision(1)
                 << Dist2d(Pos(g_veh.Get(src)), Pos(g_rsu)) << '\n';
    if (Simulator::Now() + period < Seconds(g_simTime))
    {
        Simulator::Schedule(period, &SendAlert, period);
    }
}

void
ReceiveAlert(Ptr<Socket> socket)
{
    Address from;
    while (Ptr<Packet> p = socket->RecvFrom(from))
    {
        const Header h = ReadHeader(p);
        SocketIpTtlTag ttlTag;
        int ttl = -1;
        if (p->PeekPacketTag(ttlTag))
        {
            ttl = ttlTag.GetTtl();
        }
        // the sender sets TTL 64 and every forwarder decrements it once
        const int hops = ttl >= 0 ? ALERT_TTL - ttl + 1 : -1;
        g_alertRxLog << h.a << ',' << Simulator::Now().GetNanoSeconds() << ',' << h.b << ',' << ttl << ','
                     << hops << '\n';
    }
}

// --- channel busy ratio from the PHY state trace --------------------------------------
uint32_t
NodeFromContext(const std::string& ctx)
{
    const auto a = ctx.find("/NodeList/") + 10;
    return std::stoul(ctx.substr(a, ctx.find('/', a) - a));
}

void
PhyState(std::string ctx, Time start, Time duration, WifiPhyState state)
{
    if (state != WifiPhyState::TX && state != WifiPhyState::RX && state != WifiPhyState::CCA_BUSY)
    {
        return;
    }
    const double s = std::max(start.GetSeconds(), g_warmup);
    const double e = std::min((start + duration).GetSeconds(), g_simTime);
    if (e > s)
    {
        g_busy[NodeFromContext(ctx)] += e - s;
    }
}

// --- routing overhead: control packets handed to IP by AODV / OLSR -------------------
void
IpTx(std::string ctx, Ptr<const Packet> packet, Ptr<Ipv4> ipv4, uint32_t iface)
{
    if (!Measuring())
    {
        return;
    }
    Ptr<Packet> p = packet->Copy();
    Ipv4Header ip;
    p->RemoveHeader(ip);
    if (ip.GetProtocol() != UdpL4Protocol::PROT_NUMBER)
    {
        return;
    }
    UdpHeader udp;
    p->PeekHeader(udp);
    const uint16_t port = udp.GetDestinationPort();
    if (port == AODV_PORT || port == OLSR_PORT)
    {
        const uint32_t n = NodeFromContext(ctx);
        g_ctrlPkts[n]++;
        g_ctrlBytes[n] += packet->GetSize();
    }
}

} // namespace

int
main(int argc, char* argv[])
{
    std::string mobility;
    std::string routing = "aodv";
    std::string outDir = ".";
    uint32_t run = 1;
    double beaconHz = 10.0;
    double txPower = 20.0;
    double pathLossExp = 2.5;
    double minRssi = -94.0;
    double alertPeriod = 5.0;
    bool flowmon = false;

    CommandLine cmd(__FILE__);
    cmd.AddValue("mobility", "ns-2 mobility trace from SUMO (traceExporter.py)", mobility);
    cmd.AddValue("nVeh", "number of vehicles in the trace", g_nVeh);
    cmd.AddValue("routing", "aodv or olsr", routing);
    cmd.AddValue("run", "RNG run number (beacon phases, alert sources)", run);
    cmd.AddValue("simTime", "simulated seconds", g_simTime);
    cmd.AddValue("warmup", "seconds discarded at the start", g_warmup);
    cmd.AddValue("beaconHz", "beacon rate per vehicle", beaconHz);
    cmd.AddValue("beaconSize", "beacon size (bytes)", g_beaconSize);
    cmd.AddValue("alertSize", "alert size (bytes)", g_alertSize);
    cmd.AddValue("alertPeriod", "seconds between alerts", alertPeriod);
    cmd.AddValue("txPower", "transmit power (dBm)", txPower);
    cmd.AddValue("pathLossExp", "log-distance path loss exponent", pathLossExp);
    cmd.AddValue("minRssi", "weakest frame the receiver tries to decode (dBm)", minRssi);
    cmd.AddValue("flowmon", "also write a FlowMonitor XML", flowmon);
    cmd.AddValue("outDir", "output folder", outDir);
    cmd.Parse(argc, argv);
    NS_ABORT_MSG_IF(mobility.empty() || g_nVeh == 0, "--mobility and --nVeh are required");
    NS_ABORT_MSG_IF(routing != "aodv" && routing != "olsr", "--routing must be aodv or olsr");

    RngSeedManager::SetSeed(1);
    RngSeedManager::SetRun(run);
    g_beaconPeriod = Seconds(1.0 / beaconHz);

    // ---- nodes and mobility -------------------------------------------------------------
    g_veh.Create(g_nVeh);
    NodeContainer rsuC;
    rsuC.Create(1);
    g_rsu = rsuC.Get(0);
    NodeContainer all(g_veh, rsuC);

    Ns2MobilityHelper ns2(mobility);
    ns2.Install(g_veh.Begin(), g_veh.End()); // trace node i -> vehicle i
    MobilityHelper fixed;
    Ptr<ListPositionAllocator> rsuPos = CreateObject<ListPositionAllocator>();
    rsuPos->Add(Vector(500.0, 500.0, 0.0)); // central intersection of the SUMO grid
    fixed.SetPositionAllocator(rsuPos);
    fixed.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    fixed.Install(rsuC);

    // ---- 802.11p PHY / MAC --------------------------------------------------------------
    YansWifiChannelHelper channel;
    channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    // reference loss at 1 m for 5.89 GHz free space: 20 log10(4 pi / lambda) = 47.86 dB
    channel.AddPropagationLoss("ns3::LogDistancePropagationLossModel",
                               "Exponent", DoubleValue(pathLossExp),
                               "ReferenceDistance", DoubleValue(1.0),
                               "ReferenceLoss", DoubleValue(47.86));
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());
    phy.Set("ChannelSettings", StringValue("{178, 10, BAND_5GHZ, 0}"));
    phy.Set("TxPowerStart", DoubleValue(txPower));
    phy.Set("TxPowerEnd", DoubleValue(txPower));
    // ns-3's default preamble detection threshold (-82 dBm) is the 20 MHz 802.11a CCA level and
    // would cut the radio range to ~150 m. A 10 MHz 802.11p receiver has a 3 dB lower noise floor
    // (about -97 dBm with a 7 dB noise figure); we let it try frames down to minRssi. The 4 dB
    // SNR condition of the model still applies.
    phy.SetPreambleDetectionModel("ns3::ThresholdPreambleDetectionModel",
                                  "MinimumRssi", DoubleValue(minRssi));

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211p);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode", StringValue("OfdmRate6MbpsBW10MHz"),
                                 "ControlMode", StringValue("OfdmRate6MbpsBW10MHz"),
                                 "NonUnicastMode", StringValue("OfdmRate6MbpsBW10MHz"));
    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");
    NetDeviceContainer devices = wifi.Install(phy, mac, all);

    // ---- IP and routing -------------------------------------------------------------------
    AodvHelper aodv;
    OlsrHelper olsr;
    Ipv4StaticRoutingHelper staticRouting;
    Ipv4ListRoutingHelper list;
    list.Add(staticRouting, 0);
    if (routing == "aodv")
    {
        list.Add(aodv, 100);
    }
    else
    {
        list.Add(olsr, 100);
    }
    InternetStackHelper internet;
    internet.SetRoutingHelper(list);
    internet.Install(all);
    Ipv4AddressHelper addr;
    addr.SetBase("10.1.0.0", "255.255.0.0");
    Ipv4InterfaceContainer ifaces = addr.Assign(devices);
    g_rsuAddr = ifaces.GetAddress(g_nVeh);

    // ---- sockets --------------------------------------------------------------------------
    TypeId udp = UdpSocketFactory::GetTypeId();
    g_beaconSeq.assign(g_nVeh, 0);
    for (uint32_t i = 0; i < g_nVeh; ++i)
    {
        Ptr<Node> n = g_veh.Get(i);
        Ptr<Socket> rx = Socket::CreateSocket(n, udp);
        rx->Bind(InetSocketAddress(Ipv4Address::GetAny(), BEACON_PORT));
        rx->SetRecvCallback(MakeCallback(&ReceiveBeacon));

        Ptr<Socket> btx = Socket::CreateSocket(n, udp);
        btx->SetAllowBroadcast(true);
        btx->Connect(InetSocketAddress(Ipv4Address("255.255.255.255"), BEACON_PORT));
        g_beaconTx.push_back(btx);

        Ptr<Socket> atx = Socket::CreateSocket(n, udp);
        atx->SetIpTtl(ALERT_TTL);
        atx->Connect(InetSocketAddress(g_rsuAddr, ALERT_PORT));
        g_alertTx.push_back(atx);
    }
    Ptr<Socket> rsuRx = Socket::CreateSocket(g_rsu, udp);
    rsuRx->SetIpRecvTtl(true);
    rsuRx->Bind(InetSocketAddress(Ipv4Address::GetAny(), ALERT_PORT));
    rsuRx->SetRecvCallback(MakeCallback(&ReceiveAlert));

    // ---- logs -----------------------------------------------------------------------------
    g_beaconTxLog.open(outDir + "/beacon_tx.csv");
    g_beaconRxLog.open(outDir + "/beacon_rx.csv");
    g_alertTxLog.open(outDir + "/alert_tx.csv");
    g_alertRxLog.open(outDir + "/alert_rx.csv");
    g_beaconTxLog << "t_ns,sender,seq,x,y,nb100,nb200,nb300\n";
    g_beaconRxLog << "t_ns,receiver,sender,seq,dist_m\n";
    g_alertTxLog << "id,t_ns,src,dist_rsu_m\n";
    g_alertRxLog << "id,t_ns,src,ttl,hops\n";

    // ---- traffic schedule -----------------------------------------------------------------
    g_rng = CreateObject<UniformRandomVariable>();
    Ptr<UniformRandomVariable> phase = CreateObject<UniformRandomVariable>();
    for (uint32_t i = 0; i < g_nVeh; ++i)
    {
        // start after 1 s (positions loaded), random phase within one beacon period
        Simulator::Schedule(Seconds(1.0 + phase->GetValue(0.0, g_beaconPeriod.GetSeconds())), &SendBeacon, i);
    }
    Simulator::Schedule(Seconds(g_warmup), &SendAlert, Seconds(alertPeriod));

    // ---- traces ---------------------------------------------------------------------------
    g_busy.assign(all.GetN(), 0.0);
    g_ctrlPkts.assign(all.GetN(), 0);
    g_ctrlBytes.assign(all.GetN(), 0);
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/State/State", MakeCallback(&PhyState));
    Config::Connect("/NodeList/*/$ns3::Ipv4L3Protocol/Tx", MakeCallback(&IpTx));

    Ptr<FlowMonitor> monitor;
    FlowMonitorHelper fmHelper;
    if (flowmon)
    {
        monitor = fmHelper.InstallAll();
    }

    // P0 check: which channel does the PHY really use?
    Ptr<WifiPhy> wphy = DynamicCast<WifiNetDevice>(devices.Get(0))->GetPhy();
    std::ofstream info(outDir + "/run_info.txt");
    info << "standard=802.11p channel=" << +wphy->GetChannelNumber() << " width_MHz=" << wphy->GetChannelWidth()
         << " frequency_MHz=" << wphy->GetFrequency() << " tx_power_dBm=" << txPower
         << " path_loss_exponent=" << pathLossExp << " min_rssi_dBm=" << minRssi << " routing=" << routing << " vehicles=" << g_nVeh
         << " beacon_hz=" << beaconHz << " run=" << run << "\n";
    info.close();

    Simulator::Stop(Seconds(g_simTime));
    Simulator::Run();

    // ---- per-node outputs -----------------------------------------------------------------
    const double window = g_simTime - g_warmup;
    std::ofstream cbr(outDir + "/cbr.csv");
    cbr << "node,busy_s,window_s,cbr\n";
    for (uint32_t n = 0; n < all.GetN(); ++n)
    {
        cbr << n << ',' << g_busy[n] << ',' << window << ',' << g_busy[n] / window << '\n';
    }
    std::ofstream rt(outDir + "/routing.csv");
    rt << "node,ctrl_pkts,ctrl_bytes\n";
    for (uint32_t n = 0; n < all.GetN(); ++n)
    {
        rt << n << ',' << g_ctrlPkts[n] << ',' << g_ctrlBytes[n] << '\n';
    }
    if (flowmon)
    {
        monitor->SerializeToXmlFile(outDir + "/flowmon.xml", false, false);
    }
    g_beaconTxLog.close();
    g_beaconRxLog.close();
    g_alertTxLog.close();
    g_alertRxLog.close();
    Simulator::Destroy();
    return 0;
}
